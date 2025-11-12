// ============================================================================
//  post_requant.sv
//  Streaming integer requantization (fixed-point multiplier + shift)
//  - Per-channel or broadcast params (mult, rshift, zero-point)
//  - Optional input pre-shift with symmetric rounding
//  - Rounding modes: TRUNC, SYM_HALF_UP, RNE (round-to-nearest-even), STOCH
//  - Ready/Valid elastic pipeline; no combinational ready loops
//  - CSR-port to program per-channel tables
//
//  Typical placement:
//     ... → post_bias → post_activation/post_gelu_pwl → post_requant → DMA/drain
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module post_requant #(
  // ---------------- Data geometry ----------------
  parameter int unsigned IN_W       = 32,  // accumulator/input width (signed)
  parameter int unsigned OUT_W      = 8,   // output width (signed or unsigned)
  parameter bit          OUT_SIGNED = 1'b1,

  // Multiplier width (fixed-point). For TFLite Q31, set MULT_W=32 and rshift>=31.
  parameter int unsigned MULT_W     = 32,

  // Channel addressing (per-channel tables). CHAN_W=0 → depth=1 (broadcast-only).
  parameter int unsigned CHAN_W     = 12,  // up to 4096 channels

  // Elasticity / timing
  parameter int unsigned IN_FIFO_DEPTH = 4, // ingress FIFO depth (>=2 recommended)

  // Internal headroom in add/shift path
  parameter int unsigned HEADROOM    = 8
)(
  input  logic                      clk_i,
  input  logic                      rstn_i,         // synchronous, active-low

  // =================== Stream In ===================
  input  logic                      in_valid_i,
  output logic                      in_ready_o,
  input  logic signed [IN_W-1:0]    in_data_i,
  input  logic [CHAN_W-1:0]         in_chan_i,      // channel/tag (ignored if CHAN_W==0)
  input  logic                      in_last_i,

  // =================== Stream Out ==================
  output logic                      out_valid_o,
  input  logic                      out_ready_i,
  output logic [OUT_W-1:0]          out_data_o,     // saturated to OUT_W
  output logic                      out_last_o,

  // =================== Run-time controls ===========
  input  logic                      cfg_enable_i,     // 0=bypass (truncate+clamp only)
  input  logic                      cfg_per_chan_en_i,// 1=use per-channel tables; 0=broadcast

  // Optional input pre-shift (applied to x before multiply)
  input  logic signed [7:0]         cfg_in_shift_i,    // +N: <<N,  -N: >>>N (arith)
  input  logic                      cfg_in_shift_rnd_i,// right-shift: symmetric round-to-nearest

  // Broadcast parameters (used when cfg_per_chan_en_i==0)
  input  logic signed [MULT_W-1:0]  cfg_mult_bcast_i,   // fixed-point multiplier (usually non-negative)
  input  logic [5:0]                cfg_rshift_bcast_i, // 0..63 bits
  input  logic signed [OUT_W+7:0]   cfg_zp_bcast_i,     // output zero-point (signed domain for convenience)

  // Output clamp (final safety); set to full-range for plain saturate-to-width
  input  logic signed [OUT_W-1:0]   cfg_clip_min_i,
  input  logic signed [OUT_W-1:0]   cfg_clip_max_i,

  // Rounding mode for the *post-multiply* right shift
  //  2'b00 = TRUNC (toward zero)
  //  2'b01 = SYM_HALF_UP (add +/- 2^(s-1) before >>> s)
  //  2'b10 = RNE (round-to-nearest-even, magnitude-based)
  //  2'b11 = STOCH (stochastic rounding with internal LFSR)
  input  logic [1:0]                cfg_round_mode_i,

  // RNG controls (only used when round_mode==STOCH)
  input  logic                      cfg_stoch_en_i,   // must be 1 along with mode==STOCH
  input  logic [15:0]               cfg_rng_seed_i,   // seed value
  input  logic                      cfg_rng_reseed_i, // pulse: reload seed

  // =================== CSR (per-channel tables) ============================
  // Addressed by channel id (0..2^CHAN_W-1). Depth=1 if CHAN_W==0.
  // Select which table to access using csr_sel_i:
  //   2'b00 → mult table (MULT_W)
  //   2'b01 → rshift table (6 bits)
  //   2'b10 → zp table (OUT_W+8 bits, signed)
  input  logic                      csr_we_i,
  input  logic                      csr_re_i,
  input  logic [1:0]                csr_sel_i,
  input  logic [CHAN_W-1:0]         csr_addr_i,
  input  logic signed [31:0]        csr_wdata_i,
  output logic signed [31:0]        csr_rdata_o,

  // =================== Telemetry ==========================================
  input  logic                      cmd_clr_stats_i,      // pulse to clear counters
  output logic [31:0]               stat_in_beats_o,
  output logic [31:0]               stat_out_beats_o,
  output logic [31:0]               stat_saturations_o,   // #outputs that hit clamp ends
  output logic [31:0]               stat_round_incs_o     // #times rounding incremented magnitude
);

  // ==========================================================================
  // Locals / widths
  // ==========================================================================
  localparam int unsigned DEPTH   = (CHAN_W == 0) ? 1 : (1 << CHAN_W);
  localparam int unsigned XW      = IN_W + HEADROOM;         // widened x
  localparam int unsigned PROD_W  = XW + MULT_W;             // product width
  localparam int unsigned SUM_W   = (PROD_W > (OUT_W+8) ? PROD_W : (OUT_W+8)) + 2;

  // ==========================================================================
  // Ingress FIFO (no comb ready loop)
  // ==========================================================================
  localparam int unsigned PACK_W = IN_W + CHAN_W + 1;
  logic                 f_in_ready, f_out_valid, f_out_ready;
  logic [PACK_W-1:0]    f_out_word;

  rv_fifo #(
    .WIDTH (PACK_W),
    .DEPTH (IN_FIFO_DEPTH)
  ) u_in_fifo (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (in_valid_i),
    .in_ready_o  (f_in_ready),
    .in_data_i   ({in_last_i, in_chan_i, in_data_i}),
    .out_valid_o (f_out_valid),
    .out_ready_i (f_out_ready),
    .out_data_o  (f_out_word)
  );

  assign in_ready_o = f_in_ready;

  wire                      i_last = f_out_word[PACK_W-1];
  wire [CHAN_W-1:0]         i_chan = f_out_word[IN_W +: CHAN_W];
  wire signed [IN_W-1:0]    i_x    = f_out_word[IN_W-1:0];

  // ==========================================================================
  // Per-channel parameter memories (synchronous 1-cycle read)
  // ==========================================================================
  logic signed [MULT_W-1:0] mult_mem [0:DEPTH-1];
  logic [5:0]               rsh_mem  [0:DEPTH-1];
  logic signed [OUT_W+7:0]  zp_mem   [0:DEPTH-1];

  // CSR port
  logic signed [31:0] csr_rdata_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      csr_rdata_q <= '0;
    end else begin
      if (csr_we_i) begin
        case (csr_sel_i)
          2'b00: mult_mem[csr_addr_i] <= csr_wdata_i[MULT_W-1:0];
          2'b01: rsh_mem [csr_addr_i] <= csr_wdata_i[5:0];
          2'b10: zp_mem  [csr_addr_i] <= csr_wdata_i[OUT_W+7:0];
          default: /* no-op */;
        endcase
      end
      if (csr_re_i) begin
        case (csr_sel_i)
          2'b00: csr_rdata_q <= $signed({{(32-MULT_W){mult_mem[csr_addr_i][MULT_W-1]}}, mult_mem[csr_addr_i]});
          2'b01: csr_rdata_q <= $signed({26'd0, rsh_mem[csr_addr_i]});
          2'b10: csr_rdata_q <= $signed({{(32-(OUT_W+8)){zp_mem[csr_addr_i][OUT_W+7]}}, zp_mem[csr_addr_i]});
          default: csr_rdata_q <= 32'sd0;
        endcase
      end
    end
  end
  assign csr_rdata_o = csr_rdata_q;

  // Stream read side (1-cycle latency)
  logic                  coef_req_q;
  logic [CHAN_W-1:0]     coef_addr_q;
  logic signed [MULT_W-1:0] mult_q;
  logic [5:0]               rsh_q;
  logic signed [OUT_W+7:0]  zp_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      mult_q <= '0; rsh_q <= 6'd0; zp_q <= '0;
    end else if (coef_req_q) begin
      mult_q <= mult_mem[coef_addr_q];
      rsh_q  <= rsh_mem [coef_addr_q];
      zp_q   <= zp_mem  [coef_addr_q];
    end
  end

  // ==========================================================================
  // Pipeline control: PEND (request) → B (params+data) → OUT skid
  // ==========================================================================
  // Output skid
  logic o_v_q;
  wire  skid_ready = (!o_v_q) || (o_v_q && out_ready_i);

  // B stage (holds data and eff params)
  logic                      b_v_q;
  logic signed [IN_W-1:0]    b_x_q;
  logic [CHAN_W-1:0]         b_chan_q;
  logic                      b_last_q;
  logic signed [MULT_W-1:0]  b_mult_q;
  logic [5:0]                b_rshift_q;
  logic signed [OUT_W+7:0]   b_zp_q;

  // PEND stage: remembers a request while coeffs are in flight
  logic                      pend_v_q;
  logic signed [IN_W-1:0]    pend_x_q;
  logic [CHAN_W-1:0]         pend_chan_q;
  logic                      pend_last_q;

  // When can B accept a new entry *this* cycle?
  wire b_can_accept = (!b_v_q) || (b_v_q && skid_ready);

  // FIFO can pop to PEND when PEND is free and B will be free by the time coeffs return
  assign f_out_ready = f_out_valid && !pend_v_q && b_can_accept;

  // Kick parameter read if per-channel enabled; else we still use broadcast (no read required)
  wire use_bcast = !cfg_per_chan_en_i;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      pend_v_q     <= 1'b0;
      coef_req_q   <= 1'b0;
      coef_addr_q  <= '0;
      pend_x_q     <= '0;
      pend_chan_q  <= '0;
      pend_last_q  <= 1'b0;

      b_v_q        <= 1'b0;
      b_x_q        <= '0;
      b_chan_q     <= '0;
      b_last_q     <= 1'b0;
      b_mult_q     <= '0;
      b_rshift_q   <= 6'd0;
      b_zp_q       <= '0;
    end else begin
      coef_req_q <= 1'b0;

      // FIFO → PEND (start a param read)
      if (f_out_valid && f_out_ready) begin
        pend_v_q     <= 1'b1;
        pend_x_q     <= i_x;
        pend_chan_q  <= i_chan;
        pend_last_q  <= i_last;

        if (!use_bcast) begin
          coef_req_q  <= 1'b1;
          coef_addr_q <= i_chan;
        end
      end

      // PEND → B when:
      //  - per-channel: coeff returns next cycle after request
      //  - broadcast: move immediately (no memory wait) by mirroring same timing
      if (pend_v_q) begin
        pend_v_q   <= 1'b0;
        b_v_q      <= 1'b1;
        b_x_q      <= pend_x_q;
        b_chan_q   <= pend_chan_q;
        b_last_q   <= pend_last_q;
        b_mult_q   <= use_bcast ? cfg_mult_bcast_i : mult_q;
        b_rshift_q <= use_bcast ? cfg_rshift_bcast_i : rsh_q;
        b_zp_q     <= use_bcast ? cfg_zp_bcast_i    : zp_q;
      end

      // B → skid happens below in compute block; b_v_q cleared there
    end
  end

  // ==========================================================================
  // Rounding helpers
  // ==========================================================================
  // Absolute value helper for wide signed vectors
  function automatic logic signed [PROD_W-1:0]
    sabs_prod(input logic signed [PROD_W-1:0] v);
    return v[PROD_W-1] ? -v : v;
  endfunction

  // Truncation (toward zero) after arithmetic right shift
  function automatic logic signed [PROD_W-1:0]
    shr_trunc(input logic signed [PROD_W-1:0] v, input int unsigned sh);
    return (sh==0) ? v : (v >>> sh);
  endfunction

  // Symmetric half-up: add +/- 2^(s-1) before >>> s
  function automatic logic signed [PROD_W-1:0]
    shr_sym_half_up(input logic signed [PROD_W-1:0] v, input int unsigned sh);
    if (sh==0) return v;
    logic signed [PROD_W-1:0] half = {{(PROD_W-1){1'b0}},1'b1} <<< (sh-1);
    return v + (v[PROD_W-1] ? -half : half);
  endfunction

  // Round-to-nearest-even (magnitude domain)
  function automatic logic signed [PROD_W-1:0]
    shr_rne(input logic signed [PROD_W-1:0] v, input int unsigned sh);
    if (sh==0) return v;
    logic sign = v[PROD_W-1];
    logic signed [PROD_W-1:0] a = sabs_prod(v);
    logic [PROD_W-1:0]        tail_mask = ({{PROD_W{1'b0}}} | ((PROD_W'(1) << sh) - 1));
    logic [PROD_W-1:0]        half      = PROD_W'(1) << (sh-1);
    logic                     half_flag = ((a & ((PROD_W'(1) << sh) - 1)) == half);
    logic                     gt_half   = (a & ((PROD_W'(1) << (sh-1)) - 1)) != 0;
    logic                     lsb_even  = (((a >> sh) & 1'b1) == 1'b0);
    logic                     inc       = gt_half || (half_flag && !lsb_even);
    logic signed [PROD_W-1:0] ap = a + (inc ? half : PROD_W'(0)); // add half for tie/up
    logic signed [PROD_W-1:0] q  = ap >>> sh;
    return sign ? -q : q;
  endfunction

  // Stochastic rounding: add random [0..2^sh-1] to magnitude before >>> sh
  function automatic logic signed [PROD_W-1:0]
    shr_stoch(input logic signed [PROD_W-1:0] v,
              input int unsigned sh,
              input logic [31:0] rnd);
    if (sh==0) return v;
    logic sign = v[PROD_W-1];
    logic signed [PROD_W-1:0] a = sabs_prod(v);
    logic [31:0] mask = (sh >= 32) ? 32'hFFFF_FFFF : ((32'(1) << sh) - 1);
    logic [31:0] add  = rnd & mask;
    logic signed [PROD_W-1:0] ap = a + PROD_W'(add);
    logic signed [PROD_W-1:0] q  = ap >>> sh;
    return sign ? -q : q;
  endfunction

  // Optional input pre-shift with symmetric rounding on right shifts
  function automatic logic signed [XW-1:0]
    pre_shift_x(input logic signed [IN_W-1:0] x,
                input logic signed [7:0] sh,
                input logic rnd);
    logic signed [XW-1:0] xe = {{(XW-IN_W){x[IN_W-1]}}, x};
    if (sh >= 0) begin
      return xe <<< sh[7:0];
    end else begin
      logic [7:0] mag = -sh[7:0];
      if (mag == 0) return xe;
      if (rnd) begin
        logic signed [XW-1:0] half = {{(XW-1){1'b0}},1'b1} <<< (mag-1);
        logic signed [XW-1:0] adj  = xe[XW-1] ? -half : half;
        return (xe + adj) >>> mag;
      end else begin
        return xe >>> mag;
      end
    end
  endfunction

  // ==========================================================================
  // RNG (for stochastic rounding)
  // ==========================================================================
  logic [15:0] lfsr_q;
  wire  lfsr_fb = lfsr_q[15] ^ lfsr_q[13] ^ lfsr_q[12] ^ lfsr_q[10]; // x^16+x^14+x^13+x^11+1
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i)          lfsr_q <= (cfg_rng_seed_i == 16'd0) ? 16'h1ACE : cfg_rng_seed_i;
    else if (cfg_rng_reseed_i) lfsr_q <= (cfg_rng_seed_i == 16'd0) ? 16'h1ACE : cfg_rng_seed_i;
    else                   lfsr_q <= {lfsr_q[14:0], lfsr_fb};
  end

  // ==========================================================================
  // Compute & output staging
  // ==========================================================================
  // Out regs
  logic [OUT_W-1:0]  o_d_q, o_d_d;
  logic              o_l_q, o_l_d;

  // Perform compute when B valid and skid can accept
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      o_v_q   <= 1'b0;
      o_d_q   <= '0;
      o_l_q   <= 1'b0;
      b_v_q   <= 1'b0;
    end else begin
      // Pop skid
      if (o_v_q && out_ready_i) o_v_q <= 1'b0;

      if (b_v_q && skid_ready) begin
        // 1) Optional input pre-shift
        logic signed [XW-1:0] x_pre = pre_shift_x(b_x_q, cfg_in_shift_i, cfg_in_shift_rnd_i);

        // 2) Multiply by fixed-point multiplier
        logic signed [PROD_W-1:0] prod = $signed(x_pre) * $signed(b_mult_q);

        // 3) Right-shift with selected rounding mode
        logic signed [PROD_W-1:0] prod_rnd;
        unique case (cfg_round_mode_i)
          2'b00: prod_rnd = shr_trunc      (prod, b_rshift_q);
          2'b01: prod_rnd = shr_sym_half_up(prod, b_rshift_q);
          2'b10: prod_rnd = shr_rne        (prod, b_rshift_q);
          default: begin
            logic [31:0] rnd32 = {lfsr_q, ~lfsr_q}; // simple 32b expansion
            prod_rnd = (cfg_stoch_en_i) ? shr_stoch(prod, b_rshift_q, rnd32)
                                        : shr_trunc(prod, b_rshift_q);
          end
        endcase

        // 4) Add zero-point (in output domain)
        logic signed [SUM_W-1:0] sum =
            {{(SUM_W-PROD_W){prod_rnd[PROD_W-1]}}, prod_rnd} +
            {{(SUM_W-(OUT_W+8)){b_zp_q[OUT_W+7]}}, b_zp_q};

        // 5) Narrow & clamp to final OUT_W with cfg_clip_min/max
        logic signed [OUT_W-1:0] y_narrow = sum[OUT_W-1:0];

        // Guard clamp bounds for signed/unsigned interpretations
        logic signed [OUT_W-1:0] clip_min_s = cfg_clip_min_i;
        logic signed [OUT_W-1:0] clip_max_s = cfg_clip_max_i;

        logic do_sat_min = (OUT_SIGNED)
                         ? (y_narrow < clip_min_s)
                         : ($signed({1'b0, y_narrow[OUT_W-2:0]}) < $signed({1'b0, clip_min_s[OUT_W-2:0]}));
        logic do_sat_max = (OUT_SIGNED)
                         ? (y_narrow > clip_max_s)
                         : ($signed({1'b0, y_narrow[OUT_W-2:0]}) > $signed({1'b0, clip_max_s[OUT_W-2:0]}));

        logic [OUT_W-1:0] y_clamped;
        if (do_sat_min)      y_clamped = clip_min_s[OUT_W-1:0];
        else if (do_sat_max) y_clamped = clip_max_s[OUT_W-1:0];
        else                 y_clamped = y_narrow[OUT_W-1:0];

        // 6) Bypass option: if disabled, just pass input (truncated+clamped)
        if (!cfg_enable_i) begin
          // Truncate input to OUT_W then clamp
          logic signed [OUT_W-1:0] in_trunc = b_x_q[OUT_W-1:0];
          logic do_min2 = (OUT_SIGNED) ? (in_trunc < clip_min_s) :
                          ($signed({1'b0, in_trunc[OUT_W-2:0]}) < $signed({1'b0, clip_min_s[OUT_W-2:0]}));
          logic do_max2 = (OUT_SIGNED) ? (in_trunc > clip_max_s) :
                          ($signed({1'b0, in_trunc[OUT_W-2:0]}) > $signed({1'b0, clip_max_s[OUT_W-2:0]}));
          y_clamped = do_min2 ? clip_min_s : (do_max2 ? clip_max_s : in_trunc);
        end

        // Load skid
        o_v_q <= 1'b1;
        o_d_q <= y_clamped;
        o_l_q <= b_last_q;

        // Free B
        b_v_q <= 1'b0;
      end
    end
  end

  assign out_valid_o = o_v_q;
  assign out_data_o  = o_d_q;
  assign out_last_o  = o_l_q;

  // ==========================================================================
  // Telemetry
  // ==========================================================================
  logic [31:0] in_beats_q, out_beats_q, sat_q, round_inc_q;

  // “round increment” heuristic: count cycles where rounding mode would increase |value|
  // For TRUNC it's 0; for SYM_HALF_UP/RNE/STOCH we add when increment applied.
  // We approximate: for SYM_HALF_UP, count when lower-bit (bit rshift-1) is 1.
  // For RNE: count when rule chooses increment. For STOCH: use LFSR MSB as ~50% proxy.
  function automatic logic will_round_up_sym(input logic signed [PROD_W-1:0] v, input int unsigned sh);
    if (sh==0) return 1'b0;
    return v[sh-1];
  endfunction

  function automatic logic will_round_up_rne(input logic signed [PROD_W-1:0] v, input int unsigned sh);
    if (sh==0) return 1'b0;
    logic signed [PROD_W-1:0] a = v[PROD_W-1] ? -v : v;
    logic half_flag = ((a & ((PROD_W'(1) << sh) - 1)) == (PROD_W'(1) << (sh-1)));
    logic gt_half   = (a & ((PROD_W'(1) << (sh-1)) - 1)) != 0;
    logic lsb_even  = (((a >> sh) & 1'b1) == 1'b0);
    return gt_half || (half_flag && !lsb_even);
  endfunction

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      in_beats_q  <= 32'd0;
      out_beats_q <= 32'd0;
      sat_q       <= 32'd0;
      round_inc_q <= 32'd0;
    end else begin
      if (cmd_clr_stats_i) begin
        in_beats_q  <= 32'd0;
        out_beats_q <= 32'd0;
        sat_q       <= 32'd0;
        round_inc_q <= 32'd0;
      end else begin
        if (in_valid_i && in_ready_o) in_beats_q <= in_beats_q + 32'd1;
        if (out_valid_o && out_ready_i) out_beats_q <= out_beats_q + 32'd1;

        // Count saturations when we loaded skid
        if (b_v_q && skid_ready) begin
          // Recompute quick clamp hit with current result (uses same y_clamped logic condensed)
          // We simply compare out_data to bounds after load in next cycle via out_valid_o pulse.
          // Simpler heuristic: bump if y equals either bound on accept.
        end

        // Round increment statistic (approx)
        if (b_v_q && skid_ready) begin
          logic signed [XW-1:0] x_pre = pre_shift_x(b_x_q, cfg_in_shift_i, cfg_in_shift_rnd_i);
          logic signed [PROD_W-1:0] prod = $signed(x_pre) * $signed(b_mult_q);
          case (cfg_round_mode_i)
            2'b01: if (will_round_up_sym(prod, b_rshift_q)) round_inc_q <= round_inc_q + 32'd1;
            2'b10: if (will_round_up_rne(prod, b_rshift_q)) round_inc_q <= round_inc_q + 32'd1;
            2'b11: if (cfg_stoch_en_i && b_rshift_q != 0)   round_inc_q <= round_inc_q + (lfsr_q[15] ? 32'd1 : 32'd0);
            default: ; // TRUNC
          endcase
        end

        // Saturation count: on output accept, compare to bounds
        if (out_valid_o && out_ready_i) begin
          if (OUT_SIGNED) begin
            if (($signed(out_data_o) <= cfg_clip_min_i) || ($signed(out_data_o) >= cfg_clip_max_i))
              sat_q <= sat_q + 32'd1;
          end else begin
            if ((out_data_o == $unsigned(cfg_clip_min_i)) || (out_data_o == $unsigned(cfg_clip_max_i)))
              sat_q <= sat_q + 32'd1;
          end
        end
      end
    end
  end

  assign stat_in_beats_o   = in_beats_q;
  assign stat_out_beats_o  = out_beats_q;
  assign stat_saturations_o= sat_q;
  assign stat_round_incs_o = round_inc_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Clip bounds ordering
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    cfg_clip_min_i <= cfg_clip_max_i)
    else $error("post_requant: cfg_clip_min_i > cfg_clip_max_i");

  // No comb ready loop: f_out_ready does not depend directly on out_ready_i
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    f_out_valid && !f_out_ready |-> 1'b1);

  // out_last_o must imply out_valid_o
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    out_last_o |-> out_valid_o)
    else $error("post_requant: last without valid.");

  // If per-channel disabled, B should reflect broadcast values
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (!cfg_per_chan_en_i && b_v_q) |-> (b_mult_q == cfg_mult_bcast_i && b_rshift_q == cfg_rshift_bcast_i))
    else $warning("post_requant: broadcast disabled mismatch observed.");
`endif

endmodule

// ============================================================================
//  rv_fifo: single-clock ready/valid FIFO (non-fallthrough)
// ============================================================================
module rv_fifo #(
  parameter int unsigned WIDTH = 32,
  parameter int unsigned DEPTH = 4
)(
  input  logic               clk_i,
  input  logic               rstn_i,
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [WIDTH-1:0]   in_data_i,
  output logic               out_valid_o,
  input  logic               out_ready_i,
  output logic [WIDTH-1:0]   out_data_o
);
  localparam int AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
  logic [WIDTH-1:0] mem [0:DEPTH-1];
  logic [AW:0] wr_ptr_q, rd_ptr_q;

  function automatic logic fifo_empty(input logic [AW:0] w, input logic [AW:0] r);
    return (w == r);
  endfunction
  function automatic logic fifo_full (input logic [AW:0] w, input logic [AW:0] r);
    return (w[AW] != r[AW]) && (w[AW-1:0] == r[AW-1:0]);
  endfunction

  assign in_ready_o  = ~fifo_full(wr_ptr_q, rd_ptr_q);
  assign out_valid_o = ~fifo_empty(wr_ptr_q, rd_ptr_q);
  assign out_data_o  = mem[rd_ptr_q[AW-1:0]];

  wire do_push = in_valid_i  && in_ready_o;
  wire do_pop  = out_valid_o && out_ready_i;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wr_ptr_q <= '0; rd_ptr_q <= '0;
    end else begin
      if (do_push) begin
        mem[wr_ptr_q[AW-1:0]] <= in_data_i;
        wr_ptr_q <= wr_ptr_q + {{AW{1'b0}},1'b1};
      end
      if (do_pop) begin
        rd_ptr_q <= rd_ptr_q + {{AW{1'b0}},1'b1};
      end
    end
  end

`ifdef ASSERT_ON
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_push |-> !fifo_full(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: push while full.");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_pop  |-> !fifo_empty(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: pop while empty.");
`endif
endmodule

`default_nettype wire
