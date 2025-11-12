// ============================================================================
//  Bus Traffic Injector (A-path) — Global Buffer Master
//  File: bus_inject_a.sv
//
//  Purpose:
//    - Generate write and/or read traffic to the GB fabric to validate banks,
//      measure throughput, and stress arbiters.
//    - Deterministic data pattern from byte address => verification w/o golden RAM.
//    - Modes: FILL (write pass), READ_ONLY, READ_VERIFY, FILL_THEN_VERIFY.
//    - Throttleable, pausible; bank mask + bank_enable aware.
//    - Keeps at most **one outstanding read** (crossbar has no per-request tag).
//
//  Connects to: master-side of gb_crossbar.sv
//
//  Notes:
//    - Addresses are byte addresses; beat = DATA_W/8 bytes (aligned).
//    - Bank index bits live at [BANK_SEL_LSB +: log2(NBANKS)].
//    - Pattern generator is XORSHIFT-based (no multipliers).
//    - Read responses include ECC flags bubbled from sram_bank_wrapper.sv.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module bus_inject_a #(
  // ---------- Topology / geometry ----------
  parameter int unsigned NBANKS       = 8,   // power-of-two recommended
  parameter int unsigned DATA_W       = 512, // bits, multiple of 64 preferred
  parameter int unsigned ADDR_W       = 32,  // byte address width
  parameter int unsigned BANK_SEL_LSB = (DATA_W<=8)?0:$clog2(DATA_W/8),
  parameter int unsigned LANE_W       = 64,  // lane size for data pattern (must divide DATA_W)

  // ---------- Defaults / behavior ----------
  parameter bit          START_PAUSED = 1'b0
)(
  input  logic                           clk_i,
  input  logic                           rstn_i,      // synchronous, active-low

  // ==========================================================================
  // Downstream (to gb_crossbar) — master interface
  // ==========================================================================
  // Write (single beat)
  output logic                           wr_req_valid_o,
  input  logic                           wr_req_ready_i,
  output logic [ADDR_W-1:0]              wr_addr_o,
  output logic [DATA_W-1:0]              wr_data_o,
  output logic [DATA_W/8-1:0]            wr_strb_o,

  // Read request (single beat)
  output logic                           rd_req_valid_o,
  input  logic                           rd_req_ready_i,
  output logic [ADDR_W-1:0]              rd_addr_o,

  // Read response
  input  logic                           rd_rsp_valid_i,
  output logic                           rd_rsp_ready_o,
  input  logic [DATA_W-1:0]              rd_rsp_data_i,
  input  logic                           rd_rsp_sbe_i, // corrected single-bit
  input  logic                           rd_rsp_dbe_i, // detected double-bit

  // ==========================================================================
  // Configuration (drive from CSR)
  // ==========================================================================
  // Enable + mode
  input  logic                           cfg_en_i,
  // 0=IDLE, 1=FILL_ONLY, 2=READ_ONLY, 3=READ_VERIFY, 4=FILL_THEN_VERIFY
  input  logic [2:0]                     cfg_mode_i,

  // Region selection
  input  logic [NBANKS-1:0]              cfg_bank_mask_i,   // target banks
  input  logic [NBANKS-1:0]              bank_enable_i,     // live banks (from partition/power)
  input  logic [31:0]                    cfg_words_per_bank_i, // beats per bank (must fit)
  input  logic [31:0]                    cfg_start_word_i,  // start index within bank
  input  logic [31:0]                    cfg_stride_words_i,// step (>=1)

  // Throttle
  input  logic [15:0]                    cfg_gap_cyc_i,     // idle cycles between issued ops

  // Pattern control
  input  logic [63:0]                    cfg_seed_i,        // base seed mixed into pattern

  // Control pulses
  input  logic                           cmd_start_i,       // start current mode (unpauses)
  input  logic                           cmd_pause_i,
  input  logic                           cmd_resume_i,
  input  logic                           cmd_abort_i,       // immediate idle
  input  logic                           cmd_clr_counters_i,

  // ==========================================================================
  // Status / Telemetry
  // ==========================================================================
  output logic                           active_o,
  output logic                           paused_o,
  output logic [31:0]                    op_rd_issued_o,
  output logic [31:0]                    op_wr_issued_o,
  output logic [31:0]                    verify_ok_o,
  output logic [31:0]                    verify_err_o,
  output logic [31:0]                    sbe_seen_o,
  output logic [31:0]                    dbe_seen_o,
  output logic [$clog2((NBANKS>1)?NBANKS:2)-1:0] cur_bank_o,
  output logic [31:0]                    cur_word_o,
  output logic [ADDR_W-1:0]              last_addr_o,
  output logic [DATA_W-1:0]              last_exp_o,
  output logic [DATA_W-1:0]              last_got_o,
  output logic [DATA_W-1:0]              last_diff_o
);

  // ==========================================================================
  // Derived constants / checks
  // ==========================================================================
  localparam int unsigned BEAT_BYTES = DATA_W/8;
  localparam int unsigned BEAT_LSB   = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);
  localparam int unsigned BANK_W     = (NBANKS <= 1) ? 1 : $clog2(NBANKS);

  localparam int unsigned LOWER_W    = (BANK_SEL_LSB > BEAT_LSB) ? (BANK_SEL_LSB - BEAT_LSB) : 0;
  localparam int unsigned UPPER_W    = (ADDR_W > (BANK_SEL_LSB + BANK_W)) ? (ADDR_W - (BANK_SEL_LSB + BANK_W)) : 0;
  localparam int unsigned LOCIDX_W   = LOWER_W + UPPER_W;

  localparam int unsigned NLANE      = (LANE_W==0) ? 0 : (DATA_W / LANE_W);

  // Static parameter checks
  initial begin
    if (DATA_W % 8 != 0)    $error("bus_inject_a: DATA_W must be multiple of 8.");
    if (DATA_W % LANE_W != 0) $error("bus_inject_a: DATA_W must be multiple of LANE_W.");
    if (cfg_words_per_bank_i == 0) begin end // runtime check elsewhere
  end

  // ==========================================================================
  // Helpers
  // ==========================================================================
  function automatic bit is_pow2(input int unsigned x);
    return (x >= 1) && ((x & (x-1)) == 0);
  endfunction

  // Deterministic pattern from byte address (and lane)
  //   - cheap 64b XORSHIFT rounds on (seed ^ addr ^ lane_index)
  //   - repeated/expanded across lanes to fill DATA_W
  function automatic logic [LANE_W-1:0] xorshift64(input logic [63:0] x_in);
    logic [63:0] x;
    begin
      x = x_in;
      x ^= (x << 13);
      x ^= (x >> 7);
      x ^= (x << 17);
      xorshift64 = LANE_W'(x);
    end
  endfunction

  function automatic logic [DATA_W-1:0]
    gen_pattern(input logic [ADDR_W-1:0] byte_addr, input logic [63:0] seed);
    logic [DATA_W-1:0] out;
    out = '0;
    for (int l=0; l<NLANE; l++) begin
      // Mix: seed ^ (addr << 1) ^ lane_index ^ (addr >> 17)
      logic [63:0] mix = seed ^ { {(64-ADDR_W){1'b0}}, byte_addr } ^
                         64'(l) ^ ( { {(64-ADDR_W){1'b0}}, byte_addr } >> 17 );
      out[l*LANE_W +: LANE_W] = xorshift64(mix);
    end
    return out;
  endfunction

  // Compose byte address from (bank_id, bank-local word index)
  function automatic logic [ADDR_W-1:0]
    compose_addr(input logic [BANK_W-1:0] bank,
                 input logic [LOCIIDX_W-1:0] word_idx);
    logic [ADDR_W-1:0] a;
    logic [LOWER_W-1:0] low_part;
    logic [UPPER_W-1:0] high_part;
    a = '0;
    if (LOWER_W > 0) low_part  = word_idx[LOWER_W-1:0]; else low_part  = '0;
    if (UPPER_W > 0) high_part = word_idx[LOCIIDX_W-1:LOWER_W]; else high_part = '0;

    // Beat-aligned
    if (LOWER_W > 0) a[BANK_SEL_LSB-1 : BEAT_LSB] = low_part;
    a[BANK_SEL_LSB +: BANK_W] = bank;
    if (UPPER_W > 0) a[ADDR_W-1 : BANK_SEL_LSB + BANK_W] = high_part;
    return a;
  endfunction

  // Next enabled bank (wrap) in an effective mask
  function automatic logic [BANK_W-1:0]
    next_bank(input logic [NBANKS-1:0] m, input logic [BANK_W-1:0] cur);
    logic [BANK_W-1:0] i;
    for (i = cur + 1; i < NBANKS[BANK_W-1:0]; i++) if (m[i]) return i;
    for (i = '0; i <= cur; i++) if (m[i]) return i;
    return cur;
  endfunction

  // Effective mask = configured ∩ enabled
  wire [NBANKS-1:0] eff_mask = cfg_bank_mask_i & bank_enable_i;

  // ==========================================================================
  // State / control
  // ==========================================================================
  typedef enum logic [2:0] {
    S_IDLE,       // not running
    S_WAIT_GAP,   // throttle gap between ops
    S_ISSUE_WR,   // try write
    S_ISSUE_RD,   // try read
    S_WAIT_RD     // wait for read response
  } state_e;

  state_e                   st_q, st_d;

  // Scheduling pointers
  logic [BANK_W-1:0]        bank_q, bank_d;
  logic [31:0]              word_q, word_d;

  // Throttle countdown
  logic [15:0]              gap_q, gap_d;

  // Paused flag
  logic                     paused_q, paused_d;

  // Read outstanding tuple (single)
  logic                     rd_out_q, rd_out_d;
  logic [ADDR_W-1:0]        rd_addr_lat_q, rd_addr_lat_d;

  // Telemetry
  logic [31:0]              rd_cnt_q, rd_cnt_d;
  logic [31:0]              wr_cnt_q, wr_cnt_d;
  logic [31:0]              ok_cnt_q, ok_cnt_d;
  logic [31:0]              er_cnt_q, er_cnt_d;
  logic [31:0]              sbe_cnt_q, sbe_cnt_d;
  logic [31:0]              dbe_cnt_q, dbe_cnt_d;

  logic [ADDR_W-1:0]        last_addr_q, last_addr_d;
  logic [DATA_W-1:0]        last_exp_q,  last_exp_d;
  logic [DATA_W-1:0]        last_got_q,  last_got_d;
  logic [DATA_W-1:0]        last_diff_q, last_diff_d;

  // Mode helpers
  localparam logic [2:0] MODE_IDLE  = 3'd0;
  localparam logic [2:0] MODE_FILL  = 3'd1;
  localparam logic [2:0] MODE_RONLY = 3'd2;
  localparam logic [2:0] MODE_RVFY  = 3'd3;
  localparam logic [2:0] MODE_FVFY  = 3'd4;

  // Active when enabled, not paused, mask non-empty, and words>0
  function automatic bit mask_any(input logic [NBANKS-1:0] m); return (m != '0); endfunction
  wire run_ok = cfg_en_i && !paused_q && mask_any(eff_mask) && (cfg_words_per_bank_i != 0) &&
                (cfg_stride_words_i != 0) && (cfg_mode_i != MODE_IDLE);

  // Outputs defaults
  localparam [DATA_W/8-1:0] STRB_ALL = { (DATA_W/8){1'b1} };
  always_comb begin
    wr_req_valid_o = 1'b0;
    wr_addr_o      = '0;
    wr_data_o      = '0;
    wr_strb_o      = STRB_ALL;

    rd_req_valid_o = 1'b0;
    rd_addr_o      = '0;

    // Ready for read data only when waiting for it
    rd_rsp_ready_o = (st_q == S_WAIT_RD);
  end

  // Status outputs
  assign active_o        = (st_q != S_IDLE) && run_ok;
  assign paused_o        = paused_q;
  assign op_rd_issued_o  = rd_cnt_q;
  assign op_wr_issued_o  = wr_cnt_q;
  assign verify_ok_o     = ok_cnt_q;
  assign verify_err_o    = er_cnt_q;
  assign sbe_seen_o      = sbe_cnt_q;
  assign dbe_seen_o      = dbe_cnt_q;
  assign cur_bank_o      = bank_q;
  assign cur_word_o      = word_q;
  assign last_addr_o     = last_addr_q;
  assign last_exp_o      = last_exp_q;
  assign last_got_o      = last_got_q;
  assign last_diff_o     = last_diff_q;

  // ==========================================================================
  // Next-state logic
  // ==========================================================================
  always_comb begin
    // Hold-by-default
    st_d         = st_q;
    bank_d       = bank_q;
    word_d       = word_q;
    gap_d        = gap_q;
    paused_d     = paused_q;

    rd_out_d     = rd_out_q;
    rd_addr_lat_d= rd_addr_lat_q;

    rd_cnt_d     = rd_cnt_q;
    wr_cnt_d     = wr_cnt_q;
    ok_cnt_d     = ok_cnt_q;
    er_cnt_d     = er_cnt_q;
    sbe_cnt_d    = sbe_cnt_q;
    dbe_cnt_d    = dbe_cnt_q;

    last_addr_d  = last_addr_q;
    last_exp_d   = last_exp_q;
    last_got_d   = last_got_q;
    last_diff_d  = last_diff_q;

    // External controls
    if (cmd_abort_i) begin
      st_d      = S_IDLE;
      rd_out_d  = 1'b0;
    end
    if (cmd_pause_i)  paused_d = 1'b1;
    if (cmd_resume_i) paused_d = 1'b0;
    if (cmd_start_i) begin
      paused_d = 1'b0;
      // reset scan pointers to start
      // choose first enabled bank
      if (mask_any(eff_mask)) begin
        logic [BANK_W-1:0] first;
        first = '0;
        for (int i=0;i<NBANKS;i++) if (eff_mask[i]) begin first = BANK_W'(i); break; end
        bank_d = first;
      end
      word_d = cfg_start_word_i;
      st_d   = S_WAIT_GAP;
      gap_d  = 16'd0; // begin immediately
    end
    if (cmd_clr_counters_i) begin
      rd_cnt_d  = '0; wr_cnt_d='0; ok_cnt_d='0; er_cnt_d='0; sbe_cnt_d='0; dbe_cnt_d='0;
    end

    // Stop if cannot run
    if (!run_ok) begin
      if (st_q != S_IDLE) st_d = S_IDLE;
    end

    // FSM
    unique case (st_q)
      // ------------------------------------------------------
      S_IDLE: begin
        gap_d    = 16'd0;
        rd_out_d = 1'b0;
      end

      // ------------------------------------------------------
      S_WAIT_GAP: begin
        if (!run_ok) begin
          st_d = S_IDLE;
        end else if (gap_q != 16'd0) begin
          gap_d = gap_q - 16'd1;
        end else begin
          // If finished this bank, advance to next
          if (word_q >= cfg_words_per_bank_i) begin
            logic [BANK_W-1:0] nxt = next_bank(eff_mask, bank_q);
            if (!eff_mask[nxt]) begin
              st_d = S_IDLE; // mask emptied on-the-fly
            end else begin
              // If FILL_THEN_VERIFY and mode==FILL, switch to VERIFY after one full sweep across all banks
              // (We treat bank switch as sweep boundary only if we wrapped to first bank and word==start)
              bank_d = nxt;
              word_d = cfg_start_word_i;
              gap_d  = cfg_gap_cyc_i;
            end
          end else begin
            // Choose op based on mode
            unique case (cfg_mode_i)
              MODE_FILL:  st_d = S_ISSUE_WR;
              MODE_RONLY: st_d = S_ISSUE_RD;
              MODE_RVFY:  st_d = S_ISSUE_RD;
              MODE_FVFY:  st_d = S_ISSUE_WR; // first pass is fill; host can re-run with MODE_RVFY
              default:    st_d = S_IDLE;
            endcase
          end
        end
      end

      // ------------------------------------------------------
      S_ISSUE_WR: begin
        // Compose address and pattern from (bank, word)
        logic [ADDR_W-1:0] addr = compose_addr(bank_q, word_q[LOCIIDX_W-1:0]);
        logic [DATA_W-1:0] data = gen_pattern(addr, cfg_seed_i);

        wr_req_valid_o = 1'b1;
        wr_addr_o      = addr;
        wr_data_o      = data;

        if (wr_req_valid_o && wr_req_ready_i) begin
          wr_cnt_d = wr_cnt_q + 32'd1;
          // advance word by stride and apply gap
          word_d   = word_q + cfg_stride_words_i;
          gap_d    = cfg_gap_cyc_i;
          st_d     = S_WAIT_GAP;
        end
      end

      // ------------------------------------------------------
      S_ISSUE_RD: begin
        // Only one outstanding read
        if (!rd_out_q) begin
          logic [ADDR_W-1:0] addr = compose_addr(bank_q, word_q[LOCIIDX_W-1:0]);
          rd_req_valid_o = 1'b1;
          rd_addr_o      = addr;
          if (rd_req_valid_o && rd_req_ready_i) begin
            rd_out_d      = 1'b1;
            rd_addr_lat_d = addr;
            rd_cnt_d      = rd_cnt_q + 32'd1;
            st_d          = S_WAIT_RD;
          end
        end
      end

      // ------------------------------------------------------
      S_WAIT_RD: begin
        if (rd_rsp_valid_i && rd_rsp_ready_o) begin
          // Observe ECC flags
          if (rd_rsp_sbe_i) sbe_cnt_d = sbe_cnt_q + 32'd1;
          if (rd_rsp_dbe_i) dbe_cnt_d = dbe_cnt_q + 32'd1;

          // Verification check (if enabled)
          if (cfg_mode_i == MODE_RVFY) begin
            logic [DATA_W-1:0] exp = gen_pattern(rd_addr_lat_q, cfg_seed_i);
            logic [DATA_W-1:0] dif = exp ^ rd_rsp_data_i;

            last_addr_d = rd_addr_lat_q;
            last_exp_d  = exp;
            last_got_d  = rd_rsp_data_i;
            last_diff_d = dif;

            if (dif == '0) ok_cnt_d = ok_cnt_q + 32'd1;
            else           er_cnt_d = er_cnt_q + 32'd1;
          end

          // Clear outstanding and move on
          rd_out_d = 1'b0;

          // Advance word by stride and gap
          word_d = word_q + cfg_stride_words_i;
          gap_d  = cfg_gap_cyc_i;
          st_d   = S_WAIT_GAP;
        end
      end

      default: st_d = S_IDLE;
    endcase
  end

  // ==========================================================================
  // Sequential
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      st_q          <= S_IDLE;
      bank_q        <= '0;
      word_q        <= 32'd0;
      gap_q         <= 16'd0;
      paused_q      <= START_PAUSED;

      rd_out_q      <= 1'b0;
      rd_addr_lat_q <= '0;

      rd_cnt_q      <= 32'd0;
      wr_cnt_q      <= 32'd0;
      ok_cnt_q      <= 32'd0;
      er_cnt_q      <= 32'd0;
      sbe_cnt_q     <= 32'd0;
      dbe_cnt_q     <= 32'd0;

      last_addr_q   <= '0;
      last_exp_q    <= '0;
      last_got_q    <= '0;
      last_diff_q   <= '0;
    end else begin
      st_q          <= st_d;
      bank_q        <= bank_d;
      word_q        <= word_d;
      gap_q         <= gap_d;
      paused_q      <= paused_d;

      rd_out_q      <= rd_out_d;
      rd_addr_lat_q <= rd_addr_lat_d;

      rd_cnt_q      <= rd_cnt_d;
      wr_cnt_q      <= wr_cnt_d;
      ok_cnt_q      <= ok_cnt_d;
      er_cnt_q      <= er_cnt_d;
      sbe_cnt_q     <= sbe_cnt_d;
      dbe_cnt_q     <= dbe_cnt_d;

      last_addr_q   <= last_addr_d;
      last_exp_q    <= last_exp_d;
      last_got_q    <= last_got_d;
      last_diff_q   <= last_diff_d;
    end
  end

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Single outstanding read invariant
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rd_out_q |-> !(rd_req_valid_o && rd_req_ready_i))
    else $error("bus_inject_a: issued a read while another is outstanding.");

  // Beat alignment on issued ops
  if (BEAT_LSB > 0) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      (wr_req_valid_o && wr_req_ready_i) |-> (wr_addr_o[BEAT_LSB-1:0] == '0))
      else $error("bus_inject_a: write address not beat-aligned.");
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      (rd_req_valid_o && rd_req_ready_i) |-> (rd_addr_o[BEAT_LSB-1:0] == '0))
      else $error("bus_inject_a: read address not beat-aligned.");
  end

  // LOCIDX capacity check (runtime guard)
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    cfg_words_per_bank_i <= (LOCIIDX_W == 0 ? 1 : (32'((1 << LOCIDX_W)))))
    else $error("bus_inject_a: words_per_bank exceeds addressable per-bank index.");

`endif

endmodule

`default_nettype wire
