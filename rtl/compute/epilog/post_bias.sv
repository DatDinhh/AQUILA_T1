// ============================================================================
//  post_bias.sv
//  Integer post-bias stage with optional pre-shift, ReLU, and saturation.
//
//  Placement (typical):
//     pe_cell/pe_cluster/array → (C-stream) → post_bias → DMA/drain
//
//  Features
//  --------
//  - Per-channel bias add in the integer domain.
//  - Bias source: CSR-programmable table (2^CHAN_W entries) or broadcast scalar.
//  - Bias pre-shift: signed amount (+N: left shift, -N: arithmetic right shift)
//      * Optional symmetric rounding on right shifts.
//  - Optional ReLU activation after addition (for signed outputs).
//  - Saturating clamp to DATA_W (signed or unsigned).
//  - Clean RV handshake; 2-stage internal pipeline (addr → exec) + 1-entry skid.
//  - CSR port for bias programming and readback.
//  - Telemetry: beats in/out, saturations, ReLU zeros.
//
//  Notes
//  -----
//  - This is the INT path. For FP biasing, use a dedicated FP adder block.
//  - CHAN_W = 0 is allowed (bias table has depth 1; effectively broadcast).
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module post_bias #(
  // Stream widths
  parameter int unsigned DATA_W   = 16,  // C-stream element width (output width)
  parameter bit          OUT_SIGNED = 1'b1, // interpret output as signed when clamping/ReLU

  // Bias storage width (often ACC width, e.g., 32 for INT8/INT16 outputs)
  // The bias is pre-shifted and then added to the DATA_W value in an extended adder.
  parameter int unsigned BIAS_W   = 32,

  // Number of address bits for bias table (depth = 2^CHAN_W)
  parameter int unsigned CHAN_W   = 12,  // 4096 channels; set to 0 for broadcast-only

  // Internal arithmetic headroom (adder width = max(DATA_W, BIAS_W)+HEADROOM)
  // Choose >= number of bits of maximum left shift you expect.
  parameter int unsigned HEADROOM = 8
)(
  input  logic                     clk_i,
  input  logic                     rstn_i,            // synchronous, active-low

  // =================== C-stream in (one element per beat) ===================
  input  logic                     in_valid_i,
  output logic                     in_ready_o,
  input  logic [DATA_W-1:0]        in_data_i,        // C value (already quantized to DATA_W)
  input  logic [CHAN_W-1:0]        in_chan_i,        // output channel index
  input  logic                     in_last_i,        // passes through

  // =================== C-stream out ========================================
  output logic                     out_valid_o,
  input  logic                      out_ready_i,
  output logic [DATA_W-1:0]        out_data_o,       // post-bias result (clamped)
  output logic                     out_last_o,

  // =================== Config / control ====================================
  input  logic                     cfg_enable_i,     // 0=bypass (pass through in_data_i)
  // Bias source selection:
  input  logic                     cfg_bias_broadcast_en_i, // 1: use cfg_bias_broadcast_i
  input  logic [BIAS_W-1:0]        cfg_bias_broadcast_i,

  // Bias pre-shift (applied to bias before add)
  input  logic signed [7:0]        cfg_bias_shift_i,      // +N: <<N;  -N: >>>N (arith)
  input  logic                     cfg_bias_shift_rnd_i,  // right-shift: symmetric round-to-nearest

  // Optional post-add ReLU (signed outputs only; ignored when OUT_SIGNED==0)
  input  logic                     cfg_relu_en_i,

  // Telemetry control
  input  logic                     cmd_clr_counters_i,    // pulse: clears counters below

  // =================== CSR port for bias table ==============================
  input  logic                     csr_we_i,       // write enable
  input  logic                     csr_re_i,       // read enable
  input  logic [CHAN_W-1:0]        csr_addr_i,     // address
  input  logic [BIAS_W-1:0]        csr_wdata_i,    // write data
  output logic [BIAS_W-1:0]        csr_rdata_o,    // read data (registered 1-cycle)

  // =================== Telemetry ===========================================
  output logic [31:0]              stat_in_beats_o,
  output logic [31:0]              stat_out_beats_o,
  output logic [31:0]              stat_saturations_o,   // clamp events
  output logic [31:0]              stat_relu_zero_o      // outputs forced to 0 by ReLU
);

  // --------------------------------------------------------------------------
  // Derived widths / helpers
  // --------------------------------------------------------------------------
  localparam int unsigned ACC_W = ( (DATA_W > BIAS_W) ? DATA_W : BIAS_W ) + HEADROOM;

  function automatic int unsigned imax(input int unsigned a, input int unsigned b);
    return (a>b)?a:b;
  endfunction

  // --------------------------------------------------------------------------
  // Bias memory (simple dual-port): stream read + CSR read/write
  // - Stream read is synchronous (1-cycle read latency).
  // --------------------------------------------------------------------------
  localparam int unsigned DEPTH = (CHAN_W == 0) ? 1 : (1 << CHAN_W);

  // Synthesis hints (optional; tools may ignore)
  // (* ram_style = "block" *)  // uncomment if your tool supports it
  logic [BIAS_W-1:0] bias_mem [0:DEPTH-1];

  // Stream read port (A): address from stage-A (held input)
  logic [CHAN_W-1:0] bias_rd_addr_a;
  logic              bias_rd_en_a;
  logic [BIAS_W-1:0] bias_rd_data_a_q;

  // CSR port (B): read/write under csr_* handshakes
  logic [BIAS_W-1:0] csr_rdata_q;

  // Stream side read (1-cycle latency)
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      bias_rd_data_a_q <= '0;
    end else if (bias_rd_en_a) begin
      bias_rd_data_a_q <= bias_mem[bias_rd_addr_a];
    end
  end

  // CSR write/read
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      csr_rdata_q <= '0;
    end else begin
      if (csr_we_i) bias_mem[csr_addr_i] <= csr_wdata_i;
      if (csr_re_i) csr_rdata_q <= bias_mem[csr_addr_i];
    end
  end

  assign csr_rdata_o = csr_rdata_q;

  // --------------------------------------------------------------------------
  // Two-stage pipeline:
  //   Stage A: ingress hold (address phase)  -> drives bias read
  //   Stage B: execute phase (have bias + data) -> compute & push to skid
  // --------------------------------------------------------------------------
  // Stage A (ingress)
  logic               a_v_q;
  logic [DATA_W-1:0]  a_data_q;
  logic [CHAN_W-1:0]  a_chan_q;
  logic               a_last_q;

  // Stage B (execute)
  logic               b_v_q;
  logic [DATA_W-1:0]  b_data_q;
  logic               b_last_q;
  logic [BIAS_W-1:0]  b_bias_q;

  // Output skid
  logic               o_v_q, o_v_d;
  logic [DATA_W-1:0]  o_d_q, o_d_d;
  logic               o_l_q, o_l_d;

  // Handshakes
  wire skid_ready = (!o_v_q) || (o_v_q && out_ready_i);
  wire b_ready    = skid_ready && (!o_v_q || out_ready_i); // can accept execute result
  wire a_ready    = !a_v_q;                                // stage A empty
  wire in_fire    = in_valid_i && in_ready_o;

  assign in_ready_o = a_ready;

  // Load Stage A from input
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      a_v_q    <= 1'b0;
      a_data_q <= '0;
      a_chan_q <= '0;
      a_last_q <= 1'b0;
    end else begin
      // Move into stage A when empty and input beat arrives
      if (a_ready && in_valid_i) begin
        a_v_q    <= 1'b1;
        a_data_q <= in_data_i;
        a_chan_q <= in_chan_i;
        a_last_q <= in_last_i;
      end
      // Advance to stage B when B is ready to capture (next clock holds bias)
      if (a_v_q && !b_v_q && b_ready) begin
        // Stage B captures on next always_ff block (below)
        a_v_q <= 1'b0;
      end
    end
  end

  // Drive stream read port of bias RAM while A holds an item
  assign bias_rd_en_a   = a_v_q;                   // assert continuously while A holds
  assign bias_rd_addr_a = a_chan_q;                // stable address while A holds

  // Capture Stage B when A is holding and we have a (fresh) read sample
  // The RAM provides data with 1-cycle latency; this block captures it.
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      b_v_q    <= 1'b0;
      b_data_q <= '0;
      b_last_q <= 1'b0;
      b_bias_q <= '0;
    end else begin
      // Load B when A was valid in the *previous* cycle and B is free
      // Implementation: when A is valid and B is free, we clear A in this cycle,
      // and capture RAM data in the subsequent cycle. To keep sequencing simple,
      // we also allow continuous read and capture whenever B is free and A is (still) valid.
      if (!b_v_q && a_v_q) begin
        b_v_q    <= 1'b1;
        b_data_q <= a_data_q;
        b_last_q <= a_last_q;
        // bias_rd_data_a_q corresponds to the memory content for a_chan_q
        b_bias_q <= cfg_bias_broadcast_en_i ? cfg_bias_broadcast_i : bias_rd_data_a_q;
      end
      // B will be consumed below when we push to skid
      if (b_v_q && skid_ready) begin
        // compute stage below loads the skid and then we clear b_v_q
        b_v_q <= 1'b0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Bias pre-shift (signed arithmetic), addition, ReLU and saturation
  // --------------------------------------------------------------------------
  // Sign-extend operands into ACC_W
  function automatic logic signed [ACC_W-1:0] sext_data(input logic [DATA_W-1:0] x);
    if (OUT_SIGNED) sext_data = {{(ACC_W-DATA_W){x[DATA_W-1]}}, x};
    else            sext_data = {{(ACC_W-DATA_W){1'b0}}, x};
  endfunction

  function automatic logic signed [ACC_W-1:0] sext_bias(input logic [BIAS_W-1:0] b);
    // Bias is typically in accumulator domain (signed)
    return {{(ACC_W-BIAS_W){b[BIAS_W-1]}}, b};
  endfunction

  // Apply pre-shift with optional symmetric rounding for right shifts
  function automatic logic signed [ACC_W-1:0] bias_shift_apply(
    input logic signed [ACC_W-1:0] b_ext,
    input logic signed [7:0]       sh,
    input logic                    rnd
  );
    logic signed [ACC_W-1:0] t;
    if (sh >= 0) begin
      // Left shift; beware of overflow in later saturation
      t = b_ext <<< sh;
    end else begin
      logic [7:0] mag = -sh[7:0];
      if (mag == 0) t = b_ext;
      else if (rnd) begin
        // symmetric round-to-nearest: add +2^(mag-1) for non-negative b, else -2^(mag-1)
        logic signed [ACC_W-1:0] half = {{(ACC_W-1){1'b0}},1'b1} <<< (mag-1);
        logic signed [ACC_W-1:0] adj  = b_ext[ACC_W-1] ? -half : half;
        t = (b_ext + adj) >>> mag;
      end else begin
        t = b_ext >>> mag;
      end
    end
    return t;
  endfunction

  // Saturating clamp to DATA_W
  function automatic logic [DATA_W-1:0]
    sat_to_out (input logic signed [ACC_W-1:0] x, output logic did_saturate, input logic relu_en);
    logic signed [ACC_W-1:0] y = x;
    logic relu_zero = 1'b0;
    if (OUT_SIGNED && relu_en && x[ACC_W-1]) begin
      y = '0;
      relu_zero = 1'b1;
    end
    if (OUT_SIGNED) begin
      // Signed range: [-(2^(DATA_W-1)),  2^(DATA_W-1)-1]
      logic signed [ACC_W-1:0] smax = {{(ACC_W-DATA_W){1'b0}}, {1'b0, {DATA_W-1{1'b1}}}};
      logic signed [ACC_W-1:0] smin = {{(ACC_W-DATA_W){1'b1}}, {1'b1, {DATA_W-1{1'b0}}}};
      if (y > smax) begin did_saturate=1'b1; return smax[DATA_W-1:0]; end
      if (y < smin) begin did_saturate=1'b1; return smin[DATA_W-1:0]; end
      did_saturate = 1'b0;
      return y[DATA_W-1:0];
    end else begin
      // Unsigned range: [0, 2^DATA_W-1]; ReLU when OUT_SIGNED=0 is treated as clamp to zero
      logic [ACC_W-1:0] u = (y[ACC_W-1]) ? '0 : logic'(y);
      logic [ACC_W-1:0] umax = {{(ACC_W-DATA_W){1'b0}}, {DATA_W{1'b1}}};
      if (u > umax) begin did_saturate=1'b1; return {DATA_W{1'b1}}; end
      did_saturate = 1'b0;
      return u[DATA_W-1:0];
    end
  endfunction

  // Compute result when stage B is valid and skid can accept
  logic signed [ACC_W-1:0] add_a_w, add_b_w, add_sum_w;
  logic                    sat_evt_w;
  logic [DATA_W-1:0]       res_w;

  always_comb begin
    // Defaults: hold skid
    o_v_d = o_v_q;
    o_d_d = o_d_q;
    o_l_d = o_l_q;

    if (b_v_q && skid_ready) begin
      // Prepare operands
      add_a_w  = sext_data(b_data_q);
      add_b_w  = bias_shift_apply(sext_bias(b_bias_q), cfg_bias_shift_i, cfg_bias_shift_rnd_i);

      // Bypass mode: if cfg_enable_i==0, ignore bias and pass-through 'a'
      add_sum_w = cfg_enable_i ? (add_a_w + add_b_w) : add_a_w;

      // Saturate (+ optional ReLU)
      res_w = sat_to_out(add_sum_w, sat_evt_w, cfg_relu_en_i);

      // Load skid
      o_v_d = 1'b1;
      o_d_d = res_w;
      o_l_d = b_last_q;
    end

    // Pop skid on consumer accept
    if (o_v_q && out_ready_i) begin
      o_v_d = 1'b0;
    end
  end

  // Output regs
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      o_v_q <= 1'b0;
      o_d_q <= '0;
      o_l_q <= 1'b0;
    end else begin
      o_v_q <= o_v_d;
      o_d_q <= o_d_d;
      o_l_q <= o_l_d;
    end
  end

  assign out_valid_o = o_v_q;
  assign out_data_o  = o_d_q;
  assign out_last_o  = o_l_q;

  // --------------------------------------------------------------------------
  // Telemetry
  // --------------------------------------------------------------------------
  logic [31:0] in_beats_q,  in_beats_d;
  logic [31:0] out_beats_q, out_beats_d;
  logic [31:0] sats_q,      sats_d;
  logic [31:0] relu0_q,     relu0_d;

  wire out_fire = out_valid_o && out_ready_i;

  always_comb begin
    in_beats_d  = cmd_clr_counters_i ? 32'd0 : (in_beats_q  + (in_fire  ? 32'd1 : 32'd0));
    out_beats_d = cmd_clr_counters_i ? 32'd0 : (out_beats_q + (out_fire ? 32'd1 : 32'd0));

    // Saturation & ReLU-zero events counted when we produce a result into skid
    // (i.e., in the same cycle B→skid happens)
    if (cmd_clr_counters_i) begin
      sats_d  = 32'd0;
      relu0_d = 32'd0;
    end else begin
      sats_d  = sats_q  + ((b_v_q && skid_ready && sat_evt_w) ? 32'd1 : 32'd0);
      // ReLU-zero detection: when OUT_SIGNED && cfg_relu_en and result is zero while add_sum < 0
      // We derive from add_sum_w sign and cfg flags.
      relu0_d = relu0_q + ((b_v_q && skid_ready && OUT_SIGNED && cfg_relu_en_i && add_sum_w[ACC_W-1]) ? 32'd1 : 32'd0);
    end
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      in_beats_q  <= 32'd0;
      out_beats_q <= 32'd0;
      sats_q      <= 32'd0;
      relu0_q     <= 32'd0;
    end else begin
      in_beats_q  <= in_beats_d;
      out_beats_q <= out_beats_d;
      sats_q      <= sats_d;
      relu0_q     <= relu0_d;
    end
  end

  assign stat_in_beats_o     = in_beats_q;
  assign stat_out_beats_o    = out_beats_q;
  assign stat_saturations_o  = sats_q;
  assign stat_relu_zero_o    = relu0_q;

  // --------------------------------------------------------------------------
  // Assertions (simulation only)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // No combinational ready loops: in_ready_o depends only on a_v_q.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    in_valid_i && !in_ready_o |-> a_v_q)
    else $error("post_bias: backpressure without stage-A occupancy.");

  // When bypassed, output must equal input (modulo pipeline latency).
  // Check when B→skid loads under cfg_enable_i==0.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (b_v_q && cfg_enable_i==1'b0 && skid_ready) |-> (o_d_d == b_data_q))
    else $error("post_bias: bypass enabled but output differs from input.");

  // ReLU can only zero negative values when OUT_SIGNED and enabled
  if (!OUT_SIGNED) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      cfg_relu_en_i == 1'b0)
      else $warning("post_bias: ReLU asserted but OUT_SIGNED==0 (ignored).");
  end
`endif

endmodule

`default_nettype wire
