// ============================================================================
//  pe_lane_int.sv
//  Processing-Element Lane Interface: stream → decouple → align → PE vectors
//
//  Purpose
//  -------
//  - Buffer and align activation and weight streams for a PE lane with VEC
//    parallel MACs per cycle.
//  - Present per-cycle vectors to the PE with ready/valid handshake.
//  - Gate individual multipliers via a per-element mask (e.g., 2:4 sparsity).
//  - Provide tile control (k-length) and telemetry.
//
//  Interfaces
//  ----------
//  Upstream streams (post-decompression or raw):
//    * Activations: a_valid_i/a_ready_o, a_data_i[VEC*ACT_W-1:0], a_last_i
//    * Weights    : w_valid_i/w_ready_o, w_data_i[VEC*WGT_W-1:0],
//                   w_mask_i[VEC-1:0] (1=enable this multiplier), w_last_i
//
//  Toward PE:
//    * step_valid_o / step_ready_i
//    * a_vec_o[VEC*ACT_W-1:0], w_vec_o[VEC*WGT_W-1:0], mul_en_mask_o[VEC-1:0]
//    * step_last_o (high on final K-step of the tile)
//
//  Control:
//    * tile_start_i, k_len_i (#steps in K), flush_i, stall_i, pause/resume
//
//  Telemetry:
//    * a_underflow_o, w_underflow_o, stall_cycles_o, steps_issued_o
//    * err_misaligned_o (stream 'last' disagrees with k_len_i)
//
//  Notes
//  -----
//  - No arithmetic modification of the data here (zero-points, scaling, etc.).
//    Do that in the PE or in a preceding block. This wrapper focuses on timing,
//    alignment, and multiplier enable control.
//  - FIFOs are single-clock ready/valid, implemented below as rv_fifo.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pe_lane_int #(
  // Geometry
  parameter int unsigned VEC        = 16,   // parallel MACs per cycle (>=1)
  parameter int unsigned ACT_W      = 8,    // bits per activation element
  parameter int unsigned WGT_W      = 8,    // bits per weight element

  // FIFOing
  parameter int unsigned A_FIFO_DEPTH = 16, // >=2 recommended
  parameter int unsigned W_FIFO_DEPTH = 16, // >=2 recommended

  // Behavior
  parameter bit          MASK_ENABLE_BY_DEFAULT = 1'b1  // mask used unless cfg_mask_enable_i==0
)(
  input  logic                     clk_i,
  input  logic                     rstn_i,          // synchronous, active-low

  // ---------------- Tile Control ----------------
  input  logic                     tile_start_i,    // pulse to start a new tile
  input  logic [15:0]              k_len_i,         // # of K-steps in this tile (>0)
  input  logic                     flush_i,         // pulse: clear FIFOs & counters
  input  logic                     stall_i,         // level: force hold (e.g., PE replay)
  input  logic                     pause_i,         // level: pause ingestion & stepping
  input  logic                     resume_i,        // pulse: optional external resume (no state)

  // ---------------- Activation Stream In ----------------
  input  logic                     a_valid_i,
  output logic                     a_ready_o,
  input  logic [VEC*ACT_W-1:0]     a_data_i,
  input  logic                     a_last_i,        // asserted on last K-step of the tile

  // ---------------- Weight Stream In --------------------
  input  logic                     w_valid_i,
  output logic                     w_ready_o,
  input  logic [VEC*WGT_W-1:0]     w_data_i,
  input  logic [VEC-1:0]           w_mask_i,        // 1=enable multiplier; if disabled & masking off, ignored
  input  logic                     w_last_i,

  // Optional runtime control of mask usage
  input  logic                     cfg_mask_enable_i, // ANDed with MASK_ENABLE_BY_DEFAULT

  // ---------------- Toward PE ---------------------------
  output logic                     step_valid_o,
  input  logic                     step_ready_i,
  output logic [VEC*ACT_W-1:0]     a_vec_o,
  output logic [VEC*WGT_W-1:0]     w_vec_o,
  output logic [VEC-1:0]           mul_en_mask_o,
  output logic                     step_last_o,     // final step for this tile
  output logic                     tile_active_o,
  output logic                     tile_done_o,     // 1-cycle pulse when last step fires

  // ---------------- Telemetry ---------------------------
  output logic [31:0]              a_underflow_o,   // cycles where A empty but could step
  output logic [31:0]              w_underflow_o,   // cycles where W empty but could step
  output logic [31:0]              stall_cycles_o,  // cycles where both available but PE/stall blocked
  output logic [31:0]              steps_issued_o,  // #steps actually fired
  output logic                     err_misaligned_o // sticky: stream last vs k_len disagree
);

  // ==========================================================================
  // Parameter checks
  // ==========================================================================
  initial begin
    if (VEC == 0) $error("pe_lane_int: VEC must be >= 1");
    if (A_FIFO_DEPTH < 2) $warning("pe_lane_int: A_FIFO_DEPTH < 2 (throughput/elasticity limited).");
    if (W_FIFO_DEPTH < 2) $warning("pe_lane_int: W_FIFO_DEPTH < 2 (throughput/elasticity limited).");
  end

  // ==========================================================================
  // Local types & widths
  // ==========================================================================
  localparam int unsigned A_WORD_W = VEC*ACT_W + 1;       // +1 for a_last
  localparam int unsigned W_WORD_W = VEC*WGT_W + VEC + 1; // +mask + w_last

  // ==========================================================================
  // Ready/Valid FIFOs for A & W streams
  // ==========================================================================
  // Bundle activation payload
  wire              a_in_ready, a_out_valid, a_out_ready;
  wire [A_WORD_W-1:0] a_out_word;
  assign a_ready_o = a_in_ready;

  rv_fifo #(
    .WIDTH(A_WORD_W),
    .DEPTH(A_FIFO_DEPTH)
  ) u_a_fifo (
    .clk_i  (clk_i),
    .rstn_i (rstn_i & ~flush_i), // flush clears content
    .in_valid_i (a_valid_i),
    .in_ready_o (a_in_ready),
    .in_data_i  ({a_last_i, a_data_i}),
    .out_valid_o(a_out_valid),
    .out_ready_i(a_out_ready),
    .out_data_o (a_out_word)
  );

  // Bundle weight payload
  wire                w_in_ready, w_out_valid, w_out_ready;
  wire [W_WORD_W-1:0] w_out_word;
  assign w_ready_o = w_in_ready;

  rv_fifo #(
    .WIDTH(W_WORD_W),
    .DEPTH(W_FIFO_DEPTH)
  ) u_w_fifo (
    .clk_i  (clk_i),
    .rstn_i (rstn_i & ~flush_i),
    .in_valid_i (w_valid_i),
    .in_ready_o (w_in_ready),
    .in_data_i  ({w_last_i, w_mask_i, w_data_i}),
    .out_valid_o(w_out_valid),
    .out_ready_i(w_out_ready),
    .out_data_o (w_out_word)
  );

  // De-bundle heads
  wire                     a_head_last  = a_out_word[A_WORD_W-1];
  wire [VEC*ACT_W-1:0]     a_head_data  = a_out_word[VEC*ACT_W-1:0];

  wire                     w_head_last  = w_out_word[W_WORD_W-1];
  wire [VEC-1:0]           w_head_mask  = w_out_word[VEC + VEC*WGT_W - 1 -: VEC];
  wire [VEC*WGT_W-1:0]     w_head_data  = w_out_word[VEC*WGT_W-1:0];

  // ==========================================================================
  // Tile FSM / step control
  // ==========================================================================
  typedef enum logic [1:0] { T_IDLE, T_RUN, T_DONE } tstate_e;
  tstate_e t_st_q, t_st_d;

  logic [15:0] k_cnt_q, k_cnt_d;      // steps taken in this tile
  logic        misalign_sticky_q, misalign_sticky_d;

  wire mask_enabled = MASK_ENABLE_BY_DEFAULT & cfg_mask_enable_i;

  // Can step this cycle?
  wire both_avail = a_out_valid & w_out_valid;
  wire can_step   = both_avail & ~stall_i & ~pause_i & (t_st_q == T_RUN);

  // Handshake with PE
  assign step_valid_o = can_step;
  wire   step_fire    = can_step & step_ready_i;

  // Outputs to PE (combinational from FIFO heads)
  assign a_vec_o       = a_head_data;
  assign w_vec_o       = w_head_data;
  assign mul_en_mask_o = mask_enabled ? w_head_mask : {VEC{1'b1}};
  assign step_last_o   = (t_st_q == T_RUN) && (k_cnt_q == (k_len_i - 16'd1));

  assign a_out_ready   = step_fire; // pop only when PE takes the step
  assign w_out_ready   = step_fire;

  // Tile active/done
  assign tile_active_o = (t_st_q == T_RUN);
  logic tile_done_pulse_d, tile_done_pulse_q;
  assign tile_done_o   = tile_done_pulse_q;

  // Counters / telemetry
  logic [31:0] a_under_q, a_under_d;
  logic [31:0] w_under_q, w_under_d;
  logic [31:0] stall_cyc_q, stall_cyc_d;
  logic [31:0] steps_q, steps_d;

  // Next-state logic
  always_comb begin
    // Defaults
    t_st_d            = t_st_q;
    k_cnt_d           = k_cnt_q;
    misalign_sticky_d = misalign_sticky_q;

    tile_done_pulse_d = 1'b0;

    a_under_d   = a_under_q;
    w_under_d   = w_under_q;
    stall_cyc_d = stall_cyc_q;
    steps_d     = steps_q;

    // Flush clears counters (handled in seq)
    // Start new tile
    if (tile_start_i) begin
      t_st_d  = (k_len_i != 0) ? T_RUN : T_IDLE;
      k_cnt_d = 16'd0;
    end

    // Underflow / stall accounting (only when tile running)
    if (t_st_q == T_RUN && ~pause_i) begin
      // Could step if PE ready and not stalled
      if (~stall_i && step_ready_i) begin
        // One side missing?
        if (!a_out_valid && w_out_valid) a_under_d = a_under_q + 32'd1;
        if (!w_out_valid && a_out_valid) w_under_d = w_under_q + 32'd1;
      end
      // Both present, but PE/STALL blocked?
      if (both_avail && (stall_i || ~step_ready_i)) begin
        stall_cyc_d = stall_cyc_q + 32'd1;
      end
    end

    // Fire step
    if (step_fire) begin
      steps_d = steps_q + 32'd1;

      // Check stream 'last' vs k_len boundary
      logic expect_last = (k_cnt_q == (k_len_i - 16'd1));
      logic have_last   = a_head_last & w_head_last;
      if (expect_last != have_last) misalign_sticky_d = 1'b1;

      // Advance k counter and settle tile done
      if (k_cnt_q == (k_len_i - 16'd1)) begin
        t_st_d            = T_DONE;
        k_cnt_d           = 16'd0;
        tile_done_pulse_d = 1'b1;
      end else begin
        k_cnt_d           = k_cnt_q + 16'd1;
      end
    end

    // DONE → IDLE (one-cycle pulse)
    if (t_st_q == T_DONE) begin
      t_st_d = T_IDLE;
    end
  end

  // ==========================================================================
  // Sequential
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      t_st_q            <= T_IDLE;
      k_cnt_q           <= 16'd0;
      misalign_sticky_q <= 1'b0;

      tile_done_pulse_q <= 1'b0;

      a_under_q   <= 32'd0;
      w_under_q   <= 32'd0;
      stall_cyc_q <= 32'd0;
      steps_q     <= 32'd0;
    end else begin
      t_st_q            <= t_st_d;
      k_cnt_q           <= k_cnt_d;
      misalign_sticky_q <= misalign_sticky_d;

      tile_done_pulse_q <= tile_done_pulse_d;

      if (flush_i) begin
        a_under_q   <= 32'd0;
        w_under_q   <= 32'd0;
        stall_cyc_q <= 32'd0;
        steps_q     <= 32'd0;
        misalign_sticky_q <= 1'b0;
      end else begin
        a_under_q   <= a_under_d;
        w_under_q   <= w_under_d;
        stall_cyc_q <= stall_cyc_d;
        steps_q     <= steps_d;
      end
    end
  end

  // Telemetry outputs
  assign a_underflow_o    = a_under_q;
  assign w_underflow_o    = w_under_q;
  assign stall_cycles_o   = stall_cyc_q;
  assign steps_issued_o   = steps_q;
  assign err_misaligned_o = misalign_sticky_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // k_len must be >0 when starting a tile
  assert property (@(posedge clk_i) disable iff (!rstn_i)
    tile_start_i |-> (k_len_i != 16'd0))
    else $error("pe_lane_int: tile_start_i asserted with k_len_i=0.");

  // Pop only when both FIFOs valid and PE ready in RUN
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    step_valid_o && step_ready_i |-> (a_out_valid && w_out_valid && t_st_q==T_RUN && !stall_i && !pause_i))
    else $error("pe_lane_int: step fired under invalid conditions.");

  // 'last' should be true on both streams at tile end (informational)
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    step_valid_o && step_ready_i && (k_cnt_q == (k_len_i-1)) |-> (a_head_last && w_head_last))
    else $warning("pe_lane_int: tile end without both stream LASTs asserted.");

`endif

endmodule

// ============================================================================
//  rv_fifo: single-clock ready/valid FIFO (no comb ready loops)
//  - Non-fallthrough: out_valid=1 when count>0; pop on out_valid & out_ready.
//  - Push on in_valid & in_ready. Supports simultaneous push + pop.
//  - Depth >= 2 recommended.
// ============================================================================
module rv_fifo #(
  parameter int unsigned WIDTH = 32,
  parameter int unsigned DEPTH = 16
)(
  input  logic               clk_i,
  input  logic               rstn_i,        // synchronous, active-low
  // Ingress RV
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [WIDTH-1:0]   in_data_i,
  // Egress RV
  output logic               out_valid_o,
  input  logic               out_ready_i,
  output logic [WIDTH-1:0]   out_data_o
);
  localparam int AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
  // Storage
  logic [WIDTH-1:0] mem [0:DEPTH-1];
  logic [AW:0]      wr_ptr_q, rd_ptr_q; // extra MSB for full/empty detect

  function automatic logic fifo_empty(input logic [AW:0] w, input logic [AW:0] r);
    return (w == r);
  endfunction
  function automatic logic fifo_full (input logic [AW:0] w, input logic [AW:0] r);
    return (w[AW] != r[AW]) && (w[AW-1:0] == r[AW-1:0]);
  endfunction

  wire do_push = in_valid_i  && in_ready_o;
  wire do_pop  = out_valid_o && out_ready_i;

  // Combinational flags
  always_comb begin
    in_ready_o   = ~fifo_full(wr_ptr_q, rd_ptr_q);
    out_valid_o  = ~fifo_empty(wr_ptr_q, rd_ptr_q);
  end

  // Read data
  assign out_data_o = mem[rd_ptr_q[AW-1:0]];

  // Sequential logic
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wr_ptr_q <= '0;
      rd_ptr_q <= '0;
    end else begin
      if (do_push) begin
        mem[wr_ptr_q[AW-1:0]] <= in_data_i;
        wr_ptr_q              <= wr_ptr_q + {{AW{1'b0}},1'b1};
      end
      if (do_pop) begin
        rd_ptr_q              <= rd_ptr_q + {{AW{1'b0}},1'b1};
      end
    end
  end

`ifdef ASSERT_ON
  // No write on full; no read on empty
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_push |-> !fifo_full(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: push while full.");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_pop  |-> !fifo_empty(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: pop while empty.");
`endif

endmodule

`default_nettype wire
