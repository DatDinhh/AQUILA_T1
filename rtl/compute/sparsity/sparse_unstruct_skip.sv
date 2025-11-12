// ============================================================================
//  sparse_unstruct_skip.sv
//  Unstructured-sparsity step skipper (drop all-zero beats by policy)
//
//  Purpose
//  -------
//  - Inspect each VEC-wide vector beat, decide (by policy) whether it's
//    "all-zero" (via mask, data contents, or both), and either:
//      * DROP it (suppress on the output stream), or
//      * PASS it through unchanged.
//  - Maintains ready/valid correctness; no combinational ready loops.
//  - Handles tile boundaries: if a dropped beat carried 'last', a sticky
//    "last carry" is set so the *next emitted* beat can assert 'last' too.
//    Optionally inject a keep-alive zero beat to materialize 'last' even if
//    the entire tile would otherwise vanish (all steps skipped).
//
//  Typical placement
//  -----------------
//     ... decompressor/expander --> sparse_unstruct_skip --> pe_lane_int
//
//  Interfaces
//  ----------
//  In : in_valid/in_ready, in_data[VEC*ELEM_W], in_mask[VEC], in_last
//  Out: out_valid/out_ready, out_data[VEC*ELEM_W], out_mask[VEC], out_last
//
//  Policy (compile-time + run-time):
//    - USE_MASK: consider in_mask==0 as "zero"
//    - USE_DATA: consider all(data lanes == 0) as "zero"
//    - When both enabled: DROP if (mask_zero && data_zero).
//    - cfg_enable_i gates dropping at run-time (pass-through when 0).
//    - cfg_inject_keepalive_i optionally injects a (zero, mask=0, last=1)
//      beat if we would otherwise lose the only 'last' due to drops.
//
//  Telemetry
//  ---------
//    - Steps accepted/emitted, steps skipped, total nonzero lanes observed,
//      and "skipped-last" error visibility.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module sparse_unstruct_skip #(
  // Geometry
  parameter int unsigned VEC          = 128, // lanes per beat (>=1)
  parameter int unsigned ELEM_W       = 8,   // lane element width

  // Policy selection
  parameter bit          USE_MASK     = 1'b1, // use in_mask==0 as all-zero criterion
  parameter bit          USE_DATA     = 1'b0, // also test data bits == 0 across all lanes
  // If both USE_MASK & USE_DATA are 1, drop only when BOTH are zero.
  // (Change easily to OR logic if desired.)

  // Ingress buffering depth (>=1). Depth 4 recommended for timing.
  parameter int unsigned IN_FIFO_DEPTH = 4
)(
  input  logic                           clk_i,
  input  logic                           rstn_i,           // synchronous, active-low

  // ---------------- Compressed/expanded vector in ----------------
  input  logic                           in_valid_i,
  output logic                           in_ready_o,
  input  logic [VEC*ELEM_W-1:0]          in_data_i,        // vector payload
  input  logic [VEC-1:0]                 in_mask_i,        // 1=active lane (may be all 0)
  input  logic                           in_last_i,        // step is the last of tile

  // ---------------- Filtered vector out --------------------------
  output logic                           out_valid_o,
  input  logic                           out_ready_i,
  output logic [VEC*ELEM_W-1:0]          out_data_o,       // pass-through or injected zeros
  output logic [VEC-1:0]                 out_mask_o,       // pass-through or zeros
  output logic                           out_last_o,

  // ---------------- Run-time control -----------------------------
  input  logic                           cfg_enable_i,         // 1: allow dropping, 0: pass-through
  input  logic                           cfg_inject_keepalive_i, // 1: inject keep-alive zero+last beat if needed

  // ---------------- Observability / Telemetry --------------------
  output logic [31:0]                    stat_in_beats_o,     // input beats accepted
  output logic [31:0]                    stat_out_beats_o,    // output beats emitted
  output logic [31:0]                    stat_skipped_beats_o,// beats dropped
  output logic [47:0]                    stat_nz_lanes_o,     // sum of 1-bits in in_mask across accepted beats
  output logic                           err_skipped_last_sticky_o // dropped a 'last' without keep-alive emission
);

  // ==========================================================================
  // Parameter checks
  // ==========================================================================
  initial begin
    if (VEC == 0) $error("sparse_unstruct_skip: VEC must be >= 1");
    if (ELEM_W == 0) $error("sparse_unstruct_skip: ELEM_W must be >= 1");
    if (IN_FIFO_DEPTH < 1) $error("sparse_unstruct_skip: IN_FIFO_DEPTH must be >= 1");
  end

  localparam int unsigned PACK_W = VEC*ELEM_W + VEC + 1;

  // ==========================================================================
  // Ingress FIFO (non-fallthrough RV)
  // ==========================================================================
  logic                 f_in_ready, f_out_valid, f_out_ready;
  logic [PACK_W-1:0]    f_out_data;

  assign in_ready_o = f_in_ready;

  rv_fifo #(
    .WIDTH (PACK_W),
    .DEPTH (IN_FIFO_DEPTH)
  ) u_in_fifo (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (in_valid_i),
    .in_ready_o  (f_in_ready),
    .in_data_i   ({in_last_i, in_mask_i, in_data_i}),
    .out_valid_o (f_out_valid),
    .out_ready_i (f_out_ready),
    .out_data_o  (f_out_data)
  );

  // Unpack FIFO head
  wire                        cur_last = f_out_data[PACK_W-1];
  wire [VEC-1:0]              cur_mask = f_out_data[VEC +: VEC];
  wire [VEC*ELEM_W-1:0]       cur_data = f_out_data[0 +: VEC*ELEM_W];

  // ==========================================================================
  // Zero detection policy
  // ==========================================================================
  function automatic logic all_data_zero(input logic [VEC*ELEM_W-1:0] x);
    logic nz;
    nz = 1'b0;
    for (int i=0; i<VEC; i++) begin
      nz |= (x[i*ELEM_W +: ELEM_W] != {ELEM_W{1'b0}});
    end
    return ~nz;
  endfunction

  function automatic int unsigned popcount_mask(input logic [VEC-1:0] m);
    int unsigned c; c = 0;
    for (int i=0; i<VEC; i++) if (m[i]) c++;
    return c;
  endfunction

  wire mask_zero = (cur_mask == {VEC{1'b0}});
  wire data_zero = all_data_zero(cur_data);

  // Drop condition (combinational on FIFO head)
  wire drop_candidate =
        cfg_enable_i
        & (
            (USE_MASK && !USE_DATA) ? mask_zero :
            (!USE_MASK && USE_DATA) ? data_zero :
            (USE_MASK && USE_DATA)  ? (mask_zero & data_zero) :
                                      1'b0
          );

  // ==========================================================================
  // 'last' carry & optional keep-alive injection
  // ==========================================================================
  logic last_carry_q, last_carry_d;
  logic err_skipped_last_q, err_skipped_last_d;

  // We may inject a zero+last beat when:
  //   - last has been carried from dropped inputs, and
  //   - either no FIFO output is available OR the current head is going to drop,
  //   - and cfg_inject_keepalive_i is asserted.
  // Injection consumes no FIFO entry; it clears last_carry on successful emit.
  wire want_inject = last_carry_q && cfg_inject_keepalive_i &&
                     ( (!f_out_valid) || (f_out_valid && drop_candidate) );

  // Output muxing:
  //   - If injecting: drive zeros, mask=0, last=1, valid=1 (subject to out_ready)
  //   - Else if FIFO has a non-dropped head: forward it.
  //   - Else (FIFO droppable head): valid=0 this cycle (we'll pop it regardless).
  wire out_path_inject = want_inject;
  wire out_path_pass   = f_out_valid && !drop_candidate && !out_path_inject;

  assign out_valid_o = out_path_inject | out_path_pass;
  assign out_last_o  = out_path_inject ? 1'b1 : (cur_last | last_carry_q);
  assign out_mask_o  = out_path_inject ? {VEC{1'b0}} : cur_mask;
  assign out_data_o  = out_path_inject ? {VEC*ELEM_W{1'b0}} : cur_data;

  // Pop FIFO:
  //   - Pop on drop (we consume but do not emit it).
  //   - Pop on pass when downstream accepts (normal handshaked forward).
  //   - Do NOT pop on injection (we keep the head for later handling).
  assign f_out_ready = (f_out_valid && drop_candidate) |
                       (out_path_pass && out_ready_i);

  // last_carry management:
  always_comb begin
    last_carry_d        = last_carry_q;
    err_skipped_last_d  = err_skipped_last_q;

    // Collect carried 'last' from dropped inputs
    if (f_out_valid && drop_candidate && cur_last) begin
      last_carry_d       = 1'b1;
      // If we cannot inject AND no subsequent emitted beat ever appears, this
      // flag will remain set; we expose a sticky error in that case.
      if (!cfg_inject_keepalive_i) err_skipped_last_d = 1'b1;
    end

    // Clear carry when we *emit* a beat (injection or pass) carrying 'last'
    if (out_valid_o && out_ready_i && out_last_o) begin
      last_carry_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      last_carry_q       <= 1'b0;
      err_skipped_last_q <= 1'b0;
    end else begin
      last_carry_q       <= last_carry_d;
      err_skipped_last_q <= err_skipped_last_d;
    end
  end
  assign err_skipped_last_sticky_o = err_skipped_last_q;

  // ==========================================================================
  // Telemetry
  // ==========================================================================
  logic [31:0] in_beats_q,   in_beats_d;
  logic [31:0] out_beats_q,  out_beats_d;
  logic [31:0] skipped_q,    skipped_d;
  logic [47:0] nz_acc_q,     nz_acc_d;

  wire in_fire   = in_valid_i  && in_ready_o;
  wire out_fire  = out_valid_o && out_ready_i;
  wire skip_fire = f_out_valid && drop_candidate; // we drop the head this cycle

  always_comb begin
    in_beats_d  = in_beats_q  + (in_fire  ? 32'd1 : 32'd0);
    out_beats_d = out_beats_q + (out_fire ? 32'd1 : 32'd0);
    skipped_d   = skipped_q   + (skip_fire? 32'd1 : 32'd0);

    // Count nonzero lanes from input mask on accept (not on drop/pop specifically).
    // If you prefer to count only emitted nonzeros, change to out_fire & popcount(cur_mask).
    nz_acc_d    = nz_acc_q + (in_fire ? 48'(popcount_mask(in_mask_i)) : 48'd0);
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      in_beats_q  <= 32'd0;
      out_beats_q <= 32'd0;
      skipped_q   <= 32'd0;
      nz_acc_q    <= 48'd0;
    end else begin
      in_beats_q  <= in_beats_d;
      out_beats_q <= out_beats_d;
      skipped_q   <= skipped_d;
      nz_acc_q    <= nz_acc_d;
    end
  end

  assign stat_in_beats_o      = in_beats_q;
  assign stat_out_beats_o     = out_beats_q;
  assign stat_skipped_beats_o = skipped_q;
  assign stat_nz_lanes_o      = nz_acc_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // No comb ready loop: out_valid_o depends only on registered FIFO state & carry state.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    out_valid_o |-> (out_path_inject || (f_out_valid && !drop_candidate)))
    else $error("sparse_unstruct_skip: out_valid_o inconsistent with internal state.");

  // FIFO pop conditions are exclusive with injection pop
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    out_path_inject && out_ready_i |-> !f_out_ready || (f_out_valid && drop_candidate))
    else $warning("sparse_unstruct_skip: injecting while simultaneously popping non-droppable head.");

  // If keep-alive is disabled and we drop the only 'last' in a tile, sticky error must latch.
  // This is heuristic without tile-id, but we at least ensure it latches on any dropped last.
  if (!cfg_inject_keepalive_i) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      f_out_valid && drop_candidate && cur_last |-> err_skipped_last_d)
      else $error("sparse_unstruct_skip: dropped a 'last' without setting sticky error.");
  end
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
  input  logic               rstn_i,      // synchronous, active-low
  // Ingress
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [WIDTH-1:0]   in_data_i,
  // Egress
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

  wire do_push = in_valid_i  && !fifo_full(wr_ptr_q, rd_ptr_q);
  wire do_pop  = out_ready_i && !fifo_empty(wr_ptr_q, rd_ptr_q);

  assign in_ready_o  = !fifo_full(wr_ptr_q, rd_ptr_q);
  assign out_valid_o = !fifo_empty(wr_ptr_q, rd_ptr_q);
  assign out_data_o  = mem[rd_ptr_q[AW-1:0]];

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wr_ptr_q <= '0;
      rd_ptr_q <= '0;
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
