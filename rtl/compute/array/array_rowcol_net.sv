// ============================================================================
//  array_rowcol_net.sv
//  Row/Column broadcast network with global step handshake for a systolic array
//
//  Purpose
//  -------
//  - Buffer one K-step per *row* (activations) and per *column* (weights).
//  - Assert a single step_valid_o when all enabled rows/cols are holding.
//  - On step fire (step_valid_o & step_ready_i), atomically release all holds,
//    allowing the next K-step to be ingested from upstream.
//  - Drive row A to all columns, and column W(+mask) to all rows.
//
//  Interface
//  ---------
//  Upstream (per-row):
//    a_in_valid_i[R], a_in_ready_o[R], a_in_data_i[R][VEC*ACT_W], a_in_last_i[R]
//
//  Upstream (per-col):
//    w_in_valid_i[C], w_in_ready_o[C], w_in_data_i[C][VEC*WGT_W],
//    w_in_mask_i[C][VEC], w_in_last_i[C]
//
//  Array side (fanout):
//    A rows   : a_row_data_o[R][VEC*ACT_W], a_row_last_o[R], a_row_valid_o[R]
//    W columns: w_col_data_o[C][VEC*WGT_W], w_col_mask_o[C][VEC],
//               w_col_last_o[C], w_col_valid_o[C]
//
//  Global step:
//    step_valid_o -> step_ready_i  (array's combined readiness)
//    step_last_o  (all-row last AND all-col last on this step)
//
//  Notes
//  -----
//  - No combinational ready loops. Upstream *ready* depends only on local hold.
//  - Disabled rows/cols are treated as "always ready/holding zero" for step gating.
//  - Consumers should latch A/W on (step_valid_o && step_ready_i).
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module array_rowcol_net #(
  parameter int unsigned NROW   = 4,
  parameter int unsigned NCOL   = 4,
  parameter int unsigned VEC    = 16,
  parameter int unsigned ACT_W  = 8,
  parameter int unsigned WGT_W  = 8
)(
  input  logic                                clk_i,
  input  logic                                rstn_i,   // synchronous, active-low

  // ---------------- Configuration / partitioning ----------------
  input  logic [NROW-1:0]                     cfg_row_enable_i, // 1: row participates
  input  logic [NCOL-1:0]                     cfg_col_enable_i, // 1: col participates
  input  logic                                cfg_flush_i,      // pulse: clear holds/counters

  // ---------------- Upstream: per-row activation streams --------
  input  logic [NROW-1:0]                     a_in_valid_i,
  output logic [NROW-1:0]                     a_in_ready_o,
  input  logic [NROW-1:0][VEC*ACT_W-1:0]      a_in_data_i,
  input  logic [NROW-1:0]                     a_in_last_i,

  // ---------------- Upstream: per-col weight streams ------------
  input  logic [NCOL-1:0]                     w_in_valid_i,
  output logic [NCOL-1:0]                     w_in_ready_o,
  input  logic [NCOL-1:0][VEC*WGT_W-1:0]      w_in_data_i,
  input  logic [NCOL-1:0][VEC-1:0]            w_in_mask_i,
  input  logic [NCOL-1:0]                     w_in_last_i,

  // ---------------- Array-side fanout (broadcast) ---------------
  //  Consumers latch on (step_valid_o && step_ready_i)
  output logic [NROW-1:0]                     a_row_valid_o,
  output logic [NROW-1:0][VEC*ACT_W-1:0]      a_row_data_o,
  output logic [NROW-1:0]                     a_row_last_o,

  output logic [NCOL-1:0]                     w_col_valid_o,
  output logic [NCOL-1:0][VEC*WGT_W-1:0]      w_col_data_o,
  output logic [NCOL-1:0][VEC-1:0]            w_col_mask_o,
  output logic [NCOL-1:0]                     w_col_last_o,

  // ---------------- Global step handshake -----------------------
  output logic                                step_valid_o,
  input  logic                                step_ready_i,
  output logic                                step_last_o,

  // ---------------- Telemetry -----------------------------------
  output logic [31:0]                         stat_steps_o,      // steps fired
  output logic [31:0]                         stat_row_loads_o,  // total row beats loaded
  output logic [31:0]                         stat_col_loads_o,  // total col beats loaded
  output logic [31:0]                         stat_wait_cycles_o,// cycles waiting for missing rows/cols
  output logic                                stat_err_row_last_mismatch_o, // sticky
  output logic                                stat_err_col_last_mismatch_o  // sticky
);

  // ==========================================================================
  // Per-row holding registers
  // ==========================================================================
  logic [NROW-1:0]                    row_hold_v_q, row_hold_v_d;
  logic [NROW-1:0][VEC*ACT_W-1:0]     row_data_q,   row_data_d;
  logic [NROW-1:0]                    row_last_q,   row_last_d;

  // Upstream ready: only when not holding and row is enabled
  for (genvar r=0; r<NROW; r++) begin : g_row_ready
    assign a_in_ready_o[r] = cfg_row_enable_i[r] && !row_hold_v_q[r];
  end

  // Load rows
  for (genvar r2=0; r2<NROW; r2++) begin : g_row_load
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        row_hold_v_q[r2] <= 1'b0;
        row_data_q  [r2] <= '0;
        row_last_q  [r2] <= 1'b0;
      end else if (cfg_flush_i) begin
        row_hold_v_q[r2] <= 1'b0;
        row_data_q  [r2] <= '0;
        row_last_q  [r2] <= 1'b0;
      end else begin
        // Accept when enabled, valid, and we're not holding
        if (cfg_row_enable_i[r2] && a_in_valid_i[r2] && a_in_ready_o[r2]) begin
          row_hold_v_q[r2] <= 1'b1;
          row_data_q  [r2] <= a_in_data_i[r2];
          row_last_q  [r2] <= a_in_last_i[r2];
        end else if (/* cleared at step fire below */ 1'b0) begin
          // placeholder to emphasize single writer style
        end
      end
    end
  end

  // ==========================================================================
  // Per-column holding registers
  // ==========================================================================
  logic [NCOL-1:0]                    col_hold_v_q, col_hold_v_d;
  logic [NCOL-1:0][VEC*WGT_W-1:0]     col_data_q,   col_data_d;
  logic [NCOL-1:0][VEC-1:0]           col_mask_q,   col_mask_d;
  logic [NCOL-1:0]                    col_last_q,   col_last_d;

  for (genvar c=0; c<NCOL; c++) begin : g_col_ready
    assign w_in_ready_o[c] = cfg_col_enable_i[c] && !col_hold_v_q[c];
  end

  for (genvar c2=0; c2<NCOL; c2++) begin : g_col_load
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        col_hold_v_q[c2] <= 1'b0;
        col_data_q  [c2] <= '0;
        col_mask_q  [c2] <= '0;
        col_last_q  [c2] <= 1'b0;
      end else if (cfg_flush_i) begin
        col_hold_v_q[c2] <= 1'b0;
        col_data_q  [c2] <= '0;
        col_mask_q  [c2] <= '0;
        col_last_q  [c2] <= 1'b0;
      end else begin
        if (cfg_col_enable_i[c2] && w_in_valid_i[c2] && w_in_ready_o[c2]) begin
          col_hold_v_q[c2] <= 1'b1;
          col_data_q  [c2] <= w_in_data_i[c2];
          col_mask_q  [c2] <= w_in_mask_i[c2];
          col_last_q  [c2] <= w_in_last_i[c2];
        end else if (/* cleared at step fire below */ 1'b0) begin
        end
      end
    end
  end

  // ==========================================================================
  // Array-side fanout signals (combinational from holds)
  // ==========================================================================
  assign a_row_valid_o = row_hold_v_q;
  assign w_col_valid_o = col_hold_v_q;

  for (genvar r3=0; r3<NROW; r3++) begin : g_row_fan
    assign a_row_data_o[r3] = row_data_q[r3];
    assign a_row_last_o[r3] = row_last_q[r3];
  end
  for (genvar c3=0; c3<NCOL; c3++) begin : g_col_fan
    assign w_col_data_o[c3] = col_data_q[c3];
    assign w_col_mask_o[c3] = col_mask_q[c3];
    assign w_col_last_o[c3] = col_last_q[c3];
  end

  // ==========================================================================
  // Global step gating
  //  - step_valid when all *enabled* rows/cols are holding
  //  - step_last  when all held rows have last==1 and all held cols have last==1
  // ==========================================================================
  function automatic logic all_true_masked #(int N=1)
      (input logic [N-1:0] val, input logic [N-1:0] msk);
    // true if for all i: (msk[i]==0) or (val[i]==1)
    logic res; res = 1'b1;
    for (int i=0; i<N; i++) begin
      if (msk[i] && !val[i]) res = 1'b0;
    end
    return res;
  endfunction

  // All enabled rows/cols have a hold?
  wire rows_ready = all_true_masked#(NROW)(row_hold_v_q, cfg_row_enable_i);
  wire cols_ready = all_true_masked#(NCOL)(col_hold_v_q, cfg_col_enable_i);

  // All-held rows and cols agree on "last" (if enabled)?
  function automatic logic all_last_masked #(int N=1)
      (input logic [N-1:0] last, input logic [N-1:0] hold, input logic [N-1:0] en);
    // true if for all enabled & holding entries, last==1 (and if none, return 1)
    logic res; res = 1'b1;
    for (int i=0; i<N; i++) begin
      if (en[i] && hold[i]) res &= last[i];
    end
    return res;
  endfunction

  wire rows_last = all_last_masked#(NROW)(row_last_q, row_hold_v_q, cfg_row_enable_i);
  wire cols_last = all_last_masked#(NCOL)(col_last_q, col_hold_v_q, cfg_col_enable_i);

  assign step_valid_o = rows_ready & cols_ready;
  assign step_last_o  = rows_last  & cols_last;

  // ==========================================================================
  // Telemetry and error checks
  // ==========================================================================
  logic [31:0] steps_q, steps_d;
  logic [31:0] row_loads_q, col_loads_q;
  logic [31:0] wait_cyc_q, wait_cyc_d;
  logic        err_row_last_q, err_row_last_d;
  logic        err_col_last_q, err_col_last_d;

  // Load counters (increment on upstream accept)
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      row_loads_q <= 32'd0;
      col_loads_q <= 32'd0;
    end else if (cfg_flush_i) begin
      row_loads_q <= 32'd0;
      col_loads_q <= 32'd0;
    end else begin
      // count accepts
      for (int r=0; r<NROW; r++) begin
        if (cfg_row_enable_i[r] && a_in_valid_i[r] && a_in_ready_o[r])
          row_loads_q <= row_loads_q + 32'd1;
      end
      for (int c=0; c<NCOL; c++) begin
        if (cfg_col_enable_i[c] && w_in_valid_i[c] && w_in_ready_o[c])
          col_loads_q <= col_loads_q + 32'd1;
      end
    end
  end

  // Wait cycles: cycles where not ready to step due to missing rows/cols
  always_comb begin
    wait_cyc_d = wait_cyc_q;
    if (!(rows_ready && cols_ready)) wait_cyc_d = wait_cyc_q + 32'd1;
    if (cfg_flush_i) wait_cyc_d = 32'd0;
  end

  // Steps fired
  wire step_fire = step_valid_o & step_ready_i;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      steps_q <= 32'd0;
      wait_cyc_q <= 32'd0;
      err_row_last_q <= 1'b0;
      err_col_last_q <= 1'b0;
    end else begin
      steps_q    <= cfg_flush_i ? 32'd0 : (steps_q + (step_fire ? 32'd1 : 32'd0));
      wait_cyc_q <= wait_cyc_d;

      // Sticky mismatch flags: among enabled rows/cols, not all "last" high
      // at a time when step is otherwise ready.
      if (step_valid_o && !rows_last) err_row_last_q <= 1'b1;
      if (step_valid_o && !cols_last) err_col_last_q <= 1'b1;

      if (cfg_flush_i) begin
        err_row_last_q <= 1'b0;
        err_col_last_q <= 1'b0;
      end
    end
  end

  // ==========================================================================
  // Clear holds on step fire (atomic release)
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      // already cleared in per-row/col resets above
      for (int r=0; r<NROW; r++) begin
        row_hold_v_q[r] <= 1'b0;
      end
      for (int c=0; c<NCOL; c++) begin
        col_hold_v_q[c] <= 1'b0;
      end
    end else if (cfg_flush_i) begin
      for (int r=0; r<NROW; r++) row_hold_v_q[r] <= 1'b0;
      for (int c=0; c<NCOL; c++) col_hold_v_q[c] <= 1'b0;
    end else if (step_fire) begin
      for (int r=0; r<NROW; r++) if (cfg_row_enable_i[r]) row_hold_v_q[r] <= 1'b0;
      for (int c=0; c<NCOL; c++) if (cfg_col_enable_i[c]) col_hold_v_q[c] <= 1'b0;
    end
  end

  // ==========================================================================
  // Drive telemetry outputs
  // ==========================================================================
  assign stat_steps_o                     = steps_q;
  assign stat_row_loads_o                 = row_loads_q;
  assign stat_col_loads_o                 = col_loads_q;
  assign stat_wait_cycles_o               = wait_cyc_q;
  assign stat_err_row_last_mismatch_o     = err_row_last_q;
  assign stat_err_col_last_mismatch_o     = err_col_last_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Upstream must not present valid forever to a disabled row/col (warn only)
  for (genvar r4=0; r4<NROW; r4++) begin : g_assert_row_dis
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      cfg_row_enable_i[r4] || !a_in_valid_i[r4])
      else $warning("array_rowcol_net: Row %0d disabled but a_in_valid_i asserted.", r4);
  end
  for (genvar c4=0; c4<NCOL; c4++) begin : g_assert_col_dis
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      cfg_col_enable_i[c4] || !w_in_valid_i[c4])
      else $warning("array_rowcol_net: Col %0d disabled but w_in_valid_i asserted.", c4);
  end

  // When step fires, all enabled rows and cols must be holding
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    step_valid_o && step_ready_i |-> rows_ready && cols_ready)
    else $error("array_rowcol_net: step fired without all rows/cols holding.");

  // step_last_o is meaningful only when step_valid_o
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    step_last_o |-> step_valid_o)
    else $warning("array_rowcol_net: step_last_o high while step_valid_o=0.");

`endif

endmodule

`default_nettype wire
