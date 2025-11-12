// ============================================================================
//  pe_cluster_2x2.sv
//  2×2 systolic cluster built from pe_cell.sv PEs (integer/quantized path)
//
//  Topology
//   Row 0:  (0,0)  -->  (0,1)   // activation flows west->east
//            |          |
//           (v)        (v)
//   Row 1:  (1,0)  -->  (1,1)   // weight flows north->south
//
//  Inputs per step (K-beat):
//    - a_west_i[2]: activation scalars for rows 0..1
//    - w_north_i[2]: weight scalars for cols 0..1
//    - mul_en_mask_i[3:0]: per-PE multiplier enable (sparsity gating)
//
//  Tile control:
//    - step_valid_i/step_ready_o, k_first_i/k_last_i, clear_acc_i
//
//  Output:
//    - Single C-stream, serializer order row-major: (0,0)->(0,1)->(1,0)->(1,1)
//      c_valid_o/c_ready_i, c_data_o[OUT_W-1:0], c_last_o (asserted on 4th beat)
//
//  Pass-through (to chain clusters):
//    - a_east_o[2] (+ valid) from rightmost column PE forwards
//    - w_south_o[2] (+ valid) from bottom row PE forwards
//
//  Notes
//    - Requires pe_cell.sv from this project (INT/quantized PE).
//    - No combinational ready loops: cluster ready is an AND of PE readies.
//    - If some PEs are disabled via pe_enable_i, their outputs are auto-drained
//      (ready=1) and not serialized.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pe_cluster_2x2 #(
  // ---------------- Geometry / formats (must match pe_cell) ----------------
  parameter int unsigned ACT_W       = 8,
  parameter int unsigned WGT_W       = 8,
  parameter int unsigned ACC_W       = 32,
  parameter int unsigned OUT_W       = 8,

  parameter bit          ACT_SIGNED  = 1'b1,
  parameter bit          WGT_SIGNED  = 1'b1,
  parameter bit          OUT_SIGNED  = 1'b1
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,            // synchronous, active-low

  // ======================== Step / Tile control ============================
  input  logic                         step_valid_i,
  output logic                         step_ready_o,
  input  logic                         k_first_i,
  input  logic                         k_last_i,
  input  logic                         clear_acc_i,

  // ======================== Systolic inputs ===============================
  input  logic [1:0][ACT_W-1:0]        a_west_i,         // row 0..1 activations (west edge)
  input  logic [1:0][WGT_W-1:0]        w_north_i,        // col 0..1 weights (north edge)

  // Per-step sparsity enable per PE (row-major mapping):
  //  bit0 -> (0,0), bit1 -> (0,1), bit2 -> (1,0), bit3 -> (1,1)
  input  logic [3:0]                   mul_en_mask_i,

  // Optional per-PE enable (1=active in serializer; 0=drop outputs and always-ready)
  input  logic [3:0]                   pe_enable_i,

  // =================== Quantization / Post-op controls ====================
  // Zero-points (shared per row/column)
  input  logic [1:0][ACT_W-1:0]        cfg_zero_a_row_i, // row 0..1
  input  logic [1:0][WGT_W-1:0]        cfg_zero_w_col_i, // col 0..1
  input  logic                         cfg_zp_enable_i,

  // Per-PE bias
  input  logic [3:0]                   cfg_bias_en_i,     // row-major bits
  input  logic [3:0][ACC_W-1:0]        cfg_bias_i,

  // Shared scaling / rounding / ReLU across cluster
  input  logic signed [7:0]            cfg_shift_pre_i,   // product shift pre-acc (+left / -right)
  input  logic signed [7:0]            cfg_shift_post_i,  // acc shift post (+right / -left)
  input  logic [1:0]                   cfg_round_mode_i,  // 0=TRUNC, 1=RNE
  input  logic                         cfg_relu_en_i,

  // ========================= Serialized C-stream ==========================
  output logic                         c_valid_o,
  input  logic                         c_ready_i,
  output logic [OUT_W-1:0]             c_data_o,
  output logic                         c_last_o,          // asserted on 4th emitted beat (or last enabled PE)
  output logic [1:0]                   c_src_idx_o,       // which PE (00,01,10,11) produced this beat

  // ======================= Pass-through (for tiling) ======================
  output logic [1:0][ACT_W-1:0]        a_east_o,          // to next cluster on the right
  output logic [1:0]                   a_east_valid_o,
  output logic [1:0][WGT_W-1:0]        w_south_o,         // to next cluster below
  output logic [1:0]                   w_south_valid_o
);

  // Local helpers
  localparam int R0C0 = 0;
  localparam int R0C1 = 1;
  localparam int R1C0 = 2;
  localparam int R1C1 = 3;

  // ------------------------------------------------------------------------
  // Wiring between PEs (systolic)
  // ------------------------------------------------------------------------
  // Forwarded scalars and valids
  logic [ACT_W-1:0] a_r0c0_to_r0c1;
  logic [ACT_W-1:0] a_r1c0_to_r1c1;
  logic [WGT_W-1:0] w_r0c0_to_r1c0;
  logic [WGT_W-1:0] w_r0c1_to_r1c1;

  logic v_r0c0_fwd, v_r0c1_fwd, v_r1c0_fwd, v_r1c1_fwd;

  // Step-ready feed from PEs
  logic step_rdy_r0c0, step_rdy_r0c1, step_rdy_r1c0, step_rdy_r1c1;

  // Cluster-level step ready (AND of PE readies; safe for last-step coordination)
  assign step_ready_o = step_rdy_r0c0 & step_rdy_r0c1 & step_rdy_r1c0 & step_rdy_r1c1;

  // ------------------------------------------------------------------------
  // Outputs from PEs to serializer
  // ------------------------------------------------------------------------
  logic [3:0]                 pe_c_valid;
  logic [3:0]                 pe_c_ready;
  logic [3:0]                 pe_c_last;   // (always 1 when valid, since 1 beat per tile)
  logic [3:0][OUT_W-1:0]      pe_c_data;

  // Ready defaults: if PE is disabled, hold ready high to drain any outputs
  // (prevents its skid from blocking future tiles)
  always_comb begin
    for (int i = 0; i < 4; i++) pe_c_ready[i] = 1'b0; // serializer drives selected PE, others 0
  end

  // ------------------------------------------------------------------------
  // Instantiate the 4 PEs (row-major: r0c0, r0c1, r1c0, r1c1)
  // ------------------------------------------------------------------------
  // r0c0 (top-left)
  pe_cell #(
    .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .OUT_W(OUT_W),
    .ACT_SIGNED(ACT_SIGNED), .WGT_SIGNED(WGT_SIGNED), .OUT_SIGNED(OUT_SIGNED)
  ) u_pe_r0c0 (
    .clk_i(clk_i), .rstn_i(rstn_i),
    .step_valid_i (step_valid_i),
    .step_ready_o (step_rdy_r0c0),
    .k_first_i    (k_first_i),
    .k_last_i     (k_last_i),
    .clear_acc_i  (clear_acc_i),
    .a_i          (a_west_i[0]),
    .w_i          (w_north_i[0]),
    .mul_en_i     (mul_en_mask_i[R0C0]),
    .cfg_zero_a_i (cfg_zero_a_row_i[0]),
    .cfg_zero_w_i (cfg_zero_w_col_i[0]),
    .cfg_bias_en_i(cfg_bias_en_i[R0C0]),
    .cfg_bias_i   (cfg_bias_i[R0C0]),
    .cfg_relu_en_i(cfg_relu_en_i),
    .cfg_zp_enable_i(cfg_zp_enable_i),
    .cfg_shift_pre_i (cfg_shift_pre_i),
    .cfg_shift_post_i(cfg_shift_post_i),
    .cfg_round_mode_i(cfg_round_mode_i),
    .c_valid_o    (pe_c_valid[R0C0]),
    .c_ready_i    (pe_c_ready[R0C0] | ~pe_enable_i[R0C0]), // auto-drain if disabled
    .c_data_o     (pe_c_data[R0C0]),
    .c_last_o     (pe_c_last[R0C0]),
    .acc_obs_o    (/* open or route to debug */),
    .a_fwd_o      (a_r0c0_to_r0c1),
    .w_fwd_o      (w_r0c0_to_r1c0),
    .fwd_valid_o  (v_r0c0_fwd)
  );

  // r0c1 (top-right)
  pe_cell #(
    .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .OUT_W(OUT_W),
    .ACT_SIGNED(ACT_SIGNED), .WGT_SIGNED(WGT_SIGNED), .OUT_SIGNED(OUT_SIGNED)
  ) u_pe_r0c1 (
    .clk_i(clk_i), .rstn_i(rstn_i),
    .step_valid_i (step_valid_i),
    .step_ready_o (step_rdy_r0c1),
    .k_first_i    (k_first_i),
    .k_last_i     (k_last_i),
    .clear_acc_i  (clear_acc_i),
    .a_i          (a_r0c0_to_r0c1),
    .w_i          (w_north_i[1]),
    .mul_en_i     (mul_en_mask_i[R0C1]),
    .cfg_zero_a_i (cfg_zero_a_row_i[0]),
    .cfg_zero_w_i (cfg_zero_w_col_i[1]),
    .cfg_bias_en_i(cfg_bias_en_i[R0C1]),
    .cfg_bias_i   (cfg_bias_i[R0C1]),
    .cfg_relu_en_i(cfg_relu_en_i),
    .cfg_zp_enable_i(cfg_zp_enable_i),
    .cfg_shift_pre_i (cfg_shift_pre_i),
    .cfg_shift_post_i(cfg_shift_post_i),
    .cfg_round_mode_i(cfg_round_mode_i),
    .c_valid_o    (pe_c_valid[R0C1]),
    .c_ready_i    (pe_c_ready[R0C1] | ~pe_enable_i[R0C1]),
    .c_data_o     (pe_c_data[R0C1]),
    .c_last_o     (pe_c_last[R0C1]),
    .acc_obs_o    (),
    .a_fwd_o      (a_east_o[0]),           // right edge output
    .w_fwd_o      (w_r0c1_to_r1c1),
    .fwd_valid_o  (v_r0c1_fwd)
  );
  assign a_east_valid_o[0] = v_r0c1_fwd;

  // r1c0 (bottom-left)
  pe_cell #(
    .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .OUT_W(OUT_W),
    .ACT_SIGNED(ACT_SIGNED), .WGT_SIGNED(WGT_SIGNED), .OUT_SIGNED(OUT_SIGNED)
  ) u_pe_r1c0 (
    .clk_i(clk_i), .rstn_i(rstn_i),
    .step_valid_i (step_valid_i),
    .step_ready_o (step_rdy_r1c0),
    .k_first_i    (k_first_i),
    .k_last_i     (k_last_i),
    .clear_acc_i  (clear_acc_i),
    .a_i          (a_west_i[1]),
    .w_i          (w_r0c0_to_r1c0),
    .mul_en_i     (mul_en_mask_i[R1C0]),
    .cfg_zero_a_i (cfg_zero_a_row_i[1]),
    .cfg_zero_w_i (cfg_zero_w_col_i[0]),
    .cfg_bias_en_i(cfg_bias_en_i[R1C0]),
    .cfg_bias_i   (cfg_bias_i[R1C0]),
    .cfg_relu_en_i(cfg_relu_en_i),
    .cfg_zp_enable_i(cfg_zp_enable_i),
    .cfg_shift_pre_i (cfg_shift_pre_i),
    .cfg_shift_post_i(cfg_shift_post_i),
    .cfg_round_mode_i(cfg_round_mode_i),
    .c_valid_o    (pe_c_valid[R1C0]),
    .c_ready_i    (pe_c_ready[R1C0] | ~pe_enable_i[R1C0]),
    .c_data_o     (pe_c_data[R1C0]),
    .c_last_o     (pe_c_last[R1C0]),
    .acc_obs_o    (),
    .a_fwd_o      (a_r1c0_to_r1c1),
    .w_fwd_o      (w_south_o[0]),          // bottom edge output
    .fwd_valid_o  (v_r1c0_fwd)
  );
  assign w_south_valid_o[0] = v_r1c0_fwd;

  // r1c1 (bottom-right)
  pe_cell #(
    .ACT_W(ACT_W), .WGT_W(WGT_W), .ACC_W(ACC_W), .OUT_W(OUT_W),
    .ACT_SIGNED(ACT_SIGNED), .WGT_SIGNED(WGT_SIGNED), .OUT_SIGNED(OUT_SIGNED)
  ) u_pe_r1c1 (
    .clk_i(clk_i), .rstn_i(rstn_i),
    .step_valid_i (step_valid_i),
    .step_ready_o (step_rdy_r1c1),
    .k_first_i    (k_first_i),
    .k_last_i     (k_last_i),
    .clear_acc_i  (clear_acc_i),
    .a_i          (a_r1c0_to_r1c1),
    .w_i          (w_r0c1_to_r1c1),
    .mul_en_i     (mul_en_mask_i[R1C1]),
    .cfg_zero_a_i (cfg_zero_a_row_i[1]),
    .cfg_zero_w_i (cfg_zero_w_col_i[1]),
    .cfg_bias_en_i(cfg_bias_en_i[R1C1]),
    .cfg_bias_i   (cfg_bias_i[R1C1]),
    .cfg_relu_en_i(cfg_relu_en_i),
    .cfg_zp_enable_i(cfg_zp_enable_i),
    .cfg_shift_pre_i (cfg_shift_pre_i),
    .cfg_shift_post_i(cfg_shift_post_i),
    .cfg_round_mode_i(cfg_round_mode_i),
    .c_valid_o    (pe_c_valid[R1C1]),
    .c_ready_i    (pe_c_ready[R1C1] | ~pe_enable_i[R1C1]),
    .c_data_o     (pe_c_data[R1C1]),
    .c_last_o     (pe_c_last[R1C1]),
    .acc_obs_o    (),
    .a_fwd_o      (a_east_o[1]),           // right edge output
    .w_fwd_o      (w_south_o[1]),          // bottom edge output
    .fwd_valid_o  (v_r1c1_fwd)
  );
  assign a_east_valid_o[1] = v_r1c1_fwd;
  assign w_south_valid_o[1] = v_r1c1_fwd; // same fire cadence throughout the mesh

  // ------------------------------------------------------------------------
  // Serializer: row-major (00 -> 01 -> 10 -> 11), per tile
  // ------------------------------------------------------------------------
  typedef enum logic [1:0] { SEL_00=2'd0, SEL_01=2'd1, SEL_10=2'd2, SEL_11=2'd3 } sel_e;

  sel_e                      sel_q, sel_d;
  logic [3:0]                pend_q, pend_d;   // which PEs remain to emit this tile
  logic                      busy_q, busy_d;   // currently draining a tile's 4 results
  logic                      out_v_q, out_v_d;
  logic [OUT_W-1:0]          out_d_q, out_d_d;
  logic [1:0]                out_src_q, out_src_d;

  // Start draining when the tile's last step is accepted
  wire tile_end_fire = step_valid_i & step_ready_o & k_last_i;

  // Effective set of PEs to drain (mask with pe_enable_i)
  wire [3:0] eff_mask = pe_enable_i;

  // Advance helper: find next set bit after 'sel'
  function automatic sel_e next_sel(input sel_e cur, input logic [3:0] m);
    sel_e s;
    s = cur;
    for (int k=0; k<4; k++) begin
      sel_e cand = sel_e'((cur + sel_e'(k+1)) & sel_e'('1));
      if (m[cand]) return cand;
    end
    return cur;
  endfunction

  // Selected PE wires
  wire sel_is_00 = (sel_q == SEL_00);
  wire sel_is_01 = (sel_q == SEL_01);
  wire sel_is_10 = (sel_q == SEL_10);
  wire sel_is_11 = (sel_q == SEL_11);

  // Current selected PE valid/data
  wire                 cur_v   = (sel_is_00 ? pe_c_valid[R0C0] :
                                  sel_is_01 ? pe_c_valid[R0C1] :
                                  sel_is_10 ? pe_c_valid[R1C0] :
                                              pe_c_valid[R1C1]);
  wire [OUT_W-1:0]     cur_d   = (sel_is_00 ? pe_c_data[R0C0] :
                                  sel_is_01 ? pe_c_data[R0C1] :
                                  sel_is_10 ? pe_c_data[R1C0] :
                                              pe_c_data[R1C1]);

  // Drive ready only to the selected (and enabled) PE when we handshake
  always_comb begin
    // default: 0 (set above)
    if (busy_q && cur_v && c_ready_i) begin
      case (sel_q)
        SEL_00: pe_c_ready[R0C0] = pe_enable_i[R0C0];
        SEL_01: pe_c_ready[R0C1] = pe_enable_i[R0C1];
        SEL_10: pe_c_ready[R1C0] = pe_enable_i[R1C0];
        SEL_11: pe_c_ready[R1C1] = pe_enable_i[R1C1];
        default: ;
      endcase
    end
  end

  // FSM
  always_comb begin
    busy_d   = busy_q;
    sel_d    = sel_q;
    pend_d   = pend_q;

    out_v_d  = out_v_q;
    out_d_d  = out_d_q;
    out_src_d= out_src_q;

    // Begin a new drain window on tile end
    if (tile_end_fire) begin
      busy_d = 1'b1;
      pend_d = eff_mask;   // which results to collect this tile
      sel_d  = SEL_00;
    end

    // When busy, pull from selected PE in order
    if (busy_q) begin
      // Ready to emit when current PE has its result
      if (cur_v) begin
        // Present on cluster output
        out_v_d   = 1'b1;
        out_d_d   = cur_d;
        out_src_d = sel_q;

        // On accept, consume and advance to next set bit
        if (c_ready_i) begin
          // clear this bit
          logic [3:0] nm = pend_q;
          nm[sel_q] = 1'b0;
          pend_d = nm;

          // Are we done?
          if (nm == 4'b0000) begin
            busy_d  = 1'b0;       // will deassert c_last via logic below
          end else begin
            // advance to next enabled PE
            sel_d = next_sel(sel_q, nm);
          end
        end
      end else begin
        // No data yet from current PE: keep waiting (PE holds its c_valid until serviced)
        out_v_d = 1'b0;
      end
    end else begin
      out_v_d = 1'b0;
    end
  end

  // Output register (no comb ready loops)
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      busy_q    <= 1'b0;
      sel_q     <= SEL_00;
      pend_q    <= 4'b0;
      out_v_q   <= 1'b0;
      out_d_q   <= '0;
      out_src_q <= 2'd0;
    end else begin
      busy_q    <= busy_d;
      sel_q     <= sel_d;
      pend_q    <= pend_d;
      out_v_q   <= out_v_d;
      out_d_q   <= out_d_d;
      out_src_q <= out_src_d;
    end
  end

  assign c_valid_o   = out_v_q;
  assign c_data_o    = out_d_q;
  assign c_src_idx_o = out_src_q;

  // c_last_o when accepting the final beat of the set (i.e., when pend will be 0 next)
  // We assert last on the cycle we present the *final* selected PE's beat.
  assign c_last_o = (busy_q && (pend_q == 4'b0001) && c_valid_o);

  // ------------------------------------------------------------------------
  // Pass-through valids (for chaining clusters)
  // ------------------------------------------------------------------------
  // The mesh fires in lockstep, so the forward valids share cadence.
  // We expose them per edge so neighbor clusters can synchronize.
  // Already wired above for right/bottom edges.

  // ------------------------------------------------------------------------
  // Assertions (simulation only)
  // ------------------------------------------------------------------------
`ifdef ASSERT_ON
  // Cluster ready must be 1 to accept a last step; otherwise the lane will stall
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (step_valid_i && k_last_i) |-> step_ready_o)
    else $warning("pe_cluster_2x2: last step attempted while not all PEs ready.");

  // Serializer must only ready one PE at a time
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (pe_c_ready[R0C0] + pe_c_ready[R0C1] + pe_c_ready[R1C0] + pe_c_ready[R1C1]) <= 1)
    else $error("pe_cluster_2x2: multiple PE c_ready asserted simultaneously.");

  // c_last_o only on the 4th beat (or last enabled)
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    c_last_o |-> c_valid_o)
    else $error("pe_cluster_2x2: c_last_o without c_valid_o.");

`endif

endmodule

`default_nettype wire
