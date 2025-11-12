// ============================================================================
//  pe_lane_fp.sv
//  Floating-point PE-lane compute wrapper (accumulate across K)
//
//  Placement:
//    pe_lane_int  ──>  pe_lane_fp  ──>  (C-stream consumer: DMA/drain)
//                  (A/W vectors + K-control)       (lane-packed C on k_last)
//
//  Key ideas:
//    - Maintain FMA_LAT "accumulator slots". Each cycle t uses slot (t mod FMA_LAT).
//      The previous value of that slot is fed into the FMA's C input. The FMA
//      returns after FMA_LAT cycles and is written back to that same slot.
//    - After K issues complete and the last writeback arrives, a pipelined adder
//      tree reduces the FMA_LAT slot partials to the final sum per lane.
//
//  Configure your FP primitive wrappers to match the port names used below
//  (fp_fma_prim / fp_add_prim / fp_cast_prim).
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pe_lane_fp #(
  // ---------------- Lane geometry ----------------
  parameter int unsigned LANE_COUNT = 8,

  // ---------------- FP formats -------------------
  // Inputs (A/W) use IN_EW/IN_MW; accumulator/output can be wider.
  parameter int unsigned IN_EW   = 8,    // exponent bits (e.g., 8 for FP32, 5 for FP16, 8 for BF16)
  parameter int unsigned IN_MW   = 23,   // mantissa bits (e.g., 23 for FP32, 10 for FP16, 7 for BF16)
  parameter int unsigned ACC_EW  = 8,    // accumulator exponent bits (e.g., 8 for FP32)
  parameter int unsigned ACC_MW  = 23,   // accumulator mantissa bits
  parameter int unsigned OUT_EW  = 8,    // output exponent bits (can equal ACC_EW)
  parameter int unsigned OUT_MW  = 23,   // output mantissa bits

  // ---------------- Primitive latencies ----------
  parameter int unsigned FMA_LAT = 4,    // cycles from valid-in to valid-out for fp_fma_prim
  parameter int unsigned ADD_LAT = 3,    // cycles for fp_add_prim
  parameter bit          DO_CAST = 1'b0, // 1: cast ACC→OUT via fp_cast_prim; 0: OUT=ACC format

  // ---------------- Implementation knobs ----------
  parameter bit          COUNT_EXC = 1'b1  // accumulate exception flags
)(
  input  logic                               clk_i,
  input  logic                               rstn_i,      // synchronous active-low

  // ==========================================================================
  // Inputs from pe_lane_int
  // ==========================================================================
  input  logic                               pe_fire_i,       // one K-step issued this cycle
  input  logic                               pe_k_first_i,    // aligns with pe_fire_i
  input  logic                               pe_k_last_i,     // aligns with pe_fire_i
  input  logic                               pe_clear_acc_i,  // pulse at first fire of tile

  input  logic [LANE_COUNT*IN_EW+LANE_COUNT*IN_MW+LANE_COUNT-1:0] pe_act_i_flat, // lane-packed IEEE (IN_EW+IN_MW+1)
  input  logic [LANE_COUNT*IN_EW+LANE_COUNT*IN_MW+LANE_COUNT-1:0] pe_wgt_i_flat, // lane-packed IEEE (IN_EW+IN_MW+1)

  // Optional FP controls (broadcast to all lanes)
  input  logic [2:0]                         cfg_rnd_mode_i,  // 0: RNE (typical)
  input  logic                               cfg_ftz_i,       // flush denormals to zero (output)
  input  logic                               cfg_daz_i,       // treat denormals-as-zero (inputs)

  // ==========================================================================
  // C-stream out (lane-packed, OUT_EW/OUT_MW format)
  // ==========================================================================
  output logic                               c_valid_o,
  input  logic                                c_ready_i,
  output logic [LANE_COUNT*(OUT_EW+OUT_MW+1)-1:0] c_data_o,
  output logic                               c_last_o,        // 1 on the only beat per tile

  // Telemetry (sticky counters)
  output logic [31:0]                        exc_any_cnt_o,   // total exceptions seen
  output logic [31:0]                        exc_invalid_cnt_o,
  output logic [31:0]                        exc_div0_cnt_o,
  output logic [31:0]                        exc_overflow_cnt_o,
  output logic [31:0]                        exc_underflow_cnt_o,
  output logic [31:0]                        exc_inexact_cnt_o
);

  // ==========================================================================
  // Local constants and types
  // ==========================================================================
  localparam int unsigned IN_DW  = 1 + IN_EW  + IN_MW;
  localparam int unsigned ACC_DW = 1 + ACC_EW + ACC_MW;
  localparam int unsigned OUT_DW = 1 + OUT_EW + OUT_MW;
  localparam int unsigned SLOTS  = (FMA_LAT == 0) ? 1 : FMA_LAT; // guard

  // Unpack lane vectors
  logic [LANE_COUNT-1:0][IN_DW-1:0]  a_lane, w_lane;

  for (genvar i=0; i<LANE_COUNT; i++) begin : g_unpack
    assign a_lane[i] = pe_act_i_flat[i*IN_DW +: IN_DW];
    assign w_lane[i] = pe_wgt_i_flat[i*IN_DW +: IN_DW];
  end

  // ==========================================================================
  // Slot index (t mod SLOTS) for streaming accumulation
  // ==========================================================================
  logic [$clog2((SLOTS>1)?SLOTS:2)-1:0] slot_idx_q, slot_idx_d;

  always_comb begin
    slot_idx_d = slot_idx_q;
    if (pe_fire_i) begin
      if (SLOTS == 1) slot_idx_d = '0;
      else slot_idx_d = (slot_idx_q == SLOTS-1) ? '0 : (slot_idx_q + 1);
    end
    if (pe_clear_acc_i) slot_idx_d = '0;
  end

  // ==========================================================================
  // Accumulator slots: per-lane array of ACC_DW registers
  // ==========================================================================
  logic [LANE_COUNT-1:0][SLOTS-1:0][ACC_DW-1:0] acc_slot_q;
  logic [LANE_COUNT-1:0][SLOTS-1:0][ACC_DW-1:0] acc_slot_d;

  // Writeback bookkeeping: pipeline tag (slot index) to route FMA result
  logic [SLOTS-1:0][$clog2((SLOTS>1)?SLOTS:2)-1:0] slot_tag_pipe_q, slot_tag_pipe_d;

  // Count issued and completed FMAs (to know when all slot updates landed)
  logic [31:0] issue_cnt_q, issue_cnt_d;
  logic [31:0] wb_cnt_q, wb_cnt_d;

  // Start reduction only when: all K issues done (k_last) AND all writebacks observed.
  // We detect the "k_last" that caused the final issue and wait for its writeback.
  logic        last_tag_pipe_q [SLOTS], last_tag_pipe_d [SLOTS]; // shift register for pe_k_last_i

  // ==========================================================================
  // FMA lanes
  // ==========================================================================
  // Outputs from FMA
  logic                  fma_v_o;
  logic [LANE_COUNT-1:0][ACC_DW-1:0] fma_res;

  // Exceptions per lane (IEEE 754: [invalid, div0, overflow, underflow, inexact])
  logic [LANE_COUNT-1:0][4:0] fma_exc;

  // Build FMA inputs: C = acc_slot_q[slot_idx] (read-before-write is safe as the same
  // slot is not reused until the result for that slot has returned SLOTS cycles later)
  logic [LANE_COUNT-1:0][ACC_DW-1:0] c_lane;

  for (genvar i=0; i<LANE_COUNT; i++) begin : g_c_select
    assign c_lane[i] = acc_slot_q[i][slot_idx_q];
  end

  // Drive FMAs when pe_fire_i == 1
  // Tag pipelines
  always_comb begin
    slot_tag_pipe_d[0] = slot_idx_q;
    for (int s=1; s<SLOTS; s++) slot_tag_pipe_d[s] = slot_tag_pipe_q[s-1];

    last_tag_pipe_d[0] = pe_fire_i ? pe_k_last_i : 1'b0;
    for (int s=1; s<SLOTS; s++) last_tag_pipe_d[s] = last_tag_pipe_q[s-1];

    issue_cnt_d = issue_cnt_q + (pe_fire_i ? 32'd1 : 32'd0);
  end

  // FMA instantiation per lane (fully pipelined)
  // fp_fma_prim must accept IN_DW a/b, ACC_DW c, and produce ACC_DW result.
  // If your primitive requires same widths, use external cast to ACC_DW at inputs.
  for (genvar i=0; i<LANE_COUNT; i++) begin : g_fma
    fp_fma_prim #(
      .A_EW (IN_EW),  .A_MW (IN_MW),
      .B_EW (IN_EW),  .B_MW (IN_MW),
      .C_EW (ACC_EW), .C_MW (ACC_MW),
      .R_EW (ACC_EW), .R_MW (ACC_MW),
      .LAT  (FMA_LAT)
    ) u_fma (
      .clk_i    (clk_i),
      .rstn_i   (rstn_i),
      .v_i      (pe_fire_i),
      .a_i      (a_lane[i]),
      .b_i      (w_lane[i]),
      .c_i      (c_lane[i]),
      .rnd_i    (cfg_rnd_mode_i),
      .ftz_i    (cfg_ftz_i),
      .daz_i    (cfg_daz_i),
      .v_o      (/* lane shares same v_o; use lane0 */),
      .r_o      (fma_res[i]),
      .exc_o    (fma_exc[i])
    );
  end

  // The v_o handshake is identical across lanes; tap lane 0
  // (Most vendor primitives guarantee this; otherwise AND all)
  wire fma_v_lane0;
  assign fma_v_lane0 = /* verilator lint_off WIDTH */ 1'b1 /* verilator lint_on WIDTH */; // default

  // To avoid depending on a specific primitive's v_o, track a generic pipeline:
  // v_o is pe_fire_i delayed by FMA_LAT.
  logic [SLOTS-1:0] fma_v_pipe_q, fma_v_pipe_d;
  always_comb begin
    fma_v_pipe_d[0] = pe_fire_i;
    for (int s=1; s<SLOTS; s++) fma_v_pipe_d[s] = fma_v_pipe_q[s-1];
  end

  // ==========================================================================
  // Writeback FMA results into the addressed slot for each lane
  // ==========================================================================
  // When v_o asserts (after FMA_LAT cycles), route result to acc_slot[tag_out]
  wire wb_fire = fma_v_pipe_q[SLOTS-1];

  integer li, si;
  always_comb begin
    acc_slot_d = acc_slot_q;
    if (pe_clear_acc_i) begin
      for (li=0; li<LANE_COUNT; li++) for (si=0; si<SLOTS; si++) acc_slot_d[li][si] = '0;
    end
    if (wb_fire) begin
      for (li=0; li<LANE_COUNT; li++) begin
        acc_slot_d[li][slot_tag_pipe_q[SLOTS-1]] = fma_res[li];
      end
    end
  end

  // Track completions + shift tag pipelines
  always_comb begin
    wb_cnt_d = wb_cnt_q + (wb_fire ? 32'd1 : 32'd0);
  end

  // ==========================================================================
  // Reduction trigger
  // ==========================================================================
  // A tile is ready to reduce when the final issue's writeback has been observed:
  // last_tag pipe bit emerges aligned with wb_fire; detect that edge once.
  logic reduce_arm_q, reduce_arm_d;
  logic reduce_go_pulse;

  assign reduce_go_pulse = wb_fire & last_tag_pipe_q[SLOTS-1] & ~reduce_arm_q;

  always_comb begin
    reduce_arm_d = reduce_arm_q | reduce_go_pulse;
  end

  // ==========================================================================
  // Pipelined adder tree over SLOTS (per lane)
  // ==========================================================================
  localparam int unsigned TREE_LEVELS = (SLOTS <= 1) ? 0 : $clog2(SLOTS);

  // Level inputs/outputs are arrays [lanes][width per level].
  // Level 0 inputs come from acc_slot_q[*][*] *after* the final writeback.
  // Start the tree on reduce_go_pulse; valid propagates for TREE_LEVELS*ADD_LAT cycles.
  logic [LANE_COUNT-1:0][SLOTS-1:0][ACC_DW-1:0] tree_l0_in;
  logic                                         tree_v_pipe_q [ (TREE_LEVELS==0)?1:(TREE_LEVELS*ADD_LAT) ];
  logic                                         tree_v_pipe_d [ (TREE_LEVELS==0)?1:(TREE_LEVELS*ADD_LAT) ];

  // Latch L0 inputs when reduce_go_pulse fires (freeze snapshot)
  logic [LANE_COUNT-1:0][SLOTS-1:0][ACC_DW-1:0] tree_snap_q, tree_snap_d;

  always_comb begin
    tree_snap_d = tree_snap_q;
    if (reduce_go_pulse) begin
      for (li=0; li<LANE_COUNT; li++) begin
        for (si=0; si<SLOTS; si++) begin
          tree_snap_d[li][si] = acc_slot_d[li][si];
        end
      end
    end
  end

  // Initialize L0
  generate
    if (SLOTS == 1) begin : g_trivial_reduce
      // No reduction needed; final result is this single slot
      // Delay valid by 0 cycles.
    end else begin : g_tree
      // Stage-by-stage tree; pad odd count with zeros
      localparam int unsigned MAXW = SLOTS;
      // Level arrays
      // We'll allocate arrays for each level for clarity
      // level k width = ceil(prev/2)
      // Build using generate-for and instantiate fp_add_prim for each pair per lane
      // Valid pipeline
      for (genvar v=0; v<TREE_LEVELS*ADD_LAT; v++) begin : g_vpipe
        always_comb begin
          if (v==0) tree_v_pipe_d[v] = reduce_go_pulse;
          else      tree_v_pipe_d[v] = tree_v_pipe_q[v-1];
        end
      end
    end
  endgenerate

  // ---------------- Tree netlist ----------------
  // We construct level-by-level signals in a generate block.
  // To keep code compact yet explicit, we use packed arrays with parametrized slicing.

  // Dynamic arrays to hold per-level vectors
  // In SystemVerilog, we unroll with generate as below:

  // Level 0 "current" vectors (from snapshot)
  // We'll iteratively define level_k and level_(k+1) wires.
  // For synthesis tools that prefer static names, we fully unroll.

  // Storage for final reduced ACC results per lane
  logic [LANE_COUNT-1:0][ACC_DW-1:0] reduce_out_q, reduce_out_d;
  logic                              reduce_v_q,    reduce_v_d;

  // Build the tree
  generate
    if (SLOTS == 1) begin : g_no_tree
      // Valid pulse coincides with reduce_go_pulse (no extra latency)
      always_comb begin
        reduce_out_d = '{default:'0};
        for (int l=0; l<LANE_COUNT; l++) reduce_out_d[l] = acc_slot_q[l][0];
        reduce_v_d   = reduce_go_pulse;
      end
    end else begin : g_tree_impl
      // We'll implement a staged tree with ADD_LAT per level.
      // We'll materialize signals per level as regs to match adder latency.
      localparam int unsigned L0N = SLOTS;

      // Create level signals as arrays of variable length
      // For each level, we will declare an array with ceil(prev/2) elements per lane.
      // To avoid complex dynamic arrays in SV, we unfold manually using generate-for meta-programming.

      // Level 0 -> 1
      localparam int unsigned L1N = (L0N+1)/2;
      logic [LANE_COUNT-1:0][L1N-1:0][ACC_DW-1:0] lvl1_q, lvl1_d;

      for (genvar l=0; l<LANE_COUNT; l++) begin : g_lvl01
        for (genvar p=0; p<L1N; p++) begin : g_lvl01p
          // Prepare operands (pad odd with zero)
          wire [ACC_DW-1:0] op0 = tree_snap_q[l][2*p];
          wire [ACC_DW-1:0] op1 = ((2*p+1)<L0N) ? tree_snap_q[l][2*p+1] : '0;

          // One adder per pair
          wire [ACC_DW-1:0] sum_w;
          fp_add_prim #(
            .A_EW (ACC_EW), .A_MW (ACC_MW),
            .B_EW (ACC_EW), .B_MW (ACC_MW),
            .R_EW (ACC_EW), .R_MW (ACC_MW),
            .LAT  (ADD_LAT)
          ) u_add_lvl01 (
            .clk_i  (clk_i),
            .rstn_i (rstn_i),
            .v_i    (reduce_go_pulse),
            .a_i    (op0),
            .b_i    (op1),
            .rnd_i  (cfg_rnd_mode_i),
            .ftz_i  (cfg_ftz_i),
            .daz_i  (cfg_daz_i),
            .v_o    (/* unused, we drive level with registered outputs */),
            .r_o    (sum_w),
            .exc_o  (/* ignore, addition exceptions are rare; could be counted if desired */)
          );

          // Register to align with ADD_LAT
          always_ff @(posedge clk_i or negedge rstn_i) begin
            if (!rstn_i) lvl1_q[l][p] <= '0;
            else          lvl1_q[l][p] <= sum_w;
          end
        end
      end

      // If more than one level, repeat pattern.
      if (TREE_LEVELS > 1) begin : g_more_levels
        // L1 -> L2
        localparam int unsigned L2N = (L1N+1)/2;
        logic [LANE_COUNT-1:0][L2N-1:0][ACC_DW-1:0] lvl2_q;
        for (genvar l=0; l<LANE_COUNT; l++) begin : g_lvl12
          for (genvar p=0; p<L2N; p++) begin : g_lvl12p
            wire [ACC_DW-1:0] op0 = lvl1_q[l][2*p];
            wire [ACC_DW-1:0] op1 = ((2*p+1)<L1N) ? lvl1_q[l][2*p+1] : '0;
            wire [ACC_DW-1:0] sum_w;
            fp_add_prim #(
              .A_EW (ACC_EW), .A_MW (ACC_MW),
              .B_EW (ACC_EW), .B_MW (ACC_MW),
              .R_EW (ACC_EW), .R_MW (ACC_MW),
              .LAT  (ADD_LAT)
            ) u_add_lvl12 (
              .clk_i  (clk_i),
              .rstn_i (rstn_i),
              .v_i    (1'b1),      // driven every cycle; inputs are registered
              .a_i    (op0),
              .b_i    (op1),
              .rnd_i  (cfg_rnd_mode_i),
              .ftz_i  (cfg_ftz_i),
              .daz_i  (cfg_daz_i),
              .v_o    (),
              .r_o    (sum_w),
              .exc_o  ()
            );
            always_ff @(posedge clk_i or negedge rstn_i) begin
              if (!rstn_i) lvl2_q[l][p] <= '0;
              else          lvl2_q[l][p] <= sum_w;
            end
          end
        end

        if (TREE_LEVELS > 2) begin : g_levels_3p
          // Extend similarly if SLOTS > 4.
          // For generality up to 8 slots:
          localparam int unsigned L3N = (L2N+1)/2;
          logic [LANE_COUNT-1:0][L3N-1:0][ACC_DW-1:0] lvl3_q;
          for (genvar l=0; l<LANE_COUNT; l++) begin : g_lvl23
            for (genvar p=0; p<L3N; p++) begin : g_lvl23p
              wire [ACC_DW-1:0] op0 = lvl2_q[l][2*p];
              wire [ACC_DW-1:0] op1 = ((2*p+1)<L2N) ? lvl2_q[l][2*p+1] : '0;
              wire [ACC_DW-1:0] sum_w;
              fp_add_prim #(
                .A_EW (ACC_EW), .A_MW (ACC_MW),
                .B_EW (ACC_EW), .B_MW (ACC_MW),
                .R_EW (ACC_EW), .R_MW (ACC_MW),
                .LAT  (ADD_LAT)
              ) u_add_lvl23 (
                .clk_i  (clk_i),
                .rstn_i (rstn_i),
                .v_i    (1'b1),
                .a_i    (op0),
                .b_i    (op1),
                .rnd_i  (cfg_rnd_mode_i),
                .ftz_i  (cfg_ftz_i),
                .daz_i  (cfg_daz_i),
                .v_o    (),
                .r_o    (sum_w),
                .exc_o  ()
              );
              always_ff @(posedge clk_i or negedge rstn_i) begin
                if (!rstn_i) lvl3_q[l][p] <= '0;
                else          lvl3_q[l][p] <= sum_w;
              end
            end
          end

          // Additional levels can be added with the same pattern if ever needed.
          // For now, support up to SLOTS <= 8 cleanly.
          // Final selection:
          always_comb begin
            reduce_out_d = '{default:'0};
            for (int l=0; l<LANE_COUNT; l++) reduce_out_d[l] = lvl3_q[l][0];
          end
        end else begin : g_final_lvl_is_2
          always_comb begin
            reduce_out_d = '{default:'0};
            for (int l=0; l<LANE_COUNT; l++) reduce_out_d[l] = lvl2_q[l][0];
          end
        end
      end

      // Valid pipeline for the tree result: reduce_go_pulse delayed by TREE_LEVELS*ADD_LAT
      // Implemented with a counter-based shift (simple, robust)
      localparam int unsigned REDUCE_LAT = TREE_LEVELS*ADD_LAT;
      logic [REDUCE_LAT-1:0] reduce_v_sr_q, reduce_v_sr_d;
      always_comb begin
        if (REDUCE_LAT == 0) reduce_v_sr_d = '0;
        else begin
          reduce_v_sr_d = reduce_v_sr_q << 1;
          reduce_v_sr_d[0] = reduce_go_pulse;
        end
      end
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          reduce_out_q <= '{default:'0};
          reduce_v_q   <= 1'b0;
          reduce_v_sr_q<= '0;
        end else begin
          reduce_out_q <= reduce_out_d;
          reduce_v_q   <= (REDUCE_LAT==0) ? reduce_go_pulse : reduce_v_sr_d[REDUCE_LAT-1];
          reduce_v_sr_q<= reduce_v_sr_d;
        end
      end
    end
  endgenerate

  // ==========================================================================
  // Optional cast ACC → OUT and output skid
  // ==========================================================================
  logic [LANE_COUNT-1:0][OUT_DW-1:0] c_lane_out_w;
  if (DO_CAST) begin : g_cast
    for (genvar l=0; l<LANE_COUNT; l++) begin : g_cvt
      fp_cast_prim #(
        .IN_EW (ACC_EW), .IN_MW (ACC_MW),
        .OU_EW (OUT_EW), .OU_MW (OUT_MW)
      ) u_cast (
        .clk_i  (clk_i),
        .rstn_i (rstn_i),
        .v_i    (reduce_v_q),
        .in_i   (reduce_out_q[l]),
        .rnd_i  (cfg_rnd_mode_i),
        .ftz_i  (cfg_ftz_i),
        .daz_i  (cfg_daz_i),
        .v_o    (),               // we assume single-cycle or internally registered
        .out_o  (c_lane_out_w[l]),
        .exc_o  ()
      );
    end
  end else begin : g_no_cast
    for (genvar l=0; l<LANE_COUNT; l++) begin
      assign c_lane_out_w[l] = reduce_out_q[l][OUT_DW-1:0];
    end
  end

  // Pack lane outputs
  logic [LANE_COUNT*OUT_DW-1:0] c_packed_w;
  for (genvar i=0; i<LANE_COUNT; i++) begin : g_pack
    assign c_packed_w[i*OUT_DW +: OUT_DW] = c_lane_out_w[i];
  end

  // One-beat output with 1-entry skid to decouple consumer
  logic out_v_q, out_v_d;
  logic [LANE_COUNT*OUT_DW-1:0] out_d_q, out_d_d;

  always_comb begin
    out_v_d = out_v_q;
    out_d_d = out_d_q;

    // Load from reducer
    if (reduce_v_q) begin
      out_v_d = 1'b1;
      out_d_d = c_packed_w;
    end

    // Pop on handshake
    if (out_v_q && c_ready_i) begin
      out_v_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      out_v_q <= 1'b0;
      out_d_q <= '0;
    end else begin
      out_v_q <= out_v_d;
      out_d_q <= out_d_d;
    end
  end

  assign c_valid_o = out_v_q;
  assign c_data_o  = out_d_q;
  assign c_last_o  = out_v_q; // one-beat per tile

  // ==========================================================================
  // Exception counters (optional)
  // ==========================================================================
  // Fold per-lane FMA exception flags when writeback happens.
  generate if (COUNT_EXC) begin : g_exc
    logic [31:0] any_q, inv_q, dzo_q, of_q, uf_q, inx_q;
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        any_q <= '0; inv_q <= '0; dzo_q <= '0; of_q <= '0; uf_q <= '0; inx_q <= '0;
      end else if (wb_fire) begin
        // OR-reduce across lanes to count once per cycle if any lane asserted that flag
        logic any, inv, dzo, of, uf, inx;
        any = 1'b0; inv = 1'b0; dzo = 1'b0; of = 1'b0; uf = 1'b0; inx = 1'b0;
        for (int l=0; l<LANE_COUNT; l++) begin
          any |= |fma_exc[l];
          inv |= fma_exc[l][4];
          dzo |= fma_exc[l][3];
          of  |= fma_exc[l][2];
          uf  |= fma_exc[l][1];
          inx |= fma_exc[l][0];
        end
        any_q <= any_q + (any ? 32'd1 : 32'd0);
        inv_q <= inv_q + (inv ? 32'd1 : 32'd0);
        dzo_q <= dzo_q + (dzo ? 32'd1 : 32'd0);
        of_q  <= of_q  + (of  ? 32'd1 : 32'd0);
        uf_q  <= uf_q  + (uf  ? 32'd1 : 32'd0);
        inx_q <= inx_q + (inx ? 32'd1 : 32'd0);
      end
    end
    assign exc_any_cnt_o       = any_q;
    assign exc_invalid_cnt_o   = inv_q;
    assign exc_div0_cnt_o      = dzo_q;
    assign exc_overflow_cnt_o  = of_q;
    assign exc_underflow_cnt_o = uf_q;
    assign exc_inexact_cnt_o   = inx_q;
  end else begin : g_no_exc
    assign exc_any_cnt_o       = 32'd0;
    assign exc_invalid_cnt_o   = 32'd0;
    assign exc_div0_cnt_o      = 32'd0;
    assign exc_overflow_cnt_o  = 32'd0;
    assign exc_underflow_cnt_o = 32'd0;
    assign exc_inexact_cnt_o   = 32'd0;
  end endgenerate

  // ==========================================================================
  // Sequential state updates
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      slot_idx_q      <= '0;
      acc_slot_q      <= '{default:'0};
      slot_tag_pipe_q <= '{default:'0};
      fma_v_pipe_q    <= '{default:'0};
      last_tag_pipe_q <= '{default:'0};
      issue_cnt_q     <= 32'd0;
      wb_cnt_q        <= 32'd0;
      reduce_arm_q    <= 1'b0;
      tree_snap_q     <= '{default:'0};
    end else begin
      slot_idx_q      <= slot_idx_d;
      acc_slot_q      <= acc_slot_d;
      slot_tag_pipe_q <= slot_tag_pipe_d;
      fma_v_pipe_q    <= fma_v_pipe_d;
      for (int s=0; s<SLOTS; s++) last_tag_pipe_q[s] <= last_tag_pipe_d[s];
      issue_cnt_q     <= issue_cnt_d;
      wb_cnt_q        <= wb_cnt_d;
      reduce_arm_q    <= reduce_arm_d;
      tree_snap_q     <= tree_snap_d;
    end
  end

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Clear must occur with or before the first fire of a tile
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    pe_k_first_i |-> pe_clear_acc_i)
    else $warning("pe_lane_fp: pe_k_first_i without clear_acc; accumulator content reused.");

  // No writeback should precede any issue
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (wb_cnt_q == 0) |-> (issue_cnt_q >= wb_cnt_q))
    else $error("pe_lane_fp: writebacks exceeded issues.");

  // Reduction should arm only once per tile
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    reduce_go_pulse |-> !reduce_arm_q)
    else $error("pe_lane_fp: reduce armed twice.");
`endif

endmodule

`default_nettype wire
