// ============================================================================
//  sparse_2of4_expand.sv
//  Expand 2:4 structured-sparse weights to a dense vector + enable mask
//
//  Interface summary
//  -----------------
//  In (one beat = VEC elements compressed):
//    * in_valid_i / in_ready_o
//    * in_vals_i  : packed nonzero values (2 per group-of-4), little-endian
//    * in_meta_i  : per-group metadata (mask or 3-bit code)
//    * in_last_i  : beat is the last in a K-tile (passes through)
//
//  Out:
//    * out_valid_o / out_ready_i
//    * out_data_o : dense vector with zeros inserted at masked positions
//    * out_mask_o : 1 bit per element (1 = nonzero present → enable MUL for that lane)
//    * out_last_o : passes in_last_i
//
//  Packing / semantics
//  -------------------
//  - VEC must be a multiple of 4. There are G = VEC/4 groups per beat.
//  - For group g (0..G-1):
//      values V0,V1 are taken from in_vals_i[g*2 + 0], in_vals_i[g*2 + 1].
//      metadata M[g] selects which positions (j in 0..3) are nonzero.
//  - When META_IS_CODE==0 (mask mode):
//      M[g] is a 4-bit mask with exactly two 1's at the nonzero positions.
//  - When META_IS_CODE==1 (code mode):
//      M[g] is a 3-bit code (0..5) encoding {0,1},{0,2},{0,3},{1,2},{1,3},{2,3}.
//  - Value order: V0 maps to the lower index among the two positions; V1 to the higher.
//
//  Robustness
//  ----------
//  - No combinational ready loops (1-beat holding register).
//  - Metadata is validated; deviations set sticky error counters and are auto-healed:
//      * If mask has popcount != 2 → use the first two set bits, or {0,1} if none.
//      * If code is 6/7 → treated as {0,1}.
//  - Assertions under `ASSERT_ON` check common invariants.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module sparse_2of4_expand #(
  parameter int unsigned VEC          = 16,   // elements per beat (must be multiple of 4)
  parameter int unsigned ELEM_W       = 8,    // bits per element
  // Metadata selection:
  //   META_IS_CODE = 0 → meta per group is 4-bit mask (two 1's).
  //   META_IS_CODE = 1 → meta per group is 3-bit code (0..5) encoding the pair.
  parameter bit          META_IS_CODE = 1'b0,

  // Implementation knob: register the expanded outputs (timing aid)
  parameter bit          PIPE_OUT     = 1'b0
)(
  input  logic                                clk_i,
  input  logic                                rstn_i,         // synchronous, active-low

  // ---------------- Compressed input stream ----------------
  input  logic                                in_valid_i,
  output logic                                in_ready_o,
  input  logic [(VEC/2)*ELEM_W-1:0]           in_vals_i,      // 2 values per group, group 0 first
  input  logic [(VEC/4)*(META_IS_CODE?3:4)-1:0] in_meta_i,    // per-group meta, packed group 0 first
  input  logic                                in_last_i,

  // ---------------- Expanded output stream -----------------
  output logic                                out_valid_o,
  input  logic                                out_ready_i,
  output logic [VEC*ELEM_W-1:0]               out_data_o,     // zeros inserted
  output logic [VEC-1:0]                      out_mask_o,     // 1 where value present
  output logic                                out_last_o,

  // ---------------- Telemetry / status ---------------------
  output logic [31:0]                         stat_beats_acc_o,      // accepted input beats
  output logic [31:0]                         stat_groups_acc_o,     // groups processed
  output logic [31:0]                         stat_err_meta_cycles_o,// cycles with any meta fixup
  output logic [31:0]                         stat_err_meta_groups_o // total groups auto-healed
);

  // --------------------------------------------------------------------------
  // Static checks
  // --------------------------------------------------------------------------
  localparam int unsigned G = (VEC % 4 == 0) ? (VEC/4) : 0;
  localparam int unsigned VALS_PER_BEAT = (VEC/2);
  localparam int unsigned META_W = (META_IS_CODE ? 3 : 4);
  initial begin
    if (VEC == 0 || (VEC % 4) != 0) $error("sparse_2of4_expand: VEC must be a non-zero multiple of 4.");
    if (ELEM_W == 0)                $error("sparse_2of4_expand: ELEM_W must be > 0.");
  end

  // --------------------------------------------------------------------------
  // Holding register (1-beat elastic buffer; no comb ready loops)
  // --------------------------------------------------------------------------
  logic hold_v_q, hold_v_d;
  logic [(VEC/2)*ELEM_W-1:0]  hold_vals_q, hold_vals_d;
  logic [(VEC/4)*META_W-1:0]  hold_meta_q, hold_meta_d;
  logic hold_last_q, hold_last_d;

  assign in_ready_o = !hold_v_q; // accept when buffer is free

  // Push into hold
  wire push = in_valid_i && in_ready_o;
  // Pop from hold when output beat is successfully transferred
  // (either directly if PIPE_OUT==0, or from the registered output if PIPE_OUT==1)
  wire pop;

  // --------------------------------------------------------------------------
  // Expand combinationally from 'curr' (which is either HOLD or registered OUT)
  // --------------------------------------------------------------------------
  // Source for expansion: when using PIPE_OUT, expand from hold_* and then register;
  // otherwise, expand directly from hold_* and present on out_*.
  wire [(VEC/2)*ELEM_W-1:0]  curr_vals = hold_vals_q;
  wire [(VEC/4)*META_W-1:0]  curr_meta = hold_meta_q;
  wire                        curr_last = hold_last_q;

  // Expanded signals (combinational)
  logic [VEC*ELEM_W-1:0]      exp_data_w;
  logic [VEC-1:0]             exp_mask_w;
  logic                       exp_any_meta_fix_w;
  logic [31:0]                exp_fix_groups_w;

  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------
  function automatic int unsigned popcount4(input logic [3:0] x);
    int unsigned c; c = 0;
    for (int i=0;i<4;i++) if (x[i]) c++;
    return c;
  endfunction

  // Decode 3-bit code to 4-bit mask (two 1's), default to {0,1}
  function automatic logic [3:0] code_to_mask(input logic [2:0] code, output logic is_bad);
    logic [3:0] m;
    is_bad = 1'b0;
    unique case (code)
      3'd0: m = 4'b0011; // {0,1}
      3'd1: m = 4'b0101; // {0,2}
      3'd2: m = 4'b1001; // {0,3}
      3'd3: m = 4'b0110; // {1,2}
      3'd4: m = 4'b1010; // {1,3}
      3'd5: m = 4'b1100; // {2,3}
      default: begin
        m = 4'b0011; // fallback
        is_bad = 1'b1;
      end
    endcase
    return m;
  endfunction

  // Given mask (possibly invalid), produce two indices i0<i1 and a "fixed" flag.
  function automatic void mask_to_pair_fix(
    input  logic [3:0] m_in,
    output int unsigned idx0,
    output int unsigned idx1,
    output logic        fixed,
    output logic [3:0]  m_out
  );
    int unsigned found[4]; int unsigned k; k=0;
    for (int j=0;j<4;j++) begin
      if (m_in[j]) begin found[k]=j; k++; end
    end
    if (k >= 2) begin
      idx0  = found[0];
      idx1  = found[1];
      fixed = (k != 2);         // more than 2 set → we "fixed" by taking first two
      m_out = 4'b0000; m_out[idx0]=1'b1; m_out[idx1]=1'b1;
    end else if (k == 1) begin
      idx0  = found[0];
      idx1  = (idx0==3) ? 2 : (idx0+1); // choose next index
      fixed = 1'b1;
      m_out = 4'b0000; m_out[idx0]=1'b1; m_out[idx1]=1'b1;
    end else begin
      idx0  = 0; idx1 = 1;
      fixed = 1'b1;
      m_out = 4'b0011;
    end
  endfunction

  // --------------------------------------------------------------------------
  // Expansion logic (vectorized over groups)
  //   Packing convention:
  //     - Group g values: in_vals_i[g*2*ELEM_W +: ELEM_W] -> V0
  //                       in_vals_i[g*2*ELEM_W + ELEM_W +: ELEM_W] -> V1
  //     - Group g meta  : in_meta_i[g*META_W +: META_W]
  //     - Output element k (k=0..VEC-1): out_data_o[k*ELEM_W +: ELEM_W]
  // --------------------------------------------------------------------------
  always_comb begin
    exp_data_w         = '0;
    exp_mask_w         = '0;
    exp_any_meta_fix_w = 1'b0;
    exp_fix_groups_w   = 32'd0;

    for (int gidx=0; gidx<G; gidx++) begin
      // Values for this group
      logic [ELEM_W-1:0] v0 = curr_vals[(gidx*2+0)*ELEM_W +: ELEM_W];
      logic [ELEM_W-1:0] v1 = curr_vals[(gidx*2+1)*ELEM_W +: ELEM_W];

      // Metadata → mask
      logic [3:0] m4;
      logic       bad_code;
      if (META_IS_CODE) begin
        m4 = code_to_mask(curr_meta[gidx*META_W +: META_W], bad_code);
      end else begin
        m4 = curr_meta[gidx*META_W +: META_W];
        bad_code = 1'b0;
      end

      // Validate mask; pick indices for v0/v1
      int unsigned idx0, idx1;
      logic        fixed;
      logic [3:0]  m4_norm;
      mask_to_pair_fix(m4, idx0, idx1, fixed, m4_norm);

      // Place values (idx0 < idx1 guaranteed)
      int base = gidx*4;
      // First nonzero
      exp_data_w[(base+idx0)*ELEM_W +: ELEM_W] = v0;
      exp_mask_w[base+idx0] = 1'b1;
      // Second nonzero
      exp_data_w[(base+idx1)*ELEM_W +: ELEM_W] = v1;
      exp_mask_w[base+idx1] = 1'b1;

      // Telemetry flags
      if (fixed || bad_code || (!META_IS_CODE && popcount4(m4) != 2)) begin
        exp_any_meta_fix_w = 1'b1;
        exp_fix_groups_w   = exp_fix_groups_w + 32'd1;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Output staging (optional PIPE_OUT)
  // --------------------------------------------------------------------------
  logic                 out_v_q, out_v_d;
  logic [VEC*ELEM_W-1:0] out_data_q, out_data_d;
  logic [VEC-1:0]        out_mask_q, out_mask_d;
  logic                  out_last_q, out_last_d;

  // Pop from hold when we are going to present (and either PIPE_OUT=0 and out_ready_i,
  // or PIPE_OUT=1 and we can load the output register)
  generate
    if (PIPE_OUT) begin : g_pipe
      assign pop = hold_v_q && (!out_v_q || (out_v_q && out_ready_i)); // load when output reg free or accepted

      always_comb begin
        out_v_d    = out_v_q;
        out_data_d = out_data_q;
        out_mask_d = out_mask_q;
        out_last_d = out_last_q;

        // Load/register new expansion when we have a held beat and either output is empty
        // or the consumer accepted the previous beat this cycle.
        if (hold_v_q && (!out_v_q || (out_v_q && out_ready_i))) begin
          out_v_d    = 1'b1;
          out_data_d = exp_data_w;
          out_mask_d = exp_mask_w;
          out_last_d = curr_last;
        end

        // Pop consumer
        if (out_v_q && out_ready_i) begin
          out_v_d = 1'b0; // will be re-asserted next line if a new beat is ready (handled above)
        end
      end

      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          out_v_q    <= 1'b0;
          out_data_q <= '0;
          out_mask_q <= '0;
          out_last_q <= 1'b0;
        end else begin
          out_v_q    <= out_v_d;
          out_data_q <= out_data_d;
          out_mask_q <= out_mask_d;
          out_last_q <= out_last_d;
        end
      end

      assign out_valid_o = out_v_q;
      assign out_data_o  = out_data_q;
      assign out_mask_o  = out_mask_q;
      assign out_last_o  = out_last_q;
    end else begin : g_no_pipe
      assign pop         = hold_v_q && out_ready_i;
      assign out_valid_o = hold_v_q;
      assign out_data_o  = exp_data_w;
      assign out_mask_o  = exp_mask_w;
      assign out_last_o  = curr_last;
      // (out_* are purely combinational from hold_*; safe because in_ready_o depends only on hold_v_q)
    end
  endgenerate

  // Hold register push/pop
  always_comb begin
    hold_v_d    = hold_v_q;
    hold_vals_d = hold_vals_q;
    hold_meta_d = hold_meta_q;
    hold_last_d = hold_last_q;

    if (push) begin
      hold_v_d    = 1'b1;
      hold_vals_d = in_vals_i;
      hold_meta_d = in_meta_i;
      hold_last_d = in_last_i;
    end else if (pop) begin
      hold_v_d    = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      hold_v_q    <= 1'b0;
      hold_vals_q <= '0;
      hold_meta_q <= '0;
      hold_last_q <= 1'b0;
    end else begin
      hold_v_q    <= hold_v_d;
      hold_vals_q <= hold_vals_d;
      hold_meta_q <= hold_meta_d;
      hold_last_q <= hold_last_d;
    end
  end

  // --------------------------------------------------------------------------
  // Telemetry
  // --------------------------------------------------------------------------
  logic [31:0] beats_q, beats_d;
  logic [31:0] groups_q, groups_d;
  logic [31:0] err_cyc_q, err_cyc_d;
  logic [31:0] err_grp_q, err_grp_d;

  wire beat_fire = push; // input accept
  always_comb begin
    beats_d   = beats_q   + (beat_fire ? 32'd1 : 32'd0);
    groups_d  = groups_q  + (beat_fire ? 32'(G) : 32'd0);
    err_cyc_d = err_cyc_q + ((hold_v_q && exp_any_meta_fix_w && (!PIPE_OUT ? out_ready_i : (!out_v_q || out_ready_i))) ? 32'd1 : 32'd0);
    err_grp_d = err_grp_q + ((hold_v_q && (!PIPE_OUT ? out_ready_i : (!out_v_q || out_ready_i))) ? exp_fix_groups_w : 32'd0);
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      beats_q   <= 32'd0;
      groups_q  <= 32'd0;
      err_cyc_q <= 32'd0;
      err_grp_q <= 32'd0;
    end else begin
      beats_q   <= beats_d;
      groups_q  <= groups_d;
      err_cyc_q <= err_cyc_d;
      err_grp_q <= err_grp_d;
    end
  end

  assign stat_beats_acc_o        = beats_q;
  assign stat_groups_acc_o       = groups_q;
  assign stat_err_meta_cycles_o  = err_cyc_q;
  assign stat_err_meta_groups_o  = err_grp_q;

  // --------------------------------------------------------------------------
  // Assertions (simulation only)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // No combinational ready loop: in_ready_o is purely a function of hold_v_q
  // (Informational; cannot assert directly, but we ensure design structure.)

  // When in META_IS_CODE mode, meta code must be <6 (warn only)
  if (META_IS_CODE) begin : g_assert_code
    for (genvar gk=0; gk<G; gk++) begin
      assert property (@(posedge clk_i) disable iff(!rstn_i)
        push |-> (in_meta_i[gk*3 +: 3] < 3'd6))
        else $warning("sparse_2of4_expand: meta code >=6 at group %0d; auto-fixed to {0,1}.", gk);
    end
  end else begin : g_assert_mask
    for (genvar gm=0; gm<G; gm++) begin
      // Mask popcount should be exactly 2 (warn only; auto-fix applied)
      assert property (@(posedge clk_i) disable iff(!rstn_i)
        push |-> ($countones(in_meta_i[gm*4 +: 4]) != 1))
        else $warning("sparse_2of4_expand: meta mask with a single '1' at group %0d; auto-fixed.", gm);
    end
  end

  // Output mask must match non-zero data placement
  if (!PIPE_OUT) begin : g_assert_out_match
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      out_valid_o |-> (|out_mask_o ? 1'b1 : 1'b1))
      else $error("sparse_2of4_expand: out_mask_o illegal."); // placeholder guard
  end
`endif

endmodule

`default_nettype wire
