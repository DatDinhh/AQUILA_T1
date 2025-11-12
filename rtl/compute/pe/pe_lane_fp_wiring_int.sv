localparam int LANES = 8;

// Lane interconnect (already provided)
pe_lane_int #(
  .LANE_COUNT(LANES),
  .ACT_W(16),  // e.g., FP16 payloads encoded as IEEE-754 half in 16 bits
  .W_DW(16),
  .K_W(16),
  .A_PIPE_IN(1),
  .W_PIPE_IN(1)
) u_lane_int (/* ... outputs: pe_fire_o, pe_k_first_o, pe_k_last_o, pe_clear_acc_o,
                 pe_act_o[LANES*16-1:0], pe_wgt_o[LANES*16-1:0] ... */);

// Floating-point lane compute
pe_lane_fp #(
  .LANE_COUNT (LANES),
  .IN_EW      (5),     // FP16
  .IN_MW      (10),
  .ACC_EW     (8),     // accumulate in FP32
  .ACC_MW     (23),
  .OUT_EW     (8),
  .OUT_MW     (23),
  .FMA_LAT    (4),     // match your FMA primitive
  .ADD_LAT    (3),
  .DO_CAST    (1'b0)
) u_lane_fp (
  .clk_i            (clk_core),
  .rstn_i           (rstn_core),

  .pe_fire_i        (pe_fire),
  .pe_k_first_i     (pe_k_first),
  .pe_k_last_i      (pe_k_last),
  .pe_clear_acc_i   (pe_clr_acc),
  .pe_act_i_flat    (pe_act_bus),     // 8×16-bit lane-packed FP16
  .pe_wgt_i_flat    (pe_wgt_bus),     // 8×16-bit lane-packed FP16
  .cfg_rnd_mode_i   (3'd0),           // RNE
  .cfg_ftz_i        (1'b1),
  .cfg_daz_i        (1'b1),

  .c_valid_o        (c_valid),
  .c_ready_i        (c_ready),
  .c_data_o         (c_data),         // 8×32-bit lane-packed FP32
  .c_last_o         (c_last),

  .exc_any_cnt_o    (),
  .exc_invalid_cnt_o(),
  .exc_div0_cnt_o   (),
  .exc_overflow_cnt_o(),
  .exc_underflow_cnt_o(),
  .exc_inexact_cnt_o()
);
