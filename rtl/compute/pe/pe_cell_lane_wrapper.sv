// Lane wrapper drives one vector step per cycle
pe_lane_int #(
  .VEC(16), .ACT_W(8), .WGT_W(8)
) u_lane (/* ... */);

// One PE cell (repeat per-PE)
pe_cell #(
  .VEC(16), .ACT_W(8), .WGT_W(8), .ACC_W(32), .OUT_W(16),
  .ACT_SIGNED(1), .WGT_SIGNED(1), .OUT_SIGNED(1), .SATURATE(1)
) u_pe0 (
  .clk_i(clk_core), .rstn_i(rstn_core),

  .step_valid_i    (lane.step_valid_o),
  .step_ready_o    (pe_ready),            // back to lane
  .a_vec_i         (lane.a_vec_o[0]),     // slice for this PE if lane fanouts differ
  .w_vec_i         (lane.w_vec_o[0]),
  .mul_en_mask_i   (lane.mul_en_mask_o[0]),
  .step_last_i     (lane.step_last_o),

  .tile_start_i    (tile_start),
  .acc_clear_i     (tile_start),          // or a dedicated clear pulse
  .stall_i         (1'b0),

  .cfg_bias_en_i   (1'b1),
  .cfg_bias_i      (32'sd0),
  .cfg_shift_i     (6'd8),
  .cfg_shift_rnd_i (1'b1),
  .cfg_relu_en_i   (1'b0),
  .cfg_clip_min_i  ($signed(16'sh8000)),  // full-range by default
  .cfg_clip_max_i  ($signed(16'sh7FFF)),

  .c_valid_o       (c_valid),
  .c_ready_i       (c_ready),
  .c_data_o        (c_data),
  .c_last_o        (c_last),

  .stat_steps_o    (),
  .stat_zero_steps_o(),
  .stat_saturations_o(),
  .dbg_acc_q_o     ()
);
