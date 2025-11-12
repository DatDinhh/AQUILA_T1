// Controller
array_ctrl #(
  .K_W(16), .TILE_W(16)
) u_ctrl (
  .clk_i(clk_core), .rstn_i(rstn_core),

  .cfg_enable_i       (csr.arr_en),
  .cmd_start_i        (csr.arr_start_pulse),
  .cmd_abort_i        (csr.arr_abort_pulse),
  .cmd_pause_i        (csr.arr_pause),
  .cmd_resume_i       (csr.arr_resume),
  .cfg_k_len_i        (csr.arr_k_len),
  .cfg_tiles_total_i  (csr.arr_tiles_total),
  .cfg_gap_cyc_i      (csr.arr_gap_cyc),
  .cfg_overlap_prefetch_i(csr.arr_overlap_prefetch),

  // Feeders (DMA/decompressors)
  .kick_a_prefetch_o  (ctrl2a_kick),
  .kick_w_prefetch_o  (ctrl2w_kick),
  .a_prefetch_done_i  (a2ctrl_done),
  .w_prefetch_done_i  (w2ctrl_done),

  // To array
  .step_valid_o       (net_step_valid),
  .step_first_o       (net_step_first),
  .step_last_o        (net_step_last),
  .clear_acc_o        (net_clear_acc),
  .step_ready_i       (net_step_ready),

  // From array gather
  .c_tile_last_i      (net_c_tile_last),
  .c_tile_valid_i     (net_c_valid), // validate last

  // Telemetry
  .active_o           (stat.arr_active),
  .paused_o           (stat.arr_paused),
  .k_ctr_o            (stat.arr_k_ctr),
  .tile_idx_o         (stat.arr_tile_idx),
  .steps_issued_o     (stat.arr_steps),
  .stall_cycles_o     (stat.arr_stall_cyc),
  .drain_cycles_o     (stat.arr_drain_cyc),
  .tiles_done_o       (stat.arr_tiles_done),
  .err_zero_k_sticky_o(stat.arr_err_zero_k),
  .err_c_last_timeout_o(stat.arr_err_drain_to)
);

// Array net (previous file you have)
array_rowcol_net #(
  .R_CLUST(R), .C_CLUST(C),
  .ACT_W(8), .WGT_W(8), .ACC_W(32), .OUT_W(16)
) u_net (
  .clk_i(clk_core), .rstn_i(rstn_core),

  .a_west_valid_i (...), .a_west_data_i (...),
  .a_east_valid_o (...), .a_east_data_o (...),

  .w_north_valid_i(...), .w_north_data_i(...),
  .w_south_valid_o(...), .w_south_data_o(...),

  .step_valid_i    (net_step_valid),
  .step_first_i    (net_step_first),
  .step_last_i     (net_step_last),
  .clear_acc_i     (net_clear_acc),
  .step_ready_o    (net_step_ready),

  .mul_en_mask_flat_i(csr.mul_mask_flat),

  .cfg_bias_en_i   (csr.bias_en),
  .cfg_bias_i      (csr.bias),
  .cfg_shift_i     (csr.shift),
  .cfg_shift_rnd_i (csr.rnd),
  .cfg_relu_en_i   (csr.relu),
  .cfg_clip_min_i  (csr.clip_min),
  .cfg_clip_max_i  (csr.clip_max),

  .c_valid_o       (net_c_valid),
  .c_ready_i       (dma_c_ready),
  .c_data_o        (net_c_data),
  .c_tile_last_o   (net_c_last),
  .c_tag_row_o     (),
  .c_tag_col_o     (),
  .c_tag_pe_o      ()
);

// Controller watches net_c_last/valid to exit DRAIN.
assign net_c_tile_last = net_c_last && net_c_valid;
