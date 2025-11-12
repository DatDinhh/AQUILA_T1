post_bias #(
  .DATA_W(16),            // int16 outputs
  .OUT_SIGNED(1),
  .BIAS_W(32),            // int32 bias (acc domain)
  .CHAN_W(12),            // 4096 channels
  .HEADROOM(8)
) u_post_bias (
  .clk_i(clk_core),
  .rstn_i(rstn_core),

  .in_valid_i  (c_v),
  .in_ready_o  (c_r),
  .in_data_i   (c_data),
  .in_chan_i   (c_chan),   // your array/mapper should provide this
  .in_last_i   (c_last),

  .out_valid_o (pb_v),
  .out_ready_i (dma_ready),
  .out_data_o  (pb_data),
  .out_last_o  (pb_last),

  .cfg_enable_i            (csr.pb_en),
  .cfg_bias_broadcast_en_i (csr.pb_bcast_en),
  .cfg_bias_broadcast_i    (csr.pb_bcast),
  .cfg_bias_shift_i        (csr.pb_shift),
  .cfg_bias_shift_rnd_i    (csr.pb_shift_rnd),
  .cfg_relu_en_i           (csr.pb_relu),
  .cmd_clr_counters_i      (csr.pb_clr),

  .csr_we_i    (csr.pb_we),
  .csr_re_i    (csr.pb_re),
  .csr_addr_i  (csr.pb_addr),
  .csr_wdata_i (csr.pb_wdata),
  .csr_rdata_o (csr.pb_rdata),

  .stat_in_beats_o(),
  .stat_out_beats_o(),
  .stat_saturations_o(),
  .stat_relu_zero_o()
);
