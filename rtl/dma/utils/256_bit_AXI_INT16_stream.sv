dtype_unpack #(
  .BUS_W       (256),
  .PACK_ELEM_W (8),    // bytes on the bus
  .PACK_SIGNED (0),
  .OUT_ELEM_W  (16),   // convert up to int16
  .OUT_SIGNED  (1)
) u_unpack_a (
  .clk_i(clk_axi), .rstn_i(rstn_axi),
  .in_valid_i  (axi_v),
  .in_ready_o  (axi_r),
  .in_data_i   (axi_rdata),
  .in_keep_i   (axi_rstrb), // 32-bit byte enables
  .in_last_i   (axi_rlast),
  .out_valid_o (a_v),
  .out_ready_i (a_r),
  .out_data_o  (a_data16),
  .out_last_o  (a_last),
  .cfg_little_endian_i(1'b1),
  .cmd_clr_stats_i(csr.clr),
  .stat_in_beats_o(), .stat_out_beats_o(), .stat_elem_saturations_o()
);
