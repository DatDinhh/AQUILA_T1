dtype_pack_unpack #(
  .DIR(1),               // UNPACK
  .BUS_W(256),
  .ELEM_B_W(8),          // bus elements are int8
  .ELEM_E_W(16),         // element side becomes uint16
  .ELEM_B_SIGNED(1'b1),
  .ELEM_E_SIGNED(1'b0),
  .BUF_ELEMS(128)
) u_unpack (
  .clk_i(clk), .rstn_i(rstn),

  // Element IN (unused)
  .elem_in_valid_i(1'b0), .elem_in_ready_o(),
  .elem_in_data_i ('0),   .elem_in_last_i(1'b0),

  // Element OUT
  .elem_out_valid_o (e_v),
  .elem_out_ready_i (e_r),
  .elem_out_data_o  (e_u16),
  .elem_out_last_o  (e_last),

  // Bus IN
  .bus_in_valid_i (axi_v),
  .bus_in_ready_o (axi_r),
  .bus_in_data_i  (axi_data256),
  .bus_in_keep_i  (axi_keep32), // keep (32 bytes)
  .bus_in_last_i  (axi_last),

  // Bus OUT (unused)
  .bus_out_valid_o(), .bus_out_ready_i(1'b0),
  .bus_out_data_o(),  .bus_out_keep_o(), .bus_out_last_o(),

  .cfg_big_endian_i(1'b0),

  .cmd_clr_stats_i (csr.clr),
  .stat_beats_in_o (stat.b_in),
  .stat_beats_out_o(),
  .stat_elems_in_o (stat.e_in),
  .stat_elems_out_o(stat.e_out),
  .stat_partial_beats_o(stat.partial)
);
