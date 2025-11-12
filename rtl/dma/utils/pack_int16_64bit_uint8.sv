dtype_pack_unpack #(
  .DIR(0),               // PACK
  .BUS_W(64),
  .ELEM_B_W(8),          // bus elements are 8-bit (8 elems/beat)
  .ELEM_E_W(16),         // element side is 16-bit
  .ELEM_B_SIGNED(1'b0),  // uint8 on bus
  .ELEM_E_SIGNED(1'b1),  // int16 on elem side
  .BUF_ELEMS(32)
) u_pack (
  .clk_i(clk), .rstn_i(rstn),

  // Element IN
  .elem_in_valid_i (c_v),
  .elem_in_ready_o (c_r),
  .elem_in_data_i  (c_data16),  // int16
  .elem_in_last_i  (c_last),

  // Element OUT (unused in PACK)
  .elem_out_valid_o(), .elem_out_ready_i(1'b0),
  .elem_out_data_o(),  .elem_out_last_o(),

  // Bus IN (unused in PACK)
  .bus_in_valid_i(1'b0), .bus_in_ready_o(),
  .bus_in_data_i ('0),   .bus_in_keep_i('0), .bus_in_last_i(1'b0),

  // Bus OUT
  .bus_out_valid_o (dma_v),
  .bus_out_ready_i (dma_r),
  .bus_out_data_o  (dma_data64),
  .bus_out_keep_o  (dma_keep8), // AXI tkeep (8 bytes)
  .bus_out_last_o  (dma_last),

  .cfg_big_endian_i(1'b0),

  .cmd_clr_stats_i (csr.clr),
  .stat_beats_in_o (),
  .stat_beats_out_o(stat.beats),
  .stat_elems_in_o (stat.elm_in),
  .stat_elems_out_o(stat.elm_out),
  .stat_partial_beats_o(stat.partial)
);
