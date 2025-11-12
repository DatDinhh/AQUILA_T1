// AXI constants
localparam int BUS_W      = 256;
localparam int BUS_BYTES  = BUS_W/8;
localparam int AXI_ADDR_W = 48;
localparam int AXI_ID_W   = 4;

wb_storeq #(
  .AXI_ADDR_W(AXI_ADDR_W),
  .AXI_ID_W  (AXI_ID_W),
  .AXI_USER_W(1),
  .BUS_W     (BUS_W),
  .LANES_IN  (16),
  .IN_ELEM_W (16),
  .IN_SIGNED (1),
  .STORE_ELEM_W(8),
  .STORE_SIGNED(1)
) u_wb (
  .clk_i(clk_axi),
  .rstn_i(rstn_axi),

  // Descriptor
  .desc_valid_i (desc_v),
  .desc_ready_o (desc_r),
  .desc_addr_i  (desc_addr),
  .desc_bytes_i (desc_bytes),
  .desc_id_i    (desc_id),
  .desc_awuser_i(desc_user),

  // Completion
  .done_valid_o (done_v),
  .done_ready_i (done_r),
  .done_id_o    (done_id),
  .done_bresp_o (done_resp),
  .done_err_o   (done_err),

  // C-stream to store
  .in_valid_i   (c_v),
  .in_ready_o   (c_r),
  .in_data_i    (c_data16),     // 16*16b lanes if LANES_IN=16
  .in_last_i    (c_last),

  .cfg_little_endian_i(1'b1),

  // AXI AW/W/B
  .aw_valid_o(axi_awvalid), .aw_ready_i(axi_awready),
  .aw_addr_o (axi_awaddr),  .aw_len_o  (axi_awlen),
  .aw_size_o (axi_awsize),  .aw_burst_o(axi_awburst),
  .aw_id_o   (axi_awid),    .aw_prot_o (axi_awprot),
  .aw_cache_o(axi_awcache), .aw_qos_o  (axi_awqos),
  .aw_region_o(axi_awregion), .aw_user_o(axi_awuser), .aw_lock_o(axi_awlock),

  .w_valid_o (axi_wvalid),  .w_ready_i (axi_wready),
  .w_data_o  (axi_wdata),   .w_strb_o  (axi_wstrb),
  .w_last_o  (axi_wlast),

  .b_valid_i (axi_bvalid),  .b_ready_o (axi_bready),
  .b_resp_i  (axi_bresp),   .b_id_i    (axi_bid),

  .cmd_clr_stats_i(csr.clr),
  .stat_desc_acc_o(),
  .stat_aw_beats_o(),
  .stat_w_beats_o(),
  .stat_w_bytes_o(),
  .stat_bresp_errs_o()
);
