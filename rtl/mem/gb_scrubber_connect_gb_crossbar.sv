// Scrubber master → Crossbar (master slot N)
gb_scrubber #(
  .NBANK(NBANK),
  .DATA_W(512),
  .ADDR_W(32),
  .BANK_SEL_LSB($clog2(512/8))
) u_scrub (
  .clk_i(clk_ctrl), .rstn_i(rstn_ctrl),

  // To crossbar:
  .wr_req_valid_o (mN_wr_req_valid),
  .wr_req_ready_i (mN_wr_req_ready),
  .wr_addr_o      (mN_wr_addr),
  .wr_data_o      (mN_wr_data),
  .wr_strb_o      (mN_wr_strb),

  .rd_req_valid_o (mN_rd_req_valid),
  .rd_req_ready_i (mN_rd_req_ready),
  .rd_addr_o      (mN_rd_addr),

  .rd_rsp_valid_i (mN_rd_rsp_valid),
  .rd_rsp_ready_o (mN_rd_rsp_ready),
  .rd_rsp_data_i  (mN_rd_rsp_data),
  .rd_rsp_sbe_i   (mN_rd_rsp_sbe),
  .rd_rsp_dbe_i   (mN_rd_rsp_dbe),

  // Control from CSR:
  .cfg_enable_i       (csr.scrub_en),
  .cfg_continuous_i   (csr.scrub_continuous),   // 1=loop forever
  .cfg_bank_mask_i    (csr.scrub_bank_mask),    // which banks to scrub
  .cfg_bank_words_i   (csr.scrub_bank_words),   // words per bank
  .cfg_global_base_i  (csr.scrub_base),         // 0 unless GB mapped up
  .cfg_rate_div_i     (csr.scrub_rate_div),
  .cfg_writeback_en_i (csr.scrub_wb_en),
  .cmd_start_i        (csr.scrub_start_pulse),
  .cmd_pause_i        (csr.scrub_pause),
  .cmd_abort_i        (csr.scrub_abort),

  // Status to CSR:
  .active_o           (stat.scrub_active),
  .done_o             (stat.scrub_done),
  .bank_wrap_pulse_o  (stat.scrub_wrap_pulse),
  .rd_issued_o        (stat.scrub_rd_issued),
  .rd_completed_o     (stat.scrub_rd_completed),
  .wr_issued_o        (stat.scrub_wr_issued),
  .sbe_count_o        (stat.scrub_sbe_total),
  .dbe_count_o        (stat.scrub_dbe_total),
  .sbe_count_bank_o   (stat.scrub_sbe_bank),
  .dbe_count_bank_o   (stat.scrub_dbe_bank),
  .ptr_word_o         (stat.scrub_ptr_word)
);
