bus_inject_a #(
  .NBANKS(NBANK),
  .DATA_W(512),
  .ADDR_W(32),
  .BANK_SEL_LSB($clog2(512/8))
) u_injA (
  .clk_i(clk_ctrl), .rstn_i(rstn_ctrl),

  // To crossbar (master slot e.g., M2):
  .wr_req_valid_o (m2_wr_req_valid),
  .wr_req_ready_i (m2_wr_req_ready),
  .wr_addr_o      (m2_wr_addr),
  .wr_data_o      (m2_wr_data),
  .wr_strb_o      (m2_wr_strb),
  .rd_req_valid_o (m2_rd_req_valid),
  .rd_req_ready_i (m2_rd_req_ready),
  .rd_addr_o      (m2_rd_addr),
  .rd_rsp_valid_i (m2_rd_rsp_valid),
  .rd_rsp_ready_o (m2_rd_rsp_ready),
  .rd_rsp_data_i  (m2_rd_rsp_data),
  .rd_rsp_sbe_i   (m2_rd_rsp_sbe),
  .rd_rsp_dbe_i   (m2_rd_rsp_dbe),

  // Config (drive from CSR or testbench):
  .cfg_en_i(1'b1),
  .cfg_mode_i(3'd1),               // MODE_FILL
  .cfg_bank_mask_i(8'hFF),
  .bank_enable_i(part.bank_enable),// from gb_partition_ctrl
  .cfg_words_per_bank_i(32'd65536),
  .cfg_start_word_i(32'd0),
  .cfg_stride_words_i(32'd1),
  .cfg_gap_cyc_i(16'd0),
  .cfg_seed_i(64'hA5A5_1CE5_5EED_0001),
  .cmd_start_i(csr.inj_start_pulse),
  .cmd_pause_i(csr.inj_pause),
  .cmd_resume_i(csr.inj_resume),
  .cmd_abort_i(csr.inj_abort),
  .cmd_clr_counters_i(csr.inj_clr),

  // Status to CSR:
  .active_o(stat.inj_active),
  .paused_o(stat.inj_paused),
  .op_rd_issued_o(stat.inj_rd_issued),
  .op_wr_issued_o(stat.inj_wr_issued),
  .verify_ok_o(stat.inj_ok),
  .verify_err_o(stat.inj_err),
  .sbe_seen_o(stat.inj_sbe),
  .dbe_seen_o(stat.inj_dbe),
  .cur_bank_o(stat.inj_cur_bank),
  .cur_word_o(stat.inj_cur_word),
  .last_addr_o(stat.inj_last_addr),
  .last_exp_o(stat.inj_last_exp),
  .last_got_o(stat.inj_last_got),
  .last_diff_o(stat.inj_last_diff)
);
