bus_inject_w #(
  .NBANKS(NBANK),
  .DATA_W(512),
  .ADDR_W(32),
  .BANK_SEL_LSB($clog2(512/8)),
  .ELEM_W(8)
) u_injW (
  .clk_i(clk_ctrl), .rstn_i(rstn_ctrl),

  // Crossbar master slot M3 (example)
  .wr_req_valid_o (m3_wr_req_valid),
  .wr_req_ready_i (m3_wr_req_ready),
  .wr_addr_o      (m3_wr_addr),
  .wr_data_o      (m3_wr_data),
  .wr_strb_o      (m3_wr_strb),

  .rd_req_valid_o (m3_rd_req_valid),
  .rd_req_ready_i (m3_rd_req_ready),
  .rd_addr_o      (m3_rd_addr),

  .rd_rsp_valid_i (m3_rd_rsp_valid),
  .rd_rsp_ready_o (m3_rd_rsp_ready),
  .rd_rsp_data_i  (m3_rd_rsp_data),
  .rd_rsp_sbe_i   (m3_rd_rsp_sbe),
  .rd_rsp_dbe_i   (m3_rd_rsp_dbe),

  // Config
  .cfg_en_i(1'b1),
  .cfg_mode_i(3'd1),                 // FILL_ONLY
  .cfg_bank_mask_i(8'hFF),
  .bank_enable_i(part.bank_enable),  // from gb_partition_ctrl
  .cfg_words_per_bank_i(32'd65536),
  .cfg_start_word_i(32'd0),
  .cfg_stride_words_i(32'd1),
  .cfg_interleave_en_i(1'b1),
  .cfg_interleave_words_i(16'd4),    // switch bank every 4 beats
  .cfg_gap_cyc_i(16'd0),
  .cfg_seed_i(64'hC0FE_F00D_5EED_0002),
  .cfg_signed_i(1'b1),
  .cfg_sparsity_2of4_en_i(1'b1),

  .cmd_start_i(csr.injw_start_pulse),
  .cmd_pause_i(csr.injw_pause),
  .cmd_resume_i(csr.injw_resume),
  .cmd_abort_i(csr.injw_abort),
  .cmd_clr_counters_i(csr.injw_clr),

  // Status
  .active_o(stat.injw_active),
  .paused_o(stat.injw_paused),
  .op_rd_issued_o(stat.injw_rd_issued),
  .op_wr_issued_o(stat.injw_wr_issued),
  .verify_ok_o(stat.injw_ok),
  .verify_err_o(stat.injw_err),
  .sbe_seen_o(stat.injw_sbe),
  .dbe_seen_o(stat.injw_dbe),
  .cur_bank_o(stat.injw_cur_bank),
  .cur_word_o(stat.injw_cur_word),
  .last_addr_o(stat.injw_last_addr),
  .last_exp_o(stat.injw_last_exp),
  .last_got_o(stat.injw_last_got),
  .last_diff_o(stat.injw_last_diff)
);
