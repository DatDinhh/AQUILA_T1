Write path:
  wr_cmd_valid_i, wr_cmd_ready_o, wr_addr_i[ADDR_W-1:0]  // byte address
  wr_data_valid_i, wr_data_ready_o, wr_data_i[DATA_W-1:0], wr_strb_i[DATA_W/8-1:0]

Read path:
  rd_cmd_valid_i, rd_cmd_ready_o, rd_addr_i[ADDR_W-1:0]
  rd_data_valid_o, rd_data_ready_i, rd_data_o[DATA_W-1:0]
  rd_sbe_o, rd_dbe_o         // pulse per returned word
  sbe_count_o, dbe_count_o   // sticky counters

Macro/test:
  mbist_mode_i               // 1 = connect macro pins to MBIST ports
  mbist_*                    // raw macro pins in MBIST mode
  sleep_i                    // optional retention/sleep hint (passed to macro)
