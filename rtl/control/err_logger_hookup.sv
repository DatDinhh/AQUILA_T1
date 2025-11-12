// Example: 4 sources (DMA, AXI0, AXI1, ECC)
localparam int N_SRC = 4;
logic [N_SRC-1:0] v;
logic [N_SRC*3-1:0] sev;
logic [N_SRC*12-1:0] code;
logic [N_SRC*16-1:0] info;

assign v[0] = dma_err_pulse;
sev [0*3 +: 3]  = 3'd4;                 // severity
code[0*12 +:12] = dma_err_code;
info[0*16 +:16] = dma_failing_desc_id;

// ... fill other sources ...

err_logger #(.N_SRC(N_SRC), .SEV_W(3), .CODE_W(12), .INFO_W(16), .TS_W(32), .FIFO_P2(9))
u_errlog (
  .clk_i(clk_sys), .rstn_i(rstn_sys),
  .src_valid_i(v),
  .src_ready_o(),  // can be ignored if producers don't backpressure
  .src_sev_i(sev), .src_code_i(code), .src_info_i(info),
  .csr_clk_i(csr_clk), .csr_rstn_i(csr_rstn),
  .csr_req_valid_i(csr.valid), .csr_req_ready_o(csr.ready),
  .csr_req_write_i(csr.we), .csr_req_addr_i(csr.addr),
  .csr_req_wdata_i(csr.wdata), .csr_req_wstrb_i(csr.wstrb),
  .csr_resp_valid_o(csr.rvalid), .csr_resp_ready_i(csr.rready),
  .csr_resp_rdata_o(csr.rdata), .csr_resp_err_o(csr.rerr),
  .irq_o(intc_irq_error)
);
