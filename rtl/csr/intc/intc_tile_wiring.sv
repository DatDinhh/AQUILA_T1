localparam int NSRC_EXT = 10;
localparam int NLINES   = 8;

intc_tile #(
  .NSRC_EXT (NSRC_EXT),
  .NLINES   (NLINES),
  .HAS_TIMER(1),
  .HAS_WDOG (1)
) u_intc (
  .clk_i     (clk_ctrl),
  .rstn_i    (rstn_ctrl),
  .src_ext_i ({irq_ocla, irq_errlog, irq_perf, therm_alert, mbist_fail,
               ecc_unc, ecc_cor, dma_c_done, dma_w_done, dma_a_done}),
  .paddr_i   (csr_paddr), .psel_i(csr_psel), .penable_i(csr_penable),
  .pwrite_i  (csr_pwrite), .pwdata_i(csr_pwdata),
  .prdata_o  (/* tie into CSR read mux or give it a subwindow */),
  .pready_o  (), .pslverr_o(),
  .irq_lines_o(uc_irq_lines)
);
