pll_wrapper u_pll (
  .clk_ref  (clk_ref),
  .rstn     (rstn_ref),
  .clk_arr  (clk_arr),
  .clk_sram (clk_sram),
  .clk_ctrl (clk_ctrl),
  .locked   (pll_locked)
);
