localparam int DESC_W  = 256;
localparam int DEPTH   = 32;
localparam int NSINK   = 3; // 0=A, 1=W, 2=C

desc_queue #(
  .NSINK        (NSINK),
  .DESC_W       (DESC_W),
  .DEPTH        (DEPTH),
  .MAX_INFLIGHT (2)
) u_descq (
  .clk_i        (clk_ctrl),
  .rstn_i       (rstn_ctrl),

  // Firmware pushes descriptors (via a tiny APB shim or stream)
  .enq_valid_i  (fw_enq_valid),
  .enq_ready_o  (fw_enq_ready),
  .enq_desc_i   (fw_enq_desc),
  .enq_class_i  (fw_enq_class), // 0=A,1=W,2=C

  // A-DMA
  .issue_valid_o[0] (a_issue_valid),
  .issue_ready_i[0] (a_issue_ready),
  .issue_tag_o  [0] (a_issue_tag),
  .issue_desc_o [0] (a_issue_desc),
  .done_valid_i [0] (a_done_valid),
  .done_tag_i   [0] (a_done_tag),

  // W-DMA
  .issue_valid_o[1] (w_issue_valid),
  .issue_ready_i[1] (w_issue_ready),
  .issue_tag_o  [1] (w_issue_tag),
  .issue_desc_o [1] (w_issue_desc),
  .done_valid_i [1] (w_done_valid),
  .done_tag_i   [1] (w_done_tag),

  // C-DMA
  .issue_valid_o[2] (c_issue_valid),
  .issue_ready_i[2] (c_issue_ready),
  .issue_tag_o  [2] (c_issue_tag),
  .issue_desc_o [2] (c_issue_desc),
  .done_valid_i [2] (c_done_valid),
  .done_tag_i   [2] (c_done_tag),

  .full_o(), .empty_o(),
  .free_count_o(), .pending_count_o(), .inflight_count_o(),
  .err_spurious_done_o(), .err_double_done_o(), .err_overflow_o()
);
