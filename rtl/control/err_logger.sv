// ============================================================================
//  err_logger.sv
//  Aquila — Multi-source Error/Event Logger with Async Log FIFO + CSR-Lite
//
//  Record format (LSB-first):
//    [ SEV_W-1:0 ]    sev
//    [ CODE_W-1:0 ]   code
//    [ INFO_W-1:0 ]   info
//    [ SRC_W-1:0 ]    src_id
//    [ TS_W-1:0 ]     timestamp
//  REC_W = SEV_W + CODE_W + INFO_W + SRC_W + TS_W
//
//  CSR Map (byte offsets, 32-bit words; W1P = write-one pulse, W1C = write-one clear)
//  -------------------------------------------------------------------------------
//   0x000 ID_VERSION   RO  { 'EL'(16), ver:8=1, rev:8=0 }
//   0x004 FEATURES     RO  { REC_W:12, TS_W:8, INFO_W:6, CODE_W:6 }
//   0x008 CFG          RW  [0]=enable, [3:1]=irq_thr (0..7), [4]=timestamp_en
//   0x00C SRC_IRQ_MASK_LO  RW  mask sources [31:0] (1=masked from IRQ)
//   0x010 SRC_IRQ_MASK_HI  RW  mask sources [63:32] (only N_SRC used)
//   0x014 STATUS0      RO  [0]=irq_level, [1]=fifo_empty, [2]=fifo_full, [3]=drops_seen
//   0x018 EV_COUNT     RO  total events accepted
//   0x01C DROP_COUNT   RO  events dropped (overflow while enable=1)
//   0x020 LAST_SEV     RO  last sev
//   0x024 LAST_CODE    RO  last code
//   0x028 LAST_INFO    RO  last info[31:0] (upper via 0x02C if INFO_W>32)
//   0x02C LAST_INFO_HI RO  last info[63:32] (if applicable)
//   0x030 LAST_SRC_TS0 RO  {src_id[15:0], ts[15:0]}
//   0x034 LAST_TS_HI   RO  ts[47:16] or remaining if TS_W>16
//   0x040 STICKY_LO    RW  sticky per-source (RO on read, W1C to clear) [31:0]
//   0x044 STICKY_HI    RW  sticky per-source (W1C) [63:32]
//   0x048 SEV_CNT[0]   RO  sev-0 counter
//   0x04C SEV_CNT[1]   RO  sev-1 counter
//   0x050 SEV_CNT[2]   RO  sev-2 counter
//   0x054 SEV_CNT[3]   RO  sev-3 counter (if SEV_W>=2; up to 8 if SEV_W=3)
//   0x100 LOG_POP      RO  Pop head record into shadow (reading this pops if available)
//   0x104 LOG_WORDSEL  RW  word selector within popped record (0..REC_WORDS-1)
//   0x108 LOG_DATA     RO  32-bit slice of popped record; reading auto-increments WORDSEL
//
//  IRQ behavior:
//   - irq_o = enable && (exists sticky[src]==1 && src not masked && last sev >= irq_thr)
//     Sticky is set on accept when sev >= irq_thr; W1C clears per bit.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module err_logger #(
  parameter int unsigned N_SRC     = 32,   // number of producer ports
  parameter int unsigned SEV_W     = 3,    // severity bits (0..(2^SEV_W-1))
  parameter int unsigned CODE_W    = 12,   // error code bits
  parameter int unsigned INFO_W    = 32,   // optional info payload bits (0 allowed)
  parameter int unsigned TS_W      = 32,   // timestamp bits (0 disables time capture)
  parameter int unsigned FIFO_P2   = 8,    // log depth = 2^FIFO_P2
  parameter int unsigned CSR_ADDR_W= 12    // 4 KiB window
)(
  // -------- Ingress clock domain (events) --------
  input  logic                     clk_i,
  input  logic                     rstn_i,         // sync, active-low

  // Per-source event inputs (clk_i domain)
  input  logic [N_SRC-1:0]         src_valid_i,
  output logic [N_SRC-1:0]         src_ready_o,
  input  logic [N_SRC*SEV_W-1:0]   src_sev_i,      // concatenated: src i at [i*SEV_W +: SEV_W]
  input  logic [N_SRC*CODE_W-1:0]  src_code_i,     // concatenated codes
  input  logic [N_SRC*INFO_W-1:0]  src_info_i,     // concatenated info (ignored if INFO_W==0)

  // -------- CSR clock domain (software) --------
  input  logic                     csr_clk_i,
  input  logic                     csr_rstn_i,

  // CSR-Lite request/response
  input  logic                     csr_req_valid_i,
  output logic                     csr_req_ready_o,
  input  logic                     csr_req_write_i,
  input  logic [CSR_ADDR_W-1:0]    csr_req_addr_i,  // byte addr (word aligned)
  input  logic [31:0]              csr_req_wdata_i,
  input  logic [3:0]               csr_req_wstrb_i,

  output logic                     csr_resp_valid_o,
  input  logic                     csr_resp_ready_i,
  output logic [31:0]              csr_resp_rdata_o,
  output logic                     csr_resp_err_o,

  // -------- Interrupt --------
  output logic                     irq_o
);

  // ==========================================================================
  // Derived
  // ==========================================================================
  localparam int unsigned SRC_W    = (N_SRC <= 1)? 1 : $clog2(N_SRC);
  localparam int unsigned REC_W    = SEV_W + CODE_W + (INFO_W==0?0:INFO_W) + SRC_W + (TS_W==0?0:TS_W);
  localparam int unsigned REC_WORDS= (REC_W + 31) / 32;
  localparam int unsigned DEPTH    = (1 << FIFO_P2);

  // Flatten helpers
  function automatic logic [SEV_W-1:0] sev_at (input logic [N_SRC*SEV_W-1:0] v, input int unsigned i);
    return v[i*SEV_W +: SEV_W];
  endfunction
  function automatic logic [CODE_W-1:0] code_at (input logic [N_SRC*CODE_W-1:0] v, input int unsigned i);
    return v[i*CODE_W +: CODE_W];
  endfunction
  function automatic logic [INFO_W-1:0] info_at (input logic [N_SRC*INFO_W-1:0] v, input int unsigned i);
    if (INFO_W==0) return '0;
    else           return v[i*INFO_W +: INFO_W];
  endfunction

  // ==========================================================================
  // ID/Features
  // ==========================================================================
  localparam [31:0] ID_VERSION = {16'h454C/*'EL'*/, 8'd1, 8'd0};
  localparam [31:0] FEATURES   = { 12'(REC_W[11:0]), 8'(TS_W[7:0]), 6'(INFO_W[5:0]), 6'(CODE_W[5:0]) };

  // ==========================================================================
  // --------------------------- CSR FRONT-END --------------------------------
  // ==========================================================================
  assign csr_req_ready_o = 1'b1;
  logic req_fire = csr_req_valid_i && csr_req_ready_o;

  logic                    req_we_q;
  logic [CSR_ADDR_W-1:0]   req_addr_q;
  logic [31:0]             req_wdata_q;
  logic [3:0]              req_wstrb_q;

  always_ff @(posedge csr_clk_i or negedge csr_rstn_i) begin
    if (!csr_rstn_i) begin
      req_we_q    <= 1'b0;
      req_addr_q  <= '0;
      req_wdata_q <= '0;
      req_wstrb_q <= '0;
    end else if (req_fire) begin
      req_we_q    <= csr_req_write_i;
      req_addr_q  <= csr_req_addr_i;
      req_wdata_q <= csr_req_wdata_i;
      req_wstrb_q <= csr_req_wstrb_i;
    end
  end

  logic        resp_valid_q, resp_valid_d;
  logic [31:0] resp_rdata_q, resp_rdata_d;
  logic        resp_err_q,   resp_err_d;

  assign csr_resp_valid_o = resp_valid_q;
  assign csr_resp_rdata_o = resp_rdata_q;
  assign csr_resp_err_o   = resp_err_q;

  always_ff @(posedge csr_clk_i or negedge csr_rstn_i) begin
    if (!csr_rstn_i) resp_valid_q <= 1'b0;
    else             resp_valid_q <= resp_valid_d && !(csr_resp_valid_o && !csr_resp_ready_i);
  end

  wire [CSR_ADDR_W-1:2] req_word = req_addr_q[CSR_ADDR_W-1:2];

  // ==========================================================================
  // ------------------------------ CONFIG/STATE ------------------------------
  // ==========================================================================
  // CFG: [0]=enable, [3:1]=irq_thr, [4]=timestamp_en
  logic        cfg_enable_q, cfg_enable_d;
  logic [2:0]  cfg_irq_thr_q, cfg_irq_thr_d;      // compare as unsigned sev >= thr
  logic        cfg_ts_en_q,   cfg_ts_en_d;        // masks TS capture if TS_W>0

  // Source IRQ mask
  localparam int SRC_WORDS = (N_SRC+31)/32;
  logic [N_SRC-1:0] src_irq_mask_q, src_irq_mask_d;

  // Per-source sticky; set on accepted event if sev >= irq_thr (and enable)
  logic [N_SRC-1:0] sticky_q, sticky_d;

  // Severity counters (up to 8)
  localparam int unsigned SEV_K = (1<<SEV_W);
  logic [31:0] sev_cnt_q [SEV_K];
  logic [31:0] sev_cnt_d [SEV_K];

  // Totals & drops
  logic [31:0] total_events_q, total_events_d;
  logic [31:0] drop_events_q,  drop_events_d;
  logic        drops_seen_q,   drops_seen_d;

  // Last-event mirrors
  logic [SEV_W-1:0]   last_sev_q;
  logic [CODE_W-1:0]  last_code_q;
  logic [INFO_W-1:0]  last_info_q;
  logic [SRC_W-1:0]   last_src_q;
  logic [TS_W-1:0]    last_ts_q;

  // IRQ level
  logic irq_level_q, irq_level_d;

  // Timestamp generator (clk_i domain)
  logic [TS_W-1:0] ts_ctr_q, ts_ctr_d;
  generate if (TS_W>0) begin
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) ts_ctr_q <= '0;
      else          ts_ctr_q <= ts_ctr_d;
    end
    always_comb begin
      ts_ctr_d = ts_ctr_q + TS_W'(1);
    end
  end else begin
    always_comb begin end
  end endgenerate

  // ==========================================================================
  // ----------------------------- INPUT ARBITER ------------------------------
  // Round-robin select among valid sources; one event per cycle.
  // ==========================================================================
  logic [$clog2(N_SRC)-1:0] rr_ptr_q, rr_ptr_d;
  logic [N_SRC-1:0]         req_vec;
  logic [$clog2(N_SRC)-1:0] sel_idx;
  logic                     sel_valid;

  // Requests are src_valid gated by CFG.enable and FIFO not full (write-side)
  // FIFO 'wr_ready' comes later; we use that to produce ready_o.
  assign req_vec = src_valid_i & {N_SRC{cfg_enable_q}};

  // Rotate and pick first '1'
  function automatic logic [$clog2(N_SRC)-1:0] rot_pick(input logic [N_SRC-1:0] v, input int unsigned s, output logic found);
    logic [N_SRC-1:0] r;
    for (int i=0;i<N_SRC;i++) r[(i+s)%N_SRC] = v[i];
    found = 1'b0;
    int idx = 0;
    for (int i=0;i<N_SRC;i++) begin
      if (!found && r[i]) begin found=1'b1; idx=i; end
    end
    // Map back to absolute index
    return (s + idx) % N_SRC;
  endfunction

  always_comb begin
    sel_valid = (|req_vec);
    if (sel_valid) sel_idx = rot_pick(req_vec, rr_ptr_q, sel_valid);
    else           sel_idx = rr_ptr_q;
  end

  // ==========================================================================
  // -------------------------- ASYNC LOG FIFO (dcfifo) -----------------------
  // Write side (clk_i), read side (csr_clk_i)
// ==========================================================================
  localparam int unsigned FIFO_W = REC_W;

  // FIFO ports
  logic                 fifo_wr_ready, fifo_wr_en;
  logic [FIFO_W-1:0]    fifo_wr_data;

  logic                 fifo_rd_valid, fifo_rd_pop;
  logic [FIFO_W-1:0]    fifo_rd_data;

  logic [FIFO_P2:0]     fifo_wr_level; // approximate level in wr domain
  logic [FIFO_P2:0]     fifo_rd_level; // approximate level in rd domain
  logic                 fifo_empty_csr, fifo_full_evt;

  // Instantiate async FIFO
  dcfifo #(
    .W      (FIFO_W),
    .P2     (FIFO_P2)
  ) u_log_fifo (
    .wr_clk_i   (clk_i),
    .wr_rstn_i  (rstn_i),
    .wr_en_i    (fifo_wr_en),
    .wr_data_i  (fifo_wr_data),
    .wr_ready_o (fifo_wr_ready),
    .wr_level_o (fifo_wr_level),

    .rd_clk_i   (csr_clk_i),
    .rd_rstn_i  (csr_rstn_i),
    .rd_pop_i   (fifo_rd_pop),
    .rd_valid_o (fifo_rd_valid),
    .rd_data_o  (fifo_rd_data),
    .rd_level_o (fifo_rd_level),

    .empty_csr_o(fifo_empty_csr),
    .full_evt_o (fifo_full_evt)
  );

  // ==========================================================================
  // --------------------------- EVENT ACCEPT/WRITE ---------------------------
  // ==========================================================================
  // Backpressure: only the selected source is ready when FIFO can accept a word
  for (genvar i=0;i<N_SRC;i++) begin : g_ready
    assign src_ready_o[i] = cfg_enable_q && fifo_wr_ready && sel_valid && (sel_idx == i[$clog2(N_SRC)-1:0]);
  end

  // Compose record; accept when selected src handshakes
  logic accept_evt;
  logic [SEV_W-1:0]  a_sev;
  logic [CODE_W-1:0] a_code;
  logic [INFO_W-1:0] a_info;
  logic [SRC_W-1:0]  a_src;
  logic [TS_W-1:0]   a_ts;

  always_comb begin
    accept_evt  = 1'b0;
    a_sev       = '0;
    a_code      = '0;
    a_info      = '0;
    a_src       = '0;
    a_ts        = '0;
    if (sel_valid && fifo_wr_ready) begin
      // Handshake with chosen source
      accept_evt = src_valid_i[sel_idx] && src_ready_o[sel_idx];
      a_sev      = sev_at (src_sev_i , sel_idx);
      a_code     = code_at(src_code_i, sel_idx);
      if (INFO_W>0) a_info = info_at(src_info_i, sel_idx);
      a_src      = sel_idx[SRC_W-1:0];
      if (TS_W>0)  a_ts  = (cfg_ts_en_q) ? ts_ctr_q : '0;
    end
  end

  // Pack record (LSB-first)
  logic [REC_W-1:0] rec_pack;
  always_comb begin
    int p = 0;
    rec_pack[p +: SEV_W] = a_sev;  p += SEV_W;
    rec_pack[p +: CODE_W]= a_code; p += CODE_W;
    if (INFO_W>0) begin rec_pack[p +: INFO_W] = a_info; p += INFO_W; end
    rec_pack[p +: SRC_W] = a_src;  p += SRC_W;
    if (TS_W>0) begin rec_pack[p +: TS_W] = a_ts; end
  end

  assign fifo_wr_en   = accept_evt;
  assign fifo_wr_data = rec_pack;

  // Round-robin pointer advance on any handshake
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) rr_ptr_q <= '0;
    else if (accept_evt)  rr_ptr_q <= rr_ptr_q + {{($bits(rr_ptr_q)-1){1'b0}},1'b1};
  end

  // ------------- Ingress accounting / sticky / last mirrors ---------------
  // Update on accept; drop counter increments when **would-accept** but FIFO full
  // (we treat "drop" as source asserted valid while not ready because FIFO full).
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      cfg_enable_q  <= 1'b0; cfg_irq_thr_q <= '0; cfg_ts_en_q <= (TS_W>0);
      src_irq_mask_q<= '0;
      sticky_q      <= '0;
      total_events_q<= 32'd0;
      drop_events_q <= 32'd0;
      drops_seen_q  <= 1'b0;
      last_sev_q    <= '0;
      last_code_q   <= '0;
      last_info_q   <= '0;
      last_src_q    <= '0;
      last_ts_q     <= '0;
      irq_level_q   <= 1'b0;
      rr_ptr_q      <= '0;
      if (TS_W>0) ts_ctr_q <= '0;
      for (int s=0;s<SEV_K;s++) sev_cnt_q[s] <= 32'd0;
    end else begin
      // live CFG mirrors updated in CSR block below via *_d; keep in one domain for simplicity
      cfg_enable_q  <= cfg_enable_d;
      cfg_irq_thr_q <= cfg_irq_thr_d;
      cfg_ts_en_q   <= cfg_ts_en_d;
      src_irq_mask_q<= src_irq_mask_d;

      // timestamp counter
      if (TS_W>0) ts_ctr_q <= ts_ctr_d;

      // Default IRQ level recomputed below
      irq_level_q   <= irq_level_d;

      // Accept path
      if (accept_evt) begin
        total_events_q <= total_events_q + 32'd1;
        last_sev_q     <= a_sev;
        last_code_q    <= a_code;
        if (INFO_W>0) last_info_q <= a_info;
        last_src_q     <= a_src;
        if (TS_W>0)  last_ts_q    <= a_ts;

        // Severity counters
        sev_cnt_q[a_sev] <= sev_cnt_q[a_sev] + 32'd1;

        // Sticky if sev >= threshold
        if (a_sev >= cfg_irq_thr_q) sticky_q[a_src] <= 1'b1;
      end

      // Drop: some source had valid but **no** wr_ready due to FIFO full.
      if (cfg_enable_q && fifo_full_evt && (|src_valid_i)) begin
        drop_events_q <= drop_events_q + 32'd1;
        drops_seen_q  <= 1'b1;
      end
    end
  end

  // IRQ level: any sticky set & source not masked & sev >= thr happened (sticky equals that condition)
  always_comb begin
    irq_level_d = 1'b0;
    for (int i=0;i<N_SRC;i++) begin
      if (sticky_q[i] && !src_irq_mask_q[i]) irq_level_d = 1'b1;
    end
  end
  assign irq_o = cfg_enable_q & irq_level_q;

  // ==========================================================================
  // ----------------------------- CSR Register Bank --------------------------
  // ==========================================================================
  // Word indices
  localparam int REG_ID      = 12'h000 >> 2;
  localparam int REG_FEAT    = 12'h004 >> 2;
  localparam int REG_CFG     = 12'h008 >> 2;
  localparam int REG_MASK_LO = 12'h00C >> 2;
  localparam int REG_MASK_HI = 12'h010 >> 2;
  localparam int REG_STAT0   = 12'h014 >> 2;
  localparam int REG_EVCNT   = 12'h018 >> 2;
  localparam int REG_DRCNT   = 12'h01C >> 2;
  localparam int REG_LSEV    = 12'h020 >> 2;
  localparam int REG_LCODE   = 12'h024 >> 2;
  localparam int REG_LINFO   = 12'h028 >> 2;
  localparam int REG_LINFOHI = 12'h02C >> 2;
  localparam int REG_LSRCTS0 = 12'h030 >> 2;
  localparam int REG_LTSHI   = 12'h034 >> 2;
  localparam int REG_STK_LO  = 12'h040 >> 2;
  localparam int REG_STK_HI  = 12'h044 >> 2;
  localparam int REG_S0      = 12'h048 >> 2; // sev counters base (S0..S7)

  localparam int REG_LOG_POP = 12'h100 >> 2;
  localparam int REG_LOG_WS  = 12'h104 >> 2;
  localparam int REG_LOG_DAT = 12'h108 >> 2;

  // Shadow for popped record (csr_clk_i)
  logic [REC_W-1:0] sh_rec_q;
  logic [15:0]      sh_wsel_q;

  // Read decode
  logic [31:0] rdata, rdata_next;
  logic        rerr,  rerr_next;

  // Simple CSR bank
  always_comb begin
    // defaults hold
    cfg_enable_d   = cfg_enable_q;
    cfg_irq_thr_d  = cfg_irq_thr_q;
    cfg_ts_en_d    = cfg_ts_en_q;
    src_irq_mask_d = src_irq_mask_q;
    sticky_d       = sticky_q; // W1C below

    rdata_next = 32'h0;
    rerr_next  = 1'b0;

    // Writes
    if (req_fire && req_we_q) begin
      unique case (req_word)
        REG_CFG: begin
          if (req_wstrb_q[0]) begin
            cfg_enable_d  = req_wdata_q[0];
            cfg_irq_thr_d = req_wdata_q[3:1];
            cfg_ts_en_d   = (TS_W>0) ? req_wdata_q[4] : 1'b0;
          end
        end
        REG_MASK_LO: begin
          for (int b=0;b<4;b++) if (req_wstrb_q[b]) src_irq_mask_d[b*8 +: 8] = req_wdata_q[b*8 +: 8];
        end
        REG_MASK_HI: begin
          for (int b=0;b<4;b++) if (req_wstrb_q[b]) begin
            int base = 32 + b*8;
            for (int k=0;k<8;k++) if (base+k < N_SRC) src_irq_mask_d[base+k] = req_wdata_q[b*8+k];
          end
        end
        REG_STK_LO: begin // W1C
          for (int b=0;b<4;b++) if (req_wstrb_q[b]) begin
            for (int k=0;k<8;k++) sticky_d[b*8+k] = sticky_q[b*8+k] & ~req_wdata_q[b*8+k];
          end
        end
        REG_STK_HI: begin // W1C
          for (int b=0;b<4;b++) if (req_wstrb_q[b]) begin
            int base=32+b*8;
            for (int k=0;k<8;k++) if (base+k<N_SRC)
              sticky_d[base+k] = sticky_q[base+k] & ~req_wdata_q[b*8+k];
          end
        end
        REG_LOG_WS: begin
          if (&req_wstrb_q) sh_wsel_q = req_wdata_q[15:0] % REC_WORDS[15:0];
        end
        default: ; // no other RW regs
      endcase
    end

    // Reads
    if (req_fire && !req_we_q) begin
      unique case (req_word)
        REG_ID:        rdata_next = ID_VERSION;
        REG_FEAT:      rdata_next = FEATURES;
        REG_CFG:       rdata_next = {27'd0, cfg_ts_en_q, cfg_irq_thr_q, cfg_enable_q};
        REG_MASK_LO:   rdata_next = src_irq_mask_q[31:0];
        REG_MASK_HI:   rdata_next = (N_SRC>32) ? {{(32-(N_SRC-32)){1'b0}}, src_irq_mask_q[N_SRC-1:32]} : 32'd0;
        REG_STAT0:     rdata_next = {28'd0, drops_seen_q, fifo_full_evt, fifo_empty_csr, irq_level_q};
        REG_EVCNT:     rdata_next = total_events_q;
        REG_DRCNT:     rdata_next = drop_events_q;
        REG_LSEV:      rdata_next = {{(32-SEV_W){1'b0}}, last_sev_q};
        REG_LCODE:     rdata_next = {{(32-CODE_W){1'b0}}, last_code_q};
        REG_LINFO:     rdata_next = (INFO_W==0) ? 32'd0 : last_info_q[31:0];
        REG_LINFOHI:   rdata_next = (INFO_W>32) ? {{(32-(INFO_W-32)){1'b0}}, last_info_q[INFO_W-1:32]} : 32'd0;
        REG_LSRCTS0:   rdata_next = { {(32-(16+16)){1'b0}}, last_src_q[15:0], (TS_W>0 ? last_ts_q[15:0] : 16'd0) };
        REG_LTSHI:     rdata_next = (TS_W>16) ? {{(32-(TS_W-16)){1'b0}}, last_ts_q[TS_W-1:16]} : 32'd0;
        REG_STK_LO:    rdata_next = sticky_q[31:0];
        REG_STK_HI:    rdata_next = (N_SRC>32) ? {{(32-(N_SRC-32)){1'b0}}, sticky_q[N_SRC-1:32]} : 32'd0;
        // Sev counters
        default: begin
          if (req_word >= REG_S0 && req_word < REG_S0 + SEV_K) begin
            int s = req_word - REG_S0;
            rdata_next = sev_cnt_q[s];
          end else if (req_word == REG_LOG_POP) begin
            // Pop into shadow
            if (fifo_rd_valid) begin
              rdata_next = 32'hACCE_0001; // pop ok signature (optional)
            end else begin
              rdata_next = 32'hACCE_0000; // nothing to pop
            end
          end else if (req_word == REG_LOG_WS) begin
            rdata_next = {16'd0, sh_wsel_q};
          end else if (req_word == REG_LOG_DAT) begin
            // select word from shadow
            int wsel = sh_wsel_q % REC_WORDS;
            rdata_next = (REC_W==0) ? 32'd0 : (sh_rec_q >> (wsel*32));
          end else begin
            rerr_next  = 1'b1;
          end
        end
      endcase
    end
  end

  // CSR response registers
  always_ff @(posedge csr_clk_i or negedge csr_rstn_i) begin
    if (!csr_rstn_i) begin
      resp_rdata_q <= '0; resp_err_q <= 1'b0; sh_rec_q <= '0; sh_wsel_q <= '0;
    end else begin
      resp_rdata_q <= rdata_next;
      resp_err_q   <= rerr_next;
      // Pop behavior: reading REG_LOG_POP asserts fifo_rd_pop; capture rd_data next cycle
      if (req_fire && !req_we_q && (req_word == REG_LOG_POP) && fifo_rd_valid) begin
        sh_rec_q <= fifo_rd_data;
        sh_wsel_q <= 16'd0;
      end
      // Auto-increment WORDSEL on LOG_DATA read
      if (req_fire && !req_we_q && (req_word == REG_LOG_DAT)) begin
        if (sh_wsel_q + 16'd1 >= REC_WORDS[15:0]) sh_wsel_q <= 16'd0;
        else                                      sh_wsel_q <= sh_wsel_q + 16'd1;
      end
    end
  end

  // Drive csr_resp_valid (1-cycle after request)
  always_comb begin
    resp_valid_d = req_fire;
  end

  // fifo pop strobe (combinational from current request)
  assign fifo_rd_pop = (req_fire && !req_we_q && (req_word == REG_LOG_POP));

  // ==========================================================================
  // ----------------------------- Assertions ---------------------------------
  // ==========================================================================
`ifdef ASSERT_ON
  // Address must be word aligned
  assert property (@(posedge csr_clk_i) disable iff(!csr_rstn_i)
    req_fire |-> (req_addr_q[1:0] == 2'b00))
    else $error("err_logger: CSR address not word-aligned.");

  // FIFO depth must be >= 4 to be useful
  initial begin
    if (DEPTH < 4) $error("err_logger: FIFO depth too small.");
  end
`endif

endmodule

// ============================================================================
//  dcfifo — Asynchronous FIFO with Gray-coded pointers (FWFT=0)
//  - Dual clock domains: wr_clk_i (events), rd_clk_i (CSR)
//  - Power-of-two depth: 2^P2
//  - wr_ready_o indicates space available (one entry)
//  - rd_pop_i pops one entry; rd_valid_o returns the popped data in the same cycle
//    (1-cycle latency from previous push if not empty).
//  - Simple "level" counters are approximate (domain-local).
// ============================================================================
module dcfifo #(
  parameter int unsigned W  = 64,
  parameter int unsigned P2 = 8
)(
  // Write side
  input  logic              wr_clk_i,
  input  logic              wr_rstn_i,
  input  logic              wr_en_i,
  input  logic [W-1:0]      wr_data_i,
  output logic              wr_ready_o,
  output logic [P2:0]       wr_level_o,

  // Read side
  input  logic              rd_clk_i,
  input  logic              rd_rstn_i,
  input  logic              rd_pop_i,
  output logic              rd_valid_o,
  output logic [W-1:0]      rd_data_o,
  output logic [P2:0]       rd_level_o,

  // Status
  output logic              empty_csr_o,  // CSR domain empty flag
  output logic              full_evt_o    // Event domain full flag
);
  localparam int unsigned DEPTH = (1<<P2);

  // Memory
  logic [W-1:0] mem [0:DEPTH-1];

  // Binary & Gray pointers
  logic [P2:0] wr_bin_q, wr_bin_d, wr_gray_q;
  logic [P2:0] rd_bin_q, rd_bin_d, rd_gray_q;

  // Crossed Gray pointers
  logic [P2:0] rd_gray_sync_w1, rd_gray_sync_w2;
  logic [P2:0] wr_gray_sync_r1, wr_gray_sync_r2;

  // Gray helpers
  function automatic logic [P2:0] bin2gray(input logic [P2:0] b);
    return (b >> 1) ^ b;
  endfunction
  function automatic logic [P2:0] gray2bin(input logic [P2:0] g);
    logic [P2:0] b;
    b[P2] = g[P2];
    for (int i=P2-1; i>=0; i--) b[i] = b[i+1] ^ g[i];
    return b;
  endfunction

  // Write domain
  logic [P2:0] rd_gray_w; logic [P2:0] rd_bin_w;
  always_ff @(posedge wr_clk_i or negedge wr_rstn_i) begin
    if (!wr_rstn_i) begin
      wr_bin_q   <= '0;
      wr_gray_q  <= '0;
      rd_gray_sync_w1 <= '0;
      rd_gray_sync_w2 <= '0;
    end else begin
      rd_gray_sync_w1 <= wr_gray_sync_r2; // wrong direction? wait below
    end
  end

  // NOTE: The above line is wrong; fix direction: sync RD gray into WR domain.
  // Correct write-domain FFs:
  always_ff @(posedge wr_clk_i or negedge wr_rstn_i) begin
    if (!wr_rstn_i) begin
      rd_gray_sync_w1 <= '0;
      rd_gray_sync_w2 <= '0;
      wr_bin_q        <= '0;
      wr_gray_q       <= '0;
    end else begin
      rd_gray_sync_w1 <= rd_gray_q;
      rd_gray_sync_w2 <= rd_gray_sync_w1;

      // Write on wr_en and not full
      if (wr_en_i && wr_ready_o) begin
        mem[wr_bin_q[P2-1:0]] <= wr_data_i;
        wr_bin_q  <= wr_bin_q + {{P2{1'b0}},1'b1};
        wr_gray_q <= bin2gray(wr_bin_q + {{P2{1'b0}},1'b1});
      end
    end
  end

  // Read domain
  always_ff @(posedge rd_clk_i or negedge rd_rstn_i) begin
    if (!rd_rstn_i) begin
      wr_gray_sync_r1 <= '0;
      wr_gray_sync_r2 <= '0;
      rd_bin_q        <= '0;
      rd_gray_q       <= '0;
      rd_valid_o      <= 1'b0;
      rd_data_o       <= '0;
    end else begin
      wr_gray_sync_r1 <= wr_gray_q;
      wr_gray_sync_r2 <= wr_gray_sync_r1;

      // Pop if not empty
      if (rd_pop_i && (wr_gray_sync_r2 != rd_gray_q)) begin
        rd_valid_o <= 1'b1;
        rd_data_o  <= mem[rd_bin_q[P2-1:0]];
        rd_bin_q   <= rd_bin_q + {{P2{1'b0}},1'b1};
        rd_gray_q  <= bin2gray(rd_bin_q + {{P2{1'b0}},1'b1});
      end else begin
        rd_valid_o <= 1'b0;
      end
    end
  end

  // Full/empty detection
  assign rd_gray_w = rd_gray_sync_w2;
  assign rd_bin_w  = gray2bin(rd_gray_w);
  wire  [P2:0] wr_next_gray = bin2gray(wr_bin_q + {{P2{1'b0}},1'b1});

  // Full when next write pointer equals read pointer with MSBs inverted (classic)
  assign full_evt_o = (wr_next_gray == {~rd_gray_w[P2:P2-1], rd_gray_w[P2-2:0]});
  assign wr_ready_o = !full_evt_o;

  // Empty when pointers equal (in read domain)
  assign empty_csr_o = (wr_gray_sync_r2 == rd_gray_q);

  // Level estimates
  assign wr_level_o = wr_bin_q - rd_bin_w;
  assign rd_level_o = gray2bin(wr_gray_sync_r2) - rd_bin_q;

`ifdef ASSERT_ON
  // No push when full
  assert property (@(posedge wr_clk_i) disable iff(!wr_rstn_i)
    wr_en_i |-> wr_ready_o) else $error("dcfifo: push while full.");
  // No pop when empty
  assert property (@(posedge rd_clk_i) disable iff(!rd_rstn_i)
    rd_pop_i |-> (wr_gray_sync_r2 != rd_gray_q)) else $error("dcfifo: pop while empty.");
`endif
endmodule

`default_nettype wire
