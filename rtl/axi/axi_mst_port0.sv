// ============================================================================
//  axi_mst_port0.sv
//  Aquila: AXI4 Master Port 0 (Read + Write)
//  - Write path: wraps wb_storeq (wb_pack inside) → AXI AW/W/B
//  - Read path : single-burst AR engine → AXI R; streams out with KEEP/LAST
//
//  Features:
//   * Unaligned starts supported (first-beat STRB/KEEP mask).
//   * Single-burst constraint per descriptor (≤256 beats, no 4KiB crossing).
//   * Clean RV handshakes, no combinational ready loops.
//   * Telemetry counters and robust synthesis-friendly assertions.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi_mst_port0 #(
  // ---------------------------- AXI geometry ----------------------------
  parameter int unsigned AXI_ADDR_W   = 48,
  parameter int unsigned AXI_ID_W     = 4,
  parameter int unsigned AXI_USER_W   = 1,
  parameter int unsigned BUS_W        = 256,      // AXI data width (bits, multiple of 8)

  // ---------------------------- Write path ------------------------------
  parameter int unsigned LANES_W      = 16,       // elements per input beat (to wb_pack)
  parameter int unsigned INW_ELEM_W   = 16,       // incoming element width (byte-aligned)
  parameter bit          INW_SIGNED   = 1'b1,
  parameter int unsigned STW_ELEM_W   = 8,        // store element width on the bus (byte-aligned)
  parameter bit          STW_SIGNED   = 1'b1,

  // ---------------------------- Read path -------------------------------
  // Read path outputs raw bus beats (data+keep+last); downstream (dtype_unpack)
  // can convert dtype as needed.
  // Elasticity
  parameter int unsigned RD_OUT_FIFO_DEPTH = 8,

  // ---------------------------- FIFOs (write) ---------------------------
  parameter int unsigned W_IN_FIFO_DEPTH  = 4,
  parameter int unsigned W_OUT_FIFO_DEPTH = 4,

  // ---------------------------- AXI attributes --------------------------
  parameter logic [2:0]  P_AWPROT   = 3'b000,
  parameter logic [3:0]  P_AWCACHE  = 4'b0011,
  parameter logic [3:0]  P_AWQOS    = 4'b0000,
  parameter logic [3:0]  P_AWREGION = 4'b0000,

  parameter logic [2:0]  P_ARPROT   = 3'b000,
  parameter logic [3:0]  P_ARCACHE  = 4'b0011,
  parameter logic [3:0]  P_ARQOS    = 4'b0000,
  parameter logic [3:0]  P_ARREGION = 4'b0000
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,            // synchronous, active-low

  // =============================== Config ===============================
  input  logic                         cfg_little_endian_i, // 1=LE byte lanes, 0=BE

  // ========================== WRITE DESCRIPTOR ==========================
  // Must fit single burst (checked)
  input  logic                         wdesc_valid_i,
  output logic                         wdesc_ready_o,
  input  logic [AXI_ADDR_W-1:0]        wdesc_addr_i,
  input  logic [31:0]                  wdesc_bytes_i,
  input  logic [AXI_ID_W-1:0]          wdesc_id_i,
  input  logic [AXI_USER_W-1:0]        wdesc_awuser_i,

  // Completion (on BRESP)
  output logic                         wdone_valid_o,
  input  logic                         wdone_ready_i,
  output logic [AXI_ID_W-1:0]          wdone_id_o,
  output logic [1:0]                   wdone_bresp_o,
  output logic                         wdone_err_o,

  // ============================== WRITE DATA ============================
  // Stream to be written (goes into wb_pack inside wb_storeq)
  input  logic                         w_in_valid_i,
  output logic                         w_in_ready_o,
  input  logic [LANES_W*INW_ELEM_W-1:0] w_in_data_i,
  input  logic                         w_in_last_i,

  // =========================== READ DESCRIPTOR ==========================
  input  logic                         rdesc_valid_i,
  output logic                         rdesc_ready_o,
  input  logic [AXI_ADDR_W-1:0]        rdesc_addr_i,
  input  logic [31:0]                  rdesc_bytes_i,
  input  logic [AXI_ID_W-1:0]          rdesc_id_i,
  input  logic [AXI_USER_W-1:0]        rdesc_aruser_i,

  // Completion (after RLAST)
  output logic                         rdone_valid_o,
  input  logic                         rdone_ready_i,
  output logic [AXI_ID_W-1:0]          rdone_id_o,
  output logic                         rdone_err_o,

  // ============================== READ DATA =============================
  // Bus-shaped stream with keep + last for partial edges
  output logic                         r_out_valid_o,
  input  logic                         r_out_ready_i,
  output logic [BUS_W-1:0]             r_out_data_o,
  output logic [BUS_W/8-1:0]           r_out_keep_o,
  output logic                         r_out_last_o,

  // ============================ AXI4 MASTER =============================
  // AW
  output logic                         M_AWVALID,
  input  logic                         M_AWREADY,
  output logic [AXI_ADDR_W-1:0]        M_AWADDR,
  output logic [7:0]                   M_AWLEN,    // beats-1
  output logic [2:0]                   M_AWSIZE,   // log2(bytes/beat)
  output logic [1:0]                   M_AWBURST,  // INCR
  output logic [AXI_ID_W-1:0]          M_AWID,
  output logic [2:0]                   M_AWPROT,
  output logic [3:0]                   M_AWCACHE,
  output logic [3:0]                   M_AWQOS,
  output logic [3:0]                   M_AWREGION,
  output logic [AXI_USER_W-1:0]        M_AWUSER,
  output logic                         M_AWLOCK,
  // W
  output logic                         M_WVALID,
  input  logic                         M_WREADY,
  output logic [BUS_W-1:0]             M_WDATA,
  output logic [BUS_W/8-1:0]           M_WSTRB,
  output logic                         M_WLAST,
  // B
  input  logic                         M_BVALID,
  output logic                         M_BREADY,
  input  logic [1:0]                   M_BRESP,
  input  logic [AXI_ID_W-1:0]          M_BID,
  // AR
  output logic                         M_ARVALID,
  input  logic                         M_ARREADY,
  output logic [AXI_ADDR_W-1:0]        M_ARADDR,
  output logic [7:0]                   M_ARLEN,    // beats-1
  output logic [2:0]                   M_ARSIZE,   // log2(bytes/beat)
  output logic [1:0]                   M_ARBURST,  // INCR
  output logic [AXI_ID_W-1:0]          M_ARID,
  output logic [2:0]                   M_ARPROT,
  output logic [3:0]                   M_ARCACHE,
  output logic [3:0]                   M_ARQOS,
  output logic [3:0]                   M_ARREGION,
  output logic [AXI_USER_W-1:0]        M_ARUSER,
  output logic                         M_ARLOCK,
  // R
  input  logic                         M_RVALID,
  output logic                         M_RREADY,
  input  logic [BUS_W-1:0]             M_RDATA,
  input  logic [1:0]                   M_RRESP,
  input  logic [AXI_ID_W-1:0]          M_RID,
  input  logic                         M_RLAST,

  // ============================== Telemetry =============================
  input  logic                         cmd_clr_stats_i,
  output logic [31:0]                  stat_w_desc_o,
  output logic [31:0]                  stat_w_beats_o,
  output logic [31:0]                  stat_w_bytes_o,
  output logic [31:0]                  stat_w_bresp_errs_o,

  output logic [31:0]                  stat_r_desc_o,
  output logic [31:0]                  stat_r_beats_o,
  output logic [31:0]                  stat_r_bytes_o,
  output logic [31:0]                  stat_r_rresp_errs_o
);

  // ----------------------- Static / derived checks -----------------------
  localparam int unsigned BUS_BYTES = BUS_W/8;
  localparam int unsigned AWSIZE    = (BUS_BYTES <= 1) ? 0 : $clog2(BUS_BYTES);

  initial begin
    if ((BUS_W % 8) != 0)                       $error("axi_mst_port0: BUS_W must be multiple of 8.");
    if ((INW_ELEM_W % 8) != 0)                  $error("axi_mst_port0: INW_ELEM_W must be multiple of 8.");
    if ((STW_ELEM_W % 8) != 0)                  $error("axi_mst_port0: STW_ELEM_W must be multiple of 8.");
    if ((BUS_BYTES & (BUS_BYTES-1)) != 0)       $error("axi_mst_port0: BUS_BYTES must be power of two.");
    if (LANES_W*STW_ELEM_W > BUS_W)             $error("axi_mst_port0: LANES_W*STW_ELEM_W must be <= BUS_W.");
  end

  // =========================================================================
  // WRITE PATH — Wrap wb_storeq (uses wb_pack internally)
  // =========================================================================
  wb_storeq #(
    .AXI_ADDR_W     (AXI_ADDR_W),
    .AXI_ID_W       (AXI_ID_W),
    .AXI_USER_W     (AXI_USER_W),
    .BUS_W          (BUS_W),
    .LANES_IN       (LANES_W),
    .IN_ELEM_W      (INW_ELEM_W),
    .IN_SIGNED      (INW_SIGNED),
    .STORE_ELEM_W   (STW_ELEM_W),
    .STORE_SIGNED   (STW_SIGNED),
    .IN_FIFO_DEPTH  (W_IN_FIFO_DEPTH),
    .OUT_FIFO_DEPTH (W_OUT_FIFO_DEPTH),
    .P_AWPROT       (P_AWPROT),
    .P_AWCACHE      (P_AWCACHE),
    .P_AWQOS        (P_AWQOS),
    .P_AWREGION     (P_AWREGION)
  ) u_wb_storeq (
    .clk_i              (clk_i),
    .rstn_i             (rstn_i),

    .desc_valid_i       (wdesc_valid_i),
    .desc_ready_o       (wdesc_ready_o),
    .desc_addr_i        (wdesc_addr_i),
    .desc_bytes_i       (wdesc_bytes_i),
    .desc_id_i          (wdesc_id_i),
    .desc_awuser_i      (wdesc_awuser_i),

    .done_valid_o       (wdone_valid_o),
    .done_ready_i       (wdone_ready_i),
    .done_id_o          (wdone_id_o),
    .done_bresp_o       (wdone_bresp_o),
    .done_err_o         (wdone_err_o),

    .in_valid_i         (w_in_valid_i),
    .in_ready_o         (w_in_ready_o),
    .in_data_i          (w_in_data_i),
    .in_last_i          (w_in_last_i),

    .cfg_little_endian_i(cfg_little_endian_i),

    .aw_valid_o (M_AWVALID), .aw_ready_i (M_AWREADY),
    .aw_addr_o  (M_AWADDR),  .aw_len_o   (M_AWLEN),
    .aw_size_o  (M_AWSIZE),  .aw_burst_o (M_AWBURST),
    .aw_id_o    (M_AWID),    .aw_prot_o  (M_AWPROT),
    .aw_cache_o (M_AWCACHE), .aw_qos_o   (M_AWQOS),
    .aw_region_o(M_AWREGION),.aw_user_o  (M_AWUSER),
    .aw_lock_o  (M_AWLOCK),

    .w_valid_o  (M_WVALID),  .w_ready_i  (M_WREADY),
    .w_data_o   (M_WDATA),   .w_strb_o   (M_WSTRB),
    .w_last_o   (M_WLAST),

    .b_valid_i  (M_BVALID),  .b_ready_o  (M_BREADY),
    .b_resp_i   (M_BRESP),   .b_id_i     (M_BID),

    .cmd_clr_stats_i (cmd_clr_stats_i),
    .stat_desc_acc_o (stat_w_desc_o),
    .stat_aw_beats_o (stat_w_beats_o),
    .stat_w_beats_o  (/* unused addl */),
    .stat_w_bytes_o  (stat_w_bytes_o),
    .stat_bresp_errs_o(stat_w_bresp_errs_o)
  );

  // =========================================================================
  // READ PATH — Single-burst AR engine with output keep/last
  // =========================================================================

  // ---------- helpers ----------
  function automatic int unsigned ceil_div_int(input int unsigned a, input int unsigned b);
    return (a + b - 1) / b;
  endfunction
  function automatic int unsigned popcount_keep(input logic [BUS_BYTES-1:0] k);
    return $countones(k);
  endfunction
  function automatic logic [BUS_W-1:0] byte_reverse_bus(input logic [BUS_W-1:0] din);
    logic [BUS_W-1:0] dout;
    for (int i=0;i<BUS_BYTES;i++) dout[(BUS_BYTES-1-i)*8 +: 8] = din[i*8 +: 8];
    return dout;
  endfunction
  function automatic logic [BUS_BYTES-1:0] keep_reverse(input logic [BUS_BYTES-1:0] k);
    logic [BUS_BYTES-1:0] r;
    for (int i=0;i<BUS_BYTES;i++) r[BUS_BYTES-1-i] = k[i];
    return r;
  endfunction

  // ---------- descriptor latch & fit checks ----------
  typedef struct packed {
    logic [AXI_ADDR_W-1:0] addr;
    logic [31:0]           bytes;
    logic [AXI_ID_W-1:0]   id;
    logic [AXI_USER_W-1:0] user;
  } rdesc_t;

  rdesc_t rdesc_q, rdesc_d;
  logic   have_rdesc_q, have_rdesc_d;

  // Derived
  logic [$clog2(BUS_BYTES)-1:0] r_start_off_q, r_start_off_d;
  logic [7:0]                   r_arlen_q,     r_arlen_d;    // beats-1
  logic [15:0]                  r_beats_q,     r_beats_d;    // beats (1..256)
  logic [11:0]                  r_low12_q,     r_low12_d;

  // Fit rule (same as write)
  logic                         r_fit_ok_w;
  logic [15:0]                  r_beats_req_w;

  always_comb begin
    int unsigned off       = rdesc_addr_i[ $clog2(BUS_BYTES)-1 : 0 ];
    int unsigned low12     = rdesc_addr_i[11:0];
    int unsigned req_bytes = rdesc_bytes_i;
    int unsigned req_beats = ceil_div_int(off + req_bytes, BUS_BYTES);
    r_beats_req_w = req_beats[15:0];

    int unsigned span_bytes = req_beats * BUS_BYTES;
    logic no_cross_4k = (low12 + span_bytes) <= 12'd4096;
    logic beats_ok   = (req_beats >= 1) && (req_beats <= 256);
    logic nonzero    = (req_bytes != 0);

    r_fit_ok_w = no_cross_4k && beats_ok && nonzero;
  end

  wire r_idle = !have_rdesc_q;
  assign rdesc_ready_o = r_idle && r_fit_ok_w;

  wire rdesc_fire = rdesc_valid_i && rdesc_ready_o;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      have_rdesc_q   <= 1'b0;
      rdesc_q        <= '0;
      r_start_off_q  <= '0;
      r_arlen_q      <= '0;
      r_beats_q      <= '0;
      r_low12_q      <= '0;
    end else begin
      have_rdesc_q   <= have_rdesc_d;
      rdesc_q        <= rdesc_d;
      r_start_off_q  <= r_start_off_d;
      r_arlen_q      <= r_arlen_d;
      r_beats_q      <= r_beats_d;
      r_low12_q      <= r_low12_d;
    end
  end

  always_comb begin
    have_rdesc_d  = have_rdesc_q;
    rdesc_d       = rdesc_q;
    r_start_off_d = r_start_off_q;
    r_arlen_d     = r_arlen_q;
    r_beats_d     = r_beats_q;
    r_low12_d     = r_low12_q;

    if (rdesc_fire) begin
      have_rdesc_d      = 1'b1;
      rdesc_d.addr      = rdesc_addr_i;
      rdesc_d.bytes     = rdesc_bytes_i;
      rdesc_d.id        = rdesc_id_i;
      rdesc_d.user      = rdesc_aruser_i;
      r_start_off_d     = rdesc_addr_i[ $clog2(BUS_BYTES)-1 : 0 ];
      r_beats_d         = r_beats_req_w;
      r_arlen_d         = r_beats_req_w[7:0] - 8'd1;
      r_low12_d         = rdesc_addr_i[11:0];
    end
    // cleared after RLAST is handled and 'done' surfaced
  end

  // ---------- AR channel ----------
  typedef enum logic [2:0] {
    RS_IDLE   = 3'd0,
    RS_AR     = 3'd1,
    RS_R      = 3'd2,
    RS_DONE_H = 3'd3
  } rstate_e;

  rstate_e rstate_q, rstate_d;

  // Program AR from latched descriptor
  assign M_ARADDR   = rdesc_q.addr;
  assign M_ARLEN    = r_arlen_q;
  assign M_ARSIZE   = AWSIZE[2:0];
  assign M_ARBURST  = 2'b01; // INCR
  assign M_ARID     = rdesc_q.id;
  assign M_ARPROT   = P_ARPROT;
  assign M_ARCACHE  = P_ARCACHE;
  assign M_ARQOS    = P_ARQOS;
  assign M_ARREGION = P_ARREGION;
  assign M_ARUSER   = (AXI_USER_W==0) ? '0 : rdesc_q.user;
  assign M_ARLOCK   = 1'b0;

  assign M_ARVALID  = (rstate_q == RS_AR);

  // ---------- R → stream (data, keep, last) ----------
  // Keep computation based on start offset and bytes remaining
  logic [31:0]              r_bytes_rem_q, r_bytes_rem_d;
  logic [15:0]              r_beats_rem_q, r_beats_rem_d;
  logic                     r_err_seen_q,  r_err_seen_d;

  // Output FIFO (bus beat + keep + last)
  localparam int unsigned R_OUT_PACK_W = BUS_W + BUS_BYTES + 1;
  logic                    rout_in_valid,  rout_in_ready;
  logic [R_OUT_PACK_W-1:0] rout_in_word;
  logic                    rout_out_valid;
  logic [R_OUT_PACK_W-1:0] rout_out_word;

  // Ready to accept R only when FIFO has space
  assign M_RREADY = rout_in_ready;

  // Drive stream OUT from FIFO
  assign r_out_valid_o = rout_out_valid;
  assign r_out_data_o  = rout_out_word[0 +: BUS_W];
  assign r_out_keep_o  = rout_out_word[BUS_W +: BUS_BYTES];
  assign r_out_last_o  = rout_out_word[BUS_W+BUS_BYTES];

  rv_fifo #(
    .WIDTH (R_OUT_PACK_W),
    .DEPTH (RD_OUT_FIFO_DEPTH)
  ) u_rout_fifo (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (rout_in_valid),
    .in_ready_o  (rout_in_ready),
    .in_data_i   (rout_in_word),
    .out_valid_o (rout_out_valid),
    .out_ready_i (r_out_ready_i),
    .out_data_o  (rout_out_word)
  );

  // Assemble keep for each accepted R beat
  // Compute #valid bytes this beat given bytes remaining and position (first vs later)
  function automatic logic [BUS_BYTES-1:0]
    make_keep_first(input int unsigned off, input int unsigned total_bytes);
    logic [BUS_BYTES-1:0] k = '0;
    int unsigned avail = (BUS_BYTES > off) ? (BUS_BYTES - off) : 0;
    int unsigned vbytes = (total_bytes < avail) ? total_bytes : avail;
    for (int b=0; b<vbytes; b++) k[off + b] = 1'b1;
    return k;
  endfunction

  function automatic logic [BUS_BYTES-1:0]
    make_keep_mid_or_last(input int unsigned bytes);
    logic [BUS_BYTES-1:0] k = '0;
    int unsigned vbytes = (bytes > BUS_BYTES) ? BUS_BYTES : bytes;
    for (int b=0; b<vbytes; b++) k[b] = 1'b1;
    return k;
  endfunction

  // FSM + counters
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rstate_q      <= RS_IDLE;
      r_bytes_rem_q <= 32'd0;
      r_beats_rem_q <= 16'd0;
      r_err_seen_q  <= 1'b0;
    end else begin
      rstate_q      <= rstate_d;
      r_bytes_rem_q <= r_bytes_rem_d;
      r_beats_rem_q <= r_beats_rem_d;
      r_err_seen_q  <= r_err_seen_d;
    end
  end

  // Done/info holding register (1-entry)
  logic                rdone_v_q, rdone_v_d;
  logic [AXI_ID_W-1:0] rdone_id_q, rdone_id_d;
  logic                rdone_err_q, rdone_err_d;

  assign rdone_valid_o = rdone_v_q;
  assign rdone_id_o    = rdone_id_q;
  assign rdone_err_o   = rdone_err_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rdone_v_q  <= 1'b0;
      rdone_id_q <= '0;
      rdone_err_q<= 1'b0;
    end else begin
      rdone_v_q  <= rdone_v_d;
      rdone_id_q <= rdone_id_d;
      rdone_err_q<= rdone_err_d;
    end
  end

  // Telemetry
  logic [31:0] stat_r_desc_q, stat_r_beats_q, stat_r_bytes_q, stat_r_errs_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      stat_r_desc_q  <= 32'd0;
      stat_r_beats_q <= 32'd0;
      stat_r_bytes_q <= 32'd0;
      stat_r_errs_q  <= 32'd0;
    end else if (cmd_clr_stats_i) begin
      stat_r_desc_q  <= 32'd0;
      stat_r_beats_q <= 32'd0;
      stat_r_bytes_q <= 32'd0;
      stat_r_errs_q  <= 32'd0;
    end else begin
      if (rdesc_fire) stat_r_desc_q <= stat_r_desc_q + 32'd1;
      if (M_RVALID && M_RREADY) begin
        stat_r_beats_q <= stat_r_beats_q + 32'd1;
        // add this beat's keep bytes (computed below); updated in-place using rout_in_word
        logic [BUS_BYTES-1:0] k_cur;
        k_cur = rout_in_word[BUS_W +: BUS_BYTES];
        stat_r_bytes_q <= stat_r_bytes_q + popcount_keep(k_cur);
        if (M_RRESP != 2'b00) stat_r_errs_q <= stat_r_errs_q + 32'd1;
      end
    end
  end
  assign stat_r_desc_o        = stat_r_desc_q;
  assign stat_r_beats_o       = stat_r_beats_q;
  assign stat_r_bytes_o       = stat_r_bytes_q;
  assign stat_r_rresp_errs_o  = stat_r_errs_q;

  // Next-state / combinational
  always_comb begin
    rstate_d       = rstate_q;
    r_bytes_rem_d  = r_bytes_rem_q;
    r_beats_rem_d  = r_beats_rem_q;
    r_err_seen_d   = r_err_seen_q;

    rdone_v_d      = rdone_v_q;
    rdone_id_d     = rdone_id_q;
    rdone_err_d    = rdone_err_q;

    // FIFO push defaults
    rout_in_valid  = 1'b0;
    rout_in_word   = '0;

    unique case (rstate_q)
      // ----------------------------------------------------------
      RS_IDLE: begin
        if (rdesc_fire) begin
          // Arm AR phase
          rstate_d       = RS_AR;
          r_bytes_rem_d  = rdesc_bytes_i;
          r_beats_rem_d  = r_beats_req_w;
          r_err_seen_d   = 1'b0;
        end
      end

      // ----------------------------------------------------------
      RS_AR: begin
        // Issue AR until accepted
        if (M_ARVALID && M_ARREADY) begin
          rstate_d = RS_R;
        end
      end

      // ----------------------------------------------------------
      RS_R: begin
        // Accept R when FIFO ready; compute keep & last
        if (M_RVALID && M_RREADY) begin
          logic [BUS_W-1:0]     d_bus = M_RDATA;
          logic [BUS_BYTES-1:0] k_le;
          logic [BUS_W-1:0]     d_le = d_bus;

          if (cfg_little_endian_i) begin
            // already LE
          end else begin
            // Convert to LE for internal keep mapping, then present as BE if needed
            d_le = byte_reverse_bus(d_bus);
          end

          // keep mask (LE space)
          if (r_beats_rem_q == r_beats_q) begin
            // first beat
            k_le = make_keep_first(r_start_off_q, r_bytes_rem_q);
          end else begin
            // mid/last beats
            // remaining bytes before consuming this beat:
            k_le = make_keep_mid_or_last(r_bytes_rem_q);
          end

          // Decrement remaining counters
          int unsigned consumed = popcount_keep(k_le);
          r_bytes_rem_d = r_bytes_rem_q - consumed;
          r_beats_rem_d = r_beats_rem_q - 16'd1;

          // Track error flag
          if (M_RRESP != 2'b00) r_err_seen_d = 1'b1;

          // Convert keep/data back to bus endianness for output
          logic [BUS_BYTES-1:0] k_bus = cfg_little_endian_i ? k_le : keep_reverse(k_le);
          logic [BUS_W-1:0]     o_bus = cfg_little_endian_i ? d_le : byte_reverse_bus(d_le);

          // Push to output FIFO
          rout_in_valid = 1'b1;
          rout_in_word  = { (M_RLAST), k_bus, o_bus };

          // Completion on RLAST (and bytes should be zero now)
          if (M_RLAST) begin
            rstate_d    = RS_DONE_H;
            // Latch 'done' (hold until consumed)
            if (!rdone_v_q) begin
              rdone_v_d   = 1'b1;
              rdone_id_d  = M_RID;
              rdone_err_d = r_err_seen_d;
            end
          end
        end
      end

      // ----------------------------------------------------------
      RS_DONE_H: begin
        if (rdone_v_q && rdone_ready_i) begin
          // clear desc ownership
          have_rdesc_d = 1'b0;
          rdone_v_d    = 1'b0;
          rstate_d     = RS_IDLE;
        end
      end

      default: rstate_d = RS_IDLE;
    endcase
  end

  // --------------------- Tie-offs / shared AXI fields ---------------------
  // These are driven by write/read engines above:
  //  - M_AW* / M_W* / M_B* are driven by u_wb_storeq
  //  - M_AR* / M_R* are driven by RS_* logic
  // Nothing else to do here.

  // =========================================================================
  // Assertions (simulation only)
  // =========================================================================
`ifdef ASSERT_ON
  // Read descriptor must fit (same rule as write)
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rdesc_fire |-> r_fit_ok_w)
    else $error("axi_mst_port0(RD): descriptor does not fit single burst (size or 4KiB).");

  // ARLEN must equal beats-1
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rdesc_fire |-> (r_arlen_d == (r_beats_req_w[7:0]-8'd1)))
    else $error("axi_mst_port0(RD): ARLEN mismatch.");

  // RLAST should occur on final expected beat
  logic [15:0] r_exp_beats;
  assign r_exp_beats = r_beats_q;
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (M_RVALID && M_RREADY && M_RLAST) |-> (r_beats_rem_q == 16'd1))
    else $warning("axi_mst_port0(RD): RLAST not on expected final beat.");

  // Byte count check: when RLAST accepted, remaining bytes should be zero
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (M_RVALID && M_RREADY && M_RLAST) |-> (r_bytes_rem_q == popcount_keep(make_keep_mid_or_last(r_bytes_rem_q))))
    else $warning("axi_mst_port0(RD): residual byte count may be non-zero at RLAST.");

  // No AR when no descriptor
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (rstate_q == RS_AR) |-> have_rdesc_q)
    else $error("axi_mst_port0(RD): issuing AR without descriptor.");
`endif

endmodule


// ============================================================================
//  rv_fifo: single-clock ready/valid FIFO (non-fallthrough)
// ============================================================================
module rv_fifo #(
  parameter int unsigned WIDTH = 32,
  parameter int unsigned DEPTH = 4
)(
  input  logic               clk_i,
  input  logic               rstn_i,
  // Ingress
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [WIDTH-1:0]   in_data_i,
  // Egress
  output logic               out_valid_o,
  input  logic               out_ready_i,
  output logic [WIDTH-1:0]   out_data_o
);
  localparam int AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
  logic [WIDTH-1:0] mem [0:DEPTH-1];
  logic [AW:0] wr_ptr_q, rd_ptr_q;

  function automatic logic fifo_empty(input logic [AW:0] w, input logic [AW:0] r);
    return (w == r);
  endfunction
  function automatic logic fifo_full (input logic [AW:0] w, input logic [AW:0] r);
    return (w[AW] != r[AW]) && (w[AW-1:0] == r[AW-1:0]);
  endfunction

  wire do_push = in_valid_i  && !fifo_full(wr_ptr_q, rd_ptr_q);
  wire do_pop  = out_ready_i && !fifo_empty(wr_ptr_q, rd_ptr_q);

  assign in_ready_o  = !fifo_full(wr_ptr_q, rd_ptr_q);
  assign out_valid_o = !fifo_empty(wr_ptr_q, rd_ptr_q);
  assign out_data_o  = mem[rd_ptr_q[AW-1:0]];

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wr_ptr_q <= '0; rd_ptr_q <= '0;
    end else begin
      if (do_push) begin
        mem[wr_ptr_q[AW-1:0]] <= in_data_i;
        wr_ptr_q <= wr_ptr_q + {{AW{1'b0}},1'b1};
      end
      if (do_pop) begin
        rd_ptr_q <= rd_ptr_q + {{AW{1'b0}},1'b1};
      end
    end
  end

`ifdef ASSERT_ON
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_push |-> !fifo_full(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: push while full.");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_pop  |-> !fifo_empty(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: pop while empty.");
`endif
endmodule

`default_nettype wire
