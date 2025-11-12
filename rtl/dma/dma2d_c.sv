// ============================================================================
//  DMA2D_C — Results/“C” DMA: GB Read  -> AXI4 Write (egress)
//  File: dma2d_c.sv
//  Description:
//    - Polls doorbell (AXI read), fetches 1-beat descriptor (AXI read)
//    - 2D copy from Global Buffer (GB) to Host memory (AXI write)
//    - Strict 4KiB and 256-beat burst compliance
//    - Strict alignment by default (beat-aligned addresses/lengths)
//    - One-shot IRQ pulse on done (and on error)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// ----------------------------------------------
// Top-level wrapper (distinct defaults for C-DMA)
// ----------------------------------------------
module dma2d_c #(
  // ---------------- AXI widths ----------------
  parameter int unsigned AXI_ADDR_W = 48,
  parameter int unsigned AXI_DATA_W = 512,
  parameter int unsigned AXI_ID_W   = 6,
  parameter int unsigned AXI_USER_W = 1,

  // ---------------- Descriptor/DBELL defaults -------------
  parameter logic [AXI_ADDR_W-1:0] DESC_ADDR_C     = 48'h0000_0000_C000, // 64B aligned
  parameter logic [AXI_ADDR_W-1:0] DOORBELL_ADDR_C = 48'h0000_0000_D000, // 4B aligned

  // ---------------- Engine knobs ----------------
  parameter int unsigned POLL_CYCLES     = 4096,  // doorbell poll cadence
  parameter int unsigned FIFO_DEPTH      = 32,    // W-data FIFO depth (beats)
  parameter bit          STRICT_ALIGN    = 1'b1,
  parameter logic [AXI_ID_W-1:0] AXI_FIXED_ID = 'h03, // ID for this DMA
  // AXI attributes (writes)
  parameter logic [3:0] AXI_AWQOS   = 4'h6,
  parameter logic [3:0] AXI_AWCACHE = 4'b1111,
  parameter logic [2:0] AXI_AWPROT  = 3'b000,
  // AXI attributes (reads for descriptor/doorbell)
  parameter logic [3:0] AXI_ARQOS   = 4'h2,
  parameter logic [3:0] AXI_ARCACHE = 4'b1011,
  parameter logic [2:0] AXI_ARPROT  = 3'b000
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,

  // ---------------- AXI4 Master ----------------
  // AW
  output logic [AXI_ID_W-1:0]          m_awid_o,
  output logic [AXI_ADDR_W-1:0]        m_awaddr_o,
  output logic [7:0]                   m_awlen_o,
  output logic [2:0]                   m_awsize_o,
  output logic [1:0]                   m_awburst_o,
  output logic                         m_awlock_o,
  output logic [3:0]                   m_awcache_o,
  output logic [2:0]                   m_awprot_o,
  output logic [3:0]                   m_awqos_o,
  output logic [AXI_USER_W-1:0]        m_awuser_o,
  output logic                         m_awvalid_o,
  input  logic                         m_awready_i,

  // W
  output logic [AXI_DATA_W-1:0]        m_wdata_o,
  output logic [AXI_DATA_W/8-1:0]      m_wstrb_o,
  output logic                         m_wlast_o,
  output logic [AXI_USER_W-1:0]        m_wuser_o,
  output logic                         m_wvalid_o,
  input  logic                         m_wready_i,

  // B
  input  logic [AXI_ID_W-1:0]          m_bid_i,
  input  logic [1:0]                   m_bresp_i,
  input  logic [AXI_USER_W-1:0]        m_buser_i,
  input  logic                         m_bvalid_i,
  output logic                         m_bready_o,

  // AR (descriptor/doorbell reads)
  output logic [AXI_ID_W-1:0]          m_arid_o,
  output logic [AXI_ADDR_W-1:0]        m_araddr_o,
  output logic [7:0]                   m_arlen_o,
  output logic [2:0]                   m_arsize_o,
  output logic [1:0]                   m_arburst_o,
  output logic                         m_arlock_o,
  output logic [3:0]                   m_arcache_o,
  output logic [2:0]                   m_arprot_o,
  output logic [3:0]                   m_arqos_o,
  output logic [AXI_USER_W-1:0]        m_aruser_o,
  output logic                         m_arvalid_o,
  input  logic                         m_arready_i,

  // R
  input  logic [AXI_ID_W-1:0]          m_rid_i,
  input  logic [AXI_DATA_W-1:0]        m_rdata_i,
  input  logic [1:0]                   m_rresp_i,
  input  logic                         m_rlast_i,
  input  logic [AXI_USER_W-1:0]        m_ruser_i,
  input  logic                         m_rvalid_i,
  output logic                         m_rready_o,

  // ---------------- GB Read Interface (beat-granular) ----------------
  // Command: one beat per command, addressed by byte address
  output logic                         gb_r_cmd_valid_o,
  input  logic                         gb_r_cmd_ready_i,
  output logic [31:0]                  gb_r_cmd_addr_o,  // byte address into GB

  // Data return channel
  input  logic                         gb_r_data_valid_i,
  output logic                         gb_r_data_ready_o,
  input  logic [AXI_DATA_W-1:0]        gb_r_data_i,

  // ---------------- IRQ ----------------
  output logic                         irq_o
);

  // Engine instance
  dma2d_gb2wr_core #(
    .AXI_ADDR_W     (AXI_ADDR_W),
    .AXI_DATA_W     (AXI_DATA_W),
    .AXI_ID_W       (AXI_ID_W),
    .AXI_USER_W     (AXI_USER_W),

    .DESC_ADDR_DFLT (DESC_ADDR_C),
    .DBELL_ADDR_DFLT(DOORBELL_ADDR_C),

    .POLL_CYCLES    (POLL_CYCLES),
    .FIFO_DEPTH     (FIFO_DEPTH),
    .STRICT_ALIGN   (STRICT_ALIGN),

    .AXI_FIXED_ID   (AXI_FIXED_ID),
    .AXI_AWQOS      (AXI_AWQOS),
    .AXI_AWCACHE    (AXI_AWCACHE),
    .AXI_AWPROT     (AXI_AWPROT),

    .AXI_ARQOS      (AXI_ARQOS),
    .AXI_ARCACHE    (AXI_ARCACHE),
    .AXI_ARPROT     (AXI_ARPROT)
  ) u_core (
    .clk_i          (clk_i),
    .rstn_i         (rstn_i),

    .m_awid_o       (m_awid_o),
    .m_awaddr_o     (m_awaddr_o),
    .m_awlen_o      (m_awlen_o),
    .m_awsize_o     (m_awsize_o),
    .m_awburst_o    (m_awburst_o),
    .m_awlock_o     (m_awlock_o),
    .m_awcache_o    (m_awcache_o),
    .m_awprot_o     (m_awprot_o),
    .m_awqos_o      (m_awqos_o),
    .m_awuser_o     (m_awuser_o),
    .m_awvalid_o    (m_awvalid_o),
    .m_awready_i    (m_awready_i),

    .m_wdata_o      (m_wdata_o),
    .m_wstrb_o      (m_wstrb_o),
    .m_wlast_o      (m_wlast_o),
    .m_wuser_o      (m_wuser_o),
    .m_wvalid_o     (m_wvalid_o),
    .m_wready_i     (m_wready_i),

    .m_bid_i        (m_bid_i),
    .m_bresp_i      (m_bresp_i),
    .m_buser_i      (m_buser_i),
    .m_bvalid_i     (m_bvalid_i),
    .m_bready_o     (m_bready_o),

    .m_arid_o       (m_arid_o),
    .m_araddr_o     (m_araddr_o),
    .m_arlen_o      (m_arlen_o),
    .m_arsize_o     (m_arsize_o),
    .m_arburst_o    (m_arburst_o),
    .m_arlock_o     (m_arlock_o),
    .m_arcache_o    (m_arcache_o),
    .m_arprot_o     (m_arprot_o),
    .m_arqos_o      (m_arqos_o),
    .m_aruser_o     (m_aruser_o),
    .m_arvalid_o    (m_arvalid_o),
    .m_arready_i    (m_arready_i),

    .m_rid_i        (m_rid_i),
    .m_rdata_i      (m_rdata_i),
    .m_rresp_i      (m_rresp_i),
    .m_rlast_i      (m_rlast_i),
    .m_ruser_i      (m_ruser_i),
    .m_rvalid_i     (m_rvalid_i),
    .m_rready_o     (m_rready_o),

    .gb_r_cmd_valid_o (gb_r_cmd_valid_o),
    .gb_r_cmd_ready_i (gb_r_cmd_ready_i),
    .gb_r_cmd_addr_o  (gb_r_cmd_addr_o),

    .gb_r_data_valid_i(gb_r_data_valid_i),
    .gb_r_data_ready_o(gb_r_data_ready_o),
    .gb_r_data_i      (gb_r_data_i),

    .irq_o            (irq_o)
  );

endmodule

// ============================================================================
//  dma2d_gb2wr_core
//  Purpose: GB -> AXI Write, 2D engine with doorbell/descriptor fetch (via AR/R)
//  Contract:
//   * STRICT_ALIGN=1: {dst_host_addr, src_gb_addr, row_bytes} % BEAT_BYTES == 0
//   * burst respects 4KiB boundary and <=256 beats, also capped by FIFO_DEPTH
//   * GB interface: request one-beat reads by address; data returns in-order
// ============================================================================
module dma2d_gb2wr_core #(
  parameter int unsigned AXI_ADDR_W = 48,
  parameter int unsigned AXI_DATA_W = 512,
  parameter int unsigned AXI_ID_W   = 6,
  parameter int unsigned AXI_USER_W = 1,

  parameter logic [AXI_ADDR_W-1:0] DESC_ADDR_DFLT  = '0,
  parameter logic [AXI_ADDR_W-1:0] DBELL_ADDR_DFLT = '0,

  parameter int unsigned POLL_CYCLES = 4096,
  parameter int unsigned FIFO_DEPTH  = 32,
  parameter bit          STRICT_ALIGN= 1'b1,

  parameter logic [AXI_ID_W-1:0] AXI_FIXED_ID = '0,
  parameter logic [3:0]          AXI_AWQOS    = 4'h6,
  parameter logic [3:0]          AXI_AWCACHE  = 4'hF,
  parameter logic [2:0]          AXI_AWPROT   = 3'b000,

  parameter logic [3:0]          AXI_ARQOS    = 4'h2,
  parameter logic [3:0]          AXI_ARCACHE  = 4'hB,
  parameter logic [2:0]          AXI_ARPROT   = 3'b000
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,

  // -------- AXI Write Master --------
  output logic [AXI_ID_W-1:0]          m_awid_o,
  output logic [AXI_ADDR_W-1:0]        m_awaddr_o,
  output logic [7:0]                   m_awlen_o,
  output logic [2:0]                   m_awsize_o,
  output logic [1:0]                   m_awburst_o,
  output logic                         m_awlock_o,
  output logic [3:0]                   m_awcache_o,
  output logic [2:0]                   m_awprot_o,
  output logic [3:0]                   m_awqos_o,
  output logic [AXI_USER_W-1:0]        m_awuser_o,
  output logic                         m_awvalid_o,
  input  logic                         m_awready_i,

  output logic [AXI_DATA_W-1:0]        m_wdata_o,
  output logic [AXI_DATA_W/8-1:0]      m_wstrb_o,
  output logic                         m_wlast_o,
  output logic [AXI_USER_W-1:0]        m_wuser_o,
  output logic                         m_wvalid_o,
  input  logic                         m_wready_i,

  input  logic [AXI_ID_W-1:0]          m_bid_i,
  input  logic [1:0]                   m_bresp_i,
  input  logic [AXI_USER_W-1:0]        m_buser_i,
  input  logic                         m_bvalid_i,
  output logic                         m_bready_o,

  // -------- AXI Read (descriptor & doorbell only) --------
  output logic [AXI_ID_W-1:0]          m_arid_o,
  output logic [AXI_ADDR_W-1:0]        m_araddr_o,
  output logic [7:0]                   m_arlen_o,
  output logic [2:0]                   m_arsize_o,
  output logic [1:0]                   m_arburst_o,
  output logic                         m_arlock_o,
  output logic [3:0]                   m_arcache_o,
  output logic [2:0]                   m_arprot_o,
  output logic [3:0]                   m_arqos_o,
  output logic [AXI_USER_W-1:0]        m_aruser_o,
  output logic                         m_arvalid_o,
  input  logic                         m_arready_i,

  input  logic [AXI_ID_W-1:0]          m_rid_i,
  input  logic [AXI_DATA_W-1:0]        m_rdata_i,
  input  logic [1:0]                   m_rresp_i,
  input  logic                         m_rlast_i,
  input  logic [AXI_USER_W-1:0]        m_ruser_i,
  input  logic                         m_rvalid_i,
  output logic                         m_rready_o,

  // -------- GB read side --------
  output logic                         gb_r_cmd_valid_o,
  input  logic                         gb_r_cmd_ready_i,
  output logic [31:0]                  gb_r_cmd_addr_o,

  input  logic                         gb_r_data_valid_i,
  output logic                         gb_r_data_ready_o,
  input  logic [AXI_DATA_W-1:0]        gb_r_data_i,

  // -------- IRQ --------
  output logic                         irq_o
);

  // ------------------------------
  // Derived constants / typedefs
  // ------------------------------
  localparam int unsigned BEAT_BYTES = AXI_DATA_W/8;
  localparam int unsigned AWSIZE     = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);

  typedef struct packed {
    logic [63:0]  host_addr;          // desc.src_addr (repurposed)
    logic [31:0]  gb_addr;            // desc.dst_gb_addr (repurposed)
    logic [31:0]  row_bytes;
    logic [15:0]  rows;
    logic [15:0]  flags;
    logic signed [31:0] host_stride;  // desc.src_stride_bytes
    logic signed [31:0] gb_stride;    // desc.dst_stride_bytes
    logic [15:0]  burst_bytes_hint;
    logic [15:0]  rsvd;
  } desc_t;

  // ------------------------------
  // AXI tie-offs / attributes
  // ------------------------------
  assign m_awid_o    = AXI_FIXED_ID;
  assign m_awsize_o  = AWSIZE[2:0];
  assign m_awburst_o = 2'b01; // INCR
  assign m_awlock_o  = 1'b0;
  assign m_awcache_o = AXI_AWCACHE;
  assign m_awprot_o  = AXI_AWPROT;
  assign m_awqos_o   = AXI_AWQOS;
  assign m_awuser_o  = '0;

  assign m_wuser_o   = '0;
  assign m_wstrb_o   = {BEAT_BYTES{1'b1}};

  assign m_bready_o  = 1'b1;

  // AR (reads only for descriptor and doorbell)
  assign m_arid_o    = AXI_FIXED_ID;
  assign m_arsize_o  = AWSIZE[2:0];
  assign m_arburst_o = 2'b01;
  assign m_arlock_o  = 1'b0;
  assign m_arcache_o = AXI_ARCACHE;
  assign m_arprot_o  = AXI_ARPROT;
  assign m_arqos_o   = AXI_ARQOS;
  assign m_aruser_o  = '0;

  // R channel: ready when we want a single-beat read
  // (Descriptor/doorbell are 1-beat fetches; no large FIFO needed here)
  assign m_rready_o  = 1'b1;

  // ------------------------------
  // W-data FIFO (from GB return to AXI W)
  // ------------------------------
  localparam int unsigned F_AW = (FIFO_DEPTH <= 2) ? 2 : $clog2(FIFO_DEPTH);
  logic [AXI_DATA_W-1:0] wf_mem   [0:FIFO_DEPTH-1];
  logic [F_AW:0]         wf_wptr_q, wf_rptr_q;
  logic                  wf_full, wf_empty;
  logic                  wf_push, wf_pop;
  logic [AXI_DATA_W-1:0] wf_rdata;

  assign wf_full  = (wf_wptr_q[F_AW]     != wf_rptr_q[F_AW]) &&
                    (wf_wptr_q[F_AW-1:0] == wf_rptr_q[F_AW-1:0]);
  assign wf_empty = (wf_wptr_q == wf_rptr_q);
  assign wf_rdata = wf_mem[wf_rptr_q[F_AW-1:0]];

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wf_wptr_q <= '0;
      wf_rptr_q <= '0;
    end else begin
      if (wf_push && !wf_full) begin
        wf_mem[wf_wptr_q[F_AW-1:0]] <= gb_r_data_i;
        wf_wptr_q <= wf_wptr_q + 1'b1;
      end
      if (wf_pop && !wf_empty) begin
        wf_rptr_q <= wf_rptr_q + 1'b1;
      end
    end
  end

  // Producer/consumer with GB
  assign gb_r_data_ready_o = !wf_full;
  assign wf_push           = gb_r_data_valid_i && gb_r_data_ready_o;

  // ------------------------------
  // Doorbell/descriptor control
  // ------------------------------
  logic [AXI_ADDR_W-1:0] dbell_addr_q, desc_addr_q;
  logic [31:0]           last_dbell_q, dbell_sample_q;
  logic [31:0]           poll_cnt_q;

  // Latched descriptor
  desc_t                 desc_q;

  // ------------------------------
  // Transfer bookkeeping
  // ------------------------------
  logic [31:0] beats_row_total_q, beats_row_left_q;
  logic [31:0] rows_left_q;

  logic [AXI_ADDR_W-1:0] host_addr_q;    // AXI write address cursor
  logic [31:0]           gb_addr_q;      // GB read address cursor (byte addresses)

  // Chunk planner (per-burst)
  logic [31:0] chunk_beats_q;            // beats in current chunk
  logic [31:0] chunk_cmds_issued_q;      // GB commands issued for chunk
  logic [31:0] chunk_beats_ready_q;      // data beats present in FIFO for chunk
  logic [31:0] chunk_beats_sent_q;       // beats already sent on W for chunk

  // Write stream state
  logic        aw_inflight_q;            // waiting for B after W done
  logic [31:0] w_beats_left_q;

  // Error / IRQ
  logic        irq_pulse_q, irq_pulse_d;
  logic [3:0]  err_code_q;
  localparam logic [3:0]
    EC_NONE     = 4'h0,
    EC_ALIGN    = 4'h1,
    EC_AXI_W    = 4'h2,
    EC_PARAM    = 4'h3;

  // ------------------------------
  // Utility: calc burst len (beats)
  // ------------------------------
  function automatic [7:0] calc_burst_len(input logic [AXI_ADDR_W-1:0] addr,
                                          input logic [31:0] beats_left,
                                          input logic [31:0] hint_beats,
                                          input logic [31:0] fifo_room);
    int unsigned bytes_to_4k    = 4096 - (addr[11:0]);
    int unsigned max_beats_4k   = bytes_to_4k / BEAT_BYTES;
    int unsigned cap256         = 256;
    int unsigned cap_hint       = (hint_beats == 0) ? cap256 : (hint_beats > cap256 ? cap256 : hint_beats);
    int unsigned m1             = (beats_left   < cap_hint)   ? beats_left   : cap_hint;
    int unsigned m2             = (max_beats_4k < m1)         ? max_beats_4k : m1;
    int unsigned m3             = (fifo_room    < m2)         ? fifo_room    : m2;
    calc_burst_len = (m3 == 0) ? 8'd1 : 8'(m3); // never returns 0
  endfunction

  // ------------------------------
  // FSM
  // ------------------------------
  typedef enum logic [3:0] {
    S_RESET, S_IDLE, S_POLL_AR, S_POLL_WAIT,
    S_DESC_AR, S_DESC_WAIT, S_VALIDATE, S_PREP_ROW,
    S_CHUNK_PREP, S_CHUNK_FILL, S_AW_SETUP, S_W_STREAM, S_WAIT_B,
    S_NEXT_ROW, S_DONE, S_ERROR
  } state_e;
  state_e state_q, state_d;

  // AR command regs (for doorbell/desc)
  logic [AXI_ADDR_W-1:0] ar_addr_q, ar_addr_d;
  logic [7:0]            ar_len_q,  ar_len_d;
  logic                  ar_valid_q, ar_valid_d;

  // AW command regs (for payload)
  logic [AXI_ADDR_W-1:0] aw_addr_q, aw_addr_d;
  logic [7:0]            aw_len_q,  aw_len_d;
  logic                  aw_valid_q, aw_valid_d;

  // W channel drive
  assign m_wdata_o  = wf_rdata;
  assign m_wvalid_o = (state_q == S_W_STREAM) && !wf_empty;
  assign wf_pop     = m_wvalid_o && m_wready_i;

  // wlast: true when last beat of chunk is being accepted
  assign m_wlast_o  = (w_beats_left_q == 32'd1) && m_wvalid_o && m_wready_i;

  // Ready defaults
  // (m_rready_o already tied 1; m_bready_o tied 1)

  // AXI drive
  assign m_araddr_o  = ar_addr_q;
  assign m_arlen_o   = ar_len_q;
  assign m_arvalid_o = ar_valid_q;

  assign m_awaddr_o  = aw_addr_q;
  assign m_awlen_o   = aw_len_q;
  assign m_awvalid_o = aw_valid_q;

  // GB request channel
  assign gb_r_cmd_addr_o  = gb_addr_q;
  assign gb_r_cmd_valid_o = (state_q == S_CHUNK_FILL) &&
                            (chunk_cmds_issued_q < chunk_beats_q) &&
                            !wf_full;

  // ------------------------------
  // Seq
  // ------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q         <= S_RESET;

      dbell_addr_q    <= DBELL_ADDR_DFLT;
      desc_addr_q     <= DESC_ADDR_DFLT;
      last_dbell_q    <= 32'hDEAD_BEEF;
      poll_cnt_q      <= '0;

      ar_addr_q       <= '0;
      ar_len_q        <= '0;
      ar_valid_q      <= 1'b0;

      aw_addr_q       <= '0;
      aw_len_q        <= '0;
      aw_valid_q      <= 1'b0;

      // Descriptor/counters
      desc_q          <= '0;
      beats_row_total_q <= '0;
      beats_row_left_q  <= '0;
      rows_left_q       <= '0;
      host_addr_q       <= '0;
      gb_addr_q         <= '0;

      chunk_beats_q      <= '0;
      chunk_cmds_issued_q<= '0;
      chunk_beats_ready_q<= '0;
      chunk_beats_sent_q <= '0;

      aw_inflight_q    <= 1'b0;
      w_beats_left_q   <= 32'd0;

      irq_pulse_q      <= 1'b0;
      err_code_q       <= EC_NONE;
    end
    else begin
      state_q    <= state_d;

      // Clean pulses
      irq_pulse_q <= irq_pulse_d;

      // Doorbell polling cadence
      if (state_q == S_IDLE) poll_cnt_q <= poll_cnt_q + 32'd1;
      else                   poll_cnt_q <= 32'd0;

      // AR regs
      ar_addr_q  <= ar_addr_d;
      ar_len_q   <= ar_len_d;
      ar_valid_q <= ar_valid_d && !m_arready_i; // keep asserted until accepted
      if (ar_valid_d && m_arready_i) ar_valid_q <= 1'b0;

      // AW regs
      aw_addr_q  <= aw_addr_d;
      aw_len_q   <= aw_len_d;
      aw_valid_q <= aw_valid_d && !m_awready_i; // keep asserted until accepted
      if (aw_valid_d && m_awready_i) aw_valid_q <= 1'b0;

      // Capture doorbell when R returns
      if (state_q == S_POLL_WAIT && m_rvalid_i && m_rlast_i) begin
        dbell_sample_q <= m_rdata_i[31:0];
        if (m_rdata_i[31:0] != last_dbell_q) begin
          last_dbell_q <= m_rdata_i[31:0];
        end
      end

      // Latch descriptor on read completion
      if (state_q == S_DESC_WAIT && m_rvalid_i && m_rlast_i) begin
        desc_q.host_addr     <= m_rdata_i[63:0];
        desc_q.gb_addr       <= m_rdata_i[95:64];
        desc_q.row_bytes     <= m_rdata_i[127:96];
        desc_q.rows          <= m_rdata_i[143:128];
        desc_q.flags         <= m_rdata_i[159:144];
        desc_q.host_stride   <= m_rdata_i[191:160];
        desc_q.gb_stride     <= m_rdata_i[223:192];
        desc_q.burst_bytes_hint <= m_rdata_i[239:224];
        desc_q.rsvd          <= m_rdata_i[255:240];
      end

      // FIFO accounting for chunk (beats ready)
      if (state_q == S_CHUNK_FILL) begin
        // Count accepted GB data beats into "ready for this chunk"
        if (wf_push && (chunk_beats_ready_q < chunk_beats_q))
          chunk_beats_ready_q <= chunk_beats_ready_q + 32'd1;

        // GB commands issued for chunk
        if (gb_r_cmd_valid_o && gb_r_cmd_ready_i) begin
          chunk_cmds_issued_q <= chunk_cmds_issued_q + 32'd1;
          gb_addr_q           <= gb_addr_q + BEAT_BYTES;
        end
      end

      // AW accepted (begin W streaming)
      if (state_q == S_AW_SETUP && m_awready_i) begin
        aw_inflight_q  <= 1'b1;
        w_beats_left_q <= chunk_beats_q;
      end

      // W stream progress
      if (state_q == S_W_STREAM && m_wvalid_o && m_wready_i) begin
        chunk_beats_sent_q <= chunk_beats_sent_q + 32'd1;
        if (w_beats_left_q != 0) w_beats_left_q <= w_beats_left_q - 32'd1;
      end

      // B response complete
      if (state_q == S_WAIT_B && m_bvalid_i) begin
        aw_inflight_q <= 1'b0;
      end
    end
  end

  // ------------------------------
  // Next-state & comb outputs
  // ------------------------------
  always_comb begin
    state_d       = state_q;
    irq_pulse_d   = 1'b0;

    // Default: no AR/AW new requests
    ar_valid_d    = 1'b0;  ar_addr_d = ar_addr_q; ar_len_d = ar_len_q;
    aw_valid_d    = 1'b0;  aw_addr_d = aw_addr_q; aw_len_d = aw_len_q;

    // Defaults for GB cmd (addr driven by seq)
    // gb_r_cmd_valid_o asserted combinationally in S_CHUNK_FILL (above)

    unique case (state_q)
      // ------------------------------------------------
      S_RESET: begin
        state_d = S_IDLE;
      end

      // ------------------------------------------------
      S_IDLE: begin
        if (poll_cnt_q >= POLL_CYCLES[31:0]) begin
          // Poll doorbell: AR single beat
          ar_addr_d  = (dbell_addr_q & ~({{(AXI_ADDR_W-2){1'b0}}, 2'b11})); // 4B align mask
          ar_len_d   = 8'd0;
          ar_valid_d = 1'b1;
          state_d    = S_POLL_AR;
        end
      end

      S_POLL_AR: begin
        // Wait for AR accept
        if (m_arready_i) state_d = S_POLL_WAIT;
        else begin
          ar_valid_d = 1'b1;
        end
      end

      S_POLL_WAIT: begin
        // Wait for single-beat R
        if (m_rvalid_i && m_rlast_i) begin
          if (m_rdata_i[31:0] != last_dbell_q) begin
            // Fetch descriptor (one beat, beat-aligned)
            ar_addr_d  = (desc_addr_q & ~({{(AXI_ADDR_W-$clog2(BEAT_BYTES)){1'b0}}, {$clog2(BEAT_BYTES){1'b1}}})) | '0;
            ar_len_d   = 8'd0;
            ar_valid_d = 1'b1;
            state_d    = S_DESC_AR;
          end else begin
            state_d = S_IDLE;
          end
        end
      end

      S_DESC_AR: begin
        if (m_arready_i) state_d = S_DESC_WAIT;
        else begin
          ar_valid_d = 1'b1;
        end
      end

      S_DESC_WAIT: begin
        if (m_rvalid_i && m_rlast_i) begin
          state_d = S_VALIDATE;
        end
      end

      // ------------------------------------------------
      S_VALIDATE: begin
        // Alignment & sanity checks
        bit ok = 1'b1;
        if (STRICT_ALIGN) begin
          if ((desc_q.host_addr % BEAT_BYTES) != 0) ok = 1'b0;
          if ((desc_q.gb_addr   % BEAT_BYTES) != 0) ok = 1'b0;
          if ((desc_q.row_bytes % BEAT_BYTES) != 0) ok = 1'b0;
        end
        if (desc_q.row_bytes == 0 || desc_q.rows == 0) ok = 1'b0;

        if (!ok) begin
          state_d = S_ERROR;
        end else begin
          state_d = S_PREP_ROW;
        end
      end

      S_PREP_ROW: begin
        // Init row counters in seq; move on
        state_d = S_CHUNK_PREP;
      end

      S_CHUNK_PREP: begin
        // Choose chunk beats (respecting 4KiB boundary at host_addr)
        // Also cap by FIFO room: since we prefill, we require chunk_beats <= FIFO_DEPTH
        // Move to FILL
        state_d = S_CHUNK_FILL;
      end

      S_CHUNK_FILL: begin
        // Issue GB read commands and accept data until we have chunk_beats ready
        // When ready, proceed to AW setup
        if (chunk_beats_ready_q >= chunk_beats_q) begin
          aw_addr_d  = host_addr_q;
          aw_len_d   = chunk_beats_q[7:0] - 8'd1;
          aw_valid_d = 1'b1;
          state_d    = S_AW_SETUP;
        end
      end

      S_AW_SETUP: begin
        if (m_awready_i) begin
          // Start W streaming
          state_d = S_W_STREAM;
        end else begin
          aw_valid_d = 1'b1;
        end
      end

      S_W_STREAM: begin
        // Stream chunk from FIFO to W channel
        if (m_wvalid_o && m_wready_i && (w_beats_left_q == 32'd1)) begin
          state_d = S_WAIT_B;
        end
      end

      S_WAIT_B: begin
        if (m_bvalid_i) begin
          // Check BRESP ?= OKAY
          if (m_bresp_i != 2'b00) begin
            state_d = S_ERROR;
          end else begin
            state_d = S_NEXT_ROW;
          end
        end
      end

      S_NEXT_ROW: begin
        if (rows_left_q > 32'd1) begin
          state_d = S_CHUNK_PREP;
        end else if (beats_row_left_q == 32'd0) begin
          state_d = S_DONE;
        end else begin
          state_d = S_CHUNK_PREP;
        end
      end

      S_DONE: begin
        irq_pulse_d = 1'b1;
        state_d     = S_IDLE;
      end

      S_ERROR: begin
        irq_pulse_d = 1'b1;
        state_d     = S_IDLE;
      end

      default: state_d = S_RESET;
    endcase
  end

  // ------------------------------
  // Row/chunk bookkeeping & side-effects
  // ------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      // (reset done above)
    end else begin
      // Commit descriptor → initial counters
      if (state_q == S_VALIDATE && state_d == S_PREP_ROW) begin
        beats_row_total_q <= desc_q.row_bytes / BEAT_BYTES;
        beats_row_left_q  <= desc_q.row_bytes / BEAT_BYTES;
        rows_left_q       <= {16'h0, desc_q.rows};

        host_addr_q       <= desc_q.host_addr[AXI_ADDR_W-1:0];
        gb_addr_q         <= desc_q.gb_addr;

        // Clear chunk counters
        chunk_beats_q       <= 32'd0;
        chunk_cmds_issued_q <= 32'd0;
        chunk_beats_ready_q <= 32'd0;
        chunk_beats_sent_q  <= 32'd0;

        err_code_q          <= EC_NONE;
      end

      // CHUNK_PREP: compute chunk_beats
      if (state_q == S_CHUNK_PREP && state_d == S_CHUNK_FILL) begin
        int unsigned beats_left = beats_row_left_q;
        int unsigned hint_beats = (desc_q.burst_bytes_hint == 0) ? 256
                                   : (desc_q.burst_bytes_hint / BEAT_BYTES);
        int unsigned fifo_room  = FIFO_DEPTH - (wf_wptr_q - wf_rptr_q);
        logic [7:0] blen        = calc_burst_len(host_addr_q, beats_left, hint_beats, fifo_room);
        chunk_beats_q           <= blen;
        chunk_cmds_issued_q     <= 32'd0;
        chunk_beats_ready_q     <= 32'd0;
        chunk_beats_sent_q      <= 32'd0;
      end

      // After AW/W/B completion for chunk → advance cursors and counters
      if (state_q == S_WAIT_B && m_bvalid_i && (m_bresp_i == 2'b00)) begin
        // Advance host addr and row counters
        host_addr_q      <= host_addr_q + (chunk_beats_q * BEAT_BYTES);
        beats_row_left_q <= beats_row_left_q - chunk_beats_q;

        // If row done, step to next row
        if (beats_row_left_q == chunk_beats_q) begin
          rows_left_q  <= rows_left_q - 32'd1;
          host_addr_q  <= host_addr_q + desc_q.host_stride[AXI_ADDR_W-1:0];
          gb_addr_q    <= gb_addr_q   + desc_q.gb_stride;
          beats_row_left_q <= beats_row_total_q;
        end

        // Reset chunk counters
        chunk_beats_q       <= 32'd0;
        chunk_cmds_issued_q <= 32'd0;
        chunk_beats_ready_q <= 32'd0;
        chunk_beats_sent_q  <= 32'd0;
      end

      // Error capture (BRESP)
      if ((state_q == S_WAIT_B) && m_bvalid_i && (m_bresp_i != 2'b00)) begin
        err_code_q <= EC_AXI_W;
      end
    end
  end

  // ------------------------------
  // m_wvalid / m_wdata already driven; wlast handled combinationally.
  // GB command accept bookkeeping handled above.
  // ------------------------------

  // ------------------------------
  // Assertions (sim only)
  // ------------------------------
`ifdef ASSERT_ON
  // Alignment (guarded in VALIDATE)
  if (STRICT_ALIGN) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      (state_q == S_VALIDATE && state_d == S_PREP_ROW) |->
       ((desc_q.host_addr % BEAT_BYTES) == 0) &&
       ((desc_q.gb_addr   % BEAT_BYTES) == 0) &&
       ((desc_q.row_bytes % BEAT_BYTES) == 0))
      else $error("dma2d_c: STRICT_ALIGN violation");
  end

  // FIFO safety
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(wf_push && wf_full)) else $error("dma2d_c: W-FIFO overflow");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(wf_pop && wf_empty)) else $error("dma2d_c: W-FIFO underflow");

  // Do not start W without AW accepted
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (state_q == S_W_STREAM) |-> !aw_valid_q)
    else $error("dma2d_c: W stream without AW accept");

  // 4KiB boundary protection implicit in burst planner: optional cover
`endif

  // ------------------------------
  // IRQ pulse
  // ------------------------------
  assign irq_o = irq_pulse_q;

endmodule

`default_nettype wire
