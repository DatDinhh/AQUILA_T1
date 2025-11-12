// ============================================================================
//  DMA2D_A — Activations DMA: AXI4 Read -> GB write
//  File: dma2d_a.sv
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

module dma2d_a #(
  parameter int unsigned AXI_ADDR_W = 48,
  parameter int unsigned AXI_DATA_W = 512,
  parameter int unsigned AXI_ID_W   = 6,
  parameter int unsigned AXI_USER_W = 1,

  // ---------------- Core configuration ----------------
  // Descriptor & doorbell (host memory). Provide real values in FW.
  parameter logic [AXI_ADDR_W-1:0] DESC_ADDR_A     = 48'h0000_0000_8000, // 64B aligned
  parameter logic [AXI_ADDR_W-1:0] DOORBELL_ADDR_A = 48'h0000_0000_9000, // 4B aligned
  parameter int unsigned           POLL_CYCLES     = 4096,               // polling cadence
  parameter int unsigned           MAX_OUTS        = 4,                  // outstanding bursts
  parameter int unsigned           FIFO_DEPTH      = 16,                 // R-beat FIFO
  parameter bit                    STRICT_ALIGN    = 1'b1,               // enforce beat-alignment
  parameter logic [AXI_ID_W-1:0]   AXI_FIXED_ID    = 'h01,               // AXI ID for A DMA
  parameter logic [3:0]            AXI_ARQOS       = 4'h4,               // QoS hint
  parameter logic [3:0]            AXI_ARCACHE     = 4'b1111,            // cacheable, modifiable
  parameter logic [2:0]            AXI_ARPROT      = 3'b000              // data, secure, privileged?
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,

  // ---------- AXI4 Master (Read used; Write held idle) ----------
  // AW channel (unused)
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

  // W channel (unused)
  output logic [AXI_DATA_W-1:0]        m_wdata_o,
  output logic [AXI_DATA_W/8-1:0]      m_wstrb_o,
  output logic                         m_wlast_o,
  output logic [AXI_USER_W-1:0]        m_wuser_o,
  output logic                         m_wvalid_o,
  input  logic                         m_wready_i,

  // B channel (unused -> always ready)
  input  logic [AXI_ID_W-1:0]          m_bid_i,
  input  logic [1:0]                   m_bresp_i,
  input  logic [AXI_USER_W-1:0]        m_buser_i,
  input  logic                         m_bvalid_i,
  output logic                         m_bready_o,

  // AR channel
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

  // R channel
  input  logic [AXI_ID_W-1:0]          m_rid_i,
  input  logic [AXI_DATA_W-1:0]        m_rdata_i,
  input  logic [1:0]                   m_rresp_i,
  input  logic                         m_rlast_i,
  input  logic [AXI_USER_W-1:0]        m_ruser_i,
  input  logic                         m_rvalid_i,
  output logic                         m_rready_o,

  // ---------- Local GB write port ----------
  output logic                         gb_w_req_o,
  output logic [31:0]                  gb_w_addr_o, // byte address into GB
  output logic [AXI_DATA_W/8-1:0]      gb_w_strb_o,
  output logic [AXI_DATA_W-1:0]        gb_w_data_o,
  input  logic                         gb_w_ready_i,

  // ---------- IRQ ----------
  output logic                         irq_o
);

  // Hold AXI write channels idle & safe
  assign m_awid_o    = '0;
  assign m_awaddr_o  = '0;
  assign m_awlen_o   = '0;
  assign m_awsize_o  = '0;
  assign m_awburst_o = 2'b01;
  assign m_awlock_o  = 1'b0;
  assign m_awcache_o = '0;
  assign m_awprot_o  = '0;
  assign m_awqos_o   = '0;
  assign m_awuser_o  = '0;
  assign m_awvalid_o = 1'b0;

  assign m_wdata_o   = '0;
  assign m_wstrb_o   = '0;
  assign m_wlast_o   = 1'b0;
  assign m_wuser_o   = '0;
  assign m_wvalid_o  = 1'b0;

  assign m_bready_o  = 1'b1;

  // Core instance
  dma2d_rd2gb_core #(
    .AXI_ADDR_W     (AXI_ADDR_W),
    .AXI_DATA_W     (AXI_DATA_W),
    .AXI_ID_W       (AXI_ID_W),
    .AXI_USER_W     (AXI_USER_W),

    .DESC_ADDR_DFLT (DESC_ADDR_A),
    .DBELL_ADDR_DFLT(DOORBELL_ADDR_A),
    .POLL_CYCLES    (POLL_CYCLES),
    .MAX_OUTS       (MAX_OUTS),
    .FIFO_DEPTH     (FIFO_DEPTH),
    .STRICT_ALIGN   (STRICT_ALIGN),
    .AXI_FIXED_ID   (AXI_FIXED_ID),
    .AXI_ARQOS      (AXI_ARQOS),
    .AXI_ARCACHE    (AXI_ARCACHE),
    .AXI_ARPROT     (AXI_ARPROT)
  ) u_core (
    .clk_i          (clk_i),
    .rstn_i         (rstn_i),

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

    .gb_w_req_o     (gb_w_req_o),
    .gb_w_addr_o    (gb_w_addr_o),
    .gb_w_strb_o    (gb_w_strb_o),
    .gb_w_data_o    (gb_w_data_o),
    .gb_w_ready_i   (gb_w_ready_i),

    .irq_o          (irq_o)
  );

endmodule
`default_nettype wire
