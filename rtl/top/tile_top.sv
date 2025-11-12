// ============================================================================
//  Aquila-T1 Tile Top
//  File: tile_top.sv
//  Description:
//    Top-level integration for a single accelerator tile:
//      - AXI4 (512b) Master memory interface
//      - AXI4-Lite control plane (bridged to APB CSR space)
//      - 2D systolic array with multi-precision lanes
//      - 32 MiB banked SRAM scratchpad (1R1W) via GB crossbar
//      - A/W feed buses (256 B/cyc each), C drain bus (256 B/cyc)
//      - RISC-V µC, DMAs, post-ops, sparsity/comp, perf/err, NoC router
//      - Separate clock/power/reset islands: ARR / SRAM / CTRL
//      - JTAG, IRQ aggregation, DVFS & thermal hooks
//
//  This is a production-grade integration shell with synthesizable utilities
//  (AXI-Lite->APB bridge, 2:1 APB arbiter, per-domain reset sync). Replace
//  placeholder module instantiations with your RTL as you implement blocks.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tile_top #(
  // ------------------------------
  // Host AXI4 Master parameters
  // ------------------------------
  parameter int unsigned AXI_ADDR_W  = 48,
  parameter int unsigned AXI_DATA_W  = 512,
  parameter int unsigned AXI_ID_W    = 6,
  parameter int unsigned AXI_USER_W  = 1,
  localparam int unsigned AXI_STRB_W = AXI_DATA_W/8,

  // ------------------------------
  // Host AXI-Lite (control) params
  // ------------------------------
  parameter int unsigned AXIL_ADDR_W  = 20,
  parameter int unsigned AXIL_DATA_W  = 32,

  // ------------------------------
  // On-tile NoC parameters
  // ------------------------------
  parameter int unsigned NOC_FLIT_W   = 512,

  // ------------------------------
  // Global Buffer / buses
  // ------------------------------
  parameter int unsigned GB_NUM_BANKS = 16,
  parameter int unsigned GB_DATA_W    = 256,    // per-bank data port width (bits)
  parameter int unsigned GB_STRB_W    = GB_DATA_W/8,
  parameter int unsigned GB_ADDR_W    = 16,     // 2MiB/bank w/ 32B beats => 2^16 words
  parameter int unsigned INJ_BUS_W    = 2048,   // 256 B/cycle = 2048 bits
  parameter int unsigned DRAIN_BUS_W  = 2048,   // 256 B/cycle = 2048 bits

  // ------------------------------
  // Tile configuration
  // ------------------------------
  parameter int unsigned ARRAY_M      = 128,
  parameter int unsigned ARRAY_N      = 128
)(
  // ==========================================================================
  // Clocks & Reset
  // ==========================================================================
  input  logic                       clk_ref,      // reference clock to PLL
  input  logic                       rstn_ref,     // asynchronous, active-low POR

  // ==========================================================================
  // JTAG (DFT / µC debug)
  // ==========================================================================
  input  logic                       jtag_tck,
  input  logic                       jtag_trst_n,
  input  logic                       jtag_tms,
  input  logic                       jtag_tdi,
  output logic                       jtag_tdo,

  // ==========================================================================
  // Host AXI-Lite Control (32-bit)
  // ==========================================================================
  input  logic [AXIL_ADDR_W-1:0]     s_axil_awaddr,
  input  logic [2:0]                 s_axil_awprot,
  input  logic                       s_axil_awvalid,
  output logic                       s_axil_awready,

  input  logic [AXIL_DATA_W-1:0]     s_axil_wdata,
  input  logic [AXIL_DATA_W/8-1:0]   s_axil_wstrb,
  input  logic                       s_axil_wvalid,
  output logic                       s_axil_wready,

  output logic [1:0]                 s_axil_bresp,
  output logic                       s_axil_bvalid,
  input  logic                       s_axil_bready,

  input  logic [AXIL_ADDR_W-1:0]     s_axil_araddr,
  input  logic [2:0]                 s_axil_arprot,
  input  logic                       s_axil_arvalid,
  output logic                       s_axil_arready,

  output logic [AXIL_DATA_W-1:0]     s_axil_rdata,
  output logic [1:0]                 s_axil_rresp,
  output logic                       s_axil_rvalid,
  input  logic                       s_axil_rready,

  // ==========================================================================
  // Host AXI4 Master (512-bit)
  // ==========================================================================
  output logic [AXI_ID_W-1:0]        m_axi_awid,
  output logic [AXI_ADDR_W-1:0]      m_axi_awaddr,
  output logic [7:0]                 m_axi_awlen,
  output logic [2:0]                 m_axi_awsize,
  output logic [1:0]                 m_axi_awburst,
  output logic                       m_axi_awlock,
  output logic [3:0]                 m_axi_awcache,
  output logic [2:0]                 m_axi_awprot,
  output logic [3:0]                 m_axi_awqos,
  output logic [AXI_USER_W-1:0]      m_axi_awuser,
  output logic                       m_axi_awvalid,
  input  logic                       m_axi_awready,

  output logic [AXI_DATA_W-1:0]      m_axi_wdata,
  output logic [AXI_STRB_W-1:0]      m_axi_wstrb,
  output logic                       m_axi_wlast,
  output logic [AXI_USER_W-1:0]      m_axi_wuser,
  output logic                       m_axi_wvalid,
  input  logic                       m_axi_wready,

  input  logic [AXI_ID_W-1:0]        m_axi_bid,
  input  logic [1:0]                 m_axi_bresp,
  input  logic [AXI_USER_W-1:0]      m_axi_buser,
  input  logic                       m_axi_bvalid,
  output logic                       m_axi_bready,

  output logic [AXI_ID_W-1:0]        m_axi_arid,
  output logic [AXI_ADDR_W-1:0]      m_axi_araddr,
  output logic [7:0]                 m_axi_arlen,
  output logic [2:0]                 m_axi_arsize,
  output logic [1:0]                 m_axi_arburst,
  output logic                       m_axi_arlock,
  output logic [3:0]                 m_axi_arcache,
  output logic [2:0]                 m_axi_arprot,
  output logic [3:0]                 m_axi_arqos,
  output logic [AXI_USER_W-1:0]      m_axi_aruser,
  output logic                       m_axi_arvalid,
  input  logic                       m_axi_arready,

  input  logic [AXI_ID_W-1:0]        m_axi_rid,
  input  logic [AXI_DATA_W-1:0]      m_axi_rdata,
  input  logic [1:0]                 m_axi_rresp,
  input  logic                       m_axi_rlast,
  input  logic [AXI_USER_W-1:0]      m_axi_ruser,
  input  logic                       m_axi_rvalid,
  output logic                       m_axi_rready,

  // ==========================================================================
  // On-die Mesh NoC Links (N/E/S/W) — credit/ready-valid modeled as ready/valid
  // ==========================================================================
  // North
  input  logic [NOC_FLIT_W-1:0]      noc_n_rx_flit,
  input  logic                       noc_n_rx_valid,
  output logic                       noc_n_rx_ready,

  output logic [NOC_FLIT_W-1:0]      noc_n_tx_flit,
  output logic                       noc_n_tx_valid,
  input  logic                       noc_n_tx_ready,

  // East
  input  logic [NOC_FLIT_W-1:0]      noc_e_rx_flit,
  input  logic                       noc_e_rx_valid,
  output logic                       noc_e_rx_ready,

  output logic [NOC_FLIT_W-1:0]      noc_e_tx_flit,
  output logic                       noc_e_tx_valid,
  input  logic                       noc_e_tx_ready,

  // South
  input  logic [NOC_FLIT_W-1:0]      noc_s_rx_flit,
  input  logic                       noc_s_rx_valid,
  output logic                       noc_s_rx_ready,

  output logic [NOC_FLIT_W-1:0]      noc_s_tx_flit,
  output logic                       noc_s_tx_valid,
  input  logic                       noc_s_tx_ready,

  // West
  input  logic [NOC_FLIT_W-1:0]      noc_w_rx_flit,
  input  logic                       noc_w_rx_valid,
  output logic                       noc_w_rx_ready,

  output logic [NOC_FLIT_W-1:0]      noc_w_tx_flit,
  output logic                       noc_w_tx_valid,
  input  logic                       noc_w_tx_ready,

  // ==========================================================================
  // Aggregated host interrupt
  // ==========================================================================
  output logic                       tile_irq
);

  // ==========================================================================
  // Derived constants & simple checks
  // ==========================================================================
  initial begin
    if (AXI_DATA_W != 512) $display("INFO(tile_top): AXI_DATA_W=%0d (non-512) – allowed, ensure fabric matches.", AXI_DATA_W);
    if (GB_NUM_BANKS != 16) $display("INFO(tile_top): GB_NUM_BANKS=%0d – allowed, crossbar/CSR must match.", GB_NUM_BANKS);
  end

  // ==========================================================================
  // Clocking & Reset Islands
  // ==========================================================================
  logic clk_arr, clk_sram, clk_ctrl;
  logic pll_locked;

  // Simple PLL wrapper (replace with actual analog macro wrapper)
  pll_wrapper u_pll (
    .clk_ref   (clk_ref),
    .rstn      (rstn_ref),
    .clk_arr   (clk_arr),
    .clk_sram  (clk_sram),
    .clk_ctrl  (clk_ctrl),
    .locked    (pll_locked)
  );

  // Per-domain reset synchronizers (active-low in, active-low out)
  logic rstn_arr, rstn_sram, rstn_ctrl;

  tile_reset_sync u_rst_arr  (.clk(clk_arr),  .rstn_in(rstn_ref & pll_locked),  .rstn_out(rstn_arr));
  tile_reset_sync u_rst_sram (.clk(clk_sram), .rstn_in(rstn_ref & pll_locked),  .rstn_out(rstn_sram));
  tile_reset_sync u_rst_ctrl (.clk(clk_ctrl), .rstn_in(rstn_ref & pll_locked),  .rstn_out(rstn_ctrl));

  // ==========================================================================
  // Control plane: AXI-Lite -> APB bridge + APB arbitration (µC + host)
  // ==========================================================================
  // AXI-Lite -> APB (host master)
  // APB3 signals (host)
  logic [AXIL_ADDR_W-1:0] axil_apb_paddr;
  logic                   axil_apb_psel;
  logic                   axil_apb_penable;
  logic                   axil_apb_pwrite;
  logic [AXIL_DATA_W-1:0] axil_apb_pwdata;
  logic [AXIL_DATA_W-1:0] axil_apb_prdata;
  logic                   axil_apb_pready;
  logic                   axil_apb_pslverr;

  axil2apb #(
    .AXIL_ADDR_W (AXIL_ADDR_W),
    .AXIL_DATA_W (AXIL_DATA_W)
  ) u_axil2apb (
    .clk_i        (clk_ctrl),
    .rstn_i       (rstn_ctrl),

    .s_awaddr_i   (s_axil_awaddr),
    .s_awprot_i   (s_axil_awprot),
    .s_awvalid_i  (s_axil_awvalid),
    .s_awready_o  (s_axil_awready),

    .s_wdata_i    (s_axil_wdata),
    .s_wstrb_i    (s_axil_wstrb),
    .s_wvalid_i   (s_axil_wvalid),
    .s_wready_o   (s_axil_wready),

    .s_bresp_o    (s_axil_bresp),
    .s_bvalid_o   (s_axil_bvalid),
    .s_bready_i   (s_axil_bready),

    .s_araddr_i   (s_axil_araddr),
    .s_arprot_i   (s_axil_arprot),
    .s_arvalid_i  (s_axil_arvalid),
    .s_arready_o  (s_axil_arready),

    .s_rdata_o    (s_axil_rdata),
    .s_rresp_o    (s_axil_rresp),
    .s_rvalid_o   (s_axil_rvalid),
    .s_rready_i   (s_axil_rready),

    .paddr_o      (axil_apb_paddr),
    .psel_o       (axil_apb_psel),
    .penable_o    (axil_apb_penable),
    .pwrite_o     (axil_apb_pwrite),
    .pwdata_o     (axil_apb_pwdata),
    .prdata_i     (axil_apb_prdata),
    .pready_i     (axil_apb_pready),
    .pslverr_i    (axil_apb_pslverr)
  );

  // APB3 signals (µC master)
  logic [AXIL_ADDR_W-1:0] uc_apb_paddr;
  logic                   uc_apb_psel;
  logic                   uc_apb_penable;
  logic                   uc_apb_pwrite;
  logic [AXIL_DATA_W-1:0] uc_apb_pwdata;
  logic [AXIL_DATA_W-1:0] uc_apb_prdata;
  logic                   uc_apb_pready;
  logic                   uc_apb_pslverr;

  // APB3 arbitration -> single CSR APB slave port
  logic [AXIL_ADDR_W-1:0] csr_paddr;
  logic                   csr_psel;
  logic                   csr_penable;
  logic                   csr_pwrite;
  logic [AXIL_DATA_W-1:0] csr_pwdata;
  logic [AXIL_DATA_W-1:0] csr_prdata;
  logic                   csr_pready;
  logic                   csr_pslverr;

  apb_arb2 #(
    .ADDR_W (AXIL_ADDR_W),
    .DATA_W (AXIL_DATA_W)
  ) u_apb_arb2 (
    .clk_i     (clk_ctrl),
    .rstn_i    (rstn_ctrl),

    // Master 0 (priority) = µC
    .m0_paddr_i   (uc_apb_paddr),
    .m0_psel_i    (uc_apb_psel),
    .m0_penable_i (uc_apb_penable),
    .m0_pwrite_i  (uc_apb_pwrite),
    .m0_pwdata_i  (uc_apb_pwdata),
    .m0_prdata_o  (uc_apb_prdata),
    .m0_pready_o  (uc_apb_pready),
    .m0_pslverr_o (uc_apb_pslverr),

    // Master 1 = Host (AXI-Lite bridge)
    .m1_paddr_i   (axil_apb_paddr),
    .m1_psel_i    (axil_apb_psel),
    .m1_penable_i (axil_apb_penable),
    .m1_pwrite_i  (axil_apb_pwrite),
    .m1_pwdata_i  (axil_apb_pwdata),
    .m1_prdata_o  (axil_apb_prdata),
    .m1_pready_o  (axil_apb_pready),
    .m1_pslverr_o (axil_apb_pslverr),

    // Slave (to CSR block)
    .s_paddr_o    (csr_paddr),
    .s_psel_o     (csr_psel),
    .s_penable_o  (csr_penable),
    .s_pwrite_o   (csr_pwrite),
    .s_pwdata_o   (csr_pwdata),
    .s_prdata_i   (csr_prdata),
    .s_pready_i   (csr_pready),
    .s_pslverr_i  (csr_pslverr)
  );

  // ==========================================================================
  // CSR block (APB3 slave) — generates config/control for the tile
  //   NOTE: Replace with your autogen CSR from HJSON/RAL flow.
  // ==========================================================================
  // Selected control signals (expand as you define your CSR map)
  logic        csr_array_enable;
  logic [2:0]  csr_array_mode;         // dtype/lanes selection
  logic        csr_sparsity_2of4_en;
  logic [GB_NUM_BANKS-1:0] csr_part_a_mask, csr_part_w_mask, csr_part_c_mask;
  logic        csr_doorbell;           // kick command queue
  logic [7:0]  csr_irq_mask;

  csr_block #(
    .APB_ADDR_W (AXIL_ADDR_W),
    .APB_DATA_W (AXIL_DATA_W),
    .GB_BANKS   (GB_NUM_BANKS)
  ) u_csr (
    .clk_i        (clk_ctrl),
    .rstn_i       (rstn_ctrl),

    .paddr_i      (csr_paddr),
    .psel_i       (csr_psel),
    .penable_i    (csr_penable),
    .pwrite_i     (csr_pwrite),
    .pwdata_i     (csr_pwdata),
    .prdata_o     (csr_prdata),
    .pready_o     (csr_pready),
    .pslverr_o    (csr_pslverr),

    // Control outputs to subsystems
    .array_enable_o       (csr_array_enable),
    .array_mode_o         (csr_array_mode),
    .sparsity_2of4_en_o   (csr_sparsity_2of4_en),
    .part_a_mask_o        (csr_part_a_mask),
    .part_w_mask_o        (csr_part_w_mask),
    .part_c_mask_o        (csr_part_c_mask),
    .doorbell_o           (csr_doorbell),
    .irq_mask_o           (csr_irq_mask)
  );

  // ==========================================================================
  // RISC-V µC Subsystem (APB master for CSR, IRQs, JTAG)
  // ==========================================================================
  logic [31:0]  uc_irq_lines;
  logic         uc_wdog_irq;
  assign uc_irq_lines = '0; // expand and map real interrupts as you add blocks

  riscv_subsys u_uc (
    .clk_i        (clk_ctrl),
    .rstn_i       (rstn_ctrl),

    // APB master out (to CSR via arbiter)
    .paddr_o      (uc_apb_paddr),
    .psel_o       (uc_apb_psel),
    .penable_o    (uc_apb_penable),
    .pwrite_o     (uc_apb_pwrite),
    .pwdata_o     (uc_apb_pwdata),
    .prdata_i     (uc_apb_prdata),
    .pready_i     (uc_apb_pready),
    .pslverr_i    (uc_apb_pslverr),

    // JTAG (debug)
    .jtag_tck_i   (jtag_tck),
    .jtag_trst_ni (jtag_trst_n),
    .jtag_tms_i   (jtag_tms),
    .jtag_tdi_i   (jtag_tdi),
    .jtag_tdo_o   (jtag_tdo),

    // Interrupts in / out
    .intr_i       (uc_irq_lines),
    .wdog_irq_o   (uc_wdog_irq)
  );

  // ==========================================================================
  // AXI Master arbitration (DMAs -> single 512b AXI master port)
  // ==========================================================================
  // AXI master ports from A/W/C DMAs (aggregate inside arbiter)
  //   Declare channel wires as packed structs or flat; we use flat for clarity.
  //   NOTE: Replace tile_axi_arb with your crossbar/arbiter fabric.
  //
  //   dma_aw*, dma_w*, dma_b*, dma_ar*, dma_r* are arrays [3] for A/W/C
  // ==========================================================================
  localparam int DMA_NUM_M = 3;

  // AW
  logic [DMA_NUM_M-1:0][AXI_ID_W-1:0]    dma_awid;
  logic [DMA_NUM_M-1:0][AXI_ADDR_W-1:0]  dma_awaddr;
  logic [DMA_NUM_M-1:0][7:0]             dma_awlen;
  logic [DMA_NUM_M-1:0][2:0]             dma_awsize;
  logic [DMA_NUM_M-1:0][1:0]             dma_awburst;
  logic [DMA_NUM_M-1:0]                  dma_awlock;
  logic [DMA_NUM_M-1:0][3:0]             dma_awcache;
  logic [DMA_NUM_M-1:0][2:0]             dma_awprot;
  logic [DMA_NUM_M-1:0][3:0]             dma_awqos;
  logic [DMA_NUM_M-1:0][AXI_USER_W-1:0]  dma_awuser;
  logic [DMA_NUM_M-1:0]                  dma_awvalid;
  logic [DMA_NUM_M-1:0]                  dma_awready;

  // W
  logic [DMA_NUM_M-1:0][AXI_DATA_W-1:0]  dma_wdata;
  logic [DMA_NUM_M-1:0][AXI_STRB_W-1:0]  dma_wstrb;
  logic [DMA_NUM_M-1:0]                  dma_wlast;
  logic [DMA_NUM_M-1:0][AXI_USER_W-1:0]  dma_wuser;
  logic [DMA_NUM_M-1:0]                  dma_wvalid;
  logic [DMA_NUM_M-1:0]                  dma_wready;

  // B
  logic [DMA_NUM_M-1:0][AXI_ID_W-1:0]    dma_bid;
  logic [DMA_NUM_M-1:0][1:0]             dma_bresp;
  logic [DMA_NUM_M-1:0][AXI_USER_W-1:0]  dma_buser;
  logic [DMA_NUM_M-1:0]                  dma_bvalid;
  logic [DMA_NUM_M-1:0]                  dma_bready;

  // AR
  logic [DMA_NUM_M-1:0][AXI_ID_W-1:0]    dma_arid;
  logic [DMA_NUM_M-1:0][AXI_ADDR_W-1:0]  dma_araddr;
  logic [DMA_NUM_M-1:0][7:0]             dma_arlen;
  logic [DMA_NUM_M-1:0][2:0]             dma_arsize;
  logic [DMA_NUM_M-1:0][1:0]             dma_arburst;
  logic [DMA_NUM_M-1:0]                  dma_arlock;
  logic [DMA_NUM_M-1:0][3:0]             dma_arcache;
  logic [DMA_NUM_M-1:0][2:0]             dma_arprot;
  logic [DMA_NUM_M-1:0][3:0]             dma_arqos;
  logic [DMA_NUM_M-1:0][AXI_USER_W-1:0]  dma_aruser;
  logic [DMA_NUM_M-1:0]                  dma_arvalid;
  logic [DMA_NUM_M-1:0]                  dma_arready;

  // R
  logic [DMA_NUM_M-1:0][AXI_ID_W-1:0]    dma_rid;
  logic [DMA_NUM_M-1:0][AXI_DATA_W-1:0]  dma_rdata;
  logic [DMA_NUM_M-1:0][1:0]             dma_rresp;
  logic [DMA_NUM_M-1:0]                  dma_rlast;
  logic [DMA_NUM_M-1:0][AXI_USER_W-1:0]  dma_ruser;
  logic [DMA_NUM_M-1:0]                  dma_rvalid;
  logic [DMA_NUM_M-1:0]                  dma_rready;

  // IRQs from DMAs
  logic irq_dma_a, irq_dma_w, irq_dma_c;

  // DMAs (A, W, C) — replace with your RTL
  dma2d_a #(
    .AXI_ADDR_W (AXI_ADDR_W), .AXI_DATA_W (AXI_DATA_W), .AXI_ID_W (AXI_ID_W), .AXI_USER_W (AXI_USER_W)
  ) u_dma_a (
    .clk_i       (clk_ctrl), .rstn_i (rstn_ctrl),
    // AXI master out
    .m_awid_o    (dma_awid  [0]), .m_awaddr_o (dma_awaddr[0]), .m_awlen_o (dma_awlen[0]),
    .m_awsize_o  (dma_awsize[0]), .m_awburst_o(dma_awburst[0]), .m_awlock_o(dma_awlock[0]),
    .m_awcache_o (dma_awcache[0]),.m_awprot_o (dma_awprot[0]),  .m_awqos_o (dma_awqos[0]),
    .m_awuser_o  (dma_awuser[0]), .m_awvalid_o(dma_awvalid[0]), .m_awready_i(dma_awready[0]),

    .m_wdata_o   (dma_wdata[0]), .m_wstrb_o (dma_wstrb[0]), .m_wlast_o (dma_wlast[0]),
    .m_wuser_o   (dma_wuser[0]), .m_wvalid_o(dma_wvalid[0]), .m_wready_i(dma_wready[0]),

    .m_bid_i     (dma_bid[0]), .m_bresp_i (dma_bresp[0]), .m_buser_i (dma_buser[0]),
    .m_bvalid_i  (dma_bvalid[0]), .m_bready_o(dma_bready[0]),

    .m_arid_o    (dma_arid[0]), .m_araddr_o(dma_araddr[0]), .m_arlen_o (dma_arlen[0]),
    .m_arsize_o  (dma_arsize[0]), .m_arburst_o(dma_arburst[0]), .m_arlock_o(dma_arlock[0]),
    .m_arcache_o (dma_arcache[0]), .m_arprot_o(dma_arprot[0]),  .m_arqos_o (dma_arqos[0]),
    .m_aruser_o  (dma_aruser[0]), .m_arvalid_o(dma_arvalid[0]), .m_arready_i(dma_arready[0]),

    .m_rid_i     (dma_rid[0]), .m_rdata_i (dma_rdata[0]), .m_rresp_i (dma_rresp[0]),
    .m_rlast_i   (dma_rlast[0]), .m_ruser_i(dma_ruser[0]), .m_rvalid_i(dma_rvalid[0]),
    .m_rready_o  (dma_rready[0]),

    // Local SRAM write ports (to GB crossbar) — connect below
    .gb_w_req_o  (), .gb_w_addr_o (), .gb_w_strb_o (), .gb_w_data_o (), .gb_w_ready_i (1'b1),

    .irq_o       (irq_dma_a)
  );

  dma2d_w #(
    .AXI_ADDR_W (AXI_ADDR_W), .AXI_DATA_W (AXI_DATA_W), .AXI_ID_W (AXI_ID_W), .AXI_USER_W (AXI_USER_W)
  ) u_dma_w (
    .clk_i       (clk_ctrl), .rstn_i (rstn_ctrl),
    // AXI
    .m_awid_o    (dma_awid  [1]), .m_awaddr_o (dma_awaddr[1]), .m_awlen_o (dma_awlen[1]),
    .m_awsize_o  (dma_awsize[1]), .m_awburst_o(dma_awburst[1]), .m_awlock_o(dma_awlock[1]),
    .m_awcache_o (dma_awcache[1]),.m_awprot_o (dma_awprot[1]),  .m_awqos_o (dma_awqos[1]),
    .m_awuser_o  (dma_awuser[1]), .m_awvalid_o(dma_awvalid[1]), .m_awready_i(dma_awready[1]),

    .m_wdata_o   (dma_wdata[1]), .m_wstrb_o (dma_wstrb[1]), .m_wlast_o (dma_wlast[1]),
    .m_wuser_o   (dma_wuser[1]), .m_wvalid_o(dma_wvalid[1]), .m_wready_i(dma_wready[1]),

    .m_bid_i     (dma_bid[1]), .m_bresp_i (dma_bresp[1]), .m_buser_i (dma_buser[1]),
    .m_bvalid_i  (dma_bvalid[1]), .m_bready_o(dma_bready[1]),

    .m_arid_o    (dma_arid[1]), .m_araddr_o(dma_araddr[1]), .m_arlen_o (dma_arlen[1]),
    .m_arsize_o  (dma_arsize[1]), .m_arburst_o(dma_arburst[1]), .m_arlock_o(dma_arlock[1]),
    .m_arcache_o (dma_arcache[1]), .m_arprot_o(dma_arprot[1]),  .m_arqos_o (dma_arqos[1]),
    .m_aruser_o  (dma_aruser[1]), .m_arvalid_o(dma_arvalid[1]), .m_arready_i(dma_arready[1]),

    .m_rid_i     (dma_rid[1]), .m_rdata_i (dma_rdata[1]), .m_rresp_i (dma_rresp[1]),
    .m_rlast_i   (dma_rlast[1]), .m_ruser_i(dma_ruser[1]), .m_rvalid_i(dma_rvalid[1]),
    .m_rready_o  (dma_rready[1]),

    // Local SRAM write ports (to GB crossbar)
    .gb_w_req_o  (), .gb_w_addr_o (), .gb_w_strb_o (), .gb_w_data_o (), .gb_w_ready_i (1'b1),

    .irq_o       (irq_dma_w)
  );

  dma2d_c #(
    .AXI_ADDR_W (AXI_ADDR_W), .AXI_DATA_W (AXI_DATA_W), .AXI_ID_W (AXI_ID_W), .AXI_USER_W (AXI_USER_W)
  ) u_dma_c (
    .clk_i       (clk_ctrl), .rstn_i (rstn_ctrl),
    // AXI
    .m_awid_o    (dma_awid  [2]), .m_awaddr_o (dma_awaddr[2]), .m_awlen_o (dma_awlen[2]),
    .m_awsize_o  (dma_awsize[2]), .m_awburst_o(dma_awburst[2]), .m_awlock_o(dma_awlock[2]),
    .m_awcache_o (dma_awcache[2]),.m_awprot_o (dma_awprot[2]),  .m_awqos_o (dma_awqos[2]),
    .m_awuser_o  (dma_awuser[2]), .m_awvalid_o(dma_awvalid[2]), .m_awready_i(dma_awready[2]),

    .m_wdata_o   (dma_wdata[2]), .m_wstrb_o (dma_wstrb[2]), .m_wlast_o (dma_wlast[2]),
    .m_wuser_o   (dma_wuser[2]), .m_wvalid_o(dma_wvalid[2]), .m_wready_i(dma_wready[2]),

    .m_bid_i     (dma_bid[2]), .m_bresp_i (dma_bresp[2]), .m_buser_i (dma_buser[2]),
    .m_bvalid_i  (dma_bvalid[2]), .m_bready_o(dma_bready[2]),

    .m_arid_o    (dma_arid[2]), .m_araddr_o(dma_araddr[2]), .m_arlen_o (dma_arlen[2]),
    .m_arsize_o  (dma_arsize[2]), .m_arburst_o(dma_arburst[2]), .m_arlock_o(dma_arlock[2]),
    .m_arcache_o (dma_arcache[2]), .m_arprot_o(dma_arprot[2]),  .m_arqos_o (dma_arqos[2]),
    .m_aruser_o  (dma_aruser[2]), .m_arvalid_o(dma_arvalid[2]), .m_arready_i(dma_arready[2]),

    .m_rid_i     (dma_rid[2]), .m_rdata_i (dma_rdata[2]), .m_rresp_i (dma_rresp[2]),
    .m_rlast_i   (dma_rlast[2]), .m_ruser_i(dma_ruser[2]), .m_rvalid_i(dma_rvalid[2]),
    .m_rready_o  (dma_rready[2]),

    // Local SRAM read ports (from GB crossbar) — connect below
    .gb_r_req_o  (), .gb_r_addr_o (), .gb_r_data_i (), .gb_r_valid_i (1'b0), .gb_r_ready_o (),

    .irq_o       (irq_dma_c)
  );

  // AXI arbiter to a single M_AXI — placeholder fabric instantiation
  tile_axi_arb_3to1 #(
    .AXI_ADDR_W (AXI_ADDR_W), .AXI_DATA_W (AXI_DATA_W), .AXI_ID_W (AXI_ID_W), .AXI_USER_W (AXI_USER_W)
  ) u_axi_arb (
    .clk_i         (clk_ctrl),
    .rstn_i        (rstn_ctrl),

    // Upstream masters (DMAs 0=A, 1=W, 2=C)
    .m_awid_i      (dma_awid),
    .m_awaddr_i    (dma_awaddr),
    .m_awlen_i     (dma_awlen),
    .m_awsize_i    (dma_awsize),
    .m_awburst_i   (dma_awburst),
    .m_awlock_i    (dma_awlock),
    .m_awcache_i   (dma_awcache),
    .m_awprot_i    (dma_awprot),
    .m_awqos_i     (dma_awqos),
    .m_awuser_i    (dma_awuser),
    .m_awvalid_i   (dma_awvalid),
    .m_awready_o   (dma_awready),

    .m_wdata_i     (dma_wdata),
    .m_wstrb_i     (dma_wstrb),
    .m_wlast_i     (dma_wlast),
    .m_wuser_i     (dma_wuser),
    .m_wvalid_i    (dma_wvalid),
    .m_wready_o    (dma_wready),

    .m_bid_o       (dma_bid),
    .m_bresp_o     (dma_bresp),
    .m_buser_o     (dma_buser),
    .m_bvalid_o    (dma_bvalid),
    .m_bready_i    (dma_bready),

    .m_arid_i      (dma_arid),
    .m_araddr_i    (dma_araddr),
    .m_arlen_i     (dma_arlen),
    .m_arsize_i    (dma_arsize),
    .m_arburst_i   (dma_arburst),
    .m_arlock_i    (dma_arlock),
    .m_arcache_i   (dma_arcache),
    .m_arprot_i    (dma_arprot),
    .m_arqos_i     (dma_arqos),
    .m_aruser_i    (dma_aruser),
    .m_arvalid_i   (dma_arvalid),
    .m_arready_o   (dma_arready),

    .m_rid_o       (dma_rid),
    .m_rdata_o     (dma_rdata),
    .m_rresp_o     (dma_rresp),
    .m_rlast_o     (dma_rlast),
    .m_ruser_o     (dma_ruser),
    .m_rvalid_o    (dma_rvalid),
    .m_rready_i    (dma_rready),

    // Downstream single M_AXI
    .s_awid_o      (m_axi_awid),
    .s_awaddr_o    (m_axi_awaddr),
    .s_awlen_o     (m_axi_awlen),
    .s_awsize_o    (m_axi_awsize),
    .s_awburst_o   (m_axi_awburst),
    .s_awlock_o    (m_axi_awlock),
    .s_awcache_o   (m_axi_awcache),
    .s_awprot_o    (m_axi_awprot),
    .s_awqos_o     (m_axi_awqos),
    .s_awuser_o    (m_axi_awuser),
    .s_awvalid_o   (m_axi_awvalid),
    .s_awready_i   (m_axi_awready),

    .s_wdata_o     (m_axi_wdata),
    .s_wstrb_o     (m_axi_wstrb),
    .s_wlast_o     (m_axi_wlast),
    .s_wuser_o     (m_axi_wuser),
    .s_wvalid_o    (m_axi_wvalid),
    .s_wready_i    (m_axi_wready),

    .s_bid_i       (m_axi_bid),
    .s_bresp_i     (m_axi_bresp),
    .s_buser_i     (m_axi_buser),
    .s_bvalid_i    (m_axi_bvalid),
    .s_bready_o    (m_axi_bready),

    .s_arid_o      (m_axi_arid),
    .s_araddr_o    (m_axi_araddr),
    .s_arlen_o     (m_axi_arlen),
    .s_arsize_o    (m_axi_arsize),
    .s_arburst_o   (m_axi_arburst),
    .s_arlock_o    (m_axi_arlock),
    .s_arcache_o   (m_axi_arcache),
    .s_arprot_o    (m_axi_arprot),
    .s_arqos_o     (m_axi_arqos),
    .s_aruser_o    (m_axi_aruser),
    .s_arvalid_o   (m_axi_arvalid),
    .s_arready_i   (m_axi_arready),

    .s_rid_i       (m_axi_rid),
    .s_rdata_i     (m_axi_rdata),
    .s_rresp_i     (m_axi_rresp),
    .s_rlast_i     (m_axi_rlast),
    .s_ruser_i     (m_axi_ruser),
    .s_rvalid_i    (m_axi_rvalid),
    .s_rready_o    (m_axi_rready)
  );

  // ==========================================================================
  // Global Buffer crossbar + SRAM banks
  //   - Provide streaming injection buses to the array
  //   - Accept drain stream from post-ops
  //   - Bank partitioning A/W/C per CSR masks
  // ==========================================================================
  logic [INJ_BUS_W-1:0]  inj_a_data, inj_w_data;
  logic                  inj_a_valid, inj_a_ready;
  logic                  inj_w_valid, inj_w_ready;

  logic [DRAIN_BUS_W-1:0] drain_c_data;
  logic                   drain_c_valid, drain_c_ready;

  gb_crossbar #(
    .NUM_BANKS   (GB_NUM_BANKS),
    .DATA_W      (GB_DATA_W),
    .ADDR_W      (GB_ADDR_W),
    .INJ_BUS_W   (INJ_BUS_W),
    .DRAIN_BUS_W (DRAIN_BUS_W)
  ) u_gb_xbar (
    .clk_i          (clk_sram),
    .rstn_i         (rstn_sram),

    // Partition controls
    .part_a_mask_i  (csr_part_a_mask),
    .part_w_mask_i  (csr_part_w_mask),
    .part_c_mask_i  (csr_part_c_mask),

    // DMA write/read side (A/W fill, C drain by DMAC) — connect your DMA gb_* ports here
    .dma_w_req_i    ('0), .dma_w_addr_i ('0), .dma_w_strb_i ('0), .dma_w_data_i ('0), .dma_w_ready_o (),
    .dma_r_req_i    ('0), .dma_r_addr_i ('0), .dma_r_data_o (),  .dma_r_valid_o (),  .dma_r_ready_i (1'b1),

    // Injection buses to array
    .inj_a_data_o   (inj_a_data),
    .inj_a_valid_o  (inj_a_valid),
    .inj_a_ready_i  (inj_a_ready),

    .inj_w_data_o   (inj_w_data),
    .inj_w_valid_o  (inj_w_valid),
    .inj_w_ready_i  (inj_w_ready),

    // Drain bus from post-ops
    .drain_c_data_i (drain_c_data),
    .drain_c_valid_i(drain_c_valid),
    .drain_c_ready_o(drain_c_ready)

    // Internally instantiate sram_bank_wrapper[NUM_BANKS] w/ ECC as you implement
  );

  // ==========================================================================
  // Systolic Array + Post-Ops
  // ==========================================================================
  logic [DRAIN_BUS_W-1:0] array_c_data;
  logic                   array_c_valid, array_c_ready;

  systolic_array_top #(
    .M           (ARRAY_M),
    .N           (ARRAY_N),
    .A_BUS_W     (INJ_BUS_W),
    .W_BUS_W     (INJ_BUS_W),
    .C_BUS_W     (DRAIN_BUS_W)
  ) u_array (
    .clk_i       (clk_arr),
    .rstn_i      (rstn_arr),

    .enable_i    (csr_array_enable),
    .mode_i      (csr_array_mode),
    .sparsity2of4_en_i (csr_sparsity_2of4_en),

    // Feed buses from GB crossbar
    .a_data_i    (inj_a_data),
    .a_valid_i   (inj_a_valid),
    .a_ready_o   (inj_a_ready),

    .w_data_i    (inj_w_data),
    .w_valid_i   (inj_w_valid),
    .w_ready_o   (inj_w_ready),

    // Output partials/results to Post-Ops
    .c_data_o    (array_c_data),
    .c_valid_o   (array_c_valid),
    .c_ready_i   (array_c_ready)
  );

  post_ops_top #(
    .BUS_W       (DRAIN_BUS_W)
  ) u_postops (
    .clk_i       (clk_arr),
    .rstn_i      (rstn_arr),

    // Config registers would be sourced from CSR (bias addr, act type, scales, etc.)
    .cfg_valid_i (1'b0),
    .cfg_payload_i ('0),

    // From array
    .in_data_i   (array_c_data),
    .in_valid_i  (array_c_valid),
    .in_ready_o  (array_c_ready),

    // To GB crossbar (C drain bus)
    .out_data_o  (drain_c_data),
    .out_valid_o (drain_c_valid),
    .out_ready_i (drain_c_ready)
  );

  // ==========================================================================
  // Performance counters, error logger (placeholders)
  // ==========================================================================
  logic irq_perf, irq_err, irq_ecc, irq_thermal;

  perf_counters u_perf (
    .clk_i       (clk_ctrl), .rstn_i (rstn_ctrl),
    // tap array/gb/dma handshakes as you wire real signals
    .irq_o       (irq_perf)
  );

  err_logger u_errlog (
    .clk_i       (clk_ctrl), .rstn_i (rstn_ctrl),
    .irq_o       (irq_err)
  );

  // ==========================================================================
  // NoC Router (5-port) — external links N/E/S/W, local unused for now
  // ==========================================================================
  logic [NOC_FLIT_W-1:0] noc_l_rx_flit, noc_l_tx_flit;
  logic                  noc_l_rx_valid, noc_l_rx_ready;
  logic                  noc_l_tx_valid, noc_l_tx_ready;

  assign noc_l_rx_flit  = '0;
  assign noc_l_rx_valid = 1'b0;
  assign noc_l_tx_ready = 1'b1;

  noc_router_5p #(
    .FLIT_W (NOC_FLIT_W)
  ) u_noc (
    .clk_i       (clk_ctrl),
    .rstn_i      (rstn_ctrl),

    // North
    .n_rx_flit_i (noc_n_rx_flit), .n_rx_valid_i(noc_n_rx_valid), .n_rx_ready_o(noc_n_rx_ready),
    .n_tx_flit_o (noc_n_tx_flit), .n_tx_valid_o(noc_n_tx_valid), .n_tx_ready_i(noc_n_tx_ready),

    // East
    .e_rx_flit_i (noc_e_rx_flit), .e_rx_valid_i(noc_e_rx_valid), .e_rx_ready_o(noc_e_rx_ready),
    .e_tx_flit_o (noc_e_tx_flit), .e_tx_valid_o(noc_e_tx_valid), .e_tx_ready_i(noc_e_tx_ready),

    // South
    .s_rx_flit_i (noc_s_rx_flit), .s_rx_valid_i(noc_s_rx_valid), .s_rx_ready_o(noc_s_rx_ready),
    .s_tx_flit_o (noc_s_tx_flit), .s_tx_valid_o(noc_s_tx_valid), .s_tx_ready_i(noc_s_tx_ready),

    // West
    .w_rx_flit_i (noc_w_rx_flit), .w_rx_valid_i(noc_w_rx_valid), .w_rx_ready_o(noc_w_rx_ready),
    .w_tx_flit_o (noc_w_tx_flit), .w_tx_valid_o(noc_w_tx_valid), .w_tx_ready_i(noc_w_tx_ready),

    // Local
    .l_rx_flit_i (noc_l_rx_flit), .l_rx_valid_i(noc_l_rx_valid), .l_rx_ready_o(noc_l_rx_ready),
    .l_tx_flit_o (noc_l_tx_flit), .l_tx_valid_o(noc_l_tx_valid), .l_tx_ready_i(noc_l_tx_ready)
  );

  // ==========================================================================
  // Thermal / DVFS hooks (placeholders)
  // ==========================================================================
  dvfs_ctrl u_dvfs (
    .clk_i       (clk_ctrl),
    .rstn_i      (rstn_ctrl),
    .throttle_o  (),     // connect to array clock-gates in implementation
    .irq_o       (irq_thermal)
  );

  // ==========================================================================
  // IRQ aggregation to host
  // ==========================================================================
  assign tile_irq = (irq_dma_a | irq_dma_w | irq_dma_c | irq_perf | irq_err | irq_ecc | irq_thermal | uc_wdog_irq) & (|csr_irq_mask);

  // ==========================================================================
  // Assertions / simple protocol sanity (simulation-only)
  // ==========================================================================
`ifdef ASSERT_ON
  // AXI-Lite is single beat in this design; bridge ensures that. No additional SVAs here.
`endif

endmodule


// ============================================================================
//  Reset synchronizer (active-low)
// ============================================================================
module tile_reset_sync (
  input  logic clk,
  input  logic rstn_in,
  output logic rstn_out
);
  logic d1, d2;
  always_ff @(posedge clk or negedge rstn_in) begin
    if (!rstn_in) begin
      d1 <= 1'b0;
      d2 <= 1'b0;
    end else begin
      d1 <= 1'b1;
      d2 <= d1;
    end
  end
  assign rstn_out = d2;
endmodule


// ============================================================================
//  AXI-Lite (single-beat) to APB3 bridge (synthesizable)
// ============================================================================
module axil2apb #(
  parameter int unsigned AXIL_ADDR_W = 20,
  parameter int unsigned AXIL_DATA_W = 32
)(
  input  logic                      clk_i,
  input  logic                      rstn_i,

  // AXI-Lite slave
  input  logic [AXIL_ADDR_W-1:0]    s_awaddr_i,
  input  logic [2:0]                s_awprot_i,
  input  logic                      s_awvalid_i,
  output logic                      s_awready_o,

  input  logic [AXIL_DATA_W-1:0]    s_wdata_i,
  input  logic [AXIL_DATA_W/8-1:0]  s_wstrb_i,
  input  logic                      s_wvalid_i,
  output logic                      s_wready_o,

  output logic [1:0]                s_bresp_o,
  output logic                      s_bvalid_o,
  input  logic                      s_bready_i,

  input  logic [AXIL_ADDR_W-1:0]    s_araddr_i,
  input  logic [2:0]                s_arprot_i,
  input  logic                      s_arvalid_i,
  output logic                      s_arready_o,

  output logic [AXIL_DATA_W-1:0]    s_rdata_o,
  output logic [1:0]                s_rresp_o,
  output logic                      s_rvalid_o,
  input  logic                      s_rready_i,

  // APB3 master out
  output logic [AXIL_ADDR_W-1:0]    paddr_o,
  output logic                      psel_o,
  output logic                      penable_o,
  output logic                      pwrite_o,
  output logic [AXIL_DATA_W-1:0]    pwdata_o,
  input  logic [AXIL_DATA_W-1:0]    prdata_i,
  input  logic                      pready_i,
  input  logic                      pslverr_i
);
  typedef enum logic [1:0] {W_IDLE, W_CAPT, W_APB} wstate_e;
  typedef enum logic [1:0] {R_IDLE, R_APB, R_RESP} rstate_e;

  wstate_e wstate, wstate_n;
  rstate_e rstate, rstate_n;

  logic [AXIL_ADDR_W-1:0] awaddr_q;
  logic [AXIL_DATA_W-1:0] wdata_q;

  // Default AXI responses
  assign s_bresp_o = 2'b00; // OKAY
  assign s_rresp_o = 2'b00; // OKAY

  // Write FSM
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wstate   <= W_IDLE;
      awaddr_q <= '0;
      wdata_q  <= '0;
    end else begin
      wstate <= wstate_n;
      if (s_awvalid_i && s_awready_o) awaddr_q <= s_awaddr_i;
      if (s_wvalid_i  && s_wready_o)  wdata_q  <= s_wdata_i;
    end
  end

  always_comb begin
    // defaults
    s_awready_o = 1'b0;
    s_wready_o  = 1'b0;
    s_bvalid_o  = 1'b0;

    paddr_o     = awaddr_q;
    psel_o      = 1'b0;
    penable_o   = 1'b0;
    pwrite_o    = 1'b0;
    pwdata_o    = wdata_q;

    wstate_n    = wstate;

    unique case (wstate)
      W_IDLE: begin
        s_awready_o = 1'b1;
        s_wready_o  = 1'b1;
        if (s_awvalid_i && s_wvalid_i) begin
          wstate_n = W_APB;
        end else begin
          wstate_n = W_IDLE;
        end
      end

      W_APB: begin
        // APB write cycle: SETUP -> ENABLE (single-cycle enable assumed if pready_i)
        psel_o    = 1'b1;
        pwrite_o  = 1'b1;
        penable_o = 1'b1;
        if (pready_i) begin
          wstate_n  = W_IDLE;
          s_bvalid_o= 1'b1;
        end
      end

      default: wstate_n = W_IDLE;
    endcase
  end

  // Read FSM
  logic [AXIL_ADDR_W-1:0] araddr_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rstate   <= R_IDLE;
      araddr_q <= '0;
    end else begin
      rstate <= rstate_n;
      if (s_arvalid_i && s_arready_o) araddr_q <= s_araddr_i;
    end
  end

  always_comb begin
    // defaults
    s_arready_o = 1'b0;
    s_rvalid_o  = 1'b0;
    s_rdata_o   = prdata_i;

    // APB
    // For read, reuse paddr/psel/penable; ensure write path not active concurrently.
    if (wstate != W_APB) begin
      paddr_o   = araddr_q;
    end

    rstate_n = rstate;

    unique case (rstate)
      R_IDLE: begin
        s_arready_o = (wstate != W_APB); // block if APB is busy with write
        if (s_arvalid_i && s_arready_o) begin
          rstate_n = R_APB;
        end
      end

      R_APB: begin
        // APB read cycle
        psel_o    = 1'b1;
        pwrite_o  = 1'b0;
        penable_o = 1'b1;
        if (pready_i) begin
          rstate_n = R_RESP;
        end
      end

      R_RESP: begin
        s_rvalid_o = 1'b1;
        if (s_rready_i) begin
          rstate_n = R_IDLE;
        end
      end

      default: rstate_n = R_IDLE;
    endcase
  end

endmodule


// ============================================================================
//  APB3 2-to-1 Arbiter (M0 priority) — synthesizable
// ============================================================================
module apb_arb2 #(
  parameter int unsigned ADDR_W = 20,
  parameter int unsigned DATA_W = 32
)(
  input  logic                 clk_i,
  input  logic                 rstn_i,

  // Master 0 (priority)
  input  logic [ADDR_W-1:0]    m0_paddr_i,
  input  logic                 m0_psel_i,
  input  logic                 m0_penable_i,
  input  logic                 m0_pwrite_i,
  input  logic [DATA_W-1:0]    m0_pwdata_i,
  output logic [DATA_W-1:0]    m0_prdata_o,
  output logic                 m0_pready_o,
  output logic                 m0_pslverr_o,

  // Master 1
  input  logic [ADDR_W-1:0]    m1_paddr_i,
  input  logic                 m1_psel_i,
  input  logic                 m1_penable_i,
  input  logic                 m1_pwrite_i,
  input  logic [DATA_W-1:0]    m1_pwdata_i,
  output logic [DATA_W-1:0]    m1_prdata_o,
  output logic                 m1_pready_o,
  output logic                 m1_pslverr_o,

  // Slave out
  output logic [ADDR_W-1:0]    s_paddr_o,
  output logic                 s_psel_o,
  output logic                 s_penable_o,
  output logic                 s_pwrite_o,
  output logic [DATA_W-1:0]    s_pwdata_o,
  input  logic [DATA_W-1:0]    s_prdata_i,
  input  logic                 s_pready_i,
  input  logic                 s_pslverr_i
);

  typedef enum logic [0:0] {SEL_M0, SEL_M1} sel_e;
  sel_e sel_q, sel_d;

  // Simple arbitration: if M0 asserts PSEL, it owns the bus until pready; else M1.
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) sel_q <= SEL_M1; // default prefer M1 if M0 idle
    else         sel_q <= sel_d;
  end

  always_comb begin
    // default routing from M1
    s_paddr_o   = m1_paddr_i;
    s_psel_o    = m1_psel_i;
    s_penable_o = m1_penable_i;
    s_pwrite_o  = m1_pwrite_i;
    s_pwdata_o  = m1_pwdata_i;

    m0_prdata_o = s_prdata_i;
    m1_prdata_o = s_prdata_i;

    m0_pready_o = 1'b0;
    m1_pready_o = 1'b0;

    m0_pslverr_o = s_pslverr_i;
    m1_pslverr_o = s_pslverr_i;

    sel_d = sel_q;

    unique case (sel_q)
      SEL_M0: begin
        s_paddr_o   = m0_paddr_i;
        s_psel_o    = m0_psel_i;
        s_penable_o = m0_penable_i;
        s_pwrite_o  = m0_pwrite_i;
        s_pwdata_o  = m0_pwdata_i;

        m0_pready_o = s_pready_i;
        m1_pready_o = 1'b0;

        // Next owner after completion
        if (m0_psel_i && s_pready_i) begin
          if (m1_psel_i) sel_d = SEL_M1;
          else           sel_d = SEL_M0;
        end
      end

      SEL_M1: begin
        // already wired by default to M1
        m1_pready_o = s_pready_i;
        m0_pready_o = 1'b0;

        if (m0_psel_i) sel_d = SEL_M0;
      end
    endcase
  end

endmodule


// ============================================================================
//  Placeholder declarations for key blocks (replace with your RTL)
//  You can delete these stubs once real modules are available.
// ============================================================================

module pll_wrapper (
  input  logic clk_ref,
  input  logic rstn,
  output logic clk_arr,
  output logic clk_sram,
  output logic clk_ctrl,
  output logic locked
);
  // Simple pass-through for simulation if no PLL available
  assign clk_arr  = clk_ref;
  assign clk_sram = clk_ref;
  assign clk_ctrl = clk_ref;
  assign locked   = rstn;
endmodule

module csr_block #(
  parameter int unsigned APB_ADDR_W = 20,
  parameter int unsigned APB_DATA_W = 32,
  parameter int unsigned GB_BANKS   = 16
)(
  input  logic                   clk_i,
  input  logic                   rstn_i,
  input  logic [APB_ADDR_W-1:0]  paddr_i,
  input  logic                   psel_i,
  input  logic                   penable_i,
  input  logic                   pwrite_i,
  input  logic [APB_DATA_W-1:0]  pwdata_i,
  output logic [APB_DATA_W-1:0]  prdata_o,
  output logic                   pready_o,
  output logic                   pslverr_o,
  output logic                   array_enable_o,
  output logic [2:0]             array_mode_o,
  output logic                   sparsity_2of4_en_o,
  output logic [GB_BANKS-1:0]    part_a_mask_o,
  output logic [GB_BANKS-1:0]    part_w_mask_o,
  output logic [GB_BANKS-1:0]    part_c_mask_o,
  output logic                   doorbell_o,
  output logic [7:0]             irq_mask_o
);
  // Tie-offs for stub
  assign prdata_o         = '0;
  assign pready_o         = 1'b1;
  assign pslverr_o        = 1'b0;
  assign array_enable_o   = 1'b1;
  assign array_mode_o     = 3'd1;
  assign sparsity_2of4_en_o = 1'b0;
  assign part_a_mask_o    = '1;
  assign part_w_mask_o    = '1;
  assign part_c_mask_o    = '1;
  assign doorbell_o       = 1'b0;
  assign irq_mask_o       = 8'hFF;
endmodule

module riscv_subsys (
  input  logic           clk_i,
  input  logic           rstn_i,
  output logic [19:0]    paddr_o,
  output logic           psel_o,
  output logic           penable_o,
  output logic           pwrite_o,
  output logic [31:0]    pwdata_o,
  input  logic [31:0]    prdata_i,
  input  logic           pready_i,
  input  logic           pslverr_i,
  input  logic           jtag_tck_i,
  input  logic           jtag_trst_ni,
  input  logic           jtag_tms_i,
  input  logic           jtag_tdi_i,
  output logic           jtag_tdo_o,
  input  logic [31:0]    intr_i,
  output logic           wdog_irq_o
);
  assign paddr_o     = '0;
  assign psel_o      = 1'b0;
  assign penable_o   = 1'b0;
  assign pwrite_o    = 1'b0;
  assign pwdata_o    = '0;
  assign jtag_tdo_o  = 1'b0;
  assign wdog_irq_o  = 1'b0;
endmodule

module dma2d_a #(parameter int AXI_ADDR_W=48, AXI_DATA_W=512, AXI_ID_W=6, AXI_USER_W=1) (
  input  logic                         clk_i, rstn_i,
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
  output logic                         gb_w_req_o,
  output logic [31:0]                  gb_w_addr_o,
  output logic [AXI_DATA_W/8-1:0]      gb_w_strb_o,
  output logic [AXI_DATA_W-1:0]        gb_w_data_o,
  input  logic                         gb_w_ready_i,
  output logic                         irq_o
);
  assign m_awid_o='0; assign m_awaddr_o='0; assign m_awlen_o='0; assign m_awsize_o='0;
  assign m_awburst_o='0; assign m_awlock_o='0; assign m_awcache_o='0; assign m_awprot_o='0;
  assign m_awqos_o='0; assign m_awuser_o='0; assign m_awvalid_o=1'b0;
  assign m_wdata_o='0; assign m_wstrb_o='0; assign m_wlast_o=1'b0; assign m_wuser_o='0; assign m_wvalid_o=1'b0;
  assign m_bready_o=1'b1;
  assign m_arid_o='0; assign m_araddr_o='0; assign m_arlen_o='0; assign m_arsize_o='0;
  assign m_arburst_o='0; assign m_arlock_o='0; assign m_arcache_o='0; assign m_arprot_o='0;
  assign m_arqos_o='0; assign m_aruser_o='0; assign m_arvalid_o=1'b0;
  assign m_rready_o=1'b1;
  assign gb_w_req_o=1'b0; assign gb_w_addr_o='0; assign gb_w_strb_o='0; assign gb_w_data_o='0;
  assign irq_o=1'b0;
endmodule

module dma2d_w #(parameter int AXI_ADDR_W=48, AXI_DATA_W=512, AXI_ID_W=6, AXI_USER_W=1) (
  input  logic clk_i, rstn_i,
  // (same ports as dma2d_a)
  output logic [AXI_ID_W-1:0] m_awid_o, output logic [AXI_ADDR_W-1:0] m_awaddr_o,
  output logic [7:0] m_awlen_o, output logic [2:0] m_awsize_o, output logic [1:0] m_awburst_o,
  output logic m_awlock_o, output logic [3:0] m_awcache_o, output logic [2:0] m_awprot_o,
  output logic [3:0] m_awqos_o, output logic [AXI_USER_W-1:0] m_awuser_o, output logic m_awvalid_o, input logic m_awready_i,
  output logic [AXI_DATA_W-1:0] m_wdata_o, output logic [AXI_DATA_W/8-1:0] m_wstrb_o, output logic m_wlast_o,
  output logic [AXI_USER_W-1:0] m_wuser_o, output logic m_wvalid_o, input logic m_wready_i,
  input  logic [AXI_ID_W-1:0] m_bid_i, input logic [1:0] m_bresp_i, input logic [AXI_USER_W-1:0] m_buser_i, input logic m_bvalid_i, output logic m_bready_o,
  output logic [AXI_ID_W-1:0] m_arid_o, output logic [AXI_ADDR_W-1:0] m_araddr_o, output logic [7:0] m_arlen_o,
  output logic [2:0] m_arsize_o, output logic [1:0] m_arburst_o, output logic m_arlock_o, output logic [3:0] m_arcache_o,
  output logic [2:0] m_arprot_o, output logic [3:0] m_arqos_o, output logic [AXI_USER_W-1:0] m_aruser_o, output logic m_arvalid_o, input logic m_arready_i,
  input  logic [AXI_ID_W-1:0] m_rid_i, input logic [AXI_DATA_W-1:0] m_rdata_i, input logic [1:0] m_rresp_i, input logic m_rlast_i,
  input  logic [AXI_USER_W-1:0] m_ruser_i, input logic m_rvalid_i, output logic m_rready_o,
  output logic gb_w_req_o, output logic [31:0] gb_w_addr_o, output logic [AXI_DATA_W/8-1:0] gb_w_strb_o,
  output logic [AXI_DATA_W-1:0] gb_w_data_o, input logic gb_w_ready_i, output logic irq_o
);
  // tie-offs for stub
  assign m_awid_o='0; assign m_awaddr_o='0; assign m_awlen_o='0; assign m_awsize_o='0;
  assign m_awburst_o='0; assign m_awlock_o='0; assign m_awcache_o='0; assign m_awprot_o='0;
  assign m_awqos_o='0; assign m_awuser_o='0; assign m_awvalid_o=1'b0;
  assign m_wdata_o='0; assign m_wstrb_o='0; assign m_wlast_o=1'b0; assign m_wuser_o='0; assign m_wvalid_o=1'b0;
  assign m_bready_o=1'b1;
  assign m_arid_o='0; assign m_araddr_o='0; assign m_arlen_o='0; assign m_arsize_o='0;
  assign m_arburst_o='0; assign m_arlock_o='0; assign m_arcache_o='0; assign m_arprot_o='0;
  assign m_arqos_o='0; assign m_aruser_o='0; assign m_arvalid_o=1'b0;
  assign m_rready_o=1'b1;
  assign gb_w_req_o=1'b0; assign gb_w_addr_o='0; assign gb_w_strb_o='0; assign gb_w_data_o='0;
  assign irq_o=1'b0;
endmodule

module dma2d_c #(parameter int AXI_ADDR_W=48, AXI_DATA_W=512, AXI_ID_W=6, AXI_USER_W=1) (
  input  logic clk_i, rstn_i,
  // AXI
  output logic [AXI_ID_W-1:0] m_awid_o, output logic [AXI_ADDR_W-1:0] m_awaddr_o,
  output logic [7:0] m_awlen_o, output logic [2:0] m_awsize_o, output logic [1:0] m_awburst_o,
  output logic m_awlock_o, output logic [3:0] m_awcache_o, output logic [2:0] m_awprot_o,
  output logic [3:0] m_awqos_o, output logic [AXI_USER_W-1:0] m_awuser_o, output logic m_awvalid_o, input logic m_awready_i,
  output logic [AXI_DATA_W-1:0] m_wdata_o, output logic [AXI_DATA_W/8-1:0] m_wstrb_o, output logic m_wlast_o,
  output logic [AXI_USER_W-1:0] m_wuser_o, output logic m_wvalid_o, input logic m_wready_i,
  input  logic [AXI_ID_W-1:0] m_bid_i, input logic [1:0] m_bresp_i, input logic [AXI_USER_W-1:0] m_buser_i, input logic m_bvalid_i, output logic m_bready_o,
  output logic [AXI_ID_W-1:0] m_arid_o, output logic [AXI_ADDR_W-1:0] m_araddr_o, output logic [7:0] m_arlen_o,
  output logic [2:0] m_arsize_o, output logic [1:0] m_arburst_o, output logic m_arlock_o, output logic [3:0] m_arcache_o,
  output logic [2:0] m_arprot_o, output logic [3:0] m_arqos_o, output logic [AXI_USER_W-1:0] m_aruser_o, output logic m_arvalid_o, input logic m_arready_i,
  input  logic [AXI_ID_W-1:0] m_rid_i, input logic [AXI_DATA_W-1:0] m_rdata_i, input logic [1:0] m_rresp_i, input logic m_rlast_i,
  input  logic [AXI_USER_W-1:0] m_ruser_i, input logic m_rvalid_i, output logic m_rready_o,
  // GB read side
  output logic gb_r_req_o, output logic [31:0] gb_r_addr_o, input logic [AXI_DATA_W-1:0] gb_r_data_i,
  input  logic gb_r_valid_i, output logic gb_r_ready_o,
  output logic irq_o
);
  assign m_awid_o='0; assign m_awaddr_o='0; assign m_awlen_o='0; assign m_awsize_o='0;
  assign m_awburst_o='0; assign m_awlock_o='0; assign m_awcache_o='0; assign m_awprot_o='0;
  assign m_awqos_o='0; assign m_awuser_o='0; assign m_awvalid_o=1'b0;
  assign m_wdata_o='0; assign m_wstrb_o='0; assign m_wlast_o=1'b0; assign m_wuser_o='0; assign m_wvalid_o=1'b0;
  assign m_bready_o=1'b1;
  assign m_arid_o='0; assign m_araddr_o='0; assign m_arlen_o='0; assign m_arsize_o='0;
  assign m_arburst_o='0; assign m_arlock_o='0; assign m_arcache_o='0; assign m_arprot_o='0;
  assign m_arqos_o='0; assign m_aruser_o='0; assign m_arvalid_o=1'b0;
  assign m_rready_o=1'b1;
  assign gb_r_req_o=1'b0; assign gb_r_addr_o='0; assign gb_r_ready_o=1'b1;
  assign irq_o=1'b0;
endmodule

module tile_axi_arb_3to1 #(
  parameter int AXI_ADDR_W=48, AXI_DATA_W=512, AXI_ID_W=6, AXI_USER_W=1
)(
  input  logic clk_i, rstn_i,
  // masters in (arrays as declared above)
  input  logic [3-1:0][AXI_ID_W-1:0]   m_awid_i,
  input  logic [3-1:0][AXI_ADDR_W-1:0] m_awaddr_i,
  input  logic [3-1:0][7:0]            m_awlen_i,
  input  logic [3-1:0][2:0]            m_awsize_i,
  input  logic [3-1:0][1:0]            m_awburst_i,
  input  logic [3-1:0]                 m_awlock_i,
  input  logic [3-1:0][3:0]            m_awcache_i,
  input  logic [3-1:0][2:0]            m_awprot_i,
  input  logic [3-1:0][3:0]            m_awqos_i,
  input  logic [3-1:0][AXI_USER_W-1:0] m_awuser_i,
  input  logic [3-1:0]                 m_awvalid_i,
  output logic [3-1:0]                 m_awready_o,
  input  logic [3-1:0][AXI_DATA_W-1:0] m_wdata_i,
  input  logic [3-1:0][AXI_DATA_W/8-1:0] m_wstrb_i,
  input  logic [3-1:0]                 m_wlast_i,
  input  logic [3-1:0][AXI_USER_W-1:0] m_wuser_i,
  input  logic [3-1:0]                 m_wvalid_i,
  output logic [3-1:0]                 m_wready_o,
  output logic [3-1:0][AXI_ID_W-1:0]   m_bid_o,
  output logic [3-1:0][1:0]            m_bresp_o,
  output logic [3-1:0][AXI_USER_W-1:0] m_buser_o,
  output logic [3-1:0]                 m_bvalid_o,
  input  logic [3-1:0]                 m_bready_i,
  input  logic [3-1:0][AXI_ID_W-1:0]   m_arid_i,
  input  logic [3-1:0][AXI_ADDR_W-1:0] m_araddr_i,
  input  logic [3-1:0][7:0]            m_arlen_i,
  input  logic [3-1:0][2:0]            m_arsize_i,
  input  logic [3-1:0][1:0]            m_arburst_i,
  input  logic [3-1:0]                 m_arlock_i,
  input  logic [3-1:0][3:0]            m_arcache_i,
  input  logic [3-1:0][2:0]            m_arprot_i,
  input  logic [3-1:0][3:0]            m_arqos_i,
  input  logic [3-1:0][AXI_USER_W-1:0] m_aruser_i,
  input  logic [3-1:0]                 m_arvalid_i,
  output logic [3-1:0]                 m_arready_o,
  output logic [3-1:0][AXI_ID_W-1:0]   m_rid_o,
  output logic [3-1:0][AXI_DATA_W-1:0] m_rdata_o,
  output logic [3-1:0][1:0]            m_rresp_o,
  output logic [3-1:0]                 m_rlast_o,
  output logic [3-1:0][AXI_USER_W-1:0] m_ruser_o,
  output logic [3-1:0]                 m_rvalid_o,
  input  logic [3-1:0]                 m_rready_i,
  // single slave out
  output logic [AXI_ID_W-1:0]          s_awid_o,
  output logic [AXI_ADDR_W-1:0]        s_awaddr_o,
  output logic [7:0]                   s_awlen_o,
  output logic [2:0]                   s_awsize_o,
  output logic [1:0]                   s_awburst_o,
  output logic                         s_awlock_o,
  output logic [3:0]                   s_awcache_o,
  output logic [2:0]                   s_awprot_o,
  output logic [3:0]                   s_awqos_o,
  output logic [AXI_USER_W-1:0]        s_awuser_o,
  output logic                         s_awvalid_o,
  input  logic                         s_awready_i,
  output logic [AXI_DATA_W-1:0]        s_wdata_o,
  output logic [AXI_STRB_W-1:0]        s_wstrb_o,
  output logic                         s_wlast_o,
  output logic [AXI_USER_W-1:0]        s_wuser_o,
  output logic                         s_wvalid_o,
  input  logic                         s_wready_i,
  input  logic [AXI_ID_W-1:0]          s_bid_i,
  input  logic [1:0]                   s_bresp_i,
  input  logic [AXI_USER_W-1:0]        s_buser_i,
  input  logic                         s_bvalid_i,
  output logic                         s_bready_o,
  output logic [AXI_ID_W-1:0]          s_arid_o,
  output logic [AXI_ADDR_W-1:0]        s_araddr_o,
  output logic [7:0]                   s_arlen_o,
  output logic [2:0]                   s_arsize_o,
  output logic [1:0]                   s_arburst_o,
  output logic                         s_arlock_o,
  output logic [3:0]                   s_arcache_o,
  output logic [2:0]                   s_arprot_o,
  output logic [3:0]                   s_arqos_o,
  output logic [AXI_USER_W-1:0]        s_aruser_o,
  output logic                         s_arvalid_o,
  input  logic                         s_arready_i,
  input  logic [AXI_ID_W-1:0]          s_rid_i,
  input  logic [AXI_DATA_W-1:0]        s_rdata_i,
  input  logic [1:0]                   s_rresp_i,
  input  logic                         s_rlast_i,
  input  logic [AXI_USER_W-1:0]        s_ruser_i,
  input  logic                         s_rvalid_i,
  output logic                         s_rready_o
);
  // Minimal tie-off arbiter (non-functional placeholder)
  assign m_awready_o = '0; assign m_wready_o='0; assign m_bvalid_o='0; assign m_bid_o='0; assign m_bresp_o='0; assign m_buser_o='0;
  assign m_arready_o = '0; assign m_rvalid_o='0; assign m_rid_o='0; assign m_rdata_o='0; assign m_rresp_o='0; assign m_rlast_o='0; assign m_ruser_o='0;

  assign s_awid_o='0; assign s_awaddr_o='0; assign s_awlen_o='0; assign s_awsize_o='0; assign s_awburst_o='0; assign s_awlock_o='0;
  assign s_awcache_o='0; assign s_awprot_o='0; assign s_awqos_o='0; assign s_awuser_o='0; assign s_awvalid_o=1'b0;
  assign s_wdata_o='0; assign s_wstrb_o='0; assign s_wlast_o=1'b0; assign s_wuser_o='0; assign s_wvalid_o=1'b0; assign s_bready_o=1'b1;
  assign s_arid_o='0; assign s_araddr_o='0; assign s_arlen_o='0; assign s_arsize_o='0; assign s_arburst_o='0; assign s_arlock_o='0;
  assign s_arcache_o='0; assign s_arprot_o='0; assign s_arqos_o='0; assign s_aruser_o='0; assign s_arvalid_o=1'b0; assign s_rready_o=1'b1;
endmodule

module gb_crossbar #(
  parameter int NUM_BANKS=16, DATA_W=256, ADDR_W=16, INJ_BUS_W=2048, DRAIN_BUS_W=2048
)(
  input  logic                   clk_i, rstn_i,
  input  logic [NUM_BANKS-1:0]   part_a_mask_i, part_w_mask_i, part_c_mask_i,
  // DMA write/read (placeholder)
  input  logic                   dma_w_req_i,
  input  logic [31:0]            dma_w_addr_i,
  input  logic [DATA_W/8-1:0]    dma_w_strb_i,
  input  logic [DATA_W-1:0]      dma_w_data_i,
  output logic                   dma_w_ready_o,

  input  logic                   dma_r_req_i,
  input  logic [31:0]            dma_r_addr_i,
  output logic [DATA_W-1:0]      dma_r_data_o,
  output logic                   dma_r_valid_o,
  input  logic                   dma_r_ready_i,

  // Injection buses
  output logic [INJ_BUS_W-1:0]   inj_a_data_o,
  output logic                   inj_a_valid_o,
  input  logic                   inj_a_ready_i,

  output logic [INJ_BUS_W-1:0]   inj_w_data_o,
  output logic                   inj_w_valid_o,
  input  logic                   inj_w_ready_i,

  // Drain in
  input  logic [DRAIN_BUS_W-1:0] drain_c_data_i,
  input  logic                   drain_c_valid_i,
  output logic                   drain_c_ready_o
);
  // Tie-offs for stub
  assign dma_w_ready_o   = 1'b1;
  assign dma_r_data_o    = '0;
  assign dma_r_valid_o   = 1'b0;

  assign inj_a_data_o    = '0;
  assign inj_a_valid_o   = 1'b0;
  assign inj_w_data_o    = '0;
  assign inj_w_valid_o   = 1'b0;

  assign drain_c_ready_o = 1'b1;
endmodule

module systolic_array_top #(
  parameter int M=128, N=128, A_BUS_W=2048, W_BUS_W=2048, C_BUS_W=2048
)(
  input  logic                 clk_i, rstn_i,
  input  logic                 enable_i,
  input  logic [2:0]           mode_i,
  input  logic                 sparsity2of4_en_i,
  input  logic [A_BUS_W-1:0]   a_data_i,
  input  logic                 a_valid_i,
  output logic                 a_ready_o,
  input  logic [W_BUS_W-1:0]   w_data_i,
  input  logic                 w_valid_i,
  output logic                 w_ready_o,
  output logic [C_BUS_W-1:0]   c_data_o,
  output logic                 c_valid_o,
  input  logic                 c_ready_i
);
  assign a_ready_o = 1'b1;
  assign w_ready_o = 1'b1;
  assign c_data_o  = '0;
  assign c_valid_o = 1'b0;
endmodule

module post_ops_top #(
  parameter int BUS_W=2048
)(
  input  logic                 clk_i, rstn_i,
  input  logic                 cfg_valid_i,
  input  logic [63:0]          cfg_payload_i,
  input  logic [BUS_W-1:0]     in_data_i,
  input  logic                 in_valid_i,
  output logic                 in_ready_o,
  output logic [BUS_W-1:0]     out_data_o,
  output logic                 out_valid_o,
  input  logic                 out_ready_i
);
  assign in_ready_o  = 1'b1;
  assign out_data_o  = in_data_i;
  assign out_valid_o = in_valid_i & out_ready_i; // simple pass-through for stub
endmodule

module perf_counters (
  input  logic clk_i, rstn_i,
  output logic irq_o
);
  assign irq_o = 1'b0;
endmodule

module err_logger (
  input  logic clk_i, rstn_i,
  output logic irq_o
);
  assign irq_o = 1'b0;
endmodule

module noc_router_5p #(
  parameter int FLIT_W=512
)(
  input  logic                 clk_i, rstn_i,
  // North
  input  logic [FLIT_W-1:0]    n_rx_flit_i, input logic n_rx_valid_i, output logic n_rx_ready_o,
  output logic [FLIT_W-1:0]    n_tx_flit_o, output logic n_tx_valid_o, input  logic n_tx_ready_i,
  // East
  input  logic [FLIT_W-1:0]    e_rx_flit_i, input logic e_rx_valid_i, output logic e_rx_ready_o,
  output logic [FLIT_W-1:0]    e_tx_flit_o, output logic e_tx_valid_o, input  logic e_tx_ready_i,
  // South
  input  logic [FLIT_W-1:0]    s_rx_flit_i, input logic s_rx_valid_i, output logic s_rx_ready_o,
  output logic [FLIT_W-1:0]    s_tx_flit_o, output logic s_tx_valid_o, input  logic s_tx_ready_i,
  // West
  input  logic [FLIT_W-1:0]    w_rx_flit_i, input logic w_rx_valid_i, output logic w_rx_ready_o,
  output logic [FLIT_W-1:0]    w_tx_flit_o, output logic w_tx_valid_o, input  logic w_tx_ready_i,
  // Local
  input  logic [FLIT_W-1:0]    l_rx_flit_i, input logic l_rx_valid_i, output logic l_rx_ready_o,
  output logic [FLIT_W-1:0]    l_tx_flit_o, output logic l_tx_valid_o, input  logic l_tx_ready_i
);
  // Stub: pass-through ready, no traffic
  assign n_rx_ready_o=1'b1; assign e_rx_ready_o=1'b1; assign s_rx_ready_o=1'b1; assign w_rx_ready_o=1'b1; assign l_rx_ready_o=1'b1;
  assign n_tx_flit_o='0;    assign n_tx_valid_o=1'b0;
  assign e_tx_flit_o='0;    assign e_tx_valid_o=1'b0;
  assign s_tx_flit_o='0;    assign s_tx_valid_o=1'b0;
  assign w_tx_flit_o='0;    assign w_tx_valid_o=1'b0;
  assign l_tx_flit_o='0;    assign l_tx_valid_o=1'b0;
endmodule

module dvfs_ctrl (
  input  logic clk_i, rstn_i,
  output logic throttle_o,
  output logic irq_o
);
  assign throttle_o = 1'b0;
  assign irq_o      = 1'b0;
endmodule

`default_nettype wire
