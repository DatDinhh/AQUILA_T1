// ============================================================================
//  aquila_tile_tb_top.sv
//  Aquila - SoC/Tile Top-Level Testbench Harness
// ----------------------------------------------------------------------------
//  What this TB does
//   - Generates clocks/resets for system/AXI/array/sensor domains
//   - Instantiates DUT (tile_top) and two AXI4 DRAM models
//   - Exposes JTAG pins; optional simple JTAG reset/IDCODE poke
//   - Supports two bring-up modes via +args:
//        * USE_FW=1 : firmware-in-the-loop (boot_rom.hex inside DUT)
//        * USE_FW=0 : CSR-driven mode (stubs included; connect if exposed)
//   - Includes simple monitors and a writeback detector/timeout
//
//  How to run (examples)
//   - Firmware mode, enable waves:
//       <sim> +USE_FW=1 +WAVES=1 +MEM_BYTES=134217728 +A_BASE=0x1000 +B_BASE=0x2000 +C_BASE=0x3000
//   - CSR mode (if your top exposes CSRs as AXI-Lite/CSR-Lite):
//       <sim> +USE_FW=0 +CSR_MODE=1 +CSR_BASE=0x00000000  (wire up csr driver tasks)
//
//  Notes
//   - Update DUT port mapping in the section marked "TODO: match your tile_top port names".
//   - If you have ONE AXI master, keep m0_* and comment out the m1_* block.
//   - The embedded AXI RAM model handles INCR bursts with byte-accurate WSTRB.
//   - PMIC/PLL models are stubs; if your DVFS interface is internal to the DUT,
//     you can ignore them (left unconnected).
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module aquila_tile_tb_top;

  // Parameters / configuration (overridable via +args)

  localparam int AXI_ADDR_W = 40;
  localparam int AXI_DATA_W = 256;   // 256-bit datapath (matches many of your DMAs)
  localparam int AXI_ID_W   = 4;
  localparam int AXI_STRB_W = AXI_DATA_W/8;

  // Memory size (default 128 MiB) — override with +MEM_BYTES=<dec or hex>
  longint unsigned MEM_BYTES = 128*1024*1024;

  // Optional A/B/C tile bases for firmware tests
  longint unsigned A_BASE = 'h0000_1000;
  longint unsigned B_BASE = 'h0000_2000;
  longint unsigned C_BASE = 'h0000_3000;

  // Mode selection
  bit USE_FW   = 1'b1;  // 1=firmware-in-loop, 0=TB drives CSRs (if exposed)
  bit CSR_MODE = 1'b0;  // set to 1 if your top exposes a CSR port and you want to drive it
  bit WAVES    = 1'b1;

  // Parse plusargs early
  initial begin
    void'($value$plusargs("USE_FW=%0d", USE_FW));
    void'($value$plusargs("CSR_MODE=%0d", CSR_MODE));
    void'($value$plusargs("WAVES=%0d",   WAVES));
    void'($value$plusargs("A_BASE=%h",   A_BASE));
    void'($value$plusargs("B_BASE=%h",   B_BASE));
    void'($value$plusargs("C_BASE=%h",   C_BASE));
    longint unsigned tmp_mem;
    if ($value$plusargs("MEM_BYTES=%d", tmp_mem)) MEM_BYTES = tmp_mem;
  end

  // Clocks / Resets

  logic clk_sys = 1'b0;   // ~100 MHz
  logic clk_axi = 1'b0;   // ~200 MHz
  logic clk_arr = 1'b0;   // ~200 MHz (can be tied to AXI initially)
  logic clk_sns = 1'b0;   // ~25  MHz (thermal sensor domain)

  logic rstn      = 1'b0; // active-low system reset
  logic sns_rstn  = 1'b0; // sensor domain reset
  logic trst_n    = 1'b0; // JTAG TRSTn

  // Periods (ns) — adjust as needed
  real T_SYS = 10.0;   // 100 MHz
  real T_AXI = 5.0;    // 200 MHz
  real T_ARR = 5.0;    // 200 MHz
  real T_SNS = 40.0;   // 25 MHz

  always #(T_SYS/2.0) clk_sys = ~clk_sys;
  always #(T_AXI/2.0) clk_axi = ~clk_axi;
  always #(T_ARR/2.0) clk_arr = ~clk_arr;
  always #(T_SNS/2.0) clk_sns = ~clk_sns;

  // Ordered reset release
  initial begin
    rstn     = 1'b0;
    sns_rstn = 1'b0;
    trst_n   = 1'b0;
    repeat (8) @(posedge clk_sys);
    trst_n   = 1'b1;
    repeat (8) @(posedge clk_sys);
    sns_rstn = 1'b1;
    repeat (8) @(posedge clk_sys);
    rstn     = 1'b1;
  end

  // Optional waveform dumping

  initial if (WAVES) begin
`ifdef VERILATOR
    $dumpfile("waves.fst");
    $dumpvars(0, aquila_tile_tb_top);
`else
    $value$plusargs("WAVEFILE=%s", automatic string wf);
`ifndef WAVEFILE
    string wf2 = "waves.vcd";
    $display("[%0t] WAVES on → %s", $time, wf2);
    $dumpfile(wf2);
`else
    $display("[%0t] WAVES on → %s", $time, wf);
    $dumpfile(wf);
`endif
    $dumpvars(0, aquila_tile_tb_top);
`endif
  end

  // DUT external interfaces (AXI Masters x2, JTAG)
  
  // AXI Master 0 (e.g., Read/Write combined, or Writeback DMA)
  wire [AXI_ID_W-1:0]   m0_awid;
  wire [AXI_ADDR_W-1:0] m0_awaddr;
  wire [7:0]            m0_awlen;
  wire [2:0]            m0_awsize;
  wire [1:0]            m0_awburst;
  wire                  m0_awvalid;
  wire                  m0_awready;

  wire [AXI_DATA_W-1:0] m0_wdata;
  wire [AXI_STRB_W-1:0] m0_wstrb;
  wire                  m0_wlast;
  wire                  m0_wvalid;
  wire                  m0_wready;

  wire [AXI_ID_W-1:0]   m0_bid;
  wire [1:0]            m0_bresp;
  wire                  m0_bvalid;
  wire                  m0_bready;

  wire [AXI_ID_W-1:0]   m0_arid;
  wire [AXI_ADDR_W-1:0] m0_araddr;
  wire [7:0]            m0_arlen;
  wire [2:0]            m0_arsize;
  wire [1:0]            m0_arburst;
  wire                  m0_arvalid;
  wire                  m0_arready;

  wire [AXI_ID_W-1:0]   m0_rid;
  wire [AXI_DATA_W-1:0] m0_rdata;
  wire [1:0]            m0_rresp;
  wire                  m0_rlast;
  wire                  m0_rvalid;
  wire                  m0_rready;

  // AXI Master 1 (optional second port; comment out if unused)
  wire [AXI_ID_W-1:0]   m1_awid;
  wire [AXI_ADDR_W-1:0] m1_awaddr;
  wire [7:0]            m1_awlen;
  wire [2:0]            m1_awsize;
  wire [1:0]            m1_awburst;
  wire                  m1_awvalid;
  wire                  m1_awready;

  wire [AXI_DATA_W-1:0] m1_wdata;
  wire [AXI_STRB_W-1:0] m1_wstrb;
  wire                  m1_wlast;
  wire                  m1_wvalid;
  wire                  m1_wready;

  wire [AXI_ID_W-1:0]   m1_bid;
  wire [1:0]            m1_bresp;
  wire                  m1_bvalid;
  wire                  m1_bready;

  wire [AXI_ID_W-1:0]   m1_arid;
  wire [AXI_ADDR_W-1:0] m1_araddr;
  wire [7:0]            m1_arlen;
  wire [2:0]            m1_arsize;
  wire [1:0]            m1_arburst;
  wire                  m1_arvalid;
  wire                  m1_arready;

  wire [AXI_ID_W-1:0]   m1_rid;
  wire [AXI_DATA_W-1:0] m1_rdata;
  wire [1:0]            m1_rresp;
  wire                  m1_rlast;
  wire                  m1_rvalid;
  wire                  m1_rready;

  // JTAG pins
  logic tck = 1'b0, tms = 1'b1, tdi = 1'b0;
  wire  tdo, tdo_oe;

  // Slower JTAG clock
  always #50.0 tck = ~tck;

  // DUT Instance

  // TODO: match your tile_top port names here. If your design uses different
  // names (e.g., m_axi_awaddr instead of m0_axi_awaddr), rename accordingly.
  tile_top dut (
    // Clocks / resets
    .clk_sys_i  (clk_sys),
    .clk_axi_i  (clk_axi),
    .clk_arr_i  (clk_arr),
    .sns_clk_i  (clk_sns),
    .rstn_i     (rstn),
    .sns_rstn_i (sns_rstn),

    // JTAG
    .tck_i      (tck),
    .tms_i      (tms),
    .tdi_i      (tdi),
    .tdo_o      (tdo),
    .tdo_oe_o   (tdo_oe),
    .trst_n_i   (trst_n),

    // AXI Master 0
    .m0_axi_awid    (m0_awid),
    .m0_axi_awaddr  (m0_awaddr),
    .m0_axi_awlen   (m0_awlen),
    .m0_axi_awsize  (m0_awsize),
    .m0_axi_awburst (m0_awburst),
    .m0_axi_awvalid (m0_awvalid),
    .m0_axi_awready (m0_awready),

    .m0_axi_wdata   (m0_wdata),
    .m0_axi_wstrb   (m0_wstrb),
    .m0_axi_wlast   (m0_wlast),
    .m0_axi_wvalid  (m0_wvalid),
    .m0_axi_wready  (m0_wready),

    .m0_axi_bid     (m0_bid),
    .m0_axi_bresp   (m0_bresp),
    .m0_axi_bvalid  (m0_bvalid),
    .m0_axi_bready  (m0_bready),

    .m0_axi_arid    (m0_arid),
    .m0_axi_araddr  (m0_araddr),
    .m0_axi_arlen   (m0_arlen),
    .m0_axi_arsize  (m0_arsize),
    .m0_axi_arburst (m0_arburst),
    .m0_axi_arvalid (m0_arvalid),
    .m0_axi_arready (m0_arready),

    .m0_axi_rid     (m0_rid),
    .m0_axi_rdata   (m0_rdata),
    .m0_axi_rresp   (m0_rresp),
    .m0_axi_rlast   (m0_rlast),
    .m0_axi_rvalid  (m0_rvalid),
    .m0_axi_rready  (m0_rready)

    // If you have a second AXI master at top, uncomment and wire:
    ,.m1_axi_awid    (m1_awid),
     .m1_axi_awaddr  (m1_awaddr),
     .m1_axi_awlen   (m1_awlen),
     .m1_axi_awsize  (m1_awsize),
     .m1_axi_awburst (m1_awburst),
     .m1_axi_awvalid (m1_awvalid),
     .m1_axi_awready (m1_awready),

     .m1_axi_wdata   (m1_wdata),
     .m1_axi_wstrb   (m1_wstrb),
     .m1_axi_wlast   (m1_wlast),
     .m1_axi_wvalid  (m1_wvalid),
     .m1_axi_wready  (m1_wready),

     .m1_axi_bid     (m1_bid),
     .m1_axi_bresp   (m1_bresp),
     .m1_axi_bvalid  (m1_bvalid),
     .m1_axi_bready  (m1_bready),

     .m1_axi_arid    (m1_arid),
     .m1_axi_araddr  (m1_araddr),
     .m1_axi_arlen   (m1_arlen),
     .m1_axi_arsize  (m1_arsize),
     .m1_axi_arburst (m1_arburst),
     .m1_axi_arvalid (m1_arvalid),
     .m1_axi_arready (m1_arready),

     .m1_axi_rid     (m1_rid),
     .m1_axi_rdata   (m1_rdata),
     .m1_axi_rresp   (m1_rresp),
     .m1_axi_rlast   (m1_rlast),
     .m1_axi_rvalid  (m1_rvalid),
     .m1_axi_rready  (m1_rready)

    // If your tile exposes a CSR slave externally (AXI-Lite/CSR-Lite),
    // connect it here and drive from the CSR tasks below.
  );

  // AXI DRAM Models (Slave) for DUT Masters

  axi_mem_model_simple #(
    .ADDR_W(AXI_ADDR_W), .DATA_W(AXI_DATA_W), .ID_W(AXI_ID_W), .MEM_BYTES(MEM_BYTES)
  ) dram0 (
    .ACLK    (clk_axi),
    .ARESETn (rstn),
    .AWID    (m0_awid),    .AWADDR  (m0_awaddr), .AWLEN (m0_awlen),
    .AWSIZE  (m0_awsize),  .AWBURST (m0_awburst),
    .AWVALID (m0_awvalid), .AWREADY (m0_awready),
    .WDATA   (m0_wdata),   .WSTRB   (m0_wstrb),  .WLAST (m0_wlast),
    .WVALID  (m0_wvalid),  .WREADY  (m0_wready),
    .BID     (m0_bid),     .BRESP   (m0_bresp),  .BVALID(m0_bvalid), .BREADY(m0_bready),
    .ARID    (m0_arid),    .ARADDR  (m0_araddr), .ARLEN (m0_arlen),
    .ARSIZE  (m0_arsize),  .ARBURST (m0_arburst),
    .ARVALID (m0_arvalid), .ARREADY (m0_arready),
    .RID     (m0_rid),     .RDATA   (m0_rdata),  .RRESP (m0_rresp),
    .RLAST   (m0_rlast),   .RVALID  (m0_rvalid), .RREADY(m0_rready)
  );

  axi_mem_model_simple #(
    .ADDR_W(AXI_ADDR_W), .DATA_W(AXI_DATA_W), .ID_W(AXI_ID_W), .MEM_BYTES(MEM_BYTES)
  ) dram1 (
    .ACLK    (clk_axi),
    .ARESETn (rstn),
    .AWID    (m1_awid),    .AWADDR  (m1_awaddr), .AWLEN (m1_awlen),
    .AWSIZE  (m1_awsize),  .AWBURST (m1_awburst),
    .AWVALID (m1_awvalid), .AWREADY (m1_awready),
    .WDATA   (m1_wdata),   .WSTRB   (m1_wstrb),  .WLAST (m1_wlast),
    .WVALID  (m1_wvalid),  .WREADY  (m1_wready),
    .BID     (m1_bid),     .BRESP   (m1_bresp),  .BVALID(m1_bvalid), .BREADY(m1_bready),
    .ARID    (m1_arid),    .ARADDR  (m1_araddr), .ARLEN (m1_arlen),
    .ARSIZE  (m1_arsize),  .ARBURST (m1_arburst),
    .ARVALID (m1_arvalid), .ARREADY (m1_arready),
    .RID     (m1_rid),     .RDATA   (m1_rdata),  .RRESP (m1_rresp),
    .RLAST   (m1_rlast),   .RVALID  (m1_rvalid), .RREADY(m1_rready)
  );

  // Optional: DVFS PMIC/PLL models (connect only if DUT exports them)

  // Example signals if your top exposes them (comment out if not present):
  /*
  wire        vreq_valid; wire [7:0] vsel; wire vreq_up; wire vreq_ack; wire vready;
  wire        freq_req;   wire [15:0] freq_code; wire pll_ack; wire pll_lock;

  pmic_model #(.ACK_DELAY_CYC(2), .READY_DELAY_CYC(5000)) pmic0 (
    .clk_i(clk_sys), .rstn_i(rstn),
    .vreq_valid_i(vreq_valid), .vsel_i(vsel), .vreq_up_i(vreq_up),
    .vreq_ack_o(vreq_ack), .vready_o(vready)
  );

  pll_model #(.ACK_DELAY_CYC(2), .LOCK_DELAY_CYC(15000)) pll0 (
    .clk_i(clk_sys), .rstn_i(rstn),
    .freq_req_i(freq_req), .freq_code_i(freq_code),
    .pll_ack_o(pll_ack), .pll_lock_o(pll_lock)
  );
  */

  // Simple AXI monitor (counts bursts & bytes)

  int unsigned m0_wr_beats=0, m0_rd_beats=0;
  always @(posedge clk_axi) if (rstn) begin
    if (m0_wvalid & m0_wready) m0_wr_beats++;
    if (m0_rvalid & m0_rready) m0_rd_beats++;
  end

  // Stimulus / Bring-up

  initial begin : main_stim
    @(posedge rstn);
    $display("[%0t] Reset deasserted; USE_FW=%0d CSR_MODE=%0d", $time, USE_FW, CSR_MODE);

    // Preload A/B tiles if you know firmware's expected bases
    preload_tile(A_BASE, 64/*bytes*/, 8'h01);
    preload_tile(B_BASE, 64/*bytes*/, 8'h7F);

    if (USE_FW) begin
      // Firmware-in-the-loop: wait for writeback at C_BASE, then basic checks
      wait_for_writeback(C_BASE, 64, 1000000);
      $display("[%0t] Detected writeback at 0x%0h; wr_beats=%0d rd_beats=%0d",
               $time, C_BASE, m0_wr_beats, m0_rd_beats);
    end
    else if (CSR_MODE) begin
      // CSR-driven path (uncomment and wire if your tile exposes a CSR slave)
      // program_minimal_dma_example();
      $fatal(1, "CSR_MODE selected but CSR driver is not wired in this harness.");
    end
    else begin
      $display("[%0t] Neither USE_FW nor CSR_MODE set; idle run for 1 ms", $time);
      repeat (200_000) @(posedge clk_sys);
    end

    $display("[%0t] TB PASS (smoke)", $time);
    #1000 $finish;
  end

  // Utilities: preload DRAM, wait for writeback

  task automatic preload_tile(longint unsigned base, int unsigned nbytes, byte pattern);
    if (base + nbytes >= MEM_BYTES) begin
      $display("[%0t] WARN: preload base out of range (0x%0h)", $time, base);
      return;
    end
    for (int i=0; i<nbytes; i++) begin
      dram0.mem[base+i] = (pattern + i[7:0]);
    end
  endtask

  task automatic wait_for_writeback(longint unsigned base, int unsigned nbytes, int unsigned max_cycles);
    byte snap[]; snap = new[nbytes];
    for (int i=0; i<nbytes; i++) snap[i] = dram0.mem[base+i];
    for (int cyc=0; cyc<max_cycles; cyc++) begin
      @(posedge clk_axi);
      for (int i=0; i<nbytes; i++) begin
        if (dram0.mem[base+i] !== snap[i]) begin
          $display("[%0t] Writeback detected at +%0d bytes (old=%0h new=%0h)",
                   $time, i, snap[i], dram0.mem[base+i]);
          return;
        end
      end
    end
    $fatal(1, "Timeout waiting for writeback at 0x%0h (%0d bytes).", base, nbytes);
  endtask

  // --------------------------------------------------------------------------
  // (Optional) CSR driver tasks - fill in if expose a CSR port at top
  // --------------------------------------------------------------------------
  /*
  // Example raw CSR-Lite wires (connect to dut)
  logic         csr_valid, csr_write, csr_ready;
  logic [11:0]  csr_addr;
  logic [31:0]  csr_wdata, csr_rdata;
  logic [3:0]   csr_wstrb;

  task automatic csr_write32(input logic [11:0] addr, input logic [31:0] data);
    csr_addr  <= addr; csr_wdata <= data; csr_wstrb <= 4'hF;
    csr_write <= 1'b1; csr_valid <= 1'b1;
    @(posedge clk_sys); wait (csr_ready); @(posedge clk_sys);
    csr_valid <= 1'b0; csr_write <= 1'b0; csr_wstrb <= '0;
  endtask

  task automatic csr_read32(input logic [11:0] addr, output logic [31:0] data);
    csr_addr  <= addr; csr_write <= 1'b0; csr_valid <= 1'b1;
    @(posedge clk_sys); wait (csr_ready); data = csr_rdata; @(posedge clk_sys);
    csr_valid <= 1'b0;
  endtask

  task automatic program_minimal_dma_example();
    // TODO: use addresses from your csr_map.json -> auto-generated header
    // csr_write32(ADDR_DMA_A_BASE, A_BASE[31:0]);
    // csr_write32(ADDR_DMA_B_BASE, B_BASE[31:0]);
    // csr_write32(ADDR_DMA_C_BASE, C_BASE[31:0]);
    // csr_write32(ADDR_DMA_GO, 32'h1);
  endtask
  */

  // Minimal JTAG helper (reset-only, extend to shift_ir/dr if needed)

  task automatic jtag_reset_5tms();
    // 5x TMS=1 on TCK↑ guarantees TAP reset per 1149.1
    tms <= 1'b1;
    repeat (5) @(posedge tck);
    tms <= 1'b0;
  endtask

  initial begin : jtag_boot
    @(posedge trst_n);
    repeat (2) @(posedge tck);
    jtag_reset_5tms();
  end

  // Embedded AXI4 DRAM model (INCR bursts, byte-accurate WSTRB)
  
  // This is identical to the standalone axi_mem_model_simple.sv you can keep
  // in tb/. It's embedded here for a single-file bring-up.
  module axi_mem_model_simple #(
    parameter int ADDR_W = 40,
    parameter int DATA_W = 256,
    parameter int ID_W   = 4,
    parameter longint MEM_BYTES = 64*1024*1024
  )(
    input  logic                 ACLK,
    input  logic                 ARESETn,
    input  logic [ID_W-1:0]      AWID,
    input  logic [ADDR_W-1:0]    AWADDR,
    input  logic [7:0]           AWLEN,
    input  logic [2:0]           AWSIZE,
    input  logic [1:0]           AWBURST,
    input  logic                 AWVALID,
    output logic                 AWREADY,
    input  logic [DATA_W-1:0]    WDATA,
    input  logic [DATA_W/8-1:0]  WSTRB,
    input  logic                 WLAST,
    input  logic                 WVALID,
    output logic                 WREADY,
    output logic [ID_W-1:0]      BID,
    output logic [1:0]           BRESP,
    output logic                 BVALID,
    input  logic                 BREADY,
    input  logic [ID_W-1:0]      ARID,
    input  logic [ADDR_W-1:0]    ARADDR,
    input  logic [7:0]           ARLEN,
    input  logic [2:0]           ARSIZE,
    input  logic [1:0]           ARBURST,
    input  logic                 ARVALID,
    output logic                 ARREADY,
    output logic [ID_W-1:0]      RID,
    output logic [DATA_W-1:0]    RDATA,
    output logic [1:0]           RRESP,
    output logic                 RLAST,
    output logic                 RVALID,
    input  logic                 RREADY
  );
    localparam int BYTES_PER_BEAT = DATA_W/8;
    byte mem [0:MEM_BYTES-1];

    typedef enum logic [1:0] {W_IDLE, W_DATA, W_RESP} wst_e;
    wst_e wst_q; logic [ADDR_W-1:0] waddr_q; logic [7:0] wlen_q; logic [2:0] wsize_q; logic [ID_W-1:0] wid_q;

    typedef enum logic [1:0] {R_IDLE, R_DATA} rst_e;
    rst_e rst_q; logic [ADDR_W-1:0] raddr_q; logic [7:0] rlen_q; logic [2:0] rsize_q; logic [ID_W-1:0] rid_q;

    // Defaults
    always_comb begin
      AWREADY = (wst_q == W_IDLE);
      WREADY  = (wst_q == W_DATA);
      BVALID  = (wst_q == W_RESP);
      BRESP   = 2'b00; BID = wid_q;

      ARREADY = (rst_q == R_IDLE);
      RVALID  = (rst_q == R_DATA);
      RRESP   = 2'b00; RID = rid_q;
      RLAST   = (rst_q == R_DATA) && (rlen_q == 8'd0);
    end

    // Write channel
    always_ff @(posedge ACLK or negedge ARESETn) begin
      if (!ARESETn) begin
        wst_q <= W_IDLE; waddr_q <= '0; wlen_q <= '0; wsize_q <= 3'd0; wid_q <= '0;
      end else begin
        unique case (wst_q)
          W_IDLE: if (AWVALID && AWREADY) begin
            waddr_q <= AWADDR; wlen_q <= AWLEN; wsize_q <= AWSIZE; wid_q <= AWID; wst_q <= W_DATA;
          end
          W_DATA: if (WVALID && WREADY) begin
            for (int b=0; b<BYTES_PER_BEAT; b++) if (WSTRB[b]) begin
              longint unsigned a = waddr_q + b;
              if (a < MEM_BYTES) mem[a] <= WDATA[8*b +: 8];
            end
            waddr_q <= waddr_q + (1 << wsize_q);
            if (WLAST) wst_q <= W_RESP;
          end
          W_RESP: if (BREADY) wst_q <= W_IDLE;
          default: wst_q <= W_IDLE;
        endcase
      end
    end

    // Read channel
    always_ff @(posedge ACLK or negedge ARESETn) begin
      if (!ARESETn) begin
        rst_q <= R_IDLE; raddr_q <= '0; rlen_q <= '0; rsize_q <= 3'd0; rid_q <= '0; RDATA <= '0;
      end else begin
        unique case (rst_q)
          R_IDLE: if (ARVALID && ARREADY) begin
            raddr_q <= ARADDR; rlen_q <= ARLEN; rsize_q <= ARSIZE; rid_q <= ARID; rst_q <= R_DATA;
          end
          R_DATA: if (!RVALID || (RVALID && RREADY)) begin
            logic [DATA_W-1:0] rd = '0;
            for (int b=0; b<BYTES_PER_BEAT; b++) begin
              longint unsigned a = raddr_q + b;
              rd[8*b +: 8] = (a < MEM_BYTES) ? mem[a] : 8'h00;
            end
            RDATA <= rd;
            if (rlen_q == 8'd0) begin
              if (RREADY) rst_q <= R_IDLE;
            end else begin
              raddr_q <= raddr_q + (1 << rsize_q);
              rlen_q  <= rlen_q - 8'd1;
            end
          end
          default: rst_q <= R_IDLE;
        endcase
      end
    end
  endmodule

  // DVFS stub models (used only if exported by DUT)

  module pmic_model #(
    parameter int ACK_DELAY_CYC   = 2,
    parameter int READY_DELAY_CYC = 1000
  )(
    input  logic       clk_i, rstn_i,
    input  logic       vreq_valid_i,
    input  logic [7:0] vsel_i,
    input  logic       vreq_up_i,
    output logic       vreq_ack_o,
    output logic       vready_o
  );
    int ack_cnt, ready_cnt; logic in_req;
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin vreq_ack_o<=0; vready_o<=0; ack_cnt<=0; ready_cnt<=0; in_req<=0; end
      else begin
        if (vreq_valid_i && !in_req) begin in_req<=1; ack_cnt<=ACK_DELAY_CYC; ready_cnt<=READY_DELAY_CYC; vreq_ack_o<=0; vready_o<=0; end
        if (in_req && ack_cnt>0) begin ack_cnt<=ack_cnt-1; if (ack_cnt==1) vreq_ack_o<=1; end
        if (in_req && ready_cnt>0) begin ready_cnt<=ready_cnt-1; if (ready_cnt==1) begin vready_o<=1; in_req<=0; end end
        if (!vreq_valid_i) vreq_ack_o<=0;
      end
    end
  endmodule

  module pll_model #(
    parameter int ACK_DELAY_CYC  = 2,
    parameter int LOCK_DELAY_CYC = 2000
  )(
    input  logic        clk_i, rstn_i,
    input  logic        freq_req_i,
    input  logic [15:0] freq_code_i,
    output logic        pll_ack_o,
    output logic        pll_lock_o
  );
    int ack_cnt, lock_cnt; logic in_req;
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin pll_ack_o<=0; pll_lock_o<=0; ack_cnt<=0; lock_cnt<=0; in_req<=0; end
      else begin
        if (freq_req_i && !in_req) begin in_req<=1; ack_cnt<=ACK_DELAY_CYC; lock_cnt<=LOCK_DELAY_CYC; pll_ack_o<=0; pll_lock_o<=0; end
        if (in_req && ack_cnt>0) begin ack_cnt<=ack_cnt-1; if (ack_cnt==1) pll_ack_o<=1; end
        if (in_req && lock_cnt>0) begin lock_cnt<=lock_cnt-1; if (lock_cnt==1) begin pll_lock_o<=1; in_req<=0; end end
        if (!freq_req_i) pll_ack_o<=0;
      end
    end
  endmodule

endmodule

`default_nettype wire
