// ============================================================================
//  tb/axi4_if.sv
//  Aquila TB — AXI4 Interface (AMBA 4, full)
// ----------------------------------------------------------------------------
//  Features
//   - Parameterized widths: ID/ADDR/DATA/USER
//   - All five AXI channels with standard sideband signals
//   - Modports: master, slave, monitor (passive)
//   - Helpful typedefs/constants (burst/resp enums)
//   - TB-only master BFM tasks for simple burst read/write
//   - Optional protocol SVAs (ASSERT_ON) for signal stability under backpressure,
//     address alignment, and legal burst encodings
//
//  Notes
//   - Testbench utility (not for synthesis).
//   - If not using USER fields, keep USER_W=1 and ignore them.
//   - DATA_W must be a multiple of 8; STRB_W is derived as DATA_W/8.
// ----------------------------------------------------------------------------
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

interface axi4_if #(
  parameter int unsigned ID_W   = 4,
  parameter int unsigned ADDR_W = 40,
  parameter int unsigned DATA_W = 256,
  parameter int unsigned USER_W = 1
)(
  input  logic ACLK,
  input  logic ARESETn  // Active-low reset in AXI domain
);

  // Derived
  localparam int unsigned STRB_W   = (DATA_W/8);
  localparam int unsigned SIZE_MAX = (STRB_W > 0) ? $clog2(STRB_W) : 1;

  // Static checks (TB messages)
  initial begin
    if ((DATA_W % 8) != 0) begin
      $error("axi4_if: DATA_W (%0d) must be a multiple of 8.", DATA_W);
    end
  end

  // Typedefs
  typedef logic [ID_W-1:0]     axi_id_t;
  typedef logic [ADDR_W-1:0]   axi_addr_t;
  typedef logic [DATA_W-1:0]   axi_data_t;
  typedef logic [STRB_W-1:0]   axi_strb_t;
  typedef logic [USER_W-1:0]   axi_user_t;

  typedef enum logic [1:0] {
    AXI_BURST_FIXED = 2'b00,
    AXI_BURST_INCR  = 2'b01,
    AXI_BURST_WRAP  = 2'b10
  } axi_burst_e;

  typedef enum logic [1:0] {
    AXI_RESP_OKAY   = 2'b00,
    AXI_RESP_EXOKAY = 2'b01,
    AXI_RESP_SLVERR = 2'b10,
    AXI_RESP_DECERR = 2'b11
  } axi_resp_e;

  // Channel Signals
  // Write address channel (AW)
  axi_id_t     AWID;
  axi_addr_t   AWADDR;
  logic [7:0]  AWLEN;     // Number of beats - 1
  logic [2:0]  AWSIZE;    // log2(bytes per beat)
  logic [1:0]  AWBURST;   // axi_burst_e
  logic        AWLOCK;    // AXI4: 1'b0 for normal
  logic [3:0]  AWCACHE;
  logic [2:0]  AWPROT;
  logic [3:0]  AWQOS;
  logic [3:0]  AWREGION;
  axi_user_t   AWUSER;
  logic        AWVALID;
  logic        AWREADY;

  // Write data channel (W)
  axi_data_t   WDATA;
  axi_strb_t   WSTRB;
  logic        WLAST;
  axi_user_t   WUSER;
  logic        WVALID;
  logic        WREADY;

  // Write response channel (B)
  axi_id_t     BID;
  logic [1:0]  BRESP;     // axi_resp_e
  axi_user_t   BUSER;
  logic        BVALID;
  logic        BREADY;

  // Read address channel (AR)
  axi_id_t     ARID;
  axi_addr_t   ARADDR;
  logic [7:0]  ARLEN;
  logic [2:0]  ARSIZE;
  logic [1:0]  ARBURST;
  logic        ARLOCK;
  logic [3:0]  ARCACHE;
  logic [2:0]  ARPROT;
  logic [3:0]  ARQOS;
  logic [3:0]  ARREGION;
  axi_user_t   ARUSER;
  logic        ARVALID;
  logic        ARREADY;

  // Read data channel (R)
  axi_id_t     RID;
  axi_data_t   RDATA;
  logic [1:0]  RRESP;     // axi_resp_e
  logic        RLAST;
  axi_user_t   RUSER;
  logic        RVALID;
  logic        RREADY;

  // Modports
  // Master drives AW/W/AR; receives B/R
  modport master (
    input  ACLK, ARESETn,

    output AWID, AWADDR, AWLEN, AWSIZE, AWBURST, AWLOCK, AWCACHE, AWPROT, AWQOS, AWREGION, AWUSER, AWVALID,
    input  AWREADY,

    output WDATA, WSTRB, WLAST, WUSER, WVALID,
    input  WREADY,

    input  BID, BRESP, BUSER, BVALID,
    output BREADY,

    output ARID, ARADDR, ARLEN, ARSIZE, ARBURST, ARLOCK, ARCACHE, ARPROT, ARQOS, ARREGION, ARUSER, ARVALID,
    input  ARREADY,

    input  RID, RDATA, RRESP, RLAST, RUSER, RVALID,
    output RREADY
  );

  // Slave receives AW/W/AR; drives B/R
  modport slave (
    input  ACLK, ARESETn,

    input  AWID, AWADDR, AWLEN, AWSIZE, AWBURST, AWLOCK, AWCACHE, AWPROT, AWQOS, AWREGION, AWUSER, AWVALID,
    output AWREADY,

    input  WDATA, WSTRB, WLAST, WUSER, WVALID,
    output WREADY,

    output BID, BRESP, BUSER, BVALID,
    input  BREADY,

    input  ARID, ARADDR, ARLEN, ARSIZE, ARBURST, ARLOCK, ARCACHE, ARPROT, ARQOS, ARREGION, ARUSER, ARVALID,
    output ARREADY,

    output RID, RDATA, RRESP, RLAST, RUSER, RVALID,
    input  RREADY
  );

  // Passive monitor: all inputs
  modport monitor (
    input  ACLK, ARESETn,
    input  AWID, AWADDR, AWLEN, AWSIZE, AWBURST, AWLOCK, AWCACHE, AWPROT, AWQOS, AWREGION, AWUSER, AWVALID, AWREADY,
    input  WDATA, WSTRB, WLAST, WUSER, WVALID, WREADY,
    input  BID, BRESP, BUSER, BVALID, BREADY,
    input  ARID, ARADDR, ARLEN, ARSIZE, ARBURST, ARLOCK, ARCACHE, ARPROT, ARQOS, ARREGION, ARUSER, ARVALID, ARREADY,
    input  RID, RDATA, RRESP, RLAST, RUSER, RVALID, RREADY
  );

  // Helpers
`ifndef SYNTHESIS
  // Address alignment helper
  function automatic bit addr_aligned(input axi_addr_t a, input logic [2:0] size);
    axi_addr_t mask;
    mask = (axi_addr_t'(1) << size) - 1;
    return ((a & mask) == '0);
  endfunction
`endif

  // Assertions
`ifdef ASSERT_ON
  // Stable address/control under backpressure (AW)
  // Once AWVALID is asserted, AW* must remain stable until AWREADY.
  property p_aw_stable;
    @(posedge ACLK) disable iff (!ARESETn)
      AWVALID && !AWREADY |=> ( $stable(AWID) && $stable(AWADDR) && $stable(AWLEN) &&
                                $stable(AWSIZE) && $stable(AWBURST) && $stable(AWLOCK) &&
                                $stable(AWCACHE) && $stable(AWPROT) && $stable(AWQOS) &&
                                $stable(AWREGION) && $stable(AWUSER) && AWVALID );
  endproperty
  assert property (p_aw_stable) else $error("AXI4 AW channel changed while AWVALID && !AWREADY");

  // Stable WDATA/WSTRB/WLAST under backpressure
  property p_w_stable;
    @(posedge ACLK) disable iff (!ARESETn)
      WVALID && !WREADY |=> ( $stable(WDATA) && $stable(WSTRB) && $stable(WLAST) && $stable(WUSER) && WVALID );
  endproperty
  assert property (p_w_stable) else $error("AXI4 W channel changed while WVALID && !WREADY");

  // Stable AR* under backpressure
  property p_ar_stable;
    @(posedge ACLK) disable iff (!ARESETn)
      ARVALID && !ARREADY |=> ( $stable(ARID) && $stable(ARADDR) && $stable(ARLEN) &&
                                $stable(ARSIZE) && $stable(ARBURST) && $stable(ARLOCK) &&
                                $stable(ARCACHE) && $stable(ARPROT) && $stable(ARQOS) &&
                                $stable(ARREGION) && $stable(ARUSER) && ARVALID );
  endproperty
  assert property (p_ar_stable) else $error("AXI4 AR channel changed while ARVALID && !ARREADY");

  // Stable R* under backpressure (slave must hold data)
  property p_r_stable;
    @(posedge ACLK) disable iff (!ARESETn)
      RVALID && !RREADY |=> ( $stable(RID) && $stable(RDATA) && $stable(RRESP) &&
                              $stable(RLAST) && $stable(RUSER) && RVALID );
  endproperty
  assert property (p_r_stable) else $error("AXI4 R channel changed while RVALID && !RREADY");

  // Stable B* under backpressure (slave must hold)
  property p_b_stable;
    @(posedge ACLK) disable iff (!ARESETn)
      BVALID && !BREADY |=> ( $stable(BID) && $stable(BRESP) && $stable(BUSER) && BVALID );
  endproperty
  assert property (p_b_stable) else $error("AXI4 B channel changed while BVALID && !BREADY");

  // Legal burst encoding (not 2'b11)
  property p_legal_burst_aw;
    @(posedge ACLK) disable iff (!ARESETn)
      AWVALID && AWREADY |-> (AWBURST != 2'b11);
  endproperty
  assert property (p_legal_burst_aw) else $error("AXI4 illegal AWBURST=2'b11");
  property p_legal_burst_ar;
    @(posedge ACLK) disable iff (!ARESETn)
      ARVALID && ARREADY |-> (ARBURST != 2'b11);
  endproperty
  assert property (p_legal_burst_ar) else $error("AXI4 illegal ARBURST=2'b11");

  // Address alignment check for INCR/FIXED bursts
  property p_aw_aligned;
    @(posedge ACLK) disable iff (!ARESETn)
      (AWVALID && AWREADY) |-> addr_aligned(AWADDR, AWSIZE);
  endproperty
  assert property (p_aw_aligned) else $error("AXI4 AWADDR not aligned to AWSIZE");
  property p_ar_aligned;
    @(posedge ACLK) disable iff (!ARESETn)
      (ARVALID && ARREADY) |-> addr_aligned(ARADDR, ARSIZE);
  endproperty
  assert property (p_ar_aligned) else $error("AXI4 ARADDR not aligned to ARSIZE");

  // Size within DATA width
  property p_aw_size_ok;
    @(posedge ACLK) disable iff (!ARESETn)
      (AWVALID && AWREADY) |-> (AWSIZE <= SIZE_MAX);
  endproperty
  assert property (p_aw_size_ok) else $error("AXI4 AWSIZE exceeds DATA_W");
  property p_ar_size_ok;
    @(posedge ACLK) disable iff (!ARESETn)
      (ARVALID && ARREADY) |-> (ARSIZE <= SIZE_MAX);
  endproperty
  assert property (p_ar_size_ok) else $error("AXI4 ARSIZE exceeds DATA_W");
`endif // ASSERT_ON

  // TB Master BFMs
`ifndef SYNTHESIS
  // Initialize master outputs to benign values
  task automatic master_init_outputs();
    AWID    = '0; AWADDR='0; AWLEN=8'd0; AWSIZE=SIZE_MAX[2:0]; AWBURST=AXI_BURST_INCR; AWLOCK=1'b0;
    AWCACHE = 4'h0; AWPROT=3'h0; AWQOS=4'h0; AWREGION=4'h0; AWUSER='0; AWVALID=1'b0;
    WDATA   = '0; WSTRB = '0; WLAST = 1'b0; WUSER='0; WVALID=1'b0;
    BREADY  = 1'b0;
    ARID    = '0; ARADDR='0; ARLEN=8'd0; ARSIZE=SIZE_MAX[2:0]; ARBURST=AXI_BURST_INCR; ARLOCK=1'b0;
    ARCACHE = 4'h0; ARPROT=3'h0; ARQOS=4'h0; ARREGION=4'h0; ARUSER='0; ARVALID=1'b0;
    RREADY  = 1'b0;
  endtask

  // Single-burst write (full-width beats), returns BRESP
  // beats = AWLEN+1 (1..256). Uses default SIZE based on DATA_W.
  task automatic m_write_burst(
      input  axi_addr_t               addr,
      input  int unsigned             beats,
      input  axi_data_t               data_beats[],
      input  axi_strb_t               strb_beats[],
      output axi_resp_e               resp
  );
    if (beats < 1 || beats > 256) begin
      $fatal(1, "AXI BFM: m_write_burst invalid beats=%0d", beats);
    end
    // Address phase
    @(posedge ACLK);
    AWID    <= '0;
    AWADDR  <= addr;
    AWLEN   <= beats-1;
    AWSIZE  <= SIZE_MAX[2:0];
    AWBURST <= AXI_BURST_INCR;
    AWLOCK  <= 1'b0;
    AWCACHE <= 4'h3;   // normal non-coherent, modifiable bufferable (TB hint)
    AWPROT  <= 3'h0;
    AWQOS   <= 4'h0;
    AWREGION<= 4'h0;
    AWUSER  <= '0;
    AWVALID <= 1'b1;
    // Wait for handshake
    do @(posedge ACLK); while (!AWREADY);
    AWVALID <= 1'b0;

    // Data phase
    for (int i=0; i<beats; i++) begin
      @(posedge ACLK);
      WDATA  <= data_beats[i];
      WSTRB  <= (strb_beats.size() == beats) ? strb_beats[i] : '1;
      WLAST  <= (i == beats-1);
      WUSER  <= '0;
      WVALID <= 1'b1;
      do @(posedge ACLK); while (!WREADY);
      WVALID <= 1'b0;
    end

    // Response
    @(posedge ACLK);
    BREADY <= 1'b1;
    do @(posedge ACLK); while (!BVALID);
    resp   = axi_resp_e'(BRESP);
    BREADY <= 1'b0;
  endtask

  // Single-burst read (full-width beats), returns data and RESP (per-last-beat)
  task automatic m_read_burst(
      input  axi_addr_t               addr,
      input  int unsigned             beats,
      output axi_data_t               data_beats[],
      output axi_resp_e               resp_last
  );
    if (beats < 1 || beats > 256) begin
      $fatal(1, "AXI BFM: m_read_burst invalid beats=%0d", beats);
    end
    data_beats = new[beats];

    // Address phase
    @(posedge ACLK);
    ARID    <= '0;
    ARADDR  <= addr;
    ARLEN   <= beats-1;
    ARSIZE  <= SIZE_MAX[2:0];
    ARBURST <= AXI_BURST_INCR;
    ARLOCK  <= 1'b0;
    ARCACHE <= 4'h3;
    ARPROT  <= 3'h0;
    ARQOS   <= 4'h0;
    ARREGION<= 4'h0;
    ARUSER  <= '0;
    ARVALID <= 1'b1;
    do @(posedge ACLK); while (!ARREADY);
    ARVALID <= 1'b0;

    // Data receive
    RREADY <= 1'b1;
    for (int i=0; i<beats; i++) begin
      do @(posedge ACLK); while (!RVALID);
      data_beats[i] = RDATA;
      resp_last     = axi_resp_e'(RRESP);
      if (i < beats-1 && RLAST) begin
        $error("AXI BFM: Unexpected RLAST early (i=%0d of %0d)", i, beats);
      end
      if (i == beats-1 && !RLAST) begin
        $error("AXI BFM: Missing RLAST on final beat.");
      end
    end
    RREADY <= 1'b0;
  endtask
`endif // !SYNTHESIS

endinterface

`default_nettype wire
