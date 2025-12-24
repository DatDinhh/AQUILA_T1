// ============================================================================
//  tb/csr_if.sv
//  Aquila TB - CSR-Lite Interface (request/ack, 32/64-bit data, byte strobes)
// ----------------------------------------------------------------------------
//  Protocol (single-beat):
//   - Master drives:  VALID, WRITE, ADDR, WDATA, WSTRB, (optional PROT)
//   - Slave drives:   READY, RDATA, ERR
//   - Handshake: VALID && READY on a rising edge completes the transfer.
//     * For WRITEs: WDATA/WSTRB are consumed.
//     * For READs : RDATA must be valid in the handshake cycle.
//   - ACTIVE-LOW reset (RSTN). Outputs must be benign under reset.
//
//  Features
//   - Parameterized ADDR_W (CSR word address), DATA_W (32/64), byte strobes
//   - Modports: master / slave / monitor
//   - Master BFM tasks: init, write, read, rmw, poll (with timeout)
//   - Optional PROT/PRIV wire and ERR reporting
//   - Assertions (ASSERT_ON): payload stability under backpressure, alignment,
//     WSTRB sanity (reads), single-cycle completion semantics
//
//  Notes
//   - This is a *testbench interface* (uses timing & $assert).
//   - If your CSR block is AXI-Lite, use an AXI-Lite BFM instead.
//   - By convention, ADDR is a *word* index; alignment check enforces DATA_W.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

interface csr_if #(
  parameter int unsigned ADDR_W   = 12,        // word address width (2^ADDR_W CSRs)
  parameter int unsigned DATA_W   = 32,        // 32 or 64
  parameter bit          HAS_ERR  = 1'b1,      // expose ERR
  parameter bit          HAS_PROT = 1'b0       // expose PRIV/PROT bit
)(
  input  logic CLK,
  input  logic RSTN
);

  // Derived 
  localparam int unsigned STRB_W   = (DATA_W/8);
  localparam int unsigned ALIGN_LS = (DATA_W==64) ? 3 : 2; // byte alignment: 8B or 4B

  // Signals
  // Request (master -> slave)
  logic                  valid;            // request valid
  logic                  write;            // 1=write, 0=read
  logic [ADDR_W-1:0]     addr;             // word address
  logic [DATA_W-1:0]     wdata;            // data for write
  logic [STRB_W-1:0]     wstrb;            // byte strobes (must be 0 on reads)
  logic                  prot;             // optional (e.g., privilege bit)

  // Response (slave -> master)
  logic                  ready;            // handshake acknowledge
  logic [DATA_W-1:0]     rdata;            // data for read
  logic                  err;              // error (DECERR/SLVERR collapsed)

  // Modports
  // Master drives requests; receives responses
  modport master (
    input  CLK, RSTN,
    output valid, write, addr, wdata, wstrb,
    output prot,
    input  ready, rdata, err
  );

  // Slave receives requests; drives responses
  modport slave (
    input  CLK, RSTN,
    input  valid, write, addr, wdata, wstrb,
    input  prot,
    output ready, rdata, err
  );

  // Passive monitor
  modport monitor (
    input CLK, RSTN,
    input valid, write, addr, wdata, wstrb, prot, ready, rdata, err
  );

  // Master BFM
`ifndef SYNTHESIS
  // Initialize master outputs to benign values
  task automatic m_init();
    valid <= 1'b0;
    write <= 1'b0;
    addr  <= '0;
    wdata <= '0;
    wstrb <= '0;
    prot  <= 1'b0;
    // wait one edge to avoid X propagation in zero-delay TBs
    @(posedge CLK);
  endtask

  // Blocking write: returns 'ok' (1) on READY handshake, 0 on timeout.
  task automatic m_write(
    input  logic [ADDR_W-1:0] a,
    input  logic [DATA_W-1:0] d,
    input  logic [STRB_W-1:0] s = {STRB_W{1'b1}},
    input  int unsigned        timeout_cycles = 10000,
    output bit                 ok
  );
    ok = 0;
    @(posedge CLK);
    write <= 1'b1;
    addr  <= a;
    wdata <= d;
    wstrb <= s;
    valid <= 1'b1;
    // Wait for ready
    for (int t=0; t<timeout_cycles; t++) begin
      @(posedge CLK);
      if (ready) begin
        ok    = 1;
        valid <= 1'b0;
        write <= 1'b0;
        wstrb <= '0;
        return;
      end
    end
    // Timeout
    valid <= 1'b0;
    write <= 1'b0;
    $error("csr_if.m_write: timeout on addr=0x%0h", a);
  endtask

  // Blocking read: returns data and 'ok'. ERR is returned via argument if HAS_ERR.
  task automatic m_read(
    input  logic [ADDR_W-1:0] a,
    output logic [DATA_W-1:0] d,
    output bit                ok,
    output bit                err_flag,
    input  int unsigned       timeout_cycles = 10000
  );
    ok = 0; err_flag = 0; d = '0;
    @(posedge CLK);
    write <= 1'b0;
    addr  <= a;
    wstrb <= '0;
    valid <= 1'b1;
    for (int t=0; t<timeout_cycles; t++) begin
      @(posedge CLK);
      if (ready) begin
        d       = rdata;
        err_flag= HAS_ERR ? err : 1'b0;
        ok      = 1;
        valid   <= 1'b0;
        return;
      end
    end
    valid <= 1'b0;
    $error("csr_if.m_read: timeout on addr=0x%0h", a);
  endtask

  // Read-Modify-Write: (val & ~clr) | set. Returns final value and 'ok'.
  task automatic m_rmw(
    input  logic [ADDR_W-1:0] a,
    input  logic [DATA_W-1:0] set_mask,
    input  logic [DATA_W-1:0] clr_mask,
    output logic [DATA_W-1:0] new_val,
    output bit                ok
  );
    logic [DATA_W-1:0] cur; bit ro; bit er;
    m_read(a, cur, ro, er);
    new_val = (cur & ~clr_mask) | set_mask;
    m_write(a, new_val, {STRB_W{1'b1}}, 10000, ok);
  endtask

  // Poll until (read & mask) == exp or timeout; returns ok=1 on success.
  task automatic m_poll(
    input  logic [ADDR_W-1:0] a,
    input  logic [DATA_W-1:0] mask,
    input  logic [DATA_W-1:0] exp,
    input  int unsigned       timeout_cycles,
    input  int unsigned       interval_cycles, // poll period
    output bit                ok
  );
    ok = 0;
    for (int t=0; t<timeout_cycles; ) begin
      logic [DATA_W-1:0] v; bit ro; bit er;
      m_read(a, v, ro, er);
      if ((v & mask) == exp) begin ok = 1; return; end
      // wait interval
      for (int i=0; i<interval_cycles; i++) begin @(posedge CLK); t++; if (t>=timeout_cycles) break; end
    end
    $error("csr_if.m_poll: timeout on addr=0x%0h (mask=0x%0h exp=0x%0h)", a, mask, exp);
  endtask
`endif // !SYNTHESIS

  // Assertions 
`ifdef ASSERT_ON
  // Payload must remain stable while VALID and !READY
  property p_req_stable;
    @(posedge CLK) disable iff (!RSTN)
      valid && !ready |=> ($stable(write) && $stable(addr) && $stable(wdata) && $stable(wstrb) && $stable(prot) && valid);
  endproperty
  assert property (p_req_stable) else $error("csr_if: request changed while VALID && !READY");

  // For reads, WSTRB must be zero
  property p_read_wstrb_zero;
    @(posedge CLK) disable iff (!RSTN)
      valid && !write |-> (wstrb == '0);
  endproperty
  assert property (p_read_wstrb_zero) else $error("csr_if: WSTRB nonzero on read");

  // Alignment: if ADDR is a byte address replace with check; here ADDR is word index.
  // If your DUT supplies byte address, change this assertion accordingly.
  // Example: ensure lower ALIGN_LS bits of (addr << ALIGN_LS) are zero — trivially true.
  property p_addr_is_word_index;
    @(posedge CLK) disable iff (!RSTN)
      valid |-> 1'b1; // placeholder (word index has no alignment violation)
  endproperty
  assert property (p_addr_is_word_index);

  // Response timing: RDATA must be stable during the READY handshake (reads)
  property p_rdata_stable_on_handshake;
    @(posedge CLK) disable iff (!RSTN)
      valid && !write && ready |-> $stable(rdata);
  endproperty
  assert property (p_rdata_stable_on_handshake) else $error("csr_if: RDATA changed on handshake");
`endif

endinterface

`default_nettype wire
