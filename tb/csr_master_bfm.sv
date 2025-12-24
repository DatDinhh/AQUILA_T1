// ============================================================================
//  tb/csr_master_bfm.sv
//  Aquila TB - CSR-Lite Master BFM (pairs with tb/csr_if.sv)
// ----------------------------------------------------------------------------
//  Bus semantics (single-beat request/ack):
//    - Master drives: valid, write (1=WR / 0=RD), addr, wdata, wstrb
//    - Slave  drives: ready, rdata, err (optional; ignored if HAS_ERR == 0)
//    - Handshake occurs on the cycle valid && ready == 1.
//      * WRITE: wdata/wstrb are consumed.
//      * READ : rdata valid in the handshake cycle.
//    - Master must hold request fields stable while !ready.
//
//  Features:
//   - Blocking tasks: write(), read(), rmw(), poll(), wait_bits_set/clear(),
//     burst_write()
//   - Configurable defaults: timeout, verbosity, fatal-on-error
//   - Optional idle insertion before each request (stress ready/valid hold)
//   - Auto-inits outputs, well-behaved across async reset
//
//  Usage (example):
//    csr_if #(.ADDR_W(12), .DATA_W(32), .ADDR_IS_BYTES(0)) csr_bus (.clk(clk_sys), .rstn(rstn));
//    csr_master_bfm #(.ADDR_W(12), .DATA_W(32)) csr_mst (.csr(csr_bus));
//
//    initial begin
//      csr_mst.init();
//      bit ok;
//      csr_mst.write('h010, 32'h1, '1, ok);
//      logic [31:0] v; bit er;
//      csr_mst.read('h014, v, ok, er);
//      csr_mst.wait_bits_set('h018, 32'h1, 100000);
//    end
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module csr_master_bfm #(
  parameter int unsigned ADDR_W          = 12,
  parameter int unsigned DATA_W          = 32,   // 32 or 64 typical
  parameter bit          ADDR_IS_BYTES   = 0,    // 0: word addr (CSR index), 1: byte addr
  parameter bit          HAS_ERR         = 1,    // if slave drives .err
  parameter int unsigned DEFAULT_TIMEOUT = 100000, // cycles
  parameter int unsigned MAX_PRE_IDLE    = 0,    // random 0..N idle cycles before each txn
  parameter int unsigned VERBOSE         = 1,    // 0-silent, 1-info, 2-verbose
  parameter bit          FATAL_ON_ERR    = 0     // $fatal on err from slave
)(
  // Bind to CSR interface (master modport)
  csr_if.master csr
);

  // Derived
  localparam int unsigned STRB_W     = (DATA_W/8);
  localparam int unsigned ALIGN_BITS = (ADDR_IS_BYTES) ? $clog2(STRB_W) : 0;

  // Sanity
  initial begin
    if ((DATA_W % 8) != 0) $fatal(1, "csr_master_bfm: DATA_W must be multiple of 8.");
  end

  // Internal state
  bit initialized;

  // Public tasks
  // Call once after elaboration (or rely on reset-driven init inside always_ff)
  task automatic init();
    initialized = 1'b1;
    force_idle_outputs();
    if (VERBOSE > 0) $display("[%0t] csr_master_bfm: init complete.", $time);
  endtask

  // Blocking WRITE (single word)
  //  a: address; d: data; s: byte strobes (default all 1s)
  //  ok: handshake observed; err_flag: slave err (if enabled)
  task automatic write(
      input  logic [ADDR_W-1:0] a,
      input  logic [DATA_W-1:0] d,
      input  logic [STRB_W-1:0] s = {STRB_W{1'b1}},
      output bit                ok,
      output bit                err_flag
  );
    ok = 0; err_flag = 0;
    pre_idle_delay();

    // Optionally check alignment if using byte addressing
    if (ADDR_IS_BYTES && (ALIGN_BITS>0) && (a[ALIGN_BITS-1:0] != '0)) begin
      $error("csr_master_bfm.write: unaligned byte address 0x%0h (DATA_W=%0d)", a, DATA_W);
    end

    // Drive request
    @(posedge csr.clk);
    csr.addr  <= a;
    csr.wdata <= d;
    csr.wstrb <= s;
    csr.write <= 1'b1;
    csr.valid <= 1'b1;

    // Wait for handshake or timeout
    int unsigned t = 0;
    while (!csr.ready) begin
      @(posedge csr.clk);
      if (!csr.rstn) begin force_idle_outputs(); t = 0; @(posedge csr.rstn); @(posedge csr.clk); end
      t++;
      if (t >= DEFAULT_TIMEOUT) begin
        csr.valid <= 1'b0; csr.write <= 1'b0; csr.wstrb <= '0;
        $error("csr_master_bfm.write: timeout @0x%0h (data=0x%0h)", a, d);
        return;
      end
    end

    // Handshake this cycle
    ok       = 1'b1;
    err_flag = (HAS_ERR) ? csr.err : 1'b0;

    if (VERBOSE > 1) $display("[%0t] CSR-W @0x%0h data=0x%0h wstrb=0x%0h err=%0b",
                               $time, a, d, s, err_flag);

    if (err_flag) begin
      if (FATAL_ON_ERR) $fatal(1, "csr_master_bfm.write: slave error @0x%0h", a);
      else               $error   ("csr_master_bfm.write: slave error @0x%0h", a);
    end

    // Deassert request on next edge
    @(posedge csr.clk);
    csr.valid <= 1'b0;
    csr.write <= 1'b0;
    csr.wstrb <= '0;
  endtask

  // Blocking READ (single word)
  task automatic read(
      input  logic [ADDR_W-1:0] a,
      output logic [DATA_W-1:0] d,
      output bit                ok,
      output bit                err_flag
  );
    d = '0; ok = 0; err_flag = 0;
    pre_idle_delay();

    if (ADDR_IS_BYTES && (ALIGN_BITS>0) && (a[ALIGN_BITS-1:0] != '0)) begin
      $error("csr_master_bfm.read: unaligned byte address 0x%0h (DATA_W=%0d)", a, DATA_W);
    end

    @(posedge csr.clk);
    csr.addr  <= a;
    csr.wdata <= '0;
    csr.wstrb <= '0;
    csr.write <= 1'b0;
    csr.valid <= 1'b1;

    // Wait for handshake or timeout
    int unsigned t = 0;
    while (!csr.ready) begin
      @(posedge csr.clk);
      if (!csr.rstn) begin force_idle_outputs(); t = 0; @(posedge csr.rstn); @(posedge csr.clk); end
      t++;
      if (t >= DEFAULT_TIMEOUT) begin
        csr.valid <= 1'b0;
        $error("csr_master_bfm.read: timeout @0x%0h", a);
        return;
      end
    end

    // Handshake cycle: capture data/err
    d        = csr.rdata;
    ok       = 1'b1;
    err_flag = (HAS_ERR) ? csr.err : 1'b0;

    if (VERBOSE > 1) $display("[%0t] CSR-R @0x%0h -> 0x%0h err=%0b", $time, a, d, err_flag);

    if (err_flag) begin
      if (FATAL_ON_ERR) $fatal(1, "csr_master_bfm.read: slave error @0x%0h", a);
      else               $error   ("csr_master_bfm.read: slave error @0x%0h", a);
    end

    @(posedge csr.clk);
    csr.valid <= 1'b0;
  endtask

  // Read-Modify-Write: new = (old & ~clr_mask) | set_mask
  task automatic rmw(
      input  logic [ADDR_W-1:0] a,
      input  logic [DATA_W-1:0] set_mask,
      input  logic [DATA_W-1:0] clr_mask,
      output logic [DATA_W-1:0] old_val,
      output logic [DATA_W-1:0] new_val,
      output bit                ok
  );
    bit r_ok, w_ok, r_er, w_er;
    read(a, old_val, r_ok, r_er);
    new_val = (old_val & ~clr_mask) | set_mask;
    write(a, new_val, {STRB_W{1'b1}}, w_ok, w_er);
    ok = r_ok & w_ok & ~r_er & ~w_er;
  endtask

  // Poll until (read & mask) == exp or timeout; returns ok=1 on success
  task automatic poll(
      input  logic [ADDR_W-1:0] a,
      input  logic [DATA_W-1:0] mask,
      input  logic [DATA_W-1:0] exp,
      input  int unsigned       timeout_cycles,
      input  int unsigned       interval_cycles,
      output bit                ok
  );
    ok = 0;
    for (int unsigned t = 0; t < timeout_cycles; ) begin
      logic [DATA_W-1:0] v; bit rok, er;
      read(a, v, rok, er);
      if (rok && !er && ((v & mask) == exp)) begin ok = 1; return; end
      // wait interval
      for (int i=0; i<interval_cycles; i++) begin @(posedge csr.clk); t++; if (t>=timeout_cycles) break; end
    end
    if (VERBOSE > 0) $error("csr_master_bfm.poll: timeout @0x%0h mask=0x%0h exp=0x%0h", a, mask, exp);
  endtask

  // Wait for bits set/clear helpers (fatal on timeout)
  task automatic wait_bits_set(
      input logic [ADDR_W-1:0] a,
      input logic [DATA_W-1:0] mask,
      input int unsigned       timeout_cycles = DEFAULT_TIMEOUT
  );
    bit ok;
    poll(a, mask, mask, timeout_cycles, /*interval*/10, ok);
    if (!ok) $fatal(1, "csr_master_bfm.wait_bits_set: timeout @0x%0h mask=0x%0h", a, mask);
  endtask

  task automatic wait_bits_clear(
      input logic [ADDR_W-1:0] a,
      input logic [DATA_W-1:0] mask,
      input int unsigned       timeout_cycles = DEFAULT_TIMEOUT
  );
    bit ok;
    poll(a, mask, '0, timeout_cycles, /*interval*/10, ok);
    if (!ok) $fatal(1, "csr_master_bfm.wait_bits_clear: timeout @0x%0h mask=0x%0h", a, mask);
  endtask

  // Convenience: write sequential CSRs (step = 1 word or DATA_W/8 bytes)
  task automatic burst_write(
      input logic [ADDR_W-1:0] base,
      input logic [DATA_W-1:0] data_q[],
      input int unsigned       step = (ADDR_IS_BYTES ? STRB_W : 1)
  );
    logic [ADDR_W-1:0] a = base;
    foreach (data_q[i]) begin
      bit ok, er;
      write(a, data_q[i], {STRB_W{1'b1}}, ok, er);
      a = a + step[ADDR_W-1:0];
    end
  endtask

  // Private helpers
  task automatic force_idle_outputs();
    csr.valid <= 1'b0;
    csr.write <= 1'b0;
    csr.addr  <= '0;
    csr.wdata <= '0;
    csr.wstrb <= '0;
  endtask

  // Optional random idle cycles before issuing a request
  task automatic pre_idle_delay();
    if (MAX_PRE_IDLE == 0) return;
    int unsigned n = $urandom_range(0, MAX_PRE_IDLE);
    if (VERBOSE > 1 && n != 0) $display("[%0t] csr_master_bfm: pre-idle %0d cycles", $time, n);
    repeat (n) @(posedge csr.clk);
  endtask

  // Reset handling
  // Keep outputs benign under reset; auto-init after first reset release.
  always_ff @(negedge csr.rstn or posedge csr.clk) begin
    if (!csr.rstn) begin
      force_idle_outputs();
    end
  end

  initial begin
    initialized = 1'b0;
    force_idle_outputs();
    // Auto-init after first reset release if user did not call init()
    @(posedge csr.rstn);
    if (!initialized) begin
      initialized = 1'b1;
      if (VERBOSE > 0) $display("[%0t] csr_master_bfm: auto-init after reset.", $time);
    end
  end

  // Assertions
`ifdef ASSERT_ON
  // Hold request stable while !ready (mirrors interface SVA; redundantly helpful)
  property p_req_stable_from_bfm;
    @(posedge csr.clk) disable iff (!csr.rstn)
      csr.valid && !csr.ready |=> ( $stable(csr.write) && $stable(csr.addr) &&
                                    $stable(csr.wdata) && $stable(csr.wstrb) && csr.valid );
  endproperty
  assert property (p_req_stable_from_bfm)
    else $error("csr_master_bfm: request changed while valid && !ready");

  // For reads, wstrb must be 0; for writes, at least one strobe bit when accepted
  property p_read_wstrb_zero;
    @(posedge csr.clk) disable iff (!csr.rstn)
      csr.valid && !csr.write |-> (csr.wstrb == '0);
  endproperty
  assert property (p_read_wstrb_zero) else $error("csr_master_bfm: wstrb nonzero on read");

  property p_write_wstrb_nonzero;
    @(posedge csr.clk) disable iff (!csr.rstn)
      (csr.valid && csr.write && csr.ready) |-> (|csr.wstrb);
  endproperty
  assert property (p_write_wstrb_nonzero) else $error("csr_master_bfm: write accepted with wstrb==0");

  // Optional: alignment when using byte addressing
  if (ADDR_IS_BYTES && (ALIGN_BITS>0)) begin
    property p_addr_aligned_on_accept;
      @(posedge csr.clk) disable iff (!csr.rstn)
        (csr.valid && csr.ready) |-> (csr.addr[ALIGN_BITS-1:0] == '0);
    endproperty
    assert property (p_addr_aligned_on_accept) else
      $error("csr_master_bfm: unaligned byte address accepted (addr=0x%0h)", csr.addr);
  end
`endif

endmodule

`default_nettype wire
