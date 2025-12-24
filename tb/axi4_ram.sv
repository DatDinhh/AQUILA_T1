// ============================================================================
//  tb/axi4_ram.sv
//  Aquila TB - AXI4 RAM / DRAM Behavioral Slave
// ----------------------------------------------------------------------------
//  Features
//   - AXI4 full slave using axi4_if.slave modport
//   - Parametric widths and memory size (byte addressable backing store)
//   - INCR and FIXED bursts (WRAP optional/off by default)
//   - WSTRB-accurate writes; OKAY/SLVERR responses (OOB -> SLVERR)
//   - Independent AR/AW queues; sequential service (no W interleave)
//   - Programmable latencies & randomized back-pressure
//   - Optional 4KB boundary checks (assertions)
//   - TB tasks: load/read hex, poke/peek bytes, flip bits, zero range
//
//  Notes
//   - This is a *testbench model* (uses $urandom and time-based behavior).
//   - Simplification: write channel is non-interleaved (W beats must follow AW).
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi4_ram #(
  // Parameters
  parameter int unsigned ADDR_W        = 40,
  parameter int unsigned DATA_W        = 256,
  parameter int unsigned ID_W          = 4,

  // Backing store size (bytes)
  parameter longint unsigned MEM_BYTES = 128*1024*1024,

  // Supported burst types
  parameter bit SUPPORT_FIXED          = 1,
  parameter bit SUPPORT_INCR           = 1,
  parameter bit SUPPORT_WRAP           = 0,  // not implemented; set 1 to allow but it will $fatal

  // Queue depths (requests)
  parameter int unsigned AR_QDEPTH     = 16,
  parameter int unsigned AW_QDEPTH     = 16,

  // Latencies (cycles in ACLK domain)
  parameter int unsigned READ_LATENCY  = 0,   // AR -> first RVALID
  parameter int unsigned BRESP_LATENCY = 0,   // WLAST -> BVALID

  // Randomized back-pressure (percent chances per cycle)
  parameter int unsigned STALL_ARREADY_PCT = 0,
  parameter int unsigned STALL_AWREADY_PCT = 0,
  parameter int unsigned STALL_WREADY_PCT  = 0,
  parameter int unsigned STALL_RVALID_PCT  = 0,

  // Protocol checks
  parameter bit EN_ASSERTIONS          = 1'b1, // +define+ASSERT_ON also needed to enable
  parameter bit STRICT_4KB             = 1'b1  // assert bursts don't cross 4KB page
)(
  input  logic ACLK,
  input  logic ARESETn,

  // AXI interface
  axi4_if.slave axi
);

  // Derived
  localparam int unsigned STRB_W = (DATA_W/8);
  initial begin
    if (DATA_W % 8 != 0) $fatal(1, "axi4_ram: DATA_W must be multiple of 8");
  end

  // Backing store
  byte mem [0:MEM_BYTES-1];

  // Request descriptors
  typedef struct packed {
    logic [ID_W-1:0]   id;
    logic [ADDR_W-1:0] addr;
    logic [7:0]        len;     // beats - 1
    logic [2:0]        size;    // bytes/beat = 2**size
    logic [1:0]        burst;   // 00=FIXED, 01=INCR, 10=WRAP
  } req_t;

  req_t arq [$];
  req_t awq [$];

  // Utilities
  function automatic int unsigned bytes_per_beat(input logic [2:0] size);
    return (1 << size);
  endfunction

  function automatic logic is_oob_addr(input longint unsigned a);
    return (a >= MEM_BYTES);
  endfunction

  // Simple RNG gate
  function automatic bit stall(input int unsigned pct);
    if (pct == 0) return 1'b0;
    int unsigned r = $urandom_range(0, 99);
    return (r < pct);
  endfunction

  // Reset
  task automatic clear_all();
    axi.AWREADY <= 1'b0;
    axi.WREADY  <= 1'b0;
    axi.BVALID  <= 1'b0;
    axi.BID     <= '0;
    axi.BRESP   <= 2'b00;
    axi.BUSER   <= '0;

    axi.ARREADY <= 1'b0;
    axi.RVALID  <= 1'b0;
    axi.RRESP   <= 2'b00;
    axi.RID     <= '0;
    axi.RDATA   <= '0;
    axi.RLAST   <= 1'b0;
    axi.RUSER   <= '0;

    arq.delete();
    awq.delete();

    wr_active    <= 1'b0;
    wr_beats_rem <= '0;
    wr_addr      <= '0;
    wr_id        <= '0;
    wr_oob_seen  <= 1'b0;
    wr_bytes_per <= '0;

    rd_active    <= 1'b0;
    rd_beats_rem <= '0;
    rd_addr      <= '0;
    rd_id        <= '0;
    rd_bytes_per <= '0;
    rd_latency   <= '0;
  endtask

  // Write path
  logic              wr_active;
  req_t              wr_req;
  logic [ADDR_W-1:0] wr_addr;
  logic [ID_W-1:0]   wr_id;
  logic [7:0]        wr_beats_rem;
  int unsigned       wr_bytes_per;
  bit                wr_oob_seen;

  // AWREADY generation & capture
  always_ff @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      axi.AWREADY <= 1'b0;
    end else begin
      axi.AWREADY <= 1'b0;
      if (awq.size() < AW_QDEPTH && !stall(STALL_AWREADY_PCT)) axi.AWREADY <= 1'b1;

      if (axi.AWVALID && axi.AWREADY) begin
        // Policy checks
        if ((axi.AWBURST == 2'b00 && !SUPPORT_FIXED) ||
            (axi.AWBURST == 2'b01 && !SUPPORT_INCR)  ||
            (axi.AWBURST == 2'b10 && !SUPPORT_WRAP)) begin
          $fatal(1, "axi4_ram: unsupported AWBURST %b", axi.AWBURST);
        end
        if (STRICT_4KB && axi.AWBURST == 2'b01) begin
          int unsigned bytes = (axi.AWLEN + 1) * bytes_per_beat(axi.AWSIZE);
          if ((axi.AWADDR[11:0] + bytes) > 4096)
            $fatal(1, "axi4_ram: INCR write crosses 4KB (addr=%h len=%0d size=%0d)",
                   axi.AWADDR, axi.AWLEN, axi.AWSIZE);
        end
        req_t r;
        r.id    = axi.AWID;
        r.addr  = axi.AWADDR;
        r.len   = axi.AWLEN;
        r.size  = axi.AWSIZE;
        r.burst = axi.AWBURST;
        awq.push_back(r);
      end
    end
  end

  // Start a new write when idle
  always_ff @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      wr_active <= 1'b0;
      wr_beats_rem <= '0;
      wr_oob_seen <= 1'b0;
      wr_bytes_per <= 0;
      axi.WREADY <= 1'b0;
      axi.BVALID <= 1'b0;
    end else begin
      // Consume BRESP when ready
      if (axi.BVALID && axi.BREADY) begin
        axi.BVALID <= 1'b0;
      end

      // Start next AW when idle
      if (!wr_active && axi.BVALID==1'b0 && awq.size()!=0) begin
        wr_req       <= awq.pop_front();
        wr_id        <= wr_req.id;
        wr_addr      <= wr_req.addr;
        wr_beats_rem <= wr_req.len + 8'd1;
        wr_bytes_per <= bytes_per_beat(wr_req.size);
        wr_oob_seen  <= 1'b0;
        wr_active    <= 1'b1;
      end

      // WREADY policy
      axi.WREADY <= 1'b0;
      if (wr_active && !stall(STALL_WREADY_PCT)) axi.WREADY <= 1'b1;

      // Consume W beats
      if (wr_active && axi.WVALID && axi.WREADY) begin
        // Byte-accurate write with WSTRB
        for (int b=0; b<STRB_W; b++) begin
          longint unsigned a = longint'(wr_addr) + b;
          if (axi.WSTRB[b]) begin
            if (!is_oob_addr(a)) begin
              mem[a] = axi.WDATA[8*b +: 8];
            end else begin
              wr_oob_seen = 1'b1;
            end
          end
        end

        // Early/late WLAST checks
        if ((wr_beats_rem == 8'd1) && (axi.WLAST != 1'b1)) begin
          $error("axi4_ram: expected WLAST on final beat.");
        end
        if ((wr_beats_rem != 8'd1) && (axi.WLAST == 1'b1)) begin
          $error("axi4_ram: WLAST asserted early (beats_rem=%0d).", wr_beats_rem);
        end

        // Next beat bookkeeping
        wr_beats_rem <= wr_beats_rem - 8'd1;
        unique case (wr_req.burst)
          2'b00: wr_addr <= wr_addr;                                // FIXED
          2'b01: wr_addr <= wr_addr + wr_bytes_per[ADDR_W-1:0];     // INCR
          2'b10: begin
                   $fatal(1, "axi4_ram: WRAP write not implemented.");
                 end
          default: wr_addr <= wr_addr;
        endcase

        // Finish burst -> produce BVALID after latency
        if (axi.WLAST) begin
          wr_active <= 1'b0;
          // Schedule BRESP
          automatic int cnt = BRESP_LATENCY;
          automatic logic [1:0] resp = wr_oob_seen ? 2'b10 : 2'b00; // SLVERR / OKAY
          automatic logic [ID_W-1:0] bid = wr_id;
          fork
            begin
              while (cnt > 0) begin @(posedge ACLK); cnt--; end
              axi.BID    <= bid;
              axi.BRESP  <= resp;
              axi.BUSER  <= '0;
              axi.BVALID <= 1'b1;
            end
          join_none
        end
      end
    end
  end

  // Read path
  logic              rd_active;
  req_t              rd_req;
  logic [ADDR_W-1:0] rd_addr;
  logic [ID_W-1:0]   rd_id;
  logic [7:0]        rd_beats_rem;
  int unsigned       rd_bytes_per;
  int unsigned       rd_latency;

  // ARREADY generation & capture
  always_ff @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      axi.ARREADY <= 1'b0;
    end else begin
      axi.ARREADY <= 1'b0;
      if (arq.size() < AR_QDEPTH && !stall(STALL_ARREADY_PCT)) axi.ARREADY <= 1'b1;

      if (axi.ARVALID && axi.ARREADY) begin
        if ((axi.ARBURST == 2'b00 && !SUPPORT_FIXED) ||
            (axi.ARBURST == 2'b01 && !SUPPORT_INCR)  ||
            (axi.ARBURST == 2'b10 && !SUPPORT_WRAP)) begin
          $fatal(1, "axi4_ram: unsupported ARBURST %b", axi.ARBURST);
        end
        if (STRICT_4KB && axi.ARBURST == 2'b01) begin
          int unsigned bytes = (axi.ARLEN + 1) * bytes_per_beat(axi.ARSIZE);
          if ((axi.ARADDR[11:0] + bytes) > 4096)
            $fatal(1, "axi4_ram: INCR read crosses 4KB (addr=%h len=%0d size=%0d)",
                   axi.ARADDR, axi.ARLEN, axi.ARSIZE);
        end
        req_t r;
        r.id    = axi.ARID;
        r.addr  = axi.ARADDR;
        r.len   = axi.ARLEN;
        r.size  = axi.ARSIZE;
        r.burst = axi.ARBURST;
        arq.push_back(r);
      end
    end
  end

  // R channel engine
  always_ff @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      axi.RVALID   <= 1'b0;
      axi.RLAST    <= 1'b0;
      rd_active    <= 1'b0;
      rd_latency   <= 0;
      rd_beats_rem <= '0;
    end else begin
      // Start a new read if idle and queued
      if (!rd_active && arq.size()!=0) begin
        rd_req       <= arq.pop_front();
        rd_id        <= rd_req.id;
        rd_addr      <= rd_req.addr;
        rd_beats_rem <= rd_req.len + 8'd1;
        rd_bytes_per <= bytes_per_beat(rd_req.size);
        rd_latency   <= READ_LATENCY;
        rd_active    <= 1'b1;
        axi.RVALID   <= 1'b0;
      end

      // Handle latency countdown
      if (rd_active && (rd_latency > 0)) begin
        rd_latency <= rd_latency - 1;
        axi.RVALID <= 1'b0;
      end

      // When active and latency done: produce beats
      if (rd_active && (rd_latency == 0)) begin
        // If we're not currently driving a valid beat (or it has been accepted),
        // produce / re-drive subject to STALL_RVALID_PCT
        if (!axi.RVALID || (axi.RVALID && axi.RREADY)) begin
          // Optional RVALID throttle
          if (!stall(STALL_RVALID_PCT)) begin
            // Form read beat
            axi.RDATA <= '0;
            bit any_oob = 0;
            for (int b=0; b<STRB_W; b++) begin
              longint unsigned a = longint'(rd_addr) + b;
              if (!is_oob_addr(a)) axi.RDATA[8*b +: 8] <= mem[a];
              else begin
                axi.RDATA[8*b +: 8] <= 8'h00;
                any_oob = 1;
              end
            end
            axi.RID   <= rd_id;
            axi.RRESP <= any_oob ? 2'b10 : 2'b00; // SLVERR / OKAY
            axi.RUSER <= '0;
            axi.RLAST <= (rd_beats_rem == 8'd1);
            axi.RVALID<= 1'b1;
          end else begin
            // Hold RVALID low this cycle (throttle)
            axi.RVALID<= 1'b0;
          end
        end

        // After a handshake, advance address/beat count
        if (axi.RVALID && axi.RREADY) begin
          rd_beats_rem <= rd_beats_rem - 8'd1;
          unique case (rd_req.burst)
            2'b00: rd_addr <= rd_addr;                              // FIXED
            2'b01: rd_addr <= rd_addr + rd_bytes_per[ADDR_W-1:0];   // INCR
            2'b10: begin
                     $fatal(1, "axi4_ram: WRAP read not implemented.");
                   end
            default: rd_addr <= rd_addr;
          endcase

          if (axi.RLAST) begin
            // End of burst
            axi.RVALID <= 1'b0;
            rd_active  <= 1'b0;
          end
        end
      end
    end
  end

  // Reset / init
  always_ff @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
      clear_all();
    end
  end

  // TB helper tasks
`ifndef SYNTHESIS
  // Preload from hex file (byte values). Optional base offset.
  task automatic load_hex(input string filename, input longint unsigned base = 0);
    if (base >= MEM_BYTES) begin
      $error("axi4_ram.load_hex: base 0x%0h out of range (MEM_BYTES=0x%0h)", base, MEM_BYTES);
      return;
    end
    $display("[%0t] axi4_ram: loading hex '%s' at 0x%0h", $time, filename, base);
    $readmemh(filename, mem, base);
  endtask

  // Zero a range (bytes)
  task automatic memset0(input longint unsigned base, input longint unsigned nbytes);
    for (longint unsigned i=0; i<nbytes; i++) begin
      longint unsigned a = base + i;
      if (a < MEM_BYTES) mem[a] = 8'h00;
    end
  endtask

  // Poke/peek helpers (byte granularity)
  task automatic poke8(input longint unsigned addr, input byte val);
    if (addr < MEM_BYTES) mem[addr] = val;
    else $error("axi4_ram.poke8: addr 0x%0h OOB", addr);
  endtask

  task automatic peek8(input longint unsigned addr, output byte val);
    if (addr < MEM_BYTES) val = mem[addr];
    else begin val = 8'h00; $error("axi4_ram.peek8: addr 0x%0h OOB", addr); end
  endtask

  // Flip bits within a byte (for ECC fault injection upstream)
  task automatic flip8(input longint unsigned addr, input byte mask);
    if (addr < MEM_BYTES) mem[addr] ^= mask;
    else $error("axi4_ram.flip8: addr 0x%0h OOB", addr);
  endtask
`endif

  // Assertions
`ifdef ASSERT_ON
  generate if (EN_ASSERTIONS) begin : g_axi_ram_assert
    // Hold AW* stable under backpressure
    property p_aw_stable;
      @(posedge ACLK) disable iff (!ARESETn)
        axi.AWVALID && !axi.AWREADY |-> $stable({axi.AWID,axi.AWADDR,axi.AWLEN,axi.AWSIZE,axi.AWBURST});
    endproperty
    assert property (p_aw_stable) else $error("axi4_ram: AW payload changed while AWVALID&&!AWREADY");

    // Hold W* stable under backpressure
    property p_w_stable;
      @(posedge ACLK) disable iff (!ARESETn)
        axi.WVALID && !axi.WREADY |-> $stable({axi.WDATA,axi.WSTRB,axi.WLAST});
    endproperty
    assert property (p_w_stable) else $error("axi4_ram: W payload changed while WVALID&&!WREADY");

    // Hold AR* stable under backpressure
    property p_ar_stable;
      @(posedge ACLK) disable iff (!ARESETn)
        axi.ARVALID && !axi.ARREADY |-> $stable({axi.ARID,axi.ARADDR,axi.ARLEN,axi.ARSIZE,axi.ARBURST});
    endproperty
    assert property (p_ar_stable) else $error("axi4_ram: AR payload changed while ARVALID&&!ARREADY");

    // Hold R* & B* stable under backpressure
    property p_r_stable;
      @(posedge ACLK) disable iff (!ARESETn)
        axi.RVALID && !axi.RREADY |-> $stable({axi.RID,axi.RDATA,axi.RRESP,axi.RLAST});
    endproperty
    assert property (p_r_stable) else $error("axi4_ram: R payload changed while RVALID&&!RREADY");

    property p_b_stable;
      @(posedge ACLK) disable iff (!ARESETn)
        axi.BVALID && !axi.BREADY |-> $stable({axi.BID,axi.BRESP});
    endproperty
    assert property (p_b_stable) else $error("axi4_ram: B payload changed while BVALID&&!BREADY");

    // 4KB boundary checks 
    if (STRICT_4KB) begin
      property p_aw_4k;
        @(posedge ACLK) disable iff (!ARESETn)
          (axi.AWVALID && axi.AWREADY && axi.AWBURST==2'b01)
            |-> ((axi.AWADDR[11:0] + ((axi.AWLEN+1) << axi.AWSIZE)) <= 12'd4096);
      endproperty
      assert property (p_aw_4k) else $error("axi4_ram: AW INCR crosses 4KB");

      property p_ar_4k;
        @(posedge ACLK) disable iff (!ARESETn)
          (axi.ARVALID && axi.ARREADY && axi.ARBURST==2'b01)
            |-> ((axi.ARADDR[11:0] + ((axi.ARLEN+1) << axi.ARSIZE)) <= 12'd4096);
      endproperty
      assert property (p_ar_4k) else $error("axi4_ram: AR INCR crosses 4KB");
    end
  end endgenerate
`endif

endmodule

`default_nettype wire
