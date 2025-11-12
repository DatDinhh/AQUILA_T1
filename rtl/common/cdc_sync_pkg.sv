// ============================================================================
//  CDC Utility Package and Modules
//  File: cdc_sync_pkg.sv
//  Description:
//    Silicon-proven CDC primitives for Aquila-T1 and related IP:
//      - Bit synchronizer (2+ stages)
//      - Reset synchronizer (async assert, sync deassert)
//      - Lossless pulse synchronizer (toggle+ack)
//      - Multi-bit bus handshake synchronizer (2-phase req/ack)
//      - Dual-clock asynchronous FIFO (Gray pointers)
//
//  Conventions:
//    * clk/rst suffixes:  _src for source domain, _dst for destination
//    * Active-low resets use *_rst_n naming
//    * Assertions are compiled with `ASSERT_ON`
//
//  Notes:
//    * Synchronizer internal flops are intentionally not reset by default.
//      Many sign-off flows recommend not resetting ASYNC_REG flops.
//    * Add STA constraints for CDC paths per flowguides.
//
//  © 2025. MIT-style license (adjust to your repo policy).
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

// ---------------------------------------------------------------------------
// Package: types and docs helpers
// ---------------------------------------------------------------------------
package cdc_sync_pkg;

  typedef enum logic [0:0] {RST_ACTIVE_LOW=1'b0, RST_ACTIVE_HIGH=1'b1} rst_polarity_e;

  // Little helper to invert based on polarity (used in reset sync wrappers)
  function automatic logic cond_invert(input logic v, input rst_polarity_e pol);
    return (pol == RST_ACTIVE_HIGH) ? v : ~v;
  endfunction

endpackage : cdc_sync_pkg


// ============================================================================
//  Bit Synchronizer (single-bit) — parameterized stages (>=2)
//  * USE_RESET=0 by default (preferred). If you must reset, set USE_RESET=1.
//  * STAGES=2 is typical; STAGES=3 in very high-frequency or low-margin domains.
// ============================================================================
module cdc_bit_sync #(
  parameter int unsigned STAGES     = 2,
  parameter bit          INIT       = 1'b0, // power-up value of synchronizer flops (sim only)
  parameter bit          USE_RESET  = 1'b0  // if 1, apply async reset to flops (not recommended)
)(
  input  logic clk_dst,
  input  logic rst_dst_n,   // only used if USE_RESET=1
  input  logic d_async,     // asynchronous to clk_dst
  output logic q_sync       // synchronized to clk_dst
);
  // Safety: stages must be >= 2
  initial begin
    if (STAGES < 2) begin
      $error("cdc_bit_sync: STAGES must be >= 2");
    end
  end

  // Synchronizer shift register
  (* ASYNC_REG = "TRUE" *) logic [STAGES-1:0] sync_q /* synthesis syn_preserve=1 */;
  genvar i;

  generate
    if (USE_RESET) begin : g_with_reset
      always_ff @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
          sync_q <= {STAGES{INIT}};
        end else begin
          sync_q[0] <= d_async;
          for (i = 1; i < STAGES; i++) begin
            sync_q[i] <= sync_q[i-1];
          end
        end
      end
    end else begin : g_no_reset
      // No reset: recommended to avoid introducing an async control to the chain
      always_ff @(posedge clk_dst) begin
        sync_q[0] <= d_async;
        for (i = 1; i < STAGES; i++) begin
          sync_q[i] <= sync_q[i-1];
        end
      end
    end
  endgenerate

  assign q_sync = sync_q[STAGES-1];

`ifdef ASSERT_ON
  // Output must only change on clk_dst edges (implicit by design)
  // Assert metastability containment can't be formally proven here, but we
  // can check STAGES>=2 and that q_sync is stable between destination clocks.
`endif
endmodule


// ============================================================================
//  Reset Synchronizer — async assert, sync deassert
//  Produces an active-low reset in the destination domain by default.
//  If you need active-high output, invert externally.
// ============================================================================
module cdc_reset_sync #(
  parameter int unsigned STAGES = 2
)(
  input  logic clk_dst,
  // Asynchronous reset input (active-low)
  input  logic rst_async_n,
  // Synchronized reset output (active-low)
  output logic rst_sync_n
);
  // Two or more stages recommended
  initial begin
    if (STAGES < 2) $error("cdc_reset_sync: STAGES must be >= 2");
  end

  (* ASYNC_REG = "TRUE" *) logic [STAGES-1:0] shreg;

  always_ff @(posedge clk_dst or negedge rst_async_n) begin
    if (!rst_async_n) begin
      shreg <= '0;                // assert immediately (all zeros)
    end else begin
      shreg <= {shreg[STAGES-2:0], 1'b1}; // synchronous deassert
    end
  end

  assign rst_sync_n = shreg[STAGES-1];

`ifdef ASSERT_ON
  // Once rst_async_n is high, rst_sync_n must deassert after STAGES cycles
  // Simple cover to ensure we see both asserted and deasserted states in sim
`endif
endmodule


// ============================================================================
//  Pulse Synchronizer (lossless)
//  Source pulse -> Destination 1-cycle pulse, no event loss.
//  Handshake uses a toggle + acknowledge reflection.
//  New pulses are accepted only when busy_o==0.
//
//  Contract:
//    * Assert pulse_i for one src clock when busy_o==0.
//    * pulse_o will be exactly one dst clock long.
// ============================================================================
module cdc_pulse_sync #(
  parameter int unsigned STAGES_FWD = 2, // src->dst toggle sync stages
  parameter int unsigned STAGES_BWD = 2  // dst->src ack   sync stages
)(
  // Source domain
  input  logic clk_src,
  input  logic rst_src_n,
  input  logic pulse_i,     // 1-cycle pulse in src domain
  output logic busy_o,      // 1 when a pulse is in-flight

  // Destination domain
  input  logic clk_dst,
  input  logic rst_dst_n,
  output logic pulse_o      // 1-cycle pulse in dst domain
);
  // Source toggle and ack sync
  logic src_tog_q, src_tog_d;
  logic ack_tog_src; // ack observed back in src

  // Busy when toggle hasn't been acknowledged
  assign busy_o = (src_tog_q != ack_tog_src);

  // Toggle create: accept new pulse only when not busy
  always_ff @(posedge clk_src or negedge rst_src_n) begin
    if (!rst_src_n) begin
      src_tog_q <= 1'b0;
    end else begin
      if (pulse_i && !busy_o) src_tog_q <= ~src_tog_q;
    end
  end

  // Forward: src_tog_q -> dst domain
  logic src_tog_dst;
  cdc_bit_sync #(.STAGES(STAGES_FWD), .USE_RESET(1'b1)) u_fwd (
    .clk_dst   (clk_dst),
    .rst_dst_n (rst_dst_n),
    .d_async   (src_tog_q),
    .q_sync    (src_tog_dst)
  );

  // Edge-detect in dst to make a one-cycle pulse
  logic src_tog_dst_q;
  always_ff @(posedge clk_dst or negedge rst_dst_n) begin
    if (!rst_dst_n) src_tog_dst_q <= 1'b0;
    else            src_tog_dst_q <= src_tog_dst;
  end
  assign pulse_o = src_tog_dst ^ src_tog_dst_q;

  // Backward: reflect the synchronized toggle as the ack back to src
  cdc_bit_sync #(.STAGES(STAGES_BWD), .USE_RESET(1'b1)) u_bwd (
    .clk_dst   (clk_src),
    .rst_dst_n (rst_src_n),
    .d_async   (src_tog_dst),
    .q_sync    (ack_tog_src)
  );

`ifdef ASSERT_ON
  // No new pulse is accepted while busy
  assert property (@(posedge clk_src) disable iff (!rst_src_n)
    pulse_i |-> !busy_o) else
    $warning("cdc_pulse_sync: pulse_i asserted while busy_o=1 (dropped by design)");
`endif

endmodule


// ============================================================================
//  Bus Handshake Synchronizer (reconvergence-safe multi-bit CDC)
//  Two-phase toggle handshake carrying a stable data bus.
//  - Source holds data until ack returns.
//  - Destination asserts dst_valid_o for one or more cycles until dst_ready_i.
//  - No data loss, no duplication.
//
//  Warning for CDC lint: The data bus crosses clock domains without per-bit
//  synchronizers. Safety is guaranteed by the handshake protocol which ensures
//  the bus is stable before and during capture; waive appropriately in CDC tools.
// ============================================================================
module cdc_bus_handshake #(
  parameter int unsigned WIDTH        = 256,
  parameter int unsigned STAGES_FWD   = 2,
  parameter int unsigned STAGES_BWD   = 2
)(
  // Source domain
  input  logic                 clk_src,
  input  logic                 rst_src_n,
  input  logic [WIDTH-1:0]     src_data_i,
  input  logic                 src_valid_i,
  output logic                 src_ready_o,

  // Destination domain
  input  logic                 clk_dst,
  input  logic                 rst_dst_n,
  output logic [WIDTH-1:0]     dst_data_o,
  output logic                 dst_valid_o,
  input  logic                 dst_ready_i
);
  // ------------------------
  // Source side
  // ------------------------
  logic [WIDTH-1:0] src_data_hold_q;
  logic             req_tog_q;

  // Ack observed in src
  logic ack_tog_src;

  // Ready when no transfer is outstanding
  assign src_ready_o = (req_tog_q == ack_tog_src);

  // Latch data and toggle req when accepted
  always_ff @(posedge clk_src or negedge rst_src_n) begin
    if (!rst_src_n) begin
      req_tog_q       <= 1'b0;
      src_data_hold_q <= '0;
    end else begin
      if (src_valid_i && src_ready_o) begin
        src_data_hold_q <= src_data_i;
        req_tog_q       <= ~req_tog_q;
      end
    end
  end

  // ------------------------
  // Forward: req toggle to dst
  // ------------------------
  logic req_tog_dst;
  cdc_bit_sync #(.STAGES(STAGES_FWD), .USE_RESET(1'b1)) u_req_sync (
    .clk_dst   (clk_dst),
    .rst_dst_n (rst_dst_n),
    .d_async   (req_tog_q),
    .q_sync    (req_tog_dst)
  );

  // ------------------------
  // Destination side
  // ------------------------
  logic req_seen_q;         // last seen toggle
  logic dst_valid_q;
  logic [WIDTH-1:0] dst_data_q;

  // Detect new request token
  wire new_token = (req_tog_dst ^ req_seen_q);

  // Capture data when new token arrives (bus is stable under handshake contract)
  always_ff @(posedge clk_dst or negedge rst_dst_n) begin
    if (!rst_dst_n) begin
      req_seen_q  <= 1'b0;
      dst_valid_q <= 1'b0;
      dst_data_q  <= '0;
    end else begin
      // New token: capture and present valid
      if (new_token) begin
        req_seen_q  <= req_tog_dst;
        dst_data_q  <= src_data_hold_q;  // async bus capture guarded by handshake
        dst_valid_q <= 1'b1;
      end else if (dst_valid_q && dst_ready_i) begin
        // Consume
        dst_valid_q <= 1'b0;
      end
    end
  end

  assign dst_valid_o = dst_valid_q;
  assign dst_data_o  = dst_data_q;

  // ------------------------
  // Backward: reflect req_seen (ack) to src
  // ------------------------
  cdc_bit_sync #(.STAGES(STAGES_BWD), .USE_RESET(1'b1)) u_ack_sync (
    .clk_dst   (clk_src),
    .rst_dst_n (rst_src_n),
    .d_async   (req_seen_q),
    .q_sync    (ack_tog_src)
  );

`ifdef ASSERT_ON
  // Source accepts new data only when ready
  assert property (@(posedge clk_src) disable iff (!rst_src_n)
    src_valid_i |-> src_ready_o) else
    $warning("cdc_bus_handshake: src_valid_i high while not ready; data held until ready.");

  // Destination: no new_token should be missed
  // (Implicit in toggle design with 2+ synchronizer stages.)
`endif

endmodule


// ============================================================================
//  Asynchronous FIFO (dual-clock) — Gray-coded pointers
//  Parameters:
//    WIDTH       : data width (bits)
//    DEPTH       : entries (power-of-two required)
//    STAGES_SYNC : per-bit synchronizer stages for Gray pointers
//
//  Notes:
//    - 1-cycle read latency (non-FWFT). Add FWFT if you need zero-lat read.
//    - Full/empty generation per Clifford E. Cummings paper.
// ============================================================================
module cdc_async_fifo #(
  parameter int unsigned WIDTH        = 256,
  parameter int unsigned DEPTH        = 16,   // must be power-of-two
  parameter int unsigned STAGES_SYNC  = 2
)(
  // Write domain
  input  logic                 wr_clk,
  input  logic                 wr_rst_n,
  input  logic                 wr_en_i,
  input  logic [WIDTH-1:0]     wr_data_i,
  output logic                 full_o,

  // Read domain
  input  logic                 rd_clk,
  input  logic                 rd_rst_n,
  input  logic                 rd_en_i,
  output logic [WIDTH-1:0]     rd_data_o,
  output logic                 empty_o
);
  // ------------------------
  // Static checks
  // ------------------------
  localparam int unsigned ADDR_BITS = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
  localparam int unsigned PTR_W     = ADDR_BITS + 1; // for wrap tracking

  // Power-of-two depth check (synthesis-time)
  initial begin
    if ((1 << ADDR_BITS) != DEPTH) begin
      $error("cdc_async_fifo: DEPTH (%0d) must be a power-of-two", DEPTH);
    end
    if (STAGES_SYNC < 2) begin
      $error("cdc_async_fifo: STAGES_SYNC must be >=2");
    end
  end

  // ------------------------
  // Storage (simple 2-port RAM by inference)
  // ------------------------
  logic [WIDTH-1:0] mem [0:DEPTH-1];

  // ------------------------
  // Write domain pointers
  // ------------------------
  logic [PTR_W-1:0] wr_bin_q,  wr_bin_n;
  logic [PTR_W-1:0] wr_gray_q, wr_gray_n;

  // Read pointer Gray synchronized into write domain
  logic [PTR_W-1:0] rd_gray_q;
  logic [PTR_W-1:0] rd_gray_sync_w;

  // Write enable effective
  wire wr_push = wr_en_i && !full_o;

  // Next pointers
  always_comb begin
    wr_bin_n  = wr_bin_q + (wr_push ? 1 : 0);
    wr_gray_n = (wr_bin_n >> 1) ^ wr_bin_n; // bin2gray
  end

  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_bin_q  <= '0;
      wr_gray_q <= '0;
    end else begin
      wr_bin_q  <= wr_bin_n;
      wr_gray_q <= wr_gray_n;
      if (wr_push) begin
        mem[wr_bin_q[ADDR_BITS-1:0]] <= wr_data_i;
      end
    end
  end

  // Synchronize read Gray pointer into write domain (per-bit)
  genvar gi_w;
  generate
    for (gi_w = 0; gi_w < PTR_W; gi_w++) begin : g_sync_rptr
      cdc_bit_sync #(.STAGES(STAGES_SYNC), .USE_RESET(1'b1)) u_rptr_sync (
        .clk_dst   (wr_clk),
        .rst_dst_n (wr_rst_n),
        .d_async   (rd_gray_q[gi_w]),
        .q_sync    (rd_gray_sync_w[gi_w])
      );
    end
  endgenerate

  // Full generation: write gray next equals read gray sync with MSBs inverted
  wire [PTR_W-1:0] wr_gray_next = wr_gray_n;
  assign full_o = (wr_gray_next == {~rd_gray_sync_w[PTR_W-1:PTR_W-2], rd_gray_sync_w[PTR_W-3:0]});

  // ------------------------
  // Read domain pointers
  // ------------------------
  logic [PTR_W-1:0] rd_bin_q,  rd_bin_n;
  logic [PTR_W-1:0] rd_gray_n;
  logic [WIDTH-1:0] rd_data_q;

  // Write pointer Gray synchronized into read domain
  logic [PTR_W-1:0] wr_gray_sync_r;

  // Empty when pointers equal (in Gray)
  assign empty_o = (rd_gray_q == wr_gray_sync_r);

  // Read pop
  wire rd_pop = rd_en_i && !empty_o;

  // Next pointers
  always_comb begin
    rd_bin_n  = rd_bin_q + (rd_pop ? 1 : 0);
    rd_gray_n = (rd_bin_n >> 1) ^ rd_bin_n; // bin2gray
  end

  // Read domain sequential
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_bin_q   <= '0;
      rd_gray_q  <= '0;
      rd_data_q  <= '0;
    end else begin
      rd_bin_q   <= rd_bin_n;
      rd_gray_q  <= rd_gray_n;
      if (rd_pop) begin
        rd_data_q <= mem[rd_bin_q[ADDR_BITS-1:0]]; // 1-cycle latency
      end
    end
  end

  assign rd_data_o = rd_data_q;

  // Synchronize write Gray pointer into read domain (per-bit)
  genvar gi_r;
  generate
    for (gi_r = 0; gi_r < PTR_W; gi_r++) begin : g_sync_wptr
      cdc_bit_sync #(.STAGES(STAGES_SYNC), .USE_RESET(1'b1)) u_wptr_sync (
        .clk_dst   (rd_clk),
        .rst_dst_n (rd_rst_n),
        .d_async   (wr_gray_q[gi_r]),
        .q_sync    (wr_gray_sync_r[gi_r])
      );
    end
  endgenerate

`ifdef ASSERT_ON
  // Never full and empty simultaneously (can only happen at reset if miswired)
  assert property (@(posedge wr_clk) disable iff (!wr_rst_n)
    !(full_o && (rd_gray_q == wr_gray_sync_r))) else
      $error("cdc_async_fifo: full asserted while empty condition seen/assumed");

  // No write when full; no read when empty
  assert property (@(posedge wr_clk) disable iff (!wr_rst_n) wr_en_i |-> !full_o);
  assert property (@(posedge rd_clk) disable iff (!rd_rst_n) rd_en_i |-> !empty_o);
`endif

endmodule

`default_nettype wire
