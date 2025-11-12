// ============================================================================
//  wb_pack.sv
//  Writeback packer: C-stream elements -> AXI-style W beat(s) with WSTRB/WLAST
//
//  Features
//  --------
//  - Handles unaligned starts (start_byte_off_i) and generates correct WSTRB
//    on the first beat.
//  - Saturating dtype conversion: (IN_ELEM_W, IN_SIGNED) -> (STORE_ELEM_W, STORE_SIGNED).
//  - Packs up to BUS_W bits per beat; 1 beat/cycle when downstream ready.
//  - Clean ready/valid (elastic FIFOs); no combinational ready loops.
//  - Final partial beat flushing on in_last_i.
//  - Telemetry counters and integration-friendly assertions.
//
//  Contract
//  --------
//  - Elements are byte-aligned (width % 8 == 0). BUS_W is multiple of 8.
//  - Start a transfer with (start_valid_i && start_ready_o). Provide the
//    address's byte offset (AWADDR[log2(BUS_BYTES)-1:0]) as start_byte_off_i.
//  - Provide the stream of elements for this transfer, assert in_last_i with
//    the final element. The module emits W beats; WLAST is asserted on the
//    last emitted beat of the transfer.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module wb_pack #(
  // -------------------- Stream geometry ----------------------
  parameter int unsigned LANES_IN       = 1,   // elements per input beat
  parameter int unsigned IN_ELEM_W      = 16,  // width of incoming elements (bytes aligned)
  parameter bit          IN_SIGNED      = 1'b1,

  // -------------------- Store dtype --------------------------
  parameter int unsigned STORE_ELEM_W   = 8,   // element width to store (bytes aligned)
  parameter bit          STORE_SIGNED   = 1'b1,

  // -------------------- Bus geometry -------------------------
  parameter int unsigned BUS_W          = 256, // bus width in bits (multiple of 8)

  // -------------------- Elasticity ---------------------------
  parameter int unsigned IN_FIFO_DEPTH  = 4,   // ingress buffering
  parameter int unsigned OUT_FIFO_DEPTH = 4    // egress buffering
)(
  input  logic                             clk_i,
  input  logic                             rstn_i,           // synchronous, active-low

  // ==================== Transfer start =======================
  // Provide exactly once per transfer, before the first element is accepted.
  input  logic                             start_valid_i,
  output logic                             start_ready_o,
  input  logic [$clog2(BUS_W/8)-1:0]       start_byte_off_i, // AWADDR byte offset (0..BUS_BYTES-1)

  // ==================== C-stream in ==========================
  input  logic                             in_valid_i,
  output logic                             in_ready_o,
  input  logic [LANES_IN*IN_ELEM_W-1:0]    in_data_i,
  input  logic                             in_last_i,        // last element of the transfer

  // ==================== W-channel out ========================
  output logic                             w_valid_o,
  input  logic                              w_ready_i,
  output logic [BUS_W-1:0]                 w_data_o,
  output logic [BUS_W/8-1:0]               w_strb_o,
  output logic                             w_last_o,

  // ==================== Config ===============================
  input  logic                             cfg_little_endian_i, // 1=LE lanes, 0=BE byte order

  // ==================== Telemetry ============================
  input  logic                             cmd_clr_stats_i,
  output logic [31:0]                      stat_in_beats_o,     // input beats accepted
  output logic [31:0]                      stat_w_beats_o,      // W beats emitted
  output logic [31:0]                      stat_elem_sats_o     // (optional) elem clamps
);

  // --------------------------------------------------------------------------
  // Static checks
  // --------------------------------------------------------------------------
  initial begin
    if ((IN_ELEM_W   % 8) != 0) $error("wb_pack: IN_ELEM_W must be a multiple of 8.");
    if ((STORE_ELEM_W% 8) != 0) $error("wb_pack: STORE_ELEM_W must be a multiple of 8.");
    if ((BUS_W       % 8) != 0) $error("wb_pack: BUS_W must be a multiple of 8.");
    if (LANES_IN*STORE_ELEM_W > BUS_W)
      $error("wb_pack: LANES_IN*STORE_ELEM_W must be <= BUS_W (one W beat/cycle max).");
  end

  localparam int unsigned BUS_BYTES = BUS_W/8;
  localparam int unsigned PAYLOAD_W = LANES_IN*STORE_ELEM_W;
  localparam int unsigned IN_PACK_W = LANES_IN*IN_ELEM_W + 1; // + last bit

  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------
  function automatic logic signed [63:0]
    sext_any(input logic [63:0] x, input int w, input bit is_signed);
    logic signed [63:0] r;
    if (is_signed) r = $signed(x[w-1:0]);
    else           r = $signed({1'b0, x[w-2:0]});
    return r;
  endfunction

  // INT saturating conversion: IN -> STORE
  function automatic logic [STORE_ELEM_W-1:0]
    conv_elem(input logic [IN_ELEM_W-1:0] vin);
    logic signed [63:0] v  = sext_any({{(64-IN_ELEM_W){1'b0}}, vin}, IN_ELEM_W, IN_SIGNED);
    logic signed [63:0] hi, lo;
    if (STORE_SIGNED) begin
      hi =  $signed((64'sh1 << (STORE_ELEM_W-1)) - 1);
      lo = -$signed( (64'sh1 << (STORE_ELEM_W-1)) );
    end else begin
      hi =  $signed((64'sh1 << STORE_ELEM_W) - 1);
      lo =  64'sd0;
    end
    logic signed [63:0] vs;
    if      (v > hi) vs = hi;
    else if (v < lo) vs = lo;
    else             vs = v;
    return logic'(vs[STORE_ELEM_W-1:0]);
  endfunction

  function automatic logic [BUS_W-1:0]
    byte_reverse(input logic [BUS_W-1:0] din);
    logic [BUS_W-1:0] dout;
    for (int b=0; b<BUS_BYTES; b++)
      dout[(BUS_BYTES-1-b)*8 +: 8] = din[b*8 +: 8];
    return dout;
  endfunction

  function automatic logic [BUS_BYTES-1:0]
    keep_reverse(input logic [BUS_BYTES-1:0] kin);
    logic [BUS_BYTES-1:0] kout;
    for (int b=0; b<BUS_BYTES; b++)
      kout[BUS_BYTES-1-b] = kin[b];
    return kout;
  endfunction

  // --------------------------------------------------------------------------
  // Ingress FIFO (decouple upstream). We gate acceptance to enforce start-first.
  // --------------------------------------------------------------------------
  logic                 in_fifo_in_ready, in_fifo_out_valid, in_fifo_out_ready;
  logic [IN_PACK_W-1:0] in_fifo_word;

  rv_fifo #(
    .WIDTH (IN_PACK_W),
    .DEPTH (IN_FIFO_DEPTH)
  ) u_in_fifo (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (in_valid_i),
    .in_ready_o  (in_fifo_in_ready),
    .in_data_i   ({in_last_i, in_data_i}),
    .out_valid_o (in_fifo_out_valid),
    .out_ready_i (in_fifo_out_ready),
    .out_data_o  (in_fifo_word)
  );

  // We expose in_ready_o only when a transfer has started; otherwise 0.
  assign in_ready_o = (/*started*/ 1'b1) ? in_fifo_in_ready : 1'b0;

  // Unpack current head
  wire i_last = in_fifo_word[IN_PACK_W-1];
  wire [LANES_IN*IN_ELEM_W-1:0] i_data = in_fifo_word[LANES_IN*IN_ELEM_W-1:0];

  // --------------------------------------------------------------------------
  // Egress FIFO (W channel)
  // --------------------------------------------------------------------------
  localparam int unsigned W_PACK_W = BUS_W + BUS_BYTES + 1; // {last, strb, data}
  logic                 w_fifo_in_valid,  w_fifo_in_ready;
  logic [W_PACK_W-1:0]  w_fifo_in_word;
  logic                 w_fifo_out_valid;
  logic [W_PACK_W-1:0]  w_fifo_out_word;

  rv_fifo #(
    .WIDTH (W_PACK_W),
    .DEPTH (OUT_FIFO_DEPTH)
  ) u_w_fifo (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (w_fifo_in_valid),
    .in_ready_o  (w_fifo_in_ready),
    .in_data_i   (w_fifo_in_word),
    .out_valid_o (w_fifo_out_valid),
    .out_ready_i (w_ready_i),
    .out_data_o  (w_fifo_out_word)
  );

  assign w_valid_o = w_fifo_out_valid;
  assign w_data_o  = w_fifo_out_word[BUS_W-1:0];
  assign w_strb_o  = w_fifo_out_word[BUS_W+BUS_BYTES-1:BUS_W];
  assign w_last_o  = w_fifo_out_word[BUS_W+BUS_BYTES];

  // --------------------------------------------------------------------------
  // Transfer state / alignment
  // --------------------------------------------------------------------------
  logic                         started_q, started_d;
  logic                         first_word_q, first_word_d;
  logic [$clog2(BUS_BYTES)-1:0] off_bytes_q, off_bytes_d;

  // Accumulator of word under construction (LSB-first internal)
  logic [BUS_W-1:0]             acc_q, acc_d;
  logic [$clog2(BUS_W+1)-1:0]   fill_q, fill_d; // # of occupied bits in acc_q
  logic                         last_pending_q, last_pending_d;

  // Start handshake: ready when idle and no input buffered
  wire idle = (!started_q) && (fill_q == '0) && !in_fifo_out_valid && !first_word_q && !last_pending_q;
  assign start_ready_o = idle;

  wire start_fire = start_valid_i && start_ready_o;

  // --------------------------------------------------------------------------
  // Convert current input payload (if any) to STORE_ELEM_W and pack into PAYLOAD
  // --------------------------------------------------------------------------
  logic [PAYLOAD_W-1:0] payload_w;
  always_comb begin
    payload_w = '0;
    for (int i=0; i<LANES_IN; i++) begin
      payload_w[i*STORE_ELEM_W +: STORE_ELEM_W] = conv_elem(i_data[i*IN_ELEM_W +: IN_ELEM_W]);
    end
  end

  // Will we need to emit a W beat this cycle if we pop the FIFO?
  wire [$clog2(BUS_W+1)-1:0] nfill_bits = fill_q + PAYLOAD_W[$clog2(BUS_W+1)-1:0];
  wire will_emit_full    = in_fifo_out_valid && started_q && (nfill_bits >= BUS_W[$clog2(BUS_W+1)-1:0]);
  wire will_emit_partial = in_fifo_out_valid && started_q && i_last && (nfill_bits != '0) && (nfill_bits < BUS_W);

  // We can pop the ingress FIFO when either we don't emit this cycle,
  // or we emit and the W FIFO can accept.
  assign in_fifo_out_ready = started_q &&
                             ( (!will_emit_full && !will_emit_partial) ||
                               ( (will_emit_full || will_emit_partial) && w_fifo_in_ready ) );

  // --------------------------------------------------------------------------
  // Main packer
  // --------------------------------------------------------------------------
  always_comb begin
    // Defaults
    started_d      = started_q;
    first_word_d   = first_word_q;
    off_bytes_d    = off_bytes_q;
    acc_d          = acc_q;
    fill_d         = fill_q;
    last_pending_d = last_pending_q;

    w_fifo_in_valid = 1'b0;
    w_fifo_in_word  = '0;

    // Transfer start: preload alignment state
    if (start_fire) begin
      started_d      = 1'b1;
      first_word_d   = 1'b1;
      off_bytes_d    = start_byte_off_i;
      acc_d          = '0;
      fill_d         = { {$clog2(BUS_W+1)-3{1'b0}}, start_byte_off_i, 3'b000 }; // off_bytes * 8
      last_pending_d = 1'b0;
    end

    // If we popped one input beat, merge it into the accumulator
    if (in_fifo_out_valid && in_fifo_out_ready) begin
      logic [BUS_W-1:0] merged = acc_q | ( { {(BUS_W-PAYLOAD_W){1'b0}}, payload_w } << fill_q );
      logic [$clog2(BUS_W+1)-1:0] nfill = nfill_bits;

      // Case A: emit a full beat (may be 1st or later)
      if (nfill >= BUS_W && w_fifo_in_ready) begin
        logic [BUS_W-1:0] data_le  = merged[BUS_W-1:0];
        logic [BUS_BYTES-1:0] keep_le;

        if (first_word_q) begin
          // First full beat: leading 'off_bytes' lanes are masked off
          keep_le = ({BUS_BYTES{1'b1}}) << off_bytes_q;
        end else begin
          keep_le = {BUS_BYTES{1'b1}};
        end

        logic [BUS_W-1:0]     data_bus = cfg_little_endian_i ? data_le : byte_reverse(data_le);
        logic [BUS_BYTES-1:0] strb_bus = cfg_little_endian_i ? keep_le : keep_reverse(keep_le);

        // Determine if this is also the last beat (exactly ended on boundary)
        logic make_last = i_last && (nfill == BUS_W);

        w_fifo_in_valid = 1'b1;
        w_fifo_in_word  = { make_last, strb_bus, data_bus };

        // Update accumulator (drop emitted beat)
        acc_d  = merged >> BUS_W;
        fill_d = nfill - BUS_W;

        // Clear first-word flag once we've emitted the first beat
        if (first_word_q) first_word_d = 1'b0;

        // End-of-transfer bookkeeping
        if (make_last) begin
          started_d      = 1'b0;
          last_pending_d = 1'b0;
          acc_d          = '0;
          fill_d         = '0;
          first_word_d   = 1'b0;
        end else begin
          // If input signalled last, but remainder bits exist, mark for later flush
          last_pending_d = i_last && ( (nfill - BUS_W) != '0 );
        end
      end

      // Case B: partial flush on last (no full beat formed yet)
      else if (i_last && (nfill != '0) && (nfill < BUS_W) && w_fifo_in_ready) begin
        logic [BUS_W-1:0] data_le = merged;

        // Compute #data bytes in this (partial) beat
        int unsigned used_bytes = nfill >> 3;  // safe: multiples of 8 enforced
        int unsigned data_bytes = used_bytes;
        logic [BUS_BYTES-1:0] keep_le = '0;

        if (first_word_q) begin
          // First partial beat: skip leading offset bytes
          if (used_bytes > off_bytes_q) begin
            data_bytes = used_bytes - off_bytes_q;
            for (int b=0; b<data_bytes; b++) keep_le[off_bytes_q + b] = 1'b1;
          end else begin
            data_bytes = 0; // should not happen because nfill != 0 and off_bytes < BUS_BYTES
          end
        end else begin
          // Later partial beat: occupy from byte lane 0 upward
          for (int b=0; b<data_bytes; b++) keep_le[b] = 1'b1;
        end

        logic [BUS_W-1:0]     data_bus = cfg_little_endian_i ? data_le : byte_reverse(data_le);
        logic [BUS_BYTES-1:0] strb_bus = cfg_little_endian_i ? keep_le : keep_reverse(keep_le);

        w_fifo_in_valid = 1'b1;
        w_fifo_in_word  = { 1'b1 /*last*/, strb_bus, data_bus };

        // Reset for next transfer
        acc_d          = '0;
        fill_d         = '0;
        started_d      = 1'b0;
        first_word_d   = 1'b0;
        last_pending_d = 1'b0;
      end

      // Case C: no emission this cycle; just accumulate
      else begin
        acc_d          = merged;
        fill_d         = nfill;
        last_pending_d = i_last ? 1'b1 : last_pending_q;
      end
    end

    // If no input popped but we owe a final partial flush (e.g., last came earlier),
    // flush the remainder now.
    if (!in_fifo_out_valid && last_pending_q && (fill_q != '0) && w_fifo_in_ready) begin
      logic [BUS_W-1:0] data_le   = acc_q;
      int unsigned used_bytes     = fill_q >> 3;
      int unsigned data_bytes     = used_bytes;
      logic [BUS_BYTES-1:0] keep_le = '0;

      if (first_word_q) begin
        if (used_bytes > off_bytes_q) begin
          data_bytes = used_bytes - off_bytes_q;
          for (int b=0; b<data_bytes; b++) keep_le[off_bytes_q + b] = 1'b1;
        end else begin
          data_bytes = 0;
        end
      end else begin
        for (int b=0; b<data_bytes; b++) keep_le[b] = 1'b1;
      end

      logic [BUS_W-1:0]     data_bus = cfg_little_endian_i ? data_le : byte_reverse(data_le);
      logic [BUS_BYTES-1:0] strb_bus = cfg_little_endian_i ? keep_le : keep_reverse(keep_le);

      w_fifo_in_valid = 1'b1;
      w_fifo_in_word  = { 1'b1 /*last*/, strb_bus, data_bus };

      // Reset for next transfer
      acc_d          = '0;
      fill_d         = '0;
      started_d      = 1'b0;
      first_word_d   = 1'b0;
      last_pending_d = 1'b0;
    end
  end

  // Registers
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      started_q      <= 1'b0;
      first_word_q   <= 1'b0;
      off_bytes_q    <= '0;
      acc_q          <= '0;
      fill_q         <= '0;
      last_pending_q <= 1'b0;
    end else begin
      started_q      <= started_d;
      first_word_q   <= first_word_d;
      off_bytes_q    <= off_bytes_d;
      acc_q          <= acc_d;
      fill_q         <= fill_d;
      last_pending_q <= last_pending_d;
    end
  end

  // --------------------------------------------------------------------------
  // Telemetry
  // --------------------------------------------------------------------------
  logic [31:0] in_beats_q, w_beats_q, elem_sats_q;

  // NOTE: For timing simplicity we do not compute element saturation counts here.
  // Hookup is provided for future extension (e.g., compare conv_elem inputs to clamps).
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      in_beats_q  <= 32'd0;
      w_beats_q   <= 32'd0;
      elem_sats_q <= 32'd0;
    end else if (cmd_clr_stats_i) begin
      in_beats_q  <= 32'd0;
      w_beats_q   <= 32'd0;
      elem_sats_q <= 32'd0;
    end else begin
      if (in_fifo_out_valid && in_fifo_out_ready) in_beats_q <= in_beats_q + 32'd1;
      if (w_fifo_in_valid && w_fifo_in_ready)     w_beats_q  <= w_beats_q  + 32'd1;
    end
  end

  assign stat_in_beats_o  = in_beats_q;
  assign stat_w_beats_o   = w_beats_q;
  assign stat_elem_sats_o = elem_sats_q;

  // --------------------------------------------------------------------------
  // Assertions (simulation only)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // Start must precede first input acceptance of a transfer.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    in_fifo_out_valid && !started_q |-> 1'b0)
    else $error("wb_pack: input arrived before start of transfer.");

  // First emitted full beat must mask leading offset bytes on WSTRB.
  // Check on the cycle we push the first beat.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    w_fifo_in_valid && first_word_q && ( (fill_q + PAYLOAD_W) >= BUS_W ) |->
      ( (cfg_little_endian_i
          ? w_fifo_in_word[BUS_W+BUS_BYTES-1:BUS_W]
          : keep_reverse(w_fifo_in_word[BUS_W+BUS_BYTES-1:BUS_W]) )
        == ( ({BUS_BYTES{1'b1}}) << off_bytes_q ) ) )
    else $error("wb_pack: first-beat WSTRB does not reflect start byte offset.");

  // w_last_o should only occur when w_valid_o.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    w_last_o |-> w_valid_o)
    else $error("wb_pack: w_last_o without w_valid_o.");
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
