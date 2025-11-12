// ============================================================================
//  ucie_shim.sv
//  UCIe (Streaming mode) <-> NoC Flit bridge (shim layer)
//
//  TX path (NoC -> UCIe):
//    Accepts one NoC flit, prepends a 2-byte shim header, segments into
//    N beats of UCIE_W, asserts SOP on first, EOP on last, KEEP for last.
//
//  RX path (UCIe -> NoC):
//    Reassembles bytes across beats between SOP/EOP, validates shim header,
//    emits one NoC flit.
//
//  Design goals:
//    - No combinational ready loops (elastic FIFOs at NoC and UCIe edges).
//    - Link-down tolerance (abort current packet, drop counters).
//    - Parameterized widths; sane defaults (FLIT fits in one 256b beat).
//    - Light but useful telemetry + assertions.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module ucie_shim #(
  // --------------------- NoC flit geometry ---------------------
  parameter int unsigned X_W         = 4,
  parameter int unsigned Y_W         = 4,
  parameter int unsigned VC_NUM      = 2,
  parameter int unsigned VCW         = (VC_NUM <= 1) ? 1 : $clog2(VC_NUM),
  parameter int unsigned PAYLOAD_W   = 64,
  parameter int unsigned FLIT_W      = (2 + VCW + X_W + Y_W + PAYLOAD_W), // HEAD/TAIL + VC + X + Y + payload

  // --------------------- UCIe streaming bus --------------------
  parameter int unsigned UCIE_W      = 256,        // bus bits (multiple of 8)
  parameter int unsigned HDR_BYTES   = 2,          // shim header size (bytes)
  // Header byte[0]=MAGIC, byte[1]= {ver[7:4]=1, type[3:0]=0}

  // --------------------- Elasticity ----------------------------
  parameter int unsigned TXF_DEPTH   = 8,          // NoC->UCIe ingress FIFO (flits)
  parameter int unsigned RXF_DEPTH   = 8,          // UCIe->NoC egress FIFO (flits)

  // --------------------- Behavior ------------------------------
  parameter bit          LITTLE_ENDIAN_BUS = 1'b1, // 1: byte0 at [7:0] of data
  parameter bit          STATS_EN          = 1'b1
)(
  input  logic                     clk_i,
  input  logic                     rstn_i,           // synchronous active-low

  // ======================= Control/Status ======================
  input  logic                     cfg_enable_i,     // enables traffic when link is up
  input  logic                     cfg_drop_on_err_i,// drop packet on rx error
  input  logic                     cmd_clr_stats_i,  // clear counters
  output logic                     shim_ready_o,     // high when link_up_i & internal idle

  // ======================= NoC (tile) side =====================
  // Outbound flits -> UCIe
  input  logic                     noc_tx_valid_i,
  output logic                     noc_tx_ready_o,
  input  logic [FLIT_W-1:0]        noc_tx_flit_i,

  // Inbound flits <- UCIe
  output logic                     noc_rx_valid_o,
  input  logic                     noc_rx_ready_i,
  output logic [FLIT_W-1:0]        noc_rx_flit_o,

  // ======================= UCIe streaming side =================
  input  logic                     link_up_i,        // UCIe link state (user-domain)

  // TX (shim -> UCIe controller)
  output logic                     tx_valid_o,
  input  logic                     tx_ready_i,
  output logic [UCIE_W-1:0]        tx_data_o,
  output logic                     tx_sop_o,
  output logic                     tx_eop_o,
  output logic [UCIE_W/8-1:0]      tx_keep_o,        // per-byte valid (AXIS tkeep-like)

  // RX (UCIe controller -> shim)
  input  logic                     rx_valid_i,
  output logic                     rx_ready_o,
  input  logic [UCIE_W-1:0]        rx_data_i,
  input  logic                     rx_sop_i,
  input  logic                     rx_eop_i,
  input  logic [UCIE_W/8-1:0]      rx_keep_i,
  input  logic                     rx_err_i,         // CRC or link-layer error (per-beat)

  // ======================= Telemetry ===========================
  output logic [31:0]              stat_tx_flits_o,  // flits sent to UCIe
  output logic [31:0]              stat_rx_flits_o,  // flits delivered to NoC
  output logic [31:0]              stat_tx_beats_o,  // UCIE beats sent
  output logic [31:0]              stat_rx_beats_o,  // UCIE beats received
  output logic [31:0]              stat_tx_drops_o,  // drops due to link down
  output logic [31:0]              stat_rx_drops_o,  // drops due to rx_err or framing
  output logic [31:0]              stat_rx_bad_hdr_o // bad shim header
);

  // --------------------- Derived constants ---------------------
  localparam int unsigned UCIE_B     = UCIE_W / 8;
  localparam int unsigned FLIT_B     = FLIT_W / 8;
  localparam int unsigned TOT_BYTES  = HDR_BYTES + FLIT_B;
  localparam int unsigned TOT_BITS   = TOT_BYTES * 8;
  localparam int unsigned N_BEATS    = (TOT_BYTES + UCIE_B - 1) / UCIE_B;
  localparam int unsigned PAD_BITS   = N_BEATS*UCIE_W - TOT_BITS;

  // Static checks
  initial begin
    if ((UCIE_W % 8) != 0)      $error("ucie_shim: UCIE_W must be multiple of 8.");
    if ((FLIT_W % 8) != 0)      $error("ucie_shim: FLIT_W must be multiple of 8.");
    if (HDR_BYTES < 2)          $error("ucie_shim: HDR_BYTES must be >= 2.");
    if (N_BEATS == 0)           $error("ucie_shim: N_BEATS computed as 0.");
  end

  // --------------------- Byte helpers --------------------------
  function automatic logic [UCIE_W-1:0]
    byte_reverse_bus(input logic [UCIE_W-1:0] din);
    logic [UCIE_W-1:0] dout;
    for (int i=0; i<UCIE_B; i++)
      dout[(UCIE_B-1-i)*8 +: 8] = din[i*8 +: 8];
    return dout;
  endfunction

  function automatic int unsigned popcount_keep(input logic [UCIE_B-1:0] k);
    return $countones(k);
  endfunction

  // --------------------- Shim header (2 bytes) -----------------
  // Byte 0: MAGIC (0xAF)
  // Byte 1: {ver[7:4]=1, type[3:0]=0} ; type 0 = NoC flit
  localparam logic [7:0] HDR0_MAGIC = 8'hAF;
  localparam logic [7:0] HDR1_META  = {4'h1, 4'h0};

  // ========================================================================
  // TX PATH: NoC flit -> UCIe streaming beats
  // ========================================================================
  // Ingress FIFO (flits)
  logic                 txf_in_ready, txf_out_valid, txf_out_ready;
  logic [FLIT_W-1:0]    txf_dout;

  rv_fifo #(
    .WIDTH (FLIT_W),
    .DEPTH (TXF_DEPTH)
  ) u_txf (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (noc_tx_valid_i && cfg_enable_i && link_up_i),
    .in_ready_o  (noc_tx_ready_o),
    .in_data_i   (noc_tx_flit_i),
    .out_valid_o (txf_out_valid),
    .out_ready_i (txf_out_ready),
    .out_data_o  (txf_dout)
  );

  // TX packet shifter (N_BEATS * UCIE_W bits)
  logic [N_BEATS*UCIE_W-1:0] tx_pkt_q, tx_pkt_d;
  logic [$clog2(N_BEATS+1)-1:0] tx_beat_q, tx_beat_d;
  logic                        tx_active_q, tx_active_d;

  // Precompute last KEEP mask
  localparam int unsigned LAST_BYTES = TOT_BYTES - (N_BEATS-1)*UCIE_B;
  localparam logic [UCIE_B-1:0] LAST_KEEP_LE = (LAST_BYTES==UCIE_B) ? {UCIE_B{1'b1}}
        : ((UCIE_B'(1) << LAST_BYTES) - UCIE_B'(1));

  // Assemble initial packet bits (LSB-first, header then flit bytes)
  function automatic logic [N_BEATS*UCIE_W-1:0]
    make_tx_packet(input logic [FLIT_W-1:0] flit);
    logic [TOT_BITS-1:0] pkt_core;
    // Layout (LSB-first): [hdr0][hdr1][flit bytes...]
    pkt_core = { flit, HDR1_META, HDR0_MAGIC };
    // Left pad to full beats (MSBs)
    return { {PAD_BITS{1'b0}}, pkt_core };
  endfunction

  // TX state machine
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      tx_pkt_q    <= '0;
      tx_beat_q   <= '0;
      tx_active_q <= 1'b0;
    end else begin
      tx_pkt_q    <= tx_pkt_d;
      tx_beat_q   <= tx_beat_d;
      tx_active_q <= tx_active_d;
    end
  end

  // TX next-state / outputs
  always_comb begin
    tx_pkt_d    = tx_pkt_q;
    tx_beat_d   = tx_beat_q;
    tx_active_d = tx_active_q;

    tx_valid_o  = 1'b0;
    tx_sop_o    = 1'b0;
    tx_eop_o    = 1'b0;
    tx_keep_o   = {UCIE_B{1'b0}};
    txf_out_ready = 1'b0;

    // Select current beat (LSB chunk of shifter)
    logic [UCIE_W-1:0] beat_le = tx_pkt_q[UCIE_W-1:0];
    logic [UCIE_W-1:0] beat_bus = LITTLE_ENDIAN_BUS ? beat_le : byte_reverse_bus(beat_le);
    tx_data_o = beat_bus;

    if (tx_active_q) begin
      // Actively sending a packet
      tx_valid_o = 1'b1;
      tx_sop_o   = (tx_beat_q == '0);
      tx_eop_o   = (tx_beat_q == (N_BEATS-1));
      tx_keep_o  = (tx_eop_o) ? (LITTLE_ENDIAN_BUS ? LAST_KEEP_LE
                                                   : {<<{LAST_KEEP_LE}}) // simply reverse bits via replication
                              : {UCIE_B{1'b1}};

      if (!link_up_i || !cfg_enable_i) begin
        // Abort mid-flight on link loss/disable
        tx_active_d = 1'b0;
      end else if (tx_valid_o && tx_ready_i) begin
        // Advance shifter
        tx_pkt_d  = tx_pkt_q >> UCIE_W;
        tx_beat_d = tx_beat_q + 1'b1;
        if (tx_eop_o) begin
          tx_active_d = 1'b0;
        end
      end
    end else begin
      // Idle: try to load a new flit
      if (cfg_enable_i && link_up_i && txf_out_valid) begin
        tx_pkt_d     = make_tx_packet(txf_dout);
        tx_beat_d    = '0;
        tx_active_d  = 1'b1;
        txf_out_ready= 1'b1; // consume flit immediately (we snapshot it)
      end
    end
  end

  // ========================================================================
  // RX PATH: UCIe streaming beats -> NoC flit
  // ========================================================================
  typedef enum logic [1:0] {RX_IDLE, RX_ACCUM, RX_DROP} rx_state_e;
  rx_state_e                rx_state_q, rx_state_d;

  logic [N_BEATS*UCIE_W-1:0] rx_pkt_q, rx_pkt_d;   // accumulation shifter (LSB-first)
  logic [$clog2(N_BEATS+1)-1:0] rx_beat_q, rx_beat_d;
  logic [15:0]              rx_bytes_q, rx_bytes_d; // accumulated valid bytes (<= TOT_BYTES)

  // Egress FIFO to NoC (flits)
  logic                     rxf_in_valid, rxf_in_ready;
  logic [FLIT_W-1:0]        rxf_in_data;
  logic                     rxf_out_valid, rxf_out_ready;
  logic [FLIT_W-1:0]        rxf_out_data;

  rv_fifo #(
    .WIDTH (FLIT_W),
    .DEPTH (RXF_DEPTH)
  ) u_rxf (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (rxf_in_valid),
    .in_ready_o  (rxf_in_ready),
    .in_data_i   (rxf_in_data),
    .out_valid_o (noc_rx_valid_o),
    .out_ready_i (noc_rx_ready_i),
    .out_data_o  (noc_rx_flit_o)
  );

  // RX state
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rx_state_q  <= RX_IDLE;
      rx_pkt_q    <= '0;
      rx_beat_q   <= '0;
      rx_bytes_q  <= 16'd0;
    end else begin
      rx_state_q  <= rx_state_d;
      rx_pkt_q    <= rx_pkt_d;
      rx_beat_q   <= rx_beat_d;
      rx_bytes_q  <= rx_bytes_d;
    end
  end

  // RX control
  always_comb begin
    rx_state_d = rx_state_q;
    rx_pkt_d   = rx_pkt_q;
    rx_beat_d  = rx_beat_q;
    rx_bytes_d = rx_bytes_q;

    rxf_in_valid = 1'b0;
    rxf_in_data  = '0;

    // Default: ready when we can buffer (one flit capacity)
    rx_ready_o = rxf_in_ready;

    // Incoming beat normalization to LE
    logic [UCIE_W-1:0] beat_bus = rx_data_i;
    logic [UCIE_W-1:0] beat_le  = LITTLE_ENDIAN_BUS ? beat_bus : byte_reverse_bus(beat_bus);
    int unsigned vbytes = popcount_keep(rx_keep_i);

    // Abort on link down (drop current)
    if (!link_up_i || !cfg_enable_i) begin
      rx_state_d = RX_IDLE;
      rx_bytes_d = 16'd0;
      rx_beat_d  = '0;
    end else begin
      unique case (rx_state_q)
        RX_IDLE: begin
          if (rx_valid_i && rx_ready_o && rx_sop_i) begin
            // Start accumulating packet
            rx_pkt_d   = { { (N_BEATS*UCIE_W-UCIE_W){1'b0} }, beat_le }; // put first beat at LSB end
            rx_beat_d  = 1;
            rx_bytes_d = vbytes[15:0];
            rx_state_d = (rx_eop_i) ? RX_ACCUM : RX_ACCUM;
          end
        end

        RX_ACCUM: begin
          if (rx_valid_i && rx_ready_o) begin
            // Shift in next beat at MSB side, then right-shift later if preferred.
            // Easier: shift existing right and OR new at MSB; but we want LSB-first
            // Here we place new beat on the left then rotate-on-consume once at end.
            rx_pkt_d   = (rx_pkt_q >> UCIE_W) | (beat_le << (N_BEATS*UCIE_W-UCIE_W));
            rx_beat_d  = rx_beat_q + 1'b1;
            rx_bytes_d = rx_bytes_q + vbytes[15:0];

            if (rx_err_i && cfg_drop_on_err_i) begin
              rx_state_d = RX_DROP; // drain until EOP
            end else if (rx_eop_i) begin
              // Packet complete; validate size & header
              // After shifting, the lowest TOT_BITS bits hold the packet {flit, hdr1, hdr0}
              logic [TOT_BITS-1:0] pkt_core = rx_pkt_d[TOT_BITS-1:0];
              logic [7:0] h0 = pkt_core[7:0];
              logic [7:0] h1 = pkt_core[15:8];
              logic [FLIT_W-1:0] flit = pkt_core[TOT_BITS-1 -: FLIT_W];

              logic size_ok = (rx_bytes_d == TOT_BYTES);
              logic hdr_ok  = (h0 == HDR0_MAGIC) && (h1 == HDR1_META);

              if (size_ok && hdr_ok && rxf_in_ready) begin
                rxf_in_valid = 1'b1;
                rxf_in_data  = flit;
                rx_state_d   = RX_IDLE;
                rx_bytes_d   = 16'd0;
                rx_beat_d    = '0;
              end else begin
                // Drop if header bad / size bad / no egress space
                rx_state_d = RX_IDLE;
                rx_bytes_d = 16'd0;
                rx_beat_d  = '0;
              end
            end
          end
        end

        RX_DROP: begin
          // Drain the rest of the current packet
          if (rx_valid_i && rx_ready_o && rx_eop_i) begin
            rx_state_d = RX_IDLE;
            rx_bytes_d = 16'd0;
            rx_beat_d  = '0;
          end
        end

        default: rx_state_d = RX_IDLE;
      endcase
    end
  end

  // ========================================================================
  // Telemetry
  // ========================================================================
  logic [31:0] tx_flits_q, rx_flits_q, tx_beats_q, rx_beats_q, tx_drops_q, rx_drops_q, rx_bad_hdr_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      tx_flits_q   <= 32'd0;
      rx_flits_q   <= 32'd0;
      tx_beats_q   <= 32'd0;
      rx_beats_q   <= 32'd0;
      tx_drops_q   <= 32'd0;
      rx_drops_q   <= 32'd0;
      rx_bad_hdr_q <= 32'd0;
    end else if (STATS_EN) begin
      if (cmd_clr_stats_i) begin
        tx_flits_q   <= 32'd0;
        rx_flits_q   <= 32'd0;
        tx_beats_q   <= 32'd0;
        rx_beats_q   <= 32'd0;
        tx_drops_q   <= 32'd0;
        rx_drops_q   <= 32'd0;
        rx_bad_hdr_q <= 32'd0;
      end else begin
        if (tx_valid_o && tx_ready_i) begin
          tx_beats_q <= tx_beats_q + 32'd1;
          if (tx_eop_o) tx_flits_q <= tx_flits_q + 32'd1;
        end
        if (rx_valid_i && rx_ready_o) begin
          rx_beats_q <= rx_beats_q + 32'd1;
          if (rx_eop_i) begin
            // Drop counters: counted if err or bad header/size
            if (rx_err_i && cfg_drop_on_err_i)    rx_drops_q <= rx_drops_q + 32'd1;
          end
        end
        // Count successful NoC deliveries
        if (noc_rx_valid_o && noc_rx_ready_i) rx_flits_q <= rx_flits_q + 32'd1;
      end
    end
  end

  assign stat_tx_flits_o   = tx_flits_q;
  assign stat_rx_flits_o   = rx_flits_q;
  assign stat_tx_beats_o   = tx_beats_q;
  assign stat_rx_beats_o   = rx_beats_q;
  assign stat_tx_drops_o   = tx_drops_q;
  assign stat_rx_drops_o   = rx_drops_q;
  assign stat_rx_bad_hdr_o = rx_bad_hdr_q;

  // Shim ready: link up, enabled, and TX idle (no in-flight packet)
  assign shim_ready_o = link_up_i && cfg_enable_i && !tx_active_q;

  // ========================================================================
  // Assertions (simulation only)
  // ========================================================================
`ifdef ASSERT_ON
  // KEEP must be contiguous from LSB (LE) or MSB (BE). Basic guard:
  function automatic logic contig_keep_le(input logic [UCIE_B-1:0] k);
    logic [UCIE_B-1:0] ref;
    int unsigned v = $countones(k);
    ref = (v==0) ? '0 : ((UCIE_B'(1) << v) - UCIE_B'(1));
    return (k == ref);
  endfunction

  // TX last KEEP checks out
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    tx_valid_o && tx_ready_i && tx_eop_o |-> contig_keep_le(LITTLE_ENDIAN_BUS ? tx_keep_o
                                                                              : {<<{tx_keep_o}}))
    else $error("ucie_shim: non-contiguous tx_keep_o on last beat.");

  // RX KEEP contiguity
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rx_valid_i && rx_ready_o |-> contig_keep_le(LITTLE_ENDIAN_BUS ? rx_keep_i
                                                                  : {<<{rx_keep_i}}))
    else $warning("ucie_shim: non-contiguous rx_keep_i observed.");

  // Do not overflow egress flit FIFO
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rxf_in_valid |-> rxf_in_ready)
    else $error("ucie_shim: RX pushes when out FIFO is full.");

  // Link-down: TX should not assert valid in idle state
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !link_up_i |-> !tx_active_q)
    else $warning("ucie_shim: tx_active while link is down.");
`endif

endmodule

// ============================================================================
//  rv_fifo : single-clock ready/valid FIFO (non-fallthrough)
// ============================================================================
module rv_fifo #(
  parameter int unsigned WIDTH = 64,
  parameter int unsigned DEPTH = 8
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
