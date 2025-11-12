// ============================================================================
//  wb_storeq.sv
//  Single-burst AXI4 writeback store queue wrapping wb_pack
//
//  Features
//  --------
//  - Descriptor-driven AXI4 write (AW/W/B) for one burst per descriptor.
//  - Unaligned start support: passes AWADDR[log2(BUS_BYTES)-1:0] to wb_pack.
//  - Validates descriptor: no 4KB crossing, ≤256 beats, nonzero length.
//  - Gates W channel until AW is accepted and wb_pack start is handshaked.
//  - Tracks bytes/beats; checks WLAST timing and byte count (assertions).
//  - Telemetry: descriptors accepted, beats, bytes, BRESP errors.
//
//  NOTE
//  ----
//  - This module assumes `wb_pack.sv` is available in the build.
//  - For multi-burst splitting, implement/instantiate a "splitter" wrapper
//    that presents smaller descriptors to this block.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module wb_storeq #(
  // ------------------------ AXI Geometry -------------------------
  parameter int unsigned AXI_ADDR_W   = 48,
  parameter int unsigned AXI_ID_W     = 4,
  parameter int unsigned AXI_USER_W   = 1,        // AWUSER width (0 → tied off)
  parameter int unsigned BUS_W        = 256,      // WDATA width (bits, multiple of 8)

  // ------------------------ Stream Geometry ----------------------
  parameter int unsigned LANES_IN     = 1,
  parameter int unsigned IN_ELEM_W    = 16,       // incoming element width (byte-aligned)
  parameter bit          IN_SIGNED    = 1'b1,

  // Store dtype (what we place on the bus after conversion/saturation)
  parameter int unsigned STORE_ELEM_W = 8,        // byte-aligned
  parameter bit          STORE_SIGNED = 1'b1,

  // ------------------------ Elasticity ---------------------------
  parameter int unsigned IN_FIFO_DEPTH  = 4,
  parameter int unsigned OUT_FIFO_DEPTH = 4,

  // ------------------------ AXI AW defaults ----------------------
  parameter logic [2:0]  P_AWPROT     = 3'b000,
  parameter logic [3:0]  P_AWCACHE    = 4'b0011,
  parameter logic [3:0]  P_AWQOS      = 4'b0000,
  parameter logic [3:0]  P_AWREGION   = 4'b0000
)(
  input  logic                           clk_i,
  input  logic                           rstn_i,          // synchronous, active-low

  // ===================== Descriptor In ===========================
  // One-burst write descriptor (must fit single AXI burst)
  input  logic                           desc_valid_i,
  output logic                           desc_ready_o,
  input  logic [AXI_ADDR_W-1:0]          desc_addr_i,     // byte address (unaligned allowed)
  input  logic [31:0]                    desc_bytes_i,    // total bytes to write (>0)
  input  logic [AXI_ID_W-1:0]            desc_id_i,
  input  logic [AXI_USER_W-1:0]          desc_awuser_i,   // ignored if AXI_USER_W==0

  // Done/response out (on BRESP)
  output logic                           done_valid_o,
  input  logic                           done_ready_i,
  output logic [AXI_ID_W-1:0]            done_id_o,
  output logic [1:0]                     done_bresp_o,    // AXI BRESP
  output logic                           done_err_o,      // 1 if BRESP != OKAY

  // ===================== C-Stream In (to be stored) ===============
  input  logic                           in_valid_i,
  output logic                           in_ready_o,
  input  logic [LANES_IN*IN_ELEM_W-1:0]  in_data_i,
  input  logic                           in_last_i,

  // Little-endian lane packing (1) or big-endian byte lanes (0)
  input  logic                           cfg_little_endian_i,

  // ===================== AXI4 Write Address (AW) ==================
  output logic                           aw_valid_o,
  input  logic                           aw_ready_i,
  output logic [AXI_ADDR_W-1:0]          aw_addr_o,
  output logic [7:0]                     aw_len_o,        // beats-1
  output logic [2:0]                     aw_size_o,       // log2(bytes/beat)
  output logic [1:0]                     aw_burst_o,      // INCR
  output logic [AXI_ID_W-1:0]            aw_id_o,
  output logic [2:0]                     aw_prot_o,
  output logic [3:0]                     aw_cache_o,
  output logic [3:0]                     aw_qos_o,
  output logic [3:0]                     aw_region_o,
  output logic [AXI_USER_W-1:0]          aw_user_o,
  output logic                           aw_lock_o,

  // ===================== AXI4 Write Data (W) ======================
  output logic                           w_valid_o,
  input  logic                           w_ready_i,
  output logic [BUS_W-1:0]               w_data_o,
  output logic [BUS_W/8-1:0]             w_strb_o,
  output logic                           w_last_o,

  // ===================== AXI4 Write Response (B) ==================
  input  logic                           b_valid_i,
  output logic                           b_ready_o,
  input  logic [1:0]                     b_resp_i,
  input  logic [AXI_ID_W-1:0]            b_id_i,

  // ===================== Telemetry ================================
  input  logic                           cmd_clr_stats_i,
  output logic [31:0]                    stat_desc_acc_o,
  output logic [31:0]                    stat_aw_beats_o,   // programmed beats per burst
  output logic [31:0]                    stat_w_beats_o,    // emitted W beats
  output logic [31:0]                    stat_w_bytes_o,    // emitted bytes = sum(popcount(WSTRB))
  output logic [31:0]                    stat_bresp_errs_o  // count of BRESP != OKAY
);

  // ==========================================================================
  // Static checks
  // ==========================================================================
  localparam int unsigned BUS_BYTES = BUS_W/8;
  localparam int unsigned AWSIZE    = (BUS_BYTES <= 1) ? 0 : $clog2(BUS_BYTES);

  initial begin
    if ((BUS_W % 8) != 0)        $error("wb_storeq: BUS_W must be a multiple of 8.");
    if ((IN_ELEM_W   % 8) != 0)  $error("wb_storeq: IN_ELEM_W must be a multiple of 8.");
    if ((STORE_ELEM_W% 8) != 0)  $error("wb_storeq: STORE_ELEM_W must be a multiple of 8.");
    // AWSIZE requires bytes/beat = power of two
    if ((BUS_BYTES & (BUS_BYTES-1)) != 0) $error("wb_storeq: BUS_BYTES must be a power of two.");
    if (LANES_IN*STORE_ELEM_W > BUS_W) $error("wb_storeq: LANES_IN*STORE_ELEM_W must be <= BUS_W.");
  end

  // ==========================================================================
  // Small helpers
  // ==========================================================================
  function automatic int unsigned ceil_div_int(input int unsigned a, input int unsigned b);
    return (a + b - 1) / b;
  endfunction

  // Count WSTRB bytes
  function automatic int unsigned popcount_bytes(input logic [BUS_BYTES-1:0] k);
    return $countones(k);
  endfunction

  // ==========================================================================
  // Descriptor accept & precompute burst parameters
  // ==========================================================================
  typedef struct packed {
    logic [AXI_ADDR_W-1:0] addr;
    logic [31:0]           bytes;
    logic [AXI_ID_W-1:0]   id;
    logic [AXI_USER_W-1:0] user;
  } desc_t;

  desc_t desc_q, desc_d;

  logic        have_desc_q, have_desc_d;
  logic        desc_fire;

  // Derived, held for active descriptor
  logic [7:0]  awlen_q,  awlen_d;         // beats-1
  logic [15:0] beats_q,  beats_d;         // beats (1..256)
  logic [11:0] a_off12_q, a_off12_d;      // addr[11:0] for 4KB check
  logic [$clog2(BUS_BYTES)-1:0] start_off_q, start_off_d; // addr % BUS_BYTES

  // Fit check (single burst constraints)
  logic        fit_ok_w;
  logic [15:0] beats_req_w;

  // Combinational pre-check on incoming descriptor
  always_comb begin
    int unsigned off       = desc_addr_i[ $clog2(BUS_BYTES)-1 : 0 ];
    int unsigned low12     = desc_addr_i[11:0];
    int unsigned req_bytes = desc_bytes_i;
    int unsigned req_beats = ceil_div_int(off + req_bytes, BUS_BYTES);
    beats_req_w = req_beats[15:0];

    // Must not cross 4KB boundary: low12 + req_beats*BUS_BYTES <= 4096
    int unsigned span_bytes = req_beats * BUS_BYTES;
    logic no_cross_4k = (low12 + span_bytes) <= 12'd4096;

    logic beats_ok   = (req_beats >= 1) && (req_beats <= 256);
    logic nonzero    = (req_bytes != 0);

    fit_ok_w = no_cross_4k && beats_ok && nonzero;
  end

  // Accept descriptor only when idle and it fits
  wire idle = !have_desc_q;
  assign desc_ready_o = idle && fit_ok_w;

  assign desc_fire = desc_valid_i && desc_ready_o;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      have_desc_q <= 1'b0;
      desc_q      <= '0;
      awlen_q     <= 8'd0;
      beats_q     <= 16'd0;
      a_off12_q   <= 12'd0;
      start_off_q <= '0;
    end else begin
      have_desc_q <= have_desc_d;
      desc_q      <= desc_d;
      awlen_q     <= awlen_d;
      beats_q     <= beats_d;
      a_off12_q   <= a_off12_d;
      start_off_q <= start_off_d;
    end
  end

  always_comb begin
    have_desc_d = have_desc_q;
    desc_d      = desc_q;
    awlen_d     = awlen_q;
    beats_d     = beats_q;
    a_off12_d   = a_off12_q;
    start_off_d = start_off_q;

    if (desc_fire) begin
      have_desc_d = 1'b1;
      desc_d.addr = desc_addr_i;
      desc_d.bytes= desc_bytes_i;
      desc_d.id   = desc_id_i;
      desc_d.user = desc_awuser_i;
      beats_d     = beats_req_w;
      awlen_d     = beats_req_w[7:0] - 8'd1;
      a_off12_d   = desc_addr_i[11:0];
      start_off_d = desc_addr_i[ $clog2(BUS_BYTES)-1 : 0 ];
    end

    // Clear when we finish BRESP (below)
  end

  // ==========================================================================
  // wb_pack instance (packs element stream into WDATA/WSTRB/WLAST)
  // ==========================================================================
  // Start handshake to wb_pack (issue once per descriptor, after AW handshake)
  logic pack_start_valid, pack_start_ready;
  logic [$clog2(BUS_BYTES)-1:0] pack_start_off;

  // wb_pack W side
  logic pack_w_valid, pack_w_ready;
  logic [BUS_W-1:0]      pack_w_data;
  logic [BUS_BYTES-1:0]  pack_w_strb;
  logic                  pack_w_last;

  // Gate W path until AW accepted and pack start handshaked
  logic w_gate_q, w_gate_d;

  wb_pack #(
    .LANES_IN      (LANES_IN),
    .IN_ELEM_W     (IN_ELEM_W),
    .IN_SIGNED     (IN_SIGNED),
    .STORE_ELEM_W  (STORE_ELEM_W),
    .STORE_SIGNED  (STORE_SIGNED),
    .BUS_W         (BUS_W),
    .IN_FIFO_DEPTH (IN_FIFO_DEPTH),
    .OUT_FIFO_DEPTH(OUT_FIFO_DEPTH)
  ) u_pack (
    .clk_i              (clk_i),
    .rstn_i             (rstn_i),

    .start_valid_i      (pack_start_valid),
    .start_ready_o      (pack_start_ready),
    .start_byte_off_i   (pack_start_off),

    .in_valid_i         (in_valid_i),
    .in_ready_o         (in_ready_o),
    .in_data_i          (in_data_i),
    .in_last_i          (in_last_i),

    .w_valid_o          (pack_w_valid),
    .w_ready_i          (pack_w_ready),
    .w_data_o           (pack_w_data),
    .w_strb_o           (pack_w_strb),
    .w_last_o           (pack_w_last),

    .cfg_little_endian_i(cfg_little_endian_i),

    .cmd_clr_stats_i    (cmd_clr_stats_i),
    .stat_in_beats_o    (),        // optional tap-out
    .stat_w_beats_o     (),        // optional tap-out
    .stat_elem_sats_o   ()
  );

  // ==========================================================================
  // AXI AW state and W gating
  // ==========================================================================
  typedef enum logic [2:0] {
    S_IDLE        = 3'd0,
    S_AW          = 3'd1,
    S_START_PACK  = 3'd2,
    S_W           = 3'd3,
    S_WAIT_B      = 3'd4,
    S_DONE_HOLD   = 3'd5
  } state_e;

  state_e state_q, state_d;

  // Write progress counters
  logic [15:0] beats_prog_q, beats_prog_d;  // W beats seen
  logic [31:0] bytes_prog_q, bytes_prog_d;  // sum of WSTRB bytes

  // AW channel outputs (combinational from latched desc)
  assign aw_addr_o   = desc_q.addr;
  assign aw_len_o    = awlen_q;
  assign aw_size_o   = AWSIZE[2:0];
  assign aw_burst_o  = 2'b01;                // INCR
  assign aw_id_o     = desc_q.id;
  assign aw_prot_o   = P_AWPROT;
  assign aw_cache_o  = P_AWCACHE;
  assign aw_qos_o    = P_AWQOS;
  assign aw_region_o = P_AWREGION;
  assign aw_user_o   = (AXI_USER_W==0) ? '0 : desc_q.user;
  assign aw_lock_o   = 1'b0;

  // Gate W channel with w_gate_q
  assign w_valid_o = pack_w_valid && w_gate_q;
  assign w_data_o  = pack_w_data;
  assign w_strb_o  = pack_w_strb;
  assign w_last_o  = pack_w_last;
  assign pack_w_ready = w_ready_i && w_gate_q;

  // Start handshake wiring
  assign pack_start_off = start_off_q;

  // B channel readiness: only assert when we are waiting for it and we can accept 'done'
  logic done_v_q, done_v_d;
  logic [AXI_ID_W-1:0] done_id_q, done_id_d;
  logic [1:0]          done_resp_q, done_resp_d;

  assign done_valid_o = done_v_q;
  assign done_id_o    = done_id_q;
  assign done_bresp_o = done_resp_q;
  assign done_err_o   = (done_resp_q != 2'b00); // OKAY=2'b00

  // We only accept B when our "done" slot is free
  assign b_ready_o = (state_q == S_WAIT_B) && !done_v_q;

  // AW valid when we have a descriptor and are in AW state
  assign aw_valid_o = (state_q == S_AW);

  // FSM
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q      <= S_IDLE;
      w_gate_q     <= 1'b0;
      beats_prog_q <= 16'd0;
      bytes_prog_q <= 32'd0;
      done_v_q     <= 1'b0;
      done_id_q    <= '0;
      done_resp_q  <= 2'b00;
      // descriptor ownership flag cleared above
      have_desc_q  <= 1'b0;
    end else begin
      state_q      <= state_d;
      w_gate_q     <= w_gate_d;
      beats_prog_q <= beats_prog_d;
      bytes_prog_q <= bytes_prog_d;
      done_v_q     <= done_v_d;
      done_id_q    <= done_id_d;
      done_resp_q  <= done_resp_d;
      have_desc_q  <= have_desc_d;
    end
  end

  // Next-state logic
  always_comb begin
    state_d       = state_q;
    w_gate_d      = w_gate_q;
    beats_prog_d  = beats_prog_q;
    bytes_prog_d  = bytes_prog_q;
    done_v_d      = done_v_q;
    done_id_d     = done_id_q;
    done_resp_d   = done_resp_q;

    // default: keep desc latched until DONE_HOLD
    have_desc_d   = have_desc_q;

    // pack start defaults
    pack_start_valid = 1'b0;

    // W progress
    if (w_valid_o && w_ready_i) begin
      beats_prog_d = beats_prog_q + 16'd1;
      bytes_prog_d = bytes_prog_q + popcount_bytes(w_strb_o);
    end

    // Done output handshake
    if (done_v_q && done_ready_i) begin
      done_v_d = 1'b0;
    end

    unique case (state_q)
      // ---------------------------------------------------------
      S_IDLE: begin
        w_gate_d      = 1'b0;
        beats_prog_d  = 16'd0;
        bytes_prog_d  = 32'd0;
        if (desc_fire) begin
          state_d = S_AW;
          // have_desc_d set above in descriptor latch logic
        end
      end

      // ---------------------------------------------------------
      S_AW: begin
        // Wait until AW is accepted
        if (aw_valid_o && aw_ready_i) begin
          // Trigger pack start (with byte offset) next
          state_d         = S_START_PACK;
        end
      end

      // ---------------------------------------------------------
      S_START_PACK: begin
        // Present start to wb_pack until it accepts
        pack_start_valid = 1'b1;
        if (pack_start_valid && pack_start_ready) begin
          // From now, allow W channel flow
          w_gate_d  = 1'b1;
          state_d   = S_W;
        end
      end

      // ---------------------------------------------------------
      S_W: begin
        // Wait until packer asserts WLAST and the beat is accepted
        if (w_valid_o && w_ready_i && w_last_o) begin
          // Freeze W gate, move to BRESP wait
          w_gate_d = 1'b0;
          state_d  = S_WAIT_B;
        end
      end

      // ---------------------------------------------------------
      S_WAIT_B: begin
        if (b_valid_i && b_ready_o) begin
          // Capture done info; present to software/upper layer
          done_v_d    = 1'b1;
          done_id_d   = b_id_i;
          done_resp_d = b_resp_i;
          state_d     = S_DONE_HOLD;
        end
      end

      // ---------------------------------------------------------
      S_DONE_HOLD: begin
        // Release descriptor ownership when done info consumed
        if (!done_v_q || (done_v_q && done_ready_i)) begin
          have_desc_d = 1'b0;
          state_d     = S_IDLE;
        end
      end

      default: state_d = S_IDLE;
    endcase
  end

  // ==========================================================================
  // Telemetry
  // ==========================================================================
  logic [31:0] stat_desc_acc_q, stat_aw_beats_q, stat_w_beats_q, stat_w_bytes_q, stat_bresp_errs_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      stat_desc_acc_q  <= 32'd0;
      stat_aw_beats_q  <= 32'd0;
      stat_w_beats_q   <= 32'd0;
      stat_w_bytes_q   <= 32'd0;
      stat_bresp_errs_q<= 32'd0;
    end else if (cmd_clr_stats_i) begin
      stat_desc_acc_q  <= 32'd0;
      stat_aw_beats_q  <= 32'd0;
      stat_w_beats_q   <= 32'd0;
      stat_w_bytes_q   <= 32'd0;
      stat_bresp_errs_q<= 32'd0;
    end else begin
      if (desc_fire) begin
        stat_desc_acc_q <= stat_desc_acc_q + 32'd1;
        stat_aw_beats_q <= stat_aw_beats_q + beats_req_w;
      end
      if (w_valid_o && w_ready_i) begin
        stat_w_beats_q  <= stat_w_beats_q  + 32'd1;
        stat_w_bytes_q  <= stat_w_bytes_q  + popcount_bytes(w_strb_o);
      end
      if (b_valid_i && b_ready_o && (b_resp_i != 2'b00)) begin
        stat_bresp_errs_q <= stat_bresp_errs_q + 32'd1;
      end
    end
  end

  assign stat_desc_acc_o  = stat_desc_acc_q;
  assign stat_aw_beats_o  = stat_aw_beats_q;
  assign stat_w_beats_o   = stat_w_beats_q;
  assign stat_w_bytes_o   = stat_w_bytes_q;
  assign stat_bresp_errs_o= stat_bresp_errs_q;

  // ==========================================================================
  // Assertions (simulation-only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Descriptor must fit, by contract
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    desc_fire |-> fit_ok_w)
    else $error("wb_storeq: descriptor does not fit single burst (4KB or >256 beats or zero length).");

  // AWLEN must match beats-1
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    desc_fire |-> (awlen_d == (beats_req_w[7:0]-8'd1)))
    else $error("wb_storeq: AWLEN mismatch.");

  // No W before AW accepted & pack started
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (pack_w_valid && !w_gate_q) |-> !w_valid_o)
    else $error("wb_storeq: W gated violation.");

  // WLAST must occur on the programmed final beat
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (w_valid_o && w_ready_i && w_last_o) |-> (beats_prog_q + 16'd1 == beats_q))
    else $warning("wb_storeq: WLAST not on final programmed beat (beats_prog=%0d beats=%0d).",
                  beats_prog_q, beats_q);

  // Total bytes emitted should equal descriptor bytes (checked when WLAST beats pulses)
  // This uses STRB counting; allows holes from offset masking.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (w_valid_o && w_ready_i && w_last_o) |-> (bytes_prog_q + popcount_bytes(w_strb_o) == desc_q.bytes))
    else $warning("wb_storeq: byte count mismatch on WLAST (emitted vs desc).");

  // B comes only after WLAST (protocol order)
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    b_valid_i |-> (state_q == S_WAIT_B))
    else $error("wb_storeq: BRESP observed outside S_WAIT_B.");

`endif

endmodule

// ============================================================================
//  (Dependency) rv_fifo can be provided by wb_pack; not needed here.
//  This file intentionally avoids a local FIFO to minimize duplication.
// ============================================================================

`default_nettype wire
