// ============================================================================
//  bus_drain_c.sv
//  Streaming sink for "C-path" (e.g., partial sums / results).
//  - Absorbs a byte-enabled stream with valid/ready/last.
//  - Programmable back-pressure profiles (fixed gap, credit/pause, random).
//  - Computes CRC32C per frame over asserted byte lanes.
//  - Exposes rich telemetry counters for bring-up and soak.
//  - No combinational ready loops (all decisions registered).
//
//  Stream (input to this block):
//    c_valid_i, c_ready_o, c_data_i[DATA_W-1:0], c_strb_i[DATA_W/8-1:0], c_last_i
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module bus_drain_c #(
  parameter int unsigned DATA_W      = 512,  // bits, multiple of 8
  parameter int unsigned GAP_W       = 16,   // fixed gap / pause counters
  parameter int unsigned CREDIT_W    = 16,   // credit budget width
  parameter int unsigned STALL_W     = 8     // random stall max (cycles)
)(
  input  logic                        clk_i,
  input  logic                        rstn_i,          // synchronous, active-low

  // ---------------- Stream In ----------------
  input  logic                        c_valid_i,
  output logic                        c_ready_o,
  input  logic [DATA_W-1:0]           c_data_i,
  input  logic [DATA_W/8-1:0]         c_strb_i,        // 1 bit per byte (aka TKEEP/WSTRB)
  input  logic                        c_last_i,

  // ---------------- Control ------------------
  input  logic                        cfg_enable_i,    // 0=bypass-accept (always-ready) with counters running

  // Back-pressure mode:
  //   0: ALWAYS_READY
  //   1: FIXED_GAP        (ready drops for cfg_gap_cyc_i cycles after each accepted beat)
  //   2: CREDIT_PAUSE     (accept cfg_credit_init_i beats, then pause cfg_pause_cyc_i cycles)
  //   3: RND_STALL        (on random trigger, stall up to cfg_stall_max_i cycles)
  input  logic [1:0]                  cfg_bp_mode_i,

  // Fixed-gap mode
  input  logic [GAP_W-1:0]            cfg_gap_cyc_i,

  // Credit/Pause mode
  input  logic [CREDIT_W-1:0]         cfg_credit_init_i,
  input  logic [GAP_W-1:0]            cfg_pause_cyc_i,

  // Random-stall mode (Galois LFSR based)
  input  logic [7:0]                  cfg_prob_stall_i,    // 0..255 threshold
  input  logic [STALL_W-1:0]          cfg_stall_max_i,     // max stall length when triggered

  // LFSR reseed for deterministic behavior
  input  logic                        cfg_reseed_i,
  input  logic [31:0]                 cfg_seed_i,

  // CRC configuration
  input  logic [31:0]                 cfg_crc_init_i,      // initial CRC value per frame (typ. 32'hFFFF_FFFF)
  input  logic [31:0]                 cfg_crc_xorout_i,    // XOR-out at end (typ. 32'hFFFF_FFFF)

  // Commands
  input  logic                        cmd_clear_counters_i, // pulse: clear telemetry counters
  input  logic                        cmd_soft_reset_crc_i, // pulse: reinit CRC without dropping in-flight frame

  // ---------------- Telemetry ----------------
  output logic                        stat_in_frame_o,     // 1 when inside a frame (since last accepted SOF)
  output logic [31:0]                 stat_cycles_o,
  output logic [31:0]                 stat_cycles_ready_o,
  output logic [31:0]                 stat_cycles_stalled_o,

  output logic [31:0]                 stat_beats_acc_o,
  output logic [31:0]                 stat_bytes_acc_o,
  output logic [31:0]                 stat_frames_acc_o,

  output logic [31:0]                 stat_frames_zero_bytes_o, // frames with zero effective bytes (strb all 0)
  output logic [31:0]                 stat_stall_events_o,     // number of stall episodes started
  output logic [31:0]                 stat_bp_transitions_o,   // count of ready rising edges

  output logic [31:0]                 stat_crc_last_o,         // CRC32C of last completed frame (after XOR-out)
  output logic [31:0]                 stat_crc_running_o,      // live CRC state (internal, before XOR-out)
  output logic [31:0]                 stat_last_frame_beats_o,
  output logic [31:0]                 stat_last_frame_bytes_o,

  output logic                        stat_fault_reset_in_frame_o // sticky: reset occurred mid-frame
);

  // ==========================================================================
  // Parameter checks
  // ==========================================================================
  initial begin
    if (DATA_W % 8 != 0) $error("bus_drain_c: DATA_W must be a multiple of 8.");
  end
  localparam int unsigned BYTES = DATA_W/8;

  // ==========================================================================
  // LFSR32 (Galois) for random stall decisions: x^32 + x^22 + x^2 + x + 1
  // ==========================================================================
  logic [31:0] lfsr_q;
  function automatic logic [31:0] lfsr32_next(input logic [31:0] s);
    logic fb = s[0];
    logic [31:0] n;
    n = {1'b0, s[31:1]};
    if (fb) begin
      n[31] ^= 1'b1;
      n[21] ^= 1'b1;
      n[1]  ^= 1'b1;
      n[0]  ^= 1'b1;
    end
    return n;
  endfunction

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i)        lfsr_q <= 32'h1ACE_FEED;
    else if (cfg_reseed_i) lfsr_q <= (cfg_seed_i == 32'h0) ? 32'h1 : cfg_seed_i;
    else                lfsr_q <= lfsr32_next(lfsr_q);
  end

  // ==========================================================================
  // CRC32C (Castagnoli) — reflected (poly 0x82F63B78), byte-wise update
  // ==========================================================================
  function automatic logic [31:0] crc32c_next8(
    input logic [31:0] crc_in, input logic [7:0] data
  );
    logic [31:0] c; int i;
    c = crc_in ^ {24'h0, data};
    for (i=0; i<8; i++) begin
      if (c[0]) c = (c >> 1) ^ 32'h82F63B78;
      else      c = (c >> 1);
    end
    return c;
  endfunction

  function automatic logic [31:0] crc32c_next_beat(
    input logic [31:0] crc_in,
    input logic [DATA_W-1:0] data,
    input logic [BYTES-1:0]  strb
  );
    logic [31:0] c;
    c = crc_in;
    // Process bytes little-endian by byte index 0..BYTES-1
    for (int b=0; b<BYTES; b++) begin
      if (strb[b]) c = crc32c_next8(c, data[b*8 +: 8]);
    end
    return c;
  endfunction

  // ==========================================================================
  // Back-pressure control state
  // ==========================================================================
  typedef enum logic [1:0] { BP_ALWAYS=2'd0, BP_GAP=2'd1, BP_CREDIT=2'd2, BP_RND=2'd3 } bp_e;

  logic [GAP_W-1:0]    gap_cnt_q,    gap_cnt_d;
  logic [CREDIT_W-1:0] credit_q,     credit_d;
  logic [GAP_W-1:0]    pause_cnt_q,  pause_cnt_d;
  logic [STALL_W-1:0]  stall_cnt_q,  stall_cnt_d;
  logic                stall_active_q, stall_active_d;

  // Ready generation (registered)
  logic ready_q, ready_d;

  // For transitions counting
  logic ready_q_prev;

  // ==========================================================================
  // Telemetry counters & frame tracking
  // ==========================================================================
  logic        in_frame_q, in_frame_d;
  logic [31:0] crc_q, crc_d;
  logic [31:0] bytes_in_frame_q, bytes_in_frame_d;
  logic [31:0] beats_in_frame_q, beats_in_frame_d;

  // Sticky fault: reset occurred mid-frame
  logic fault_reset_in_frame_q, fault_reset_in_frame_d;

  // Wide counters
  logic [31:0] cycles_q, cycles_ready_q, cycles_stalled_q;
  logic [31:0] beats_acc_q, bytes_acc_q, frames_acc_q;
  logic [31:0] frames_zero_bytes_q, stall_events_q, bp_transitions_q;
  logic [31:0] crc_last_q, last_frame_beats_q, last_frame_bytes_q;

  // Clear counters
  wire clr = cmd_clear_counters_i;

  // ==========================================================================
  // Ready / back-pressure state machine (registered ready)
  // ==========================================================================
  wire fire = c_valid_i && ready_q;

  // Random stall trigger
  wire rnd_stall_trig = (lfsr_q[7:0] < cfg_prob_stall_i) && (cfg_prob_stall_i != 8'd0);

  always_comb begin
    // Defaults
    ready_d         = 1'b1;
    gap_cnt_d       = gap_cnt_q;
    credit_d        = credit_q;
    pause_cnt_d     = pause_cnt_q;
    stall_cnt_d     = stall_cnt_q;
    stall_active_d  = stall_active_q;

    // Back-pressure profiles (if disabled, still compute counters but hold ready=1)
    unique case (cfg_bp_mode_i)
      BP_ALWAYS: begin
        ready_d = 1'b1;
      end

      BP_GAP: begin
        if (gap_cnt_q != '0) begin
          ready_d   = 1'b0;
          gap_cnt_d = gap_cnt_q - {{(GAP_W-1){1'b0}},1'b1};
        end else begin
          ready_d = 1'b1;
          if (fire) gap_cnt_d = cfg_gap_cyc_i; // reload after acceptance
        end
      end

      BP_CREDIT: begin
        // Simple token bucket: when credit>0 -> ready=1; when 0 -> pause for cfg_pause_cyc_i
        if (credit_q != '0) begin
          ready_d = 1'b1;
          if (fire) credit_d = credit_q - {{(CREDIT_W-1){1'b0}},1'b1};
        end else begin
          // Pause window
          if (pause_cnt_q != '0) begin
            ready_d     = 1'b0;
            pause_cnt_d = pause_cnt_q - {{(GAP_W-1){1'b0}},1'b1};
          end else begin
            // Refill
            credit_d    = cfg_credit_init_i;
            pause_cnt_d = cfg_pause_cyc_i;
            ready_d     = (cfg_credit_init_i != '0);
          end
        end
      end

      BP_RND: begin
        if (stall_active_q) begin
          ready_d    = 1'b0;
          if (stall_cnt_q != '0) stall_cnt_d = stall_cnt_q - {{(STALL_W-1){1'b0}},1'b1};
          else                   stall_active_d = 1'b0; // stall ends next cycle
        end else begin
          ready_d = 1'b1;
          // Start a stall after an acceptance or freely? Choose “after acceptance” to regulate rate.
          if (fire && rnd_stall_trig) begin
            stall_active_d = 1'b1;
            stall_cnt_d    = (cfg_stall_max_i == '0) ? '0
                              : (lfsr_q[STALL_W-1:0] % (cfg_stall_max_i + {{(STALL_W-1){1'b0}},1'b1}));
          end
        end
      end

      default: ready_d = 1'b1;
    endcase
  end

  // ==========================================================================
  // Sequential: ready/reg state, counters, CRC, frame tracking
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      ready_q               <= 1'b1;
      ready_q_prev          <= 1'b0;
      gap_cnt_q             <= '0;
      credit_q              <= '0;
      pause_cnt_q           <= '0;
      stall_cnt_q           <= '0;
      stall_active_q        <= 1'b0;

      cycles_q              <= 32'd0;
      cycles_ready_q        <= 32'd0;
      cycles_stalled_q      <= 32'd0;
      beats_acc_q           <= 32'd0;
      bytes_acc_q           <= 32'd0;
      frames_acc_q          <= 32'd0;
      frames_zero_bytes_q   <= 32'd0;
      stall_events_q        <= 32'd0;
      bp_transitions_q      <= 32'd0;

      in_frame_q            <= 1'b0;
      crc_q                 <= 32'h0;
      bytes_in_frame_q      <= 32'd0;
      beats_in_frame_q      <= 32'd0;
      crc_last_q            <= 32'h0;
      last_frame_beats_q    <= 32'd0;
      last_frame_bytes_q    <= 32'd0;

      fault_reset_in_frame_q<= 1'b0;
    end else begin
      // Ready and BP state
      ready_q      <= cfg_enable_i ? ready_d : 1'b1;
      ready_q_prev <= ready_q;

      gap_cnt_q    <= gap_cnt_d;
      credit_q     <= credit_d;
      pause_cnt_q  <= pause_cnt_d;
      stall_cnt_q  <= stall_cnt_d;

      // Count a new stall episode whenever RANDOM mode arms a stall
      if (!stall_active_q && stall_active_d) stall_events_q <= (clr ? 32'd1 : (stall_events_q + 32'd1))
                                                           - (clr ? stall_events_q : 32'd0);
      stall_active_q <= stall_active_d;

      // Per-cycle counters
      cycles_q         <= clr ? 32'd0 : (cycles_q + 32'd1);
      cycles_ready_q   <= clr ? 32'd0 : (cycles_ready_q + (ready_q ? 32'd1 : 32'd0));
      cycles_stalled_q <= clr ? 32'd0 : (cycles_stalled_q + (!ready_q ? 32'd1 : 32'd0));

      // Count ready transitions (rising edges)
      if (ready_q && !ready_q_prev) begin
        bp_transitions_q <= clr ? 32'd1 : (bp_transitions_q + 32'd1);
      end else if (clr) begin
        bp_transitions_q <= 32'd0;
      end

      // Frame reset behavior
      if (!rstn_i) begin end // (placeholder - already in reset branch)
      else if (!cfg_enable_i) begin
        // Still accept & count even if disabled; ready forced high.
      end

      // Fault: reset-in-frame sticky (if an external reset happens mid-frame, flag)
      if (!rstn_i) begin end else begin
        if (!rstn_i) fault_reset_in_frame_q <= fault_reset_in_frame_q | in_frame_q;
      end

      // CRC state soft reset (keep in_frame)
      if (cmd_soft_reset_crc_i) begin
        crc_q <= cfg_crc_init_i;
      end

      // Acceptance path (valid & ready)
      if (c_valid_i && ready_q) begin
        // Enter frame on first accepted beat after being idle
        if (!in_frame_q) begin
          in_frame_q       <= 1'b1;
          crc_q            <= cfg_crc_init_i;
          bytes_in_frame_q <= 32'd0;
          beats_in_frame_q <= 32'd0;
        end

        // Accumulate CRC over asserted bytes
        crc_q            <= crc32c_next_beat(crc_q, c_data_i, c_strb_i);

        // Count this beat + effective bytes
        beats_acc_q      <= clr ? 32'd1 : (beats_acc_q + 32'd1);
        bytes_acc_q      <= clr ? 32'(BYTES'(c_strb_i.countones())) :
                                  (bytes_acc_q + 32'(BYTES'(c_strb_i.countones())));
        beats_in_frame_q <= beats_in_frame_q + 32'd1;
        bytes_in_frame_q <= bytes_in_frame_q + 32'(BYTES'(c_strb_i.countones()));

        // On last, close frame
        if (c_last_i) begin
          in_frame_q           <= 1'b0;
          frames_acc_q         <= clr ? 32'd1 : (frames_acc_q + 32'd1);
          if (bytes_in_frame_q + 32'(BYTES'(c_strb_i.countones())) == 32'd0)
            frames_zero_bytes_q <= clr ? 32'd1 : (frames_zero_bytes_q + 32'd1);

          crc_last_q           <= (crc32c_next_beat(crc_q, c_data_i, c_strb_i) ^ cfg_crc_xorout_i);

          last_frame_beats_q   <= beats_in_frame_q + 32'd1;
          last_frame_bytes_q   <= bytes_in_frame_q + 32'(BYTES'(c_strb_i.countones()));
        end
      end

      // Clear counters (edge)
      if (clr) begin
        cycles_q            <= 32'd0;
        cycles_ready_q      <= 32'd0;
        cycles_stalled_q    <= 32'd0;
        beats_acc_q         <= 32'd0;
        bytes_acc_q         <= 32'd0;
        frames_acc_q        <= 32'd0;
        frames_zero_bytes_q <= 32'd0;
        stall_events_q      <= 32'd0;
        bp_transitions_q    <= 32'd0;
        // Keep crc_last_q so software can read last frame CRC after clear; comment next line if you prefer to clear it
        // crc_last_q        <= 32'd0;
        last_frame_beats_q  <= 32'd0;
        last_frame_bytes_q  <= 32'd0;
      end
    end
  end

  // Drive outputs
  assign c_ready_o                = ready_q;

  assign stat_in_frame_o          = in_frame_q;
  assign stat_cycles_o            = cycles_q;
  assign stat_cycles_ready_o      = cycles_ready_q;
  assign stat_cycles_stalled_o    = cycles_stalled_q;

  assign stat_beats_acc_o         = beats_acc_q;
  assign stat_bytes_acc_o         = bytes_acc_q;
  assign stat_frames_acc_o        = frames_acc_q;

  assign stat_frames_zero_bytes_o = frames_zero_bytes_q;
  assign stat_stall_events_o      = stall_events_q;
  assign stat_bp_transitions_o    = bp_transitions_q;

  assign stat_crc_last_o          = crc_last_q;
  assign stat_crc_running_o       = crc_q;
  assign stat_last_frame_beats_o  = last_frame_beats_q;
  assign stat_last_frame_bytes_o  = last_frame_bytes_q;

  assign stat_fault_reset_in_frame_o = fault_reset_in_frame_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Ready must be 1 in ALWAYS mode
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (cfg_bp_mode_i == BP_ALWAYS) |-> c_ready_o)
    else $error("bus_drain_c: READY deasserted in ALWAYS mode.");

  // last must only terminate frames when in_frame=1
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (c_valid_i && c_ready_o && c_last_i) |-> in_frame_q || (beats_in_frame_q == 0))
    else $warning("bus_drain_c: LAST seen while not in-frame.");

  // Strb has no X when valid (optional)
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    c_valid_i |-> !$isunknown(c_strb_i))
    else $warning("bus_drain_c: STRB unknown under VALID.");
`endif

endmodule

`default_nettype wire
