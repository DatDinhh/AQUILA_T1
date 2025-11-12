// ============================================================================
//  array_ctrl.sv
//  Tile / K-step controller for a systolic array driven via array_rowcol_net.sv
//
//  Features
//    - Start/Pause/Abort commands
//    - Prefetch handshakes for A/W (kick + done)
//    - Optional overlap of prefetch with drain for the next tile
//    - K-step streaming with optional inter-step gaps (throttle)
//    - Telemetry counters & sticky error flags
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module array_ctrl #(
  // K length / tiling counter widths
  parameter int unsigned K_W     = 16,  // width for K length (steps per tile)
  parameter int unsigned TILE_W  = 16   // width for M×N tile loops (linearized)
)(
  input  logic                    clk_i,
  input  logic                    rstn_i,          // synchronous, active-low

  // ---------------- Commands ----------------
  input  logic                    cfg_enable_i,    // global enable gate
  input  logic                    cmd_start_i,     // pulse: begin a multi-tile job
  input  logic                    cmd_abort_i,     // pulse: immediate stop -> IDLE
  input  logic                    cmd_pause_i,     // level: pause streaming (sticky until resume)
  input  logic                    cmd_resume_i,    // pulse: clears pause

  // ---------------- Tile programming ----------------
  input  logic [K_W-1:0]          cfg_k_len_i,     // steps per tile (>0)
  input  logic [TILE_W-1:0]       cfg_tiles_total_i, // number of tiles to run (linearized M×N)
  input  logic [15:0]             cfg_gap_cyc_i,   // cycles to idle between accepted steps (throttle)
  input  logic                    cfg_overlap_prefetch_i, // 1: prefetch next tile during DRAIN

  // ---------------- Prefetch handshakes (to/from feeders) ----------------
  // Kick at tile start, wait for *_done before RUN. If overlap enabled, kick next tile
  // as soon as we enter DRAIN for current tile.
  output logic                    kick_a_prefetch_o,
  output logic                    kick_w_prefetch_o,
  input  logic                    a_prefetch_done_i,
  input  logic                    w_prefetch_done_i,

  // ---------------- Array step handshake (to/from array_rowcol_net) ------
  output logic                    step_valid_o,
  output logic                    step_first_o,
  output logic                    step_last_o,
  output logic                    clear_acc_o,     // 1-cycle pulse at tile start
  input  logic                    step_ready_i,    // AND of all cluster readies

  // ---------------- Result-drain boundary cue (from array_rowcol_net) ----
  // One array tile produces 4 * (R*C) beats; your net asserts c_tile_last_i
  // on the final beat of that tile (global gather).
  input  logic                    c_tile_last_i,
  input  logic                    c_tile_valid_i,  // valid qualifies c_tile_last_i

  // ---------------- Status / Telemetry -----------------------------------
  output logic                    active_o,        // 1 when controller not IDLE
  output logic                    paused_o,        // reflects pause state
  output logic [K_W-1:0]          k_ctr_o,         // current K index during RUN
  output logic [TILE_W-1:0]       tile_idx_o,      // which tile (0..cfg_tiles_total_i-1)
  output logic [31:0]             steps_issued_o,  // accepted steps
  output logic [31:0]             stall_cycles_o,  // cycles waiting for step_ready_i
  output logic [31:0]             drain_cycles_o,  // cycles spent draining
  output logic [31:0]             tiles_done_o,    // tiles completed
  output logic                    err_zero_k_sticky_o, // started with K==0
  output logic                    err_c_last_timeout_o  // sticky: drain timeout (optional guard)
);

  // ==========================================================================
  // Parameter checks
  // ==========================================================================
  initial begin
    if (K_W < 1) $error("array_ctrl: K_W must be >=1");
    if (TILE_W < 1) $error("array_ctrl: TILE_W must be >=1");
  end

  // ==========================================================================
  // FSM
  // ==========================================================================
  typedef enum logic [2:0] {
    S_IDLE,
    S_PREFILL,
    S_RUN,
    S_DRAIN,
    S_PAUSE
  } state_e;

  state_e st_q, st_d;

  // ==========================================================================
  // State / counters
  // ==========================================================================
  // Tile index and remaining tiles
  logic [TILE_W-1:0] tile_idx_q, tile_idx_d;
  logic [TILE_W-1:0] tiles_left_q, tiles_left_d;

  // K counter within a tile
  logic [K_W-1:0]    k_ctr_q, k_ctr_d;

  // Step throttle gap counter
  logic [15:0]       gap_q, gap_d;

  // Pause latch
  logic              paused_q, paused_d;

  // Prefetch tracking
  logic              a_done_q, a_done_d;
  logic              w_done_q, w_done_d;
  logic              kick_a_q, kick_a_d;
  logic              kick_w_q, kick_w_d;

  // One-cycle pulses (registered outputs)
  logic              clear_acc_q, clear_acc_d;

  // Telemetry counters
  logic [31:0]       steps_q, steps_d;
  logic [31:0]       stall_q, stall_d;
  logic [31:0]       drain_q, drain_d;
  logic [31:0]       tiles_q, tiles_d;

  // Errors
  logic              err_zero_k_q, err_zero_k_d;
  logic              err_drain_to_q, err_drain_to_d;

  // Optional drain timeout (defensive): if you want one, set a large bound here.
  localparam int unsigned DRAIN_TO_W = 24;
  logic [DRAIN_TO_W-1:0] drain_to_ctr_q, drain_to_ctr_d;

  // ==========================================================================
  // Derived wires
  // ==========================================================================
  wire go           = cfg_enable_i && cmd_start_i && (st_q == S_IDLE);
  wire abort_now    = cmd_abort_i;
  wire pause_req    = cmd_pause_i;
  wire resume_req   = cmd_resume_i;

  // Step fire: we assert step_valid_o in S_RUN, array returns step_ready_i.
  // A step "fires" when both 1 in the same cycle.
  wire step_fire    = step_valid_o && step_ready_i;

  // Last step accepted this tile?
  wire last_step_fire = step_fire && (k_ctr_q == (cfg_k_len_i - K_W'(1)));

  // Drain completion condition (qualify last flag with valid)
  wire drain_done   = (st_q == S_DRAIN) && c_tile_valid_i && c_tile_last_i;

  // ==========================================================================
  // Defaults
  // ==========================================================================
  always_comb begin
    st_d          = st_q;
    tile_idx_d    = tile_idx_q;
    tiles_left_d  = tiles_left_q;

    k_ctr_d       = k_ctr_q;
    gap_d         = gap_q;

    paused_d      = paused_q;

    a_done_d      = a_done_q;
    w_done_d      = w_done_q;
    kick_a_d      = 1'b0;  // single-cycle pulse
    kick_w_d      = 1'b0;

    clear_acc_d   = 1'b0;  // single-cycle pulse

    steps_d       = steps_q;
    stall_d       = stall_q;
    drain_d       = drain_q;
    tiles_d       = tiles_q;

    err_zero_k_d  = err_zero_k_q;
    err_drain_to_d= err_drain_to_q;
    drain_to_ctr_d= drain_to_ctr_q;

    // ---------------- Global transitions ----------------
    if (abort_now) begin
      st_d = S_IDLE;
    end

    if (pause_req && (st_q == S_RUN)) begin
      st_d     = S_PAUSE;
      paused_d = 1'b1;
    end
    if (resume_req) begin
      paused_d = 1'b0;
      if (st_q == S_PAUSE) st_d = S_RUN;
    end

    // ---------------- IDLE ----------------
    if (st_q == S_IDLE) begin
      if (go) begin
        // Program acceptance
        tiles_left_d  = (cfg_tiles_total_i == 0) ? TILE_W'(1) : cfg_tiles_total_i;
        tile_idx_d    = '0;

        // Guard: K must be >0
        if (cfg_k_len_i == '0) begin
          err_zero_k_d = 1'b1;
          // Remain in IDLE; nothing to do
        end else begin
          // Kick prefetch for first tile
          kick_a_d     = 1'b1;
          kick_w_d     = 1'b1;
          a_done_d     = 1'b0;
          w_done_d     = 1'b0;

          st_d         = S_PREFILL;
        end
      end
    end

    // ---------------- PREFILL ----------------
    else if (st_q == S_PREFILL) begin
      // Track feeder completions
      a_done_d = a_done_q | a_prefetch_done_i;
      w_done_d = w_done_q | w_prefetch_done_i;

      // Once both ready → start tile
      if (a_done_d && w_done_d) begin
        // Initialize K/run counters
        k_ctr_d     = '0;
        gap_d       = 16'd0;
        clear_acc_d = 1'b1;         // pulse at tile start
        st_d        = S_RUN;
      end
    end

    // ---------------- RUN (emit K steps) ----------------
    else if (st_q == S_RUN) begin
      // Throttle bookkeeping
      if (gap_q != 16'd0) begin
        gap_d = gap_q - 16'd1;
      end

      // Stall cycles: ready low while we want to run and not throttled
      if ((gap_q == 16'd0) && !cmd_pause_i && !paused_d) begin
        if (!step_ready_i) stall_d = stall_q + 32'd1;
      end

      // On step fire, advance K
      if (step_fire) begin
        steps_d = steps_q + 32'd1;

        if (k_ctr_q == (cfg_k_len_i - K_W'(1))) begin
          // Finished K: go to drain
          st_d   = S_DRAIN;
          // Optionally kick next tile prefetch ASAP (overlap)
          if (cfg_overlap_prefetch_i) begin
            kick_a_d = 1'b1;
            kick_w_d = 1'b1;
            a_done_d = 1'b0;
            w_done_d = 1'b0;
          end
          // Initialize drain timeout counter
          drain_to_ctr_d = '0;
        end else begin
          k_ctr_d = k_ctr_q + K_W'(1);
          gap_d   = cfg_gap_cyc_i;
        end
      end
    end

    // ---------------- DRAIN (wait for net to finish emitting tile) --------
    else if (st_q == S_DRAIN) begin
      drain_d = drain_q + 32'd1;

      // Optional prefetch (if not overlapped earlier)
      if (!cfg_overlap_prefetch_i && (tiles_left_q > TILE_W'(1))) begin
        // Kick only once after entering DRAIN
        if (!a_done_q && !w_done_q) begin
          kick_a_d = 1'b1;
          kick_w_d = 1'b1;
        end
        a_done_d = a_done_q | a_prefetch_done_i;
        w_done_d = w_done_q | w_prefetch_done_i;
      end

      // Drain timeout (defensive; can be ignored if undesired)
      drain_to_ctr_d = drain_to_ctr_q + DRAIN_TO_W'(1);
      // Example heuristic: flag if > ~16M cycles (24b all ones). Adjust to taste.
      if (&drain_to_ctr_q) err_drain_to_d = 1'b1;

      if (drain_done) begin
        // Count completed tile
        tiles_d      = tiles_q + 32'd1;
        tiles_left_d = tiles_left_q - TILE_W'(1);

        if (tiles_left_q > TILE_W'(1)) begin
          // More tiles to run
          tile_idx_d = tile_idx_q + TILE_W'(1);

          // If we overlapped, ensure both feeders are ready before RUN
          // Otherwise we might need a short PREFILL if they are not done yet.
          if (cfg_overlap_prefetch_i) begin
            if (a_done_d && w_done_d) begin
              // direct RUN
              k_ctr_d     = '0;
              gap_d       = 16'd0;
              clear_acc_d = 1'b1;
              st_d        = S_RUN;
            end else begin
              st_d        = S_PREFILL;
            end
          end else begin
            // Always PREFILL between tiles
            st_d = S_PREFILL;
          end
        end else begin
          // All tiles done -> IDLE
          st_d = S_IDLE;
        end
      end
    end

    // ---------------- PAUSE ----------------
    else if (st_q == S_PAUSE) begin
      // Nothing; wait for resume or abort
    end
  end

  // ==========================================================================
  // Outputs (registered)
  //   - clear_acc_o is a pulse (from clear_acc_q)
  //   - step_{valid,first,last}_o driven combinationally from state/k_ctr/gap
  // ==========================================================================
  assign active_o  = (st_q != S_IDLE);
  assign paused_o  = paused_q;
  assign k_ctr_o   = k_ctr_q;
  assign tile_idx_o= tile_idx_q;
  assign steps_issued_o = steps_q;
  assign stall_cycles_o = stall_q;
  assign drain_cycles_o = drain_q;
  assign tiles_done_o   = tiles_q;
  assign err_zero_k_sticky_o  = err_zero_k_q;
  assign err_c_last_timeout_o = err_drain_to_q;

  // Step strobes
  // We only propose a step when:
  //   - In RUN
  //   - Not paused
  //   - Throttle gap has expired
  // step_first/last qualify the VALID for that cycle.
  wire step_can_propose = (st_q == S_RUN) && !paused_q && (gap_q == 16'd0);

  assign step_valid_o = step_can_propose;
  assign step_first_o = step_can_propose && (k_ctr_q == '0);
  assign step_last_o  = step_can_propose && (k_ctr_q == (cfg_k_len_i - K_W'(1)));
  assign clear_acc_o  = clear_acc_q;

  // ==========================================================================
  // Sequential
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      st_q           <= S_IDLE;
      tile_idx_q     <= '0;
      tiles_left_q   <= '0;

      k_ctr_q        <= '0;
      gap_q          <= 16'd0;

      paused_q       <= 1'b0;

      a_done_q       <= 1'b0;
      w_done_q       <= 1'b0;
      kick_a_q       <= 1'b0;
      kick_w_q       <= 1'b0;

      clear_acc_q    <= 1'b0;

      steps_q        <= 32'd0;
      stall_q        <= 32'd0;
      drain_q        <= 32'd0;
      tiles_q        <= 32'd0;

      err_zero_k_q   <= 1'b0;
      err_drain_to_q <= 1'b0;
      drain_to_ctr_q <= '0;
    end else begin
      st_q           <= st_d;
      tile_idx_q     <= tile_idx_d;
      tiles_left_q   <= tiles_left_d;

      k_ctr_q        <= k_ctr_d;
      gap_q          <= gap_d;

      paused_q       <= paused_d;

      a_done_q       <= a_done_d;
      w_done_q       <= w_done_d;
      kick_a_q       <= kick_a_d;
      kick_w_q       <= kick_w_d;

      clear_acc_q    <= clear_acc_d;

      steps_q        <= steps_d;
      stall_q        <= stall_d;
      drain_q        <= drain_d;
      tiles_q        <= tiles_d;

      err_zero_k_q   <= err_zero_k_d;
      err_drain_to_q <= err_drain_to_d;
      drain_to_ctr_q <= drain_to_ctr_d;
    end
  end

  // Export kick pulses
  assign kick_a_prefetch_o = kick_a_q;
  assign kick_w_prefetch_o = kick_w_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Do not start with K==0
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    cmd_start_i |-> (cfg_k_len_i != 0))
    else $error("array_ctrl: cmd_start_i with cfg_k_len_i == 0");

  // step_last_o must only occur in RUN
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    step_last_o |-> (st_q == S_RUN))
    else $error("array_ctrl: step_last_o asserted outside RUN.");

  // step_first_o only when k_ctr==0
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    step_first_o |-> (k_ctr_q == 0))
    else $error("array_ctrl: step_first_o without k_ctr==0.");

  // last_step_fire implies next state is DRAIN
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    last_step_fire |-> (st_d == S_DRAIN))
    else $error("array_ctrl: accepted last step but not transitioning to DRAIN.");

  // drain_done only in DRAIN
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    c_tile_last_i |-> (st_q == S_DRAIN))
    else $warning("array_ctrl: c_tile_last_i outside DRAIN phase.");

`endif

endmodule

`default_nettype wire
