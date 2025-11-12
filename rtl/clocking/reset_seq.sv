// ============================================================================
//  Aquila-T1 Reset & Bring-up Sequencer
//  File: reset_seq.sv
//  Description:
//    Deterministic bring-up FSM for a single accelerator tile.
//
//    Sequence:
//      IDLE → WAIT_PLL(lock filtering) → RELEASE_SRAM
//      → MBIST(start + wait) → RELEASE_UC → WAIT_UC(ready/heartbeat)
//      → (optional) DMA_SANITY(start + wait) → RELEASE_ARRAY → RUN
//
//    Features:
//      - Parameterized timeouts & post-reset settle delays
//      - PLL lock stability filter (N consecutive cycles)
//      - One-cycle start pulses: MBIST, DMA sanity
//      - Error codes & sticky error latch; SW clear input
//      - Done pulse on transition to RUN; IRQ on done/error
//      - Thermal alert gating of array enable (optional)
//
//    Notes:
//      - All logic is synchronous to clk_i, active-low rstn_i.
//      - Reset outputs (active-low) are driven here and should be
//        synchronized before entering other clock domains.
//      - Connect status outputs to CSRs for visibility.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module reset_seq #(
  // ------------------------------
  // PLL / Timeouts / Delays
  // ------------------------------
  parameter int unsigned PLL_LOCK_STABLE_CYCLES   = 8,           // consecutive locks
  parameter int unsigned PLL_LOCK_TIMEOUT_CYCLES  = 2_000_000,   // safety net

  parameter int unsigned POST_SRAM_RESET_CYCLES   = 4,           // settle after SRAM reset deassert
  parameter int unsigned POST_UC_RESET_CYCLES     = 4,           // settle after µC reset deassert
  parameter int unsigned POST_ARRAY_RESET_CYCLES  = 8,           // settle after array reset deassert

  parameter int unsigned MBIST_TIMEOUT_CYCLES     = 5_000_000,
  parameter int unsigned UC_BOOT_TIMEOUT_CYCLES   = 50_000_000,  // allow ROM init etc.
  parameter int unsigned DMA_SANITY_TIMEOUT_CYCLES= 5_000_000,

  // ------------------------------
  // Options
  // ------------------------------
  parameter bit          ENABLE_DMA_SANITY        = 1'b1,        // include DMA sanity step
  parameter bit          GATE_ARRAY_ON_THERMAL    = 1'b1,        // gate array_enable on therm_alert
  parameter int unsigned MIN_HEARTBEAT_TOGGLES    = 1            // µC heartbeat toggles required
)(
  // Clock / Reset (control domain)
  input  logic clk_i,
  input  logic rstn_i,           // active-low, synchronous to clk_i

  // Controls
  input  logic start_i,          // start bring-up sequence (can be tied high)
  input  logic test_bypass_i,    // bypass checks, go RUN quickly (DFT bring-up)
  input  logic sw_clear_error_i, // clear sticky error and return to IDLE
  input  logic sw_force_error_i, // force error for testing paths

  // Status inputs
  input  logic pll_locked_i,     // from PLL wrapper
  input  logic mbist_all_done_i, // all banks complete
  input  logic mbist_any_fail_i, // OR of fail flags
  input  logic uc_ready_i,       // µC boot completes (e.g., set by FW/CSR)
  input  logic uc_heartbeat_i,   // optional free-running heartbeat (FW)
  input  logic dma_sanity_done_i,// DMA loopback/sanity completes
  input  logic dma_sanity_pass_i,// DMA sanity passed
  input  logic therm_alert_i,    // thermal alert (throttle/gate array)

  // Outputs: active-low resets (to be synchronized per destination domain)
  output logic sram_resetn_o,
  output logic uc_resetn_o,
  output logic array_resetn_o,

  // One-shot kick signals
  output logic sram_mbist_start_o,  // pulse 1 cycle at MBIST start
  output logic dma_sanity_start_o,  // pulse 1 cycle at DMA sanity start

  // Enables / status
  output logic array_enable_o,   // asserted in RUN (gated by thermal if enabled)
  output logic busy_o,           // 1 when FSM is doing bring-up
  output logic done_o,           // one-cycle pulse when entering RUN
  output logic error_o,          // sticky error status
  output logic [3:0] error_code_o,
  output logic [7:0] state_o,    // current state encoding (for CSR/debug)
  output logic irq_o             // one-cycle pulse on done or error
);

  // --------------------------------------------------------------------------
  // State encoding
  // --------------------------------------------------------------------------
  typedef enum logic [7:0] {
    S_IDLE          = 8'h00,
    S_WAIT_PLL      = 8'h10,
    S_RELEASE_SRAM  = 8'h20,
    S_MBIST         = 8'h21,
    S_RELEASE_UC    = 8'h30,
    S_WAIT_UC       = 8'h31,
    S_DMA_SANITY    = 8'h40,
    S_RELEASE_ARRAY = 8'h50,
    S_RUN           = 8'h60,
    S_ERROR         = 8'hF0
  } state_e;

  state_e state_q, state_d;
  assign state_o = state_q;

  // --------------------------------------------------------------------------
  // Error codes
  // --------------------------------------------------------------------------
  localparam logic [3:0]
    EC_NONE        = 4'h0,
    EC_PLL_TIMEOUT = 4'h1,
    EC_MBIST_TO    = 4'h2,
    EC_MBIST_FAIL  = 4'h3,
    EC_UC_TIMEOUT  = 4'h4,
    EC_DMA_TO      = 4'h5,
    EC_DMA_FAIL    = 4'h6,
    EC_SW_FORCED   = 4'h7;

  // Sticky error latch
  logic        err_sticky_q, err_sticky_d;
  logic [3:0]  err_code_q,   err_code_d;

  // --------------------------------------------------------------------------
  // Internal timers / counters
  // --------------------------------------------------------------------------
  logic [31:0] timer_q, timer_d;       // generic per-state timer
  logic        timer_en;               // counts when high
  logic        timer_clr;              // clears to 0

  // PLL stable filter
  logic [15:0] pll_cnt_q, pll_cnt_d;
  logic        pll_stable;

  // Heartbeat toggle detector
  logic uc_hb_q;
  logic [7:0] hb_toggles_q, hb_toggles_d;

  // One-cycle pulses
  logic done_pulse, err_pulse;

  // Outputs (regs)
  logic sram_resetn_q,  sram_resetn_d;
  logic uc_resetn_q,    uc_resetn_d;
  logic array_resetn_q, array_resetn_d;

  logic sram_mbist_start_q,  sram_mbist_start_d;
  logic dma_sanity_start_q,  dma_sanity_start_d;

  logic array_enable_q, array_enable_d;

  // --------------------------------------------------------------------------
  // Sequential
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q             <= S_IDLE;

      timer_q             <= '0;
      pll_cnt_q           <= '0;
      uc_hb_q             <= 1'b0;
      hb_toggles_q        <= '0;

      err_sticky_q        <= 1'b0;
      err_code_q          <= EC_NONE;

      sram_resetn_q       <= 1'b0;
      uc_resetn_q         <= 1'b0;
      array_resetn_q      <= 1'b0;

      sram_mbist_start_q  <= 1'b0;
      dma_sanity_start_q  <= 1'b0;

      array_enable_q      <= 1'b0;
    end
    else begin
      state_q             <= state_d;

      timer_q             <= (timer_clr ? '0 : (timer_en ? (timer_q + 32'd1) : timer_q));
      pll_cnt_q           <= pll_cnt_d;

      uc_hb_q             <= uc_heartbeat_i;
      hb_toggles_q        <= hb_toggles_d;

      err_sticky_q        <= err_sticky_d;
      err_code_q          <= err_code_d;

      sram_resetn_q       <= sram_resetn_d;
      uc_resetn_q         <= uc_resetn_d;
      array_resetn_q      <= array_resetn_d;

      sram_mbist_start_q  <= sram_mbist_start_d;
      dma_sanity_start_q  <= dma_sanity_start_d;

      array_enable_q      <= array_enable_d;
    end
  end

  // --------------------------------------------------------------------------
  // Edge detects and pulses
  // --------------------------------------------------------------------------
  // done: one cycle when entering RUN
  assign done_pulse = (state_q != S_RUN)  && (state_d == S_RUN);
  // error: one cycle when entering ERROR
  assign err_pulse  = (state_q != S_ERROR) && (state_d == S_ERROR);

  // IRQ pulse on done or error
  assign irq_o = done_pulse | err_pulse;

  // Sticky error & code update
  always_comb begin
    err_sticky_d = err_sticky_q;
    err_code_d   = err_code_q;

    if (sw_clear_error_i) begin
      err_sticky_d = 1'b0;
      err_code_d   = EC_NONE;
    end
    // Set by state machine on error transitions
  end

  // Timer control defaults
  always_comb begin
    timer_en  = 1'b0;
    timer_clr = 1'b0;
  end

  // PLL lock stability counter: counts consecutive cycles of pll_locked_i
  always_comb begin
    pll_cnt_d  = pll_cnt_q;
    if (state_q == S_WAIT_PLL) begin
      if (pll_locked_i) begin
        if (pll_cnt_q != 16'hFFFF) pll_cnt_d = pll_cnt_q + 16'd1;
      end else begin
        pll_cnt_d = '0;
      end
    end else begin
      pll_cnt_d = '0;
    end
  end
  assign pll_stable = (pll_cnt_q >= PLL_LOCK_STABLE_CYCLES[15:0]);

  // Heartbeat toggles counter (only in WAIT_UC)
  always_comb begin
    hb_toggles_d = hb_toggles_q;
    if (state_q == S_WAIT_UC) begin
      if (uc_heartbeat_i ^ uc_hb_q) begin
        if (hb_toggles_q != 8'hFF) hb_toggles_d = hb_toggles_q + 8'd1;
      end
    end else begin
      hb_toggles_d = '0;
    end
  end

  // --------------------------------------------------------------------------
  // Outputs defaulting
  // --------------------------------------------------------------------------
  always_comb begin
    // default outputs hold previous
    sram_resetn_d       = sram_resetn_q;
    uc_resetn_d         = uc_resetn_q;
    array_resetn_d      = array_resetn_q;

    sram_mbist_start_d  = 1'b0; // pulses are one-shot on state entry
    dma_sanity_start_d  = 1'b0;

    array_enable_d      = array_enable_q;
  end

  // status/busy/done/error
  assign busy_o        = (state_q != S_IDLE) && (state_q != S_RUN) && (state_q != S_ERROR);
  assign done_o        = done_pulse;
  assign error_o       = err_sticky_q;
  assign error_code_o  = err_code_q;

  // Drive reset outputs to ports
  assign sram_resetn_o  = sram_resetn_q;
  assign uc_resetn_o    = uc_resetn_q;
  assign array_resetn_o = array_resetn_q;

  // Array enable with optional thermal gating
  wire array_enable_gated = (GATE_ARRAY_ON_THERMAL) ? (array_enable_q & ~therm_alert_i) : array_enable_q;
  assign array_enable_o   = array_enable_gated;

  // --------------------------------------------------------------------------
  // Next-state logic
  // --------------------------------------------------------------------------
  always_comb begin
    state_d = state_q;

    // timer default: disabled; most states enable it
    timer_en  = 1'b0;
    timer_clr = 1'b0;

    // Keep resets asserted by default unless set by states
    // (already held in *_q; individual states adjust *_d)

    // Error forcing (software)
    if (sw_force_error_i && state_q != S_ERROR) begin
      state_d     = S_ERROR;
    end
    else begin
      unique case (state_q)
        // ----------------------------------------------------------
        S_IDLE: begin
          // Assert all resets; deassert enables
          sram_resetn_d  = 1'b0;
          uc_resetn_d    = 1'b0;
          array_resetn_d = 1'b0;
          array_enable_d = 1'b0;

          if (sw_clear_error_i) begin
            // stay in IDLE, cleared error
          end else if (test_bypass_i && start_i) begin
            // Quick path for DFT/bring-up
            sram_resetn_d  = 1'b1;
            uc_resetn_d    = 1'b1;
            array_resetn_d = 1'b1;
            array_enable_d = 1'b1;
            state_d        = S_RUN;
          end else if (start_i) begin
            timer_clr = 1'b1;
            state_d   = S_WAIT_PLL;
          end
        end

        // ----------------------------------------------------------
        S_WAIT_PLL: begin
          timer_en = 1'b1; // watchdog timeout
          if (pll_stable) begin
            // Deassert SRAM reset next
            timer_clr      = 1'b1;
            sram_resetn_d  = 1'b1;
            state_d        = S_RELEASE_SRAM;
          end else if (timer_q >= PLL_LOCK_TIMEOUT_CYCLES) begin
            state_d        = S_ERROR;
          end
        end

        // ----------------------------------------------------------
        S_RELEASE_SRAM: begin
          // SRAM reset deasserted; wait a few cycles for I/O to settle
          timer_en = 1'b1;
          if (timer_q >= POST_SRAM_RESET_CYCLES) begin
            timer_clr          = 1'b1;
            sram_mbist_start_d = 1'b1;   // kick MBIST
            state_d            = S_MBIST;
          end
        end

        // ----------------------------------------------------------
        S_MBIST: begin
          timer_en = 1'b1;
          if (mbist_all_done_i) begin
            if (mbist_any_fail_i) begin
              state_d   = S_ERROR;
            end else begin
              // Release microcontroller next
              timer_clr   = 1'b1;
              uc_resetn_d = 1'b1;
              state_d     = S_RELEASE_UC;
            end
          end else if (timer_q >= MBIST_TIMEOUT_CYCLES) begin
            state_d = S_ERROR;
          end
        end

        // ----------------------------------------------------------
        S_RELEASE_UC: begin
          timer_en = 1'b1;
          if (timer_q >= POST_UC_RESET_CYCLES) begin
            timer_clr = 1'b1;
            state_d   = S_WAIT_UC;
          end
        end

        // ----------------------------------------------------------
        S_WAIT_UC: begin
          timer_en = 1'b1;
          // Accept either ready flag or sufficient heartbeat toggles
          if (uc_ready_i || (hb_toggles_q >= MIN_HEARTBEAT_TOGGLES[7:0])) begin
            timer_clr = 1'b1;
            if (ENABLE_DMA_SANITY) begin
              dma_sanity_start_d = 1'b1;
              state_d            = S_DMA_SANITY;
            end else begin
              // Move to array release
              array_resetn_d = 1'b1;
              state_d        = S_RELEASE_ARRAY;
            end
          end else if (timer_q >= UC_BOOT_TIMEOUT_CYCLES) begin
            state_d = S_ERROR;
          end
        end

        // ----------------------------------------------------------
        S_DMA_SANITY: begin
          timer_en = 1'b1;
          if (dma_sanity_done_i) begin
            if (dma_sanity_pass_i) begin
              // Release array and proceed
              timer_clr      = 1'b1;
              array_resetn_d = 1'b1;
              state_d        = S_RELEASE_ARRAY;
            end else begin
              state_d        = S_ERROR;
            end
          end else if (timer_q >= DMA_SANITY_TIMEOUT_CYCLES) begin
            state_d = S_ERROR;
          end
        end

        // ----------------------------------------------------------
        S_RELEASE_ARRAY: begin
          timer_en = 1'b1;
          if (timer_q >= POST_ARRAY_RESET_CYCLES) begin
            timer_clr      = 1'b1;
            array_enable_d = 1'b1;
            state_d        = S_RUN;
          end
        end

        // ----------------------------------------------------------
        S_RUN: begin
          // Normal operation; keep resets deasserted
          sram_resetn_d   = 1'b1;
          uc_resetn_d     = 1'b1;
          array_resetn_d  = 1'b1;

          // Optionally gate array_enable via thermal alert (applied in assign)
          array_enable_d  = 1'b1;

          // Stay here until SW forces error or global reset
        end

        // ----------------------------------------------------------
        S_ERROR: begin
          // Sticky error latch/code handled below
          // Strategy: keep µC out of reset so FW can diagnose; hold array disabled.
          sram_resetn_d   = 1'b1;   // SRAM kept up for diagnostics
          uc_resetn_d     = 1'b1;   // allow FW access to CSRs
          array_resetn_d  = 1'b0;   // ensure array reset when in error
          array_enable_d  = 1'b0;

          // Wait SW clear to return to IDLE
          if (sw_clear_error_i) begin
            // Fully re-run sequence on next start
            state_d = S_IDLE;
          end
        end

        default: state_d = S_ERROR;
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Error latch/code updates on entering ERROR
  // --------------------------------------------------------------------------
  always_comb begin
    // Keep previous unless we detect the cause this cycle
    err_sticky_d = err_sticky_q;
    err_code_d   = err_code_q;

    if (sw_clear_error_i) begin
      err_sticky_d = 1'b0;
      err_code_d   = EC_NONE;
    end
    else if ((state_q != S_ERROR) && (state_d == S_ERROR)) begin
      err_sticky_d = 1'b1;
      // Derive a code based on the source state that failed
      unique case (state_q)
        S_WAIT_PLL     : err_code_d = EC_PLL_TIMEOUT;
        S_MBIST        : err_code_d = (mbist_any_fail_i) ? EC_MBIST_FAIL : EC_MBIST_TO;
        S_WAIT_UC      : err_code_d = EC_UC_TIMEOUT;
        S_DMA_SANITY   : err_code_d = (dma_sanity_done_i && !dma_sanity_pass_i) ? EC_DMA_FAIL : EC_DMA_TO;
        default        : err_code_d = sw_force_error_i ? EC_SW_FORCED : EC_SW_FORCED;
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // One-shot start pulses issued exactly on state entry
  // --------------------------------------------------------------------------
  always_comb begin
    sram_mbist_start_d = 1'b0;
    dma_sanity_start_d = 1'b0;

    if ((state_q != S_RELEASE_SRAM) && (state_d == S_MBIST)) begin
      sram_mbist_start_d = 1'b1; // kick MBIST
    end
    if ((state_q != S_WAIT_UC) && (state_d == S_DMA_SANITY)) begin
      dma_sanity_start_d = 1'b1; // kick DMA loopback
    end
  end

  // --------------------------------------------------------------------------
  // Synthesis-time safety assertions (simulation only)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // MBIST start should be a one-cycle pulse
  assert property (@(posedge clk_i) disable iff (!rstn_i)
    sram_mbist_start_q |-> !sram_mbist_start_d) else
      $error("reset_seq: sram_mbist_start pulse must be one cycle");

  // DMA sanity start should be a one-cycle pulse
  assert property (@(posedge clk_i) disable iff (!rstn_i)
    dma_sanity_start_q |-> !dma_sanity_start_d) else
      $error("reset_seq: dma_sanity_start pulse must be one cycle");

  // In RUN, all resetn must be deasserted
  assert property (@(posedge clk_i) disable iff (!rstn_i)
    (state_q == S_RUN) |-> (sram_resetn_q && uc_resetn_q && array_resetn_q))
    else $error("reset_seq: RUN state requires all resets deasserted");

  // In ERROR, array must be held in reset
  assert property (@(posedge clk_i) disable iff (!rstn_i)
    (state_q == S_ERROR) |-> (!array_resetn_q))
    else $error("reset_seq: ERROR state must assert array reset");
`endif

endmodule

`default_nettype wire
