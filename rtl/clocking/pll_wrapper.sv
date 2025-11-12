// ============================================================================
//  Aquila-T1 PLL Wrapper
//  File: pll_wrapper.sv
//  Description:
//    - Uniform digital wrapper around a vendor/foundry analog PLL macro.
//    - Exposes 3 clock outputs (ARR/SRAM/CTRL) + a filtered LOCKED.
//    - In early RTL / when no PLL macro is present, behaves as:
//        * clk_arr/clk_sram/clk_ctrl = gated pass-through of clk_ref
//        * locked asserted after N stable ref cycles post-reset
//    - Glitch-free clock enable at the wrapper boundary (ICG style), so
//      downstream logic never sees unstable edges before lock.
//
//  Ports (unchanged from tile_top.sv):
//    input  clk_ref   : reference clock into PLL
//    input  rstn      : asynchronous reset (active-low) for the PLL wrapper
//    output clk_arr   : array domain base clock (to clock_islands)
//    output clk_sram  : SRAM domain base clock (to clock_islands)
//    output clk_ctrl  : control domain base clock (to clock_islands)
//    output locked    : filtered PLL lock indication
//
//  Notes:
//    * Under USE_TECH_PLL, wire your vendor macro in place of tech_pll u_pll.
//    * This wrapper does NOT implement runtime div/mux; that is handled in
//      clock_islands.sv to keep DVFS logic in one place.
//    * The ICG cells here only hold clocks low until lock is asserted. Once
//      released, further gating/division is managed by clock_islands.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pll_wrapper #(
  // Consecutive ref cycles required to consider PLL "stably locked"
  parameter int unsigned LOCK_FILTER_CYCLES   = 16,
  // Consecutive ref cycles of unlock before we drop locked low
  parameter int unsigned UNLOCK_FILTER_CYCLES = 2,
  // Hold output clocks low until LOCKED=1 (recommended)
  parameter bit          GATE_OUTPUTS_UNTIL_LOCK = 1'b1
)(
  input  logic clk_ref,
  input  logic rstn,        // async, active-low

  output logic clk_arr,
  output logic clk_sram,
  output logic clk_ctrl,
  output logic locked
);

  // --------------------------------------------------------------------------
  // Raw clocks and raw lock (from tech PLL or fallback)
  // --------------------------------------------------------------------------
  logic clk_arr_raw, clk_sram_raw, clk_ctrl_raw;
  logic locked_raw;

`ifdef USE_TECH_PLL
  // ==========================================================================
  // TECH MACRO INSTANTIATION
  // Replace 'tech_pll' with your vendor PLL macro.
  // Typical vendor ports (example – adjust names to your IP):
  //   .ref_clk, .rst_n, .out_clk[0..2], .locked
  // ==========================================================================
  tech_pll u_pll (
    .ref_clk  (clk_ref),
    .rst_n    (rstn),
    .out_clk0 (clk_arr_raw),
    .out_clk1 (clk_sram_raw),
    .out_clk2 (clk_ctrl_raw),
    .locked   (locked_raw)
  );
`else
  // ==========================================================================
  // BEHAVIORAL / FALLBACK (no real PLL)
  // - Pass-through reference clock to all three outputs (raw)
  // - Create a synthetic 'locked_raw' after reset for simulation/bring-up
  // ==========================================================================
  assign clk_arr_raw  = clk_ref;
  assign clk_sram_raw = clk_ref;
  assign clk_ctrl_raw = clk_ref;

  // Simple lock counter on ref clock: after LOCK_FILTER_CYCLES from reset release
  logic [31:0] lock_cnt_q;
  always_ff @(posedge clk_ref or negedge rstn) begin
    if (!rstn) begin
      lock_cnt_q <= '0;
    end else if (lock_cnt_q != 32'hFFFF_FFFF) begin
      lock_cnt_q <= lock_cnt_q + 32'd1;
    end
  end
  assign locked_raw = (lock_cnt_q >= LOCK_FILTER_CYCLES);
`endif

  // --------------------------------------------------------------------------
  // LOCK FILTERING (debounce to avoid chatter)
  // We filter the vendor 'locked_raw' to assert only after N consecutive
  // high cycles, and to deassert only after M consecutive low cycles.
  // --------------------------------------------------------------------------
  logic [31:0] lock_hi_cnt_q, lock_lo_cnt_q;
  logic        locked_q, locked_d;

  always_ff @(posedge clk_ref or negedge rstn) begin
    if (!rstn) begin
      lock_hi_cnt_q <= '0;
      lock_lo_cnt_q <= '0;
      locked_q      <= 1'b0;
    end else begin
      lock_hi_cnt_q <= (locked_raw ? (lock_hi_cnt_q + 32'd1) : '0);
      lock_lo_cnt_q <= (!locked_raw ? (lock_lo_cnt_q + 32'd1) : '0);

      locked_q      <= locked_d;
    end
  end

  always_comb begin
    locked_d = locked_q;
    // Assert when consecutive highs exceed threshold
    if (!locked_q && (lock_hi_cnt_q >= LOCK_FILTER_CYCLES[31:0])) begin
      locked_d = 1'b1;
    end
    // Deassert when consecutive lows exceed threshold
    if (locked_q && (lock_lo_cnt_q >= UNLOCK_FILTER_CYCLES[31:0])) begin
      locked_d = 1'b0;
    end
  end

  assign locked = locked_q;

  // --------------------------------------------------------------------------
  // OUTPUT GATING (glitch-free ICG) — hold clocks low until LOCKED=1
  // This uses a transparent-low latch + AND, which maps to vendor ICG cells.
  // --------------------------------------------------------------------------
  function automatic logic icg_en_effective(input logic en);
    if (GATE_OUTPUTS_UNTIL_LOCK) icg_en_effective = en;
    else                         icg_en_effective = 1'b1; // always pass-through
  endfunction

  logic gclk_arr, gclk_sram, gclk_ctrl;

  pll_icg u_icg_arr  ( .clk_i(clk_arr_raw),  .en_i(icg_en_effective(locked_q)), .gclk_o(gclk_arr)  );
  pll_icg u_icg_sram ( .clk_i(clk_sram_raw), .en_i(icg_en_effective(locked_q)), .gclk_o(gclk_sram) );
  pll_icg u_icg_ctrl ( .clk_i(clk_ctrl_raw), .en_i(icg_en_effective(locked_q)), .gclk_o(gclk_ctrl) );

  assign clk_arr  = gclk_arr;
  assign clk_sram = gclk_sram;
  assign clk_ctrl = gclk_ctrl;

`ifdef ASSERT_ON
  // Once locked=0, all outputs must be held low within a bounded number of ref cycles.
  // (The ICG latch updates on low phase; we can't bound to exactly 1 cycle without timing.)
`endif

endmodule


// ============================================================================
//  Glitch-free ICG used inside the PLL wrapper
//  Transparent-low latch captures EN, output is CLK & EN_LAT.
//  No test/scan bypass here — wrapper ICG is only for lock gating.
//  In PD, map this to your library ICG cell.
// ============================================================================
module pll_icg (
  input  logic clk_i,
  input  logic en_i,   // 1 = pass clock, 0 = hold low
  output logic gclk_o
);
  // Transparent-low latch on enable
  (* syn_keep = 1, keep = "true" *)
  logic en_lat;
  always_latch begin
    if (~clk_i) en_lat <= en_i;
  end

  assign gclk_o = clk_i & en_lat;
endmodule

`default_nettype wire
