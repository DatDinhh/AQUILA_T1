// ============================================================================
//  clk_rst_gen.sv
//  Aquila TB - Multi-domain Clock & Reset Generator (SYS/AXI/ARR/SNS)
// ----------------------------------------------------------------------------
//  Features
//   - 4 clocks: SYS, AXI, ARR, SNS (sensor/aux) with independent frequencies
//   - Async reset assert, sync deassert aligned to each clock domain
//   - Ordered reset release: TRSTn → SNS → SYS (configurable)
//   - Runtime control tasks: set frequency (MHz), set jitter, pause/resume,
//     inject skipped edges (glitch), phase align ARR to AXI, global/local resets
//   - Plusargs override defaults at runtime (see below)
//   - Optional waveform announcements + minimal health assertions
//
//  Plusargs (all optional)
//   +CLK_SYS_MHZ=<real>, +CLK_AXI_MHZ=<real>, +CLK_ARR_MHZ=<real>, +CLK_SNS_MHZ=<real>
//   +J_SYS_PS=<int>, +J_AXI_PS=<int>, +J_ARR_PS=<int>, +J_SNS_PS=<int>         (peak-to-peak/2)
//   +PHASE_ARR2AXI_NS=<real>  (initial ARR start delay relative to AXI start)
//   +PHASE_SYS_NS=<real>, +PHASE_AXI_NS=<real>, +PHASE_SNS_NS=<real>           (absolute offsets)
//   +RST_HOLD_SYS=<int>, +RST_HOLD_SNS=<int>, +RST_HOLD_TRST=<int>             (cycles)
//   +WAVES=1 to print configuration banner
//
//  Notes
//   - This is a *testbench-only* utility (uses real time delays, random jitter).
//   - Jitter is uniform in ±J_<dom>_PS converted to ns per edge. Set 0 for ideal.
//   - Changing a clock frequency takes effect on the *next* edge.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module clk_rst_gen #(
  // Default frequencies (MHz)
  real SYS_MHZ = 100.0,
  real AXI_MHZ = 200.0,
  real ARR_MHZ = 200.0,
  real SNS_MHZ = 25.0,

  // Default reset hold (cycles counted in the corresponding domain)
  int unsigned RST_HOLD_SYS_CYC = 8,
  int unsigned RST_HOLD_SNS_CYC = 8,
  int unsigned RST_HOLD_TRST_CYC= 8
)(
  output logic clk_sys_o,
  output logic clk_axi_o,
  output logic clk_arr_o,
  output logic sns_clk_o,

  output logic rstn_o,        // active-low system reset (sync-deassert to SYS)
  output logic sns_rstn_o,    // active-low sensor reset (sync-deassert to SNS)
  output logic trst_n_o       // active-low JTAG reset (sync-deassert to SYS)
);

  // Internal configuration (overridable via +args)
  // Periods (ns) and jitter (ns) per domain
  real T_SYS_NS, T_AXI_NS, T_ARR_NS, T_SNS_NS;
  real J_SYS_NS, J_AXI_NS, J_ARR_NS, J_SNS_NS;

  // Initial phase offsets (ns)
  real PH_SYS_NS   = 0.0;
  real PH_AXI_NS   = 0.0;
  real PH_SNS_NS   = 0.0;
  real PH_ARR2AXI  = 0.0; // ARR relative to AXI start

  // Reset holds (cycles)
  int unsigned HOLD_SYS_CYC = RST_HOLD_SYS_CYC;
  int unsigned HOLD_SNS_CYC = RST_HOLD_SNS_CYC;
  int unsigned HOLD_TRST_CYC= RST_HOLD_TRST_CYC;

  bit WAVES = 1'b0;

  // Plusarg parsing
  initial begin
    void'($value$plusargs("WAVES=%0d", WAVES));

    // Frequencies override
    real v;
    if ($value$plusargs("CLK_SYS_MHZ=%f", v)) SYS_MHZ = v;
    if ($value$plusargs("CLK_AXI_MHZ=%f", v)) AXI_MHZ = v;
    if ($value$plusargs("CLK_ARR_MHZ=%f", v)) ARR_MHZ = v;
    if ($value$plusargs("CLK_SNS_MHZ=%f", v)) SNS_MHZ = v;

    // Periods from MHz (guard against divide-by-zero)
    T_SYS_NS = (SYS_MHZ > 0.0) ? (1000.0 / SYS_MHZ) : 10.0;
    T_AXI_NS = (AXI_MHZ > 0.0) ? (1000.0 / AXI_MHZ) : 5.0;
    T_ARR_NS = (ARR_MHZ > 0.0) ? (1000.0 / ARR_MHZ) : 5.0;
    T_SNS_NS = (SNS_MHZ > 0.0) ? (1000.0 / SNS_MHZ) : 40.0;

    // Jitter (peak) in ps → ns
    int jp;
    if ($value$plusargs("J_SYS_PS=%d", jp)) J_SYS_NS = jp * 1e-3; else J_SYS_NS = 0.0;
    if ($value$plusargs("J_AXI_PS=%d", jp)) J_AXI_NS = jp * 1e-3; else J_AXI_NS = 0.0;
    if ($value$plusargs("J_ARR_PS=%d", jp)) J_ARR_NS = jp * 1e-3; else J_ARR_NS = 0.0;
    if ($value$plusargs("J_SNS_PS=%d", jp)) J_SNS_NS = jp * 1e-3; else J_SNS_NS = 0.0;

    // Phase offsets
    real ph;
    if ($value$plusargs("PHASE_SYS_NS=%f", ph)) PH_SYS_NS = ph;
    if ($value$plusargs("PHASE_AXI_NS=%f", ph)) PH_AXI_NS = ph;
    if ($value$plusargs("PHASE_SNS_NS=%f", ph)) PH_SNS_NS = ph;
    if ($value$plusargs("PHASE_ARR2AXI_NS=%f", ph)) PH_ARR2AXI = ph;

    // Reset holds
    int h;
    if ($value$plusargs("RST_HOLD_SYS=%d", h))  HOLD_SYS_CYC  = (h<1)?1:h;
    if ($value$plusargs("RST_HOLD_SNS=%d", h))  HOLD_SNS_CYC  = (h<1)?1:h;
    if ($value$plusargs("RST_HOLD_TRST=%d", h)) HOLD_TRST_CYC = (h<1)?1:h;

    if (WAVES) begin
      $display("[%0t] clk_rst_gen cfg: SYS=%0.3fMHz (T=%0.3fns J=%0.3fns) AXI=%0.3fMHz (T=%0.3fns J=%0.3fns) ARR=%0.3fMHz (T=%0.3fns J=%0.3fns) SNS=%0.3fMHz (T=%0.3fns J=%0.3fns)",
               $time, SYS_MHZ, T_SYS_NS, J_SYS_NS, AXI_MHZ, T_AXI_NS, J_AXI_NS, ARR_MHZ, T_ARR_NS, J_ARR_NS, SNS_MHZ, T_SNS_NS, J_SNS_NS);
      $display("[%0t] phases: SYS=%0.3fns AXI=%0.3fns SNS=%0.3fns ARR2AXI=%0.3fns; reset holds (cyc): TRST=%0d SNS=%0d SYS=%0d",
               $time, PH_SYS_NS, PH_AXI_NS, PH_SNS_NS, PH_ARR2AXI, HOLD_TRST_CYC, HOLD_SNS_CYC, HOLD_SYS_CYC);
    end
  end

  // Clock enable gates (pause/resume)

  bit en_sys = 1'b1, en_axi = 1'b1, en_arr = 1'b1, en_sns = 1'b1;

  // Random jitter helper (uniform in [-amp, +amp])
  function automatic real jitter_ns(input real amp);
    if (amp <= 0.0) return 0.0;
    int s = $urandom_range(-1000, 1000);
    return (s / 1000.0) * amp;
  endfunction

  // Clock generators (each starts after its own phase; ARR waits for AXI + offset)

  initial begin : gen_sys
    clk_sys_o = 1'b0;
    #(PH_SYS_NS);
    forever begin
      if (!en_sys) @(posedge en_sys);
      #(0.5*T_SYS_NS + jitter_ns(J_SYS_NS));
      clk_sys_o = ~clk_sys_o;
    end
  end

  initial begin : gen_axi
    clk_axi_o = 1'b0;
    #(PH_AXI_NS);
    forever begin
      if (!en_axi) @(posedge en_axi);
      #(0.5*T_AXI_NS + jitter_ns(J_AXI_NS));
      clk_axi_o = ~clk_axi_o;
    end
  end

  initial begin : gen_arr
    clk_arr_o = 1'b0;
    // Start ARR relative to AXI (phase to model multi-PLL skew)
    wait (clk_axi_o === 1'b0); // ensure known state
    #(PH_ARR2AXI);
    forever begin
      if (!en_arr) @(posedge en_arr);
      #(0.5*T_ARR_NS + jitter_ns(J_ARR_NS));
      clk_arr_o = ~clk_arr_o;
    end
  end

  initial begin : gen_sns
    sns_clk_o = 1'b0;
    #(PH_SNS_NS);
    forever begin
      if (!en_sns) @(posedge en_sns);
      #(0.5*T_SNS_NS + jitter_ns(J_SNS_NS));
      sns_clk_o = ~sns_clk_o;
    end
  end

  // Reset generators (async assert, synchronous release)
  //  Order: assert all low immediately; release TRSTn → SNS → SYS

  initial begin : gen_reset
    rstn_o     = 1'b0;
    sns_rstn_o = 1'b0;
    trst_n_o   = 1'b0;

    // TRSTn release aligned to SYS domain (JTAG typically synchronous to TCK in practice;
    // using SYS is OK for TB default; adjust if you export TCK-driven release elsewhere)
    repeat (HOLD_TRST_CYC) @(posedge clk_sys_o);
    trst_n_o <= 1'b1;

    // SNS reset release
    repeat (HOLD_SNS_CYC) @(posedge sns_clk_o);
    sns_rstn_o <= 1'b1;

    // SYS reset release
    repeat (HOLD_SYS_CYC) @(posedge clk_sys_o);
    rstn_o <= 1'b1;
  end

  // Health assertions (lightweight)

`ifdef ASSERT_ON
  // Clocks should toggle eventually (simple liveness checks)
  initial begin
    int unsigned tmo;
    tmo = 0; while (clk_sys_o === 1'b0 && tmo < 100000) begin @(posedge clk_sys_o or #1); tmo++; end
    assert (tmo < 100000) else $error("clk_rst_gen: SYS clock not toggling.");
  end
`endif

  // RUNTIME CONTROL TASKS (call from TB: <inst>.task(...))

  // Frequency changes
  task automatic set_clk_freq_mhz(input string dom, input real mhz);
    if (mhz <= 0.0) begin
      $error("[%0t] clk_rst_gen.set_clk_freq_mhz('%s'): invalid freq %0.3f", $time, dom, mhz);
      return;
    end
    if (dom == "sys") begin
      SYS_MHZ = mhz; T_SYS_NS = 1000.0 / mhz;
      $display("[%0t] SYS freq -> %0.3f MHz (T=%0.3f ns)", $time, mhz, T_SYS_NS);
    end else if (dom == "axi") begin
      AXI_MHZ = mhz; T_AXI_NS = 1000.0 / mhz;
      $display("[%0t] AXI freq -> %0.3f MHz (T=%0.3f ns)", $time, mhz, T_AXI_NS);
    end else if (dom == "arr") begin
      ARR_MHZ = mhz; T_ARR_NS = 1000.0 / mhz;
      $display("[%0t] ARR freq -> %0.3f MHz (T=%0.3f ns)", $time, mhz, T_ARR_NS);
    end else if (dom == "sns") begin
      SNS_MHZ = mhz; T_SNS_NS = 1000.0 / mhz;
      $display("[%0t] SNS freq -> %0.3f MHz (T=%0.3f ns)", $time, mhz, T_SNS_NS);
    end else begin
      $error("[%0t] clk_rst_gen: unknown domain '%s' in set_clk_freq_mhz()", $time, dom);
    end
  endtask

  // Jitter control (peak; uniform)
  task automatic set_clk_jitter_ps(input string dom, input int unsigned p2p_half_ps);
    real ns = p2p_half_ps * 1e-3;
    if (dom == "sys") J_SYS_NS = ns;
    else if (dom == "axi") J_AXI_NS = ns;
    else if (dom == "arr") J_ARR_NS = ns;
    else if (dom == "sns") J_SNS_NS = ns;
    else $error("[%0t] clk_rst_gen: unknown domain '%s' in set_clk_jitter_ps()", $time, dom);
    $display("[%0t] %s jitter -> ±%0.3f ns", $time, dom, ns);
  endtask

  // Pause/resume clocks 
  task automatic pause_clk(input string dom);
    if (dom == "sys") en_sys = 1'b0;
    else if (dom == "axi") en_axi = 1'b0;
    else if (dom == "arr") en_arr = 1'b0;
    else if (dom == "sns") en_sns = 1'b0;
    else $error("[%0t] clk_rst_gen: unknown domain '%s' in pause_clk()", $time, dom);
    $display("[%0t] %s clock paused.", $time, dom);
  endtask

  task automatic resume_clk(input string dom);
    if (dom == "sys") en_sys = 1'b1;
    else if (dom == "axi") en_axi = 1'b1;
    else if (dom == "arr") en_arr = 1'b1;
    else if (dom == "sns") en_sns = 1'b1;
    else $error("[%0t] clk_rst_gen: unknown domain '%s' in resume_clk()", $time, dom);
    $display("[%0t] %s clock resumed.", $time, dom);
  endtask

  // Skip N rising edges (glitch-injection / gating emulation)
  task automatic skip_edges(input string dom, input int unsigned n_edges);
    if (n_edges == 0) return;
    if (dom == "sys") begin
      for (int i=0;i<n_edges;i++) @(posedge clk_sys_o);
    end else if (dom == "axi") begin
      for (int i=0;i<n_edges;i++) @(posedge clk_axi_o);
    end else if (dom == "arr") begin
      for (int i=0;i<n_edges;i++) @(posedge clk_arr_o);
    end else if (dom == "sns") begin
      for (int i=0;i<n_edges;i++) @(posedge sns_clk_o);
    end else $error("[%0t] clk_rst_gen: unknown domain '%s' in skip_edges()", $time, dom);
    $display("[%0t] %s skipped %0d edges (no action on clock waveform — for sequencing only).", $time, dom, n_edges);
  endtask

  // Phase-align ARR to next AXI rising edge
  task automatic align_arr_to_axi();
    bit prev_en = en_arr;
    en_arr = 1'b0;    // hold ARR
    @(posedge clk_axi_o);
    // Re-enable ARR, the generator will resume from its next delay (no hard phase lock, but aligned start)
    en_arr = prev_en ? 1'b1 : 1'b0;
    $display("[%0t] ARR re-aligned to AXI next edge.", $time);
  endtask

  // Global reset sequence: assert all, release TRSTn -> SNS -> SYS
  task automatic global_reset(
    input int unsigned trst_hold_cyc = 0,
    input int unsigned sns_hold_cyc  = 0,
    input int unsigned sys_hold_cyc  = 0
  );
    int unsigned t_h = (trst_hold_cyc==0) ? HOLD_TRST_CYC : trst_hold_cyc;
    int unsigned s_h = (sns_hold_cyc ==0) ? HOLD_SNS_CYC  : sns_hold_cyc;
    int unsigned y_h = (sys_hold_cyc ==0) ? HOLD_SYS_CYC  : sys_hold_cyc;

    // Assert asynchronously
    rstn_o     <= 1'b0;
    sns_rstn_o <= 1'b0;
    trst_n_o   <= 1'b0;

    // Release in order with sync
    repeat (t_h) @(posedge clk_sys_o); trst_n_o   <= 1'b1;
    repeat (s_h) @(posedge sns_clk_o); sns_rstn_o <= 1'b1;
    repeat (y_h) @(posedge clk_sys_o); rstn_o     <= 1'b1;

    $display("[%0t] global_reset: TRSTn↑ after %0d, SNS↑ after %0d, SYS↑ after %0d cycles.", $time, t_h, s_h, y_h);
  endtask

  // Targeted reset: pulse only a specific domain (active-low)
  task automatic pulse_reset_sys(input int unsigned sys_cycles_low = 4);
    rstn_o <= 1'b0;
    repeat (sys_cycles_low) @(posedge clk_sys_o);
    rstn_o <= 1'b1;
    $display("[%0t] pulse_reset_sys: %0d SYS cycles low.", $time, sys_cycles_low);
  endtask

  task automatic pulse_reset_sns(input int unsigned sns_cycles_low = 4);
    sns_rstn_o <= 1'b0;
    repeat (sns_cycles_low) @(posedge sns_clk_o);
    sns_rstn_o <= 1'b1;
    $display("[%0t] pulse_reset_sns: %0d SNS cycles low.", $time, sns_cycles_low);
  endtask

  task automatic pulse_trst(input int unsigned sys_cycles_low = 2);
    trst_n_o <= 1'b0;
    repeat (sys_cycles_low) @(posedge clk_sys_o);
    trst_n_o <= 1'b1;
    $display("[%0t] pulse_trst: %0d SYS cycles low.", $time, sys_cycles_low);
  endtask

endmodule

`default_nettype wire
