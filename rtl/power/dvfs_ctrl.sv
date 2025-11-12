// ============================================================================
//  dvfs_ctrl.sv
//  Aquila — Dynamic Voltage and Frequency Scaling Controller
// ----------------------------------------------------------------------------
//  Policy & sequencing:
//   - Computes a policy target P-state from SW request, perf hint, and clamps:
//       * thermal clamp (warn/crit)
//       * droop clamp (lvl 0..3, emergency)
//       * software min/max clamps
//   - If raising performance (p_target > p_curr): raise V, then raise f.
//   - If lowering performance (p_target < p_curr): lower f, then lower V.
//   - Optional "step mode": move 1 OPP at a time to reduce large jumps.
//   - Enforces dwell between transitions; times out if PMIC/PLL do not respond.
//   - Emits events (start/done/timeout/emergency) for bring-up and telemetry.
//
//  OPP table:
//   - Up to N_OPP entries, index 0..N_OPP-1. By convention 0 = lowest perf.
//   - Each OPP has voltage selector (VSEL_W bits) and frequency code (PLL_W bits).
//   - A separate "emergency" P-state index can be used on droop emergency.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module dvfs_ctrl #(
  // ---------------------- OPP geometry --------------------------------------
  parameter int unsigned N_OPP    = 8,
  parameter int unsigned VSEL_W   = 8,     // width of voltage selector code
  parameter int unsigned PLL_W    = 16,    // width of frequency/PLL code
  localparam int unsigned PIDX_W  = (N_OPP <= 1) ? 1 : $clog2(N_OPP),

  // ---------------------- CSR window ----------------------------------------
  parameter int unsigned CSR_AW   = 12,    // 4 KiB window

  // ---------------------- Timing / watchdogs --------------------------------
  parameter int unsigned WDOG_W        = 24,
  parameter logic [WDOG_W-1:0] PMIC_TO = 24'd10_000_000,  // e.g., 100 ms @ 100 MHz
  parameter logic [WDOG_W-1:0] PLL_TO  = 24'd2_000_000,   // e.g., 20 ms @ 100 MHz
  parameter logic [WDOG_W-1:0] DWELL_DFLT = 24'd500_000,  // e.g., 5 ms @ 100 MHz

  // ---------------------- Event source id -----------------------------------
  parameter logic [7:0] EVT_SRC_ID = 8'h47   // 'G' Governor
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,       // synchronous, active-low

  // ====================== Health + performance inputs =======================
  // Thermal clamp (from thermal_sense_if)
  input  logic                         therm_warn_i,
  input  logic                         therm_crit_i,

  // Droop clamp (from voltage_droop_if)
  input  logic [1:0]                   droop_lvl_i,  // 0=none,1=warn,2=hot,3=crit
  input  logic                         droop_emergency_i, // hard emergency

  // Workload/performance hint (0..15, higher = more perf)
  input  logic [3:0]                   perf_hint_i,

  // Software request (optional). If force_override is set in CSR, this is taken as target.
  input  logic                         sw_req_valid_i,
  input  logic [PIDX_W-1:0]            sw_req_pstate_i,

  // Optional hold/freeze (e.g., during scan/test)
  input  logic                         hold_i,

  // ====================== Regulator / PMIC handshake ========================
  output logic                         vreq_valid_o,      // pulse/level request
  output logic [VSEL_W-1:0]            vsel_o,            // target VSEL
  output logic                         vreq_up_o,         // 1=voltage increase, 0=decrease
  input  logic                         vreq_ack_i,        // PMIC accepted request
  input  logic                         vready_i,          // power-good at new setpoint

  // ====================== PLL / clock handshake =============================
  output logic                         freq_req_o,        // pulse/level request
  output logic [PLL_W-1:0]             freq_code_o,       // target PLL/clock code
  input  logic                         pll_ack_i,         // PLL accepted reprogram
  input  logic                         pll_lock_i,        // PLL locked at new setting

  // ====================== Status / control outputs ===========================
  output logic [PIDX_W-1:0]            cur_pstate_o,
  output logic [PIDX_W-1:0]            tgt_pstate_o,      // effective policy target
  output logic                         in_transit_o,      // 1 while sequencing
  output logic                         slow_clk_hint_o,   // 1 when not at max OPP

  // ====================== IRQ / events ======================================
  output logic                         irq_o,             // sticky W1C via CSR
  output logic                         evt_valid_o,
  input  logic                         evt_ready_i,
  output logic [7:0]                   evt_src_o,
  output logic [1:0]                   evt_sev_o,
  output logic [11:0]                  evt_code_o,
  output logic [31:0]                  evt_info_o,

  // ====================== CSR-Lite interface ================================
  input  logic                         csr_valid_i,
  input  logic                         csr_write_i,
  input  logic [CSR_AW-1:0]            csr_addr_i,        // byte address
  input  logic [31:0]                  csr_wdata_i,
  input  logic [3:0]                   csr_wstrb_i,
  output logic                         csr_ready_o,
  output logic [31:0]                  csr_rdata_o
);

  // ==========================================================================
  // OPP tables (programmable via CSR)
  // ==========================================================================
  logic [VSEL_W-1:0] opp_v_q [N_OPP];
  logic [PLL_W-1:0]  opp_f_q [N_OPP];

  // Index helpers
  typedef logic [PIDX_W-1:0] pidx_t;

  // ==========================================================================
  // Configuration registers
  // ==========================================================================
  // CTRL
  logic        cfg_enable_q,        cfg_enable_d;
  logic        cfg_force_override_q,cfg_force_override_d;
  logic        cfg_step_mode_q,     cfg_step_mode_d;      // 1=walk one step per transition
  logic        cfg_irq_en_q,        cfg_irq_en_d;

  // Clamps
  pidx_t       cfg_pmin_q,          cfg_pmin_d;           // min allowed P-state index
  pidx_t       cfg_pmax_q,          cfg_pmax_d;           // max allowed P-state index
  pidx_t       cfg_pcrit_q,         cfg_pcrit_d;          // clamp for thermal crit
  pidx_t       cfg_pwarn_q,         cfg_pwarn_d;          // clamp for thermal warn
  pidx_t       cfg_pdl1_q,          cfg_pdl1_d;           // clamp for droop level 1
  pidx_t       cfg_pdl2_q,          cfg_pdl2_d;           // clamp for droop level 2
  pidx_t       cfg_pdl3_q,          cfg_pdl3_d;           // clamp for droop level 3
  pidx_t       cfg_emerg_q,         cfg_emerg_d;          // emergency P-state

  // Time constants
  logic [WDOG_W-1:0] cfg_dwell_q,   cfg_dwell_d;          // min dwell between transitions
  logic [WDOG_W-1:0] cfg_pmic_to_q, cfg_pmic_to_d;        // PMIC timeout
  logic [WDOG_W-1:0] cfg_pll_to_q,  cfg_pll_to_d;         // PLL timeout

  // Policy mapping (perf hint -> pstate)
  // perf_hint 0..15 mapped to P-state via two cut-points (simple 3-band mapping)
  pidx_t       cfg_p_idle_q,        cfg_p_idle_d;
  pidx_t       cfg_p_typ_q,         cfg_p_typ_d;
  pidx_t       cfg_p_max_q,         cfg_p_max_d;

  // Sticky / status / IRQ
  logic        irq_q, irq_d;
  logic [7:0]  last_err_q, last_err_d;

  // Current/target tracking
  pidx_t       cur_p_q,  cur_p_d;
  pidx_t       req_p_q,  req_p_d;     // raw request (pre-clamp)
  pidx_t       tgt_p_q,  tgt_p_d;     // post-clamp effective target

  // Dwell timer
  logic [WDOG_W-1:0] dwell_q, dwell_d;

  // ==========================================================================
  // Health clamp evaluation and request selection
  // ==========================================================================
  // Map perf hint to baseline pstate (only if SW doesn't force override)
  function automatic pidx_t perf_to_pstate(input logic [3:0] hint);
    // 0..15 → idle / typical / max
    if (hint <= 4'd3)      return cfg_p_idle_q;
    else if (hint <= 4'd10)return cfg_p_typ_q;
    else                   return cfg_p_max_q;
  endfunction

  // Calculate raw request
  always_comb begin
    req_p_d = req_p_q;
    if (cfg_force_override_q && sw_req_valid_i) begin
      req_p_d = sw_req_pstate_i;
    end else begin
      // Software may still push a request without force — treat it as "strong hint"
      pidx_t hint_p = perf_to_pstate(perf_hint_i);
      if (sw_req_valid_i) req_p_d = sw_req_pstate_i;
      else                req_p_d = hint_p;
    end
  end

  // Apply clamps (min/max + thermal + droop)
  function automatic pidx_t clamp_p(input pidx_t p);
    pidx_t r;
    // Start with min/max
    r = (p < cfg_pmin_q) ? cfg_pmin_q : ((p > cfg_pmax_q) ? cfg_pmax_q : p);
    // Thermal
    if (therm_crit_i)      r = (r > cfg_pcrit_q) ? cfg_pcrit_q : r;
    else if (therm_warn_i) r = (r > cfg_pwarn_q) ? cfg_pwarn_q : r;
    // Droop levels (stronger wins)
    if (droop_lvl_i == 2'd3)      r = (r > cfg_pdl3_q) ? cfg_pdl3_q : r;
    else if (droop_lvl_i == 2'd2) r = (r > cfg_pdl2_q) ? cfg_pdl2_q : r;
    else if (droop_lvl_i == 2'd1) r = (r > cfg_pdl1_q) ? cfg_pdl1_q : r;
    return r;
  endfunction

  // Emergency preemption (highest priority)
  wire emerg_w = droop_emergency_i;

  // ----------------------------------------------------------------------------
  // Transition FSM
  // ----------------------------------------------------------------------------
  typedef enum logic [2:0] {
    S_IDLE,         // steady at cur_p
    S_WAIT_DWELL,   // enforce dwell time after last change
    S_V_UP,         // request voltage increase
    S_V_DOWN,       // request voltage decrease
    S_V_WAIT,       // wait vready or timeout
    S_F_UP,         // request frequency increase
    S_F_DOWN,       // request frequency decrease
    S_F_WAIT,       // wait pll_lock or timeout
    S_EMERG         // emergency fallback sequencing
  } state_e;

  state_e st_q, st_d;

  // Watchdogs
  logic [WDOG_W-1:0] wdog_q, wdog_d;

  // Handshake outputs (level while in the corresponding request state)
  assign vreq_valid_o = (st_q == S_V_UP) | (st_q == S_V_DOWN) | (st_q == S_EMERG);
  assign freq_req_o   = (st_q == S_F_UP) | (st_q == S_F_DOWN) | (st_q == S_EMERG);
  assign in_transit_o = (st_q != S_IDLE) && (st_q != S_WAIT_DWELL);

  // Effective target calculation + stepping
  always_comb begin
    // Default: compute tgt from req and clamps
    pidx_t unclamped = req_p_q;
    pidx_t clamped   = clamp_p(unclamped);
    tgt_p_d          = clamped;

    // Step mode: if enabled and distance > 1, move one step per transition
    if (cfg_step_mode_q && (clamped != cur_p_q)) begin
      if (clamped > cur_p_q) tgt_p_d = cur_p_q + pidx_t'(1);
      else                   tgt_p_d = cur_p_q - pidx_t'(1);
    end

    // Emergency has its own target
    if (emerg_w) tgt_p_d = cfg_emerg_q;
  end

  // Which direction is the requested movement relative to current
  wire up_w   = (tgt_p_q > cur_p_q);
  wire down_w = (tgt_p_q < cur_p_q);

  // Drive requested setpoints
  assign vsel_o      = opp_v_q[tgt_p_q];
  assign vreq_up_o   = up_w;               // for information/debug only
  assign freq_code_o = opp_f_q[tgt_p_q];

  // Slow-clock hint when not at max state
  assign slow_clk_hint_o = (cur_p_q != cfg_pmax_q);

  // Main FSM next-state
  localparam logic [11:0] EV_DVFS_START   = 12'h500;
  localparam logic [11:0] EV_DVFS_DONE    = 12'h501;
  localparam logic [11:0] EV_DVFS_PMIC_TO = 12'h502;
  localparam logic [11:0] EV_DVFS_PLL_TO  = 12'h503;
  localparam logic [11:0] EV_DVFS_EMERG   = 12'h504;

  // Event queue (single-entry)
  logic evt_q, evt_d;
  logic [11:0] evt_code_q, evt_code_d;
  logic [31:0] evt_info_q, evt_info_d;
  logic [1:0]  evt_sev_q,  evt_sev_d;

  assign evt_src_o = EVT_SRC_ID;

  // Defaults
  always_comb begin
    st_d       = st_q;
    wdog_d     = (st_q==S_V_WAIT || st_q==S_F_WAIT) ? (wdog_q + {{WDOG_W-1{1'b0}},1'b1}) : '0;

    cur_p_d    = cur_p_q;
    dwell_d    = dwell_q;
    last_err_d = last_err_q;

    // IRQ/event defaults
    irq_d      = irq_q;
    evt_d      = evt_q;
    evt_code_d = evt_code_q;
    evt_info_d = evt_info_q;
    evt_sev_d  = evt_sev_q;

    // Clear event when accepted
    if (evt_q && evt_ready_i) evt_d = 1'b0;

    // Recompute target every cycle
    // (tgt_p_d computed above; committed on seq side)

    unique case (st_q)
      S_IDLE: begin
        if (!cfg_enable_q || hold_i) begin
          // stay idle, no sequencing
        end else if (emerg_w) begin
          // Preempt immediately
          evt_d      = 1'b1; evt_code_d = EV_DVFS_EMERG; evt_sev_d = 2'd3;
          evt_info_d = { 16'd0, 8'(cur_p_q), 8'(cfg_emerg_q) };
          st_d       = S_EMERG;
          wdog_d     = '0;
        end else begin
          // Start a transition if target differs and dwell satisfied
          if ((tgt_p_q != cur_p_q) && (dwell_q == '0)) begin
            // Emit start event
            evt_d      = 1'b1; evt_code_d = EV_DVFS_START; evt_sev_d = 2'd2;
            evt_info_d = { 16'd0, 8'(cur_p_q), 8'(tgt_p_q) };
            // Decide which sub-state to start with
            if (tgt_p_q > cur_p_q) st_d = S_V_UP;   // raise V first
            else                   st_d = S_F_DOWN; // lower f first
          end
        end
      end

      // --- Voltage path ---
      S_V_UP: begin
        // Request new voltage (higher). Wait for ack then power-good.
        if (vreq_ack_i) st_d = S_V_WAIT;
      end
      S_V_DOWN: begin
        // Request lower voltage. Wait for ack then power-good.
        if (vreq_ack_i) st_d = S_V_WAIT;
      end
      S_V_WAIT: begin
        if (vready_i) begin
          // Voltage settled; now adjust frequency depending on direction
          if (up_w) st_d = S_F_UP;
          else      st_d = S_F_DOWN; // only used when sequencing via S_F_DOWN → S_V_DOWN
          wdog_d = '0;
        end else if (wdog_q >= cfg_pmic_to_q) begin
          // PMIC timeout
          last_err_d = 8'h01;
          irq_d      = cfg_irq_en_q ? 1'b1 : irq_q;
          evt_d      = 1'b1; evt_code_d = EV_DVFS_PMIC_TO; evt_sev_d = 2'd3;
          evt_info_d = { 16'd0, 8'(cur_p_q), 8'(tgt_p_q) };
          st_d       = S_WAIT_DWELL; // back off; keep cur_p unchanged
          dwell_d    = cfg_dwell_q;
        end
      end

      // --- Frequency path ---
      S_F_UP: begin
        if (pll_ack_i) st_d = S_F_WAIT;
      end
      S_F_DOWN: begin
        if (pll_ack_i) st_d = S_F_WAIT;
      end
      S_F_WAIT: begin
        if (pll_lock_i) begin
          // Frequency settled; if we were moving down, now lower voltage; if moving up, we're done
          if (down_w) begin
            st_d = S_V_DOWN;
            wdog_d = '0;
          end else begin
            // Commit new P-state
            cur_p_d = tgt_p_q;
            st_d    = S_WAIT_DWELL;
            dwell_d = cfg_dwell_q;
            // Finish event
            evt_d      = 1'b1; evt_code_d = EV_DVFS_DONE; evt_sev_d = 2'd2;
            evt_info_d = { 16'd0, 8'(cur_p_q), 8'(tgt_p_q) };
          end
        end else if (wdog_q >= cfg_pll_to_q) begin
          // PLL timeout
          last_err_d = 8'h02;
          irq_d      = cfg_irq_en_q ? 1'b1 : irq_q;
          evt_d      = 1'b1; evt_code_d = EV_DVFS_PLL_TO; evt_sev_d = 2'd3;
          evt_info_d = { 16'd0, 8'(cur_p_q), 8'(tgt_p_q) };
          st_d       = S_WAIT_DWELL;
          dwell_d    = cfg_dwell_q;
        end
      end

      // --- Emergency: go to cfg_emerg_q in a safe order ---------------------
      S_EMERG: begin
        // Always: lower f to emerg first (if emerg < cur)
        // or raise V first (if emerg > cur)
        if (cur_p_q == cfg_emerg_q) begin
          st_d    = S_WAIT_DWELL;
          dwell_d = cfg_dwell_q;
        end else if (cur_p_q > cfg_emerg_q) begin
          // stepping down performance: lower f then lower V
          if (pll_ack_i) st_d = S_F_WAIT;
          else           st_d = S_F_DOWN;
        end else begin
          // stepping up performance (rare for emergency): raise V then raise f
          if (vreq_ack_i) st_d = S_V_WAIT;
          else            st_d = S_V_UP;
        end
      end

      // --- Dwell between transitions ----------------------------------------
      S_WAIT_DWELL: begin
        if (dwell_q != '0) dwell_d = dwell_q - {{WDOG_W-1{1'b0}},1'b1};
        else               st_d     = S_IDLE;
      end

      default: st_d = S_IDLE;
    endcase
  end

  // ==========================================================================
  // Sequential section
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      st_q            <= S_IDLE;
      wdog_q          <= '0;

      cfg_enable_q        <= 1'b0;
      cfg_force_override_q<= 1'b0;
      cfg_step_mode_q     <= 1'b1;
      cfg_irq_en_q        <= 1'b1;

      cfg_pmin_q          <= pidx_t'(0);
      cfg_pmax_q          <= pidx_t'(N_OPP-1);
      cfg_pcrit_q         <= pidx_t'(0);
      cfg_pwarn_q         <= pidx_t'(N_OPP>1 ? 1 : 0);
      cfg_pdl1_q          <= pidx_t'(N_OPP>1 ? N_OPP-2 : 0);
      cfg_pdl2_q          <= pidx_t'(N_OPP>2 ? N_OPP-3 : 0);
      cfg_pdl3_q          <= pidx_t'(N_OPP>3 ? N_OPP-4 : 0);
      cfg_emerg_q         <= pidx_t'(0);

      cfg_dwell_q         <= DWELL_DFLT;
      cfg_pmic_to_q       <= PMIC_TO;
      cfg_pll_to_q        <= PLL_TO;

      cfg_p_idle_q        <= pidx_t'(0);
      cfg_p_typ_q         <= pidx_t'( (N_OPP>1) ? N_OPP/2 : 0 );
      cfg_p_max_q         <= pidx_t'(N_OPP-1);

      cur_p_q             <= pidx_t'(0);
      req_p_q             <= pidx_t'(0);
      tgt_p_q             <= pidx_t'(0);
      dwell_q             <= '0;

      irq_q               <= 1'b0;
      last_err_q          <= 8'd0;

      evt_q               <= 1'b0;
      evt_code_q          <= 12'd0;
      evt_info_q          <= 32'd0;
      evt_sev_q           <= 2'd0;

      // Default OPPs = zero
      for (int i=0;i<N_OPP;i++) begin
        opp_v_q[i] <= '0;
        opp_f_q[i] <= '0;
      end

    end else begin
      st_q        <= st_d;
      wdog_q      <= wdog_d;

      cfg_enable_q        <= cfg_enable_d;
      cfg_force_override_q<= cfg_force_override_d;
      cfg_step_mode_q     <= cfg_step_mode_d;
      cfg_irq_en_q        <= cfg_irq_en_d;

      cfg_pmin_q          <= cfg_pmin_d;
      cfg_pmax_q          <= cfg_pmax_d;
      cfg_pcrit_q         <= cfg_pcrit_d;
      cfg_pwarn_q         <= cfg_pwarn_d;
      cfg_pdl1_q          <= cfg_pdl1_d;
      cfg_pdl2_q          <= cfg_pdl2_d;
      cfg_pdl3_q          <= cfg_pdl3_d;
      cfg_emerg_q         <= cfg_emerg_d;

      cfg_dwell_q         <= cfg_dwell_d;
      cfg_pmic_to_q       <= cfg_pmic_to_d;
      cfg_pll_to_q        <= cfg_pll_to_d;

      cfg_p_idle_q        <= cfg_p_idle_d;
      cfg_p_typ_q         <= cfg_p_typ_d;
      cfg_p_max_q         <= cfg_p_max_d;

      cur_p_q             <= cur_p_d;
      req_p_q             <= req_p_d;
      tgt_p_q             <= tgt_p_d;
      dwell_q             <= dwell_d;

      irq_q               <= irq_d;
      last_err_q          <= last_err_d;

      evt_q               <= evt_d;
      evt_code_q          <= evt_code_d;
      evt_info_q          <= evt_info_d;
      evt_sev_q           <= evt_sev_d;
    end
  end

  // External status
  assign cur_pstate_o = cur_p_q;
  assign tgt_pstate_o = tgt_p_q;

  // ==========================================================================
  // CSR bank (single-cycle ready)
  // ==========================================================================
  assign csr_ready_o = csr_valid_i;

  // CSR map
  localparam [31:0] ID_VERSION = {8'h47, 8'h56, 8'd1, 8'd0}; // 'G''V' v1.0

  localparam int REG_ID        = 'h000 >> 2;
  localparam int REG_CTRL      = 'h004 >> 2; // [0]=enable [1]=force_override [2]=step_mode [8]=irq_en
  localparam int REG_STATUS    = 'h008 >> 2; // [7:0]=last_err [15:8]=st [23:16]=cur_p [31:24]=tgt_p
  localparam int REG_CLAMP0    = 'h00C >> 2; // [7:0]=pmin [15:8]=pmax [23:16]=pwarn [31:24]=pcrit
  localparam int REG_CLAMP1    = 'h010 >> 2; // [7:0]=pdl1 [15:8]=pdl2 [23:16]=pdl3 [31:24]=emerg
  localparam int REG_TIMEOUTS  = 'h014 >> 2; // [15:0]=pmic_to[15:0], [31:16]=pll_to[15:0] (LSBs)
  localparam int REG_TIMEOUTS_H= 'h018 >> 2; // high halves for long counters (optional)
  localparam int REG_DWELL     = 'h01C >> 2; // dwell cycles (WDOG_W LSBs)
  localparam int REG_MAP       = 'h020 >> 2; // [7:0]=p_idle [15:8]=p_typ [23:16]=p_max
  localparam int REG_IRQ       = 'h024 >> 2; // [0]=irq (RO), [8]=irq_en (RW), [16]=W1C clear
  localparam int REG_PSTATE    = 'h028 >> 2; // [7:0]=cur [15:8]=tgt [23:16]=req (raw)
  // OPP table: base 0x100, each OPP uses two words: V then F.
  localparam int REG_OPP_BASE  = 'h100 >> 2;

  // Writes
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin end
    else if (csr_valid_i && csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_CTRL: begin
          if (csr_wstrb_i[0]) begin
            cfg_enable_q         <= csr_wdata_i[0];
            cfg_force_override_q <= csr_wdata_i[1];
            cfg_step_mode_q      <= csr_wdata_i[2];
          end
          if (csr_wstrb_i[1]) begin
            cfg_irq_en_q         <= csr_wdata_i[8];
          end
        end
        REG_CLAMP0: if (&csr_wstrb_i) begin
          cfg_pmin_q  <= pidx_t'(csr_wdata_i[7:0]);
          cfg_pmax_q  <= pidx_t'(csr_wdata_i[15:8]);
          cfg_pwarn_q <= pidx_t'(csr_wdata_i[23:16]);
          cfg_pcrit_q <= pidx_t'(csr_wdata_i[31:24]);
        end
        REG_CLAMP1: if (&csr_wstrb_i) begin
          cfg_pdl1_q  <= pidx_t'(csr_wdata_i[7:0]);
          cfg_pdl2_q  <= pidx_t'(csr_wdata_i[15:8]);
          cfg_pdl3_q  <= pidx_t'(csr_wdata_i[23:16]);
          cfg_emerg_q <= pidx_t'(csr_wdata_i[31:24]);
        end
        REG_TIMEOUTS: begin
          if (&csr_wstrb_i) begin
            cfg_pmic_to_q[15:0] <= csr_wdata_i[15:0];
            cfg_pll_to_q [15:0] <= csr_wdata_i[31:16];
          end
        end
        REG_TIMEOUTS_H: begin
          if (&csr_wstrb_i) begin
            cfg_pmic_to_q[WDOG_W-1:16] <= csr_wdata_i[15:0];
            cfg_pll_to_q [WDOG_W-1:16] <= csr_wdata_i[31:16];
          end
        end
        REG_DWELL: if (&csr_wstrb_i) cfg_dwell_q <= csr_wdata_i[WDOG_W-1:0];
        REG_MAP: begin
          if (&csr_wstrb_i) begin
            cfg_p_idle_q <= pidx_t'(csr_wdata_i[7:0]);
            cfg_p_typ_q  <= pidx_t'(csr_wdata_i[15:8]);
            cfg_p_max_q  <= pidx_t'(csr_wdata_i[23:16]);
          end
        end
        REG_IRQ: begin
          if (csr_wstrb_i[2] && csr_wdata_i[16]) irq_q <= 1'b0; // W1C
          if (csr_wstrb_i[1]) cfg_irq_en_q <= csr_wdata_i[8];
        end
        default: begin
          // OPP table window
          if (csr_addr_i[CSR_AW-1:2] >= REG_OPP_BASE &&
              csr_addr_i[CSR_AW-1:2] <  REG_OPP_BASE + (N_OPP*2)) begin
            int idx = csr_addr_i[CSR_AW-1:2] - REG_OPP_BASE;
            int op  = idx >> 1;            // OPP index
            int vw  = idx[0];              // 0 = V word, 1 = F word
            if (vw == 0) begin
              // write VSEL (LSBs of word)
              for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
                for (int k=0;k<8;k++)
                  if ((b*8+k) < VSEL_W) opp_v_q[op][b*8+k] <= csr_wdata_i[b*8+k];
              end
            end else begin
              // write FREQ code (LSBs of word)
              for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
                for (int k=0;k<8;k++)
                  if ((b*8+k) < PLL_W) opp_f_q[op][b*8+k] <= csr_wdata_i[b*8+k];
              end
            end
          end
        end
      endcase
    end
  end

  // Reads
  always_comb begin
    csr_rdata_o = 32'h0;
    if (csr_valid_i && !csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_ID    : csr_rdata_o = ID_VERSION;
        REG_CTRL  : csr_rdata_o = { 23'd0, cfg_irq_en_q, 5'd0, cfg_step_mode_q,
                                    cfg_force_override_q, cfg_enable_q };
        REG_STATUS: csr_rdata_o = { tgt_p_q, cur_p_q, st_q, last_err_q };
        REG_CLAMP0: csr_rdata_o = { cfg_pcrit_q, cfg_pwarn_q, cfg_pmax_q, cfg_pmin_q };
        REG_CLAMP1: csr_rdata_o = { cfg_emerg_q, cfg_pdl3_q, cfg_pdl2_q, cfg_pdl1_q };
        REG_TIMEOUTS:  csr_rdata_o = { cfg_pll_to_q[15:0],  cfg_pmic_to_q[15:0] };
        REG_TIMEOUTS_H:csr_rdata_o = { cfg_pll_to_q[WDOG_W-1:16], cfg_pmic_to_q[WDOG_W-1:16] };
        REG_DWELL : csr_rdata_o = { {(32-WDOG_W){1'b0}}, cfg_dwell_q };
        REG_MAP   : csr_rdata_o = { 8'd0, cfg_p_max_q, cfg_p_typ_q, cfg_p_idle_q };
        REG_IRQ   : csr_rdata_o = { 15'd0, 1'(cfg_irq_en_q), 7'd0, irq_q };
        REG_PSTATE: csr_rdata_o = { 8'(req_p_q), 8'(tgt_p_q), 8'(cur_p_q), 8'd0 };

        default: begin
          if (csr_addr_i[CSR_AW-1:2] >= REG_OPP_BASE &&
              csr_addr_i[CSR_AW-1:2] <  REG_OPP_BASE + (N_OPP*2)) begin
            int idx = csr_addr_i[CSR_AW-1:2] - REG_OPP_BASE;
            int op  = idx >> 1;
            int vw  = idx[0];
            if (vw == 0) begin
              logic [31:0] v; v='0;
              for (int k=0;k<VSEL_W;k++) v[k]=opp_v_q[op][k];
              csr_rdata_o = v;
            end else begin
              logic [31:0] v; v='0;
              for (int k=0;k<PLL_W;k++) v[k]=opp_f_q[op][k];
              csr_rdata_o = v;
            end
          end else csr_rdata_o = 32'hBADC_AB1E;
        end
      endcase
    end
  end

  // ==========================================================================
  // IRQ and Events
  // ==========================================================================
  // Raise IRQ on timeouts; W1C via CSR
  assign irq_o     = irq_q;

  // Event wires
  assign evt_valid_o = evt_q;
  assign evt_code_o  = evt_code_q;
  assign evt_info_o  = evt_info_q;
  assign evt_sev_o   = evt_sev_q;

  // ==========================================================================
  // Synthesis / safety assertions
  // ==========================================================================
`ifdef ASSERT_ON
  // Pmin <= Pmax
  always_ff @(posedge clk_i) begin
    assert (cfg_pmin_q <= cfg_pmax_q)
      else $error("dvfs_ctrl: pmin > pmax");
  end
  // OPP indices within range
  initial begin
    if (N_OPP < 2) $warning("dvfs_ctrl: N_OPP<2; DVFS degenerates to single OPP.");
  end
`endif

endmodule

`default_nettype wire
