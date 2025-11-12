// ============================================================================
//  voltage_droop_if.sv
//  Aquila — Voltage Droop Interface (async pulse CDC + windowed rate + CSR)
// ----------------------------------------------------------------------------
//  Features
//   - N asynchronous droop inputs -> 3FF synchronizers + configurable pulse stretch
//   - Per-channel:
//       * total_count (saturating)
//       * window_count (cleared each window)
//       * last_event timestamp (free-running counter)
//   - Global aggregation per window: SUM or MAX of channel window_count[]
//   - Trip levels with hysteresis/hold (WARN / HOT / CRIT), sticky IRQ bits
//   - "Storm" detector: too many droops in a window -> event
//   - CSR-Lite window (single-cycle ready) with per-channel indexed access
//   - Software injection (W1P) to emulate droop pulses
//   - Structured events to err_logger (CRIT entry, STORM)
// ----------------------------------------------------------------------------
//  Clocks/Domains
//   - All logic synchronous to clk_i. droop_ai[*] is asynchronous and is
//     synchronized internally using 3-stage sync and optional pulse stretch.
// ----------------------------------------------------------------------------
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module voltage_droop_if #(
  parameter int unsigned N_CH       = 16,   // number of droop monitors
  parameter int unsigned CSR_AW     = 12,   // 4 KiB CSR window
  parameter bit          USE_SUM    = 1'b1, // 1: aggregate by SUM; 0: aggregate by MAX
  // Default thresholds (events-per-window)
  parameter int unsigned DEF_T_WARN = 1,
  parameter int unsigned DEF_T_HOT  = 4,
  parameter int unsigned DEF_T_CRIT = 8,
  parameter int unsigned DEF_STORM  = 32,   // sum/max events per window to flag storm
  // Event source ID
  parameter logic [7:0]  EVT_SRC_ID = 8'h56 // 'V' for Voltage
)(
  input  logic                     clk_i,
  input  logic                     rstn_i,           // synchronous, active-low

  // ======================= Asynchronous droop inputs =========================
  input  logic [N_CH-1:0]          droop_ai,         // async pulses (active-high)

  // ======================= CSR-Lite =========================================
  input  logic                     csr_valid_i,
  input  logic                     csr_write_i,
  input  logic [CSR_AW-1:0]        csr_addr_i,       // byte address
  input  logic [31:0]              csr_wdata_i,
  input  logic [3:0]               csr_wstrb_i,
  output logic                     csr_ready_o,
  output logic [31:0]              csr_rdata_o,

  // ======================= Actions / status outputs ==========================
  output logic [1:0]               throttle_lvl_o,   // 0=none,1=warn,2=hot,3=crit
  output logic                     dvfs_boost_req_o, // request higher V/f (lvl>=1)
  output logic                     emergency_o,      // assert at CRIT
  output logic                     irq_o,            // OR of unmasked sticky IRQs

  // ======================= Events to err_logger ==============================
  output logic                     evt_valid_o,
  input  logic                     evt_ready_i,
  output logic [7:0]               evt_src_o,
  output logic [1:0]               evt_sev_o,        // 2=ERR, 3=FATAL
  output logic [11:0]              evt_code_o,
  output logic [31:0]              evt_info_o
);

  // ==========================================================================
  // Constants / CSR map
  // ==========================================================================
  localparam int unsigned IDX_W = (N_CH<=1) ? 1 : $clog2(N_CH);
  localparam [31:0] ID_VERSION = {8'h56, 8'h44, 8'd1, 8'd0}; // 'V''D' v1.0

  // CSR word indices (byte>>2)
  localparam int REG_ID        = 'h000 >> 2;
  localparam int REG_CTRL      = 'h004 >> 2; // [0]=enable [1]=clr_counts(W1P) [2]=clr_irq(W1C)
                                             // [3]=force_window_end(W1P) [7:4]=reserved
  localparam int REG_STATUS    = 'h008 >> 2; // [0]=warn [1]=hot [2]=crit [3]=storm [15:8]=hot_idx
                                             // [31:16]=cur_window_rem (low 16b)
  localparam int REG_CFG_WIN   = 'h00C >> 2; // [15:0]=window_cycles (must be >=1)
                                             // [31:16]=hold_windows (hysteresis in windows)
  localparam int REG_CFG_THR   = 'h010 >> 2; // [7:0]=t_warn [15:8]=t_hot [23:16]=t_crit [31:24]=storm_thr (sum/max)
  localparam int REG_CFG_MISC  = 'h014 >> 2; // [7:0]=stretch_cycles [8]=aggregate_mode(1=sum,0=max)
  localparam int REG_IRQSTAT   = 'h018 >> 2; // W1C sticky: [0]=warn [1]=hot [2]=crit [3]=storm
  localparam int REG_IRQMASK   = 'h01C >> 2; // [3:0] mask (1=masked)
  localparam int REG_CH_EN_LO  = 'h020 >> 2; // [31:0] enable mask
  localparam int REG_CH_EN_HI  = 'h024 >> 2; // [N_CH-1:32]
  localparam int REG_SEL_CH    = 'h028 >> 2; // select channel index for indexed regs below
  localparam int REG_CH_TOTAL  = 'h02C >> 2; // RO total_count[selected]
  localparam int REG_CH_WIN    = 'h030 >> 2; // RO last_window_count[selected]
  localparam int REG_CH_LASTTS = 'h034 >> 2; // RO last_event timestamp (low 32b)
  localparam int REG_INJECT_LO = 'h038 >> 2; // W1P software inject bits [31:0]
  localparam int REG_INJECT_HI = 'h03C >> 2; // W1P software inject bits [N_CH-1:32]
  localparam int REG_SUM_LAST  = 'h040 >> 2; // RO last_window_sum (or max)
  localparam int REG_STORMCNT  = 'h044 >> 2; // RO storm_count_total (saturating)

  // Event codes
  localparam logic [11:0] EV_DROOP_CRIT  = 12'h320;
  localparam logic [11:0] EV_DROOP_STORM = 12'h321;

  // ==========================================================================
  // Configuration / State Registers
  // ==========================================================================
  // Control
  logic        en_q, en_d;
  logic [15:0] hold_windows_q, hold_windows_d;
  logic [15:0] window_cycles_q, window_cycles_d;
  logic [7:0]  stretch_cycles_q, stretch_cycles_d;  // input pulse stretch (sync domain)
  logic        aggregate_sum_q, aggregate_sum_d;    // 1=sum, 0=max (can override USE_SUM)

  // Thresholds (events per window)
  logic [7:0]  thr_warn_q, thr_warn_d;
  logic [7:0]  thr_hot_q,  thr_hot_d;
  logic [7:0]  thr_crit_q, thr_crit_d;
  logic [7:0]  thr_storm_q,thr_storm_d;

  // Channel mask
  logic [N_CH-1:0] ch_en_q, ch_en_d;

  // IRQs
  logic [3:0] irq_stat_q, irq_stat_d;  // [0]=warn [1]=hot [2]=crit [3]=storm
  logic [3:0] irq_mask_q, irq_mask_d;
  logic       irq_latched_w;

  // Aggregation & windowing
  logic [31:0] timebase_q, timebase_d; // free-running (cycles)
  logic [31:0] win_cnt_q, win_cnt_d;   // counts down to window end (0->end)
  logic [15:0] hold_cnt_q, hold_cnt_d; // windows to keep level after quiet
  logic [1:0]  lvl_q, lvl_d;           // 0..3 (none/warn/hot/crit)
  logic        storm_q, storm_d;
  logic [31:0] last_window_sum_q, last_window_sum_d;
  logic [31:0] storm_total_q, storm_total_d;

  // Per-channel telemetry
  logic [31:0] total_cnt_q [N_CH];
  logic [15:0] win_cnt_ch_q[N_CH];
  logic [31:0] last_ts_q   [N_CH];
  logic [IDX_W-1:0] hot_idx_q, hot_idx_d;

  // Sync / stretch machinery per channel
  logic [N_CH-1:0] inject_pulse_w; // from CSR
  logic [N_CH-1:0] droop_sync2_q, droop_sync3_q;
  logic [N_CH-1:0] droop_sync1_q;
  logic [N_CH-1:0] droop_edge_w;
  logic [N_CH-1:0] stretch_act_q, stretch_act_d;
  logic [7:0]      stretch_cnt_q [N_CH];
  logic [7:0]      stretch_cnt_d [N_CH];

  // CSR selected channel index
  logic [IDX_W-1:0] sel_ch_q, sel_ch_d;

  // Events (single-entry)
  logic evt_v_q, evt_v_d;
  logic [11:0] evt_code_q, evt_code_d;
  logic [31:0] evt_info_q, evt_info_d;

  // ==========================================================================
  // CSR front-end
  // ==========================================================================
  assign csr_ready_o = csr_valid_i;

  // Writes
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      en_q            <= 1'b0;
      window_cycles_q <= 16'd10000; // default window
      hold_windows_q  <= 16'd2;
      stretch_cycles_q<= 8'd2;
      aggregate_sum_q <= (USE_SUM ? 1'b1 : 1'b0);

      thr_warn_q      <= 8'(DEF_T_WARN);
      thr_hot_q       <= 8'(DEF_T_HOT);
      thr_crit_q      <= 8'(DEF_T_CRIT);
      thr_storm_q     <= 8'(DEF_STORM);

      ch_en_q         <= {N_CH{1'b1}};
      irq_mask_q      <= 4'b0000;

      sel_ch_q        <= '0;

      // telemetry/counters init
      timebase_q      <= 32'd0;
      win_cnt_q       <= 32'd0;
      hold_cnt_q      <= 16'd0;
      lvl_q           <= 2'd0;
      storm_q         <= 1'b0;
      last_window_sum_q <= 32'd0;
      storm_total_q   <= 32'd0;
      irq_stat_q      <= 4'b0000;

      for (int i=0;i<N_CH;i++) begin
        total_cnt_q[i]   <= 32'd0;
        win_cnt_ch_q[i]  <= 16'd0;
        last_ts_q[i]     <= 32'd0;
        stretch_cnt_q[i] <= 8'd0;
        stretch_act_q[i] <= 1'b0;
      end
    end else if (csr_valid_i && csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_CTRL: begin
          if (csr_wstrb_i[0]) begin
            en_q <= csr_wdata_i[0];
            // clr_counts (W1P)
            if (csr_wdata_i[1]) begin
              for (int i=0;i<N_CH;i++) begin
                total_cnt_q[i]  <= 32'd0;
                win_cnt_ch_q[i] <= 16'd0;
              end
              storm_total_q <= 32'd0;
            end
            // clr_irq (W1C)
            if (csr_wdata_i[2]) irq_stat_q <= 4'b0000;
            // force_window_end (W1P)
            if (csr_wdata_i[3]) win_cnt_q <= 32'd0;
          end
        end
        REG_CFG_WIN: if (&csr_wstrb_i) begin
          window_cycles_q <= (csr_wdata_i[15:0] == 16'd0) ? 16'd1 : csr_wdata_i[15:0];
          hold_windows_q  <= csr_wdata_i[31:16];
        end
        REG_CFG_THR: if (&csr_wstrb_i) begin
          thr_warn_q   <= csr_wdata_i[7:0];
          thr_hot_q    <= csr_wdata_i[15:8];
          thr_crit_q   <= csr_wdata_i[23:16];
          thr_storm_q  <= csr_wdata_i[31:24];
        end
        REG_CFG_MISC: begin
          if (csr_wstrb_i[0]) stretch_cycles_q <= csr_wdata_i[7:0];
          if (csr_wstrb_i[1]) aggregate_sum_q  <= csr_wdata_i[8];
        end
        REG_IRQMASK: if (csr_wstrb_i[0]) irq_mask_q <= csr_wdata_i[3:0];
        REG_CH_EN_LO: for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
                         for (int k=0;k<8;k++) if (b*8+k < N_CH) ch_en_q[b*8+k] <= csr_wdata_i[b*8+k];
                       end
        REG_CH_EN_HI: for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
                         int base = 32 + b*8;
                         for (int k=0;k<8;k++) if (base+k < N_CH) ch_en_q[base+k] <= csr_wdata_i[b*8+k];
                       end
        REG_SEL_CH:  if (csr_wstrb_i[0]) sel_ch_q <= csr_wdata_i[IDX_W-1:0];
        REG_INJECT_LO: begin
          // W1P inject pulses (lower 32 channels)
          for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
            for (int k=0;k<8;k++) if ((b*8+k) < N_CH) begin
              if (csr_wdata_i[b*8+k]) stretch_act_q[b*8+k] <= 1'b1; // treat as immediate stretched pulse
              if (csr_wdata_i[b*8+k] && stretch_cnt_q[b*8+k] == 8'd0)
                stretch_cnt_q[b*8+k] <= (stretch_cycles_q==8'd0) ? 8'd1 : stretch_cycles_q;
            end
          end
        end
        REG_INJECT_HI: begin
          // W1P inject pulses (channels 32..)
          for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
            int base = 32 + b*8;
            for (int k=0;k<8;k++) if (base+k < N_CH) begin
              if (csr_wdata_i[b*8+k]) stretch_act_q[base+k] <= 1'b1;
              if (csr_wdata_i[b*8+k] && stretch_cnt_q[base+k] == 8'd0)
                stretch_cnt_q[base+k] <= (stretch_cycles_q==8'd0) ? 8'd1 : stretch_cycles_q;
            end
          end
        end
        default: ; // no-op
      endcase
    end
  end

  // Reads
  always_comb begin
    csr_rdata_o = 32'h0;
    if (csr_valid_i && !csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_ID      : csr_rdata_o = ID_VERSION;
        REG_CTRL    : csr_rdata_o = {24'd0, 4'd0/*force/clr shown via W1P*/, 1'b0/*clr_irq*/,
                                     1'b0/*clr_counts*/, en_q};
        REG_STATUS  : csr_rdata_o = { win_cnt_q[31:16], {8'(hot_idx_q)}, storm_q, (lvl_q==2'd3),
                                      (lvl_q>=2'd2), (lvl_q>=2'd1) };
        REG_CFG_WIN : csr_rdata_o = { hold_windows_q, window_cycles_q };
        REG_CFG_THR : csr_rdata_o = { thr_storm_q, thr_crit_q, thr_hot_q, thr_warn_q };
        REG_CFG_MISC: csr_rdata_o = { 23'd0, aggregate_sum_q, stretch_cycles_q };
        REG_IRQSTAT : csr_rdata_o = { 28'd0, irq_stat_q };
        REG_IRQMASK : csr_rdata_o = { 28'd0, irq_mask_q };
        REG_CH_EN_LO: csr_rdata_o = ch_en_q[31:0];
        REG_CH_EN_HI: csr_rdata_o = (N_CH>32) ? {{(32-(N_CH-32)){1'b0}}, ch_en_q[N_CH-1:32]} : 32'd0;
        REG_SEL_CH  : csr_rdata_o = {{(32-IDX_W){1'b0}}, sel_ch_q};
        REG_CH_TOTAL: csr_rdata_o = total_cnt_q[sel_ch_q];
        REG_CH_WIN  : csr_rdata_o = { 16'd0, win_cnt_ch_q[sel_ch_q] };
        REG_CH_LASTTS:csr_rdata_o = last_ts_q   [sel_ch_q];
        REG_SUM_LAST: csr_rdata_o = last_window_sum_q;
        REG_STORMCNT: csr_rdata_o = storm_total_q;
        default     : csr_rdata_o = 32'hBADC_AB1E;
      endcase
    end
  end

  // ==========================================================================
  // Asynchronous input sync + stretch + counting
  // ==========================================================================
  // Free-running timebase
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) timebase_q <= 32'd0;
    else         timebase_q <= timebase_q + 32'd1;
  end

  // 3-stage synchronizer per channel + rising-edge detect
  generate
    for (genvar i=0;i<N_CH;i++) begin : g_sync
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          droop_sync1_q[i] <= 1'b0;
          droop_sync2_q[i] <= 1'b0;
          droop_sync3_q[i] <= 1'b0;
        end else begin
          droop_sync1_q[i] <= droop_ai[i];
          droop_sync2_q[i] <= droop_sync1_q[i];
          droop_sync3_q[i] <= droop_sync2_q[i];
        end
      end
      assign droop_edge_w[i] = droop_sync2_q[i] & ~droop_sync3_q[i];
    end
  endgenerate

  // Optional stretch per channel (sync domain)
  generate
    for (genvar i=0;i<N_CH;i++) begin : g_stretch
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          stretch_cnt_q[i] <= 8'd0;
          stretch_act_q[i] <= 1'b0;
        end else begin
          // Start stretch on edge or software inject
          if ((droop_edge_w[i] | inject_pulse_w[i]) && ch_en_q[i] && en_q) begin
            stretch_act_q[i] <= 1'b1;
            stretch_cnt_q[i] <= (stretch_cycles_q==8'd0) ? 8'd1 : stretch_cycles_q;
          end else if (stretch_act_q[i]) begin
            if (stretch_cnt_q[i] > 8'd1) begin
              stretch_cnt_q[i] <= stretch_cnt_q[i] - 8'd1;
            end else begin
              // end of stretch
              stretch_cnt_q[i] <= 8'd0;
              stretch_act_q[i] <= 1'b0;
            end
          end
        end
      end

      // Count & timestamp on first cycle of stretch
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          total_cnt_q[i]  <= 32'd0;
          win_cnt_ch_q[i] <= 16'd0;
          last_ts_q[i]    <= 32'd0;
        end else if (en_q) begin
          // rising of stretch_act -> event
          if ((droop_edge_w[i] | inject_pulse_w[i]) && ch_en_q[i]) begin
            // total (saturate)
            if (total_cnt_q[i] != 32'hFFFF_FFFF) total_cnt_q[i] <= total_cnt_q[i] + 32'd1;
            // window (saturate at 0xFFFF)
            if (win_cnt_ch_q[i] != 16'hFFFF) win_cnt_ch_q[i] <= win_cnt_ch_q[i] + 16'd1;
            // timestamp
            last_ts_q[i] <= timebase_q;
          end
        end
      end
    end
  endgenerate

  // CSR software injection (single-cycle combinational decode)
  // (Already applied in write path by forcing stretch start; retain simple view)
  assign inject_pulse_w = '0; // we incorporated injection directly when writing REG_INJECT_*

  // ==========================================================================
  // Window end handling, aggregation, trip levels, IRQs, events
  // ==========================================================================
  // Window countdown
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      win_cnt_q <= 32'd0;
    end else if (!en_q) begin
      win_cnt_q <= 32'd0;
    end else begin
      if (win_cnt_q == 32'd0) win_cnt_q <= {16'd0, window_cycles_q} - 32'd1;
      else                     win_cnt_q <= win_cnt_q - 32'd1;
    end
  end

  wire window_end_w = en_q && (win_cnt_q == 32'd0);

  // Compute this-window aggregate and hot_idx
  logic [31:0] sum_w; logic [15:0] max_w; logic [IDX_W-1:0] max_idx_w;
  always_comb begin
    sum_w = 32'd0; max_w = 16'd0; max_idx_w = '0;
    for (int i=0;i<N_CH;i++) if (ch_en_q[i]) begin
      sum_w += win_cnt_ch_q[i];
      if (win_cnt_ch_q[i] > max_w) begin
        max_w     = win_cnt_ch_q[i];
        max_idx_w = IDX_W'(i);
      end
    end
  end

  // Trip level decision at window end
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      lvl_q              <= 2'd0;
      hold_cnt_q         <= 16'd0;
      storm_q            <= 1'b0;
      last_window_sum_q  <= 32'd0;
      hot_idx_q          <= '0;
      storm_total_q      <= 32'd0;
      irq_stat_q         <= 4'b0000;
      evt_v_q            <= 1'b0; evt_code_q <= 12'd0; evt_info_q <= 32'd0;
    end else begin
      // default: drop events when accepted
      if (evt_v_q && evt_ready_i) evt_v_q <= 1'b0;

      if (window_end_w) begin
        // Latch aggregates
        last_window_sum_q <= aggregate_sum_q ? sum_w : {16'd0, max_w};
        hot_idx_q         <= max_idx_w;

        // Determine raw level suggestion based on aggregate
        logic [7:0] agg8 = aggregate_sum_q ? (sum_w[7:0] | (sum_w>32'd255 ? 8'hFF : 8'h00))
                                           : (max_w[7:0]  | (max_w >16'd255 ? 8'hFF : 8'h00));
        logic [1:0] lvl_eval;
        if      (agg8 >= thr_crit_q) lvl_eval = 2'd3;
        else if (agg8 >= thr_hot_q ) lvl_eval = 2'd2;
        else if (agg8 >= thr_warn_q) lvl_eval = 2'd1;
        else                         lvl_eval = 2'd0;

        // Storm detection (uses storm threshold on same aggregate)
        logic storm_hit = (agg8 >= thr_storm_q);
        storm_q <= storm_hit;
        if (storm_hit && storm_total_q != 32'hFFFF_FFFF) storm_total_q <= storm_total_q + 32'd1;

        // Trip hysteresis in units of windows
        if (lvl_eval > lvl_q) begin
          // escalate immediately, reload hold counter
          lvl_q      <= lvl_eval;
          hold_cnt_q <= hold_windows_q;
          // Sticky IRQs on rising into bands
          if (lvl_q < 2'd1 && lvl_eval >= 2'd1) irq_stat_q[0] <= 1'b1;
          if (lvl_q < 2'd2 && lvl_eval >= 2'd2) irq_stat_q[1] <= 1'b1;
          if (lvl_q < 2'd3 && lvl_eval >= 2'd3) begin
            irq_stat_q[2] <= 1'b1;
            // Event: enter CRIT
            if (!evt_v_q) begin
              evt_v_q    <= 1'b1;
              evt_code_q <= EV_DROOP_CRIT;
              evt_info_q <= { 8'd0, 8'(lvl_eval), 8'(aggregate_sum_q ? 8'd1 : 8'd0), 8'(agg8) };
            end
          end
        end else begin
          // quiet window
          if (lvl_eval == 2'd0) begin
            if (hold_cnt_q != 16'd0) hold_cnt_q <= hold_cnt_q - 16'd1;
            else                      lvl_q     <= 2'd0;
          end else begin
            // still above some threshold; maintain level and refresh hold
            hold_cnt_q <= hold_windows_q;
          end
        end

        // Storm event (non-sticky IRQ bit 3)
        if (storm_hit) begin
          irq_stat_q[3] <= 1'b1;
          if (!evt_v_q) begin
            evt_v_q    <= 1'b1;
            evt_code_q <= EV_DROOP_STORM;
            evt_info_q <= { 8'd0, 8'(thr_storm_q), 8'(aggregate_sum_q ? 8'd1 : 8'd0), 8'(agg8) };
          end
        end

        // Clear window counts
        for (int i=0;i<N_CH;i++) win_cnt_ch_q[i] <= 16'd0;
      end
    end
  end

  // ==========================================================================
  // Outputs
  // ==========================================================================
  assign throttle_lvl_o  = lvl_q;
  assign dvfs_boost_req_o= (lvl_q != 2'd0);
  assign emergency_o     = (lvl_q == 2'd3);

  assign irq_latched_w   = |(irq_stat_q & ~irq_mask_q);
  assign irq_o           = irq_latched_w;

  // Events to err_logger
  assign evt_src_o = EVT_SRC_ID;
  // Treat CRIT as severity 3, STORM as severity 2
  assign evt_sev_o = (evt_code_q == EV_DROOP_CRIT) ? 2'd3 : 2'd2;
  assign evt_valid_o = evt_v_q;
  assign evt_code_o  = evt_code_q;
  assign evt_info_o  = evt_info_q;

  // ==========================================================================
  // Defensive assertions
  // ==========================================================================
`ifdef ASSERT_ON
  // Threshold monotonicity
  always_ff @(posedge clk_i) begin
    assert (thr_warn_q <= thr_hot_q) else $error("voltage_droop_if: warn>hot.");
    assert (thr_hot_q  <= thr_crit_q) else $error("voltage_droop_if: hot>crit.");
    assert (window_cycles_q != 16'd0) else $error("voltage_droop_if: window_cycles==0.");
  end
`endif

endmodule
`default_nettype wire
