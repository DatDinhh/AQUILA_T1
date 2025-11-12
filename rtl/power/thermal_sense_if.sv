// ============================================================================
//  thermal_sense_if.sv
//  Aquila — Thermal Sensor Interface (N channels, CDC + calibration + filter)
// ----------------------------------------------------------------------------
//  Features
//  - N parallel channels; raw codes sampled in sns_clk_i domain with valid strobe
//  - Safe CDC into clk_i via req/ack toggle + two-sample stability check
//  - Per-channel calibration: T = gain * code + offset (fixed-point Q8.8)
//  - IIR low-pass filter (y += (x - y) >> iir_shift), per-channel enable
//  - Warn/Crit comparators with hysteresis and per-channel enable masks
//  - Faults: timeout (no sample), code range violation, stuck (no change)
//  - Aggregation: hot-spot (max filtered), ORed warn/crit to throttle/shutdown
//  - CSR-Lite programming + status (single-cycle ready)
//  - Event emission to err_logger (warn/crit/fault) with compact payload
//
//  Notes
//  - CODE_W raw sensor code width (e.g., 12). Temperature uses Q8.8 fixed-point.
//  - Provide fuse/OTP trims by writing gains/offsets via CSR at boot.
//  - If sensors are already synchronous to clk_i, tie sns_clk_i=clk_i.
//  - This block does not generate ADC timing; it only consumes sampled codes.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module thermal_sense_if #(
  parameter int unsigned N_CH      = 8,     // number of thermal channels
  parameter int unsigned CODE_W    = 12,    // raw sensor code width
  parameter int unsigned QF        = 8,     // fractional bits for Q8.8 fixed-point
  parameter int unsigned TEMP_W    = 24,    // width of temperature fixed-point (>= QF+8)
  parameter int unsigned CSR_AW    = 12,    // CSR byte address width (4 KiB window)
  parameter int unsigned TO_BITS   = 24,    // timeout counter bits
  parameter int unsigned STUCK_BITS= 8,     // stuck window counter bits
  parameter logic [7:0]   EVT_SRC_ID = 8'h54 // 'T' Thermal
)(
  // =================== System clock domain ===================
  input  logic                         clk_i,
  input  logic                         rstn_i,       // synchronous, active-low

  // =================== Sensor clock domain ===================
  input  logic                         sns_clk_i,
  input  logic                         sns_rstn_i,   // synchronous to sns_clk_i
  input  logic [N_CH-1:0]              sns_valid_i,  // pulse per channel
  input  logic [N_CH*CODE_W-1:0]       sns_code_i,   // concat: ch i at [i*CODE_W +: CODE_W]

  // =================== Aggregated outputs ====================
  output logic                         warn_any_o,
  output logic                         crit_any_o,
  output logic                         shutdown_o,   // alias of crit_any_o (can be gated by CSR)
  output logic [TEMP_W-1:0]            hot_temp_o,   // max filtered temp across enabled chans
  output logic [$clog2(N_CH)-1:0]      hot_idx_o,

  // =================== CSR-Lite interface ====================
  input  logic                         csr_valid_i,
  input  logic                         csr_write_i,
  input  logic [CSR_AW-1:0]            csr_addr_i,   // byte address
  input  logic [31:0]                  csr_wdata_i,
  input  logic [3:0]                   csr_wstrb_i,
  output logic                         csr_ready_o,
  output logic [31:0]                  csr_rdata_o,

  // =================== Event to err_logger ===================
  output logic                         evt_valid_o,
  input  logic                         evt_ready_i,
  output logic [7:0]                   evt_src_o,
  output logic [1:0]                   evt_sev_o,    // 2=ERR, 3=FATAL
  output logic [11:0]                  evt_code_o,
  output logic [31:0]                  evt_info_o
);

  // -------------------- Helpers --------------------
  function automatic logic [CODE_W-1:0]
    code_at(input logic [N_CH*CODE_W-1:0] bus, input int unsigned i);
    return bus[i*CODE_W +: CODE_W];
  endfunction

  localparam int unsigned CH_W = (N_CH <= 1) ? 1 : $clog2(N_CH);
  localparam [31:0] ID_VERSION = {8'h54, 8'h53, 8'd1, 8'd0}; // 'T''S' v1.0

  // ==========================================================================
  // Sensor-domain capture with req/ack handshake per channel
  // ==========================================================================
  // For each channel i:
  //  sns_valid_i[i] latches sns_code_i into a hold register if no request is
  //  outstanding, toggles req_tgl[i] and waits for ack_tgl_sync==req_tgl before
  //  accepting the next sample. Overflows are counted as drops.
  //
  // System domain synchronizes req_tgl, performs a two-sample stability check
  // on the data bus (to minimize risk of meta on wide bus), then toggles ack.
  // --------------------------------------------------------------------------

  // Sensor domain storage and toggles
  logic [N_CH*CODE_W-1:0]   sns_hold_q;
  logic [N_CH-1:0]          sns_req_tgl_q;
  logic [N_CH-1:0]          sns_ack_tgl_sync_w1, sns_ack_tgl_sync_w2;
  logic [15:0]              sns_drop_cnt_q [N_CH];

  // Ack toggle synchronizers into sns_clk_i
  generate
    for (genvar i=0;i<N_CH;i++) begin : g_sns_sync
      always_ff @(posedge sns_clk_i or negedge sns_rstn_i) begin
        if (!sns_rstn_i) begin
          sns_ack_tgl_sync_w1[i] <= 1'b0;
          sns_ack_tgl_sync_w2[i] <= 1'b0;
          sns_hold_q[i*CODE_W +: CODE_W] <= '0;
          sns_req_tgl_q[i] <= 1'b0;
          sns_drop_cnt_q[i] <= 16'd0;
        end else begin
          sns_ack_tgl_sync_w1[i] <= ack_tgl_q[i];
          sns_ack_tgl_sync_w2[i] <= sns_ack_tgl_sync_w1[i];

          // Accept a new raw sample if no in-flight request
          if (sns_valid_i[i]) begin
            if (sns_req_tgl_q[i] == sns_ack_tgl_sync_w2[i]) begin
              sns_hold_q[i*CODE_W +: CODE_W] <= code_at(sns_code_i, i);
              sns_req_tgl_q[i]               <= ~sns_req_tgl_q[i];
            end else begin
              // outstanding sample not yet consumed -> drop
              if (sns_drop_cnt_q[i] != 16'hFFFF) sns_drop_cnt_q[i] <= sns_drop_cnt_q[i] + 16'd1;
            end
          end
        end
      end
    end
  endgenerate

  // ==========================================================================
  // System-domain side of the CDC handshake + stability sampler
  // ==========================================================================
  logic [N_CH-1:0] req_tgl_sync_w1, req_tgl_sync_w2;
  logic [N_CH-1:0] req_seen_q, ack_tgl_q;
  logic [N_CH-1:0] sample_pending_w, accept_pulse_w;

  // Double-flop req toggles
  generate
    for (genvar i=0;i<N_CH;i++) begin : g_req_sync
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          req_tgl_sync_w1[i] <= 1'b0;
          req_tgl_sync_w2[i] <= 1'b0;
          req_seen_q[i]      <= 1'b0;
          ack_tgl_q[i]       <= 1'b0;
        end else begin
          req_tgl_sync_w1[i] <= sns_req_tgl_q[i];
          req_tgl_sync_w2[i] <= req_tgl_sync_w1[i];
          req_seen_q[i]      <= req_tgl_sync_w2[i];
          // ack_tgl_q toggled when sampler accepts (in logic below)
        end
      end
    end
  endgenerate

  assign sample_pending_w = (req_tgl_sync_w2 ^ ack_tgl_q);

  // Per-channel two-sample stability sampler
  typedef enum logic [1:0] {S_IDLE, S_SAMP_A, S_SAMP_B} samp_e;
  samp_e                 samp_st_q   [N_CH], samp_st_d   [N_CH];
  logic [CODE_W-1:0]     samp_a_q    [N_CH], samp_b_q    [N_CH];
  logic [2:0]            samp_try_q  [N_CH], samp_try_d  [N_CH]; // retries (cap at 7)
  logic [CODE_W-1:0]     code_sys_pulse_w [N_CH];

  generate
    for (genvar i=0;i<N_CH;i++) begin : g_sampler
      // Default combinational
      always_comb begin
        samp_st_d[i]  = samp_st_q[i];
        samp_try_d[i] = samp_try_q[i];
        samp_b_q[i]   = samp_b_q[i];
        samp_a_q[i]   = samp_a_q[i];
      end

      // Sequential sampler FSM
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          samp_st_q[i]  <= S_IDLE;
          samp_try_q[i] <= 3'd0;
          samp_a_q[i]   <= '0;
          samp_b_q[i]   <= '0;
          code_sys_pulse_w[i] <= '0;
          // accept_pulse_w asserted below (continuous assignment not allowed inside ff)
        end else begin
          code_sys_pulse_w[i] <= code_sys_pulse_w[i]; // default hold
          case (samp_st_q[i])
            S_IDLE: begin
              code_sys_pulse_w[i] <= '0;
              if (sample_pending_w[i]) begin
                samp_a_q[i]  <= sns_hold_q[i*CODE_W +: CODE_W]; // asynchronous bus — stable under handshake
                samp_st_q[i] <= S_SAMP_A;
                samp_try_q[i]<= 3'd0;
              end
            end
            S_SAMP_A: begin
              samp_b_q[i]  <= sns_hold_q[i*CODE_W +: CODE_W];
              samp_st_q[i] <= S_SAMP_B;
            end
            S_SAMP_B: begin
              if (samp_b_q[i] == samp_a_q[i]) begin
                // Accept
                code_sys_pulse_w[i] <= samp_b_q[i];
                ack_tgl_q[i]        <= ~ack_tgl_q[i]; // complete handshake
                samp_st_q[i]        <= S_IDLE;
              end else begin
                // Retry up to 7 times
                samp_a_q[i]  <= samp_b_q[i];
                samp_try_q[i]<= samp_try_q[i] + 3'd1;
                if (&samp_try_q[i]) begin
                  // Give up and accept latest anyway
                  code_sys_pulse_w[i] <= samp_b_q[i];
                  ack_tgl_q[i]        <= ~ack_tgl_q[i];
                  samp_st_q[i]        <= S_IDLE;
                end
              end
            end
            default: samp_st_q[i] <= S_IDLE;
          endcase
        end
      end
    end
  endgenerate

  // Produce one-cycle accept pulse + code in clk_i domain
  logic [N_CH-1:0]       acc_pulse_w;
  logic [N_CH*CODE_W-1:0] acc_code_w;
  generate
    for (genvar i=0;i<N_CH;i++) begin : g_acc_pack
      assign acc_pulse_w[i] = (code_sys_pulse_w[i] !== 'x) && (samp_st_q[i] == S_IDLE) && (sample_pending_w[i]==1'b0); // simple detect on handshake close
      assign acc_code_w[i*CODE_W +: CODE_W] = code_sys_pulse_w[i];
    end
  endgenerate

  // ==========================================================================
  // Per-channel calibration, filter, fault detection (clk_i domain)
  // ==========================================================================
  // Config registers (CSR-programmable)
  // gain_i:  signed Q8.8  (TEMP = gain * code + offset)
  // offs_i:  signed Q8.8
  // mask/enables & iir shift are global (per-channel enable via en_mask)
  localparam int GAIN_W = 16;
  localparam int OFFS_W = 24; // enough headroom

  logic                    cfg_enable_q, cfg_enable_d;
  logic [3:0]              cfg_iir_shift_q, cfg_iir_shift_d; // 0..15
  logic [3:0]              cfg_decim_log2_q, cfg_decim_log2_d;
  logic [N_CH-1:0]         cfg_en_mask_q, cfg_en_mask_d;     // channel enable mask
  logic [N_CH-1:0]         cfg_irq_mask_q, cfg_irq_mask_d;   // mask warn/crit events per ch
  logic [GAIN_W-1:0]       cfg_gain_q [N_CH];  logic [GAIN_W-1:0] cfg_gain_d [N_CH];
  logic signed [OFFS_W-1:0]cfg_offs_q [N_CH];  logic signed [OFFS_W-1:0] cfg_offs_d [N_CH];

  // Thresholds (global), hysteresis
  logic signed [TEMP_W-1:0] cfg_warn_th_q, cfg_warn_th_d;
  logic signed [TEMP_W-1:0] cfg_crit_th_q, cfg_crit_th_d;
  logic [TEMP_W-1:0]        cfg_hyst_q,    cfg_hyst_d;
  logic                     cfg_shutdown_en_q, cfg_shutdown_en_d;

  // Fault configuration
  logic [TO_BITS-1:0]       cfg_timeout_cycles_q, cfg_timeout_cycles_d;
  logic [CODE_W-1:0]        cfg_code_min_q, cfg_code_min_d, cfg_code_max_q, cfg_code_max_d;
  logic [CODE_W-1:0]        cfg_stuck_delta_q, cfg_stuck_delta_d;
  logic [STUCK_BITS-1:0]    cfg_stuck_win_q,   cfg_stuck_win_d;

  // Live state per channel
  logic [CODE_W-1:0]        last_code_q [N_CH], last_code_d [N_CH];
  logic [STUCK_BITS-1:0]    stuck_ctr_q [N_CH], stuck_ctr_d [N_CH];
  logic [TO_BITS-1:0]       age_ctr_q   [N_CH], age_ctr_d   [N_CH];
  logic [TEMP_W-1:0]        temp_raw_q  [N_CH], temp_raw_d  [N_CH];
  logic [TEMP_W-1:0]        temp_filt_q [N_CH], temp_filt_d [N_CH];

  // Status bits
  logic [N_CH-1:0]          st_warn_q, st_warn_d;
  logic [N_CH-1:0]          st_crit_q, st_crit_d;
  logic [N_CH-1:0]          st_fault_to_q, st_fault_to_d;
  logic [N_CH-1:0]          st_fault_rng_q, st_fault_rng_d;
  logic [N_CH-1:0]          st_fault_stk_q, st_fault_stk_d;

  // Decimator (global)
  logic [3:0]               decim_cnt_q, decim_cnt_d;
  wire                      decim_fire = (cfg_decim_log2_q == 4'd0) ? 1'b1
                                  : (decim_cnt_q == 4'd0);

  // Fixed-point multiply helper
  function automatic logic signed [TEMP_W-1:0]
    fxp_mul_add(input logic [CODE_W-1:0] code_u,
                input logic signed [GAIN_W-1:0] gain_q88,
                input logic signed [OFFS_W-1:0] offs_q88);
    // code_u * gain (Q8.8) -> Q8.8 in wide int
    logic signed [31:0] prod;
    prod = $signed({1'b0, code_u}) * $signed(gain_q88); // Q8.8
    // widen offset to TEMP_W
    logic signed [TEMP_W-1:0] offs_w;
    offs_w = {{(TEMP_W-OFFS_W){offs_q88[OFFS_W-1]}}, offs_q88};
    // align prod (already Q8.8), fit to TEMP_W with saturation
    logic signed [TEMP_W-1:0] prod_w;
    prod_w = {{(TEMP_W-16){prod[15]}}, prod[15:0]}; // lower 16b hold Q8.8
    // sum
    logic signed [TEMP_W-1:0] sum = prod_w + offs_w;
    return sum;
  endfunction

  // IIR filter step: y += (x - y) >> k
  function automatic logic [TEMP_W-1:0]
    iir_step(input logic [TEMP_W-1:0] y, input logic [TEMP_W-1:0] x, input logic [3:0] k);
    logic signed [TEMP_W:0] diff;
    diff = $signed({1'b0,x}) - $signed({1'b0,y});
    return (k == 4'd0) ? x : (y + (diff >>> k));
  endfunction

  // Per-channel processing
  generate
    for (genvar i=0;i<N_CH;i++) begin : g_chan
      always_comb begin
        // defaults: hold
        last_code_d[i]   = last_code_q[i];
        stuck_ctr_d[i]   = stuck_ctr_q[i];
        age_ctr_d[i]     = cfg_enable_q ? (age_ctr_q[i] + {{(TO_BITS-1){1'b0}},1'b1}) : '0;
        temp_raw_d[i]    = temp_raw_q[i];
        temp_filt_d[i]   = temp_filt_q[i];
        st_fault_to_d[i] = st_fault_to_q[i];
        st_fault_rng_d[i]= st_fault_rng_q[i];
        st_fault_stk_d[i]= st_fault_stk_q[i];
        st_warn_d[i]     = st_warn_q[i];
        st_crit_d[i]     = st_crit_q[i];

        // consume accepted sample
        if (acc_pulse_w[i] && cfg_enable_q && cfg_en_mask_q[i]) begin
          logic [CODE_W-1:0] c = acc_code_w[i*CODE_W +: CODE_W];
          // range check
          if (c < cfg_code_min_q || c > cfg_code_max_q)
            st_fault_rng_d[i] = 1'b1;

          // timeout age reset
          age_ctr_d[i] = '0;

          // stuck detection
          logic [CODE_W-1:0] delta = (c >= last_code_q[i]) ? (c - last_code_q[i]) : (last_code_q[i] - c);
          if (cfg_stuck_win_q != '0 && delta <= cfg_stuck_delta_q) begin
            if (stuck_ctr_q[i] != {STUCK_BITS{1'b1}})
              stuck_ctr_d[i] = stuck_ctr_q[i] + {{(STUCK_BITS-1){1'b0}},1'b1};
            if (stuck_ctr_q[i] >= cfg_stuck_win_q) st_fault_stk_d[i] = 1'b1;
          end else begin
            stuck_ctr_d[i] = '0;
          end
          last_code_d[i] = c;

          // calibration to temperature
          temp_raw_d[i] = fxp_mul_add(c, cfg_gain_q[i], cfg_offs_q[i]);

          // IIR filter (only when decimator fires or k==0)
          if (decim_fire || cfg_iir_shift_q==4'd0)
            temp_filt_d[i] = iir_step(temp_filt_q[i], temp_raw_d[i], cfg_iir_shift_q);

          // Threshold comparators with hysteresis (filtered)
          logic signed [TEMP_W-1:0] t = temp_filt_d[i];
          // Warn
          if (!st_warn_q[i] && (t >= cfg_warn_th_q)) st_warn_d[i] = 1'b1;
          else if (st_warn_q[i] && (t <= (cfg_warn_th_q - $signed(cfg_hyst_q)))) st_warn_d[i] = 1'b0;
          // Crit
          if (!st_crit_q[i] && (t >= cfg_crit_th_q)) st_crit_d[i] = 1'b1;
          else if (st_crit_q[i] && (t <= (cfg_crit_th_q - $signed(cfg_hyst_q)))) st_crit_d[i] = 1'b0;
        end

        // timeout fault if age exceeds threshold
        if (cfg_timeout_cycles_q != '0 && age_ctr_q[i] >= cfg_timeout_cycles_q)
          st_fault_to_d[i] = 1'b1;

        // If channel disabled, clear dynamic state (keeps trims)
        if (!cfg_en_mask_q[i]) begin
          st_warn_d[i]      = 1'b0;
          st_crit_d[i]      = 1'b0;
          st_fault_to_d[i]  = 1'b0;
          st_fault_rng_d[i] = 1'b0;
          st_fault_stk_d[i] = 1'b0;
          age_ctr_d[i]      = '0;
          stuck_ctr_d[i]    = '0;
        end
      end
    end
  endgenerate

  // Decimator counter (global)
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) decim_cnt_q <= 4'd0;
    else begin
      if (cfg_decim_log2_q == 4'd0) decim_cnt_q <= 4'd0;
      else if (decim_cnt_q == 4'd0) decim_cnt_q <= (4'(1) << cfg_decim_log2_q) - 4'd1;
      else decim_cnt_q <= decim_cnt_q - 4'd1;
    end
  end

  // ==========================================================================
  // Aggregation, IRQ/Throttle, and Event emission
  // ==========================================================================
  // Aggregate hot-spot (max filtered temp) among enabled channels.
  logic [TEMP_W-1:0] hot_temp_w;
  logic [CH_W-1:0]   hot_idx_w;
  always_comb begin
    hot_temp_w = {TEMP_W{1'b0}};
    hot_idx_w  = '0;
    for (int i=0;i<N_CH;i++) begin
      if (cfg_en_mask_q[i]) begin
        if ($signed(temp_filt_q[i]) > $signed(hot_temp_w)) begin
          hot_temp_w = temp_filt_q[i];
          hot_idx_w  = CH_W'(i);
        end
      end
    end
  end
  assign hot_temp_o = hot_temp_w;
  assign hot_idx_o  = hot_idx_w;

  // Any warn/crit across enabled & unmasked channels
  logic warn_any_w, crit_any_w;
  always_comb begin
    warn_any_w = 1'b0;
    crit_any_w = 1'b0;
    for (int i=0;i<N_CH;i++) begin
      if (cfg_en_mask_q[i]) begin
        if (st_warn_q[i] && !cfg_irq_mask_q[i]) warn_any_w = 1'b1;
        if (st_crit_q[i] && !cfg_irq_mask_q[i]) crit_any_w = 1'b1;
      end
    end
  end

  assign warn_any_o  = warn_any_w;
  assign crit_any_o  = crit_any_w;
  assign shutdown_o  = cfg_shutdown_en_q ? crit_any_w : 1'b0;

  // Event generator (single-entry)
  localparam logic [11:0] EV_WARN = 12'h300;
  localparam logic [11:0] EV_CRIT = 12'h301;
  localparam logic [11:0] EV_FAULT= 12'h302;

  logic evt_pend_q, evt_pend_d;
  logic [11:0] evt_code_q, evt_code_d;
  logic [31:0] evt_info_q, evt_info_d;
  logic [1:0]  evt_sev_q,  evt_sev_d;

  // Edge detection for any warn/crit or any new fault bit per channel
  logic warn_any_q, crit_any_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin warn_any_q <= 1'b0; crit_any_q <= 1'b0; end
    else begin warn_any_q <= warn_any_w; crit_any_q <= crit_any_w; end
  end

  // Aggregate fault rising (any new fault across channels)
  logic any_fault_rise_w;
  logic [N_CH-1:0] fault_any_q, fault_any_w;
  always_comb begin
    fault_any_w = '0;
    for (int i=0;i<N_CH;i++) fault_any_w[i] = st_fault_to_q[i] | st_fault_rng_q[i] | st_fault_stk_q[i];
  end
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) any_fault_rise_w <= 1'b0;
    else any_fault_rise_w <= |(fault_any_w & ~fault_any_q);
  end
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) fault_any_q <= '0;
    else fault_any_q <= fault_any_w;
  end

  // Event FSM (very small)
  assign evt_src_o = EVT_SRC_ID;

  always_comb begin
    evt_pend_d = evt_pend_q;
    evt_code_d = evt_code_q;
    evt_info_d = evt_info_q;
    evt_sev_d  = evt_sev_q;

    // Queue on rising edges if idle
    if (!evt_pend_q) begin
      if (crit_any_w && !crit_any_q) begin
        evt_pend_d = 1'b1; evt_code_d = EV_CRIT; evt_sev_d = 2'd3;
        evt_info_d = { hot_idx_w, {(32-CH_W-16){1'b0}}, hot_temp_w[15:0] };
      end else if (warn_any_w && !warn_any_q) begin
        evt_pend_d = 1'b1; evt_code_d = EV_WARN; evt_sev_d = 2'd2;
        evt_info_d = { hot_idx_w, {(32-CH_W-16){1'b0}}, hot_temp_w[15:0] };
      end else if (any_fault_rise_w) begin
        evt_pend_d = 1'b1; evt_code_d = EV_FAULT; evt_sev_d = 2'd2;
        evt_info_d = 32'h0000_FF01; // generic fault (details via CSR status)
      end
    end else if (evt_pend_q && evt_ready_i) begin
      evt_pend_d = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      evt_pend_q <= 1'b0; evt_code_q <= 12'd0; evt_info_q <= 32'd0; evt_sev_q <= 2'd0;
    end else begin
      evt_pend_q <= evt_pend_d; evt_code_q <= evt_code_d; evt_info_q <= evt_info_d; evt_sev_q <= evt_sev_d;
    end
  end

  assign evt_valid_o = evt_pend_q;
  assign evt_code_o  = evt_code_q;
  assign evt_info_o  = evt_info_q;
  assign evt_sev_o   = evt_sev_q;

  // ==========================================================================
  // CSR bank (single-cycle ready)
  // ==========================================================================
  assign csr_ready_o = csr_valid_i;

  // CSR map (byte addresses, 32-bit words)
  localparam int REG_ID        = 'h000 >> 2;
  localparam int REG_CTRL      = 'h004 >> 2; // [0]=enable [1]=shutdown_en [7:4]=iir_shift [11:8]=decim_log2
  localparam int REG_STATUS    = 'h008 >> 2; // [0]=warn_any [1]=crit_any [15:8]=hot_idx [31:16]=hot_temp[15:0]
  localparam int REG_WARN_TH   = 'h00C >> 2; // Q8.8 in low 24 bits
  localparam int REG_CRIT_TH   = 'h010 >> 2;
  localparam int REG_HYST      = 'h014 >> 2;
  localparam int REG_TIMEOUT   = 'h018 >> 2;
  localparam int REG_CODE_MIN  = 'h01C >> 2; // [15:0]=min [31:16]=max
  localparam int REG_STUCKCFG  = 'h020 >> 2; // [15:0]=delta [31:16]=win
  localparam int REG_ENMASK_LO = 'h024 >> 2; // channel enable mask [31:0]
  localparam int REG_IRQMASK_LO= 'h028 >> 2; // irq mask [31:0]
  localparam int REG_GAIN_BASE = 'h040 >> 2; // N_CH words, gain_i in low 16b (Q8.8)
  localparam int REG_OFFS_BASE = 'h080 >> 2; // N_CH words, offs_i in low 24b (Q8.8 signed)
  localparam int REG_TEMP_FILT = 'h0C0 >> 2; // N_CH words, filtered temp[15:0]
  localparam int REG_FAULT_TO  = 'h100 >> 2; // fault bitmaps (W0: timeout, W1: range, W2: stuck)
  localparam int REG_DROP_LO   = 'h110 >> 2; // per-channel drop counters (first 32 chans)

  // Write path
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      cfg_enable_q      <= 1'b0;
      cfg_shutdown_en_q <= 1'b1;
      cfg_iir_shift_q   <= 4'(4);
      cfg_decim_log2_q  <= 4'd0;
      cfg_warn_th_q     <= TEMP_W'( (16'd85)  << QF ); // 85°C default
      cfg_crit_th_q     <= TEMP_W'( (16'd95)  << QF ); // 95°C default
      cfg_hyst_q        <= TEMP_W'( (16'd2)   << QF ); // 2°C hysteresis
      cfg_timeout_cycles_q <= '0;
      cfg_code_min_q    <= CODE_W'(0);
      cfg_code_max_q    <= CODE_W'((1<<CODE_W)-1);
      cfg_stuck_delta_q <= CODE_W'(0);
      cfg_stuck_win_q   <= '0;
      cfg_en_mask_q     <= {N_CH{1'b1}};
      cfg_irq_mask_q    <= '0;
      for (int i=0;i<N_CH;i++) begin
        cfg_gain_q[i] <= GAIN_W'(16'd32);  // 0.125°C/LSB * 256 = 32
        cfg_offs_q[i] <= '0;
      end
    end else if (csr_valid_i && csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_CTRL: begin
          if (csr_wstrb_i[0]) begin
            cfg_enable_q      <= csr_wdata_i[0];
            cfg_shutdown_en_q <= csr_wdata_i[1];
            cfg_iir_shift_q   <= csr_wdata_i[7:4];
            cfg_decim_log2_q  <= csr_wdata_i[11:8];
          end
        end
        REG_WARN_TH: if (&csr_wstrb_i) cfg_warn_th_q <= TEMP_W'(csr_wdata_i[23:0]);
        REG_CRIT_TH: if (&csr_wstrb_i) cfg_crit_th_q <= TEMP_W'(csr_wdata_i[23:0]);
        REG_HYST   : if (&csr_wstrb_i) cfg_hyst_q    <= TEMP_W'(csr_wdata_i[23:0]);
        REG_TIMEOUT: if (&csr_wstrb_i) cfg_timeout_cycles_q <= csr_wdata_i[TO_BITS-1:0];
        REG_CODE_MIN: begin
          if (&csr_wstrb_i) begin
            cfg_code_min_q <= csr_wdata_i[15:0];
            cfg_code_max_q <= csr_wdata_i[31:16];
          end
        end
        REG_STUCKCFG: begin
          if (&csr_wstrb_i) begin
            cfg_stuck_delta_q <= csr_wdata_i[15:0];
            cfg_stuck_win_q   <= csr_wdata_i[31:16];
          end
        end
        REG_ENMASK_LO: begin
          for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
            for (int k=0;k<8;k++) if ((b*8+k) < N_CH) cfg_en_mask_q[b*8+k] <= csr_wdata_i[b*8+k];
          end
        end
        REG_IRQMASK_LO: begin
          for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
            for (int k=0;k<8;k++) if ((b*8+k) < N_CH) cfg_irq_mask_q[b*8+k] <= csr_wdata_i[b*8+k];
          end
        end
        default: begin
          // Gains
          if ((csr_addr_i[CSR_AW-1:2] >= REG_GAIN_BASE) &&
              (csr_addr_i[CSR_AW-1:2] <  REG_GAIN_BASE + N_CH)) begin
            int i = csr_addr_i[CSR_AW-1:2] - REG_GAIN_BASE;
            if (i < N_CH && csr_wstrb_i[0]) cfg_gain_q[i][15:0] <= csr_wdata_i[15:0];
          end
          // Offsets
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_OFFS_BASE) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_OFFS_BASE + N_CH)) begin
            int i = csr_addr_i[CSR_AW-1:2] - REG_OFFS_BASE;
            if (i < N_CH) begin
              for (int b=0;b<3;b++) if (csr_wstrb_i[b]) cfg_offs_q[i][b*8 +: 8] <= csr_wdata_i[b*8 +: 8];
            end
          end
        end
      endcase
    end
  end

  // Read path
  always_comb begin
    csr_rdata_o = 32'h0;
    if (csr_valid_i && !csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_ID     : csr_rdata_o = ID_VERSION;
        REG_CTRL   : csr_rdata_o = { 20'd0, cfg_decim_log2_q, cfg_iir_shift_q, cfg_shutdown_en_q, cfg_enable_q };
        REG_STATUS : csr_rdata_o = { hot_temp_o[15:0], 8'(hot_idx_o), 6'd0, crit_any_w, warn_any_w };
        REG_WARN_TH: csr_rdata_o = { 8'd0, cfg_warn_th_q[23:0] };
        REG_CRIT_TH: csr_rdata_o = { 8'd0, cfg_crit_th_q[23:0] };
        REG_HYST   : csr_rdata_o = { 8'd0, cfg_hyst_q[23:0] };
        REG_TIMEOUT: csr_rdata_o = { {(32-TO_BITS){1'b0}}, cfg_timeout_cycles_q };
        REG_CODE_MIN:csr_rdata_o = { cfg_code_max_q, cfg_code_min_q };
        REG_STUCKCFG:csr_rdata_o = { cfg_stuck_win_q, cfg_stuck_delta_q };
        REG_ENMASK_LO:csr_rdata_o= { {(32-N_CH){1'b0}}, cfg_en_mask_q };
        REG_IRQMASK_LO:csr_rdata_o={ {(32-N_CH){1'b0}}, cfg_irq_mask_q };
        default: begin
          if ((csr_addr_i[CSR_AW-1:2] >= REG_GAIN_BASE) &&
              (csr_addr_i[CSR_AW-1:2] <  REG_GAIN_BASE + N_CH)) begin
            int i = csr_addr_i[CSR_AW-1:2] - REG_GAIN_BASE;
            csr_rdata_o = { 16'd0, cfg_gain_q[i][15:0] };
          end
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_OFFS_BASE) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_OFFS_BASE + N_CH)) begin
            int i = csr_addr_i[CSR_AW-1:2] - REG_OFFS_BASE;
            csr_rdata_o = { 8'd0, cfg_offs_q[i][23:0] };
          end
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_TEMP_FILT) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_TEMP_FILT + N_CH)) begin
            int i = csr_addr_i[CSR_AW-1:2] - REG_TEMP_FILT;
            csr_rdata_o = { 16'd0, temp_filt_q[i][15:0] };
          end
          else if (csr_addr_i[CSR_AW-1:2] == REG_FAULT_TO) begin
            csr_rdata_o = { {(32-N_CH){1'b0}}, st_fault_to_q };
          end
          else if (csr_addr_i[CSR_AW-1:2] == (REG_FAULT_TO+1)) begin
            csr_rdata_o = { {(32-N_CH){1'b0}}, st_fault_rng_q };
          end
          else if (csr_addr_i[CSR_AW-1:2] == (REG_FAULT_TO+2)) begin
            csr_rdata_o = { {(32-N_CH){1'b0}}, st_fault_stk_q };
          end
          else if (csr_addr_i[CSR_AW-1:2] == REG_DROP_LO) begin
            // low 16 channels drop counters (lower 16 bits)
            logic [31:0] v; v='0;
            for (int k=0;k<16 && k<N_CH;k++) v[k] = |sns_drop_cnt_q[k];
            csr_rdata_o = v;
          end
          else csr_rdata_o = 32'hBADC_AB1E;
        end
      endcase
    end
  end

  // ==========================================================================
  // Sequential state updates (clk_i)
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      // clear per-channel
      for (int i=0;i<N_CH;i++) begin
        last_code_q[i]   <= '0;
        stuck_ctr_q[i]   <= '0;
        age_ctr_q[i]     <= '0;
        temp_raw_q[i]    <= '0;
        temp_filt_q[i]   <= '0;
        st_fault_to_q[i] <= 1'b0;
        st_fault_rng_q[i]<= 1'b0;
        st_fault_stk_q[i]<= 1'b0;
        st_warn_q[i]     <= 1'b0;
        st_crit_q[i]     <= 1'b0;
      end
    end else begin
      for (int i=0;i<N_CH;i++) begin
        last_code_q[i]   <= last_code_d[i];
        stuck_ctr_q[i]   <= stuck_ctr_d[i];
        age_ctr_q[i]     <= age_ctr_d[i];
        temp_raw_q[i]    <= temp_raw_d[i];
        temp_filt_q[i]   <= temp_filt_d[i];
        st_fault_to_q[i] <= st_fault_to_d[i];
        st_fault_rng_q[i]<= st_fault_rng_d[i];
        st_fault_stk_q[i]<= st_fault_stk_d[i];
        st_warn_q[i]     <= st_warn_d[i];
        st_crit_q[i]     <= st_crit_d[i];
      end
    end
  end

`ifdef ASSERT_ON
  initial begin
    if (TEMP_W < (QF+12)) $error("thermal_sense_if: TEMP_W too small for QF.");
  end
`endif

endmodule

`default_nettype wire
