// ============================================================================
//  lbist_ctrl.sv
//  Aquila — Logic BIST (LBIST) Controller (PRPG + Phase Shifter + MISR)
// ----------------------------------------------------------------------------
//  Architecture (STUMPS style, non-overlapped for simplicity & robustness)
//   Pattern loop (repeat N patterns):
//     1) LOAD_SHIFT:   scan_en=1, shift SCAN_LEN bits of PRPG→phase-shifter→scan_si_o[*]
//                      (MISR is gated OFF in this phase)
//     2) CAPTURE_REQ:  scan_en=0, request 1 or 2 functional capture pulses via cap_req_o
//                      wait for cap_done_i (watchdog protected)
//     3) RESP_SHIFT:   scan_en=1, shift SCAN_LEN bits out of scan_so_i[*] into MISR
//   After last RESP_SHIFT: compare MISR to golden signature → PASS/FAIL
//
//  Interfaces
//  ----------
//  clk_i, rstn_i            : LBIST clock domain (should be routed as SCAN CLK)
//  scan_en_o                : drives scan enable for all scan FFs (active high)
//  scan_si_o[NUM_CHAINS-1:0]: LBIST scan-in bits per chain (one bit per shift)
//  scan_so_i[NUM_CHAINS-1:0]: scan-out feedback from scan chains
//  xmask_hw_i               : hardware-provided X-mask (1=mask that SO bit)
//  cap_req_o                : one-cycle request to functional capture clock generator
//  cap_two_pulses_o         : 0=single-capture, 1=two-pulse at-speed capture
//  cap_done_i               : one-cycle ack when capture pulses have been applied
//
//  CSR-Lite window (byte addresses, single-cycle ready)
//  ---------------------------------------------------
//   0x000 ID_VERSION          RO  {'L','B', maj:8=1, min:8=0}
//   0x004 CTRL                RW  [0]=start(W1P) [1]=abort(W1P) [2]=clr_fail(W1P)
//                                 [8]=auto_loop  [9]=stop_on_timeout  [10]=cap_two_pulses
//                                 [11]=xmask_en
//   0x008 STATUS              RO  [0]=busy [1]=done_pulse [2]=pass [3]=fail_sticky
//                                 [4]=aborted [5]=running
//                                 [31:16]=shift_progress (low 16b)
//   0x00C CFG0                RW  [15:0]=scan_len    [31:16]=num_patterns
//   0x010 SEED_SEL            RW  [0]=use_seed_reg (1) else default nonzero
//   0x020.. PRPG_SEED[k]      RW  PRPG_W in 32b words (k=0..(PRPG_W+31)/32-1)
//   0x040.. GOLDEN_SIG[k]     RW  MISR_W in 32b words
//   0x060.. CUR_SIGNATURE[k]  RO  MISR current value in 32b words
//   0x080.. XMASK[k]          RW  NUM_CHAINS bits in 32b words (ORed with xmask_hw_i)
//
//  Events (to err_logger)
//  ----------------------
//   - Signature mismatch: code 0x210, info = {pattern_cnt[15:0], 16'hDEAD}
//   - Capture timeout   : code 0x211, info = {pattern_idx[15:0], 16'hBEEF}
//
//  Notes
//  -----
//  - Non-overlapped flow keeps timing simple and deterministic for ASIC bring-up.
//  - PRPG & MISR are Galois form with configurable polynomials.
//  - For very large NUM_CHAINS, input compaction groups scan_so_i into INJ_W slices.
//  - Seed of all-zeros is auto-fixed to 1 for PRPG/MISR to avoid lock-up.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module lbist_ctrl #(
  // ----------------------------- Geometry -----------------------------------
  parameter int unsigned NUM_CHAINS  = 16,          // parallel scan chains
  parameter int unsigned SCANLEN_MAX = 65535,       // CSR-bounded
  parameter int unsigned PRPG_W      = 64,          // PRPG LFSR width
  parameter int unsigned MISR_W      = 32,          // MISR width
  // Compaction: inject up to INJ_W bits per shift into MISR (1..MISR_W)
  parameter int unsigned INJ_W       = (NUM_CHAINS < MISR_W) ? NUM_CHAINS : MISR_W,

  // ----------------------------- Polynomials --------------------------------
  // Galois form polynomials (excluding the x^N implicit term). Example defaults:
  //  - PRPG: x^64 + x^4 + x^3 + x + 1   => 64'h_0000_0000_0000_001B
  //  - MISR: CRC-32 (IEEE) polynomial   => 32'h_04C11DB7
  parameter logic [PRPG_W-1:0] PRPG_POLY = (PRPG_W==64) ? 64'h0000_0000_0000_001B : {{(PRPG_W-1){1'b0}},1'b1},
  parameter logic [MISR_W-1:0] MISR_POLY = (MISR_W==32) ? 32'h04C11DB7            : {{(MISR_W-1){1'b0}},1'b1},

  // ----------------------------- CSR window ---------------------------------
  parameter int unsigned CSR_AW     = 12,           // 4 KiB window
  parameter logic [7:0]   EVT_SRC_ID= 8'h4C,        // 'L' for LBIST
  // Watchdog for capture handshake
  parameter int unsigned WDOG_BITS  = 24,
  parameter logic [WDOG_BITS-1:0] CAP_WDOG_MAX = 24'd8_000_000 // ~80ms @100MHz
)(
  input  logic                          clk_i,
  input  logic                          rstn_i,        // synchronous, active-low

  // ============================ Scan interface ===============================
  output logic                          scan_en_o,     // high during SHIFT phases
  output logic [NUM_CHAINS-1:0]         scan_si_o,     // scan-in
  input  logic [NUM_CHAINS-1:0]         scan_so_i,     // scan-out
  input  logic [NUM_CHAINS-1:0]         xmask_hw_i,    // X-mask from HW (1=mask/ignore)

  // ============================ Capture handshake ============================
  output logic                          cap_req_o,     // pulse to request capture
  output logic                          cap_two_pulses_o, // 0=single, 1=two pulses
  input  logic                          cap_done_i,    // one-cycle ack when capture pulses done

  // ============================ CSR-Lite ====================================
  input  logic                          csr_valid_i,
  input  logic                          csr_write_i,
  input  logic [CSR_AW-1:0]             csr_addr_i,    // byte address
  input  logic [31:0]                   csr_wdata_i,
  input  logic [3:0]                    csr_wstrb_i,
  output logic                          csr_ready_o,
  output logic [31:0]                   csr_rdata_o,

  // ============================ Event (err_logger) ===========================
  output logic                          evt_valid_o,
  input  logic                          evt_ready_i,
  output logic [7:0]                    evt_src_o,
  output logic [1:0]                    evt_sev_o,     // 2=ERR, 3=FATAL (unused here)
  output logic [11:0]                   evt_code_o,
  output logic [31:0]                   evt_info_o
);

  // ==========================================================================
  // Constants / CSR map
  // ==========================================================================
  localparam [31:0] ID_VERSION = {8'h4C, 8'h42, 8'd1, 8'd0}; // 'L''B' v1.0
  localparam int REG_ID     = 'h000 >> 2;
  localparam int REG_CTRL   = 'h004 >> 2;
  localparam int REG_STATUS = 'h008 >> 2;
  localparam int REG_CFG0   = 'h00C >> 2;
  localparam int REG_SEEDSEL= 'h010 >> 2;

  localparam int REG_PRPG_SEED_BASE = 'h020 >> 2;
  localparam int REG_SIG_GOLD_BASE  = 'h040 >> 2;
  localparam int REG_SIG_NOW_BASE   = 'h060 >> 2;
  localparam int REG_XMASK_BASE     = 'h080 >> 2;

  localparam int PRPG_WORDS = (PRPG_W + 31) / 32;
  localparam int MISR_WORDS = (MISR_W + 31) / 32;
  localparam int XMSK_WORDS = (NUM_CHAINS + 31) / 32;

  // Event codes
  localparam logic [11:0] EV_LBIST_FAILSIG = 12'h210;
  localparam logic [11:0] EV_LBIST_TIMEOUT = 12'h211;

  // CTRL bits
  //  [0]=start (W1P)  [1]=abort (W1P)  [2]=clr_fail (W1P)
  //  [8]=auto_loop     [9]=stop_on_timeout
  //  [10]=cap_two_pulses [11]=xmask_en
  // STATUS bits
  //  [0]=busy [1]=done_pulse [2]=pass [3]=fail_sticky [4]=aborted [5]=running
  //  [31:16]=shift_progress

  // ==========================================================================
  // Registers & runtime state
  // ==========================================================================
  // Config / control
  logic        cfg_auto_loop_q,    cfg_auto_loop_d;
  logic        cfg_stop_on_to_q,   cfg_stop_on_to_d;
  logic        cfg_cap_two_q,      cfg_cap_two_d;
  logic        cfg_xmask_en_q,     cfg_xmask_en_d;

  logic [15:0] cfg_scan_len_q,     cfg_scan_len_d;     // 1..SCANLEN_MAX
  logic [15:0] cfg_num_patterns_q, cfg_num_patterns_d; // 1..65535
  logic        cfg_use_seed_q,     cfg_use_seed_d;     // 1=use PRPG_SEED regs

  // Seeds & signatures
  logic [PRPG_W-1:0] prpg_seed_q, prpg_seed_d;
  logic [MISR_W-1:0] sig_golden_q, sig_golden_d;

  // CSR X-mask (ORed with HW xmask)
  logic [NUM_CHAINS-1:0] xmask_csr_q, xmask_csr_d;

  // Sticky fail & status
  logic fail_sticky_q, fail_sticky_d;
  logic aborted_q,     aborted_d;
  logic pass_q,        pass_d;
  logic busy_q,        busy_d;
  logic done_pulse_q,  done_pulse_d;

  // Progress / telemetry
  logic [15:0] shift_progress_q, shift_progress_d; // for STATUS
  logic [15:0] pat_idx_q, pat_idx_d;

  // Core LBIST engines
  logic [PRPG_W-1:0] prpg_q, prpg_d;
  logic [MISR_W-1:0] misr_q, misr_d;

  // FSM
  typedef enum logic [2:0] {S_IDLE, S_INIT, S_LOAD_SHIFT, S_CAP_REQ, S_CAP_WAIT, S_RESP_SHIFT, S_FINAL, S_DONE} state_e;
  state_e st_q, st_d;

  // Handshakes
  logic cap_req_d, cap_req_q;
  logic cap_done_sync;  // assume synchronous; if not, pulse sync externally

  // Watchdog
  logic [WDOG_BITS-1:0] wdog_q, wdog_d;

  // ==========================================================================
  // Utility: Galois LFSR/MISR step functions (left shift)
  // ==========================================================================
  function automatic logic [PRPG_W-1:0] prpg_step(input logic [PRPG_W-1:0] s);
    logic fb;
    logic [PRPG_W-1:0] nxt;
    fb  = s[PRPG_W-1];
    nxt = {s[PRPG_W-2:0], 1'b0};
    if (fb) nxt ^= PRPG_POLY;
    // All-zero lock-up guard
    if (nxt == '0) nxt = {{(PRPG_W-1){1'b0}},1'b1};
    return nxt;
  endfunction

  // MISR step with external input bit 'din' (Galois, left shift)
  function automatic logic [MISR_W-1:0] misr_step(input logic [MISR_W-1:0] s, input logic din);
    logic fb;
    logic [MISR_W-1:0] nxt;
    fb  = s[MISR_W-1] ^ din;
    nxt = {s[MISR_W-2:0], 1'b0};
    if (fb) nxt ^= MISR_POLY;
    // Avoid all-zero (optional; not strictly needed). Keep as-is for signature space.
    return nxt;
  endfunction

  // Reduce scan_so_i[*] into INJ_W groups by modulo bucketing, apply X-mask
  function automatic logic [INJ_W-1:0]
    compact_inj(input logic [NUM_CHAINS-1:0] so, input logic [NUM_CHAINS-1:0] xmask);
    logic [INJ_W-1:0] r; r = '0;
    for (int i=0;i<NUM_CHAINS;i++) begin
      if (!xmask[i]) r[i % INJ_W] ^= so[i];
    end
    return r;
  endfunction

  // Phase shifter: decorrelate PRPG bits to per-chain scan-in
  function automatic logic [NUM_CHAINS-1:0]
    phase_shifter(input logic [PRPG_W-1:0] prpg);
    logic [NUM_CHAINS-1:0] s;
    for (int i=0;i<NUM_CHAINS;i++) begin
      // 4-tap XOR with rotated taps; simple, synthesizable, decent dispersion
      logic b0 = prpg[(i       ) % PRPG_W];
      logic b1 = prpg[(i +  7  ) % PRPG_W];
      logic b2 = prpg[(i + 13  ) % PRPG_W];
      logic b3 = prpg[(i + 23  ) % PRPG_W];
      s[i] = b0 ^ b1 ^ b2 ^ b3;
    end
    return s;
  endfunction

  // ==========================================================================
  // CSR front-end
  // ==========================================================================
  assign csr_ready_o = csr_valid_i;

  // Write path
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      cfg_auto_loop_q   <= 1'b0;
      cfg_stop_on_to_q  <= 1'b1;
      cfg_cap_two_q     <= 1'b0;
      cfg_xmask_en_q    <= 1'b0;
      cfg_scan_len_q    <= 16'd1024;
      cfg_num_patterns_q<= 16'd256;
      cfg_use_seed_q    <= 1'b0;

      prpg_seed_q       <= {{(PRPG_W-1){1'b0}},1'b1};
      sig_golden_q      <= '0;
      xmask_csr_q       <= '0;

      fail_sticky_q     <= 1'b0;
      aborted_q         <= 1'b0;

    end else if (csr_valid_i && csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_CTRL: begin
          if (csr_wstrb_i[0]) begin
            // W1P fields handled below (in FSM): start/abort/clr_fail
          end
          if (csr_wstrb_i[1]) begin
            cfg_auto_loop_q  <= csr_wdata_i[8];
            cfg_stop_on_to_q <= csr_wdata_i[9];
            cfg_cap_two_q    <= csr_wdata_i[10];
            cfg_xmask_en_q   <= csr_wdata_i[11];
          end
        end
        REG_CFG0: begin
          if (&csr_wstrb_i) begin
            cfg_scan_len_q     <= (csr_wdata_i[15:0]  == 16'd0) ? 16'd1  :
                                  (csr_wdata_i[15:0]  >  SCANLEN_MAX) ? SCANLEN_MAX[15:0] :
                                  csr_wdata_i[15:0];
            cfg_num_patterns_q <= (csr_wdata_i[31:16] == 16'd0) ? 16'd1  : csr_wdata_i[31:16];
          end
        end
        REG_SEEDSEL: begin
          if (csr_wstrb_i[0]) cfg_use_seed_q <= csr_wdata_i[0];
        end
        default: begin
          // PRPG seed
          if ((csr_addr_i[CSR_AW-1:2] >= REG_PRPG_SEED_BASE) &&
              (csr_addr_i[CSR_AW-1:2] <  REG_PRPG_SEED_BASE + PRPG_WORDS)) begin
            int w = csr_addr_i[CSR_AW-1:2] - REG_PRPG_SEED_BASE;
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
              int lo = w*32 + b*8;
              for (int k=0;k<8;k++) if (lo+k < PRPG_W) prpg_seed_q[lo+k] <= csr_wdata_i[b*8+k];
            end
          end
          // Golden signature
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_SIG_GOLD_BASE) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_SIG_GOLD_BASE + MISR_WORDS)) begin
            int w = csr_addr_i[CSR_AW-1:2] - REG_SIG_GOLD_BASE;
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
              int lo = w*32 + b*8;
              for (int k=0;k<8;k++) if (lo+k < MISR_W) sig_golden_q[lo+k] <= csr_wdata_i[b*8+k];
            end
          end
          // X-mask CSR
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_XMASK_BASE) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_XMASK_BASE + XMSK_WORDS)) begin
            int w = csr_addr_i[CSR_AW-1:2] - REG_XMASK_BASE;
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
              int lo = w*32 + b*8;
              for (int k=0;k<8;k++) if (lo+k < NUM_CHAINS) xmask_csr_q[lo+k] <= csr_wdata_i[b*8+k];
            end
          end
        end
      endcase
      // W1P control bits
      if (csr_valid_i && csr_write_i && csr_addr_i[CSR_AW-1:2]==REG_CTRL && csr_wstrb_i[0]) begin
        if (csr_wdata_i[2]) fail_sticky_q <= 1'b0; // clr_fail
      end
    end
  end

  // Read path
  always_comb begin
    csr_rdata_o = 32'h0;
    if (csr_valid_i && !csr_write_i) begin
      unique case (csr_addr_i[CSR_AW-1:2])
        REG_ID:       csr_rdata_o = ID_VERSION;
        REG_CTRL:     csr_rdata_o = {20'd0, cfg_xmask_en_q, cfg_cap_two_q, cfg_stop_on_to_q,
                                     cfg_auto_loop_q, 5'd0};
        REG_STATUS:   csr_rdata_o = { shift_progress_q, 10'd0, busy_q, 1'b0/*running*/,
                                      aborted_q, fail_sticky_q, pass_q, done_pulse_q, busy_q };
        REG_CFG0:     csr_rdata_o = { cfg_num_patterns_q, cfg_scan_len_q };
        REG_SEEDSEL:  csr_rdata_o = {31'd0, cfg_use_seed_q};
        default: begin
          if ((csr_addr_i[CSR_AW-1:2] >= REG_PRPG_SEED_BASE) &&
              (csr_addr_i[CSR_AW-1:2] <  REG_PRPG_SEED_BASE + PRPG_WORDS)) begin
            int w = csr_addr_i[CSR_AW-1:2] - REG_PRPG_SEED_BASE;
            int lo = w*32;
            logic [31:0] tmp; tmp = 32'd0;
            for (int k=0;k<32;k++) if (lo+k < PRPG_W) tmp[k] = prpg_seed_q[lo+k];
            csr_rdata_o = tmp;
          end
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_SIG_GOLD_BASE) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_SIG_GOLD_BASE + MISR_WORDS)) begin
            int w = csr_addr_i[CSR_AW-1:2] - REG_SIG_GOLD_BASE;
            int lo = w*32;
            logic [31:0] tmp; tmp = 32'd0;
            for (int k=0;k<32;k++) if (lo+k < MISR_W) tmp[k] = sig_golden_q[lo+k];
            csr_rdata_o = tmp;
          end
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_SIG_NOW_BASE) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_SIG_NOW_BASE + MISR_WORDS)) begin
            int w = csr_addr_i[CSR_AW-1:2] - REG_SIG_NOW_BASE;
            int lo = w*32;
            logic [31:0] tmp; tmp = 32'd0;
            for (int k=0;k<32;k++) if (lo+k < MISR_W) tmp[k] = misr_q[lo+k];
            csr_rdata_o = tmp;
          end
          else if ((csr_addr_i[CSR_AW-1:2] >= REG_XMASK_BASE) &&
                   (csr_addr_i[CSR_AW-1:2] <  REG_XMASK_BASE + XMSK_WORDS)) begin
            int w = csr_addr_i[CSR_AW-1:2] - REG_XMASK_BASE;
            int lo = w*32;
            logic [31:0] tmp; tmp = 32'd0;
            for (int k=0;k<32;k++) if (lo+k < NUM_CHAINS) tmp[k] = xmask_csr_q[lo+k];
            csr_rdata_o = tmp;
          end
          else csr_rdata_o = 32'hBADC_AB1E;
        end
      endcase
    end
  end

  // ==========================================================================
  // FSM & datapath
  // ==========================================================================
  // Defaults
  assign evt_src_o = EVT_SRC_ID;
  assign evt_sev_o = 2'd2; // ERR

  // Start/abort decoding (W1P)
  wire start_w = csr_valid_i && csr_write_i && csr_addr_i[CSR_AW-1:2]==REG_CTRL &&
                 csr_wstrb_i[0] && csr_wdata_i[0];
  wire abort_w = csr_valid_i && csr_write_i && csr_addr_i[CSR_AW-1:2]==REG_CTRL &&
                 csr_wstrb_i[0] && csr_wdata_i[1];

  // Combine CSR & HW masks
  wire [NUM_CHAINS-1:0] xmask_eff = cfg_xmask_en_q ? (xmask_hw_i | xmask_csr_q) : xmask_hw_i;

  // PRPG→phase shifter→scan_si_o
  logic [NUM_CHAINS-1:0] si_vec_w;
  assign si_vec_w = phase_shifter(prpg_q);

  // MISR injection vector from scan_so_i
  logic [INJ_W-1:0] inj_bits_w;
  assign inj_bits_w = compact_inj(scan_so_i, xmask_eff);

  // Shift counter
  logic [15:0] shift_cnt_q, shift_cnt_d;

  // Capture request / handshake
  assign cap_two_pulses_o = cfg_cap_two_q;

  // scan_en high during shift phases
  assign scan_en_o = (st_q == S_LOAD_SHIFT) || (st_q == S_RESP_SHIFT);
  assign scan_si_o = si_vec_w;

  // Event defaults
  always_comb begin
    evt_valid_o = 1'b0;
    evt_code_o  = 12'h000;
    evt_info_o  = 32'h0;
  end

  // Next-state
  always_comb begin
    // hold config registers unless CSR writes (done earlier)
    cfg_auto_loop_d   = cfg_auto_loop_q;
    cfg_stop_on_to_d  = cfg_stop_on_to_q;
    cfg_cap_two_d     = cfg_cap_two_q;
    cfg_xmask_en_d    = cfg_xmask_en_q;
    cfg_scan_len_d    = cfg_scan_len_q;
    cfg_num_patterns_d= cfg_num_patterns_q;
    cfg_use_seed_d    = cfg_use_seed_q;
    prpg_seed_d       = prpg_seed_q;
    sig_golden_d      = sig_golden_q;
    xmask_csr_d       = xmask_csr_q;

    // defaults
    st_d            = st_q;
    busy_d          = busy_q;
    pass_d          = pass_q;
    aborted_d       = aborted_q;
    fail_sticky_d   = fail_sticky_q;
    done_pulse_d    = 1'b0;

    shift_cnt_d       = shift_cnt_q;
    shift_progress_d  = shift_progress_q;
    pat_idx_d         = pat_idx_q;

    prpg_d            = prpg_q;
    misr_d            = misr_q;

    cap_req_d         = 1'b0;
    wdog_d            = (st_q == S_CAP_WAIT) ? (wdog_q + {{(WDOG_BITS-1){1'b0}},1'b1}) : '0;

    // ----------------- IDLE -----------------
    if (st_q == S_IDLE) begin
      if (start_w && !busy_q) begin
        busy_d        = 1'b1;
        pass_d        = 1'b0;
        aborted_d     = 1'b0;
        fail_sticky_d = 1'b0;
        done_pulse_d  = 1'b0;

        // Initialize seeds and signature
        prpg_d        = cfg_use_seed_q ? (prpg_seed_q=='0 ? {{(PRPG_W-1){1'b0}},1'b1} : prpg_seed_q)
                                       : {{(PRPG_W-1){1'b0}},1'b1};
        misr_d        = {{(MISR_W-1){1'b0}},1'b1}; // nonzero init recommended

        shift_cnt_d      = 16'd0;
        shift_progress_d = 16'd0;
        pat_idx_d        = 16'd0;

        st_d = S_INIT;
      end
    end

    // ----------------- INIT -----------------
    else if (st_q == S_INIT) begin
      // Move to first load shift
      st_d = S_LOAD_SHIFT;
    end

    // ----------------- LOAD_SHIFT -----------------
    else if (st_q == S_LOAD_SHIFT) begin
      // Present SI bits; update PRPG each shift
      prpg_d       = prpg_step(prpg_q);
      shift_cnt_d  = shift_cnt_q + 16'd1;
      shift_progress_d = shift_cnt_d;

      if (shift_cnt_q + 16'd1 >= cfg_scan_len_q) begin
        shift_cnt_d = 16'd0;
        st_d        = S_CAP_REQ;
      end
    end

    // ----------------- CAPTURE_REQ -----------------
    else if (st_q == S_CAP_REQ) begin
      cap_req_d = 1'b1;   // single-cycle pulse
      st_d      = S_CAP_WAIT;
    end

    // ----------------- CAPTURE_WAIT -----------------
    else if (st_q == S_CAP_WAIT) begin
      if (cap_done_i) begin
        wdog_d = '0;
        st_d   = S_RESP_SHIFT;
      end else if (wdog_q >= CAP_WDOG_MAX) begin
        // Timeout
        fail_sticky_d = 1'b1;
        pass_d        = 1'b0;
        busy_d        = cfg_auto_loop_q;  // auto-loop? then restart at IDLE start; else go DONE
        done_pulse_d  = 1'b1;
        aborted_d     = 1'b0;
        st_d          = cfg_auto_loop_q ? S_IDLE : S_DONE;

        evt_valid_o   = 1'b1;
        evt_code_o    = EV_LBIST_TIMEOUT;
        evt_info_o    = { pat_idx_q, 16'hBEEF };
      end
    end

    // ----------------- RESP_SHIFT -----------------
    else if (st_q == S_RESP_SHIFT) begin
      // Update MISR with compacted injection bits (INJ_W-bit serial fold)
      logic [MISR_W-1:0] m; m = misr_q;
      for (int k=0;k<INJ_W;k++) m = misr_step(m, inj_bits_w[k]);
      misr_d = m;

      shift_cnt_d       = shift_cnt_q + 16'd1;
      shift_progress_d  = shift_cnt_d;

      if (shift_cnt_q + 16'd1 >= cfg_scan_len_q) begin
        shift_cnt_d = 16'd0;
        // Next pattern or finalize
        if (pat_idx_q + 16'd1 >= cfg_num_patterns_q) begin
          st_d = S_FINAL;
        end else begin
          pat_idx_d = pat_idx_q + 16'd1;
          st_d      = S_LOAD_SHIFT;
        end
      end
    end

    // ----------------- FINAL -----------------
    else if (st_q == S_FINAL) begin
      // Compare MISR to golden signature
      if (misr_q === sig_golden_q) begin
        pass_d       = 1'b1;
        busy_d       = cfg_auto_loop_q;
        done_pulse_d = 1'b1;
        st_d         = cfg_auto_loop_q ? S_IDLE : S_DONE;
      end else begin
        pass_d        = 1'b0;
        fail_sticky_d = 1'b1;
        busy_d        = cfg_auto_loop_q;
        done_pulse_d  = 1'b1;
        st_d          = cfg_auto_loop_q ? S_IDLE : S_DONE;

        evt_valid_o   = 1'b1;
        evt_code_o    = EV_LBIST_FAILSIG;
        evt_info_o    = { pat_idx_q, 16'hDEAD };
      end
    end

    // ----------------- DONE -----------------
    else if (st_q == S_DONE) begin
      // Wait for next start
      if (start_w) st_d = S_INIT;
    end

    // Abort request at any time
    if (abort_w) begin
      st_d         = S_DONE;
      busy_d       = 1'b0;
      aborted_d    = 1'b1;
      done_pulse_d = 1'b1;
    end
  end

  // ==========================================================================
  // Sequential regs
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      st_q            <= S_IDLE;
      busy_q          <= 1'b0; pass_q <= 1'b0; aborted_q <= 1'b0; fail_sticky_q <= 1'b0;
      done_pulse_q    <= 1'b0;
      shift_cnt_q     <= 16'd0; shift_progress_q <= 16'd0; pat_idx_q <= 16'd0;
      prpg_q          <= {{(PRPG_W-1){1'b0}},1'b1};
      misr_q          <= {{(MISR_W-1){1'b0}},1'b1};
      cap_req_q       <= 1'b0;
      wdog_q          <= '0;
    end else begin
      st_q            <= st_d;
      busy_q          <= busy_d;
      pass_q          <= pass_d;
      aborted_q       <= aborted_d;
      fail_sticky_q   <= fail_sticky_d;
      done_pulse_q    <= done_pulse_d;

      shift_cnt_q     <= shift_cnt_d;
      shift_progress_q<= shift_progress_d;
      pat_idx_q       <= pat_idx_d;

      prpg_q          <= prpg_d;
      misr_q          <= misr_d;

      cap_req_q       <= cap_req_d;
      wdog_q          <= wdog_d;
    end
  end

  assign cap_req_o = cap_req_q;

  // ==========================================================================
  // Assertions / Guards
  // ==========================================================================
`ifdef ASSERT_ON
  initial begin
    if (INJ_W < 1 || INJ_W > MISR_W) $error("lbist_ctrl: INJ_W out of range.");
    if (NUM_CHAINS == 0)             $error("lbist_ctrl: NUM_CHAINS must be > 0.");
  end

  // shift length must be non-zero
  always_ff @(posedge clk_i) begin
    if (st_q == S_INIT) begin
      assert (cfg_scan_len_q != 16'd0) else $error("lbist_ctrl: scan_len==0.");
      assert (cfg_num_patterns_q != 16'd0) else $error("lbist_ctrl: num_patterns==0.");
    end
  end

  // scan_en only during shift states
  assert property (@(posedge clk_i) scan_en_o |-> (st_q==S_LOAD_SHIFT || st_q==S_RESP_SHIFT))
    else $error("lbist_ctrl: scan_en asserted outside SHIFT.");

`endif

endmodule

`default_nettype wire
