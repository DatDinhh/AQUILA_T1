// ============================================================================
//  Global Buffer Scrubber (GB ECC scrubber master)
//  File: gb_scrubber.sv
//
//  Purpose:
//    - Walk all enabled banks and words (beat-granular), issue READs.
//    - If READ returns with SBE (single-bit corrected), immediately WRITE back
//      the corrected data to scrub the cell (refresh ECC).
//    - Count SBEs/DBEs per bank and globally. Optional one-pass completion.
//    - Low-priority: programmable issue rate; at most 1 outstanding read.
//    - Master-side interface matches gb_crossbar master port conventions.
//
//  Key design choices:
//    - MAX_OUTSTANDING = 1 keeps pairing of read response to last request trivial,
//      avoiding tags or address FIFOs even when fabric return order is interleaved.
//    - Round-robin bank scheduler; per-bank word pointer (wraps modulo configured words).
//    - Address composer inserts bank-index bits at BANK_SEL_LSB to form a global address.
//      Crossbar then routes to the correct bank and removes those bits for bank-local.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module gb_scrubber #(
  // ---------- Geometry ----------
  parameter int unsigned NBANK        = 8,     // power-of-two
  parameter int unsigned DATA_W       = 512,   // bits, multiple of 8
  parameter int unsigned ADDR_W       = 32,    // bytes address width

  // Bank slice convention (must match gb_crossbar)
  parameter int unsigned BANK_SEL_LSB = (DATA_W<=8)?0:$clog2(DATA_W/8),

  // ---------- Control ----------
  // Minimal cycles between issued READ commands (0 = as fast as possible)
  parameter int unsigned RATE_DIV_W   = 16,

  // ---------- Safety ----------
  // Sanity guard to keep design single-outstanding (required with untaged fabric)
  parameter int unsigned MAX_OUTSTANDING = 1
)(
  input  logic                      clk_i,
  input  logic                      rstn_i,          // synchronous, active-low

  // ==========================================================================
  // Master-side interface toward gb_crossbar (single master)
  // ==========================================================================
  // Write request (single-beat)
  output logic                      wr_req_valid_o,
  input  logic                      wr_req_ready_i,
  output logic [ADDR_W-1:0]         wr_addr_o,
  output logic [DATA_W-1:0]         wr_data_o,
  output logic [DATA_W/8-1:0]       wr_strb_o,

  // Read request (single-beat)
  output logic                      rd_req_valid_o,
  input  logic                      rd_req_ready_i,
  output logic [ADDR_W-1:0]         rd_addr_o,

  // Read response (from crossbar → banks)
  input  logic                      rd_rsp_valid_i,
  output logic                      rd_rsp_ready_o,
  input  logic [DATA_W-1:0]         rd_rsp_data_i,
  input  logic                      rd_rsp_sbe_i,    // single-bit corrected
  input  logic                      rd_rsp_dbe_i,    // double-bit detected

  // ==========================================================================
  // Configuration & control (typically from CSR block)
  // ==========================================================================
  input  logic                      cfg_enable_i,       // 1 = scrubber active
  input  logic                      cfg_continuous_i,   // 1 = wrap and continue; 0 = one pass then done
  input  logic [NBANK-1:0]          cfg_bank_mask_i,    // banks to include in scan
  // Per-bank length (words / beats). Must be >0 if bank is enabled in the mask.
  input  logic [NBANK-1:0][31:0]    cfg_bank_words_i,
  // Optional global base address offset in bytes (OR'ed/added after composing)
  input  logic [ADDR_W-1:0]         cfg_global_base_i,
  // Rate limiter: issue one READ every (cfg_rate_div_i + 1) cycles
  input  logic [RATE_DIV_W-1:0]     cfg_rate_div_i,
  // Enable writeback on SBE (normally 1). If 0, only count SBE (no scrub write).
  input  logic                      cfg_writeback_en_i,
  // Manual control
  input  logic                      cmd_start_i,        // pulse: reset pointers & start
  input  logic                      cmd_pause_i,        // level: pause issue (keeps state)
  input  logic                      cmd_abort_i,        // pulse: abort current op; go idle

  // ==========================================================================
  // Status / telemetry
  // ==========================================================================
  output logic                      active_o,           // high when issuing/awaiting completion
  output logic                      done_o,             // one-pass complete (sticky until start)
  output logic [NBANK-1:0]          bank_wrap_pulse_o,  // pulse when a bank pointer wraps
  output logic [31:0]               rd_issued_o,
  output logic [31:0]               rd_completed_o,
  output logic [31:0]               wr_issued_o,
  output logic [31:0]               sbe_count_o,        // total
  output logic [31:0]               dbe_count_o,        // total
  output logic [NBANK-1:0][31:0]    sbe_count_bank_o,
  output logic [NBANK-1:0][31:0]    dbe_count_bank_o,
  output logic [NBANK-1:0][31:0]    ptr_word_o          // current bank-local word pointer (observable)
);

  // ==========================================================================
  // Derived constants & checks
  // ==========================================================================
  localparam int unsigned BEAT_BYTES = DATA_W/8;
  localparam int unsigned BEAT_LSB   = (BEAT_BYTES<=1)?0:$clog2(BEAT_BYTES);
  localparam int unsigned BANK_W     = (NBANK<=1)?1:$clog2(NBANK);

  function automatic bit is_pow2(input int unsigned x); return (x & (x-1)) == 0; endfunction

  initial begin
    if (!is_pow2(NBANK)) $error("gb_scrubber: NBANK must be power-of-two.");
    if (DATA_W % 8 != 0) $error("gb_scrubber: DATA_W must be multiple of 8.");
    if (MAX_OUTSTANDING != 1) $error("gb_scrubber: this implementation assumes MAX_OUTSTANDING==1.");
  end

  // ==========================================================================
  // Helpers
  // ==========================================================================
  // Insert bank bits into a bank-local byte address to form global address:
  //   let 'boff' be bank-local byte offset (word_idx << BEAT_LSB);
  //   global = ( (boff >> BANK_SEL_LSB) << (BANK_SEL_LSB + BANK_W) )
  //          | ( (bank_id) << BANK_SEL_LSB )
  //          | ( boff & ((1<<BANK_SEL_LSB)-1) )
  function automatic logic [ADDR_W-1:0]
    compose_addr(input logic [BANK_W-1:0] bank_id, input logic [ADDR_W-1:0] bank_local_boff);
    logic [ADDR_W-1:0] low_mask;
    logic [ADDR_W-1:0] low;
    logic [ADDR_W-1:0] upper;
    begin
      low_mask = (ADDR_W'({{(ADDR_W-1){1'b0}},1'b1}) << BANK_SEL_LSB) - 1'b1; // (1<<BANK_SEL_LSB)-1
      low      = bank_local_boff & low_mask;
      upper    = bank_local_boff >> BANK_SEL_LSB;
      compose_addr =
        (upper << (BANK_SEL_LSB + BANK_W)) |
        (ADDR_W'(bank_id) << BANK_SEL_LSB) |
        low;
    end
  endfunction

  // ==========================================================================
  // State / registers
  // ==========================================================================
  typedef enum logic [1:0] { S_IDLE, S_ISSUE_RD, S_WAIT_RD, S_ISSUE_WB } state_e;
  state_e state_q, state_d;

  // Round-robin bank scheduler
  logic [BANK_W-1:0] rr_bank_q, rr_bank_d;

  // Per-bank word pointers and wrap pulses
  logic [NBANK-1:0][31:0] ptr_q, ptr_d;
  logic [NBANK-1:0]       wrap_pulse_d, wrap_pulse_q;

  // Current in-flight (only one allowed)
  logic                   inflight_q;
  logic [BANK_W-1:0]      inflight_bank_q;
  logic [ADDR_W-1:0]      inflight_addr_q; // full global address used for READ
  logic [DATA_W-1:0]      inflight_data_q; // latched data for writeback

  // READ/WRITE issue strobes
  logic issue_rd, issue_wb;

  // Rate limiter
  logic [RATE_DIV_W-1:0] rate_cnt_q, rate_cnt_d;
  logic rate_tick;

  // Summary counters
  logic [31:0] rd_issued_q, rd_completed_q, wr_issued_q;
  logic [31:0] sbe_cnt_q, dbe_cnt_q;
  logic [NBANK-1:0][31:0] sbe_bank_q, dbe_bank_q;

  // One-pass 'done' tracking (each bank must wrap at least once)
  logic [NBANK-1:0] pass_done_mask_q, pass_done_mask_d;
  logic             done_q, done_d;

  // Busy/active
  assign active_o = (state_q != S_IDLE) || inflight_q;

  // ==========================================================================
  // Rate limiter
  // ==========================================================================
  always_comb begin
    rate_tick = (rate_cnt_q == '0);
    rate_cnt_d = rate_cnt_q;
    if (!cfg_enable_i || cmd_abort_i || cmd_start_i) begin
      rate_cnt_d = cfg_rate_div_i;
    end else if (state_q == S_ISSUE_RD && issue_rd && rd_req_ready_i) begin
      // reload after a successful READ issue
      rate_cnt_d = cfg_rate_div_i;
    end else if (rate_cnt_q != '0) begin
      rate_cnt_d = rate_cnt_q - 1'b1;
    end
  end

  // ==========================================================================
  // Next-bank search (round-robin) — combinational
  // ==========================================================================
  function automatic logic [BANK_W-1:0] next_enabled_bank(
    input logic [BANK_W-1:0] start,
    input logic [NBANK-1:0]  mask,
    input logic [NBANK-1:0][31:0] words
  );
    logic [BANK_W-1:0] idx;
    idx = start;
    for (int k=0; k<NBANK; k++) begin
      logic [BANK_W-1:0] cand = (start + BANK_W'(k)) & BANK_W'({{(BANK_W-1){1'b1}},1'b1} << 0);
      if (mask[cand] && (words[cand] != 32'd0)) return cand;
    end
    return start; // fallback
  endfunction

  // ==========================================================================
  // FSM: Scrub loop
  // ==========================================================================
  // Defaults
  always_comb begin
    state_d        = state_q;
    rr_bank_d      = rr_bank_q;
    ptr_d          = ptr_q;
    wrap_pulse_d   = '0;

    issue_rd       = 1'b0;
    issue_wb       = 1'b0;

    // Master IF defaults
    rd_req_valid_o = 1'b0;
    rd_addr_o      = inflight_addr_q; // stable when waiting
    wr_req_valid_o = 1'b0;
    wr_addr_o      = inflight_addr_q;
    wr_data_o      = inflight_data_q;
    wr_strb_o      = {DATA_W/8{1'b1}};

    // Read response ready: accept whenever waiting for it
    rd_rsp_ready_o = (state_q == S_WAIT_RD);

    // One-pass done tracking
    pass_done_mask_d = pass_done_mask_q;
    done_d           = done_q;

    if (cmd_abort_i) begin
      state_d = S_IDLE;
    end

    unique case (state_q)
      // ------------------------------------------------------
      S_IDLE: begin
        if (cmd_start_i) begin
          // reset pointers and pass_done
          for (int b=0; b<NBANK; b++) begin
            ptr_d[b] = 32'd0;
          end
          rr_bank_d        = '0;
          pass_done_mask_d = '0;
          done_d           = 1'b0;
        end

        if (cfg_enable_i && !cmd_pause_i && !done_q && rate_tick) begin
          // Choose first bank
          rr_bank_d = next_enabled_bank(rr_bank_q, cfg_bank_mask_i, cfg_bank_words_i);
          state_d   = S_ISSUE_RD;
        end
      end

      // ------------------------------------------------------
      S_ISSUE_RD: begin
        if (cfg_enable_i && !cmd_pause_i && !inflight_q && rate_tick) begin
          // Compose READ address for current (rr_bank_q, ptr_q[bank])
          logic [ADDR_W-1:0] boff = (ptr_q[rr_bank_q] << BEAT_LSB);
          logic [ADDR_W-1:0] addr = compose_addr(rr_bank_q, boff) | cfg_global_base_i;

          rd_req_valid_o = 1'b1;
          rd_addr_o      = addr;
          issue_rd       = 1'b1;

          if (rd_req_ready_i) begin
            // Latch inflight info
            // (sequential part will capture)
            // Advance bank-local pointer
            if (ptr_q[rr_bank_q] + 32'd1 == cfg_bank_words_i[rr_bank_q]) begin
              ptr_d[rr_bank_q]      = 32'd0;
              wrap_pulse_d[rr_bank_q] = 1'b1;
              pass_done_mask_d[rr_bank_q] = 1'b1;
            end else begin
              ptr_d[rr_bank_q] = ptr_q[rr_bank_q] + 32'd1;
            end
            // Advance RR to next bank for next READ
            rr_bank_d = next_enabled_bank(rr_bank_q + BANK_W'(1), cfg_bank_mask_i, cfg_bank_words_i);
            state_d   = S_WAIT_RD;
          end
        end else if (!cfg_enable_i || cmd_pause_i || done_q) begin
          state_d = S_IDLE;
        end
      end

      // ------------------------------------------------------
      S_WAIT_RD: begin
        // Wait for response; always ready to take it
        if (rd_rsp_valid_i && rd_rsp_ready_o) begin
          // Sequencing in seq block (counts). Decide writeback.
          if (rd_rsp_sbe_i && cfg_writeback_en_i) begin
            state_d = S_ISSUE_WB;
          end else begin
            state_d = S_IDLE;
          end
        end
      end

      // ------------------------------------------------------
      S_ISSUE_WB: begin
        // Attempt scrub writeback of corrected data
        wr_req_valid_o = 1'b1;
        wr_addr_o      = inflight_addr_q;
        wr_data_o      = inflight_data_q;
        if (wr_req_ready_i) begin
          state_d = S_IDLE;
        end
      end

      default: state_d = S_IDLE;
    endcase

    // One-pass completion check: when all enabled banks have wrapped at least once
    if (cfg_enable_i && !cfg_continuous_i) begin
      logic all_done;
      all_done = 1'b1;
      for (int b=0; b<NBANK; b++) begin
        if (cfg_bank_mask_i[b] && (cfg_bank_words_i[b] != 0) && !pass_done_mask_d[b]) all_done = 1'b0;
      end
      if (all_done) done_d = 1'b1;
    end
  end // always_comb

  // ==========================================================================
  // Sequential: capture inflight, counters, wrap pulses
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q          <= S_IDLE;
      rr_bank_q        <= '0;
      for (int b=0; b<NBANK; b++) begin
        ptr_q[b]       <= 32'd0;
        sbe_bank_q[b]  <= 32'd0;
        dbe_bank_q[b]  <= 32'd0;
      end
      wrap_pulse_q     <= '0;

      inflight_q       <= 1'b0;
      inflight_bank_q  <= '0;
      inflight_addr_q  <= '0;
      inflight_data_q  <= '0;

      rate_cnt_q       <= '0;

      rd_issued_q      <= 32'd0;
      rd_completed_q   <= 32'd0;
      wr_issued_q      <= 32'd0;
      sbe_cnt_q        <= 32'd0;
      dbe_cnt_q        <= 32'd0;

      pass_done_mask_q <= '0;
      done_q           <= 1'b0;
    end else begin
      state_q          <= state_d;
      rr_bank_q        <= rr_bank_d;
      ptr_q            <= ptr_d;
      wrap_pulse_q     <= wrap_pulse_d;
      rate_cnt_q       <= rate_cnt_d;

      pass_done_mask_q <= pass_done_mask_d;
      done_q           <= done_d;

      // On READ issue accept: latch inflight tuple
      if (state_q == S_ISSUE_RD && issue_rd && rd_req_ready_i) begin
        inflight_q      <= 1'b1;
        inflight_bank_q <= rr_bank_q;
        inflight_addr_q <= rd_addr_o; // already composed
        rd_issued_q     <= rd_issued_q + 32'd1;
      end

      // On READ response accept
      if (state_q == S_WAIT_RD && rd_rsp_valid_i && rd_rsp_ready_o) begin
        inflight_q      <= 1'b0; // completed
        rd_completed_q  <= rd_completed_q + 32'd1;

        // Latch data for potential writeback
        inflight_data_q <= rd_rsp_data_i;

        if (rd_rsp_sbe_i) begin
          sbe_cnt_q                 <= sbe_cnt_q + 32'd1;
          sbe_bank_q[inflight_bank_q] <= sbe_bank_q[inflight_bank_q] + 32'd1;
        end
        if (rd_rsp_dbe_i) begin
          dbe_cnt_q                 <= dbe_cnt_q + 32'd1;
          dbe_bank_q[inflight_bank_q] <= dbe_bank_q[inflight_bank_q] + 32'd1;
        end
      end

      // On WRITEBACK accept
      if (state_q == S_ISSUE_WB && wr_req_valid_o && wr_req_ready_i) begin
        wr_issued_q <= wr_issued_q + 32'd1;
      end
    end
  end

  // ==========================================================================
  // Outputs
  // ==========================================================================
  assign bank_wrap_pulse_o = wrap_pulse_q;

  assign rd_issued_o     = rd_issued_q;
  assign rd_completed_o  = rd_completed_q;
  assign wr_issued_o     = wr_issued_q;
  assign sbe_count_o     = sbe_cnt_q;
  assign dbe_count_o     = dbe_cnt_q;

  generate
    for (genvar b=0; b<NBANK; b++) begin : G_STAT
      assign sbe_count_bank_o[b] = sbe_bank_q[b];
      assign dbe_count_bank_o[b] = dbe_bank_q[b];
      assign ptr_word_o[b]       = ptr_q[b];
    end
  endgenerate

  // One-pass done: sticky until next cmd_start_i
  assign done_o = done_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Single outstanding enforced
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (state_q==S_WAIT_RD) |-> inflight_q)
    else $error("gb_scrubber: WAIT_RD without inflight.");

  // Beat alignment of issued addresses
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rd_req_valid_o && rd_req_ready_i |-> (rd_addr_o[BEAT_LSB-1:0] == '0))
    else $error("gb_scrubber: issued READ not beat-aligned.");

  assert property (@(posedge clk_i) disable iff(!rstn_i)
    wr_req_valid_o && wr_req_ready_i |-> (wr_addr_o[BEAT_LSB-1:0] == '0))
    else $error("gb_scrubber: issued WRITE not beat-aligned.");

  // If writeback disabled, never drive write channel
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (!cfg_writeback_en_i) |-> !(state_q==S_ISSUE_WB))
    else $warning("gb_scrubber: writeback disabled but in ISSUE_WB.");
`endif

endmodule

`default_nettype wire
