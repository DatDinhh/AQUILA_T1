// ============================================================================
//  Bus Traffic Injector (W-path) — Global Buffer Master with 2:4 sparsity
//  File: bus_inject_w.sv
//
//  Purpose:
//    - Generate write/read traffic that looks like "weights":
//        * element-quantized payload (ELEM_W bits, default 8)
//        * optional 2:4 structured sparsity (two zeros per group of four)
//    - Deterministic verification: expected pattern is recomputed from
//      (byte address, element index, seed).
//    - Modes: FILL_ONLY, READ_ONLY, READ_VERIFY, FILL_THEN_VERIFY.
//    - Bank interleaving: rotate target bank every N words.
//    - Single outstanding read for tag-less fabrics.
//    - Robust counters and last-error snapshot.
//
//  Connects to: master-side of gb_crossbar.sv
//
//  Addressing:
//    - Byte addresses, beat = DATA_W/8 bytes (aligned).
//    - Bank index field sits at addr[BANK_SEL_LSB +: log2(NBANKS)].
//    - Interleaving rotates banks every cfg_interleave_words_i accepted ops.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module bus_inject_w #(
  // ---------- Topology / geometry ----------
  parameter int unsigned NBANKS       = 8,    // power-of-two recommended
  parameter int unsigned DATA_W       = 512,  // bits, multiple of 8
  parameter int unsigned ADDR_W       = 32,   // byte address width
  parameter int unsigned BANK_SEL_LSB = (DATA_W<=8)?0:$clog2(DATA_W/8),

  // Element granularity for "weights"
  parameter int unsigned ELEM_W       = 8,    // bits per element (8 or 16 typical)
  parameter int unsigned GROUP_2OF4   = 4,    // group size for 2:4 (keep as 4)

  // ---------- Defaults / behavior ----------
  parameter bit          START_PAUSED = 1'b0
)(
  input  logic                           clk_i,
  input  logic                           rstn_i,      // synchronous, active-low

  // ==========================================================================
  // Downstream (to gb_crossbar) — master interface
  // ==========================================================================
  // Write (single beat)
  output logic                           wr_req_valid_o,
  input  logic                           wr_req_ready_i,
  output logic [ADDR_W-1:0]              wr_addr_o,
  output logic [DATA_W-1:0]              wr_data_o,
  output logic [DATA_W/8-1:0]            wr_strb_o,

  // Read request (single beat)
  output logic                           rd_req_valid_o,
  input  logic                           rd_req_ready_i,
  output logic [ADDR_W-1:0]              rd_addr_o,

  // Read response
  input  logic                           rd_rsp_valid_i,
  output logic                           rd_rsp_ready_o,
  input  logic [DATA_W-1:0]              rd_rsp_data_i,
  input  logic                           rd_rsp_sbe_i, // corrected single-bit
  input  logic                           rd_rsp_dbe_i, // detected double-bit

  // ==========================================================================
  // Configuration (typically via CSR)
  // ==========================================================================
  // Enable + mode
  input  logic                           cfg_en_i,
  // 0=IDLE, 1=FILL_ONLY, 2=READ_ONLY, 3=READ_VERIFY, 4=FILL_THEN_VERIFY
  input  logic [2:0]                     cfg_mode_i,

  // Region & banking
  input  logic [NBANKS-1:0]              cfg_bank_mask_i,     // which banks to touch
  input  logic [NBANKS-1:0]              bank_enable_i,       // live banks (from partition/power)
  input  logic [31:0]                    cfg_words_per_bank_i,// beats per bank
  input  logic [31:0]                    cfg_start_word_i,    // starting word index per bank
  input  logic [31:0]                    cfg_stride_words_i,  // step (>=1)

  // Interleave: rotate bank after this many accepted ops
  input  logic                           cfg_interleave_en_i,
  input  logic [15:0]                    cfg_interleave_words_i, // >=1 when enabled

  // Throttle
  input  logic [15:0]                    cfg_gap_cyc_i,       // idle cycles between accepted ops

  // Pattern
  input  logic [63:0]                    cfg_seed_i,          // base seed
  input  logic                           cfg_signed_i,        // 1: signed two's-complement elements
  // 2:4 sparsity (two zeros per four consecutive elements)
  input  logic                           cfg_sparsity_2of4_en_i,

  // Control pulses
  input  logic                           cmd_start_i,         // resets pointers & (un)pauses
  input  logic                           cmd_pause_i,
  input  logic                           cmd_resume_i,
  input  logic                           cmd_abort_i,         // immediate idle
  input  logic                           cmd_clr_counters_i,

  // ==========================================================================
  // Status / Telemetry
  // ==========================================================================
  output logic                           active_o,
  output logic                           paused_o,
  output logic [31:0]                    op_rd_issued_o,
  output logic [31:0]                    op_wr_issued_o,
  output logic [31:0]                    verify_ok_o,
  output logic [31:0]                    verify_err_o,
  output logic [31:0]                    sbe_seen_o,
  output logic [31:0]                    dbe_seen_o,
  output logic [$clog2((NBANKS>1)?NBANKS:2)-1:0] cur_bank_o,
  output logic [31:0]                    cur_word_o,
  output logic [ADDR_W-1:0]              last_addr_o,
  output logic [DATA_W-1:0]              last_exp_o,
  output logic [DATA_W-1:0]              last_got_o,
  output logic [DATA_W-1:0]              last_diff_o
);

  // ==========================================================================
  // Derived constants / checks
  // ==========================================================================
  localparam int unsigned BEAT_BYTES = DATA_W/8;
  localparam int unsigned BEAT_LSB   = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);
  localparam int unsigned BANK_W     = (NBANKS <= 1) ? 1 : $clog2(NBANKS);

  // Address split around bank bits (LBITS below bank field, UBITS above)
  localparam int unsigned LOWER_W = (BANK_SEL_LSB > BEAT_LSB) ? (BANK_SEL_LSB - BEAT_LSB) : 0;
  localparam int unsigned UPPER_W = (ADDR_W > (BANK_SEL_LSB + BANK_W)) ? (ADDR_W - (BANK_SEL_LSB + BANK_W)) : 0;
  localparam int unsigned LOCIDX_W = LOWER_W + UPPER_W; // per-bank word index width in beats

  localparam int unsigned ELEM_PER_BEAT = (DATA_W / ELEM_W);
  localparam int unsigned GROUPS_PER_BEAT = (ELEM_PER_BEAT / GROUP_2OF4);

  initial begin
    if (DATA_W % 8 != 0)     $error("bus_inject_w: DATA_W must be multiple of 8.");
    if (DATA_W % ELEM_W != 0)$error("bus_inject_w: DATA_W must be divisible by ELEM_W.");
    if (GROUP_2OF4 != 4)     $error("bus_inject_w: GROUP_2OF4 must be 4 for 2:4 sparsity.");
  end

  // ==========================================================================
  // Helpers
  // ==========================================================================
  function automatic bit mask_any(input logic [NBANKS-1:0] m); return (m != '0); endfunction

  // Effective bank mask (configured ∩ enabled)
  wire [NBANKS-1:0] eff_mask = cfg_bank_mask_i & bank_enable_i;

  // Address composer: from (bank, per-bank word index) → global byte address
  function automatic logic [ADDR_W-1:0]
    compose_addr(input logic [BANK_W-1:0] bank,
                 input logic [LOCIDX_W-1:0] word_idx);
    logic [ADDR_W-1:0] a;
    a = '0;
    if (LOWER_W > 0) a[BANK_SEL_LSB-1 : BEAT_LSB] = word_idx[LOWER_W-1:0];
    a[BANK_SEL_LSB +: BANK_W] = bank;
    if (UPPER_W > 0) a[ADDR_W-1 : BANK_SEL_LSB + BANK_W] = word_idx[LOCIDX_W-1:LOWER_W];
    return a;
  endfunction

  // Next enabled bank (wrap)
  function automatic logic [BANK_W-1:0]
    next_bank(input logic [NBANKS-1:0] m, input logic [BANK_W-1:0] cur);
    logic [BANK_W-1:0] i;
    for (i = cur + 1; i < NBANKS[BANK_W-1:0]; i++) if (m[i]) return i;
    for (i = '0; i <= cur; i++) if (m[i]) return i;
    return cur;
  endfunction

  // Simple 64b xorshift (cheap mixing; no multipliers)
  function automatic logic [63:0] xorshift64(input logic [63:0] x_in);
    logic [63:0] x;
    begin
      x = x_in;
      x ^= (x << 13);
      x ^= (x >>  7);
      x ^= (x << 17);
      return x;
    end
  endfunction

  // Element generator (ELEM_W bits) from (addr, elem_idx, seed, signed)
  function automatic logic [ELEM_W-1:0]
    gen_elem(input logic [ADDR_W-1:0] addr_b, input int unsigned eidx,
             input logic [63:0] seed, input logic signed_mode);
    logic [63:0] mix;
    logic [ELEM_W-1:0] v;
    begin
      mix = seed
          ^ { {(64-ADDR_W){1'b0}}, addr_b }
          ^ 64'(eidx)
          ^ ({ {(64-ADDR_W){1'b0}}, addr_b } >> 19);
      mix = xorshift64(mix);

      // Take low bits for magnitude, add sign if requested
      v = mix[ELEM_W-1:0];
      if (signed_mode) begin
        // Bias away from large negative wrap for visibility
        if (ELEM_W >= 4) v[ELEM_W-1 -: 4] ^= 4'h1;
      end
      return v;
    end
  endfunction

  // Apply 2:4 sparsity mask (zero exactly two elems per group of 4)
  function automatic void apply_2of4(
    input  logic [ADDR_W-1:0] addr_b,
    input  logic [63:0]       seed,
    inout  logic [DATA_W-1:0] data // ELEM_W-packed, little-endian within beat
  );
    if (!cfg_sparsity_2of4_en_i) return;

    // Map sel in {0..5} to pairs {00,01,02,03,12,13} (indices within group)
    automatic logic [1:0] map_i0 [0:5] = '{2'd0,2'd0,2'd0,2'd0,2'd1,2'd1};
    automatic logic [1:0] map_i1 [0:5] = '{2'd1,2'd2,2'd3,2'd2,2'd2,2'd3};

    for (int g=0; g<GROUPS_PER_BEAT; g++) begin
      // Deterministic draw from address+group
      logic [63:0] r = xorshift64(seed ^ { {(64-ADDR_W){1'b0}}, addr_b } ^ 64'(g*0x9E37));
      int unsigned sel = r[2:0]; // 0..7; fold to 0..5
      sel = (sel >= 6) ? (sel - 6) : sel;
      // Compute element indices inside this group
      int base = g*GROUP_2OF4;
      int k0 = base + map_i0[sel];
      int k1 = base + map_i1[sel];

      // Zero the two chosen indices
      for (int j=0; j<GROUP_2OF4; j++) begin
        int ej = base + j;
        if ((ej == k0) || (ej == k1)) begin
          int lo = ej*ELEM_W;
          data[lo +: ELEM_W] = '0;
        end
      end
    end
  endfunction

  // Beat generator: fill DATA_W with ELEM_PER_BEAT elements
  function automatic logic [DATA_W-1:0]
    gen_weight_beat(input logic [ADDR_W-1:0] addr_b, input logic [63:0] seed, input logic signed_mode);
    logic [DATA_W-1:0] out;
    begin
      out = '0;
      for (int e=0; e<ELEM_PER_BEAT; e++) begin
        out[e*ELEM_W +: ELEM_W] = gen_elem(addr_b, e, seed, signed_mode);
      end
      apply_2of4(addr_b, seed, out);
      return out;
    end
  endfunction

  // ==========================================================================
  // State / control
  // ==========================================================================
  typedef enum logic [2:0] {
    S_IDLE,
    S_WAIT_GAP,
    S_ISSUE_WR,
    S_ISSUE_RD,
    S_WAIT_RD
  } state_e;

  state_e                   st_q, st_d;

  // Per-bank word pointers (+ done mask)
  logic [31:0]              ptr_q    [NBANKS];
  logic [31:0]              ptr_d    [NBANKS];
  logic [NBANKS-1:0]        bank_done_q, bank_done_d;

  // Scheduler: current bank + run-length within interleave block
  logic [$clog2((NBANKS>1)?NBANKS:2)-1:0] bank_q, bank_d;
  logic [15:0]              blk_run_q, blk_run_d;

  // Throttle gap
  logic [15:0]              gap_q, gap_d;

  // Pause
  logic                     paused_q, paused_d;

  // One outstanding read
  logic                     rd_out_q, rd_out_d;
  logic [ADDR_W-1:0]        rd_addr_lat_q, rd_addr_lat_d;

  // Telemetry
  logic [31:0]              rd_cnt_q, rd_cnt_d;
  logic [31:0]              wr_cnt_q, wr_cnt_d;
  logic [31:0]              ok_cnt_q, ok_cnt_d;
  logic [31:0]              er_cnt_q, er_cnt_d;
  logic [31:0]              sbe_cnt_q, sbe_cnt_d;
  logic [31:0]              dbe_cnt_q, dbe_cnt_d;

  logic [ADDR_W-1:0]        last_addr_q, last_addr_d;
  logic [DATA_W-1:0]        last_exp_q,  last_exp_d;
  logic [DATA_W-1:0]        last_got_q,  last_got_d;
  logic [DATA_W-1:0]        last_diff_q, last_diff_d;

  // Mode helpers
  localparam logic [2:0] MODE_IDLE  = 3'd0;
  localparam logic [2:0] MODE_FILL  = 3'd1;
  localparam logic [2:0] MODE_RONLY = 3'd2;
  localparam logic [2:0] MODE_RVFY  = 3'd3;
  localparam logic [2:0] MODE_FVFY  = 3'd4;

  // Active when enabled, not paused, mask non-empty, stride OK, etc.
  wire run_ok = cfg_en_i && !paused_q && mask_any(eff_mask) &&
                (cfg_words_per_bank_i != 0) && (cfg_stride_words_i != 0) &&
                (cfg_mode_i != MODE_IDLE);

  // Outputs defaults
  localparam [DATA_W/8-1:0] STRB_ALL = { (DATA_W/8){1'b1} };
  always_comb begin
    wr_req_valid_o = 1'b0;
    wr_addr_o      = '0;
    wr_data_o      = '0;
    wr_strb_o      = STRB_ALL;

    rd_req_valid_o = 1'b0;
    rd_addr_o      = '0;

    rd_rsp_ready_o = (st_q == S_WAIT_RD);
  end

  // Status
  assign active_o        = (st_q != S_IDLE) && run_ok;
  assign paused_o        = paused_q;
  assign op_rd_issued_o  = rd_cnt_q;
  assign op_wr_issued_o  = wr_cnt_q;
  assign verify_ok_o     = ok_cnt_q;
  assign verify_err_o    = er_cnt_q;
  assign sbe_seen_o      = sbe_cnt_q;
  assign dbe_seen_o      = dbe_cnt_q;
  assign cur_bank_o      = bank_q;
  assign cur_word_o      = (bank_q < NBANKS) ? ptr_q[bank_q] : 32'd0;
  assign last_addr_o     = last_addr_q;
  assign last_exp_o      = last_exp_q;
  assign last_got_o      = last_got_q;
  assign last_diff_o     = last_diff_q;

  // All banks finished?
  function automatic bit all_done(input logic [NBANKS-1:0] mask, input logic [NBANKS-1:0] done);
    for (int b=0; b<NBANKS; b++) if (mask[b] && !done[b]) return 1'b0;
    return 1'b1;
  endfunction

  // ==========================================================================
  // Next-state logic
  // ==========================================================================
  always_comb begin
    // Hold-by-default
    st_d            = st_q;
    paused_d        = paused_q;
    gap_d           = gap_q;

    bank_d          = bank_q;
    blk_run_d       = blk_run_q;

    for (int b=0; b<NBANKS; b++) begin
      ptr_d[b]      = ptr_q[b];
    end
    bank_done_d     = bank_done_q;

    rd_out_d        = rd_out_q;
    rd_addr_lat_d   = rd_addr_lat_q;

    rd_cnt_d        = rd_cnt_q;
    wr_cnt_d        = wr_cnt_q;
    ok_cnt_d        = ok_cnt_q;
    er_cnt_d        = er_cnt_q;
    sbe_cnt_d       = sbe_cnt_q;
    dbe_cnt_d       = dbe_cnt_q;

    last_addr_d     = last_addr_q;
    last_exp_d      = last_exp_q;
    last_got_d      = last_got_q;
    last_diff_d     = last_diff_q;

    // External control
    if (cmd_abort_i) begin
      st_d      = S_IDLE;
      rd_out_d  = 1'b0;
    end
    if (cmd_pause_i)  paused_d = 1'b1;
    if (cmd_resume_i) paused_d = 1'b0;

    if (cmd_start_i) begin
      paused_d    = 1'b0;
      blk_run_d   = 16'd0;
      gap_d       = 16'd0;
      // Reset per-bank pointers and done flags
      for (int b=0; b<NBANKS; b++) begin
        ptr_d[b]      = cfg_start_word_i;
        bank_done_d[b]= 1'b0;
      end
      // Pick first enabled bank
      logic [$clog2((NBANKS>1)?NBANKS:2)-1:0] fb; fb = '0;
      for (int b=0; b<NBANKS; b++) if (eff_mask[b]) begin fb = b[$clog2((NBANKS>1)?NBANKS:2)-1:0]; break; end
      bank_d = fb;
      st_d   = S_WAIT_GAP;
    end

    if (cmd_clr_counters_i) begin
      rd_cnt_d='0; wr_cnt_d='0; ok_cnt_d='0; er_cnt_d='0; sbe_cnt_d='0; dbe_cnt_d='0;
    end

    // Stop if cannot run
    if (!run_ok) begin
      if (st_q != S_IDLE) st_d = S_IDLE;
    end

    // Interleave policy: next bank when block count reaches threshold OR bank done
    automatic logic need_rotate;
    need_rotate = 1'b0;
    if (cfg_interleave_en_i) begin
      if (blk_run_q >= (cfg_interleave_words_i == 0 ? 16'd0 : (cfg_interleave_words_i - 16'd1)))
        need_rotate = 1'b1;
    end

    // FSM
    unique case (st_q)
      // ------------------------------------------------------
      S_IDLE: begin
        gap_d    = 16'd0;
        rd_out_d = 1'b0;
      end

      // ------------------------------------------------------
      S_WAIT_GAP: begin
        if (!run_ok) begin
          st_d = S_IDLE;
        end else if (gap_q != 16'd0) begin
          gap_d = gap_q - 16'd1;
        end else begin
          // If current bank is exhausted, mark done
          if (ptr_q[bank_q] >= cfg_words_per_bank_i) begin
            bank_done_d[bank_q] = 1'b1;
            // Rotate immediately to next enabled/remaining bank
            logic [$clog2((NBANKS>1)?NBANKS:2)-1:0] nb = next_bank(eff_mask & ~bank_done_d, bank_q);
            if (!eff_mask[nb] || all_done(eff_mask, bank_done_d)) begin
              // Phase transition for FILL_THEN_VERIFY: after a full sweep, switch to READ_VERIFY
              if (cfg_mode_i == MODE_FVFY) begin
                // Reset pointers/flags for verify pass
                for (int b=0; b<NBANKS; b++) begin
                  ptr_d[b]      = cfg_start_word_i;
                  bank_done_d[b]= 1'b0;
                end
                blk_run_d = 16'd0;
                bank_d    = nb;
                st_d      = S_WAIT_GAP;
                // Switch effective behavior to READ_VERIFY by falling into S_ISSUE_RD next
                // We keep cfg_mode_i as is; logic below chooses per-cycle op.
              end else begin
                st_d = S_IDLE;
              end
            end else begin
              bank_d    = nb;
              blk_run_d = 16'd0;
              gap_d     = cfg_gap_cyc_i;
            end
          end else begin
            // Choose op by mode
            unique case (cfg_mode_i)
              MODE_FILL:  st_d = S_ISSUE_WR;
              MODE_RONLY: st_d = S_ISSUE_RD;
              MODE_RVFY:  st_d = S_ISSUE_RD;
              MODE_FVFY: begin
                // In FILL_THEN_VERIFY, first sweep behaves like FILL; after marking all banks done,
                // we re-enter here with pointers reset, and will behave like READ_VERIFY.
                if (!all_done(eff_mask, bank_done_q)) st_d = S_ISSUE_WR;
                else                                  st_d = S_ISSUE_RD;
              end
              default:    st_d = S_IDLE;
            endcase
          end
        end
      end

      // ------------------------------------------------------
      S_ISSUE_WR: begin
        // Address for current bank & pointer
        logic [LOCIDX_W-1:0] word_idx = LOCIDX_W'(ptr_q[bank_q]);
        logic [ADDR_W-1:0] addr = compose_addr(bank_q, word_idx);
        logic [DATA_W-1:0] data = gen_weight_beat(addr, cfg_seed_i, cfg_signed_i);

        wr_req_valid_o = 1'b1;
        wr_addr_o      = addr;
        wr_data_o      = data;

        if (wr_req_valid_o && wr_req_ready_i) begin
          wr_cnt_d              = wr_cnt_q + 32'd1;
          blk_run_d             = cfg_interleave_en_i ? (blk_run_q + 16'd1) : 16'd0;
          // Advance pointer for this bank
          ptr_d[bank_q]         = ptr_q[bank_q] + cfg_stride_words_i;
          // Rotate if interleave block ended
          if (cfg_interleave_en_i && need_rotate) begin
            bank_d    = next_bank(eff_mask & ~bank_done_d, bank_q);
            blk_run_d = 16'd0;
          end
          // Gap after acceptance
          gap_d = cfg_gap_cyc_i;
          st_d  = S_WAIT_GAP;
        end
      end

      // ------------------------------------------------------
      S_ISSUE_RD: begin
        // One outstanding read at a time
        if (!rd_out_q) begin
          logic [LOCIDX_W-1:0] word_idx = LOCIDX_W'(ptr_q[bank_q]);
          logic [ADDR_W-1:0] addr = compose_addr(bank_q, word_idx);
          rd_req_valid_o = 1'b1;
          rd_addr_o      = addr;
          if (rd_req_valid_o && rd_req_ready_i) begin
            rd_out_d        = 1'b1;
            rd_addr_lat_d   = addr;
            rd_cnt_d        = rd_cnt_q + 32'd1;
            // Move pointer now; writeback to gap logic afterwards
            ptr_d[bank_q]   = ptr_q[bank_q] + cfg_stride_words_i;
            blk_run_d       = cfg_interleave_en_i ? (blk_run_q + 16'd1) : 16'd0;
            if (cfg_interleave_en_i && need_rotate) begin
              bank_d    = next_bank(eff_mask & ~bank_done_d, bank_q);
              blk_run_d = 16'd0;
            end
            st_d            = S_WAIT_RD;
          end
        end
      end

      // ------------------------------------------------------
      S_WAIT_RD: begin
        if (rd_rsp_valid_i && rd_rsp_ready_o) begin
          // Observe ECC flags
          if (rd_rsp_sbe_i) sbe_cnt_d = sbe_cnt_q + 32'd1;
          if (rd_rsp_dbe_i) dbe_cnt_d = dbe_cnt_q + 32'd1;

          // Verification (READ_VERIFY or FVFY second pass)
          if ((cfg_mode_i == MODE_RVFY) ||
              (cfg_mode_i == MODE_FVFY && all_done(eff_mask, bank_done_q))) begin
            logic [DATA_W-1:0] exp = gen_weight_beat(rd_addr_lat_q, cfg_seed_i, cfg_signed_i);
            logic [DATA_W-1:0] dif = exp ^ rd_rsp_data_i;

            last_addr_d = rd_addr_lat_q;
            last_exp_d  = exp;
            last_got_d  = rd_rsp_data_i;
            last_diff_d = dif;

            if (dif == '0) ok_cnt_d = ok_cnt_q + 32'd1;
            else           er_cnt_d = er_cnt_q + 32'd1;
          end

          rd_out_d = 1'b0;
          gap_d    = cfg_gap_cyc_i;
          st_d     = S_WAIT_GAP;
        end
      end

      default: st_d = S_IDLE;
    endcase
  end

  // ==========================================================================
  // Sequential
  // ==========================================================================
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      st_q          <= S_IDLE;
      paused_q      <= START_PAUSED;
      gap_q         <= 16'd0;

      for (int b=0; b<NBANKS; b++) begin
        ptr_q[b]      <= 32'd0;
      end
      bank_done_q   <= '0;

      bank_q        <= '0;
      blk_run_q     <= 16'd0;

      rd_out_q      <= 1'b0;
      rd_addr_lat_q <= '0;

      rd_cnt_q      <= 32'd0;
      wr_cnt_q      <= 32'd0;
      ok_cnt_q      <= 32'd0;
      er_cnt_q      <= 32'd0;
      sbe_cnt_q     <= 32'd0;
      dbe_cnt_q     <= 32'd0;

      last_addr_q   <= '0;
      last_exp_q    <= '0;
      last_got_q    <= '0;
      last_diff_q   <= '0;
    end else begin
      st_q          <= st_d;
      paused_q      <= paused_d;
      gap_q         <= gap_d;

      for (int b=0; b<NBANKS; b++) begin
        ptr_q[b]    <= ptr_d[b];
      end
      bank_done_q   <= bank_done_d;

      bank_q        <= bank_d;
      blk_run_q     <= blk_run_d;

      rd_out_q      <= rd_out_d;
      rd_addr_lat_q <= rd_addr_lat_d;

      rd_cnt_q      <= rd_cnt_d;
      wr_cnt_q      <= wr_cnt_d;
      ok_cnt_q      <= ok_cnt_d;
      er_cnt_q      <= er_cnt_d;
      sbe_cnt_q     <= sbe_cnt_d;
      dbe_cnt_q     <= dbe_cnt_d;

      last_addr_q   <= last_addr_d;
      last_exp_q    <= last_exp_d;
      last_got_q    <= last_got_d;
      last_diff_q   <= last_diff_d;
    end
  end

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Single outstanding read invariant
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rd_out_q |-> !(rd_req_valid_o && rd_req_ready_i))
    else $error("bus_inject_w: issued a read while another is outstanding.");

  // Beat alignment on issued ops
  if (BEAT_LSB > 0) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      (wr_req_valid_o && wr_req_ready_i) |-> (wr_addr_o[BEAT_LSB-1:0] == '0))
      else $error("bus_inject_w: write address not beat-aligned.");
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      (rd_req_valid_o && rd_req_ready_i) |-> (rd_addr_o[BEAT_LSB-1:0] == '0))
      else $error("bus_inject_w: read address not beat-aligned.");
  end

  // Interleave guard
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    cfg_interleave_en_i -> (cfg_interleave_words_i != 0))
    else $error("bus_inject_w: interleave enabled but words==0.");

  // Bounds on words_per_bank vs addressable LOCIDX_W (best-effort runtime guard)
  if (LOCIDX_W < 32) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      cfg_words_per_bank_i <= (32'(1) << LOCIDX_W))
      else $error("bus_inject_w: words_per_bank exceeds per-bank address capacity.");
  end
`endif

endmodule

`default_nettype wire
