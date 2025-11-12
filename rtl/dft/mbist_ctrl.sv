// ============================================================================
//  mbist_ctrl.sv
//  Aquila — Memory BIST Controller (multi-bank, March-X / March-C-, backgrounds)
// ----------------------------------------------------------------------------
//  Test-port model (per cycle):
//    - Assert mem_req_o with mem_write_o=1 for WRITE and wdata/wmask valid.
//    - Assert mem_req_o with mem_write_o=0 for READ; data returns after RD_LATENCY
//      cycles with mem_rvalid_i high and mem_rdata_i stable.
//  The wrapper must connect this test-port to the selected bank when test_mode_o=1,
//  using bank_sel_o to choose which bank is under test.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module mbist_ctrl #(
  // ---------------------------- Geometry ------------------------------------
  parameter int unsigned NUM_BANKS     = 8,
  parameter int unsigned BANK_W        = (NUM_BANKS <= 1) ? 1 : $clog2(NUM_BANKS),

  parameter int unsigned ADDR_W        = 12,      // depth per bank = 2^ADDR_W
  parameter int unsigned DATA_W        = 64,
  parameter int unsigned WMASK_W       = (DATA_W/8),

  // ---------------------------- Memory timing --------------------------------
  parameter int unsigned RD_LATENCY    = 1,       // cycles from read req to rvalid

  // ---------------------------- PRBS/LFSR ------------------------------------
  parameter int unsigned LFSR_W        = 16,      // PRBS width (<=32 suggested)

  // ---------------------------- Safety / Limits ------------------------------
  parameter int unsigned MAX_OPS_PER_STEP = 2     // keep <= 3 if you extend algorithms
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,        // synchronous active-low

  // ============================ Control ======================================
  input  logic                         start_i,       // pulse to start run
  input  logic                         resume_i,      // pulse to resume when paused_on_fail
  input  logic [1:0]                   algo_sel_i,    // 0=March-X, 1=March-C-
  input  logic [2:0]                   bg_sel_i,      // 0=0s, 1=1s, 2=checker, 3=addr, 4=PRBS
  input  logic [NUM_BANKS-1:0]         bank_mask_i,   // which banks to test (1=include)
  input  logic                         order_up_i,    // bank iteration direction (1=0..N-1)
  input  logic [ADDR_W-1:0]            addr_lo_i,     // inclusive low address
  input  logic [ADDR_W-1:0]            addr_hi_i,     // inclusive high address
  input  logic [15:0]                  loops_i,       // number of full algorithm loops (0->1)
  input  logic                         stop_on_fail_i,
  input  logic                         pause_on_fail_i,
  input  logic                         ecc_bypass_i,  // request ECC bypass in wrapper during MBIST

  // Optional PRBS seed (used when bg_sel_i==PRBS); latched on start
  input  logic [LFSR_W-1:0]            prbs_seed_i,

  // ============================ Status/Results ================================
  output logic                         busy_o,
  output logic                         done_o,         // pulse at end of run
  output logic                         pass_o,         // sticky pass (until next start)
  output logic [31:0]                  fail_count_o,   // total mismatches (saturates)
  output logic                         fail_valid_o,   // first-fail latched
  output logic [BANK_W-1:0]            fail_bank_o,
  output logic [ADDR_W-1:0]            fail_addr_o,
  output logic [7:0]                   fail_step_o,    // step index
  output logic [3:0]                   fail_op_o,      // op index within step
  output logic [DATA_W-1:0]            fail_exp_o,
  output logic [DATA_W-1:0]            fail_obs_o,

  // ============================ Test-port to SRAM wrapper =====================
  output logic                         test_mode_o,    // 1 = MBIST owns the port
  output logic [BANK_W-1:0]            bank_sel_o,     // bank index under test
  output logic                         ecc_bypass_o,   // pass-through of ecc_bypass_i

  output logic                         mem_req_o,
  output logic                         mem_write_o,    // 1=WRITE, 0=READ
  output logic [ADDR_W-1:0]            mem_addr_o,
  output logic [DATA_W-1:0]            mem_wdata_o,
  output logic [WMASK_W-1:0]           mem_wmask_o,
  input  logic [DATA_W-1:0]            mem_rdata_i,
  input  logic                         mem_rvalid_i
);

  // ==========================================================================
  // Derived constants and helpers
  // ==========================================================================
  localparam int unsigned DEPTH = (1 << ADDR_W);

  // Clamp loops: 0 means 1 loop
  function automatic logic [15:0] norm_loops(input logic [15:0] L);
    return (L == 16'd0) ? 16'd1 : L;
  endfunction

  // ----------------------------- LFSR (PRBS background) ----------------------
  function automatic logic [LFSR_W-1:0] lfsr_next(input logic [LFSR_W-1:0] s);
    logic fb;
    if (LFSR_W >= 16) begin
      // taps for x^16 + x^14 + x^13 + x^11 + 1 (mask 0xB400) generalized for width
      fb = s[LFSR_W-1] ^ s[(LFSR_W>2)?(LFSR_W-3):0] ^ s[(LFSR_W>3)?(LFSR_W-4):0] ^ s[(LFSR_W>5)?(LFSR_W-6):0];
    end else begin
      // simple taps for smaller widths
      fb = s[LFSR_W-1] ^ s[(LFSR_W>3)?(LFSR_W-4):0];
    end
    return {s[LFSR_W-2:0], fb};
  endfunction

  // Replicate a byte into a DATA_W vector (utility)
  function automatic logic [DATA_W-1:0] rep8(input logic [7:0] b);
    logic [DATA_W-1:0] r;
    for (int i=0;i<DATA_W;i+=8) r[i +: 8] = b;
    return r;
  endfunction

  // Background generator P(address):
  //  - 0     : all 0s
  //  - 1     : all 1s
  //  - 2     : checkerboard: address parity selects 0xAA/0x55 per byte
  //  - 3     : address pattern: replicate addr bits
  //  - 4     : PRBS seeded at start, stepped by address
  function automatic logic [DATA_W-1:0]
    make_bg(input logic [2:0] bg_sel,
            input logic [ADDR_W-1:0] addr,
            input logic [LFSR_W-1:0] prbs_state);
    logic [DATA_W-1:0] p;
    unique case (bg_sel)
      3'd0: p = '0;
      3'd1: p = ~('0);
      3'd2: begin
        logic [7:0] a = (addr[0]) ? 8'hAA : 8'h55;
        p = rep8(a);
      end
      3'd3: begin
        // Address pattern: spread address bits across the data bus
        p = '0;
        for (int i=0;i<DATA_W;i++) p[i] = addr[(i % ADDR_W)];
      end
      3'd4: begin
        // PRBS: expand prbs_state across data words by repeating pattern
        p = '0;
        for (int i=0;i<DATA_W;i++) p[i] = prbs_state[(i % LFSR_W)];
      end
      default: p = '0;
    endcase
    return p;
  endfunction

  // For March ops we interpret W0/R0 as 'write/read background P', and W1/R1 as '~P'
  function automatic logic [DATA_W-1:0]
    pattern_for(input logic is_one, input logic [DATA_W-1:0] P);
    return is_one ? ~P : P;
  endfunction

  // ==========================================================================
  // Algorithm description
  // ==========================================================================
  typedef enum logic [2:0] { OP_NONE=3'd0, OP_W0=3'd1, OP_W1=3'd2, OP_R0=3'd3, OP_R1=3'd4 } op_e;

  // Fetch step characteristics for a given algorithm + step index.
  // dir_up=1 -> address asc, dir_up=0 -> desc.
  // ops_len <= MAX_OPS_PER_STEP (default 2) for these algorithms.
  function automatic void alg_get_step(
    input  logic [1:0] algo_sel,
    input  logic [7:0] step_idx,
    output logic       valid,
    output logic       dir_up,
    output op_e        op0,
    output op_e        op1,
    output logic [1:0] ops_len
  );
    valid   = 1'b1;
    dir_up  = 1'b1;
    op0     = OP_NONE;
    op1     = OP_NONE;
    ops_len = 2'd0;

    unique case (algo_sel)
      2'd0: begin : MARCH_X
        // March-X (4N): up w0; up r0,w1; up r1,w0; up r0
        unique case (step_idx)
          8'd0: begin dir_up=1; op0=OP_W0;                ops_len=2'd1; end
          8'd1: begin dir_up=1; op0=OP_R0; op1=OP_W1;     ops_len=2'd2; end
          8'd2: begin dir_up=1; op0=OP_R1; op1=OP_W0;     ops_len=2'd2; end
          8'd3: begin dir_up=1; op0=OP_R0;                ops_len=2'd1; end
          default: valid = 1'b0;
        endcase
      end
      2'd1: begin : MARCH_CMINUS
        // March C- (≈6N variant):
        // 0) up   w0
        // 1) up   r0, w1
        // 2) up   r1, w0
        // 3) down r0, w1
        // 4) down r1, w0
        // 5) up   r0
        unique case (step_idx)
          8'd0: begin dir_up=1; op0=OP_W0;                ops_len=2'd1; end
          8'd1: begin dir_up=1; op0=OP_R0; op1=OP_W1;     ops_len=2'd2; end
          8'd2: begin dir_up=1; op0=OP_R1; op1=OP_W0;     ops_len=2'd2; end
          8'd3: begin dir_up=0; op0=OP_R0; op1=OP_W1;     ops_len=2'd2; end
          8'd4: begin dir_up=0; op0=OP_R1; op1=OP_W0;     ops_len=2'd2; end
          8'd5: begin dir_up=1; op0=OP_R0;                ops_len=2'd1; end
          default: valid = 1'b0;
        endcase
      end
      default: begin
        valid = 1'b0;
      end
    endcase
  endfunction

  // ==========================================================================
  // State machine
  // ==========================================================================
  typedef enum logic [3:0] {
    S_IDLE, S_PREP_BANK, S_PREP_STEP, S_PREP_ADDR,
    S_ISSUE_OP, S_WAIT_RD,
    S_NEXT_OP, S_NEXT_ADDR, S_NEXT_STEP, S_NEXT_BANK,
    S_PAUSED, S_DONE
  } state_e;

  state_e                 st_q, st_d;

  // Bookkeeping
  logic [BANK_W-1:0]      bank_q, bank_d;
  logic [7:0]             step_q, step_d;
  logic [1:0]             op_q,   op_d;      // 0/1 for op within step
  logic                   dir_up_q, dir_up_d;
  op_e                    step_op0_q, step_op1_q;
  logic [1:0]             step_ops_len_q;

  logic [ADDR_W-1:0]      addr_q, addr_d;
  logic [ADDR_W-1:0]      addr_lo_q, addr_hi_q;

  logic [15:0]            loop_total_q, loop_left_q, loop_left_d;

  // Data background state (PRBS)
  logic [LFSR_W-1:0]      prbs_q, prbs_d;

  // Outputs / status regs
  logic                   busy_q, busy_d;
  logic                   done_pulse_d;
  logic                   pass_q, pass_d;

  logic [31:0]            fail_count_q, fail_count_d;
  logic                   fail_captured_q, fail_captured_d;
  logic [BANK_W-1:0]      fail_bank_q;   logic [ADDR_W-1:0] fail_addr_q;
  logic [7:0]             fail_step_q;   logic [3:0]        fail_op_idx_q;
  logic [DATA_W-1:0]      fail_exp_q,    fail_obs_q;

  // Compare pipeline
  logic [RD_LATENCY-1:0]  rd_pipe_q;
  logic [DATA_W-1:0]      exp_pipe_q   [RD_LATENCY]; // expected data at compare point
  logic [BANK_W-1:0]      bnk_pipe_q   [RD_LATENCY];
  logic [ADDR_W-1:0]      adr_pipe_q   [RD_LATENCY];
  logic [7:0]             stp_pipe_q   [RD_LATENCY];
  logic [3:0]             opi_pipe_q   [RD_LATENCY];

  // Misc
  logic [WMASK_W-1:0]     all_wmask;
  assign all_wmask = {WMASK_W{1'b1}};

  // Shared defaults
  assign ecc_bypass_o = ecc_bypass_i;
  assign busy_o       = busy_q;
  assign pass_o       = pass_q;
  assign fail_count_o = fail_count_q;

  // Test-mode asserted whenever not IDLE/DONE
  assign test_mode_o  = (st_q != S_IDLE) && (st_q != S_DONE);
  assign bank_sel_o   = bank_q;

  // ========================= FSM combinational =================================
  always_comb begin
    // Default outputs to idle
    mem_req_o     = 1'b0;
    mem_write_o   = 1'b0;
    mem_addr_o    = '0;
    mem_wdata_o   = '0;
    mem_wmask_o   = all_wmask;

    st_d           = st_q;
    bank_d         = bank_q;
    step_d         = step_q;
    op_d           = op_q;
    dir_up_d       = dir_up_q;
    step_op0_q     = step_op0_q; // hold (updated in PREP_STEP)
    step_op1_q     = step_op1_q;
    step_ops_len_q = step_ops_len_q;

    addr_d         = addr_q;
    addr_lo_q      = addr_lo_q;
    addr_hi_q      = addr_hi_q;

    prbs_d         = prbs_q;

    busy_d         = busy_q;
    pass_d         = pass_q;
    done_pulse_d   = 1'b0;

    loop_left_d    = loop_left_q;

    fail_count_d     = fail_count_q;
    fail_captured_d  = fail_captured_q;

    // ---- background P for current address ----
    logic [DATA_W-1:0] P = make_bg(bg_sel_i, addr_q, prbs_q);
    // Next PRBS value if selected (advance per address)
    logic [LFSR_W-1:0] PRBS_NEXT = lfsr_next(prbs_q);

    // Resolve this-address + op expected/write data
    op_e cur_op;
    cur_op = (op_q == 2'd0) ? step_op0_q : step_op1_q;

    logic is_read, is_one;
    is_read = (cur_op == OP_R0) || (cur_op == OP_R1);
    is_one  = (cur_op == OP_W1) || (cur_op == OP_R1);

    logic [DATA_W-1:0] wd = pattern_for(is_one, P); // data to write or expect

    // ========= FSM =========
    unique case (st_q)
      // ----------------------------------------------------------------------
      S_IDLE: begin
        busy_d        = 1'b0;
        done_pulse_d  = 1'b0;
        // Wait for start
        if (start_i) begin
          // Initialize
          busy_d        = 1'b1;
          pass_d        = 1'b1;
          fail_count_d  = 32'd0;
          fail_captured_d = 1'b0;

          // Address window (sanity clamp)
          addr_lo_q     = addr_lo_i;
          addr_hi_q     = (addr_hi_i < addr_lo_i) ? addr_lo_i : addr_hi_i;

          // Bank order: pick first set bit in requested direction
          if (order_up_i) begin
            for (int i=0;i<NUM_BANKS;i++) if (bank_mask_i[i]) begin bank_d = i[BANK_W-1:0]; break; end
          end else begin
            for (int i=NUM_BANKS-1;i>=0;i--) if (bank_mask_i[i]) begin bank_d = i[BANK_W-1:0]; break; end
          end

          loop_total_q  = norm_loops(loops_i);
          loop_left_d   = norm_loops(loops_i);

          // PRBS seed
          prbs_d        = (bg_sel_i==3'd4) ? (prbs_seed_i == '0 ? {{(LFSR_W-1){1'b0}},1'b1} : prbs_seed_i) : prbs_seed_i;

          st_d          = S_PREP_BANK;
        end
      end

      // ----------------------------------------------------------------------
      S_PREP_BANK: begin
        // Skip bank if masked off
        if (!bank_mask_i[bank_q]) begin
          st_d = S_NEXT_BANK;
        end else begin
          // Start first step on this bank
          step_d = 8'd0;
          st_d   = S_PREP_STEP;
        end
      end

      // ----------------------------------------------------------------------
      S_PREP_STEP: begin
        logic v; op_e o0, o1; logic [1:0] k; logic dir;
        alg_get_step(algo_sel_i, step_q, v, dir, o0, o1, k);
        if (!v) begin
          // No more steps -> go next loop / bank
          st_d = S_NEXT_BANK;
        end else begin
          dir_up_d        = dir;
          step_op0_q      = o0;
          step_op1_q      = o1;
          step_ops_len_q  = k;

          // Prepare address counter
          addr_d = dir ? addr_lo_q : addr_hi_q;

          // Reset op index
          op_d   = 2'd0;

          st_d   = S_PREP_ADDR;
        end
      end

      // ----------------------------------------------------------------------
      S_PREP_ADDR: begin
        // Prepare PRBS per address if selected
        if (bg_sel_i == 3'd4) prbs_d = (dir_up_q ? prbs_q : prbs_q); // step after address completes
        st_d = S_ISSUE_OP;
      end

      // ----------------------------------------------------------------------
      S_ISSUE_OP: begin
        // Issue current op for (bank_q, addr_q)
        mem_req_o   = 1'b1;
        mem_addr_o  = addr_q;
        if (cur_op == OP_W0 || cur_op == OP_W1) begin
          mem_write_o = 1'b1;
          mem_wdata_o = wd;
          mem_wmask_o = all_wmask;
          // Writes are single-cycle; advance to next op
          st_d = S_NEXT_OP;
        end else if (cur_op == OP_R0 || cur_op == OP_R1) begin
          mem_write_o = 1'b0;
          // Prime compare pipeline slot 0
          if (RD_LATENCY == 0) begin
            // (not expected) immediate compare mode
          end
          // Fill stage 0
          exp_pipe_q[0] = wd;
          bnk_pipe_q[0] = bank_q;
          adr_pipe_q[0] = addr_q;
          stp_pipe_q[0] = step_q;
          opi_pipe_q[0] = {2'b00, op_q};
          // Push the pipeline tag forward next clock in sequential block
          st_d = S_WAIT_RD;
        end else begin
          // OP_NONE should not appear
          st_d = S_NEXT_OP;
        end
      end

      // ----------------------------------------------------------------------
      S_WAIT_RD: begin
        // Reads return after RD_LATENCY cycles; comparison is handled in the
        // sequential block on mem_rvalid_i (see below). When a read is issued,
        // we simply wait one cycle to continue issuing further ops (fully
        // serialized per address to keep it simple and robust).
        if (mem_rvalid_i) begin
          st_d = S_NEXT_OP;
        end
      end

      // ----------------------------------------------------------------------
      S_NEXT_OP: begin
        if (op_q + 2'd1 < step_ops_len_q) begin
          op_d = op_q + 2'd1;
          st_d = S_ISSUE_OP;
        end else begin
          // Address complete; advance address
          st_d = S_NEXT_ADDR;
        end
      end

      // ----------------------------------------------------------------------
      S_NEXT_ADDR: begin
        // Advance address; complete step if window exhausted
        if (dir_up_q) begin
          if (addr_q >= addr_hi_q) begin
            st_d  = S_NEXT_STEP;
          end else begin
            addr_d = addr_q + {{(ADDR_W-1){1'b0}},1'b1};
            if (bg_sel_i == 3'd4) prbs_d = PRBS_NEXT;
            op_d   = 2'd0;
            st_d   = S_PREP_ADDR;
          end
        end else begin
          if (addr_q <= addr_lo_q) begin
            st_d  = S_NEXT_STEP;
          end else begin
            addr_d = addr_q - {{(ADDR_W-1){1'b0}},1'b1};
            if (bg_sel_i == 3'd4) prbs_d = PRBS_NEXT;
            op_d   = 2'd0;
            st_d   = S_PREP_ADDR;
          end
        end
      end

      // ----------------------------------------------------------------------
      S_NEXT_STEP: begin
        step_d = step_q + 8'd1;
        st_d   = S_PREP_STEP;
      end

      // ----------------------------------------------------------------------
      S_NEXT_BANK: begin
        // Completed all steps on this bank; choose next bank or loop
        // Find next bank respecting iteration order and mask
        logic found; found = 1'b0;
        if (order_up_i) begin
          for (int i=bank_q+1; i<NUM_BANKS; i++) begin
            if (bank_mask_i[i]) begin bank_d = i[BANK_W-1:0]; found=1'b1; break; end
          end
        end else begin
          for (int i=bank_q; i>0; i--) begin
            if (bank_mask_i[i-1]) begin bank_d = (i-1)[BANK_W-1:0]; found=1'b1; break; end
          end
        end

        if (found) begin
          st_d = S_PREP_BANK;
        end else begin
          // Completed all banks in this loop
          if (loop_left_q > 16'd1) begin
            loop_left_d = loop_left_q - 16'd1;
            // Restart from first bank again
            if (order_up_i) begin
              for (int i=0;i<NUM_BANKS;i++) if (bank_mask_i[i]) begin bank_d = i[BANK_W-1:0]; break; end
            end else begin
              for (int i=NUM_BANKS-1;i>=0;i--) if (bank_mask_i[i]) begin bank_d = i[BANK_W-1:0]; break; end
            end
            st_d = S_PREP_BANK;
          end else begin
            // Fully complete
            busy_d       = 1'b0;
            done_pulse_d = 1'b1;
            st_d         = S_DONE;
          end
        end
      end

      // ----------------------------------------------------------------------
      S_PAUSED: begin
        // Wait for resume
        if (resume_i) begin
          st_d = S_ISSUE_OP; // re-issue on same address/op
        end
      end

      // ----------------------------------------------------------------------
      S_DONE: begin
        // Hold until next start edge (handled in IDLE)
        st_d = S_IDLE;
      end

      default: st_d = S_IDLE;
    endcase

    // Stop/Pause policy on fail (decision made in sequential compare below sets pass_q/fail_count)
    if (pass_q == 1'b0) begin
      if (stop_on_fail_i) begin
        // Hard stop at end of current cycle
        busy_d       = 1'b0;
        done_pulse_d = 1'b1;
        st_d         = S_DONE;
      end else if (pause_on_fail_i && st_q != S_PAUSED) begin
        st_d = S_PAUSED;
      end
    end
  end

  // ========================= Compare pipeline & failure capture ==============
  // Shift pipeline and compare at tail when mem_rvalid_i asserted.
  // We keep simple serialization so at most one read outstanding at a time.

  // Synchronous region
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      st_q         <= S_IDLE;
      bank_q       <= '0;
      step_q       <= 8'd0;
      op_q         <= 2'd0;
      dir_up_q     <= 1'b1;
      addr_q       <= '0;
      addr_lo_q    <= '0;
      addr_hi_q    <= {ADDR_W{1'b1}};
      prbs_q       <= '0;

      busy_q       <= 1'b0;
      pass_q       <= 1'b1;
      done_o       <= 1'b0;

      loop_total_q <= 16'd1;
      loop_left_q  <= 16'd1;

      fail_count_q    <= 32'd0;
      fail_captured_q <= 1'b0;
      fail_bank_q     <= '0;
      fail_addr_q     <= '0;
      fail_step_q     <= '0;
      fail_op_idx_q   <= '0;
      fail_exp_q      <= '0;
      fail_obs_q      <= '0;

      // Pipeline clear
      for (int s=0;s<RD_LATENCY;s++) begin
        rd_pipe_q[s] <= 1'b0;
        exp_pipe_q[s]<= '0;
        bnk_pipe_q[s]<= '0;
        adr_pipe_q[s]<= '0;
        stp_pipe_q[s]<= '0;
        opi_pipe_q[s]<= '0;
      end
    end else begin
      st_q    <= st_d;
      bank_q  <= bank_d;
      step_q  <= step_d;
      op_q    <= op_d;
      dir_up_q<= dir_up_d;
      addr_q  <= addr_d;
      prbs_q  <= prbs_d;

      busy_q  <= busy_d;
      pass_q  <= pass_d;

      // one-cycle pulse
      done_o  <= done_pulse_d;

      loop_left_q <= loop_left_d;

      // Capture expected and tags into stage 0 on read issue (S_ISSUE_OP path)
      // Shift pipeline every cycle; simple one-deep valid shift.
      if (RD_LATENCY > 0) begin
        rd_pipe_q[0] <= (st_q == S_ISSUE_OP) && ( (step_op0_q==OP_R0 && op_q==2'd0) ||
                                                  (step_op0_q==OP_R1 && op_q==2'd0) ||
                                                  (step_op1_q==OP_R0 && op_q==2'd1) ||
                                                  (step_op1_q==OP_R1 && op_q==2'd1) );
        // Shift deeper stages
        for (int s=RD_LATENCY-1; s>0; s--) begin
          rd_pipe_q[s]   <= rd_pipe_q[s-1];
          exp_pipe_q[s]  <= exp_pipe_q[s-1];
          bnk_pipe_q[s]  <= bnk_pipe_q[s-1];
          adr_pipe_q[s]  <= adr_pipe_q[s-1];
          stp_pipe_q[s]  <= stp_pipe_q[s-1];
          opi_pipe_q[s]  <= opi_pipe_q[s-1];
        end
      end

      // Compare when memory returns valid (tail of pipe); we assume 1 outstanding read.
      if (mem_rvalid_i) begin
        logic mismatch;
        mismatch = (mem_rdata_i ^ exp_pipe_q[RD_LATENCY-1]) != '0;

        if (mismatch) begin
          pass_q        <= 1'b0;
          // Increment fail counter (saturating)
          if (fail_count_q != 32'hFFFF_FFFF) fail_count_q <= fail_count_q + 32'd1;

          if (!fail_captured_q) begin
            fail_captured_q <= 1'b1;
            fail_bank_q     <= bnk_pipe_q[RD_LATENCY-1];
            fail_addr_q     <= adr_pipe_q[RD_LATENCY-1];
            fail_step_q     <= stp_pipe_q[RD_LATENCY-1];
            fail_op_idx_q   <= opi_pipe_q[RD_LATENCY-1];
            fail_exp_q      <= exp_pipe_q[RD_LATENCY-1];
            fail_obs_q      <= mem_rdata_i;
          end
        end
      end

      // Latch first-fail/summary outputs (readback)
    end
  end

  // Export first-fail
  assign fail_valid_o = fail_captured_q;
  assign fail_bank_o  = fail_bank_q;
  assign fail_addr_o  = fail_addr_q;
  assign fail_step_o  = fail_step_q;
  assign fail_op_o    = fail_op_idx_q[3:0];
  assign fail_exp_o   = fail_exp_q;
  assign fail_obs_o   = fail_obs_q;

  // ========================= Simple safety assertions ========================
`ifdef ASSERT_ON
  // Address window is sane
  always_ff @(posedge clk_i) begin
    if (!rstn_i) begin end
    else begin
      assert (addr_hi_i >= addr_lo_i)
        else $error("mbist_ctrl: addr_hi_i < addr_lo_i.");
    end
  end

  // Only one read outstanding (design intent)
  // With serialized per-address ops, this should hold.
  property p_one_read_outstanding;
    @(posedge clk_i) disable iff(!rstn_i)
      (st_q == S_WAIT_RD) |-> ##1 !((st_q == S_WAIT_RD) && (st_d == S_WAIT_RD));
  endproperty

`endif

endmodule
`default_nettype wire
