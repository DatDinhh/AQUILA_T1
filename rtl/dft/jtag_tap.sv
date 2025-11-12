// ============================================================================
//  jtag_tap.sv
//  IEEE 1149.1 JTAG TAP Controller (TCK domain)
//
//  - Implements the 16-state TAP controller FSM
//  - Parametric IR (default 5 bits) with BYPASS/IDCODE + user instructions
//  - DR chain mux for: BYPASS, IDCODE, BSR, DEBUG, MBIST (plug-in ports)
//  - TMS/TDI sampled on TCK rising edge; TDO updates on TCK falling edge
//  - TRSTn (optional) and "5*TMS=1" reset supported
//  - Clean per-state strobes and DR selects
//
//  Timing notes (1149.1):
//   * Sample TMS/TDI on TCK↑
//   * Change TDO on TCK↓ (valid at next TCK↑)
//   * Drive TDO only in SHIFT_DR and SHIFT_IR (tdo_oe_o)
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module jtag_tap #(
  // ---------------- IR / mandatory instructions ----------------
  parameter int unsigned IR_LEN        = 5,
  // Instruction opcodes (IR_LEN wide). BYPASS must be all-ones by spec.
  parameter logic [IR_LEN-1:0] OPC_EXTEST  = 'h00,
  parameter logic [IR_LEN-1:0] OPC_SAMPLE  = 'h01,  // SAMPLE / PRELOAD
  parameter logic [IR_LEN-1:0] OPC_IDCODE  = 'h02,
  parameter logic [IR_LEN-1:0] OPC_DEBUG   = 'h08,  // user
  parameter logic [IR_LEN-1:0] OPC_MBIST   = 'h09,  // user
  parameter logic [IR_LEN-1:0] OPC_BYPASS  = {IR_LEN{1'b1}},

  // ---------------- IDCODE (LSB-first shift) -------------------
  parameter logic [31:0]       IDCODE_VAL  = 32'h1234_567B  // bit0 must be 1 per 1149.1
)(
  // JTAG pins
  input  wire  tck_i,     // Test Clock
  input  wire  tms_i,     // Test Mode Select
  input  wire  tdi_i,     // Test Data In
  output wire  tdo_o,     // Test Data Out
  output wire  tdo_oe_o,  // Output-enable (1 in SHIFT states)
  input  wire  trst_n_i,  // (optional) active-low TAP reset (tie high if unused)

  // ---------------- DR client ports (TCK domain) ----------------
  // Boundary-Scan Register (BSR)
  output logic       bsr_sel_o,        // active when current IR selects BSR (EXTEST/SAMPLE)
  output logic       bsr_capture_o,    // pulses in CAPTURE_DR
  output logic       bsr_update_o,     // pulses in UPDATE_DR
  output logic       bsr_shift_o,      // 1 while in SHIFT_DR with BSR selected
  output logic       bsr_tdi_o,        // serial in to BSR
  input  logic       bsr_tdo_i,        // serial out from BSR

  // DEBUG DR (user)
  output logic       dbg_sel_o,
  output logic       dbg_capture_o,
  output logic       dbg_update_o,
  output logic       dbg_shift_o,
  output logic       dbg_tdi_o,
  input  logic       dbg_tdo_i,

  // MBIST DR (user)
  output logic       mbist_sel_o,
  output logic       mbist_capture_o,
  output logic       mbist_update_o,
  output logic       mbist_shift_o,
  output logic       mbist_tdi_o,
  input  logic       mbist_tdo_i,

  // ---------------- Useful state strobes (TCK domain) ------------
  output logic       test_logic_reset_o,
  output logic       run_test_idle_o,
  output logic       select_dr_scan_o, capture_dr_o, shift_dr_o, exit1_dr_o,
                     pause_dr_o,       exit2_dr_o,   update_dr_o,
  output logic       select_ir_scan_o, capture_ir_o, shift_ir_o, exit1_ir_o,
                     pause_ir_o,       exit2_ir_o,   update_ir_o,

  // Latched instruction (decoded IR)
  output logic [IR_LEN-1:0] ir_o
);

  // --------------------------------------------------------------------------
  // Reset handling (TRSTn and 5*TMS=1 detector)
  // --------------------------------------------------------------------------
  // Edge-aligned to TCK; TAP resets when trst_n_i==0 or after 5 consecutive TCK
  // cycles with TMS=1 while TCK toggles.
  logic [4:0] tms_hist_q;
  wire tms_five_ones = &tms_hist_q;

  always_ff @(posedge tck_i or negedge trst_n_i) begin
    if (!trst_n_i) tms_hist_q <= 5'b0;
    else           tms_hist_q <= {tms_hist_q[3:0], tms_i};
  end

  // Combined reset
  wire tap_rst = (~trst_n_i) | tms_five_ones;

  // --------------------------------------------------------------------------
  // TAP state machine (16 states)
  // --------------------------------------------------------------------------
  typedef enum logic [3:0] {
    TL_RESET    = 4'd0,
    RUN_IDLE    = 4'd1,
    SEL_DR      = 4'd2,
    CAP_DR      = 4'd3,
    SHIFT_DR    = 4'd4,
    EXIT1_DR    = 4'd5,
    PAUSE_DR    = 4'd6,
    EXIT2_DR    = 4'd7,
    UPDATE_DR   = 4'd8,
    SEL_IR      = 4'd9,
    CAP_IR      = 4'd10,
    SHIFT_IR    = 4'd11,
    EXIT1_IR    = 4'd12,
    PAUSE_IR    = 4'd13,
    EXIT2_IR    = 4'd14,
    UPDATE_IR   = 4'd15
  } tap_state_e;

  tap_state_e st_q, st_d;

  // State register: sample on TCK↑
  always_ff @(posedge tck_i or posedge tap_rst) begin
    if (tap_rst) st_q <= TL_RESET;
    else         st_q <= st_d;
  end

  // Next-state logic (per 1149.1)
  always_comb begin
    unique case (st_q)
      TL_RESET : st_d = (tms_i ? TL_RESET : RUN_IDLE);
      RUN_IDLE : st_d = (tms_i ? SEL_DR   : RUN_IDLE);
      SEL_DR   : st_d = (tms_i ? SEL_IR   : CAP_DR);
      CAP_DR   : st_d = (tms_i ? EXIT1_DR : SHIFT_DR);
      SHIFT_DR : st_d = (tms_i ? EXIT1_DR : SHIFT_DR);
      EXIT1_DR : st_d = (tms_i ? UPDATE_DR: PAUSE_DR);
      PAUSE_DR : st_d = (tms_i ? EXIT2_DR : PAUSE_DR);
      EXIT2_DR : st_d = (tms_i ? UPDATE_DR: SHIFT_DR);
      UPDATE_DR: st_d = (tms_i ? SEL_DR   : RUN_IDLE);
      SEL_IR   : st_d = (tms_i ? TL_RESET : CAP_IR);
      CAP_IR   : st_d = (tms_i ? EXIT1_IR : SHIFT_IR);
      SHIFT_IR : st_d = (tms_i ? EXIT1_IR : SHIFT_IR);
      EXIT1_IR : st_d = (tms_i ? UPDATE_IR: PAUSE_IR);
      PAUSE_IR : st_d = (tms_i ? EXIT2_IR : PAUSE_IR);
      EXIT2_IR : st_d = (tms_i ? UPDATE_IR: SHIFT_IR);
      UPDATE_IR: st_d = (tms_i ? SEL_DR   : RUN_IDLE);
      default  : st_d = TL_RESET;
    endcase
  end

  // One-hot strobes (combinational)
  always_comb begin
    test_logic_reset_o = (st_q == TL_RESET);
    run_test_idle_o    = (st_q == RUN_IDLE);

    select_dr_scan_o   = (st_q == SEL_DR);
    capture_dr_o       = (st_q == CAP_DR);
    shift_dr_o         = (st_q == SHIFT_DR);
    exit1_dr_o         = (st_q == EXIT1_DR);
    pause_dr_o         = (st_q == PAUSE_DR);
    exit2_dr_o         = (st_q == EXIT2_DR);
    update_dr_o        = (st_q == UPDATE_DR);

    select_ir_scan_o   = (st_q == SEL_IR);
    capture_ir_o       = (st_q == CAP_IR);
    shift_ir_o         = (st_q == SHIFT_IR);
    exit1_ir_o         = (st_q == EXIT1_IR);
    pause_ir_o         = (st_q == PAUSE_IR);
    exit2_ir_o         = (st_q == EXIT2_IR);
    update_ir_o        = (st_q == UPDATE_IR);
  end

  // --------------------------------------------------------------------------
  // Instruction Register (IR)
  // --------------------------------------------------------------------------
  // IR shift register and latched current instruction.
  logic [IR_LEN-1:0] ir_shift_q, ir_latched_q;

  // Capture-IR loads 01..1 pattern (per 1149.1)
  function automatic logic [IR_LEN-1:0] ir_capture_pattern;
    logic [IR_LEN-1:0] v; v = {IR_LEN{1'b1}}; v[0] = 1'b1; // '01...1' (LSB=1)
    return v;
  endfunction

  // Shift/capture on TCK↑, update on UPDATE_IR (TCK↑)
  always_ff @(posedge tck_i or posedge tap_rst) begin
    if (tap_rst) begin
      ir_shift_q   <= ir_capture_pattern();
      ir_latched_q <= OPC_IDCODE;        // power-on instruction: IDCODE (typical)
    end else begin
      if (capture_ir_o) ir_shift_q <= ir_capture_pattern();
      else if (shift_ir_o) ir_shift_q <= {tdi_i, ir_shift_q[IR_LEN-1:1]}; // LSB-first shift

      if (update_ir_o) ir_latched_q <= ir_shift_q;
    end
  end
  assign ir_o = ir_latched_q;

  // --------------------------------------------------------------------------
  // Data registers
  // --------------------------------------------------------------------------
  // BYPASS: 1-bit, captured as 0, shifts TDI -> TDO
  logic bypass_q;
  always_ff @(posedge tck_i or posedge tap_rst) begin
    if (tap_rst) bypass_q <= 1'b0;
    else if (capture_dr_o && ir_latched_q == OPC_BYPASS) bypass_q <= 1'b0;
    else if (shift_dr_o   && ir_latched_q == OPC_BYPASS) bypass_q <= tdi_i;
  end

  // IDCODE: 32-bit, captured with fixed value, shifted LSB-first
  logic [31:0] idcode_shift_q;
  always_ff @(posedge tck_i or posedge tap_rst) begin
    if (tap_rst) idcode_shift_q <= IDCODE_VAL;
    else if (capture_dr_o && ir_latched_q == OPC_IDCODE) idcode_shift_q <= IDCODE_VAL;
    else if (shift_dr_o   && ir_latched_q == OPC_IDCODE) idcode_shift_q <= {tdi_i, idcode_shift_q[31:1]};
  end

  // -------------------- BSR / DEBUG / MBIST plug-ins -----------------------
  // Common strobes/selects
  wire use_bsr   = (ir_latched_q == OPC_EXTEST) || (ir_latched_q == OPC_SAMPLE);
  wire use_id    = (ir_latched_q == OPC_IDCODE);
  wire use_byp   = (ir_latched_q == OPC_BYPASS);
  wire use_dbg   = (ir_latched_q == OPC_DEBUG);
  wire use_mbist = (ir_latched_q == OPC_MBIST);

  // Export select + strobes (TCK domain)
  assign bsr_sel_o     = use_bsr;
  assign bsr_capture_o = capture_dr_o & use_bsr;
  assign bsr_update_o  = update_dr_o  & use_bsr;
  assign bsr_shift_o   = shift_dr_o   & use_bsr;
  assign bsr_tdi_o     = tdi_i;

  assign dbg_sel_o     = use_dbg;
  assign dbg_capture_o = capture_dr_o & use_dbg;
  assign dbg_update_o  = update_dr_o  & use_dbg;
  assign dbg_shift_o   = shift_dr_o   & use_dbg;
  assign dbg_tdi_o     = tdi_i;

  assign mbist_sel_o     = use_mbist;
  assign mbist_capture_o = capture_dr_o & use_mbist;
  assign mbist_update_o  = update_dr_o  & use_mbist;
  assign mbist_shift_o   = shift_dr_o   & use_mbist;
  assign mbist_tdi_o     = tdi_i;

  // --------------------------------------------------------------------------
  // TDO Muxing (changes on TCK↓)
  // --------------------------------------------------------------------------
  // During SHIFT_IR: TDO = IR shift LSB
  // During SHIFT_DR: TDO = selected DR LSB (IDCODE/BYPASS/BSR/DBG/MBIST)
  // Otherwise, TDO is don't-care and tdo_oe_o==0
  logic tdo_q;        // registered on TCK↓ (for clean timing)
  logic tdo_sel_w;    // combinational: next TDO bit
  logic tdo_oe_w;

  always_comb begin
    tdo_sel_w = 1'b0;
    tdo_oe_w  = 1'b0;

    if (shift_ir_o) begin
      tdo_sel_w = ir_shift_q[0];
      tdo_oe_w  = 1'b1;
    end else if (shift_dr_o) begin
      tdo_oe_w  = 1'b1;
      unique case (1'b1)
        use_byp  : tdo_sel_w = bypass_q;
        use_id   : tdo_sel_w = idcode_shift_q[0];
        use_bsr  : tdo_sel_w = bsr_tdo_i;
        use_dbg  : tdo_sel_w = dbg_tdo_i;
        use_mbist: tdo_sel_w = mbist_tdo_i;
        default  : tdo_sel_w = 1'b0;
      endcase
    end
  end

  // Register on TCK falling edge (TDO changes on ↓)
  always_ff @(negedge tck_i or posedge tap_rst) begin
    if (tap_rst) tdo_q <= 1'b0;
    else         tdo_q <= tdo_sel_w;
  end

  assign tdo_o    = tdo_q;
  assign tdo_oe_o = tdo_oe_w;

  // --------------------------------------------------------------------------
  // Synthesis guards and basic assertions (can be removed if undesired)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // BYPASS opcode must be all ones
  initial begin
    if (OPC_BYPASS !== {IR_LEN{1'b1}})
      $error("jtag_tap: OPC_BYPASS must be all ones.");
    // IDCODE bit0 must be 1 by spec
    if (IDCODE_VAL[0] !== 1'b1)
      $error("jtag_tap: IDCODE_VAL bit[0] must be 1.");
  end

  // TDO should be enabled only in shift states
  always_ff @(posedge tck_i) begin
    if (!(shift_ir_o || shift_dr_o)) begin
      assert (tdo_oe_w == 1'b0) else $error("jtag_tap: tdo_oe set outside shift.");
    end
  end
`endif

endmodule

`default_nettype wire
