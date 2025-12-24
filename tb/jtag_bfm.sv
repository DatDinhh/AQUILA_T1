// ============================================================================
//  tb/jtag_bfm.sv
//  Aquila TB - IEEE 1149.1 (JTAG) Bus Functional Model
// ----------------------------------------------------------------------------
//  Drives TCK/TMS/TDI/TRSTn, samples TDO, models TAP state, and provides
//  timing‑correct tasks for reset, IR/DR shifts, run‑test‑idle, and utilities.
//  Bit order for shifts is LSB‑first (IEEE 1149.1).
//
//  Usage (example):
//    jtag_bfm #(.TCK_HALF_NS(50.0)) jtag (
//      .tck_o(tck), .tms_o(tms), .tdi_o(tdi), .tdo_i(tdo), .trst_n_o(trst_n)
//    );
//
//    initial begin
//      jtag.tap_reset();
//      jtag.goto_run_test_idle();
//      logic [31:0] id;
//      jtag.read_idcode(id);
//      $display("IDCODE=0x%08h", id);
//    end
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module jtag_bfm #(
  // TCK half period (ns). TCK period = 2 * TCK_HALF_NS
  parameter real TCK_HALF_NS_DEFAULT = 50.0,    // 10 MHz default
  parameter bit  HAS_TRST            = 1'b1,    // Drive TRSTn if present
  parameter int  IR_MAX_BITS         = 64,      // Upper bound for convenience wrappers
  parameter int  DR_MAX_BITS         = 1024,    // Upper bound for convenience wrappers
  parameter int  VERBOSE             = 1        // 0=silent, 1=state msgs, 2=edges
)(
  output logic tck_o,
  output logic tms_o,
  output logic tdi_o,
  input  logic tdo_i,
  output logic trst_n_o
);

  // TAP State Machine

  typedef enum logic [4:0] {
    TLRESET        = 5'd0,
    RUN_TEST_IDLE  = 5'd1,
    SELECT_DR_SCAN = 5'd2,
    CAPTURE_DR     = 5'd3,
    SHIFT_DR       = 5'd4,
    EXIT1_DR       = 5'd5,
    PAUSE_DR       = 5'd6,
    EXIT2_DR       = 5'd7,
    UPDATE_DR      = 5'd8,
    SELECT_IR_SCAN = 5'd9,
    CAPTURE_IR     = 5'd10,
    SHIFT_IR       = 5'd11,
    EXIT1_IR       = 5'd12,
    PAUSE_IR       = 5'd13,
    EXIT2_IR       = 5'd14,
    UPDATE_IR      = 5'd15
  } tap_state_e;

  tap_state_e state_q;

  // Human‑readable name (for debug prints)
  function automatic string tap_name(tap_state_e s);
    case (s)
      TLRESET:        return "TLRESET";
      RUN_TEST_IDLE:  return "RUN_TEST_IDLE";
      SELECT_DR_SCAN: return "SELECT_DR_SCAN";
      CAPTURE_DR:     return "CAPTURE_DR";
      SHIFT_DR:       return "SHIFT_DR";
      EXIT1_DR:       return "EXIT1_DR";
      PAUSE_DR:       return "PAUSE_DR";
      EXIT2_DR:       return "EXIT2_DR";
      UPDATE_DR:      return "UPDATE_DR";
      SELECT_IR_SCAN: return "SELECT_IR_SCAN";
      CAPTURE_IR:     return "CAPTURE_IR";
      SHIFT_IR:       return "SHIFT_IR";
      EXIT1_IR:       return "EXIT1_IR";
      PAUSE_IR:       return "PAUSE_IR";
      EXIT2_IR:       return "EXIT2_IR";
      UPDATE_IR:      return "UPDATE_IR";
      default:        return "UNKNOWN";
    endcase
  endfunction

  // Next‑state function per IEEE 1149.1 (TMS sampled on TCK rising edge)
  function automatic tap_state_e tap_next(tap_state_e s, bit tms);
    unique case (s)
      TLRESET:        tap_next = (tms) ? TLRESET        : RUN_TEST_IDLE;
      RUN_TEST_IDLE:  tap_next = (tms) ? SELECT_DR_SCAN : RUN_TEST_IDLE;
      SELECT_DR_SCAN: tap_next = (tms) ? SELECT_IR_SCAN : CAPTURE_DR;
      CAPTURE_DR:     tap_next = (tms) ? EXIT1_DR       : SHIFT_DR;
      SHIFT_DR:       tap_next = (tms) ? EXIT1_DR       : SHIFT_DR;
      EXIT1_DR:       tap_next = (tms) ? UPDATE_DR      : PAUSE_DR;
      PAUSE_DR:       tap_next = (tms) ? EXIT2_DR       : PAUSE_DR;
      EXIT2_DR:       tap_next = (tms) ? UPDATE_DR      : SHIFT_DR;
      UPDATE_DR:      tap_next = (tms) ? SELECT_DR_SCAN : RUN_TEST_IDLE;
      SELECT_IR_SCAN: tap_next = (tms) ? TLRESET        : CAPTURE_IR;
      CAPTURE_IR:     tap_next = (tms) ? EXIT1_IR       : SHIFT_IR;
      SHIFT_IR:       tap_next = (tms) ? EXIT1_IR       : SHIFT_IR;
      EXIT1_IR:       tap_next = (tms) ? UPDATE_IR      : PAUSE_IR;
      PAUSE_IR:       tap_next = (tms) ? EXIT2_IR       : PAUSE_IR;
      EXIT2_IR:       tap_next = (tms) ? UPDATE_IR      : SHIFT_IR;
      UPDATE_IR:      tap_next = (tms) ? SELECT_DR_SCAN : RUN_TEST_IDLE;
      default:        tap_next = TLRESET;
    endcase
  endfunction

  // Timing control

  real tck_half_ns;

  // Drive defaults and reset TAP state
  initial begin
    tck_o     = 1'b0;
    tms_o     = 1'b1; // hold high => TLRESET if clocked
    tdi_o     = 1'b0;
    trst_n_o  = (HAS_TRST) ? 1'b1 : 1'bz; // default deasserted / high‑Z when not used
    tck_half_ns = TCK_HALF_NS_DEFAULT;
    state_q   = TLRESET;
    if (VERBOSE) $display("[%0t] JTAG BFM init: TCK_HALF_NS=%0.3f", $time, tck_half_ns);
  end

  // Set TCK frequency in MHz (period = 1000/mhz ns)
  task automatic set_tck_mhz(input real mhz);
    if (mhz <= 0.0) begin
      $error("JTAG BFM: invalid MHz %0.3f", mhz);
      return;
    end
    tck_half_ns = (1000.0/mhz)/2.0;
    if (VERBOSE) $display("[%0t] JTAG BFM: TCK set to %0.3f MHz (half=%0.3f ns)",
                          $time, mhz, tck_half_ns);
  endtask

  // Set TCK period (ns)
  task automatic set_tck_period_ns(input real period_ns);
    if (period_ns <= 0.0) begin
      $error("JTAG BFM: invalid TCK period %0.3f ns", period_ns);
      return;
    end
    tck_half_ns = period_ns/2.0;
    if (VERBOSE) $display("[%0t] JTAG BFM: TCK period set to %0.3f ns (half=%0.3f ns)",
                          $time, period_ns, tck_half_ns);
  endtask

  // One TCK cycle with given TMS/TDI (update on falling, sample on rising)
  task automatic tck_cycle(input bit tms, input bit tdi, output bit tdo_s);
    // Drive inputs while TCK is low (setup)
    tms_o = tms;
    tdi_o = tdi;
    #(tck_half_ns);
    tck_o = 1'b1;                     // Rising edge
    #(1e-3);                          // tiny delta to avoid race in zero‑delay sims
    tdo_s = tdo_i;                    // Sample on rising edge
    state_q = tap_next(state_q, tms); // Advance TAP
    if (VERBOSE > 1) $display("[%0t] JTAG: ↑ TCK  TMS=%0b TDI=%0b TDO=%0b  -> %s",
                              $time, tms, tdi, tdo_s, tap_name(state_q));
    #(tck_half_ns);
    tck_o = 1'b0;                     // Falling edge
    if (VERBOSE > 1) $display("[%0t] JTAG: ↓ TCK", $time);
  endtask

  // Clock N cycles with constant TMS/TDI (e.g., idle)
  task automatic tck_pulse_n(input int unsigned n, input bit tms, input bit tdi);
    bit dummy;
    for (int unsigned i=0; i<n; i++) begin
      tck_cycle(tms, tdi, dummy);
    end
  endtask


  // Public API - Reset / Navigation
  
  // Asynchronous TAP reset via TRSTn (if present), then to Run‑Test/Idle
  task automatic tap_reset_trst();
    if (!HAS_TRST) begin
      $error("JTAG BFM: TRSTn not available in this configuration.");
      return;
    end
    trst_n_o <= 1'b0;
    // Hold TRST low for a few TCK half‑periods
    #(5.0 * tck_half_ns);
    trst_n_o <= 1'b1;
    // TRST places TAP in TLRESET; move to Run‑Test/Idle
    state_q = TLRESET;
    // One cycle with TMS=0 to go to RTI
    bit sink;
    tck_cycle(1'b0, 1'b0, sink); // TLRESET -> RTI
    if (VERBOSE) $display("[%0t] JTAG: tap_reset_trst -> %s", $time, tap_name(state_q));
  endtask

  // Synchronous reset by driving TMS=1 for 5 TCKs (IEEE rule), then go to RTI
  task automatic tap_reset();
    bit sink;
    // Ensure TCK low and TMS high
    if (VERBOSE) $display("[%0t] JTAG: tap_reset (5*TMS=1)", $time);
    tms_o = 1'b1; tdi_o = 1'b0;
    tck_pulse_n(5, 1'b1, 1'b0); // Now guaranteed TLRESET
    // One 0 to enter RTI
    tck_cycle(1'b0, 1'b0, sink); // TLRESET -> RUN_TEST/IDLE
    if (VERBOSE) $display("[%0t] JTAG: state=%s", $time, tap_name(state_q));
  endtask

  // From anywhere, return to Run‑Test/Idle (safe path via 5*TMS=1)
  task automatic goto_run_test_idle();
    tap_reset(); // ensures RTI
  endtask

  // Stay in RTI for N cycles (Run‑Test/Idle)
  task automatic runtest_cycles(input int unsigned n);
    if (state_q != RUN_TEST_IDLE) begin
      if (VERBOSE) $display("[%0t] JTAG: runtest -> forcing RTI", $time);
      goto_run_test_idle();
    end
    tck_pulse_n(n, 1'b0, 1'b0); // TMS=0 keeps RTI
  endtask

  // Public API — IR/DR access (LSB‑first)
  // Move to SHIFT‑IR from RTI (keeps TAP correct through Select/Capture)
  task automatic enter_shift_ir();
    bit sink;
    if (state_q != RUN_TEST_IDLE) goto_run_test_idle();
    // RTI -> Select‑DR (1) -> Select‑IR (1) -> Capture‑IR (0) -> Shift‑IR (0)
    tck_cycle(1'b1, 1'b0, sink); // RTI -> SELECT_DR_SCAN
    tck_cycle(1'b1, 1'b0, sink); // SELECT_DR_SCAN -> SELECT_IR_SCAN
    tck_cycle(1'b0, 1'b0, sink); // SELECT_IR_SCAN -> CAPTURE_IR
    tck_cycle(1'b0, 1'b0, sink); // CAPTURE_IR -> SHIFT_IR
    if (VERBOSE) $display("[%0t] JTAG: entered SHIFT_IR", $time);
  endtask

  // Move to SHIFT‑DR from RTI
  task automatic enter_shift_dr();
    bit sink;
    if (state_q != RUN_TEST_IDLE) goto_run_test_idle();
    // RTI -> Select‑DR (1) -> Capture‑DR (0) -> Shift‑DR (0)
    tck_cycle(1'b1, 1'b0, sink); // RTI -> SELECT_DR_SCAN
    tck_cycle(1'b0, 1'b0, sink); // SELECT_DR_SCAN -> CAPTURE_DR
    tck_cycle(1'b0, 1'b0, sink); // CAPTURE_DR -> SHIFT_DR
    if (VERBOSE) $display("[%0t] JTAG: entered SHIFT_DR", $time);
  endtask

  typedef enum logic [1:0] { END_IDLE=2'b00, END_PAUSE=2'b01 } end_state_e;

  // Shift IR (LSB‑first). On the last bit, TMS is driven high to exit SHIFT_IR.
  // After Exit1‑IR, choose END_IDLE (Update‑IR -> RTI) or END_PAUSE (Pause‑IR).
  task automatic shift_ir(
      input  bit   din_lsb_first[],  // dynamic array of bits, LSB first
      input  int   nbits,
      output bit   dout_lsb_first[], // captured TDO, LSB first
      input  end_state_e end_state = END_IDLE
  );
    if (nbits <= 0) begin
      $error("JTAG BFM: shift_ir nbits must be > 0");
      return;
    end
    dout_lsb_first = new[nbits];
    enter_shift_ir();

    // Shift nbits: keep TMS=0 except for last bit (-> EXIT1_IR)
    bit tdo_s;
    for (int i=0; i<nbits; i++) begin
      bit last = (i == nbits-1);
      tck_cycle(last ? 1'b1 : 1'b0, din_lsb_first[i], tdo_s); // SHIFT_IR -> EXIT1_IR on last
      dout_lsb_first[i] = tdo_s;
    end

    // EXIT1_IR -> (PAUSE_IR or UPDATE_IR)
    bit sink;
    unique case (end_state)
      END_PAUSE: begin
        tck_cycle(1'b0, 1'b0, sink); // EXIT1_IR -> PAUSE_IR
        if (VERBOSE) $display("[%0t] JTAG: SHIFT_IR end -> PAUSE_IR", $time);
      end
      default: begin
        tck_cycle(1'b1, 1'b0, sink); // EXIT1_IR -> UPDATE_IR
        tck_cycle(1'b0, 1'b0, sink); // UPDATE_IR -> RTI
        if (VERBOSE) $display("[%0t] JTAG: SHIFT_IR end -> UPDATE_IR -> RTI", $time);
      end
    endcase
  endtask

  // Shift DR (LSB‑first). Similar semantics to shift_ir.
  task automatic shift_dr(
      input  bit   din_lsb_first[],
      input  int   nbits,
      output bit   dout_lsb_first[],
      input  end_state_e end_state = END_IDLE
  );
    if (nbits <= 0) begin
      $error("JTAG BFM: shift_dr nbits must be > 0");
      return;
    end
    dout_lsb_first = new[nbits];
    enter_shift_dr();

    bit tdo_s;
    for (int i=0; i<nbits; i++) begin
      bit last = (i == nbits-1);
      tck_cycle(last ? 1'b1 : 1'b0, din_lsb_first[i], tdo_s); // SHIFT_DR -> EXIT1_DR on last
      dout_lsb_first[i] = tdo_s;
    end

    bit sink;
    unique case (end_state)
      END_PAUSE: begin
        tck_cycle(1'b0, 1'b0, sink); // EXIT1_DR -> PAUSE_DR
        if (VERBOSE) $display("[%0t] JTAG: SHIFT_DR end -> PAUSE_DR", $time);
      end
      default: begin
        tck_cycle(1'b1, 1'b0, sink); // EXIT1_DR -> UPDATE_DR
        tck_cycle(1'b0, 1'b0, sink); // UPDATE_DR -> RTI
        if (VERBOSE) $display("[%0t] JTAG: SHIFT_DR end -> UPDATE_DR -> RTI", $time);
      end
    endcase
  endtask

  // Convenience: load an IR opcode (LSB‑first value with explicit length)
  task automatic load_ir(input logic [IR_MAX_BITS-1:0] opcode_lsb_first, input int ir_len);
    bit din[], dout[];
    if (ir_len <= 0 || ir_len > IR_MAX_BITS) begin
      $fatal(1, "JTAG BFM: load_ir invalid ir_len=%0d (max %0d)", ir_len, IR_MAX_BITS);
    end
    din = new[ir_len];
    for (int i=0; i<ir_len; i++) din[i] = opcode_lsb_first[i];
    shift_ir(din, ir_len, dout, END_IDLE);
  endtask

  // Convenience: shift DR with a packed vector (LSB‑first, up to DR_MAX_BITS)
  task automatic shift_dr_vec(input logic [DR_MAX_BITS-1:0] din_vec,
                              input int dr_len,
                              output logic [DR_MAX_BITS-1:0] dout_vec);
    bit din[], dout[];
    if (dr_len <= 0 || dr_len > DR_MAX_BITS) begin
      $fatal(1, "JTAG BFM: shift_dr_vec invalid dr_len=%0d (max %0d)", dr_len, DR_MAX_BITS);
    end
    din  = new[dr_len];
    dout = new[dr_len];
    for (int i=0; i<dr_len; i++) din[i] = din_vec[i];
    shift_dr(din, dr_len, dout, END_IDLE);
    dout_vec = '0;
    for (int i=0; i<dr_len; i++) dout_vec[i] = dout[i];
  endtask

  // Read 32‑bit IDCODE (assumes the device defaults to IDCODE after reset; if not,
  // call load_ir(IDCODE_OPCODE, IR_LEN) first).
  task automatic read_idcode(output logic [31:0] idcode);
    bit din[32], dout[32];
    // Shift out DR with zeros on TDI, 32 bits
    for (int i=0; i<32; i++) din[i] = 1'b0;
    enter_shift_dr();
    bit tdo_s;
    for (int i=0; i<32; i++) begin
      bit last = (i == 31);
      tck_cycle(last ? 1'b1 : 1'b0, din[i], tdo_s);
      dout[i] = tdo_s; // capture LSB‑first
    end
    // EXIT1_DR -> UPDATE_DR -> RTI
    bit sink;
    tck_cycle(1'b1, 1'b0, sink);
    tck_cycle(1'b0, 1'b0, sink);

    // Assemble LSB‑first
    idcode = '0;
    for (int i=0; i<32; i++) idcode[i] = dout[i];

    if (VERBOSE) $display("[%0t] JTAG: IDCODE read 0x%08h", $time, idcode);
  endtask


  // Optional helpers
  
  // Exit from PAUSE_DR/PAUSE_IR back to RTI (via EXIT2 -> UPDATE -> RTI)
  task automatic pause_to_idle();
    bit sink;
    if (state_q == PAUSE_DR) begin
      tck_cycle(1'b1, 1'b0, sink); // PAUSE_DR -> EXIT2_DR
      tck_cycle(1'b1, 1'b0, sink); // EXIT2_DR -> UPDATE_DR
      tck_cycle(1'b0, 1'b0, sink); // UPDATE_DR -> RTI
    end else if (state_q == PAUSE_IR) begin
      tck_cycle(1'b1, 1'b0, sink); // PAUSE_IR -> EXIT2_IR
      tck_cycle(1'b1, 1'b0, sink); // EXIT2_IR -> UPDATE_IR
      tck_cycle(1'b0, 1'b0, sink); // UPDATE_IR -> RTI
    end else begin
      if (VERBOSE) $display("[%0t] JTAG: pause_to_idle called outside PAUSE_* (state=%s)",
                            $time, tap_name(state_q));
    end
  endtask

  // Drive an arbitrary TMS sequence (MSB‑first in the vector) with constant TDI=0.
  // Useful for debugging unusual TAP paths.
  task automatic drive_tms_sequence(input logic [255:0] tms_bits, input int nbits);
    bit sink;
    for (int i=nbits-1; i>=0; i--) begin
      tck_cycle(tms_bits[i], 1'b0, sink);
    end
  endtask

endmodule

`default_nettype wire
