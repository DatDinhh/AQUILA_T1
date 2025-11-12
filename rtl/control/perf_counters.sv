// ============================================================================
//  perf_counters.sv
//  Aquila Project — Parameterized Performance Counter Block
//
//  Interfaces
//  ----------
//  clk_i, rstn_i                 : single clock domain for counters
//  evt_pulse_i[EVT_W-1:0]        : one-hot-ish event pulses (1-cycle) in clk_i
//  active_i                      : qualify "active cycles" (e.g., array busy)
//  CSR (local, 32-bit words)     : single-cycle ready, no wait-states
//  irq_overflow_o                : level when any counter overflowed and enabled
//
//  Key features
//  ------------
//  - 64-bit (configurable) counters; per-counter enable, saturate/ wrap,
//    optional clear-on-read, and gate-by-active.
//  - Global cycle counters: MCYCLE (always), MACTIVE (when active_i).
//  - Atomic reads: write CTRL.snap=1 to capture a consistent snapshot of all
//    64-bit counters; subsequent reads return snapshot until cleared.
//  - Per-counter sticky overflow bits; vector readable via CSR; optional IRQ.
//  - Clean parametric addressing; single-cycle CSR access (ack combinational).
//
//  CSR map (word addresses; each word is 32b; '[]' = bit fields)
//  -------------------------------------------------------------
//   0x000 CTRL      [0]=enable, [1]=freeze, [2]=clear_all (self-clear),
//                    [3]=snapshot (self-clear), [4]=clr_overflow (self-clear),
//                    [5]=ovf_irq_en
//   0x001 STATUS    [0]=snap_valid, [1]=overflow_any
//   0x002 MCYCLE_LO
//   0x003 MCYCLE_HI
//   0x004 MACT_LO
//   0x005 MACT_HI
//   0x010.. CFG[i]  one per counter (i=0..NUM_CNTRS-1)
//                    [SEL_W-1:0]=evtsel, [8]=enable, [9]=saturate,
//                    [10]=clr_on_read, [11]=gate_active
//   0x100.. CNT[i]_LO (live-or-snap); 0x100 + 2*i
//   0x101.. CNT[i]_HI (live-or-snap); 0x101 + 2*i
//   0x180.. OVF_VEC[k] (k over 32-bit words) sticky overflow status
//
//  Notes
//  -----
//  - Reads return SNAPSHOT values when snap_valid=1; otherwise live.
//  - If CLR_ON_READ is set for a counter, reading the *HI* word clears it.
//  - To avoid torn 64-bit reads, prefer the snapshot flow.
//  - Integrate by mapping this "local CSR" onto your csr_block window.
//    The simple handshake is: csr_en_i selects the block; access completes
//    in one cycle with csr_ready_o=1.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module perf_counters #(
  // Event & counter geometry
  parameter int unsigned EVT_W        = 64,   // number of input events
  parameter int unsigned NUM_CNTRS    = 16,   // number of programmable counters
  parameter int unsigned CNT_W        = 64,   // width of each counter (<=64)
  // CSR addressing (word address width)
  parameter int unsigned CSR_ADDR_W   = 12
)(
  input  logic                      clk_i,
  input  logic                      rstn_i,        // synchronous, active-low

  // Event inputs (clk_i domain)
  input  logic [EVT_W-1:0]         evt_pulse_i,   // count rising pulses (1 clk)
  input  logic                      active_i,      // qualifies MACTIVE and optional gates

  // Local CSR (map/bridge to your csr_block)
  input  logic                      csr_en_i,      // 1 to select this block
  input  logic                      csr_we_i,      // 1 = write, 0 = read
  input  logic [CSR_ADDR_W-1:0]     csr_addr_i,    // WORD address (32-bit words)
  input  logic [31:0]               csr_wdata_i,
  input  logic [3:0]                csr_be_i,      // byte enables (1=write that byte)
  output logic [31:0]               csr_rdata_o,
  output logic                      csr_ready_o,

  // Interrupt
  output logic                      irq_overflow_o
);

  // --------------------------- Static checks -------------------------------
  localparam int unsigned SEL_W = (EVT_W <= 1) ? 1 : $clog2(EVT_W);
  initial begin
    if (CNT_W == 0 || CNT_W > 64) $error("perf_counters: CNT_W must be 1..64.");
    if (NUM_CNTRS == 0)           $error("perf_counters: NUM_CNTRS must be >=1.");
  end

  // ----------------------------- CSR decode --------------------------------
  // Word address map constants
  localparam int unsigned A_CTRL       = 12'h000;
  localparam int unsigned A_STATUS     = 12'h001;
  localparam int unsigned A_MCYCLE_LO  = 12'h002;
  localparam int unsigned A_MCYCLE_HI  = 12'h003;
  localparam int unsigned A_MACT_LO    = 12'h004;
  localparam int unsigned A_MACT_HI    = 12'h005;

  localparam int unsigned A_CFG_BASE   = 12'h010;                   // + i
  localparam int unsigned A_CNT_BASE   = 12'h100;                   // + 2*i (lo/hi)
  localparam int unsigned A_OVF_BASE   = 12'h180;                   // + k (32-bit chunks)

  // ----------------------------- Controls ----------------------------------
  logic ctrl_enable_q, ctrl_enable_d;
  logic ctrl_freeze_q, ctrl_freeze_d;
  logic ctrl_irq_en_q, ctrl_irq_en_d;

  // Self-clearing pulses from CTRL writes
  logic pulse_clear_all, pulse_snapshot, pulse_clr_ovf;

  // --------------------------- Global counters -----------------------------
  logic [63:0] mcycle_q, mcycle_d;
  logic [63:0] mactive_q, mactive_d;

  // -------------------------- Programmable cntrs ---------------------------
  typedef struct packed {
    logic [SEL_W-1:0] evtsel;     // which event line to count
    logic             en;         // per-counter enable
    logic             sat;        // 1=saturate at all-ones; 0=wrap
    logic             clr_on_rd;  // clear on read of HI word
    logic             gate_active;// AND with active_i
  } cfg_t;

  cfg_t cfg_q   [NUM_CNTRS];
  cfg_t cfg_d   [NUM_CNTRS];

  logic [63:0] cnt_q [NUM_CNTRS];
  logic [63:0] cnt_d [NUM_CNTRS];

  logic        ovf_q [NUM_CNTRS]; // sticky overflow per counter
  logic        ovf_d [NUM_CNTRS];

  // ------------------------- Snapshot registers ----------------------------
  logic              snap_valid_q, snap_valid_d;
  logic [63:0]       snap_mcycle_q, snap_mcycle_d;
  logic [63:0]       snap_mactive_q, snap_mactive_d;
  logic [63:0]       snap_cnt_q [NUM_CNTRS];
  logic [63:0]       snap_cnt_d [NUM_CNTRS];

  // -------------------------- CSR ack & data -------------------------------
  assign csr_ready_o = csr_en_i;  // single-cycle access

  // Utility: write strobes per byte
  function automatic logic [31:0] wmask(input logic [3:0] be);
    logic [31:0] m;
    m = { {8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}} };
    return m;
  endfunction

  // -------------------------- Global counters logic ------------------------
  // Increment rules:
  // - mcycle: increments every cycle when ctrl_enable (freeze disables)
  // - mactive: increments when ctrl_enable & active_i
  // - freeze: holds values (no increment)
  // - clear_all pulse clears both

  always_comb begin
    mcycle_d  = mcycle_q;
    mactive_d = mactive_q;

    if (ctrl_enable_q && !ctrl_freeze_q) begin
      mcycle_d  = mcycle_q + 64'd1;
      if (active_i) mactive_d = mactive_q + 64'd1;
    end

    if (pulse_clear_all) begin
      mcycle_d  = 64'd0;
      mactive_d = 64'd0;
    end
  end

  // -------------------------- Counter increment core -----------------------
  for (genvar i=0; i<NUM_CNTRS; i++) begin : g_cnt
    // Selected event pulse (1-cycle)
    wire ev_pulse = evt_pulse_i[cfg_q[i].evtsel];
    // Optional gating by active_i
    wire do_count = ctrl_enable_q && cfg_q[i].en && (!ctrl_freeze_q) &&
                    (cfg_q[i].gate_active ? active_i : 1'b1) &&
                    ev_pulse;

    always_comb begin
      cnt_d[i] = cnt_q[i];
      ovf_d[i] = ovf_q[i];

      if (do_count) begin
        if (cfg_q[i].sat) begin
          if (&cnt_q[i][CNT_W-1:0]) begin
            // already saturated
            ovf_d[i] = 1'b1;
          end else begin
            // increment with saturation
            logic [63:0] inc = cnt_q[i] + 64'd1;
            cnt_d[i][CNT_W-1:0] = inc[CNT_W-1:0];
            if (&inc[CNT_W-1:0]) ovf_d[i] = 1'b1; // reached max
          end
        end else begin
          // wrap and set overflow on wrap-around
          logic [63:0] next = cnt_q[i] + 64'd1;
          cnt_d[i][CNT_W-1:0] = next[CNT_W-1:0];
          if (next[CNT_W-1:0] == '0) ovf_d[i] = 1'b1;
        end
      end

      if (pulse_clear_all) begin
        cnt_d[i] = '0;
      end
      if (pulse_clr_ovf) begin
        ovf_d[i] = 1'b0;
      end
    end
  end

  // ------------------------------ Snapshot ---------------------------------
  // When CTRL.snapshot is written '1', latch all counters for coherent reads.
  // STATUS.snap_valid indicates the read path is serving snapshot values.
  always_comb begin
    snap_valid_d  = snap_valid_q;
    snap_mcycle_d = snap_mcycle_q;
    snap_mactive_d= snap_mactive_q;
    for (int i=0;i<NUM_CNTRS;i++) snap_cnt_d[i] = snap_cnt_q[i];

    if (pulse_snapshot) begin
      snap_valid_d   = 1'b1;
      snap_mcycle_d  = mcycle_q;
      snap_mactive_d = mactive_q;
      for (int i=0;i<NUM_CNTRS;i++) snap_cnt_d[i] = cnt_q[i];
    end
    // Clear snapshot when CTRL.snapshot is written again with clear_all OR
    // any explicit software action by writing CTRL.snapshot=0 with write mask
    // (handled below by writing CTRL with bit3=0 and others unchanged).
    // Here we keep it sticky until SW clears by rewriting CTRL with bit3=0
    // and 'pulse_snapshot' not asserted.
  end

  // ------------------------------- CSR R/W ---------------------------------
  // Default: hold previous controls; clear pulses are combinational single-shot
  always_comb begin
    ctrl_enable_d = ctrl_enable_q;
    ctrl_freeze_d = ctrl_freeze_q;
    ctrl_irq_en_d = ctrl_irq_en_q;

    pulse_clear_all = 1'b0;
    pulse_snapshot  = 1'b0;
    pulse_clr_ovf   = 1'b0;

    // Default: mirror current snapshot_valid; SW can clear it by writing CTRL
    // with bit[3]==0 and byte-enable covering that bit; implement below.

    // Config default hold
    for (int i=0;i<NUM_CNTRS;i++) cfg_d[i] = cfg_q[i];

    // CSR write handling
    if (csr_en_i && csr_we_i) begin
      logic [31:0] w = (csr_wdata_i & wmask(csr_be_i));
      // CTRL
      if (csr_addr_i == A_CTRL) begin
        // Bits: [0]=enable, [1]=freeze, [2]=clear_all (pulse), [3]=snapshot (pulse),
        //       [4]=clr_overflow (pulse), [5]=ovf_irq_en
        ctrl_enable_d = w[0];
        ctrl_freeze_d = w[1];
        ctrl_irq_en_d = w[5];

        if (w[2]) pulse_clear_all = 1'b1;
        if (w[3]) pulse_snapshot  = 1'b1;
        if (w[4]) pulse_clr_ovf   = 1'b1;

        // SW can clear snapshot by writing CTRL with bit3==0 and no pulse
        if (!w[3]) snap_valid_d = 1'b0;
      end
      // CFG[i]
      else if (csr_addr_i >= A_CFG_BASE && csr_addr_i < A_CFG_BASE + NUM_CNTRS) begin
        int unsigned i = csr_addr_i - A_CFG_BASE;
        cfg_d[i].evtsel      = w[SEL_W-1:0];
        cfg_d[i].en          = w[8];
        cfg_d[i].sat         = w[9];
        cfg_d[i].clr_on_rd   = w[10];
        cfg_d[i].gate_active = w[11];
      end
      // Writes to counter value are ignored (read-only by design).
      // If you need software-writable counters for testing, add a write path here.
    end
  end

  // CSR read mux (single cycle)
  always_comb begin
    csr_rdata_o = 32'h0;

    if (csr_en_i && !csr_we_i) begin
      case (csr_addr_i)
        A_CTRL: begin
          csr_rdata_o[0]  = ctrl_enable_q;
          csr_rdata_o[1]  = ctrl_freeze_q;
          csr_rdata_o[5]  = ctrl_irq_en_q;
        end
        A_STATUS: begin
          csr_rdata_o[0]  = snap_valid_q;
          // overflow_any bit
          logic ovf_any; ovf_any = 1'b0;
          for (int i=0;i<NUM_CNTRS;i++) ovf_any |= ovf_q[i];
          csr_rdata_o[1]  = ovf_any;
        end
        A_MCYCLE_LO: csr_rdata_o = (snap_valid_q ? snap_mcycle_q[31:0]  : mcycle_q[31:0]);
        A_MCYCLE_HI: csr_rdata_o = (snap_valid_q ? snap_mcycle_q[63:32] : mcycle_q[63:32]);
        A_MACT_LO  : csr_rdata_o = (snap_valid_q ? snap_mactive_q[31:0] : mactive_q[31:0]);
        A_MACT_HI  : csr_rdata_o = (snap_valid_q ? snap_mactive_q[63:32]: mactive_q[63:32]);
        default: begin
          // CFG[i]
          if (csr_addr_i >= A_CFG_BASE && csr_addr_i < A_CFG_BASE + NUM_CNTRS) begin
            int unsigned i = csr_addr_i - A_CFG_BASE;
            csr_rdata_o[SEL_W-1:0] = cfg_q[i].evtsel;
            csr_rdata_o[8]         = cfg_q[i].en;
            csr_rdata_o[9]         = cfg_q[i].sat;
            csr_rdata_o[10]        = cfg_q[i].clr_on_rd;
            csr_rdata_o[11]        = cfg_q[i].gate_active;
          end
          // CNT[i] LO/HI
          else if (csr_addr_i >= A_CNT_BASE && csr_addr_i < A_CNT_BASE + (2*NUM_CNTRS)) begin
            int unsigned off  = csr_addr_i - A_CNT_BASE;
            int unsigned i    = off >> 1;
            logic [63:0] val  = snap_valid_q ? snap_cnt_q[i] : cnt_q[i];
            csr_rdata_o       = (off[0] == 1'b0) ? val[31:0] : val[63:32];
          end
          // OVF vector (32 counters per word)
          else if (csr_addr_i >= A_OVF_BASE &&
                   csr_addr_i <  A_OVF_BASE + ((NUM_CNTRS+31)/32)) begin
            int unsigned k = csr_addr_i - A_OVF_BASE;
            for (int b=0; b<32; b++) begin
              int unsigned idx = k*32 + b;
              if (idx < NUM_CNTRS) csr_rdata_o[b] = ovf_q[idx];
            end
          end
        end
      endcase
    end
  end

  // Clear-on-read for counters: reading HI word clears that counter if configured.
  // (live or snapshot read both trigger clear of live when configured)
  always_comb begin
    // Defaults: no action
    for (int i=0;i<NUM_CNTRS;i++) begin end
    if (csr_en_i && !csr_we_i) begin
      if (csr_addr_i >= A_CNT_BASE && csr_addr_i < A_CNT_BASE + (2*NUM_CNTRS)) begin
        int unsigned off = csr_addr_i - A_CNT_BASE;
        int unsigned i   = off >> 1;
        if (off[0] == 1'b1) begin // HI word read
          if (cfg_q[i].clr_on_rd) begin
            // Clear in the sequential block below by detecting this condition
          end
        end
      end
    end
  end

  // --------------------------- Sequential regs -----------------------------
  // Track 'clear on read' strobes for counters (one-cycle pulse per HI read)
  logic [NUM_CNTRS-1:0] clr_on_rd_pulse;

  always_comb begin
    clr_on_rd_pulse = '0;
    if (csr_en_i && !csr_we_i) begin
      if (csr_addr_i >= A_CNT_BASE && csr_addr_i < A_CNT_BASE + (2*NUM_CNTRS)) begin
        int unsigned off = csr_addr_i - A_CNT_BASE;
        int unsigned i   = off >> 1;
        if (off[0] == 1'b1 && cfg_q[i].clr_on_rd) clr_on_rd_pulse[i] = 1'b1;
      end
    end
  end

  // Registers
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      ctrl_enable_q <= 1'b0;
      ctrl_freeze_q <= 1'b0;
      ctrl_irq_en_q <= 1'b0;
      mcycle_q      <= 64'd0;
      mactive_q     <= 64'd0;
      snap_valid_q  <= 1'b0;
      snap_mcycle_q <= 64'd0;
      snap_mactive_q<= 64'd0;
      for (int i=0;i<NUM_CNTRS;i++) begin
        cfg_q[i].evtsel      <= '0;
        cfg_q[i].en          <= 1'b0;
        cfg_q[i].sat         <= 1'b1;  // default saturating (safer for debug)
        cfg_q[i].clr_on_rd   <= 1'b0;
        cfg_q[i].gate_active <= 1'b0;
        cnt_q[i]             <= '0;
        ovf_q[i]             <= 1'b0;
        snap_cnt_q[i]        <= '0;
      end
    end else begin
      ctrl_enable_q <= ctrl_enable_d;
      ctrl_freeze_q <= ctrl_freeze_d;
      ctrl_irq_en_q <= ctrl_irq_en_d;

      mcycle_q      <= mcycle_d;
      mactive_q     <= mactive_d;

      snap_valid_q  <= snap_valid_d;
      snap_mcycle_q <= snap_mcycle_d;
      snap_mactive_q<= snap_mactive_d;

      for (int i=0;i<NUM_CNTRS;i++) begin
        cfg_q[i]   <= cfg_d[i];
        // Clear-on-read path takes priority over increment result
        if (clr_on_rd_pulse[i]) begin
          cnt_q[i] <= '0;
          ovf_q[i] <= 1'b0;
        end else begin
          cnt_q[i] <= cnt_d[i];
          ovf_q[i] <= ovf_d[i];
        end
      end
    end
  end

  // ---------------------------- Overflow IRQ -------------------------------
  logic ovf_any_q;
  always_comb begin
    ovf_any_q = 1'b0;
    for (int i=0;i<NUM_CNTRS;i++) ovf_any_q |= ovf_q[i];
  end
  assign irq_overflow_o = ctrl_irq_en_q & ovf_any_q;

  // ------------------------------- Assertions ------------------------------
`ifdef ASSERT_ON
  // CSR access completes within one cycle
  assert property (@(posedge clk_i) csr_en_i |-> csr_ready_o)
    else $error("perf_counters: CSR not ready in single cycle.");

  // SEL_W fit
  initial begin
    if (SEL_W > 16) $warning("perf_counters: SEL_W=%0d; ensure CSR field width is sufficient.", SEL_W);
  end

  // Freeze should hold counts constant (checked for mcycle)
  logic [63:0] mcycle_prev;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) mcycle_prev <= 64'd0;
    else begin
      if (ctrl_freeze_q) begin
        assert (mcycle_d == mcycle_q) else $error("perf_counters: mcycle advanced while frozen.");
      end
      mcycle_prev <= mcycle_q;
    end
  end
`endif

endmodule

`default_nettype wire
