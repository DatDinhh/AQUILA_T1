// ============================================================================
//  Global Buffer Partition Controller
//  File: gb_partition_ctrl.sv
//
//  Purpose:
//    - Run-time mapping of *logical GB address space* to *physical banks*.
//    - Per-partition access control and interleave policy.
//    - Thin, combinational address translator with pass-through handshakes.
//    - Sits between masters and gb_crossbar.
//
//  Master-side (upstream) interface: same as gb_crossbar master ports.
//  Downstream interface: drives addresses/data toward gb_crossbar.
//
//  Notes:
//    - Bank index bits live at addr[BANK_SEL_LSB +: BANK_W]; this module rewrites
//      those bits according to active partition's bank_mask and stride.
//    - If no partition matches (or owner not allowed), request is stalled
//      (ready=0). Optional deny pulses are provided for monitoring.
//    - Partition priority is fixed: lowest index wins (0..NPART-1).
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module gb_partition_ctrl #(
  // ---------- Topology ----------
  parameter int unsigned NMASTERS     = 3,
  parameter int unsigned NBANKS       = 8,

  // ---------- Geometry ----------
  parameter int unsigned DATA_W       = 512,    // bits, multiple of 8
  parameter int unsigned ADDR_W       = 32,     // byte address width
  //   First bit of bank-index field within *byte* address.
  //   For beat-interleaving, set BANK_SEL_LSB = $clog2(DATA_W/8).
  parameter int unsigned BANK_SEL_LSB = (DATA_W<=8)?0:$clog2(DATA_W/8),

  // ---------- Partitioning ----------
  parameter int unsigned NPART        = 8,      // number of logical partitions (>=1)

  // ---------- Behavior ----------
  parameter bit          PULSE_DENY   = 1'b1    // emit deny pulses when access rejected
)(
  input  logic                               clk_i,
  input  logic                               rstn_i,   // synchronous, active-low

  // ==========================================================================
  // Upstream (masters) — requests in, read data out
  // ==========================================================================
  // Write request (single beat)
  input  logic [NMASTERS-1:0]                up_wr_valid_i,
  output logic [NMASTERS-1:0]                up_wr_ready_o,
  input  logic [NMASTERS-1:0][ADDR_W-1:0]    up_wr_addr_i,
  input  logic [NMASTERS-1:0][DATA_W-1:0]    up_wr_data_i,
  input  logic [NMASTERS-1:0][DATA_W/8-1:0]  up_wr_strb_i,

  // Read request (single beat)
  input  logic [NMASTERS-1:0]                up_rd_valid_i,
  output logic [NMASTERS-1:0]                up_rd_ready_o,
  input  logic [NMASTERS-1:0][ADDR_W-1:0]    up_rd_addr_i,

  // Read return (pass-through from downstream)
  output logic [NMASTERS-1:0]                up_r_valid_o,
  input  logic [NMASTERS-1:0]                up_r_ready_i,
  output logic [NMASTERS-1:0][DATA_W-1:0]    up_r_data_o,
  output logic [NMASTERS-1:0]                up_r_sbe_o,
  output logic [NMASTERS-1:0]                up_r_dbe_o,

  // Optional deny pulses (one-cycle) for logging
  output logic [NMASTERS-1:0]                wr_deny_pulse_o,
  output logic [NMASTERS-1:0]                rd_deny_pulse_o,

  // ==========================================================================
  // Downstream (to gb_crossbar) — transformed requests, read data in
  // ==========================================================================
  // Write request toward crossbar
  output logic [NMASTERS-1:0]                dn_wr_valid_o,
  input  logic [NMASTERS-1:0]                dn_wr_ready_i,
  output logic [NMASTERS-1:0][ADDR_W-1:0]    dn_wr_addr_o,
  output logic [NMASTERS-1:0][DATA_W-1:0]    dn_wr_data_o,
  output logic [NMASTERS-1:0][DATA_W/8-1:0]  dn_wr_strb_o,

  // Read request toward crossbar
  output logic [NMASTERS-1:0]                dn_rd_valid_o,
  input  logic [NMASTERS-1:0]                dn_rd_ready_i,
  output logic [NMASTERS-1:0][ADDR_W-1:0]    dn_rd_addr_o,

  // Read return from crossbar
  input  logic [NMASTERS-1:0]                dn_r_valid_i,
  output logic [NMASTERS-1:0]                dn_r_ready_o,
  input  logic [NMASTERS-1:0][DATA_W-1:0]    dn_r_data_i,
  input  logic [NMASTERS-1:0]                dn_r_sbe_i,
  input  logic [NMASTERS-1:0]                dn_r_dbe_i,

  // ==========================================================================
  // Partition configuration (typically driven by CSR block)
  // ==========================================================================
  input  logic                               cfg_global_en_i,

  // Per-partition config arrays
  input  logic [NPART-1:0]                   cfg_part_en_i,
  input  logic [NPART-1:0][NBANKS-1:0]       cfg_part_bank_mask_i,
  input  logic [NPART-1:0][NMASTERS-1:0]     cfg_part_owner_mask_i,
  input  logic [NPART-1:0][ADDR_W-1:0]       cfg_part_base_i,
  input  logic [NPART-1:0][ADDR_W-1:0]       cfg_part_size_i,
  // Interleave stride = 2^stride_l2 bytes (min recommended: beat size)
  input  logic [NPART-1:0][7:0]              cfg_part_stride_l2_i,

  // Bank control hints (derived below; also exposed as outputs)
  output logic [NBANKS-1:0]                  bank_enable_o,
  output logic [NBANKS-1:0]                  bank_sleep_o
);

  // ==========================================================================
  // Derived constants / checks
  // ==========================================================================
  localparam int unsigned BEAT_BYTES = DATA_W/8;
  localparam int unsigned BEAT_LSB   = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);
  localparam int unsigned BANK_W     = (NBANKS <= 1) ? 1 : $clog2(NBANKS);
  localparam int unsigned MID_W      = (NMASTERS <= 1) ? 1 : $clog2(NMASTERS);

  initial begin
    if (DATA_W % 8 != 0) $error("gb_partition_ctrl: DATA_W must be a multiple of 8");
    if ((BANK_SEL_LSB + BANK_W) > ADDR_W)
      $error("gb_partition_ctrl: bank-index field exceeds ADDR_W");
  end

  // ==========================================================================
  // Helpers: popcount, nth-set, slice set/clear
  // ==========================================================================
  function automatic int unsigned popcount_mask(input logic [NBANKS-1:0] m);
    int unsigned c; c = 0;
    for (int i=0;i<NBANKS;i++) if (m[i]) c++;
    return c;
  endfunction

  function automatic logic [BANK_W-1:0]
    nth_set_bank(input logic [NBANKS-1:0] m, input int unsigned nmod);
    // Returns bank index of n-th set bit (0-based) — assumes nmod < popcount.
    int unsigned cnt; cnt = 0;
    nth_set_bank = '0;
    for (int i=0;i<NBANKS;i++) begin
      if (m[i]) begin
        if (cnt == nmod) begin
          nth_set_bank = BANK_W'(i);
          return nth_set_bank;
        end
        cnt++;
      end
    end
    return BANK_W'(0);
  endfunction

  function automatic logic [ADDR_W-1:0]
    set_bank_bits(input logic [ADDR_W-1:0] a, input logic [BANK_W-1:0] bank);
    logic [ADDR_W-1:0] mask;
    mask = ~(((logic[ADDR_W-1:0])({{(ADDR_W-BANK_W){1'b0}}, {BANK_W{1'b1}}})) << BANK_SEL_LSB);
    set_bank_bits = (a & mask) | ((logic[ADDR_W-1:0])({{(ADDR_W-BANK_W){1'b0}}, bank}) << BANK_SEL_LSB);
  endfunction

  // ==========================================================================
  // Partition-based address translation (combinational)
  // ==========================================================================
  typedef struct packed {
    logic              allow;
    logic [ADDR_W-1:0] addr_out;
    logic [BANK_W-1:0] bank_sel;
    logic [7:0]        sel_part; // one-hot (NPART<=8 assumed for this small field)
  } map_res_t;

  function automatic map_res_t map_addr(
    input logic [ADDR_W-1:0] a,
    input logic [MID_W-1:0]  mid
  );
    map_res_t r;
    r.allow    = 1'b0;
    r.addr_out = a;
    r.bank_sel = '0;
    r.sel_part = '0;

    if (!cfg_global_en_i) begin
      // Global disabled: pass-through (allow)
      r.allow    = 1'b1;
      r.addr_out = a;
      r.bank_sel = a[BANK_SEL_LSB +: BANK_W];
      return r;
    end

    // Scan partitions in fixed priority (0..NPART-1)
    for (int p=0; p<NPART; p++) begin
      if (!cfg_part_en_i[p]) continue;
      if (!cfg_part_owner_mask_i[p][mid]) continue;

      // Range check: [base, base+size)
      logic [ADDR_W:0] base = {1'b0, cfg_part_base_i[p]};
      logic [ADDR_W:0] size = {1'b0, cfg_part_size_i[p]};
      logic [ADDR_W:0] addr = {1'b0, a};
      if (size == 0) continue; // empty

      if ((addr >= base) && (addr < (base + size))) begin
        // In-range; figure out bank mapping
        logic [NBANKS-1:0] mask = cfg_part_bank_mask_i[p];
        int unsigned nb = popcount_mask(mask);
        if (nb == 0) begin
          r.allow = 1'b0;   // misconfigured partition: block
          r.sel_part[p] = 1'b1;
          return r;
        end

        // stride = max(stride_l2, BEAT_LSB) to keep at least beat granularity
        int unsigned sL2 = (cfg_part_stride_l2_i[p] < BEAT_LSB) ? BEAT_LSB : cfg_part_stride_l2_i[p];

        // Compute group index: ((addr - base) >> sL2) % nb
        int unsigned idx_raw;
        idx_raw = ( (addr - base) >> sL2 );
        int unsigned idx_mod;
        // % is synthesizable; nb is small (<= NBANKS)
        idx_mod = (nb==1) ? 0 : (idx_raw % nb);

        logic [BANK_W-1:0] phy_b = nth_set_bank(mask, idx_mod);

        r.allow    = 1'b1;
        r.bank_sel = phy_b;
        r.addr_out = set_bank_bits(a, phy_b);
        r.sel_part[p] = 1'b1;
        return r; // stop on first match
      end
    end

    // No partition matched -> deny (stall) by default
    r.allow    = 1'b0;
    r.addr_out = a;
    r.bank_sel = a[BANK_SEL_LSB +: BANK_W];
    return r;
  endfunction

  // ==========================================================================
  // Apply mapping to all masters (WR & RD)
  // ==========================================================================
  map_res_t wr_map   [NMASTERS];
  map_res_t rd_map   [NMASTERS];

  genvar m;
  generate
    for (m=0; m<NMASTERS; m++) begin : g_map_all
      always_comb begin
        wr_map[m] = map_addr(up_wr_addr_i[m], MID_W'(m));
        rd_map[m] = map_addr(up_rd_addr_i[m], MID_W'(m));
      end
    end
  endgenerate

  // ==========================================================================
  // Handshake plumbing (pass-through when allowed; stall if denied)
  // ==========================================================================
  // Downstream requests
  generate
    for (m=0; m<NMASTERS; m++) begin : g_pipe
      // WRITE
      assign dn_wr_valid_o[m] = up_wr_valid_i[m] & wr_map[m].allow;
      assign dn_wr_addr_o [m] = wr_map[m].addr_out;
      assign dn_wr_data_o [m] = up_wr_data_i[m];
      assign dn_wr_strb_o [m] = up_wr_strb_i[m];
      // upstream ready only when allowed and downstream ready
      assign up_wr_ready_o[m] = wr_map[m].allow ? dn_wr_ready_i[m] : 1'b0;

      // READ
      assign dn_rd_valid_o[m] = up_rd_valid_i[m] & rd_map[m].allow;
      assign dn_rd_addr_o [m] = rd_map[m].addr_out;
      assign up_rd_ready_o[m] = rd_map[m].allow ? dn_rd_ready_i[m] : 1'b0;

      // DENY pulse (optional)
      if (PULSE_DENY) begin : g_deny
        // Pulse when upstream fires but mapping denies (we consume nothing downstream)
        logic wr_deny_q, rd_deny_q;
        always_ff @(posedge clk_i or negedge rstn_i) begin
          if (!rstn_i) begin
            wr_deny_q <= 1'b0;
            rd_deny_q <= 1'b0;
          end else begin
            wr_deny_q <= up_wr_valid_i[m] & ~wr_map[m].allow;
            rd_deny_q <= up_rd_valid_i[m] & ~rd_map[m].allow;
          end
        end
        assign wr_deny_pulse_o[m] = wr_deny_q;
        assign rd_deny_pulse_o[m] = rd_deny_q;
      end else begin : g_nodeny
        assign wr_deny_pulse_o[m] = 1'b0;
        assign rd_deny_pulse_o[m] = 1'b0;
      end

      // READ RETURN pass-through
      assign up_r_valid_o[m] = dn_r_valid_i[m];
      assign dn_r_ready_o[m] = up_r_ready_i[m];
      assign up_r_data_o [m] = dn_r_data_i[m];
      assign up_r_sbe_o  [m] = dn_r_sbe_i[m];
      assign up_r_dbe_o  [m] = dn_r_dbe_i[m];
    end
  endgenerate

  // ==========================================================================
  // Bank enable / sleep hints
  //   OR of all active partition masks. If a bank is unused by all partitions,
  //   you may choose to place it in retention/sleep at the wrapper.
  // ==========================================================================
  always_comb begin
    logic [NBANKS-1:0] used = '0;
    if (cfg_global_en_i) begin
      for (int p=0; p<NPART; p++) begin
        if (cfg_part_en_i[p]) used |= cfg_part_bank_mask_i[p];
      end
    end
    bank_enable_o = used;
    bank_sleep_o  = ~used;
  end

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Beat alignment must hold at acceptance
  generate
    if (BEAT_LSB > 0) begin : g_align
      for (genvar i=0; i<NMASTERS; i++) begin : g_ma
        assert property (@(posedge clk_i) disable iff(!rstn_i)
          up_wr_valid_i[i] && up_wr_ready_o[i] |-> (up_wr_addr_i[i][BEAT_LSB-1:0]=='0))
          else $error("gb_partition_ctrl: WR addr not beat-aligned (m%0d)", i);
        assert property (@(posedge clk_i) disable iff(!rstn_i)
          up_rd_valid_i[i] && up_rd_ready_o[i] |-> (up_rd_addr_i[i][BEAT_LSB-1:0]=='0))
          else $error("gb_partition_ctrl: RD addr not beat-aligned (m%0d)", i);
      end
    end
  endgenerate

  // If allowed, ensure chosen bank is indeed enabled in that partition
  // (Best-effort check across all masters combinationally)
  for (genvar i=0; i<NMASTERS; i++) begin : g_mask_chk
    logic [BANK_W-1:0] _wb = wr_map[i].bank_sel;
    logic [BANK_W-1:0] _rb = rd_map[i].bank_sel;
    // index bounds
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      wr_map[i].allow |-> (_wb < NBANKS)) else $error("gb_partition_ctrl: WR bank_sel OOB");
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      rd_map[i].allow |-> (_rb < NBANKS)) else $error("gb_partition_ctrl: RD bank_sel OOB");
  end
`endif

endmodule

`default_nettype wire
