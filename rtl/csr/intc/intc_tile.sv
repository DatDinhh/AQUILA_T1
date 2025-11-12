// ============================================================================
//  Aquila-T1 Tile Interrupt Controller (PLIC-like) + Timer + Watchdog
//  File: intc_tile.sv
//  Description:
//    - Parametric interrupt controller with APB3 CSR interface
//    - Edge/level + polarity per source; priority per source; threshold per line
//    - Claim/complete per line (PLIC-like)
//    - Optional 64-bit timer (mtime/mtimecmp, periodic) and watchdog sources
//
//  Parameters:
//    NSRC_EXT   : number of external (raw) interrupt sources (e.g., DMA/ECC/...)
//    NLINES     : number of interrupt lines to RISC-V uC (PLIC inputs)
//    HAS_TIMER  : include internal timer source
//    HAS_WDOG   : include internal watchdog source
//    APB_ADDR_W : APB address width (bytes); APB_DATA_W must be 32
//
//  Ports:
//    clk_i, rstn_i     : control clock domain, sync active-low reset
//    src_ext_i         : NSRC_EXT raw sources (already bit-synchronized to clk_i)
//    APB3 slave ports  : paddr_i, psel_i, penable_i, pwrite_i, pwdata_i, prdata_o
//    irq_lines_o       : NLINES level-high interrupt outputs to RISC-V PLIC/core
//
//  Notes:
//    - Pending for level sources is latched on level=1 and cleared by complete;
//      if level remains asserted, pending re-sets immediately (PLIC semantics).
//    - Claim returns 1-based source ID; 0 means "no pending".
//    - Priorities are 3-bit (0..7). A source is eligible on a line if
//      enabled && mapped-to-line && priority > threshold[line] && pending=1.
//
//  © 2025. MIT-style license (adjust per repo policy).
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module intc_tile #(
  parameter int unsigned NSRC_EXT   = 12,
  parameter int unsigned NLINES     = 8,
  parameter bit          HAS_TIMER  = 1'b1,
  parameter bit          HAS_WDOG   = 1'b1,
  parameter int unsigned APB_ADDR_W = 20,
  parameter int unsigned APB_DATA_W = 32
)(
  // Clock & Reset (control domain)
  input  logic                         clk_i,
  input  logic                         rstn_i,       // synchronous active-low

  // External raw sources (already synchronized to clk_i)
  input  logic [NSRC_EXT-1:0]          src_ext_i,

  // APB3 Slave
  input  logic [APB_ADDR_W-1:0]        paddr_i,
  input  logic                         psel_i,
  input  logic                         penable_i,
  input  logic                         pwrite_i,
  input  logic [APB_DATA_W-1:0]        pwdata_i,
  output logic [APB_DATA_W-1:0]        prdata_o,
  output logic                         pready_o,
  output logic                         pslverr_o,

  // Interrupt lines to RISC-V
  output logic [NLINES-1:0]            irq_lines_o
);

  // --------------------------------------------------------------------------
  // Derived constants
  // --------------------------------------------------------------------------
  localparam int unsigned NSRC_INT = (HAS_TIMER ? 1 : 0) + (HAS_WDOG ? 1 : 0);
  localparam int unsigned NSRC     = NSRC_EXT + NSRC_INT;
  localparam int unsigned SRC_W    = (NSRC <= 1) ? 1 : $clog2(NSRC);
  localparam int unsigned LINE_W   = (NLINES <= 1) ? 1 : $clog2(NLINES);
  localparam int unsigned NWORDS   = (NSRC + 31) / 32;  // 32-bit bitfield words

  // Indices for internal sources
  localparam int signed SRC_IDX_TIMER = HAS_TIMER ? (NSRC_EXT + 0) : -1;
  localparam int signed SRC_IDX_WDOG  = HAS_WDOG  ? (NSRC_EXT + (HAS_TIMER?1:0)) : -1;

  // --------------------------------------------------------------------------
  // APB decode helpers
  // --------------------------------------------------------------------------
  wire [APB_ADDR_W-1:0] addr_aligned = {paddr_i[APB_ADDR_W-1:2], 2'b00};
  wire                   wr_en        = psel_i && penable_i && pwrite_i;
  wire                   rd_en        = psel_i && !pwrite_i;
  assign                 pready_o     = 1'b1;

  // --------------------------------------------------------------------------
  // Register map (byte addresses)
  //   ID/PARAM                  0x0000..0x000C
  //   SRC_TYPE[n] (edge=1)      0x0100 + 0x4*n   (n in 0..NWORDS-1)
  //   SRC_POL[n]  (active-high) 0x0140 + 0x4*n
  //   SRC_EN[n]                 0x0180 + 0x4*n
  //   SRC_PEND[n] (W1C)         0x01C0 + 0x4*n
  //   SRC_PRIO[i]               0x0200 + 0x4*i   (i in 0..NSRC-1)
  //   SRC_MAP[i]                0x0400 + 0x4*i   (line index)
  //   LINE_THRESH[l]            0x0600 + 0x4*l   (l in 0..NLINES-1)
  //   CLAIM[l]  (R=claim, W=complete) 0x0700 + 0x4*l
  //   TIMER block               0x0800..0x081C
  //   WDOG  block               0x0820..0x0830
  // --------------------------------------------------------------------------
  localparam logic [APB_ADDR_W-1:0] ADDR_ID           = 32'h0000;
  localparam logic [APB_ADDR_W-1:0] ADDR_PARAM        = 32'h0004;
  localparam logic [APB_ADDR_W-1:0] ADDR_REV          = 32'h0008;

  localparam logic [APB_ADDR_W-1:0] BASE_TYPE         = 32'h0100;
  localparam logic [APB_ADDR_W-1:0] BASE_POL          = 32'h0140;
  localparam logic [APB_ADDR_W-1:0] BASE_EN           = 32'h0180;
  localparam logic [APB_ADDR_W-1:0] BASE_PEND         = 32'h01C0;

  localparam logic [APB_ADDR_W-1:0] BASE_PRIO         = 32'h0200;
  localparam logic [APB_ADDR_W-1:0] BASE_MAP          = 32'h0400;
  localparam logic [APB_ADDR_W-1:0] BASE_THRESH       = 32'h0600;
  localparam logic [APB_ADDR_W-1:0] BASE_CLAIM        = 32'h0700;

  localparam logic [APB_ADDR_W-1:0] BASE_TIMER        = 32'h0800;
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_CTRL     = BASE_TIMER + 32'h00; // [0]en [1]periodic
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_PRESC    = BASE_TIMER + 32'h04; // prescaler
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_MTIME_LO = BASE_TIMER + 32'h08;
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_MTIME_HI = BASE_TIMER + 32'h0C;
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_CMP_LO   = BASE_TIMER + 32'h10;
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_CMP_HI   = BASE_TIMER + 32'h14;
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_PER_LO   = BASE_TIMER + 32'h18;
  localparam logic [APB_ADDR_W-1:0] ADDR_TMR_PER_HI   = BASE_TIMER + 32'h1C;

  localparam logic [APB_ADDR_W-1:0] BASE_WDOG         = 32'h0820;
  localparam logic [APB_ADDR_W-1:0] ADDR_WDG_CTRL     = BASE_WDOG + 32'h00; // [0]en [1]autorl
  localparam logic [APB_ADDR_W-1:0] ADDR_WDG_RELOAD   = BASE_WDOG + 32'h04; // reload value
  localparam logic [APB_ADDR_W-1:0] ADDR_WDG_COUNT    = BASE_WDOG + 32'h08; // current count (RW)
  localparam logic [APB_ADDR_W-1:0] ADDR_WDG_KICK     = BASE_WDOG + 32'h0C; // W1P kick
  localparam logic [APB_ADDR_W-1:0] ADDR_WDG_STATUS   = BASE_WDOG + 32'h10; // [0]fired (W1C)

  // --------------------------------------------------------------------------
  // Source control registers
  // --------------------------------------------------------------------------
  logic [NSRC-1:0] src_type_edge_q;   // 1=edge, 0=level
  logic [NSRC-1:0] src_pol_high_q;    // 1=active-high, 0=active-low
  logic [NSRC-1:0] src_en_q;          // enable mask
  logic [NSRC-1:0] src_pending_q;     // pending (W1C or claim/complete clear)
  logic [2:0]      src_prio_q   [NSRC];
  logic [LINE_W-1:0] src_map_q  [NSRC];

  // Threshold per line (3-bit)
  logic [2:0] line_thresh_q [NLINES];

  // For address-indexed tables
  integer i, w, l;

  // --------------------------------------------------------------------------
  // Internal sources: TIMER and WDOG (raw pulses)
  // --------------------------------------------------------------------------
  // Timer registers/state
  logic        tmr_en_q, tmr_periodic_q;
  logic [15:0] tmr_presc_q;
  logic [15:0] tmr_psc_cnt_q;
  logic [63:0] mtime_q, mtimecmp_q, tmr_period_q;
  logic        tmr_tick;
  logic        tmr_fire_pulse;

  // Watchdog registers/state
  logic        wdg_en_q, wdg_autorl_q;
  logic [31:0] wdg_reload_q, wdg_count_q;
  logic        wdg_kick_pulse;
  logic        wdg_fire_pulse;
  logic        wdg_fired_sticky_q;

  // --------------------------------------------------------------------------
  // Raw -> Active (polarity), edge detect for edge sources
  // --------------------------------------------------------------------------
  logic [NSRC-1:0] src_raw_vec;       // combined external + internal pulses
  logic [NSRC-1:0] src_active_now;    // after polarity
  logic [NSRC-1:0] src_active_d;      // delayed for edge detect
  logic [NSRC-1:0] src_event_set;     // sets pending

  // --------------------------------------------------------------------------
  // Reset defaults
  // --------------------------------------------------------------------------
  initial begin
    if (APB_DATA_W != 32) $error("intc_tile: APB_DATA_W must be 32.");
    if (NSRC == 0)        $error("intc_tile: NSRC must be > 0.");
    if (NLINES == 0)      $error("intc_tile: NLINES must be > 0.");
  end

  // --------------------------------------------------------------------------
  // APB write side-effects (register writes)
  // --------------------------------------------------------------------------
  // Writes to table words (TYPE/POL/EN/PEND) span NWORDS; indexed by offset.
  // Writes to PRIO[i], MAP[i], THRESH[l], CLAIM[l], Timer/Watchdog.
  // --------------------------------------------------------------------------
  // Default resets
  generate
    genvar gi;
    for (gi = 0; gi < NSRC; gi++) begin : g_def_prio_map
      // synthesis translate_off
      initial begin
        src_prio_q[gi] = 3'd1; // default prio > 0 so it can trigger with thresh=0
        src_map_q [gi] = '0;   // default to line 0
      end
      // synthesis translate_on
    end
  endgenerate

  // Proper resets & write logic
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      src_type_edge_q   <= '0;       // default level
      src_pol_high_q    <= {NSRC{1'b1}}; // default active-high
      src_en_q          <= '0;
      src_pending_q     <= '0;

      for (i = 0; i < NSRC; i++) begin
        src_prio_q[i]   <= 3'd1;
        src_map_q[i]    <= '0;
      end

      for (l = 0; l < NLINES; l++) begin
        line_thresh_q[l] <= 3'd0;
      end

      // Timer defaults
      tmr_en_q          <= 1'b0;
      tmr_periodic_q    <= 1'b0;
      tmr_presc_q       <= 16'd0;
      tmr_psc_cnt_q     <= 16'd0;
      mtime_q           <= 64'd0;
      mtimecmp_q        <= 64'hFFFF_FFFF_FFFF_FFFF; // far future
      tmr_period_q      <= 64'd0;

      // Watchdog defaults
      wdg_en_q          <= 1'b0;
      wdg_autorl_q      <= 1'b0;
      wdg_reload_q      <= 32'd0;
      wdg_count_q       <= 32'd0;
      wdg_fired_sticky_q<= 1'b0;
    end
    else begin
      // Clear one-shot pulses by default
      wdg_kick_pulse    <= 1'b0;

      // ----- APB writes -----
      if (wr_en) begin
        // TYPE / POL / EN / PEND bitfields (NWORDS)
        for (w = 0; w < NWORDS; w++) begin
          if (addr_aligned == (BASE_TYPE + (w<<2)))
            src_type_edge_q[w*32 +: 32] <= pwdata_i;
          if (addr_aligned == (BASE_POL + (w<<2)))
            src_pol_high_q [w*32 +: 32] <= pwdata_i;
          if (addr_aligned == (BASE_EN + (w<<2)))
            src_en_q       [w*32 +: 32] <= pwdata_i;
          if (addr_aligned == (BASE_PEND + (w<<2)))
            src_pending_q  [w*32 +: 32] <= src_pending_q[w*32 +: 32] & ~pwdata_i; // W1C
        end

        // PRIO[i]
        for (i = 0; i < NSRC; i++) begin
          if (addr_aligned == (BASE_PRIO + (i<<2)))
            src_prio_q[i] <= pwdata_i[2:0];
        end

        // MAP[i]
        for (i = 0; i < NSRC; i++) begin
          if (addr_aligned == (BASE_MAP + (i<<2)))
            src_map_q[i] <= pwdata_i[LINE_W-1:0];
        end

        // THRESH[l]
        for (l = 0; l < NLINES; l++) begin
          if (addr_aligned == (BASE_THRESH + (l<<2)))
            line_thresh_q[l] <= pwdata_i[2:0];
        end

        // CLAIM complete: write 1-based ID to complete
        for (l = 0; l < NLINES; l++) begin
          if (addr_aligned == (BASE_CLAIM + (l<<2))) begin
            if (pwdata_i[SRC_W-1:0] != '0) begin
              int unsigned id = pwdata_i[SRC_W-1:0] - 1; // 0..NSRC-1
              if (id < NSRC) begin
                // W1C via complete: clear pending for that source
                src_pending_q[id] <= 1'b0;
              end
            end
          end
        end

        // ----- Timer block -----
        if (HAS_TIMER) begin
          if (addr_aligned == ADDR_TMR_CTRL) begin
            tmr_en_q       <= pwdata_i[0];
            tmr_periodic_q <= pwdata_i[1];
          end
          if (addr_aligned == ADDR_TMR_PRESC) begin
            tmr_presc_q    <= pwdata_i[15:0];
          end
          if (addr_aligned == ADDR_TMR_MTIME_LO)
            mtime_q[31:0]  <= pwdata_i;
          if (addr_aligned == ADDR_TMR_MTIME_HI)
            mtime_q[63:32] <= pwdata_i;
          if (addr_aligned == ADDR_TMR_CMP_LO)
            mtimecmp_q[31:0]  <= pwdata_i;
          if (addr_aligned == ADDR_TMR_CMP_HI)
            mtimecmp_q[63:32] <= pwdata_i;
          if (addr_aligned == ADDR_TMR_PER_LO)
            tmr_period_q[31:0]  <= pwdata_i;
          if (addr_aligned == ADDR_TMR_PER_HI)
            tmr_period_q[63:32] <= pwdata_i;
        end

        // ----- Watchdog block -----
        if (HAS_WDOG) begin
          if (addr_aligned == ADDR_WDG_CTRL) begin
            wdg_en_q     <= pwdata_i[0];
            wdg_autorl_q <= pwdata_i[1];
          end
          if (addr_aligned == ADDR_WDG_RELOAD)
            wdg_reload_q <= pwdata_i;
          if (addr_aligned == ADDR_WDG_COUNT)
            wdg_count_q  <= pwdata_i;
          if (addr_aligned == ADDR_WDG_KICK)
            wdg_kick_pulse <= 1'b1; // one-shot
          if (addr_aligned == ADDR_WDG_STATUS) begin
            // W1C fired sticky
            if (pwdata_i[0]) wdg_fired_sticky_q <= 1'b0;
          end
        end
      end

      // ----- Timer/Watchdog free-running state -----
      // Timer prescaled tick
      if (HAS_TIMER) begin
        {tmr_psc_cnt_q, tmr_tick} <= (tmr_psc_cnt_q == tmr_presc_q) ?
                                     {16'd0, 1'b1} : {tmr_psc_cnt_q + 16'd1, 1'b0};

        if (tmr_en_q && tmr_tick)   mtime_q <= mtime_q + 64'd1;

        // Fire when reached
        tmr_fire_pulse <= 1'b0;
        if (tmr_en_q && (mtime_q >= mtimecmp_q)) begin
          tmr_fire_pulse <= 1'b1;
          if (tmr_periodic_q) begin
            mtimecmp_q <= mtimecmp_q + tmr_period_q;
          end else begin
            // One-shot: move cmp to far future
            mtimecmp_q <= 64'hFFFF_FFFF_FFFF_FFFF;
          end
        end
      end else begin
        tmr_fire_pulse <= 1'b0;
      end

      // Watchdog countdown
      if (HAS_WDOG) begin
        if (wdg_kick_pulse) begin
          wdg_count_q <= wdg_reload_q;
        end else if (wdg_en_q && (wdg_count_q != 32'd0)) begin
          wdg_count_q <= wdg_count_q - 32'd1;
        end

        wdg_fire_pulse <= 1'b0;
        if (wdg_en_q && (wdg_count_q == 32'd1)) begin
          // will hit zero next cycle
          wdg_fire_pulse    <= 1'b1;
          wdg_fired_sticky_q<= 1'b1;
          if (wdg_autorl_q) wdg_count_q <= wdg_reload_q;
        end
      end else begin
        wdg_fire_pulse <= 1'b0;
      end
    end
  end // always_ff

  // --------------------------------------------------------------------------
  // Build combined raw source vector: external + timer + watchdog pulses
  // Timer/Wdog modeled as active-high pulses
  // --------------------------------------------------------------------------
  integer si;
  always_comb begin
    src_raw_vec = '0;
    for (si = 0; si < NSRC_EXT; si++) begin
      src_raw_vec[si] = src_ext_i[si];
    end
    if (HAS_TIMER) src_raw_vec[SRC_IDX_TIMER] = tmr_fire_pulse;
    if (HAS_WDOG)  src_raw_vec[SRC_IDX_WDOG]  = wdg_fire_pulse;
  end

  // Polarity application and edge detect
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      src_active_d <= '0;
    end else begin
      src_active_d <= src_active_now;
    end
  end

  genvar gj;
  generate
    for (gj = 0; gj < NSRC; gj++) begin : g_active
      assign src_active_now[gj] = src_pol_high_q[gj] ? src_raw_vec[gj] : ~src_raw_vec[gj];
      wire edge_evt = src_active_now[gj] & ~src_active_d[gj];
      wire lvl_evt  = src_active_now[gj];
      assign src_event_set[gj] = src_type_edge_q[gj] ? edge_evt : lvl_evt;
    end
  endgenerate

  // Pending: set by event_set, cleared by W1C or claim complete
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      src_pending_q <= '0;
    end else begin
      src_pending_q <= (src_pending_q | src_event_set); // set on events
      // (clears handled in APB writes & completes above)
    end
  end

  // --------------------------------------------------------------------------
  // Priority arbitration per line and IRQ line outputs
  // --------------------------------------------------------------------------
  logic [SRC_W-1:0] claim_id   [NLINES]; // 1-based; 0 if none
  logic             line_irq   [NLINES];

  // Build per-line best source
  integer li, sj;
  always_comb begin
    for (li = 0; li < NLINES; li++) begin
      // Defaults
      line_irq[li]  = 1'b0;
      claim_id[li]  = '0;

      // Track best
      logic [2:0] best_prio = 3'd0;
      logic [SRC_W-1:0] best_id = '0;
      bit found = 1'b0;

      for (sj = 0; sj < NSRC; sj++) begin
        // Eligible if: enabled, mapped to line, pending, prio > threshold
        bit mapped = (src_map_q[sj] == li[LINE_W-1:0]);
        if (src_en_q[sj] && mapped && src_pending_q[sj] && (src_prio_q[sj] > line_thresh_q[li])) begin
          if (!found || (src_prio_q[sj] > best_prio) || ((src_prio_q[sj] == best_prio) && (sj < (best_id)))) begin
            found     = 1'b1;
            best_prio = src_prio_q[sj];
            best_id   = sj[SRC_W-1:0];
          end
        end
      end

      line_irq[li] = found;
      claim_id[li] = found ? (best_id + {{(SRC_W-1){1'b0}},1'b1}) : '0; // 1-based
    end
  end

  // Drive IRQ lines
  generate
    genvar gl;
    for (gl = 0; gl < NLINES; gl++) begin : g_irq
      assign irq_lines_o[gl] = line_irq[gl];
    end
  endgenerate

  // --------------------------------------------------------------------------
  // APB read mux
  // --------------------------------------------------------------------------
  logic [31:0] rdata;

  always_comb begin
    rdata = 32'h0;

    // ID / PARAM / REV
    if (addr_aligned == ADDR_ID)    rdata = 32'h54494E54; // 'TINT' (Tile INTerrupt)
    if (addr_aligned == ADDR_PARAM) rdata = { 8'(NLINES[7:0]), 8'(NSRC[7:0]), 8'h00, 8'h10 }; // lines,sources,rev=0x10
    if (addr_aligned == ADDR_REV)   rdata = 32'h0001_0000; // major/minor

    // TYPE/POL/EN/PEND bitfields
    for (w = 0; w < NWORDS; w++) begin
      if (addr_aligned == (BASE_TYPE + (w<<2))) rdata = src_type_edge_q[w*32 +: 32];
      if (addr_aligned == (BASE_POL  + (w<<2))) rdata = src_pol_high_q [w*32 +: 32];
      if (addr_aligned == (BASE_EN   + (w<<2))) rdata = src_en_q       [w*32 +: 32];
      if (addr_aligned == (BASE_PEND + (w<<2))) rdata = src_pending_q  [w*32 +: 32];
    end

    // PRIO[i]
    for (i = 0; i < NSRC; i++) begin
      if (addr_aligned == (BASE_PRIO + (i<<2)))
        rdata = {29'h0, src_prio_q[i]};
    end

    // MAP[i]
    for (i = 0; i < NSRC; i++) begin
      if (addr_aligned == (BASE_MAP + (i<<2)))
        rdata = {{(32-LINE_W){1'b0}}, src_map_q[i]};
    end

    // THRESH[l]
    for (l = 0; l < NLINES; l++) begin
      if (addr_aligned == (BASE_THRESH + (l<<2)))
        rdata = {29'h0, line_thresh_q[l]};
    end

    // CLAIM[l] — read claim ID (1-based)
    for (l = 0; l < NLINES; l++) begin
      if (addr_aligned == (BASE_CLAIM + (l<<2)))
        rdata = {{(32-SRC_W){1'b0}}, claim_id[l]};
    end

    // Timer block
    if (HAS_TIMER) begin
      if (addr_aligned == ADDR_TMR_CTRL)     rdata = {30'h0, tmr_periodic_q, tmr_en_q};
      if (addr_aligned == ADDR_TMR_PRESC)    rdata = {16'h0, tmr_presc_q};
      if (addr_aligned == ADDR_TMR_MTIME_LO) rdata = mtime_q[31:0];
      if (addr_aligned == ADDR_TMR_MTIME_HI) rdata = mtime_q[63:32];
      if (addr_aligned == ADDR_TMR_CMP_LO)   rdata = mtimecmp_q[31:0];
      if (addr_aligned == ADDR_TMR_CMP_HI)   rdata = mtimecmp_q[63:32];
      if (addr_aligned == ADDR_TMR_PER_LO)   rdata = tmr_period_q[31:0];
      if (addr_aligned == ADDR_TMR_PER_HI)   rdata = tmr_period_q[63:32];
    end

    // Watchdog block
    if (HAS_WDOG) begin
      if (addr_aligned == ADDR_WDG_CTRL)   rdata = {30'h0, wdg_autorl_q, wdg_en_q};
      if (addr_aligned == ADDR_WDG_RELOAD) rdata = wdg_reload_q;
      if (addr_aligned == ADDR_WDG_COUNT)  rdata = wdg_count_q;
      if (addr_aligned == ADDR_WDG_KICK)   rdata = 32'h0;
      if (addr_aligned == ADDR_WDG_STATUS) rdata = {31'h0, wdg_fired_sticky_q};
    end
  end

  assign prdata_o = rdata;

  // --------------------------------------------------------------------------
  // APB error for illegal writes (simple policy)
  // --------------------------------------------------------------------------
  function automatic bit addr_is_mapped_write(input logic [APB_ADDR_W-1:0] A);
    bit hit;
    hit = 1'b0;

    for (w = 0; w < NWORDS; w++) begin
      if (A == (BASE_TYPE + (w<<2))) hit = 1'b1;
      if (A == (BASE_POL  + (w<<2))) hit = 1'b1;
      if (A == (BASE_EN   + (w<<2))) hit = 1'b1;
      if (A == (BASE_PEND + (w<<2))) hit = 1'b1;
    end
    for (i = 0; i < NSRC; i++) begin
      if (A == (BASE_PRIO + (i<<2))) hit = 1'b1;
      if (A == (BASE_MAP  + (i<<2))) hit = 1'b1;
    end
    for (l = 0; l < NLINES; l++) begin
      if (A == (BASE_THRESH + (l<<2))) hit = 1'b1;
      if (A == (BASE_CLAIM  + (l<<2))) hit = 1'b1;
    end
    if (HAS_TIMER) begin
      if (A == ADDR_TMR_CTRL)  hit = 1'b1;
      if (A == ADDR_TMR_PRESC) hit = 1'b1;
      if (A == ADDR_TMR_MTIME_LO || A == ADDR_TMR_MTIME_HI) hit = 1'b1;
      if (A == ADDR_TMR_CMP_LO   || A == ADDR_TMR_CMP_HI)   hit = 1'b1;
      if (A == ADDR_TMR_PER_LO   || A == ADDR_TMR_PER_HI)   hit = 1'b1;
    end
    if (HAS_WDOG) begin
      if (A == ADDR_WDG_CTRL  || A == ADDR_WDG_RELOAD ||
          A == ADDR_WDG_COUNT || A == ADDR_WDG_KICK ||
          A == ADDR_WDG_STATUS) hit = 1'b1;
    end
    return hit;
  endfunction

  assign pslverr_o = (psel_i && penable_i && pwrite_i && !addr_is_mapped_write(addr_aligned));

  // --------------------------------------------------------------------------
  // Assertions (simulation)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // Claim ID always in range
  genvar al;
  generate for (al = 0; al < NLINES; al++) begin : g_a_claim
    assert property (@(posedge clk_i) disable iff (!rstn_i)
      claim_id[al] <= NSRC) else
      $error("intc_tile: claim_id out of range.");
  end endgenerate

  // Edge events must be one-cycle for pulse sources (timer/wdog)
  if (HAS_TIMER) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      tmr_fire_pulse |-> !($past(tmr_fire_pulse))) else
      $warning("intc_tile: tmr_fire_pulse multi-cycle.");
  end
  if (HAS_WDOG) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      wdg_fire_pulse |-> !($past(wdg_fire_pulse))) else
      $warning("intc_tile: wdg_fire_pulse multi-cycle.");
  end
`endif

endmodule
`default_nettype wire
