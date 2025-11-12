// ============================================================================
//  ocla_trace.sv
//  Aquila — On-Chip Logic Analyzer (OCLA) Trace Buffer
//
//  Features
//  --------
//  - Single clock domain logic analyzer with circular buffer storage
//  - Pre-/post-trigger capture with mask/value & edge (rise/fall) triggers
//  - Optional external trigger input and force-trigger via CSR
//  - Decimation (sample every 2^K cycles) and optional capture qualifier
//  - Store-all or store-on-change modes
//  - CSR-Lite configuration + readout (32-bit words); snapshot indices
//  - Robust parameter checks & assertions; synthesizable SRAM-style array
//
//  Notes
//  -----
//  - All signals synchronous to clk_i.
//  - DEPTH must be a power-of-two (for ring buffer arithmetic).
//  - Readout exposes samples as 32-bit slices via RD_SAMPLE_IDX + RD_WORD_SEL.
//  - Trigger is evaluated at *sampling instants* (after decimation/qualify).
//
//  CSR Map (byte addresses; 32-bit words)
//  --------------------------------------
//   0x000  ID_VERSION         RO  [31:24]='O', [23:16]='C', [15:8]=MAJ(1), [7:0]=MIN(0)
//   0x004  CTRL               RW  [0]=ARM(W1) [1]=DISARM(W1) [2]=FORCE_TRIG(W1)
//                                 [3]=CLR_DONE(W1) [4]=CLR_MEM_IGN (reserved)
//                                 [5]=STORE_ON_CHANGE [6]=EXT_TRIG_EN [7]=TRIG_OUT_EN
//                                 [8]=QUAL_EN [9]=QUAL_INV
//   0x008  STATUS             RO  [0]=armed [1]=triggered [2]=done [3]=primed
//                                 [15:4]=decim_log2 [31:16]=captured_samples (valid after done)
//   0x00C  CONFIG0            RW  [15:0]=pre_samples  [31:16]=post_samples
//   0x010  DECIM_LOG2         RW  sample every 2^K (K in [0..15])
//   0x014  QUAL_SRC_SEL       RW  [15:0]=level event index for qualifier; 0xFFFF → use qual_in_i
//   0x018  EXT_TRIG_POL       RW  [0]=polarity for ext_trig_i (0=rising,1=high-level)
//   0x01C  PROBE_WIDTH        RO  PROBE_W
//   0x020.. (TRIG_VAL)        RW  WORDS×32-bit covering PROBE_W bits
//   0x040.. (TRIG_MASK)       RW  WORDS×32-bit covering PROBE_W bits
//   0x060.. (RISE_MASK)       RW  WORDS×32-bit
//   0x080.. (FALL_MASK)       RW  WORDS×32-bit
//   0x0A0  TRIG_SRC_SEL       RW  [0]=mask/value enable [1]=edge enable [2]=ext enable
//   0x0A4  INDICES0           RO  [15:0]=wr_ptr  [31:16]=trig_ptr (index of trigger sample)
//   0x0A8  INDICES1           RO  [15:0]=start_ptr (chronological start) [31:16]=cap_count
//   0x100  RD_SAMPLE_IDX      RW  sample index relative to start_ptr (0..cap_count-1)
//   0x104  RD_WORD_SEL        RW  which 32-bit word within a sample (0..WORDS-1)
//   0x108  RD_DATA            RO  32-bit slice of the selected sample
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module ocla_trace #(
  parameter int unsigned PROBE_W   = 128,  // width of probe bus (bits)
  parameter int unsigned DEPTH     = 1024, // # of samples (power-of-two)
  parameter int unsigned N_LEVELS  = 64,   // # of external level events for qualifier select
  parameter int unsigned ADDR_W    = 12    // CSR byte address width (e.g., 4 KiB window)
)(
  input  logic                    clk_i,
  input  logic                    rstn_i,        // synchronous, active-low

  // ---------------------- Capture inputs ----------------------
  input  logic [PROBE_W-1:0]     probe_i,       // sampled bus
  input  logic [N_LEVELS-1:0]    level_bus_i,   // optional level events (qualifier select source)
  input  logic                    qual_in_i,     // external qualifier (used if QUAL_SRC_SEL==0xFFFF)
  input  logic                    ext_trig_i,    // external trigger (polarity per EXT_TRIG_POL)
  output logic                    trig_out_o,    // pulse on trigger (if TRIG_OUT_EN)

  // ---------------------- CSR-Lite port -----------------------
  input  logic                    csr_valid_i,
  input  logic                    csr_write_i,
  input  logic [ADDR_W-1:0]       csr_addr_i,   // byte address
  input  logic [31:0]             csr_wdata_i,
  input  logic [3:0]              csr_wstrb_i,
  output logic                    csr_ready_o,
  output logic [31:0]             csr_rdata_o
);

  // -------------------------- Derived --------------------------
  localparam int unsigned WORDS   = (PROBE_W + 31) / 32;
  localparam int unsigned LGDEPTH = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
  localparam logic [31:0] ID_VERSION = {8'h4F, 8'h43, 8'h01, 8'h00}; // 'O''C' v1.0

  // Static checks
  initial begin
    if (DEPTH == 0 || (DEPTH & (DEPTH-1)) != 0)
      $error("ocla_trace: DEPTH (%0d) must be power-of-two.", DEPTH);
    if (PROBE_W == 0)
      $error("ocla_trace: PROBE_W must be > 0.");
  end

  // ---------------------- Control/Config regs ------------------
  // CTRL bits
  logic ctrl_store_on_change_q, ctrl_store_on_change_d;
  logic ctrl_ext_trig_en_q,     ctrl_ext_trig_en_d;
  logic ctrl_trig_out_en_q,     ctrl_trig_out_en_d;
  logic ctrl_qual_en_q,         ctrl_qual_en_d;
  logic ctrl_qual_inv_q,        ctrl_qual_inv_d;

  // Globals
  logic armed_q, armed_d;
  logic triggered_q, triggered_d;
  logic done_q, done_d;
  logic primed_q, primed_d;

  logic [15:0] pre_cfg_q,   pre_cfg_d;
  logic [15:0] post_cfg_q,  post_cfg_d;
  logic [15:0] decim_log2_q,decim_log2_d;
  logic [15:0] qual_sel_q,  qual_sel_d;   // 0xFFFF -> use qual_in_i directly
  logic        ext_trig_pol_q, ext_trig_pol_d; // 0=rising pulse, 1=level-high

  // Trigger masks/values (wide)
  logic [PROBE_W-1:0] trig_val_q,  trig_val_d;
  logic [PROBE_W-1:0] trig_mask_q, trig_mask_d;
  logic [PROBE_W-1:0] rise_mask_q, rise_mask_d;
  logic [PROBE_W-1:0] fall_mask_q, fall_mask_d;

  // Trigger source enables
  logic trig_en_maskval_q, trig_en_maskval_d;
  logic trig_en_edge_q,     trig_en_edge_d;
  logic trig_en_ext_q,      trig_en_ext_d;

  // Indices
  logic [LGDEPTH-1:0] wr_ptr_q, wr_ptr_d;
  logic [LGDEPTH-1:0] trig_ptr_q, trig_ptr_d;
  logic [LGDEPTH-1:0] start_ptr_q, start_ptr_d;
  logic [LGDEPTH:0]   cap_count_q, cap_count_d; // up to DEPTH

  // Decimator
  logic [15:0]        decim_cnt_q, decim_cnt_d;

  // Bookkeeping
  logic [LGDEPTH:0]   samples_since_arm_q, samples_since_arm_d;
  logic [15:0]        post_left_q, post_left_d;

  // Previous sample for store-on-change & edge detect
  logic [PROBE_W-1:0] prev_sample_q, prev_sample_d;

  // ---------------------- Storage (SRAM-style) -----------------
  // Single-port write during capture; CSR reads via mux from array
  logic [PROBE_W-1:0] mem [0:DEPTH-1];

  // ---------------------- Qualifier & decimation ---------------
  logic qual_level_sel = (qual_sel_q == 16'hFFFF) ? qual_in_i
                                                  : level_bus_i[qual_sel_q[$clog2(N_LEVELS)-1:0]];
  logic qual_gate = ctrl_qual_en_q ? (qual_level_sel ^ ctrl_qual_inv_q) : 1'b1;

  // Decimator fires when counter is zero
  wire decim_fire = (decim_cnt_q == 16'd0);

  // ---------------------- Trigger evaluation -------------------
  // Evaluate only at sampling instants (decim_fire && qual_gate)
  // Compute compare & edges on current sample vs previous captured sample
  logic [PROBE_W-1:0] sample_now;
  logic trig_maskval_hit, trig_edge_hit, trig_ext_hit;

  // Edge detection requires a reference; use previous sample value
  logic [PROBE_W-1:0] rising_bits, falling_bits;

  assign sample_now = probe_i;

  always_comb begin
    // Mask/value compare
    trig_maskval_hit = (((sample_now ^ trig_val_q) & trig_mask_q) == '0);

    // Edges
    rising_bits  = ~prev_sample_q & sample_now & rise_mask_q;
    falling_bits =  prev_sample_q & ~sample_now & fall_mask_q;
    trig_edge_hit = (|rising_bits) | (|falling_bits);

    // External trigger
    // Polarity: 0=rising pulse; 1=level-high
    trig_ext_hit = 1'b0;
  end

  // External trigger conditioner
  logic ext_trig_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) ext_trig_q <= 1'b0;
    else         ext_trig_q <= ext_trig_i;
  end
  wire ext_rise = ext_trig_i & ~ext_trig_q;
  wire ext_cond = (ext_trig_pol_q == 1'b0) ? ext_rise : ext_trig_i;

  // Combined trigger (only while armed and not yet triggered)
  wire trig_candidates = (trig_en_maskval_q && trig_maskval_hit) |
                         (trig_en_edge_q     && trig_edge_hit)    |
                         (trig_en_ext_q      && ext_cond);

  // trig_out pulse generator
  logic trig_out_pulse;
  assign trig_out_o = ctrl_trig_out_en_q ? trig_out_pulse : 1'b0;

  // ---------------------- Capture core FSM (implicit) ----------
  // Operation:
  //  - When armed, on each decimated & qualified cycle, write sample to mem[wr_ptr]
  //  - While not triggered: wr_ptr advances circularly; primed when >= pre_cfg
  //  - On trigger: latch trig_ptr=wr_ptr, set post_left=post_cfg
  //  - After trigger: continue sampling until post_left==0 -> done
  //  - Compute start_ptr/cap_count at done for chronological reading
  //
  always_comb begin
    // Defaults
    armed_d           = armed_q;
    triggered_d       = triggered_q;
    done_d            = done_q;
    primed_d          = primed_q;
    wr_ptr_d          = wr_ptr_q;
    trig_ptr_d        = trig_ptr_q;
    start_ptr_d       = start_ptr_q;
    cap_count_d       = cap_count_q;
    samples_since_arm_d = samples_since_arm_q;
    post_left_d       = post_left_q;
    prev_sample_d     = prev_sample_q;

    decim_cnt_d       = decim_cnt_q;

    trig_out_pulse    = 1'b0;

    // Decimator update (free-running while armed)
    if (armed_q) begin
      if (decim_log2_q == 16'd0)       decim_cnt_d = 16'd0;
      else if (decim_cnt_q == 16'd0)   decim_cnt_d = (16'(1) << decim_log2_q) - 16'd1;
      else                             decim_cnt_d = decim_cnt_q - 16'd1;
    end else begin
      decim_cnt_d = (decim_log2_q == 16'd0) ? 16'd0 : (16'(1) << decim_log2_q) - 16'd1;
    end

    // Sample condition
    logic will_sample = armed_q && !done_q && decim_fire && qual_gate;

    // Handle sampling
    if (will_sample) begin
      // Store-on-change filter (optional)
      logic change_ok = 1'b1;
      if (ctrl_store_on_change_q) change_ok = (sample_now != prev_sample_q);

      if (change_ok) begin
        // Write to memory
        mem[wr_ptr_q] = sample_now;

        // Update counters/ptrs
        wr_ptr_d = wr_ptr_q + {{LGDEPTH-1{1'b0}},1'b1};
        samples_since_arm_d = (samples_since_arm_q == DEPTH[LGDEPTH:0]) ? samples_since_arm_q
                                  : (samples_since_arm_q + {{LGDEPTH{1'b0}},1'b1});
        if (!primed_q && (samples_since_arm_q >= pre_cfg_q)) primed_d = 1'b1;

        // Trigger evaluation at sampling instant
        if (!triggered_q && (trig_candidates)) begin
          triggered_d   = 1'b1;
          trig_ptr_d    = wr_ptr_q;  // index of the triggering sample
          post_left_d   = post_cfg_q;
          trig_out_pulse= 1'b1;
        end else if (triggered_q) begin
          if (post_left_q != 16'd0) begin
            post_left_d = post_left_q - 16'd1;
            if (post_left_q == 16'd1) begin
              // Done with post samples; freeze capture
              done_d = 1'b1;

              // Compute start_ptr and cap_count for readout
              logic [LGDEPTH:0] pre_avail = (samples_since_arm_q > pre_cfg_q)
                                            ? {{(LGDEPTH+1-16){1'b0}}, pre_cfg_q}
                                            : samples_since_arm_q;
              logic [LGDEPTH:0] total = pre_avail + {{(LGDEPTH+1-16){1'b0}}, 16'd1} // trigger sample
                                        + {{(LGDEPTH+1-16){1'b0}}, post_cfg_q};
              // Clamp to DEPTH
              if (total > DEPTH[LGDEPTH:0]) total = DEPTH[LGDEPTH:0];

              cap_count_d = total;

              // start_ptr = trig_ptr - pre_avail (mod DEPTH)
              start_ptr_d = trig_ptr_q - pre_avail[LGDEPTH-1:0];
            end
          end
        end
      end

      // Update prev_sample reference once per *attempted* sample (even if filtered),
      // so edge detection reflects most recent observed state at sampling boundary.
      prev_sample_d = sample_now;
    end
  end

  // -------------------------- CSR logic ------------------------
  // Address regions
  localparam logic [ADDR_W-1:0] G_BASE  = 'h000;
  localparam logic [ADDR_W-1:0] T_VAL   = 'h020; // TRIG_VAL range
  localparam logic [ADDR_W-1:0] T_MASK  = 'h040; // TRIG_MASK range
  localparam logic [ADDR_W-1:0] T_RISE  = 'h060; // RISE_MASK
  localparam logic [ADDR_W-1:0] T_FALL  = 'h080; // FALL_MASK
  localparam logic [ADDR_W-1:0] T_CTRL  = 'h0A0; // TRIG_SRC_SEL
  localparam logic [ADDR_W-1:0] IDX0    = 'h0A4;
  localparam logic [ADDR_W-1:0] IDX1    = 'h0A8;
  localparam logic [ADDR_W-1:0] RD_BASE = 'h100;

  // RD selectors
  logic [LGDEPTH-1:0] rd_sample_idx_q, rd_sample_idx_d; // 0..cap_count-1 relative to start_ptr
  logic [$clog2(WORDS)-1:0] rd_word_sel_q, rd_word_sel_d;

  // CSR ready: single-cycle return
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      csr_ready_o <= 1'b0;
      csr_rdata_o <= 32'd0;
    end else begin
      csr_ready_o <= csr_valid_i;
      // Read data below (combinational into rdata_next, registered here)
      csr_rdata_o <= rdata_next;
    end
  end

  // W1* pulses from CTRL
  logic arm_pulse, disarm_pulse, force_trig_pulse, clr_done_pulse;
  always_comb begin
    arm_pulse        = 1'b0;
    disarm_pulse     = 1'b0;
    force_trig_pulse = 1'b0;
    clr_done_pulse   = 1'b0;
  end

  // CSR read mux
  logic [31:0] rdata_next;
  always_comb begin
    rdata_next = 32'h0;

    if (csr_valid_i && !csr_write_i) begin
      if (csr_addr_i < T_VAL) begin
        unique case (csr_addr_i)
          12'h000: rdata_next = ID_VERSION;
          12'h004: rdata_next = {22'd0,
                                 ctrl_qual_inv_q, ctrl_qual_en_q,
                                 ctrl_trig_out_en_q, ctrl_ext_trig_en_q,
                                 ctrl_store_on_change_q,
                                 3'd0 /*W1 bits*/};
          12'h008: rdata_next = { cap_count_q[31:16], decim_log2_q[11:0], 4'd0,
                                  primed_q, done_q, triggered_q, armed_q };
          12'h00C: rdata_next = { post_cfg_q, pre_cfg_q };
          12'h010: rdata_next = { 16'd0, decim_log2_q };
          12'h014: rdata_next = { 16'd0, qual_sel_q };
          12'h018: rdata_next = { 31'd0, ext_trig_pol_q };
          12'h01C: rdata_next = PROBE_W;
          default: rdata_next = 32'hDEAD_BEAF;
        endcase
      end
      else if (csr_addr_i >= T_VAL && csr_addr_i < T_MASK) begin
        // TRIG_VAL window: 0x020 + w*4
        int w = (csr_addr_i - T_VAL) >> 2;
        if (w < WORDS) begin
          int lo = w*32;
          int hi = (lo+31>=PROBE_W) ? (PROBE_W-1) : (lo+31);
          logic [31:0] tmp; tmp = 32'd0;
          tmp[hi-lo:0] = trig_val_q[hi:lo];
          rdata_next   = tmp;
        end
      end
      else if (csr_addr_i >= T_MASK && csr_addr_i < T_RISE) begin
        int w = (csr_addr_i - T_MASK) >> 2;
        if (w < WORDS) begin
          int lo = w*32;
          int hi = (lo+31>=PROBE_W) ? (PROBE_W-1) : (lo+31);
          logic [31:0] tmp; tmp = 32'd0;
          tmp[hi-lo:0] = trig_mask_q[hi:lo];
          rdata_next   = tmp;
        end
      end
      else if (csr_addr_i >= T_RISE && csr_addr_i < T_FALL) begin
        int w = (csr_addr_i - T_RISE) >> 2;
        if (w < WORDS) begin
          int lo = w*32;
          int hi = (lo+31>=PROBE_W) ? (PROBE_W-1) : (lo+31);
          logic [31:0] tmp; tmp = 32'd0;
          tmp[hi-lo:0] = rise_mask_q[hi:lo];
          rdata_next   = tmp;
        end
      end
      else if (csr_addr_i >= T_FALL && csr_addr_i < T_CTRL) begin
        int w = (csr_addr_i - T_FALL) >> 2;
        if (w < WORDS) begin
          int lo = w*32;
          int hi = (lo+31>=PROBE_W) ? (PROBE_W-1) : (lo+31);
          logic [31:0] tmp; tmp = 32'd0;
          tmp[hi-lo:0] = fall_mask_q[hi:lo];
          rdata_next   = tmp;
        end
      end
      else if (csr_addr_i == T_CTRL) begin
        rdata_next = {29'd0, trig_en_ext_q, trig_en_edge_q, trig_en_maskval_q};
      end
      else if (csr_addr_i == IDX0) begin
        rdata_next = { trig_ptr_q, wr_ptr_q };
      end
      else if (csr_addr_i == IDX1) begin
        rdata_next = { cap_count_q[15:0], start_ptr_q };
      end
      else if (csr_addr_i >= RD_BASE) begin
        // RD_DATA path: set RD_SAMPLE_IDX and RD_WORD_SEL before reading at RD_DATA
        if (csr_addr_i == 12'h100)       rdata_next = { {(32-LGDEPTH){1'b0}}, rd_sample_idx_q };
        else if (csr_addr_i == 12'h104)  rdata_next = { {(32-$clog2(WORDS)){1'b0}}, rd_word_sel_q };
        else if (csr_addr_i == 12'h108) begin
          // Map logical (start_ptr + rd_sample_idx) modulo DEPTH
          logic [LGDEPTH-1:0] phys = start_ptr_q + rd_sample_idx_q;
          logic [PROBE_W-1:0] samp = mem[phys];
          int lo = rd_word_sel_q*32;
          int hi = (lo+31>=PROBE_W) ? (PROBE_W-1) : (lo+31);
          logic [31:0] tmp; tmp = 32'd0;
          tmp[hi-lo:0] = samp[hi:lo];
          rdata_next   = tmp;
        end else begin
          rdata_next = 32'hBADC_AB1E;
        end
      end
    end
  end

  // CSR writes / state updates
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      // Defaults
      {armed_q, triggered_q, done_q, primed_q} <= '0;
      wr_ptr_q       <= '0;
      trig_ptr_q     <= '0;
      start_ptr_q    <= '0;
      cap_count_q    <= '0;
      post_left_q    <= '0;
      samples_since_arm_q <= '0;
      prev_sample_q  <= '0;

      ctrl_store_on_change_q <= 1'b0;
      ctrl_ext_trig_en_q     <= 1'b0;
      ctrl_trig_out_en_q     <= 1'b0;
      ctrl_qual_en_q         <= 1'b0;
      ctrl_qual_inv_q        <= 1'b0;

      pre_cfg_q       <= 16'(DEPTH/4);
      post_cfg_q      <= 16'(DEPTH/4);
      decim_log2_q    <= 16'd0;
      decim_cnt_q     <= 16'd0;
      qual_sel_q      <= 16'hFFFF;
      ext_trig_pol_q  <= 1'b0;

      trig_val_q      <= '0;
      trig_mask_q     <= '0;
      rise_mask_q     <= '0;
      fall_mask_q     <= '0;

      trig_en_maskval_q <= 1'b1;
      trig_en_edge_q    <= 1'b0;
      trig_en_ext_q     <= 1'b0;

      rd_sample_idx_q <= '0;
      rd_word_sel_q   <= '0;
    end else begin
      // Core state
      armed_q           <= armed_d;
      triggered_q       <= triggered_d;
      done_q            <= done_d;
      primed_q          <= primed_d;
      wr_ptr_q          <= wr_ptr_d;
      trig_ptr_q        <= trig_ptr_d;
      start_ptr_q       <= start_ptr_d;
      cap_count_q       <= cap_count_d;
      post_left_q       <= post_left_d;
      samples_since_arm_q <= samples_since_arm_d;
      prev_sample_q     <= prev_sample_d;
      decim_cnt_q       <= decim_cnt_d;

      // CSR state
      ctrl_store_on_change_q <= ctrl_store_on_change_d;
      ctrl_ext_trig_en_q     <= ctrl_ext_trig_en_d;
      ctrl_trig_out_en_q     <= ctrl_trig_out_en_d;
      ctrl_qual_en_q         <= ctrl_qual_en_d;
      ctrl_qual_inv_q        <= ctrl_qual_inv_d;

      pre_cfg_q       <= pre_cfg_d;
      post_cfg_q      <= post_cfg_d;
      decim_log2_q    <= decim_log2_d;
      qual_sel_q      <= qual_sel_d;
      ext_trig_pol_q  <= ext_trig_pol_d;

      trig_val_q      <= trig_val_d;
      trig_mask_q     <= trig_mask_d;
      rise_mask_q     <= rise_mask_d;
      fall_mask_q     <= fall_mask_d;

      trig_en_maskval_q <= trig_en_maskval_d;
      trig_en_edge_q    <= trig_en_edge_d;
      trig_en_ext_q     <= trig_en_ext_d;

      rd_sample_idx_q <= rd_sample_idx_d;
      rd_word_sel_q   <= rd_word_sel_d;

      // --- CSR writes ---
      if (csr_valid_i && csr_write_i) begin
        // Globals and controls
        unique case (csr_addr_i)
          12'h004: begin
            // CTRL: W1 bits + mode enables
            if (csr_wstrb_i[0]) begin
              if (csr_wdata_i[0]) begin
                // ARM
                armed_q           <= 1'b1;
                triggered_q       <= 1'b0;
                done_q            <= 1'b0;
                primed_q          <= 1'b0;
                wr_ptr_q          <= '0;
                samples_since_arm_q <= '0;
                decim_cnt_q       <= (decim_log2_q==16'd0) ? 16'd0 : (16'(1)<<decim_log2_q)-16'd1;
              end
              if (csr_wdata_i[1]) begin
                // DISARM
                armed_q     <= 1'b0;
              end
              if (csr_wdata_i[2]) begin
                // FORCE_TRIG (only if armed and not yet triggered)
                if (armed_q && !triggered_q && !done_q) begin
                  triggered_q   <= 1'b1;
                  trig_ptr_q    <= wr_ptr_q;  // current write slot
                  post_left_q   <= post_cfg_q;
                end
              end
              if (csr_wdata_i[3]) begin
                // CLR_DONE
                done_q <= 1'b0;
              end
            end
            // mode bits reside in higher bytes
            if (csr_wstrb_i[1]) begin
              ctrl_store_on_change_q <= csr_wdata_i[13];
              ctrl_ext_trig_en_q     <= csr_wdata_i[14];
              ctrl_trig_out_en_q     <= csr_wdata_i[15];
              ctrl_qual_en_q         <= csr_wdata_i[16];
              ctrl_qual_inv_q        <= csr_wdata_i[17];
            end
          end

          12'h00C: begin
            // CONFIG0: pre/post
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) begin
              if (b<2) pre_cfg_q[b*8 +: 8]  <= csr_wdata_i[b*8 +: 8];
              else     post_cfg_q[(b-2)*8 +: 8] <= csr_wdata_i[b*8 +: 8];
            end
          end
          12'h010: begin
            if (&csr_wstrb_i) decim_log2_q <= csr_wdata_i[15:0];
          end
          12'h014: begin
            if (&csr_wstrb_i) qual_sel_q <= csr_wdata_i[15:0];
          end
          12'h018: begin
            if (csr_wstrb_i[0]) ext_trig_pol_q <= csr_wdata_i[0];
          end

          default: ; // handled below for ranges
        endcase

        // TRIG windows
        if (csr_addr_i >= T_VAL && csr_addr_i < T_MASK) begin
          int w = (csr_addr_i - T_VAL) >> 2;
          if (w < WORDS) begin
            int lo = w*32; int hi = (lo+31>=PROBE_W) ? (PROBE_W-1) : (lo+31);
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) trig_val_q[lo + b*8 +: 8] <= csr_wdata_i[b*8 +: 8];
          end
        end
        else if (csr_addr_i >= T_MASK && csr_addr_i < T_RISE) begin
          int w = (csr_addr_i - T_MASK) >> 2;
          if (w < WORDS) begin
            int lo = w*32; int hi = (lo+31>=PROBE_W) ? (PROBE_W-1) : (lo+31);
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) trig_mask_q[lo + b*8 +: 8] <= csr_wdata_i[b*8 +: 8];
          end
        end
        else if (csr_addr_i >= T_RISE && csr_addr_i < T_FALL) begin
          int w = (csr_addr_i - T_RISE) >> 2;
          if (w < WORDS) begin
            int lo = w*32;
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) rise_mask_q[lo + b*8 +: 8] <= csr_wdata_i[b*8 +: 8];
          end
        end
        else if (csr_addr_i >= T_FALL && csr_addr_i < T_CTRL) begin
          int w = (csr_addr_i - T_FALL) >> 2;
          if (w < WORDS) begin
            int lo = w*32;
            for (int b=0;b<4;b++) if (csr_wstrb_i[b]) fall_mask_q[lo + b*8 +: 8] <= csr_wdata_i[b*8 +: 8];
          end
        end
        else if (csr_addr_i == T_CTRL) begin
          if (csr_wstrb_i[0]) begin
            trig_en_maskval_q <= csr_wdata_i[0];
            trig_en_edge_q    <= csr_wdata_i[1];
            trig_en_ext_q     <= csr_wdata_i[2] & ctrl_ext_trig_en_q;
          end
        end
        // RD selectors
        else if (csr_addr_i == 12'h100) begin
          if (&csr_wstrb_i) rd_sample_idx_q <= csr_wdata_i[LGDEPTH-1:0];
        end
        else if (csr_addr_i == 12'h104) begin
          if (&csr_wstrb_i) rd_word_sel_q <= csr_wdata_i[$clog2(WORDS)-1:0];
        end
      end // write
    end
  end

  // Drive *_d defaults (mirror current regs — used for clean synthesis)
  always_comb begin
    ctrl_store_on_change_d = ctrl_store_on_change_q;
    ctrl_ext_trig_en_d     = ctrl_ext_trig_en_q;
    ctrl_trig_out_en_d     = ctrl_trig_out_en_q;
    ctrl_qual_en_d         = ctrl_qual_en_q;
    ctrl_qual_inv_d        = ctrl_qual_inv_q;
    pre_cfg_d              = pre_cfg_q;
    post_cfg_d             = post_cfg_q;
    decim_log2_d           = decim_log2_q;
    qual_sel_d             = qual_sel_q;
    ext_trig_pol_d         = ext_trig_pol_q;
    trig_val_d             = trig_val_q;
    trig_mask_d            = trig_mask_q;
    rise_mask_d            = rise_mask_q;
    fall_mask_d            = fall_mask_q;
    trig_en_maskval_d      = trig_en_maskval_q;
    trig_en_edge_d         = trig_en_edge_q;
    trig_en_ext_d          = trig_en_ext_q;
    rd_sample_idx_d        = rd_sample_idx_q;
    rd_word_sel_d          = rd_word_sel_q;
  end

  // -------------------------- Assertions -----------------------
`ifdef ASSERT_ON
  // Ensure pre+post+1 <= DEPTH (soft check at runtime)
  always_ff @(posedge clk_i) begin
    if (armed_q) begin
      assert ( (pre_cfg_q + post_cfg_q + 16'd1) <= DEPTH )
        else $error("ocla_trace: pre(%0d)+post(%0d)+1 exceeds DEPTH(%0d).",
                    pre_cfg_q, post_cfg_q, DEPTH);
    end
  end

  // Word selector range
  initial begin
    if (WORDS == 0) $fatal("ocla_trace: WORDS computed as 0.");
  end
`endif

endmodule

`default_nettype wire
