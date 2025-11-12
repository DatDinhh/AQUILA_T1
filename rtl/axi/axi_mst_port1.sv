// ============================================================================
//  axi_mst_port1.sv
//  Aquila — AXI4 Master Port 1 (Read-optimized, 2D/strided fetch)
//
//  Descriptor per transfer (2D rectangle):
//    - base_addr:   byte address of row 0
//    - width_bytes: bytes to fetch per row (>0)
//    - rows:        number of rows   (>0)
//    - row_stride:  byte distance from row i to row i+1 (can be >= width_bytes)
//  Produces an AXI-width stream with (data, keep, row_last, last).
//
//  Features:
//   * Legal AR bursts only (<=256 beats, no 4KB boundary crossing).
//   * Multi-outstanding (single ID, in-order return).
//   * Unaligned row starts handled with correct 'keep' on first beat,
//     tail handled on the last row beat.
//   * No combinational ready loops (R channel decoupled via FIFO).
//   * Robust assertions + telemetry + sticky error flag.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi_mst_port1 #(
  // ---------------- AXI geometry ----------------
  parameter int unsigned AXI_ADDR_W    = 48,
  parameter int unsigned AXI_DATA_W    = 256,    // bus width (bits, multiple of 8)
  parameter int unsigned AXI_ID_W      = 4,
  parameter int unsigned AXI_USER_W    = 1,      // not used by this port (tied-off)
  localparam int unsigned AXI_BYTES    = AXI_DATA_W/8,

  // ---------------- Engine config ---------------
  parameter int unsigned RD_FIFO_DEPTH = 128,    // beats buffered on R channel
  parameter int unsigned RD_OUTS_MAX   = 8,      // max outstanding AR bursts
  parameter int unsigned RD_MAX_BURST  = 32,     // beats/burst (1..256)

  // ---------------- Default AXI attrs -----------
  parameter logic [AXI_ID_W-1:0] P_ID   = '0,
  parameter logic [3:0]          P_CACHE= 4'b0011,
  parameter logic [2:0]          P_PROT = 3'b000,
  parameter logic [3:0]          P_QOS  = 4'b0000
)(
  input  logic                        clk_i,
  input  logic                        rstn_i,         // synchronous, active-low

  // ======================= 2D READ DESCRIPTOR IN ===========================
  input  logic                        desc_valid_i,
  output logic                        desc_ready_o,
  input  logic [AXI_ADDR_W-1:0]       desc_base_addr_i,
  input  logic [31:0]                 desc_width_bytes_i, // >0
  input  logic [31:0]                 desc_rows_i,        // >0
  input  logic [AXI_ADDR_W-1:0]       desc_row_stride_i,  // byte stride (>= width suggested)

  // Completion out (pulsed after final beat delivered)
  output logic                        done_valid_o,
  input  logic                        done_ready_i,
  output logic                        done_err_o,        // sticky RRESP!=OKAY for this transfer

  // ======================= Stream OUT (bus-shaped) =========================
  output logic                        out_valid_o,
  input  logic                        out_ready_i,
  output logic [AXI_DATA_W-1:0]       out_data_o,
  output logic [AXI_BYTES-1:0]        out_keep_o,        // valid byte lanes
  output logic                        out_row_last_o,    // end of current row
  output logic                        out_last_o,        // end of whole 2D transfer

  // ======================= AXI4 MASTER (Read) ==============================
  // AR
  output logic                        M_ARVALID,
  input  logic                        M_ARREADY,
  output logic [AXI_ID_W-1:0]         M_ARID,
  output logic [AXI_ADDR_W-1:0]       M_ARADDR,
  output logic [7:0]                  M_ARLEN,    // beats-1
  output logic [2:0]                  M_ARSIZE,   // log2(bytes/beat)
  output logic [1:0]                  M_ARBURST,  // INCR
  output logic [3:0]                  M_ARCACHE,
  output logic [2:0]                  M_ARPROT,
  output logic [3:0]                  M_ARQOS,
  output logic                        M_ARLOCK,

  // R
  input  logic                        M_RVALID,
  output logic                        M_RREADY,
  input  logic [AXI_DATA_W-1:0]       M_RDATA,
  input  logic [1:0]                  M_RRESP,
  input  logic [AXI_ID_W-1:0]         M_RID,
  input  logic                        M_RLAST,

  // ======================= Config / Telemetry ==============================
  input  logic                        cfg_little_endian_i, // 1=LE lane mapping, 0=BE
  input  logic                        cmd_clr_stats_i,
  output logic [31:0]                 stat_ar_bursts_o,
  output logic [31:0]                 stat_r_beats_o,
  output logic [31:0]                 stat_4kb_splits_o,
  output logic [31:0]                 stat_rresp_errs_o
);

  // ------------------------------------------------------------------------
  // Static checks
  // ------------------------------------------------------------------------
  initial begin
    if ((AXI_DATA_W % 8) != 0)                       $error("axi_mst_port1: AXI_DATA_W must be multiple of 8.");
    if (((AXI_BYTES) & (AXI_BYTES-1)) != 0)          $error("axi_mst_port1: AXI_BYTES must be power of two.");
    if (RD_MAX_BURST == 0 || RD_MAX_BURST > 256)     $error("axi_mst_port1: RD_MAX_BURST must be 1..256.");
  end

  // ------------------------------------------------------------------------
  // Small helpers
  // ------------------------------------------------------------------------
  function automatic int unsigned ceil_div(input int unsigned a, input int unsigned b);
    return (a + b - 1) / b;
  endfunction

  function automatic int unsigned beats_to_4kb(input logic [AXI_ADDR_W-1:0] a);
    int unsigned bytes_left = 12'd4096 - a[11:0];
    return (bytes_left / AXI_BYTES); // floor
  endfunction

  function automatic int unsigned choose_beats(
    input int unsigned bytes_left_in_row,
    input int unsigned start_off,
    input logic [AXI_ADDR_W-1:0] addr_now
  );
    int unsigned need_beats = ceil_div(start_off + bytes_left_in_row, AXI_BYTES);
    if (need_beats == 0) need_beats = 1;
    int unsigned lim4k    = beats_to_4kb(addr_now);         if (lim4k == 0) lim4k = 1;
    int unsigned lim_burst= (RD_MAX_BURST < need_beats) ? RD_MAX_BURST : need_beats;
    return (lim_burst < lim4k) ? lim_burst : lim4k;
  endfunction

  function automatic logic [AXI_BYTES-1:0]
    keep_reverse(input logic [AXI_BYTES-1:0] k);
    logic [AXI_BYTES-1:0] r;
    for (int i=0;i<AXI_BYTES;i++) r[AXI_BYTES-1-i] = k[i];
    return r;
  endfunction

  // ------------------------------------------------------------------------
  // Descriptor & top-level state
  // ------------------------------------------------------------------------
  typedef struct packed {
    logic [AXI_ADDR_W-1:0] base;
    logic [31:0]           width_b;
    logic [31:0]           rows;
    logic [AXI_ADDR_W-1:0] stride_b;
  } desc_t;

  desc_t desc_q;

  typedef enum logic [1:0] {S_IDLE, S_ISSUE, S_DRAIN, S_DONE_HOLD} state_e;
  state_e state_q, state_d;

  // Issue-side row iterator (what we *schedule* on AR)
  logic [31:0]           iss_row_idx_q, iss_row_idx_d;
  logic [AXI_ADDR_W-1:0] iss_row_addr_q, iss_row_addr_d;
  logic [$clog2(AXI_BYTES)-1:0] iss_off_q, iss_off_d;    // row_addr % AXI_BYTES
  logic [31:0]           iss_row_bytes_q, iss_row_bytes_d;

  // Outstanding counter and burst queue (for RLAST verification)
  logic [7:0]            outs_q, outs_d;

  localparam int ARQ_DEPTH = (RD_OUTS_MAX < 2) ? 2 : RD_OUTS_MAX;
  localparam int ARQ_AW    = (ARQ_DEPTH <= 2) ? 1 : $clog2(ARQ_DEPTH);
  logic [8:0]             arq_len [0:ARQ_DEPTH-1];   // beats per burst
  logic [ARQ_AW:0]        arq_wr_q, arq_rd_q;
  logic [8:0]             arq_head_left_q, arq_head_left_d;

  function automatic logic arq_empty(input logic [ARQ_AW:0] w, input logic [ARQ_AW:0] r);
    return (w == r);
  endfunction
  function automatic logic arq_full (input logic [ARQ_AW:0] w, input logic [ARQ_AW:0] r);
    return (w[ARQ_AW] != r[ARQ_AW]) && (w[ARQ_AW-1:0] == r[ARQ_AW-1:0]);
  endfunction
  function automatic logic [ARQ_AW:0] arq_inc(input logic [ARQ_AW:0] p);
    return p + {{ARQ_AW{1'b0}},1'b1};
  endfunction

  // Output-side row iterator (what we *emit* to downstream)
  logic [31:0]           out_row_idx_q, out_row_idx_d;
  logic [AXI_ADDR_W-1:0] out_row_addr_q, out_row_addr_d;
  logic [$clog2(AXI_BYTES)-1:0] out_off_q, out_off_d;
  logic [31:0]           out_row_bytes_q, out_row_bytes_d;

  // Total rows complete for *output*
  logic [31:0]           out_rows_done_q, out_rows_done_d;

  // Sticky RRESP error for this transfer
  logic                  err_sticky_q, err_sticky_d;
  assign done_err_o = err_sticky_q;

  // Telemetry
  logic [31:0] stat_ar_q, stat_r_q, stat_4k_q, stat_err_q;

  // ------------------------------------------------------------------------
  // R beat FIFO (decouple AXI R from consumer)
  // ------------------------------------------------------------------------
  localparam int RF_AW = (RD_FIFO_DEPTH <= 2) ? 1 : $clog2(RD_FIFO_DEPTH);
  logic [AXI_DATA_W-1:0] r_mem_data [0:RD_FIFO_DEPTH-1];
  logic [1:0]            r_mem_resp [0:RD_FIFO_DEPTH-1];
  logic                  r_mem_rlast[0:RD_FIFO_DEPTH-1];

  logic [RF_AW:0]        r_wr_q, r_rd_q;

  function automatic logic r_empty(input logic [RF_AW:0] w, input logic [RF_AW:0] r);
    return (w == r);
  endfunction
  function automatic logic r_full (input logic [RF_AW:0] w, input logic [RF_AW:0] r);
    return (w[RF_AW] != r[RF_AW]) && (w[RF_AW-1:0] == r[RF_AW-1:0]);
  endfunction
  function automatic logic [RF_AW:0] r_inc(input logic [RF_AW:0] p);
    return p + {{RF_AW{1'b0}},1'b1};
  endfunction

  // AXI R channel ready depends only on FIFO fullness
  assign M_RREADY = !r_full(r_wr_q, r_rd_q);

  wire r_fire = M_RVALID && M_RREADY;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      r_wr_q <= '0;
    end else if (r_fire) begin
      r_mem_data[r_wr_q[RF_AW-1:0]] <= M_RDATA;
      r_mem_resp[r_wr_q[RF_AW-1:0]] <= M_RRESP;
      r_mem_rlast[r_wr_q[RF_AW-1:0]]<= M_RLAST;
      r_wr_q <= r_inc(r_wr_q);
    end
  end

  // ------------------------------------------------------------------------
  // Descriptor accept
  // ------------------------------------------------------------------------
  assign desc_ready_o = (state_q == S_IDLE);
  wire   desc_fire    = desc_valid_i && desc_ready_o;

  // ------------------------------------------------------------------------
  // AR generation
  // ------------------------------------------------------------------------
  // When issuing, we use iss_* (issue-side row cursor).
  wire                       have_rows_to_issue = (iss_row_idx_q < desc_q.rows);
  wire                       can_issue          = (outs_q < RD_OUTS_MAX) && !arq_full(arq_wr_q, arq_rd_q);
  int unsigned               beats_next;
  always_comb begin
    beats_next = (have_rows_to_issue)
               ? choose_beats(iss_row_bytes_q, iss_off_q, iss_row_addr_q)
               : 1;
  end
  wire [7:0]                 ar_len_next = (beats_next==0) ? 8'd0 : 8'(beats_next-1);

  assign M_ARVALID = (state_q == S_ISSUE) && have_rows_to_issue && can_issue;
  assign M_ARID    = P_ID;
  assign M_ARADDR  = iss_row_addr_q;
  assign M_ARLEN   = ar_len_next;
  assign M_ARSIZE  = $clog2(AXI_BYTES);
  assign M_ARBURST = 2'b01; // INCR
  assign M_ARCACHE = P_CACHE;
  assign M_ARPROT  = P_PROT;
  assign M_ARQOS   = P_QOS;
  assign M_ARLOCK  = 1'b0;

  wire ar_fire = M_ARVALID && M_ARREADY;

  // ------------------------------------------------------------------------
  // State registers / descriptor latch
  // ------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q        <= S_IDLE;
      desc_q         <= '0;

      iss_row_idx_q  <= 32'd0;
      iss_row_addr_q <= '0;
      iss_off_q      <= '0;
      iss_row_bytes_q<= 32'd0;

      outs_q         <= 8'd0;
      arq_wr_q       <= '0;
      arq_rd_q       <= '0;
      arq_head_left_q<= 9'd0;

      out_row_idx_q  <= 32'd0;
      out_row_addr_q <= '0;
      out_off_q      <= '0;
      out_row_bytes_q<= 32'd0;
      out_rows_done_q<= 32'd0;

      err_sticky_q   <= 1'b0;

      r_rd_q         <= '0;
    end else begin
      state_q        <= state_d;
      desc_q         <= desc_q;

      iss_row_idx_q  <= iss_row_idx_d;
      iss_row_addr_q <= iss_row_addr_d;
      iss_off_q      <= iss_off_d;
      iss_row_bytes_q<= iss_row_bytes_d;

      outs_q         <= outs_d;
      arq_wr_q       <= arq_wr_q;
      arq_rd_q       <= arq_rd_q;
      arq_head_left_q<= arq_head_left_d;

      out_row_idx_q  <= out_row_idx_d;
      out_row_addr_q <= out_row_addr_d;
      out_off_q      <= out_off_d;
      out_row_bytes_q<= out_row_bytes_d;
      out_rows_done_q<= out_rows_done_d;

      err_sticky_q   <= (cmd_clr_stats_i) ? 1'b0
                        : (r_fire && (M_RRESP != 2'b00)) ? 1'b1 : err_sticky_q;

      if (desc_fire) begin
        // Latch descriptor
        desc_q.base     <= desc_base_addr_i;
        desc_q.width_b  <= desc_width_bytes_i;
        desc_q.rows     <= desc_rows_i;
        desc_q.stride_b <= desc_row_stride_i;

        // Initialize issue cursors
        iss_row_idx_q   <= 32'd0;
        iss_row_addr_q  <= desc_base_addr_i;
        iss_off_q       <= desc_base_addr_i[$clog2(AXI_BYTES)-1:0];
        iss_row_bytes_q <= desc_width_bytes_i;

        // Initialize output cursors
        out_row_idx_q   <= 32'd0;
        out_row_addr_q  <= desc_base_addr_i;
        out_off_q       <= desc_base_addr_i[$clog2(AXI_BYTES)-1:0];
        out_row_bytes_q <= desc_width_bytes_i;
        out_rows_done_q <= 32'd0;

        // Clear outstanding & queues
        outs_q          <= 8'd0;
        arq_wr_q        <= '0;
        arq_rd_q        <= '0;
        arq_head_left_q <= 9'd0;

        // Clear R FIFO pointers
        r_rd_q          <= '0;
      end
    end
  end

  // ------------------------------------------------------------------------
  // AR queue bookkeeping (push on AR fire; decrement on R fire)
  // ------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      // already reset above
    end else begin
      // Push on AR fire
      if (ar_fire) begin
        logic [8:0] beats = 9'({1'b0, M_ARLEN}) + 9'd1;
        arq_len[arq_wr_q[ARQ_AW-1:0]] <= beats;
        arq_wr_q <= arq_inc(arq_wr_q);
        // If queue was empty, new head becomes beats
        if (arq_wr_q == arq_rd_q) begin
          arq_head_left_q <= beats;
        end
        // Advance issue-side cursor
        // Compute payload bytes covered by this burst (account for first off)
        int unsigned cover_payload = (iss_off_q != 0)
                                   ? ((beats * AXI_BYTES > iss_off_q)
                                       ? ((beats * AXI_BYTES) - iss_off_q) : 0)
                                   : (beats * AXI_BYTES);
        if (cover_payload > iss_row_bytes_q) cover_payload = iss_row_bytes_q;

        iss_row_bytes_q <= iss_row_bytes_q - cover_payload;
        iss_off_q       <= '0;
        iss_row_addr_q  <= iss_row_addr_q + (beats * AXI_BYTES);

        // If row finished, advance to next row base
        if (iss_row_bytes_q - cover_payload == 0) begin
          iss_row_idx_q  <= iss_row_idx_q + 32'd1;
          iss_row_addr_q <= desc_q.base + ( (iss_row_idx_q + 32'd1) * desc_q.stride_b );
          iss_off_q      <= (desc_q.base + ( (iss_row_idx_q + 32'd1) * desc_q.stride_b ))[$clog2(AXI_BYTES)-1:0];
          iss_row_bytes_q<= desc_q.width_b;
        end

        // Outstanding++
        outs_q <= outs_q + 8'd1;
      end

      // On R acceptance, decrement current head beats; when it reaches 0 pop
      if (r_fire) begin
        if (arq_head_left_q != 0) begin
          arq_head_left_q <= arq_head_left_q - 9'd1;
          if (arq_head_left_q == 9'd1) begin
            // pop head
            arq_rd_q <= arq_inc(arq_rd_q);
            // load next head if any
            if (arq_rd_q != arq_wr_q - {{ARQ_AW{1'b0}},1'b1}) begin
              arq_head_left_q <= arq_len[arq_rd_q[ARQ_AW-1:0] + 1'b1];
            end
            // Outstanding--
            if (outs_q != 0) outs_q <= outs_q - 8'd1;
          end
        end
      end
    end
  end

  // ------------------------------------------------------------------------
  // State transition (issue/drain/done)
  // ------------------------------------------------------------------------
  always_comb begin
    state_d = state_q;

    case (state_q)
      S_IDLE: begin
        if (desc_fire) state_d = S_ISSUE;
      end

      S_ISSUE: begin
        // Stay issuing bursts while we have rows and can issue. We can move to
        // DRAIN when all rows have been fully scheduled (iss_row_idx_q == rows)
        // and no more AR can reduce that (bytes == 0).
        if ((iss_row_idx_q >= desc_q.rows) && (outs_q != 0 || !r_empty(r_wr_q, r_rd_q)))
          state_d = S_DRAIN;
      end

      S_DRAIN: begin
        // After all bursts return and FIFO drains to consumer, complete.
        if (outs_q == 0 && r_empty(r_wr_q, r_rd_q)) state_d = S_DONE_HOLD;
      end

      S_DONE_HOLD: begin
        // Done signaled via done_valid_o; return to IDLE when consumed.
        if (done_valid_o && done_ready_i) state_d = S_IDLE;
      end

      default: state_d = S_IDLE;
    endcase
  end

  // ------------------------------------------------------------------------
  // AXI R → OUT stream (compute keep/row_last/last)
  // ------------------------------------------------------------------------
  // Output is valid when FIFO not empty and engine not idle.
  assign out_valid_o = !r_empty(r_wr_q, r_rd_q) && (state_q != S_IDLE);

  // Data is just the FIFO head word (no endian transform — consumer can handle)
  assign out_data_o  = r_mem_data[r_rd_q[RF_AW-1:0]];

  // Keep mask (LE space); reverse for BE if requested
  logic [AXI_BYTES-1:0] keep_le_w;
  always_comb begin
    keep_le_w = '0;
    if (out_row_bytes_q != 0) begin
      int unsigned first_bytes = (out_off_q == 0) ? AXI_BYTES : (AXI_BYTES - out_off_q);
      int unsigned val_bytes   = (out_row_bytes_q > first_bytes) ? first_bytes : out_row_bytes_q;
      if (out_off_q == 0) begin
        for (int b=0; b<val_bytes; b++) keep_le_w[b] = 1'b1;
      end else begin
        for (int b=0; b<val_bytes; b++) keep_le_w[out_off_q + b] = 1'b1;
      end
    end
  end

  assign out_keep_o   = cfg_little_endian_i ? keep_le_w : keep_reverse(keep_le_w);

  // Row-last / transfer-last flags
  assign out_row_last_o = (out_row_bytes_q <= ((out_off_q==0)? AXI_BYTES : (AXI_BYTES - out_off_q))) &&
                          (out_row_bytes_q != 0);
  assign out_last_o     = out_row_last_o && (out_row_idx_q + 32'd1 == desc_q.rows);

  // Pop on consumer accept
  wire out_fire = out_valid_o && out_ready_i;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      r_rd_q           <= '0;
      out_row_bytes_q  <= 32'd0;
      out_off_q        <= '0;
      out_row_idx_q    <= 32'd0;
      out_row_addr_q   <= '0;
      out_rows_done_q  <= 32'd0;
    end else if (out_fire) begin
      r_rd_q <= r_inc(r_rd_q);

      int unsigned first_bytes = (out_off_q == 0) ? AXI_BYTES : (AXI_BYTES - out_off_q);
      int unsigned val_bytes   = (out_row_bytes_q > first_bytes) ? first_bytes : out_row_bytes_q;

      // Consume bytes in current row
      out_row_bytes_q <= out_row_bytes_q - val_bytes;

      // After first emitted beat of a row, offset becomes zero.
      out_off_q <= '0;

      // Completed row?
      if (val_bytes == out_row_bytes_q) begin
        // Move to next row
        out_rows_done_q <= out_rows_done_q + 32'd1;
        out_row_idx_q   <= out_row_idx_q + 32'd1;

        // Initialize next row counters if not the last row emitted
        if (out_row_idx_q + 32'd1 < desc_q.rows) begin
          logic [AXI_ADDR_W-1:0] next_addr = desc_q.base + ( (out_row_idx_q + 32'd1) * desc_q.stride_b );
          out_row_addr_q  <= next_addr;
          out_off_q       <= next_addr[$clog2(AXI_BYTES)-1:0];
          out_row_bytes_q <= desc_q.width_b;
        end
      end
    end

    if (desc_fire) begin
      // Initialize output cursors for new transfer (already set in reg block)
    end
  end

  // ------------------------------------------------------------------------
  // Done pulse & sticky error; simple 1-entry "done" holder
  // ------------------------------------------------------------------------
  logic done_v_q, done_v_d;

  assign done_valid_o = done_v_q;
  assign done_err_o   = err_sticky_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) done_v_q <= 1'b0;
    else begin
      if (state_q == S_DRAIN && state_d == S_DONE_HOLD) begin
        done_v_q <= 1'b1;
      end else if (done_valid_o && done_ready_i) begin
        done_v_q <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------------
  // Update AR/issue-side state progression in comb
  // ------------------------------------------------------------------------
  always_comb begin
    state_d          = state_q;

    iss_row_idx_d    = iss_row_idx_q;
    iss_row_addr_d   = iss_row_addr_q;
    iss_off_d        = iss_off_q;
    iss_row_bytes_d  = iss_row_bytes_q;

    out_row_idx_d    = out_row_idx_q;
    out_row_addr_d   = out_row_addr_q;
    out_off_d        = out_off_q;
    out_row_bytes_d  = out_row_bytes_q;
    out_rows_done_d  = out_rows_done_q;

    arq_head_left_d  = arq_head_left_q;
    outs_d           = outs_q;

    err_sticky_d     = err_sticky_q;

    // State transitions handled above (state_d)
  end

  // ------------------------------------------------------------------------
  // Telemetry
  // ------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      stat_ar_q  <= 32'd0;
      stat_r_q   <= 32'd0;
      stat_4k_q  <= 32'd0;
      stat_err_q <= 32'd0;
    end else if (cmd_clr_stats_i) begin
      stat_ar_q  <= 32'd0;
      stat_r_q   <= 32'd0;
      stat_4k_q  <= 32'd0;
      stat_err_q <= 32'd0;
    end else begin
      if (ar_fire) begin
        stat_ar_q <= stat_ar_q + 32'd1;
        if (beats_to_4kb(M_ARADDR) <= (8'(M_ARLEN)+1)) stat_4k_q <= stat_4k_q + 32'd1;
      end
      if (r_fire) begin
        stat_r_q <= stat_r_q + 32'd1;
        if (M_RRESP != 2'b00) stat_err_q <= stat_err_q + 32'd1;
      end
    end
  end

  assign stat_ar_bursts_o  = stat_ar_q;
  assign stat_r_beats_o    = stat_r_q;
  assign stat_4kb_splits_o = stat_4k_q;
  assign stat_rresp_errs_o = stat_err_q;

  // ------------------------------------------------------------------------
  // Assertions (simulation only)
  // ------------------------------------------------------------------------
`ifdef ASSERT_ON
  // Descriptor sanity
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    desc_valid_i && desc_ready_o |-> (desc_width_bytes_i > 0 && desc_rows_i > 0))
    else $error("axi_mst_port1: width/rows must be > 0.");

  // R channel ID must match our single ID (if any)
  if (AXI_ID_W > 0) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      M_RVALID |-> (M_RID == P_ID))
      else $warning("axi_mst_port1: R.ID mismatch (ignored).");
  end

  // ARLEN range
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    M_ARVALID |-> (M_ARLEN <= 8'd255))
    else $error("axi_mst_port1: ARLEN out of range.");

  // No 4KB crossing
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    M_ARVALID && M_ARREADY |->
      (M_ARADDR[AXI_ADDR_W-1:12] ==
       (M_ARADDR + ( (9'({1'b0,M_ARLEN}) + 9'd1) * AXI_BYTES ) - 1)[AXI_ADDR_W-1:12]))
    else $error("axi_mst_port1: AR burst crosses 4KB boundary.");

  // Burst tracker alignment: when head_left==1 on accepted beat, RLAST must be 1
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    M_RVALID && M_RREADY && (arq_head_left_q == 9'd1) |-> M_RLAST)
    else $error("axi_mst_port1: RLAST not asserted at end of burst.");

  // OUT 'last' implies 'row_last'
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    out_last_o |-> out_row_last_o)
    else $error("axi_mst_port1: transfer last without row last.");
`endif

endmodule

`default_nettype wire
