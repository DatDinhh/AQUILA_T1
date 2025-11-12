// ============================================================================
//  dma2d_rd2gb_core
//  Purpose: AXI4 Read -> GB write, 2D copy engine with doorbell+descriptor
//  Notes:
//    * STRICT_ALIGN=1 requires src_addr, dst_gb_addr, row_bytes all multiples
//      of (AXI_DATA_W/8). This yields the simplest and fastest datapath.
//    * The GB address is a BYTE address. The engine increments it by one beat
//      (AXI_DATA_W/8) per write.
//    * TODO hooks noted for enabling an unaligned realigner in a future rev.
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

module dma2d_rd2gb_core #(
  parameter int unsigned AXI_ADDR_W = 48,
  parameter int unsigned AXI_DATA_W = 512,
  parameter int unsigned AXI_ID_W   = 6,
  parameter int unsigned AXI_USER_W = 1,

  parameter logic [AXI_ADDR_W-1:0] DESC_ADDR_DFLT  = '0,
  parameter logic [AXI_ADDR_W-1:0] DBELL_ADDR_DFLT = '0,
  parameter int unsigned           POLL_CYCLES     = 4096,
  parameter int unsigned           MAX_OUTS        = 4,
  parameter int unsigned           FIFO_DEPTH      = 16,
  parameter bit                    STRICT_ALIGN    = 1'b1,

  parameter logic [AXI_ID_W-1:0]   AXI_FIXED_ID    = '0,
  parameter logic [3:0]            AXI_ARQOS       = 4'h0,
  parameter logic [3:0]            AXI_ARCACHE     = 4'hF,
  parameter logic [2:0]            AXI_ARPROT      = 3'b000
)(
  input  logic                         clk_i,
  input  logic                         rstn_i,

  // AXI Read Master
  output logic [AXI_ID_W-1:0]          m_arid_o,
  output logic [AXI_ADDR_W-1:0]        m_araddr_o,
  output logic [7:0]                   m_arlen_o,
  output logic [2:0]                   m_arsize_o,
  output logic [1:0]                   m_arburst_o,
  output logic                         m_arlock_o,
  output logic [3:0]                   m_arcache_o,
  output logic [2:0]                   m_arprot_o,
  output logic [3:0]                   m_arqos_o,
  output logic [AXI_USER_W-1:0]        m_aruser_o,
  output logic                         m_arvalid_o,
  input  logic                         m_arready_i,

  input  logic [AXI_ID_W-1:0]          m_rid_i,
  input  logic [AXI_DATA_W-1:0]        m_rdata_i,
  input  logic [1:0]                   m_rresp_i,
  input  logic                         m_rlast_i,
  input  logic [AXI_USER_W-1:0]        m_ruser_i,
  input  logic                         m_rvalid_i,
  output logic                         m_rready_o,

  // GB write side (byte-addressed)
  output logic                         gb_w_req_o,
  output logic [31:0]                  gb_w_addr_o,
  output logic [AXI_DATA_W/8-1:0]      gb_w_strb_o,
  output logic [AXI_DATA_W-1:0]        gb_w_data_o,
  input  logic                         gb_w_ready_i,

  // Interrupt pulse on completion (and on error)
  output logic                         irq_o
);

  // --------------------------------------------
  // Constants, typedefs
  // --------------------------------------------
  localparam int unsigned BEAT_BYTES = AXI_DATA_W/8;
  localparam int unsigned ARSIZE     = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);

  // Descriptor layout (32 bytes). We read 1 aligned AXI beat and extract.
  typedef struct packed {
    logic [63:0]  src_addr;
    logic [31:0]  dst_gb_addr;
    logic [31:0]  row_bytes;
    logic [15:0]  rows;
    logic [15:0]  flags;
    logic signed [31:0] src_stride_bytes;
    logic signed [31:0] dst_stride_bytes;
    logic [15:0]  burst_bytes_hint;
    logic [15:0]  rsvd;
  } desc_t;

  // Error codes (for internal visibility / future CSR hookup)
  localparam logic [3:0]
    EC_NONE        = 4'h0,
    EC_ALIGN       = 4'h1,
    EC_AXI_RESP    = 4'h2,
    EC_PARAM       = 4'h3;

  // --------------------------------------------
  // Registers / state
  // --------------------------------------------
  logic [AXI_ADDR_W-1:0] dbell_addr_q, desc_addr_q;
  logic [31:0]           last_dbell_q;
  logic [31:0]           poll_cnt_q;

  // Descriptor shadow
  desc_t                 desc_q;

  // Transfer bookkeeping
  logic [31:0]           beats_row_total_q;   // row_bytes / BEAT_BYTES
  logic [31:0]           beats_row_left_q;
  logic [31:0]           rows_left_q;

  logic [AXI_ADDR_W-1:0] src_addr_q;
  logic [31:0]           dst_addr_q;          // GB byte address

  // AXI issue / credit
  logic [15:0]           outs_bursts_q;       // number of in-flight bursts
  logic [15:0]           outs_beats_q;        // in-flight beats (optional guard)

  // IRQ pulse
  logic                  irq_pulse_d, irq_pulse_q;

  // Error
  logic [3:0]            err_code_q;

  // --------------------------------------------
  // Simple R-beat FIFO (depth FIFO_DEPTH)
  // --------------------------------------------
  localparam int unsigned F_AW = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);

  logic [AXI_DATA_W-1:0] rf_mem   [0:FIFO_DEPTH-1];
  logic [F_AW:0]         rf_wptr_q, rf_rptr_q;  // one extra bit for full/empty
  logic                  rf_full, rf_empty;
  logic                  rf_push, rf_pop;

  assign rf_full  = (rf_wptr_q[F_AW]     != rf_rptr_q[F_AW]) &&
                    (rf_wptr_q[F_AW-1:0] == rf_rptr_q[F_AW-1:0]);
  assign rf_empty = (rf_wptr_q == rf_rptr_q);

  logic [AXI_DATA_W-1:0] rf_rdata;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rf_wptr_q <= '0;
      rf_rptr_q <= '0;
    end else begin
      if (rf_push && !rf_full) begin
        rf_mem[rf_wptr_q[F_AW-1:0]] <= m_rdata_i;
        rf_wptr_q <= rf_wptr_q + 1'b1;
      end
      if (rf_pop && !rf_empty) begin
        rf_rptr_q <= rf_rptr_q + 1'b1;
      end
    end
  end
  assign rf_rdata = rf_mem[rf_rptr_q[F_AW-1:0]];

  // --------------------------------------------
  // AXI defaults
  // --------------------------------------------
  assign m_arid_o    = AXI_FIXED_ID;
  assign m_arsize_o  = ARSIZE[2:0];
  assign m_arburst_o = 2'b01; // INCR
  assign m_arlock_o  = 1'b0;
  assign m_arcache_o = AXI_ARCACHE;
  assign m_arprot_o  = AXI_ARPROT;
  assign m_arqos_o   = AXI_ARQOS;
  assign m_aruser_o  = '0;

  // R channel ready when FIFO has space
  assign m_rready_o  = !rf_full;

  // --------------------------------------------
  // Top FSM
  // --------------------------------------------
  typedef enum logic [3:0] {
    S_RESET, S_IDLE, S_POLL_SETUP, S_POLL_ACCESS, S_POLL_WAIT,
    S_READ_DESC_SETUP, S_READ_DESC_ACCESS, S_READ_DESC_WAIT,
    S_VALIDATE, S_PREP_ROW, S_ISSUE, S_RUN, S_NEXT_ROW, S_DONE, S_ERROR
  } state_e;

  state_e state_q, state_d;

  // AR command registers
  logic               ar_fire;
  logic [AXI_ADDR_W-1:0] ar_addr_d, ar_addr_q;
  logic [7:0]         ar_len_d,  ar_len_q;
  logic               ar_valid_d, ar_valid_q;

  // Track rresp errors
  logic               rresp_err_d, rresp_err_q;

  // Doorbell sample
  logic [31:0]        dbell_sample_q;

  // Burst planning helpers
  function automatic [7:0] calc_burst_len(input logic [AXI_ADDR_W-1:0] addr,
                                          input logic [31:0] beats_left,
                                          input logic [31:0] beats_hint);
    // Limit by 256 beats and 4KiB boundary
    int unsigned bytes_to_4k = 4096 - (addr[11:0]);
    int unsigned max_beats_4k = bytes_to_4k / BEAT_BYTES;
    int unsigned cap = (beats_hint == 0) ? 256 : (beats_hint > 256 ? 256 : beats_hint);
    int unsigned m1 = (beats_left  < cap) ? beats_left  : cap;
    int unsigned m2 = (max_beats_4k < m1) ? max_beats_4k : m1;
    calc_burst_len = (m2 == 0) ? 8'd1 : 8'(m2); // never emit 0; minimum 1 beat
  endfunction

  // Beat bytes as constant strobe (aligned path)
  localparam logic [AXI_DATA_W/8-1:0] ALL_BYTES = { (AXI_DATA_W/8){1'b1} };

  // --------------------------------------------
  // Sequential
  // --------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q       <= S_RESET;

      dbell_addr_q  <= DBELL_ADDR_DFLT;
      desc_addr_q   <= DESC_ADDR_DFLT;
      last_dbell_q  <= 32'hDEAD_BEEF;
      poll_cnt_q    <= '0;

      ar_addr_q     <= '0;
      ar_len_q      <= 8'd0;
      ar_valid_q    <= 1'b0;

      rresp_err_q   <= 1'b0;

      outs_bursts_q <= '0;
      outs_beats_q  <= '0;

      irq_pulse_q   <= 1'b0;
      err_code_q    <= EC_NONE;

      // GB side defaults
      gb_w_req_o    <= 1'b0;
      gb_w_addr_o   <= '0;
      gb_w_data_o   <= '0;
      gb_w_strb_o   <= ALL_BYTES;

      // Descriptor reset
      desc_q        <= '0;
      beats_row_total_q <= '0;
      beats_row_left_q  <= '0;
      rows_left_q       <= '0;
      src_addr_q        <= '0;
      dst_addr_q        <= '0;
    end else begin
      state_q     <= state_d;

      // doorbell poll cadence
      if (state_q == S_IDLE) begin
        poll_cnt_q <= poll_cnt_q + 32'd1;
      end else begin
        poll_cnt_q <= '0;
      end

      // AR command registers
      ar_addr_q   <= ar_addr_d;
      ar_len_q    <= ar_len_d;
      ar_valid_q  <= ar_valid_d;

      // Track R channel outstanding
      if (m_arvalid_o && m_arready_i) begin
        outs_bursts_q <= outs_bursts_q + 16'd1;
        outs_beats_q  <= outs_beats_q + {8'd0, (ar_len_q + 8'd1)};
      end
      if (m_rvalid_i && m_rready_o) begin
        if (m_rlast_i && outs_bursts_q != 0) outs_bursts_q <= outs_bursts_q - 16'd1;
        if (outs_beats_q  != 0)              outs_beats_q  <= outs_beats_q - 16'd1;
      end

      // R push & error capture
      rresp_err_q <= rresp_err_q | (m_rvalid_i && (m_rresp_i != 2'b00));
      if (m_rvalid_i && m_rready_o) begin
        // Push into FIFO
        // (Guarded combinationally by rf_full==0 via m_rready_o)
      end

      // IRQ pulse one-shot
      irq_pulse_q <= irq_pulse_d;

      // GB write request default deassert; driven in RUN
      if (!(gb_w_req_o && !gb_w_ready_i)) begin
        gb_w_req_o <= 1'b0;
      end
    end
  end

  // --------------------------------------------
  // Combinational defaults
  // --------------------------------------------
  assign irq_o = irq_pulse_q;

  // AR channel drive
  assign m_araddr_o  = ar_addr_q;
  assign m_arlen_o   = ar_len_q;
  assign m_arvalid_o = ar_valid_q;

  // FIFO push/pop controls
  assign rf_push     = m_rvalid_i && m_rready_o;
  assign rf_pop      = (state_q == S_RUN) && gb_w_ready_i && !rf_empty;

  // --------------------------------------------
  // Next-state logic
  // --------------------------------------------
  always_comb begin
    state_d      = state_q;
    irq_pulse_d  = 1'b0;

    // Default: keep AR idle unless set in a state
    ar_valid_d   = 1'b0;
    ar_addr_d    = ar_addr_q;
    ar_len_d     = ar_len_q;

    // GB write defaults; driven in RUN
    // (gb_w_* are registered in seq block)

    unique case (state_q)

      // --------------------------
      S_RESET: begin
        state_d = S_IDLE;
      end

      // --------------------------
      S_IDLE: begin
        // Periodically poll the doorbell
        if (poll_cnt_q >= POLL_CYCLES[31:0]) begin
          ar_addr_d   = dbell_addr_q & {{(AXI_ADDR_W-2){1'b1}}, 2'b00}; // align to 4B
          ar_len_d    = 8'd0;    // 1 beat of BEAT_BYTES, doorbell occupies lower 4B
          ar_valid_d  = 1'b1;
          state_d     = S_POLL_SETUP;
        end
      end

      S_POLL_SETUP: begin
        // Handshake AR
        if (ar_valid_q && m_arready_i) begin
          state_d = S_POLL_WAIT;
        end else begin
          // Keep driving until accepted
          ar_valid_d = 1'b1;
        end
      end

      S_POLL_WAIT: begin
        // Wait for single-beat R
        if (m_rvalid_i && m_rready_o) begin
          // Sample doorbell word (lower 32b of first beat)
          dbell_sample_q = m_rdata_i[31:0];
          if (m_rlast_i) begin
            if (dbell_sample_q != last_dbell_q) begin
              // Fetch descriptor next
              // Require descriptor aligned to beat
              ar_addr_d  = (desc_addr_q & {{(AXI_ADDR_W- $clog2(BEAT_BYTES)){1'b1}}, {$clog2(BEAT_BYTES){1'b0}}});
              ar_len_d   = 8'd0;     // read exactly 1 beat; descriptor is within
              ar_valid_d = 1'b1;
              state_d    = S_READ_DESC_SETUP;
            end else begin
              state_d = S_IDLE;
            end
          end
        end
      end

      S_READ_DESC_SETUP: begin
        if (ar_valid_q && m_arready_i) begin
          state_d = S_READ_DESC_WAIT;
        end else begin
          ar_valid_d = 1'b1;
        end
      end

      S_READ_DESC_WAIT: begin
        // Capture descriptor from lower 256b of the beat
        if (m_rvalid_i && m_rready_o && m_rlast_i) begin
          // Parse fields
          desc_t desc_tmp;
          desc_tmp.src_addr         = m_rdata_i[63:0];
          desc_tmp.dst_gb_addr      = m_rdata_i[95:64];
          desc_tmp.row_bytes        = m_rdata_i[127:96];
          desc_tmp.rows             = m_rdata_i[143:128];
          desc_tmp.flags            = m_rdata_i[159:144];
          desc_tmp.src_stride_bytes = m_rdata_i[191:160];
          desc_tmp.dst_stride_bytes = m_rdata_i[223:192];
          desc_tmp.burst_bytes_hint = m_rdata_i[239:224];
          desc_tmp.rsvd             = m_rdata_i[255:240];

          // Latch descriptor
          // (seq logic writes desc_q on next clock by mirroring desc_tmp,
          //  but we fold it here: we’ll reuse in VALIDATE from m_rdata_i.)
          // For clarity, just use desc_tmp as implied current descriptor:

          // Alignment/parameter checks done in next state
          state_d = S_VALIDATE;

          // Use blocking assigns to module regs via automatic variables:
          // (We cannot write desc_q here; it’s sequential. We'll recompute below.)
        end
      end

      S_VALIDATE: begin
        // Re-decode from latched (one-cycle earlier) beat
        // In practice, we hold a shadow 'desc_q' in seq; here we just proceed.

        // Compute derived
        logic [63:0] src_addr  = m_rdata_i[63:0];
        logic [31:0] dst_addr  = m_rdata_i[95:64];
        logic [31:0] row_bytes = m_rdata_i[127:96];
        logic [15:0] rows      = m_rdata_i[143:128];
        logic [15:0] flags     = m_rdata_i[159:144];
        logic signed [31:0] src_stride = m_rdata_i[191:160];
        logic signed [31:0] dst_stride = m_rdata_i[223:192];
        logic [15:0] burst_hint= m_rdata_i[239:224];

        // Alignment enforcement (STRICT)
        bit ok = 1'b1;
        if (STRICT_ALIGN) begin
          if ((src_addr  % BEAT_BYTES) != 0) ok = 1'b0;
          if ((dst_addr  % BEAT_BYTES) != 0) ok = 1'b0;
          if ((row_bytes % BEAT_BYTES) != 0) ok = 1'b0;
        end
        if (row_bytes == 0 || rows == 0) ok = 1'b0;

        if (!ok) begin
          // Error: parameters invalid
          state_d   = S_ERROR;
        end else begin
          // Commit descriptor and derived counters
          // (We commit into the seq regs via qbanks: compute values to be used in PREP_ROW)
          // Initialize current addresses and counters
          // Note: seq regs are assigned on clock edge; we set combinationally here then
          // move to PREP_ROW where they’re used.

          state_d = S_PREP_ROW;
        end
      end

      S_PREP_ROW: begin
        // Initialize counters for the first row
        // beats_row_total_q = row_bytes / BEAT_BYTES
        // rows_left_q       = rows
        // src_addr_q, dst_addr_q set to base
        // Note: these assignments occur in the sequential block; we just advance state.
        state_d = S_ISSUE;
      end

      S_ISSUE: begin
        // Issue AR bursts while:
        // - We have beats to fetch for the current row
        // - Outstanding bursts < MAX_OUTS
        // - FIFO has enough space (heuristic)
        if ((beats_row_left_q != 0) &&
            (outs_bursts_q < MAX_OUTS) &&
            ((rf_full == 1'b0))) begin

          // Burst length planning
          // Use burst_hint (in beats) but cap by remaining beats and 4KiB boundary
          int unsigned beats_left = beats_row_left_q;
          int unsigned hint_beats = (desc_q.burst_bytes_hint == 0) ?
                                    256 : (desc_q.burst_bytes_hint / BEAT_BYTES);
          logic [7:0] blen = calc_burst_len(src_addr_q, beats_left, hint_beats);

          ar_addr_d  = src_addr_q;
          ar_len_d   = blen - 8'd1;
          ar_valid_d = 1'b1;

          if (ar_valid_q && m_arready_i) begin
            // Commit address advance when accepted
            // (seq: src_addr_q += blen*BEAT_BYTES; beats_row_left_q -= blen)
          end
        end else if ((beats_row_left_q == 0) && (outs_beats_q == 0) && (rf_empty)) begin
          // Done issuing & receiving for current row
          state_d = S_RUN; // proceed to draining FIFO if any (should be empty), then to NEXT_ROW
        end else begin
          // Hold until R channel completes and FIFO drains
          state_d = S_RUN;
        end
      end

      S_RUN: begin
        // Write FIFO payload into GB as ready allows
        if (!rf_empty && gb_w_ready_i) begin
          // Drive req and payload; seq will hold if ready=0
          // (Data/addr registered in seq)
        end

        // When row completely written: beats_row_left_q==0 && fifo empty && no outstanding
        if ((beats_row_left_q == 0) && (rf_empty) && (outs_beats_q == 0)) begin
          state_d = S_NEXT_ROW;
        end
      end

      S_NEXT_ROW: begin
        if (rows_left_q > 1) begin
          // Advance to next row (seq does address += stride and counters reload)
          state_d = S_ISSUE;
        end else begin
          state_d = S_DONE;
        end
      end

      S_DONE: begin
        // Latch doorbell (so we won’t re-run the same work), raise IRQ pulse
        irq_pulse_d = 1'b1;
        state_d     = S_IDLE;
      end

      S_ERROR: begin
        // Pulse IRQ to signal error; remain usable (will re-poll)
        irq_pulse_d = 1'b1;
        state_d     = S_IDLE;
      end

      default: state_d = S_RESET;
    endcase
  end

  // --------------------------------------------
  // Side-effects & counters driven alongside FSM
  // --------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      // handled above
    end else begin
      // On finishing S_POLL_WAIT, update last_dbell if we are going to run
      if (state_q == S_POLL_WAIT && m_rvalid_i && m_rready_o && m_rlast_i) begin
        if (dbell_sample_q != last_dbell_q) begin
          last_dbell_q <= dbell_sample_q;
        end
      end

      // Latch descriptor in S_READ_DESC_WAIT completion
      if (state_q == S_READ_DESC_WAIT && m_rvalid_i && m_rready_o && m_rlast_i) begin
        desc_q.src_addr          <= m_rdata_i[63:0];
        desc_q.dst_gb_addr       <= m_rdata_i[95:64];
        desc_q.row_bytes         <= m_rdata_i[127:96];
        desc_q.rows              <= m_rdata_i[143:128];
        desc_q.flags             <= m_rdata_i[159:144];
        desc_q.src_stride_bytes  <= m_rdata_i[191:160];
        desc_q.dst_stride_bytes  <= m_rdata_i[223:192];
        desc_q.burst_bytes_hint  <= m_rdata_i[239:224];
        desc_q.rsvd              <= m_rdata_i[255:240];
      end

      // Commit validate → PREP_ROW derived counters
      if (state_q == S_VALIDATE && state_d == S_PREP_ROW) begin
        beats_row_total_q <= desc_q.row_bytes / BEAT_BYTES;
        beats_row_left_q  <= desc_q.row_bytes / BEAT_BYTES;
        rows_left_q       <= {16'h0, desc_q.rows};
        src_addr_q        <= desc_q.src_addr[AXI_ADDR_W-1:0];
        dst_addr_q        <= desc_q.dst_gb_addr;
        rresp_err_q       <= 1'b0;
        err_code_q        <= EC_NONE;
      end

      // AR accept updates
      if (m_arvalid_o && m_arready_i) begin
        src_addr_q       <= src_addr_q + ( (ar_len_q + 8'd1) * BEAT_BYTES );
        beats_row_left_q <= beats_row_left_q - (ar_len_q + 8'd1);
      end

      // GB writer (RUN): pop FIFO and write
      if (state_q == S_RUN && !rf_empty && gb_w_ready_i) begin
        gb_w_req_o   <= 1'b1;
        gb_w_data_o  <= rf_rdata;
        gb_w_addr_o  <= dst_addr_q;
        gb_w_strb_o  <= ALL_BYTES;
        dst_addr_q   <= dst_addr_q + BEAT_BYTES;
      end

      // NEXT_ROW bookkeeping
      if (state_q == S_NEXT_ROW && state_d == S_ISSUE) begin
        rows_left_q <= rows_left_q - 32'd1;
        src_addr_q  <= src_addr_q + desc_q.src_stride_bytes[AXI_ADDR_W-1:0];
        dst_addr_q  <= dst_addr_q + desc_q.dst_stride_bytes;
        beats_row_left_q <= beats_row_total_q;
      end

      // Error capture
      if ((state_q == S_RUN || state_q == S_ISSUE || state_q == S_POLL_WAIT || state_q == S_READ_DESC_WAIT)
           && rresp_err_q) begin
        err_code_q <= EC_AXI_RESP;
      end
    end
  end

  // --------------------------------------------
  // Assertions (sim only)
  // --------------------------------------------
`ifdef ASSERT_ON
  // AXI: never issue with len==0 and then not accept data
  // (Covered implicitly by m_rready gating).
  // Alignment constraints in STRICT mode:
  if (STRICT_ALIGN) begin
    // On PREP_ROW, assert desc align
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      (state_q == S_VALIDATE && state_d == S_PREP_ROW) |->
       ((desc_q.src_addr % BEAT_BYTES) == 0) &&
       ((desc_q.dst_gb_addr % BEAT_BYTES) == 0) &&
       ((desc_q.row_bytes % BEAT_BYTES) == 0))
      else $error("dma2d: STRICT_ALIGN violation");
  end
  // FIFO bounds
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(rf_push && rf_full)) else $error("dma2d: R FIFO overflow");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(rf_pop && rf_empty)) else $error("dma2d: R FIFO underflow");
`endif

endmodule
`default_nettype wire
