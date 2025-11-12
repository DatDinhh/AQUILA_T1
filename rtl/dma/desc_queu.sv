// ============================================================================
//  Descriptor Queue with Multi-Sink Dispatch
//  File: desc_queu.sv
//  Description:
//    - Enqueue descriptors (DESC_W wide) with a sink/class tag
//    - Per-sink pending FIFO (bitmap-based), issue handshake, and done(tag)
//    - Track in-flight per sink (credit-like), configurable max inflight
//    - Sticky error flags + assertions for misuse
//
//  Typical use:
//    * NSINK=3  (A/W/C DMAs).  DESC_W=256 (32-byte descriptor).
//    * Enqueue from µC via a small APB->stream shim.
//    * Each DMA connects to one sink (issue[*]/done[*]).
//
//  © 2025. MIT-style license (adjust as needed).
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module desc_queue #(
  // Number of consumers (sinks)
  parameter int unsigned NSINK          = 3,
  // Descriptor width (bits) — 256 fits your 32-byte descriptor
  parameter int unsigned DESC_W         = 256,
  // Queue depth (number of descriptors stored at once)
  parameter int unsigned DEPTH          = 32,
  // Max number of in-flight descriptors per sink
  parameter int unsigned MAX_INFLIGHT   = 2
)(
  input  logic                       clk_i,
  input  logic                       rstn_i,       // synchronous, active-low

  // ---------------- Enqueue interface (producer side) ----------------
  input  logic                       enq_valid_i,
  output logic                       enq_ready_o,
  input  logic [DESC_W-1:0]          enq_desc_i,
  // Class selects the sink: 0..NSINK-1
  input  logic [$clog2((NSINK>1)?NSINK:2)-1:0] enq_class_i,

  // ---------------- Per-sink issue (to consumers) ----------------
  output logic [NSINK-1:0]                           issue_valid_o,
  input  logic  [NSINK-1:0]                          issue_ready_i,
  output logic [NSINK-1:0][$clog2((DEPTH>1)?DEPTH:2)-1:0] issue_tag_o,
  output logic [NSINK-1:0][DESC_W-1:0]               issue_desc_o,

  // ---------------- Per-sink completion (from consumers) ----------------
  input  logic [NSINK-1:0]                           done_valid_i,
  input  logic [NSINK-1:0][$clog2((DEPTH>1)?DEPTH:2)-1:0] done_tag_i,

  // ---------------- Status / debug ----------------
  output logic                                       full_o,
  output logic [NSINK-1:0]                           empty_o,      // no pending for that sink
  output logic [$clog2(DEPTH+1)-1:0]                 free_count_o,
  output logic [NSINK-1:0][$clog2(DEPTH+1)-1:0]      pending_count_o,
  output logic [NSINK-1:0][$clog2(MAX_INFLIGHT+1)-1:0] inflight_count_o,

  // Sticky error flags (clear by reset)
  output logic                                       err_spurious_done_o, // done(tag) not issued
  output logic                                       err_double_done_o,   // same tag done twice
  output logic                                       err_overflow_o       // enqueue when full (shouldn't happen if enq_ready obeyed)
);

  // --------------------------------------------------------------------------
  // Derived local params
  // --------------------------------------------------------------------------
  localparam int unsigned TAG_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
  localparam int unsigned CLS_W = (NSINK <= 1) ? 1 : $clog2(NSINK);
  localparam int unsigned CNT_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH+1);
  localparam int unsigned INF_W = (MAX_INFLIGHT <= 1) ? 1 : $clog2(MAX_INFLIGHT+1);

  // Static sanity checks (synthesis time)
  initial begin
    if (NSINK < 1)       $error("desc_queue: NSINK must be >=1");
    if (DESC_W < 32)     $error("desc_queue: DESC_W too small");
    if (DEPTH < 2)       $warning("desc_queue: DEPTH<2 is allowed but not practical");
    if (MAX_INFLIGHT < 1)$error("desc_queue: MAX_INFLIGHT must be >=1");
  end

  // --------------------------------------------------------------------------
  // Storage for descriptors
  // --------------------------------------------------------------------------
  (* ram_style = "distributed" *)
  logic [DESC_W-1:0] desc_mem [0:DEPTH-1];

  // --------------------------------------------------------------------------
  // Bookkeeping bitmaps and counters
  //   free_mask[t]   = 1 -> tag 't' is free/allocatable
  //   issued_mask[t] = 1 -> tag 't' is issued to a sink and not yet done
  //   pending_mask[s][t] = 1 -> tag 't' is pending for sink s (not yet issued)
  // --------------------------------------------------------------------------
  logic [DEPTH-1:0] free_mask_q,   free_mask_d;
  logic [DEPTH-1:0] issued_mask_q, issued_mask_d;
  logic [NSINK-1:0][DEPTH-1:0] pending_mask_q, pending_mask_d;

  // Counters
  logic [CNT_W-1:0] free_count_q, free_count_d;
  logic [NSINK-1:0][CNT_W-1:0] pending_count_q, pending_count_d;
  logic [NSINK-1:0][INF_W-1:0] inflight_count_q, inflight_count_d;

  // Sticky errors
  logic err_spurious_done_q, err_double_done_q, err_overflow_q;
  assign err_spurious_done_o = err_spurious_done_q;
  assign err_double_done_o   = err_double_done_q;
  assign err_overflow_o      = err_overflow_q;

  // --------------------------------------------------------------------------
  // Helpers: first-set-bit (FF1) and has_any
  // --------------------------------------------------------------------------
  function automatic [TAG_W-1:0] ff1(input logic [DEPTH-1:0] v);
    int i;
    ff1 = '0;
    for (i = 0; i < DEPTH; i++) begin
      if (v[i]) begin
        ff1 = TAG_W'(i);
        break;
      end
    end
  endfunction

  function automatic logic any1(input logic [DEPTH-1:0] v);
    any1 = (v != '0);
  endfunction

  // --------------------------------------------------------------------------
  // Enqueue acceptance: choose free tag
  // --------------------------------------------------------------------------
  wire                 have_free   = any1(free_mask_q);
  wire [TAG_W-1:0]     alloc_tag   = ff1(free_mask_q);

  // Map class → sink index (saturate to 0 if out-of-range)
  wire [CLS_W-1:0]     sink_sel    = (enq_class_i < NSINK[CLS_W-1:0]) ? enq_class_i[CLS_W-1:0] : '0;

  // enq_ready: require a free tag
  assign enq_ready_o = have_free;

  // Fire marks an enqueue this cycle
  wire enq_fire = enq_valid_i & enq_ready_o;

  // --------------------------------------------------------------------------
  // Issue side: per-sink next pending tag, valid gating on inflight credits
  // --------------------------------------------------------------------------
  logic [NSINK-1:0]          has_pending;
  logic [NSINK-1:0][TAG_W-1:0] next_tag;
  genvar s;
  generate
    for (s = 0; s < NSINK; s++) begin : g_pick
      assign has_pending[s]   = any1(pending_mask_q[s]);
      assign next_tag   [s]   = ff1(pending_mask_q[s]);

      assign issue_valid_o[s] = has_pending[s] && (inflight_count_q[s] < MAX_INFLIGHT[INF_W-1:0]);
      assign issue_tag_o  [s] = next_tag[s];
      assign issue_desc_o [s] = desc_mem[next_tag[s]];
    end
  endgenerate

  // Fire when sink accepts an issued descriptor
  logic [NSINK-1:0] issue_fire;
  generate
    for (s = 0; s < NSINK; s++) begin : g_issue_fire
      assign issue_fire[s] = issue_valid_o[s] & issue_ready_i[s];
    end
  endgenerate

  // Per-sink empty flags
  generate
    for (s = 0; s < NSINK; s++) begin : g_empty
      assign empty_o[s] = ~has_pending[s];
    end
  endgenerate

  // Queue full flag (no free tags)
  assign full_o        = ~have_free;
  assign free_count_o  = free_count_q;
  assign pending_count_o = pending_count_q;
  assign inflight_count_o = inflight_count_q;

  // --------------------------------------------------------------------------
  // Next-state: masks and counters
  // --------------------------------------------------------------------------
  integer i;

  always_comb begin
    // Defaults (hold)
    free_mask_d        = free_mask_q;
    issued_mask_d      = issued_mask_q;
    pending_mask_d     = pending_mask_q;

    free_count_d       = free_count_q;
    pending_count_d    = pending_count_q;
    inflight_count_d   = inflight_count_q;

    // Errors hold
    err_spurious_done_q = err_spurious_done_q;
    err_double_done_q   = err_double_done_q;
    err_overflow_q      = err_overflow_q;

    // ---------------- Enqueue ----------------
    if (enq_fire) begin
      // Write descriptor into storage at alloc_tag
      // (write actually occurs sequentially below, but mask/counters update here)
      free_mask_d[alloc_tag] = 1'b0;
      free_count_d           = free_count_d - {{(CNT_W-1){1'b0}}, 1'b1};

      pending_mask_d[sink_sel][alloc_tag] = 1'b1;
      pending_count_d[sink_sel] = pending_count_d[sink_sel] + {{(CNT_W-1){1'b0}},1'b1};
    end else if (enq_valid_i && !enq_ready_o) begin
      // Producer violated back-pressure; sticky overflow flag
      err_overflow_q = 1'b1;
    end

    // ---------------- Issue accept per sink ----------------
    for (i = 0; i < NSINK; i++) begin
      if (issue_fire[i]) begin
        // Remove from pending, mark issued, bump inflight
        pending_mask_d[i][ next_tag[i] ] = 1'b0;
        pending_count_d[i]               = pending_count_d[i] - {{(CNT_W-1){1'b0}},1'b1};

        issued_mask_d [ next_tag[i] ]    = 1'b1;
        inflight_count_d[i]              = inflight_count_d[i] + {{(INF_W-1){1'b0}},1'b1};
      end
    end

    // ---------------- Done returns per sink ----------------
    for (i = 0; i < NSINK; i++) begin
      if (done_valid_i[i]) begin
        logic [TAG_W-1:0] t = done_tag_i[i];

        // Spurious done (not issued) or double-done (already free) detection
        if (!issued_mask_q[t]) begin
          err_spurious_done_q = 1'b1;
        end else begin
          // Clear issued, free tag, decrement inflight
          issued_mask_d[t] = 1'b0;

          // Guard against freeing a tag that is already free (should not happen)
          if (free_mask_q[t]) begin
            err_double_done_q = 1'b1;
          end else begin
            free_mask_d[t]  = 1'b1;
            free_count_d    = free_count_d + {{(CNT_W-1){1'b0}},1'b1};
          end

          // Inflight decrement (saturate at zero for safety)
          if (inflight_count_d[i] != '0) begin
            inflight_count_d[i] = inflight_count_d[i] - {{(INF_W-1){1'b0}},1'b1};
          end
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Sequential state
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      // All tags free on reset
      free_mask_q   <= {DEPTH{1'b1}};
      free_count_q  <= CNT_W'(DEPTH);

      issued_mask_q <= '0;

      for (i = 0; i < NSINK; i++) begin
        pending_mask_q[i]  <= '0;
        pending_count_q[i] <= '0;
        inflight_count_q[i]<= '0;
      end

      err_spurious_done_q <= 1'b0;
      err_double_done_q   <= 1'b0;
      err_overflow_q      <= 1'b0;
    end else begin
      free_mask_q   <= free_mask_d;
      issued_mask_q <= issued_mask_d;
      for (i = 0; i < NSINK; i++) begin
        pending_mask_q[i]  <= pending_mask_d[i];
        pending_count_q[i] <= pending_count_d[i];
        inflight_count_q[i]<= inflight_count_d[i];
      end

      free_count_q        <= free_count_d;

      // Write descriptor payload on enqueue
      if (enq_fire) begin
        desc_mem[alloc_tag] <= enq_desc_i;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Assertions (simulation only)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // Producer must not ignore back-pressure
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    enq_valid_i |-> enq_ready_o) else
      $warning("desc_queue: enqueue while full; ignoring but flagging err_overflow");

  // Issue handshake: next_tag must actually be pending for that sink
  genvar a;
  generate for (a = 0; a < NSINK; a++) begin : g_assert_issue
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      issue_fire[a] |-> pending_mask_q[a][ next_tag[a] ]) else
        $error("desc_queue: issued tag not pending for sink %0d", a);
  end endgenerate

  // Done must correspond to an issued tag (we flag sticky error otherwise)
  generate for (a = 0; a < NSINK; a++) begin : g_assert_done
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      done_valid_i[a] |-> issued_mask_q[ done_tag_i[a] ]) else
        $warning("desc_queue: spurious done(tag) for sink %0d", a);
  end endgenerate

  // Free_count bookkeeping never underflows/overflows
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (free_count_d <= DEPTH)) else
      $error("desc_queue: free_count exceeded DEPTH");
`endif

endmodule

`default_nettype wire
