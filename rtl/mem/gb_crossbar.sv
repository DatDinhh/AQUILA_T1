// ============================================================================
//  Global Buffer Crossbar (Masters x Banks)
//  File: gb_crossbar.sv
//  Description:
//    - Routes NM masters to NBANK SRAM banks (each bank exposes 1-beat R/W ports).
//    - Beat-striped banking by default (bank select at beat LSB .. +BANK_SEL_BITS-1).
//    - Independent per-bank arbitration for writes and reads (round-robin).
//    - Per-bank read tag FIFOs track requestor IDs; return path includes
//      per-master arbitration (one bank may respond to a master per cycle).
//    - Clean ready/valid flow control, no comb loops.
//
//  Master-side interface (per master m):
//    Write request:
//      wr_req_valid_i[m], wr_req_ready_o[m],
//      wr_addr_i[m][ADDR_W-1:0], wr_data_i[m][DATA_W-1:0], wr_strb_i[m][DATA_W/8-1:0]
//    Read request:
//      rd_req_valid_i[m], rd_req_ready_o[m], rd_addr_i[m][ADDR_W-1:0]
//    Read response:
//      rd_rsp_valid_o[m], rd_rsp_ready_i[m],
//      rd_rsp_data_o[m][DATA_W-1:0], rd_rsp_sbe_o[m], rd_rsp_dbe_o[m]
//
//  Bank-side interface (per bank b), to sram_bank_wrapper:
//    Write:
//      b_wr_cmd_valid_o[b], b_wr_cmd_ready_i[b], b_wr_addr_o[b],
//      b_wr_data_valid_o[b], b_wr_data_ready_i[b], b_wr_data_o[b], b_wr_strb_o[b]
//    Read:
//      b_rd_cmd_valid_o[b], b_rd_cmd_ready_i[b], b_rd_addr_o[b],
//      b_rd_data_valid_i[b], b_rd_data_ready_o[b], b_rd_data_i[b],
//      b_rd_sbe_i[b], b_rd_dbe_i[b]
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module gb_crossbar #(
  // ---------- Topology ----------
  parameter int unsigned NM          = 3,          // number of masters
  parameter int unsigned NBANK       = 8,          // number of banks (power-of-two)

  // ---------- Geometry ----------
  parameter int unsigned DATA_W      = 512,        // beat width (bits)
  parameter int unsigned ADDR_W      = 32,         // byte address width

  // Bank select field: addr[BANK_SEL_LSB +: BANK_SEL_BITS]
  parameter int unsigned BANK_SEL_LSB  = (DATA_W <= 8) ? 0 : $clog2(DATA_W/8),
  parameter int unsigned BANK_SEL_BITS = (NBANK <= 1) ? 1 : $clog2(NBANK),

  // ---------- Arbitration / Queues ----------
  parameter string ARB_WR        = "RR",    // "RR" (round-robin) | "W_FIRST" (not used here)
  parameter string ARB_RD        = "RR",
  parameter int unsigned RD_TAG_DEPTH = 32, // per-bank read tag FIFO depth (>= bank RD queue)

  // ---------- Implementation knobs ----------
  parameter bit PIPE_BANK_ADDR   = 1'b0     // optional 1-cycle pipe on bank cmd addr (timing)
)(
  input  logic                                clk_i,
  input  logic                                rstn_i,

  // ---------------- Masters (arrays) ----------------
  // Write
  input  logic        [NM-1:0]                wr_req_valid_i,
  output logic        [NM-1:0]                wr_req_ready_o,
  input  logic        [NM-1:0][ADDR_W-1:0]    wr_addr_i,
  input  logic        [NM-1:0][DATA_W-1:0]    wr_data_i,
  input  logic        [NM-1:0][DATA_W/8-1:0]  wr_strb_i,

  // Read requests
  input  logic        [NM-1:0]                rd_req_valid_i,
  output logic        [NM-1:0]                rd_req_ready_o,
  input  logic        [NM-1:0][ADDR_W-1:0]    rd_addr_i,

  // Read responses
  output logic        [NM-1:0]                rd_rsp_valid_o,
  input  logic        [NM-1:0]                rd_rsp_ready_i,
  output logic        [NM-1:0][DATA_W-1:0]    rd_rsp_data_o,
  output logic        [NM-1:0]                rd_rsp_sbe_o,
  output logic        [NM-1:0]                rd_rsp_dbe_o,

  // ---------------- Banks (arrays) ----------------
  // Write toward banks
  output logic        [NBANK-1:0]             b_wr_cmd_valid_o,
  input  logic        [NBANK-1:0]             b_wr_cmd_ready_i,
  output logic        [NBANK-1:0][ADDR_W-1:0] b_wr_addr_o,
  output logic        [NBANK-1:0]             b_wr_data_valid_o,
  input  logic        [NBANK-1:0]             b_wr_data_ready_i,
  output logic        [NBANK-1:0][DATA_W-1:0] b_wr_data_o,
  output logic        [NBANK-1:0][DATA_W/8-1:0] b_wr_strb_o,

  // Read toward banks
  output logic        [NBANK-1:0]             b_rd_cmd_valid_o,
  input  logic        [NBANK-1:0]             b_rd_cmd_ready_i,
  output logic        [NBANK-1:0][ADDR_W-1:0] b_rd_addr_o,

  // Read return from banks
  input  logic        [NBANK-1:0]             b_rd_data_valid_i,
  output logic        [NBANK-1:0]             b_rd_data_ready_o,
  input  logic        [NBANK-1:0][DATA_W-1:0] b_rd_data_i,
  input  logic        [NBANK-1:0]             b_rd_sbe_i,
  input  logic        [NBANK-1:0]             b_rd_dbe_i
);

  // ===========================================================================
  // Derived constants & parameter checks
  // ===========================================================================
  localparam int unsigned BEAT_BYTES = DATA_W/8;
  localparam int unsigned BEAT_LSB   = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);
  localparam int unsigned MID_W      = (NM    <= 1) ? 1 : $clog2(NM);
  localparam int unsigned BID_W      = (NBANK <= 1) ? 1 : $clog2(NBANK);

  // Static checks
  function automatic bit is_pow2(input int unsigned x);
    return (x & (x-1)) == 0;
  endfunction
  initial begin
    if (DATA_W % 8 != 0)  $error("gb_crossbar: DATA_W must be multiple of 8.");
    if (!is_pow2(NBANK))  $error("gb_crossbar: NBANK must be power-of-two.");
    if (BANK_SEL_LSB < BEAT_LSB)
      $warning("gb_crossbar: BANK_SEL_LSB < BEAT_LSB; sub-beat striping is unusual.");
    if ((BANK_SEL_LSB + BANK_SEL_BITS) > ADDR_W)
      $error("gb_crossbar: bank select field exceeds ADDR_W.");
  end

  // ===========================================================================
  // Address decode helpers
  // ===========================================================================
  // Bank select from a byte address
  function automatic logic [BID_W-1:0] bank_sel(input logic [ADDR_W-1:0] addr);
    bank_sel = addr[BANK_SEL_LSB +: BANK_SEL_BITS];
  endfunction

  // Remove bank-select bits to form bank-local byte address
  function automatic logic [ADDR_W-1:0] bank_local_addr(input logic [ADDR_W-1:0] addr);
    logic [ADDR_W-1:0] upper, lower;
    if (BANK_SEL_BITS == 0) begin
      bank_local_addr = addr;
    end else begin
      upper = addr[ADDR_W-1 : BANK_SEL_LSB + BANK_SEL_BITS];
      lower = addr[BANK_SEL_LSB-1 : 0];
      bank_local_addr = { upper, lower };
    end
  endfunction

  // ===========================================================================
  // Per-bank write & read arbitration (round-robin)
  //   - Grant one master per bank per channel.
  //   - Bank wrapper will queue internally (it has own small FIFOs).
  // ===========================================================================
  // Requests targeting each bank
  logic [NBANK-1:0][NM-1:0] wr_req_to_bank, rd_req_to_bank;

  genvar m, b;
  generate
    for (m = 0; m < NM; m++) begin : g_decode_req
      for (b = 0; b < NBANK; b++) begin : g_bank_bits
        wire [BID_W-1:0] bsel_wr = bank_sel(wr_addr_i[m]);
        wire [BID_W-1:0] bsel_rd = bank_sel(rd_addr_i[m]);
        assign wr_req_to_bank[b][m] = wr_req_valid_i[m] && (bsel_wr == BID_W'(b));
        assign rd_req_to_bank[b][m] = rd_req_valid_i[m] && (bsel_rd == BID_W'(b));
      end
    end
  endgenerate

  // Round-robin pointers
  logic [NBANK-1:0][MID_W-1:0] rr_wr_ptr_q, rr_rd_ptr_q;

  // Selected grants per bank (one-hot + index)
  logic [NBANK-1:0][NM-1:0] wr_gnt_oh, rd_gnt_oh;
  logic [NBANK-1:0][MID_W-1:0] wr_gnt_idx, rd_gnt_idx;
  logic [NBANK-1:0] wr_has_req, rd_has_req;

  // Find next grant helper
  function automatic void pick_rr #(
    int N
  )(
    input  logic [N-1:0] req,
    input  int unsigned  last,
    output logic [N-1:0] g_oh,
    output int unsigned  g_idx,
    output logic         g_val
  );
    g_oh  = '0; g_idx = last; g_val = (req != '0);
    if (!g_val) return;
    // Search starting from last+1 modulo N
    int unsigned i;
    for (i = 1; i <= N; i++) begin
      int unsigned k = (last + i) % N;
      if (req[k]) begin
        g_idx = k;
        g_oh  = '0; g_oh[k] = 1'b1;
        return;
      end
    end
  endfunction

  // Combinational grant selection
  generate
    for (b = 0; b < NBANK; b++) begin : g_arb
      int unsigned gidx_wr, gidx_rd;
      always_comb begin
        pick_rr#(NM)(wr_req_to_bank[b], rr_wr_ptr_q[b], wr_gnt_oh[b], gidx_wr, wr_has_req[b]);
        wr_gnt_idx[b] = MID_W'(gidx_wr);

        pick_rr#(NM)(rd_req_to_bank[b], rr_rd_ptr_q[b], rd_gnt_oh[b], gidx_rd, rd_has_req[b]);
        rd_gnt_idx[b] = MID_W'(gidx_rd);
      end
    end
  endgenerate

  // ===========================================================================
  // Drive bank command channels & master readies
  // ===========================================================================
  // Default outputs
  always_comb begin
    // Masters
    wr_req_ready_o  = '0;
    rd_req_ready_o  = '0;

    // Banks
    b_wr_cmd_valid_o  = '0;
    b_wr_addr_o       = '{default:'0};
    b_wr_data_valid_o = '0;
    b_wr_data_o       = '{default:'0};
    b_wr_strb_o       = '{default:'0};

    b_rd_cmd_valid_o  = '0;
    b_rd_addr_o       = '{default:'0};
  end

  // Per-bank muxing of selected master's payloads
  generate
    for (b = 0; b < NBANK; b++) begin : g_bank_mux

      // -------- WRITE channel --------
      // Bank sees valid if there is a granted requester
      wire wr_sel_v = wr_has_req[b];

      // Selected master's fields (default zero if no request)
      logic [ADDR_W-1:0] wr_addr_sel;
      logic [DATA_W-1:0] wr_data_sel;
      logic [DATA_W/8-1:0] wr_strb_sel;

      always_comb begin
        wr_addr_sel = '0; wr_data_sel = '0; wr_strb_sel = '0;
        for (int im=0; im<NM; im++) begin
          if (wr_gnt_oh[b][im]) begin
            wr_addr_sel = bank_local_addr(wr_addr_i[im]);
            wr_data_sel = wr_data_i[im];
            wr_strb_sel = wr_strb_i[im];
          end
        end
      end

      // Bank write valids
      assign b_wr_cmd_valid_o [b] = wr_sel_v;
      assign b_wr_data_valid_o[b] = wr_sel_v;
      assign b_wr_addr_o      [b] = wr_addr_sel;
      assign b_wr_data_o      [b] = wr_data_sel;
      assign b_wr_strb_o      [b] = wr_strb_sel;

      // Master write ready only for the selected master and only when BOTH bank
      // cmd and data channels are ready (single-beat atomic acceptance)
      for (m = 0; m < NM; m++) begin : g_wr_ready
        assign wr_req_ready_o[m] = wr_req_ready_o[m] |
                                   ( wr_gnt_oh[b][m] & b_wr_cmd_ready_i[b] & b_wr_data_ready_i[b] );
      end

      // Advance RR pointer when the bank actually accepts (cmd+data)
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) rr_wr_ptr_q[b] <= '0;
        else if (wr_sel_v && b_wr_cmd_ready_i[b] && b_wr_data_ready_i[b])
          rr_wr_ptr_q[b] <= wr_gnt_idx[b];
      end

      // -------- READ channel --------
      // Selected master's read address
      wire rd_sel_v = rd_has_req[b];
      logic [ADDR_W-1:0] rd_addr_sel;
      always_comb begin
        rd_addr_sel = '0;
        for (int im=0; im<NM; im++) begin
          if (rd_gnt_oh[b][im]) rd_addr_sel = bank_local_addr(rd_addr_i[im]);
        end
      end

      assign b_rd_cmd_valid_o[b] = rd_sel_v;
      assign b_rd_addr_o     [b] = rd_addr_sel;

      // Master read ready only for selected master when bank is ready
      for (m = 0; m < NM; m++) begin : g_rd_ready
        assign rd_req_ready_o[m] = rd_req_ready_o[m] |
                                   ( rd_gnt_oh[b][m] & b_rd_cmd_ready_i[b] );
      end

      // RR pointer updates + Read tag FIFO push done below (after we know accept)
    end
  endgenerate

  // ===========================================================================
  // Per-bank Read Tag FIFO (records which master owns each read response)
  //   - Push on read command accept at that bank.
  //   - Pop when a read data beat is consumed from that bank by its owner master.
  // ===========================================================================
  localparam int unsigned RT_AW = (RD_TAG_DEPTH <= 2) ? 1 : $clog2(RD_TAG_DEPTH);
  logic [NBANK-1:0][RT_AW:0]   rt_wptr_q, rt_rptr_q;
  logic [NBANK-1:0][MID_W-1:0] rt_mem   [0:RD_TAG_DEPTH-1];

  function automatic logic rt_empty(input logic [RT_AW:0] wptr, input logic [RT_AW:0] rptr);
    return (wptr == rptr);
  endfunction
  function automatic logic rt_full(input logic [RT_AW:0] wptr, input logic [RT_AW:0] rptr);
    return (wptr[RT_AW]     != rptr[RT_AW]) &&
           (wptr[RT_AW-1:0] == rptr[RT_AW-1:0]);
  endfunction

  // Tag FIFO push when a read command is accepted by bank b
  generate
    for (b = 0; b < NBANK; b++) begin : g_rt_fifo
      wire rd_cmd_acc = b_rd_cmd_valid_o[b] & b_rd_cmd_ready_i[b];
      // Who was granted?
      logic [MID_W-1:0] rd_mid_sel;
      always_comb begin
        rd_mid_sel = '0;
        for (int im=0; im<NM; im++) if (rd_gnt_oh[b][im]) rd_mid_sel = MID_W'(im);
      end

      // FIFO write
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          rt_wptr_q[b] <= '0;
          rt_rptr_q[b] <= '0;
        end else begin
          if (rd_cmd_acc && !rt_full(rt_wptr_q[b], rt_rptr_q[b])) begin
            rt_mem[rt_wptr_q[b][RT_AW-1:0]][b] <= rd_mid_sel;
            rt_wptr_q[b] <= rt_wptr_q[b] + 1'b1;
          end
          // Pop handled in return-path arb when bank beat is consumed
        end
      end

      // Update RR pointer on accept
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) rr_rd_ptr_q[b] <= '0;
        else if (rd_cmd_acc) rr_rd_ptr_q[b] <= rd_gnt_idx[b];
      end
    end
  endgenerate

  // Helper to read FIFO head master id for bank b
  function automatic logic [MID_W-1:0] rt_head_mid(input logic [RT_AW:0] rptr, input int bank);
    rt_head_mid = rt_mem[rptr[RT_AW-1:0]][bank];
  endfunction

  // ===========================================================================
  // Read return path arbitration (per master)
  //   - Multiple banks may have data for the same master -> choose one (RR).
  //   - Backpressure other banks by deasserting b_rd_data_ready_o[*].
  // ===========================================================================
  // Which bank wants to respond to master m this cycle?
  logic [NM-1:0][NBANK-1:0] resp_req_matrix;    // [m][b] = bank b has data for m
  logic [NBANK-1:0]         bank_can_respond;   // bank has valid & tag not empty

  // Build resp matrix and default bank backpressure = 0
  always_comb begin
    for (b = 0; b < NBANK; b++) begin
      bank_can_respond[b] = b_rd_data_valid_i[b] && !rt_empty(rt_wptr_q[b], rt_rptr_q[b]);
    end
    for (m = 0; m < NM; m++) begin
      for (b = 0; b < NBANK; b++) begin
        resp_req_matrix[m][b] = 1'b0;
      end
    end
    // set 1 where bank's fifo head == master m
    for (b = 0; b < NBANK; b++) begin
      if (bank_can_respond[b]) begin
        logic [MID_W-1:0] mid = rt_head_mid(rt_rptr_q[b], b);
        resp_req_matrix[mid][b] = 1'b1;
      end
    end
  end

  // Per-master RR pointer & selections
  logic [NM-1:0][BID_W-1:0] rr_rsp_ptr_q;
  logic [NM-1:0][NBANK-1:0] rsp_sel_oh;
  logic [NM-1:0][BID_W-1:0] rsp_sel_idx;
  logic [NM-1:0]            rsp_sel_v;

  // Per-master grant
  generate
    for (m = 0; m < NM; m++) begin : g_rsp_arb
      int unsigned sel_idx;
      always_comb begin
        pick_rr#(NBANK)(resp_req_matrix[m], rr_rsp_ptr_q[m], rsp_sel_oh[m], sel_idx, rsp_sel_v[m]);
        rsp_sel_idx[m] = BID_W'(sel_idx);
      end

      // Update RR pointer when response beat is accepted by this master
      // Accept: selected bank valid & routed to this master & master ready
      wire sel_bank_valid = |(rsp_sel_oh[m] & bank_can_respond);
      wire rsp_acc = sel_bank_valid && rd_rsp_ready_i[m];

      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) rr_rsp_ptr_q[m] <= '0;
        else if (rsp_acc) rr_rsp_ptr_q[m] <= rsp_sel_idx[m];
      end
    end
  endgenerate

  // Drive b_rd_data_ready_o per bank, and route data to the chosen master
  // Defaults
  always_comb begin
    b_rd_data_ready_o = '0;
    rd_rsp_valid_o    = '0;
    rd_rsp_data_o     = '{default:'0};
    rd_rsp_sbe_o      = '0;
    rd_rsp_dbe_o      = '0;

    // For each master, if a bank is selected, route that bank's beat
    for (int im=0; im<NM; im++) begin
      if (rsp_sel_v[im]) begin
        // Find the selected bank
        for (int ib=0; ib<NBANK; ib++) begin
          if (rsp_sel_oh[im][ib]) begin
            rd_rsp_valid_o[im] = bank_can_respond[ib];
            rd_rsp_data_o [im] = b_rd_data_i[ib];
            rd_rsp_sbe_o  [im] = b_rd_sbe_i[ib];
            rd_rsp_dbe_o  [im] = b_rd_dbe_i[ib];

            // Backpressure the bank with the master's ready
            b_rd_data_ready_o[ib] = rd_rsp_ready_i[im];
          end
        end
      end
    end
  end

  // Pop read tag fifo when bank beat is consumed
  generate
    for (b = 0; b < NBANK; b++) begin : g_rt_pop
      // A bank beat is consumed if any master selected this bank and asserted ready
      logic bank_selected_and_accepted;
      always_comb begin
        bank_selected_and_accepted = 1'b0;
        for (int im=0; im<NM; im++) begin
          if (rsp_sel_oh[im][b] && bank_can_respond[b] && rd_rsp_ready_i[im])
            bank_selected_and_accepted = 1'b1;
        end
      end
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          // already cleared at FIFO init
        end else if (bank_selected_and_accepted) begin
          rt_rptr_q[b] <= rt_rptr_q[b] + 1'b1;
        end
      end
    end
  endgenerate

  // ===========================================================================
  // Optional: simple pipelining of bank command address (for timing)
  //   (Kept as parameter hook; in this revision we drive directly.)
  // ===========================================================================

  // ===========================================================================
  // Assertions (simulation only)
  // ===========================================================================
`ifdef ASSERT_ON
  // Ingress: bank tag FIFO should never overflow if upstream obeys ready
  generate
    for (b = 0; b < NBANK; b++) begin : g_assert_b
      // No data without a tag
      assert property (@(posedge clk_i) disable iff(!rstn_i)
        b_rd_data_valid_i[b] |-> !rt_empty(rt_wptr_q[b], rt_rptr_q[b]))
        else $error("gb_crossbar: bank %0d produced RDATA with empty tag FIFO", b);

      // No read cmd accepted when tag FIFO full
      assert property (@(posedge clk_i) disable iff(!rstn_i)
        !(b_rd_cmd_valid_o[b] && b_rd_cmd_ready_i[b] && rt_full(rt_wptr_q[b], rt_rptr_q[b])))
        else $error("gb_crossbar: bank %0d tag FIFO overflow", b);
    end
  endgenerate

  // Egress: do not drive two banks into the same master in one cycle (by design)
  generate
    for (m = 0; m < NM; m++) begin : g_assert_m
      assert property (@(posedge clk_i) disable iff(!rstn_i)
        $onehot0(rsp_sel_oh[m]))
        else $error("gb_crossbar: multiple banks selected to master %0d", m);
    end
  endgenerate
`endif

endmodule

`default_nettype wire
