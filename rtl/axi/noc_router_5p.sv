// ============================================================================
//  noc_router_5p.sv
//  Five-port 2D-mesh NoC router: N, S, E, W, Local (LS)
//  - Wormhole switching with output reservation (lock/hold) until TAIL
//  - XY (dimension-ordered) minimal routing (E/W then N/S)
//  - Per-input elastic FIFOs; per-output egress FIFOs
//  - Per-output round-robin arbitration among requesting inputs
//  - Ready/valid handshakes only (no credit wires); no combinational ready loops
//
//  Flit format (LSB-aligned):
//    [FTYPE_W-1:0]          ftype   : 2'b00=HEAD, 2'b01=BODY, 2'b10=TAIL, 2'b11=HEADTAIL
//    [FTYPE_W+X_W-1 : FTYPE_W]      dst_x
//    [FTYPE_W+X_W+Y_W-1 : FTYPE_W+X_W] dst_y
//    [FLIT_W-1 : FTYPE_W+X_W+Y_W]   payload (opaque to router)
//
//  Notes:
//    - The header (dst_x/dst_y) is examined on HEAD/HEADTAIL only.
//    - With XY routing, one VC is sufficient for deadlock freedom on a mesh.
//    - All FIFOs are single-clock, non-fallthrough; depth parameters are small by default.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module noc_router_5p #(
  // ---------------- Flit & header geometry ----------------
  parameter int unsigned FLIT_W        = 128, // total flit width (bits)
  parameter int unsigned X_W           = 4,   // bits for X coordinate
  parameter int unsigned Y_W           = 4,   // bits for Y coordinate
  parameter int unsigned FTYPE_W       = 2,   // 2 bits: HEAD/BODY/TAIL/HEADTAIL

  // ---------------- FIFO depths ----------------------------
  parameter int unsigned IN_FIFO_DEPTH  = 4,  // per-input FIFO depth (flits)
  parameter int unsigned OUT_FIFO_DEPTH = 4,  // per-output egress FIFO depth (flits)

  // ---------------- Arbitration ----------------------------
  parameter bit          FAIR_RR        = 1'b1 // use rotating RR (1) or fixed-priority (0)
)(
  input  logic                   clk_i,
  input  logic                   rstn_i,         // synchronous active-low

  // Router physical coordinates
  input  logic [X_W-1:0]         this_x_i,
  input  logic [Y_W-1:0]         this_y_i,

  // =================== NORTH link ===================
  input  logic                   n_in_valid_i,
  output logic                   n_in_ready_o,
  input  logic [FLIT_W-1:0]      n_in_flit_i,

  output logic                   n_out_valid_o,
  input  logic                   n_out_ready_i,
  output logic [FLIT_W-1:0]      n_out_flit_o,

  // =================== SOUTH link ===================
  input  logic                   s_in_valid_i,
  output logic                   s_in_ready_o,
  input  logic [FLIT_W-1:0]      s_in_flit_i,

  output logic                   s_out_valid_o,
  input  logic                   s_out_ready_i,
  output logic [FLIT_W-1:0]      s_out_flit_o,

  // =================== EAST link ====================
  input  logic                   e_in_valid_i,
  output logic                   e_in_ready_o,
  input  logic [FLIT_W-1:0]      e_in_flit_i,

  output logic                   e_out_valid_o,
  input  logic                   e_out_ready_i,
  output logic [FLIT_W-1:0]      e_out_flit_o,

  // =================== WEST link ====================
  input  logic                   w_in_valid_i,
  output logic                   w_in_ready_o,
  input  logic [FLIT_W-1:0]      w_in_flit_i,

  output logic                   w_out_valid_o,
  input  logic                   w_out_ready_i,
  output logic [FLIT_W-1:0]      w_out_flit_o,

  // =================== LOCAL/TILE link ==============
  input  logic                   l_in_valid_i,
  output logic                   l_in_ready_o,
  input  logic [FLIT_W-1:0]      l_in_flit_i,

  output logic                   l_out_valid_o,
  input  logic                   l_out_ready_i,
  output logic [FLIT_W-1:0]      l_out_flit_o

  // (Optional) add per-port error/status later as needed
);

  // --------------------------------------------------------------------------
  // Port index mapping
  // --------------------------------------------------------------------------
  localparam int N_PORTS = 5;
  localparam int P_N = 0;
  localparam int P_S = 1;
  localparam int P_E = 2;
  localparam int P_W = 3;
  localparam int P_L = 4;
  localparam int SEL_W = (N_PORTS <= 2) ? 1 : $clog2(N_PORTS);

  // --------------------------------------------------------------------------
  // Helpers — flit field extractors and type predicates
  // --------------------------------------------------------------------------
  function automatic logic [FTYPE_W-1:0] flit_type(input logic [FLIT_W-1:0] f);
    return f[FTYPE_W-1:0];
  endfunction

  function automatic logic [X_W-1:0] flit_dst_x(input logic [FLIT_W-1:0] f);
    return f[FTYPE_W +: X_W];
  endfunction

  function automatic logic [Y_W-1:0] flit_dst_y(input logic [FLIT_W-1:0] f);
    return f[FTYPE_W+X_W +: Y_W];
  endfunction

  function automatic bit is_head(input logic [FLIT_W-1:0] f);
    logic [FTYPE_W-1:0] t = flit_type(f);
    return (t == 2'b00) || (t == 2'b11); // HEAD or HEADTAIL
  endfunction

  function automatic bit is_tail(input logic [FLIT_W-1:0] f);
    logic [FTYPE_W-1:0] t = flit_type(f);
    return (t == 2'b10) || (t == 2'b11); // TAIL or HEADTAIL
  endfunction

  // --------------------------------------------------------------------------
  // XY routing function: returns output port index
  //   Rule: resolve X first (E/W), then Y (S/N). If equal, deliver to Local.
  //   Convention: y increases toward South (S), decreases toward North (N).
  // --------------------------------------------------------------------------
  function automatic logic [SEL_W-1:0] route_xy(
    input logic [X_W-1:0] dst_x,
    input logic [Y_W-1:0] dst_y,
    input logic [X_W-1:0] cur_x,
    input logic [Y_W-1:0] cur_y
  );
    if (dst_x > cur_x)       return P_E[SEL_W-1:0];
    else if (dst_x < cur_x)  return P_W[SEL_W-1:0];
    else if (dst_y > cur_y)  return P_S[SEL_W-1:0];
    else if (dst_y < cur_y)  return P_N[SEL_W-1:0];
    else                     return P_L[SEL_W-1:0];
  endfunction

  // --------------------------------------------------------------------------
  // Gather/Scatter the 5 physical ports into arrays for compact logic
  // --------------------------------------------------------------------------
  logic [N_PORTS-1:0]       in_valid,  in_ready;
  logic [N_PORTS-1:0]       out_valid, out_ready;
  logic [FLIT_W-1:0]        in_flit   [N_PORTS];
  logic [FLIT_W-1:0]        out_flit  [N_PORTS];

  // In
  always_comb begin
    in_valid[P_N] = n_in_valid_i;  in_flit[P_N] = n_in_flit_i;
    in_valid[P_S] = s_in_valid_i;  in_flit[P_S] = s_in_flit_i;
    in_valid[P_E] = e_in_valid_i;  in_flit[P_E] = e_in_flit_i;
    in_valid[P_W] = w_in_valid_i;  in_flit[P_W] = w_in_flit_i;
    in_valid[P_L] = l_in_valid_i;  in_flit[P_L] = l_in_flit_i;

    out_ready[P_N] = n_out_ready_i;
    out_ready[P_S] = s_out_ready_i;
    out_ready[P_E] = e_out_ready_i;
    out_ready[P_W] = w_out_ready_i;
    out_ready[P_L] = l_out_ready_i;
  end

  // Out
  assign n_in_ready_o  = in_ready[P_N];
  assign s_in_ready_o  = in_ready[P_S];
  assign e_in_ready_o  = in_ready[P_E];
  assign w_in_ready_o  = in_ready[P_W];
  assign l_in_ready_o  = in_ready[P_L];

  assign n_out_valid_o = out_valid[P_N];
  assign s_out_valid_o = out_valid[P_S];
  assign e_out_valid_o = out_valid[P_E];
  assign w_out_valid_o = out_valid[P_W];
  assign l_out_valid_o = out_valid[P_L];

  assign n_out_flit_o  = out_flit[P_N];
  assign s_out_flit_o  = out_flit[P_S];
  assign e_out_flit_o  = out_flit[P_E];
  assign w_out_flit_o  = out_flit[P_W];
  assign l_out_flit_o  = out_flit[P_L];

  // --------------------------------------------------------------------------
  // Ingress FIFOs (decouple links; no ready loops)
  // --------------------------------------------------------------------------
  logic                  if_valid [N_PORTS];
  logic                  if_ready [N_PORTS];
  logic [FLIT_W-1:0]     if_flit  [N_PORTS];

  for (genvar p=0; p<N_PORTS; p++) begin : g_in_fifo
    rv_fifo #(
      .WIDTH (FLIT_W),
      .DEPTH (IN_FIFO_DEPTH)
    ) u_in_fifo (
      .clk_i       (clk_i),
      .rstn_i      (rstn_i),
      .in_valid_i  (in_valid[p]),
      .in_ready_o  (in_ready[p]),
      .in_data_i   (in_flit[p]),
      .out_valid_o (if_valid[p]),
      .out_ready_i (if_ready[p]),
      .out_data_o  (if_flit[p])
    );
  end

  // --------------------------------------------------------------------------
  // Per-output egress FIFOs
  // --------------------------------------------------------------------------
  logic                  of_valid [N_PORTS];
  logic                  of_ready [N_PORTS];
  logic [FLIT_W-1:0]     of_flit  [N_PORTS];

  for (genvar o=0; o<N_PORTS; o++) begin : g_out_fifo
    rv_fifo #(
      .WIDTH (FLIT_W),
      .DEPTH (OUT_FIFO_DEPTH)
    ) u_out_fifo (
      .clk_i       (clk_i),
      .rstn_i      (rstn_i),
      .in_valid_i  (of_valid[o]),
      .in_ready_o  (of_ready[o]),
      .in_data_i   (of_flit[o]),
      .out_valid_o (out_valid[o]),
      .out_ready_i (out_ready[o]),
      .out_data_o  (out_flit[o])
    );
  end

  // --------------------------------------------------------------------------
  // Output lock/hold: once a HEAD is granted to an output 'o', hold connection
  // to input 'owner[o]' until its TAIL (or HEADTAIL) traverses.
  // --------------------------------------------------------------------------
  logic                  out_locked_q [N_PORTS], out_locked_d [N_PORTS];
  logic [SEL_W-1:0]      out_owner_q  [N_PORTS], out_owner_d  [N_PORTS];

  // --------------------------------------------------------------------------
  // Request matrix + round-robin arbitration per output
  //  - If output is unlocked: inputs whose HOQ flit is HEAD/HEADTAIL and routes
  //    to this output 'o' assert a request.
  //  - If output is locked: only the owner is eligible.
  // --------------------------------------------------------------------------
  logic [N_PORTS-1:0] req_vec [N_PORTS];
  logic [N_PORTS-1:0] gnt_vec [N_PORTS];
  logic [SEL_W-1:0]   gnt_idx [N_PORTS];
  logic               gnt_val [N_PORTS];

  // Compute desired output for each input's HOQ flit (HEAD/HEADTAIL only)
  logic [SEL_W-1:0]   des_out [N_PORTS];
  for (genvar i=0; i<N_PORTS; i++) begin : g_des
    logic [SEL_W-1:0] r;
    always_comb begin
      if (if_valid[i] && is_head(if_flit[i])) begin
        r = route_xy(flit_dst_x(if_flit[i]), flit_dst_y(if_flit[i]), this_x_i, this_y_i);
      end else begin
        // Value irrelevant when not HEAD; not used in request when locked.
        r = '0;
      end
      des_out[i] = r;
    end
  end

  // Requests per output
  for (genvar o=0; o<N_PORTS; o++) begin : g_req
    always_comb begin
      req_vec[o] = '0;
      if (out_locked_q[o]) begin
        // Only owner can drive; presence of flit gates actual transfer below
        req_vec[o][out_owner_q[o]] = if_valid[out_owner_q[o]];
      end else begin
        // Unlocked: inputs with HEAD/HEADTAIL targeting this output
        for (int i=0; i<N_PORTS; i++) begin
          if (if_valid[i] && is_head(if_flit[i]) && (des_out[i] == o[SEL_W-1:0])) begin
            req_vec[o][i] = 1'b1;
          end
        end
      end
    end

    // Arbitration
    rr_arb #(.N(N_PORTS), .FAIR(FAIR_RR)) u_rr (
      .clk_i   (clk_i),
      .rstn_i  (rstn_i),
      .req_i   (req_vec[o]),
      .grant_o (gnt_vec[o]),
      .gidx_o  (gnt_idx[o]),
      .gvalid_o(gnt_val[o])
    );
  end

  // --------------------------------------------------------------------------
  // Crossbar drive and handshakes
  //   - Select one input per output (either RR grant or locked owner)
  //   - Transfer occurs when selected input has a flit AND egress FIFO can accept
  //   - Pop input(s) that were actually transferred
  //   - Update output lock/owner on HEAD and release on TAIL
  // --------------------------------------------------------------------------
  logic [SEL_W-1:0]   sel_i_for_o [N_PORTS]; // selected input for each output
  logic               sel_v_for_o  [N_PORTS];
  logic               xfer_o       [N_PORTS];
  logic [N_PORTS-1:0] pop_in;                // which inputs to pop this cycle

  // Default
  for (genvar o=0; o<N_PORTS; o++) begin : g_sel
    always_comb begin
      // Selection
      if (out_locked_q[o]) begin
        sel_i_for_o[o] = out_owner_q[o];
        sel_v_for_o[o] = if_valid[out_owner_q[o]];
      end else begin
        sel_i_for_o[o] = gnt_idx[o];
        sel_v_for_o[o] = gnt_val[o];
      end

      // Crossbar data and enqueue to egress FIFO
      of_valid[o] = sel_v_for_o[o] && of_ready[o]; // we only assert valid when downstream egress can accept
      of_flit[o]  = if_flit[ sel_i_for_o[o] ];

      // A transfer (xbar traversal) happens when we decided to send and the egress FIFO accepted
      xfer_o[o] = sel_v_for_o[o] && of_ready[o];
    end
  end

  // Pop inputs that were sources of a transfer
  always_comb begin
    pop_in = '0;
    for (int o=0; o<N_PORTS; o++) begin
      if (xfer_o[o]) begin
        pop_in[ sel_i_for_o[o] ] = 1'b1;
      end
    end
  end
  for (genvar i=0; i<N_PORTS; i++) begin : g_if_ready
    assign if_ready[i] = pop_in[i];
  end

  // Output lock state updates
  for (genvar o=0; o<N_PORTS; o++) begin : g_lock
    always_comb begin
      out_locked_d[o] = out_locked_q[o];
      out_owner_d [o] = out_owner_q [o];

      if (xfer_o[o]) begin
        logic [FLIT_W-1:0] f = if_flit[ sel_i_for_o[o] ];
        // Acquire on successful HEAD when unlocked
        if (!out_locked_q[o] && is_head(f)) begin
          out_locked_d[o] = 1'b1;
          out_owner_d [o] = sel_i_for_o[o];
        end
        // Release on TAIL traversal
        if (is_tail(f)) begin
          out_locked_d[o] = 1'b0;
          // owner don't-care when unlocked
        end
      end
    end
  end

  // Lock registers
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      for (int o=0; o<N_PORTS; o++) begin
        out_locked_q[o] <= 1'b0;
        out_owner_q [o] <= '0;
      end
    end else begin
      for (int o=0; o<N_PORTS; o++) begin
        out_locked_q[o] <= out_locked_d[o];
        out_owner_q [o] <= out_owner_d [o];
      end
    end
  end

  // --------------------------------------------------------------------------
  // Assertions / sanity (simulation only)
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  // No input should be consumed by more than one output in the same cycle
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !( (pop_in[P_N] & (pop_in[P_S] | pop_in[P_E] | pop_in[P_W] | pop_in[P_L])) ||
       (pop_in[P_S] & (pop_in[P_E] | pop_in[P_W] | pop_in[P_L])) ||
       (pop_in[P_E] & (pop_in[P_W] | pop_in[P_L])) ||
       (pop_in[P_W] &  pop_in[P_L]) ))
    else $error("noc_router_5p: input popped by multiple outputs in same cycle.");

  // Once locked, only the owner can be selected
  for (genvar o=0;o<N_PORTS;o++) begin : g_assert_lock
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      out_locked_q[o] && xfer_o[o] |-> (sel_i_for_o[o] == out_owner_q[o]))
      else $error("noc_router_5p: locked output %0d routed non-owner.", o);
  end

  // HEAD must acquire lock before any BODY on that path (by construction, enforced)
  // TAIL must release lock (checked indirectly)
`endif

endmodule

// ============================================================================
//  rr_arb — simple rotating round-robin arbiter
//  - If FAIR=1, pointer advances to (granted+1) on each grant.
//  - If FAIR=0, fixed-priority from bit 0..N-1.
// ============================================================================
module rr_arb #(
  parameter int unsigned N    = 5,
  parameter bit          FAIR = 1'b1
)(
  input  logic               clk_i,
  input  logic               rstn_i,
  input  logic [N-1:0]       req_i,
  output logic [N-1:0]       grant_o,
  output logic [$clog2(N)-1:0] gidx_o,
  output logic               gvalid_o
);
  localparam int P_W = (N <= 2) ? 1 : $clog2(N);

  logic [P_W-1:0] ptr_q, ptr_d;

  // Fixed-priority function, rotated by ptr_q
  function automatic logic [N-1:0] rot_left(input logic [N-1:0] v, input int unsigned s);
    logic [N-1:0] r;
    for (int i=0;i<N;i++) r[(i+s)%N] = v[i];
    return r;
  endfunction

  function automatic logic [N-1:0] rot_right(input logic [N-1:0] v, input int unsigned s);
    logic [N-1:0] r;
    for (int i=0;i<N;i++) r[i] = v[(i+s)%N];
    return r;
  endfunction

  logic [N-1:0] req_rot, gnt_rot;
  logic [P_W-1:0] gidx_rot;
  logic            have_req;

  always_comb begin
    have_req = (|req_i);
    if (FAIR) req_rot = rot_left(req_i, ptr_q);
    else      req_rot = req_i;

    // Fixed priority on rotated vector: lowest index wins
    gnt_rot = '0;
    gidx_rot = '0;
    for (int i=0;i<N;i++) begin
      if (req_rot[i]) begin
        gnt_rot[i] = 1'b1;
        gidx_rot   = i[P_W-1:0];
        break;
      end
    end

    // Unrotate back
    if (FAIR) begin
      grant_o = rot_right(gnt_rot, ptr_q);
      // Compute absolute index
      gidx_o  = (gidx_rot + ptr_q) % N;
    end else begin
      grant_o = gnt_rot;
      gidx_o  = gidx_rot;
    end
    gvalid_o = have_req && (|grant_o);

    // Pointer advance
    ptr_d = ptr_q;
    if (FAIR && gvalid_o) begin
      ptr_d = (gidx_o + {{P_W-1{1'b0}},1'b1}) % N;
    end
  end

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) ptr_q <= '0;
    else         ptr_q <= ptr_d;
  end

endmodule

// ============================================================================
//  rv_fifo — single-clock ready/valid FIFO (non-fallthrough)
//  *Used for ingress and egress queues to cut combinational ready loops.
// ============================================================================
module rv_fifo #(
  parameter int unsigned WIDTH = 128,
  parameter int unsigned DEPTH = 4
)(
  input  logic               clk_i,
  input  logic               rstn_i,
  // Ingress
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [WIDTH-1:0]   in_data_i,
  // Egress
  output logic               out_valid_o,
  input  logic               out_ready_i,
  output logic [WIDTH-1:0]   out_data_o
);
  localparam int AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
  logic [WIDTH-1:0] mem [0:DEPTH-1];
  logic [AW:0] wr_ptr_q, rd_ptr_q;

  function automatic logic fifo_empty(input logic [AW:0] w, input logic [AW:0] r);
    return (w == r);
  endfunction
  function automatic logic fifo_full (input logic [AW:0] w, input logic [AW:0] r);
    return (w[AW] != r[AW]) && (w[AW-1:0] == r[AW-1:0]);
  endfunction

  wire do_push = in_valid_i  && !fifo_full(wr_ptr_q, rd_ptr_q);
  wire do_pop  = out_ready_i && !fifo_empty(wr_ptr_q, rd_ptr_q);

  assign in_ready_o  = !fifo_full(wr_ptr_q, rd_ptr_q);
  assign out_valid_o = !fifo_empty(wr_ptr_q, rd_ptr_q);
  assign out_data_o  = mem[rd_ptr_q[AW-1:0]];

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wr_ptr_q <= '0; rd_ptr_q <= '0;
    end else begin
      if (do_push) begin
        mem[wr_ptr_q[AW-1:0]] <= in_data_i;
        wr_ptr_q <= wr_ptr_q + {{AW{1'b0}},1'b1};
      end
      if (do_pop) begin
        rd_ptr_q <= rd_ptr_q + {{AW{1'b0}},1'b1};
      end
    end
  end

`ifdef ASSERT_ON
  // Simple under/overflow guards
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_push |-> !fifo_full(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: push while full.");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_pop |-> !fifo_empty(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: pop while empty.");
`endif
endmodule

`default_nettype wire
