// ============================================================================
//  Aquila Packet Unpacker / Router
//  File: pkt_unpack.sv
//  Description:
//    - Parses fixed 16B header (Aquila Packet v1) from a streaming byte input.
//    - Routes the payload to one of N destinations based on dest_id.
//    - Exposes the header (packed) on a sideband for that destination.
//    - Cut-through forwarding with back-pressure; small input byte FIFO.
//    - Robust error/status and SVAs.
//
//  Header v1 (little-endian):
//    0x00: magic[15:0]      = 16'hA151
//    0x02: version[7:0]     = 8'h01
//    0x03: hdr_lenB[7:0]    = 8'd16
//    0x04: dest_id[7:0]
//    0x05: opcode[7:0]
//    0x06: flags[15:0]
//    0x08: payload_lenB[31:0]
//    0x0C: crc32[31:0]      (optional; not checked here)
//
//  Notes:
//    - in_strb_i must be full on non-final beats; final beat must be LSB-contiguous.
//    - OUT_W may differ from IN_W. Bytes are repacked to OUT_W on egress.
//    - Unknown dest_id -> packet dropped; counters + error flag incremented.
//    - Bad magic or hdr_len -> packet dropped; counters + error flag.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pkt_unpack #(
  // ---------------- Bus widths ----------------
  parameter int unsigned IN_W    = 128,  // input bus width  (bits), multiple of 8
  parameter int unsigned OUT_W   = 128,  // egress bus width (bits), multiple of 8

  // ---------------- Topology ----------------
  parameter int unsigned NDEST   = 4,    // number of destinations

  // ---------------- FIFO / buffers ----------------
  parameter int unsigned IN_BYTE_FIFO_DEPTH = 1024, // bytes, power-of-two recommended
  parameter int unsigned MAX_PAYLOAD_BYTES  = 1<<20, // guard: 1 MiB per packet

  // ---------------- Options ----------------
  parameter bit          STRICT_STRB = 1'b1, // enforce contiguous strobes
  parameter bit          CHECK_MAGIC = 1'b1, // check magic + version + hdr_len
  parameter bit          EXPOSE_CRC  = 1'b1  // pass-through crc32 field in header sideband
)(
  input  logic                         clk_i,
  input  logic                         rstn_i, // synchronous, active-low

  // -------- Ingress stream (byte-oriented) --------
  input  logic                         in_valid_i,
  output logic                         in_ready_o,
  input  logic [IN_W-1:0]              in_data_i,
  input  logic [IN_W/8-1:0]            in_strb_i,   // LSB-contiguous on last beat

  // -------- Egress streams (arrays over destinations) --------
  output logic [NDEST-1:0]             out_valid_o,
  input  logic  [NDEST-1:0]            out_ready_i,
  output logic [NDEST-1:0][OUT_W-1:0]  out_data_o,
  output logic [NDEST-1:0][OUT_W/8-1:0]out_strb_o,
  output logic [NDEST-1:0]             out_last_o,

  // -------- Header sideband (one pulse per packet on selected dest) --------
  output logic [NDEST-1:0]             hdr_valid_o,
  input  logic  [NDEST-1:0]            hdr_ready_i,
  // Packed header info (same layout as table, little-endian fields)
  output logic [NDEST-1:0][127:0]      hdr_info_o,

  // -------- Status --------
  output logic                         busy_o,
  output logic [31:0]                  pkt_cnt_o,
  output logic [31:0]                  drop_cnt_o,
  output logic                         err_magic_o,
  output logic                         err_dest_o,
  output logic                         err_hdrlen_o,
  output logic                         err_overflow_o
);

  // --------------------------------------------------------------------------
  // Parameter checks
  // --------------------------------------------------------------------------
  initial begin
    if (IN_W % 8 != 0)  $error("pkt_unpack: IN_W must be multiple of 8.");
    if (OUT_W % 8 != 0) $error("pkt_unpack: OUT_W must be multiple of 8.");
    if (NDEST < 1)      $error("pkt_unpack: NDEST must be >= 1.");
  end

  localparam int unsigned IN_BYTES  = IN_W/8;
  localparam int unsigned OUT_BYTES = OUT_W/8;

  // ==========================================================================
  // Ingress Byte FIFO (word-wide push, byte-pop to parser/egress packer)
  // ==========================================================================
  localparam int unsigned BF_AW = (IN_BYTE_FIFO_DEPTH <= 2) ? 1 : $clog2(IN_BYTE_FIFO_DEPTH);
  logic [7:0]  bf_mem [0:IN_BYTE_FIFO_DEPTH-1];
  logic [BF_AW-1:0] bf_wptr_q, bf_rptr_q;
  logic [BF_AW:0]   bf_count_q;

  wire bf_full  = (bf_count_q == IN_BYTE_FIFO_DEPTH[BF_AW:0]);
  wire bf_empty = (bf_count_q == '0);

  // input ready when room for entire in_strb this beat
  function automatic [7:0] popcountN(input logic [IN_BYTES-1:0] v);
    automatic int i; automatic [7:0] s;
    begin s = 8'd0; for (i=0;i<IN_BYTES;i++) s = s + v[i]; popcountN = s; end
  endfunction

  logic [7:0] nbytes_this_beat;
  always_comb begin
    nbytes_this_beat = popcountN(in_strb_i);
  end

  assign in_ready_o = (bf_count_q + nbytes_this_beat) <= IN_BYTE_FIFO_DEPTH;

  // FIFO push
  integer ib;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      bf_wptr_q  <= '0;
      bf_rptr_q  <= '0;
      bf_count_q <= '0;
    end else if (in_valid_i && in_ready_o) begin
      for (ib = 0; ib < IN_BYTES; ib++) begin
        if (in_strb_i[ib]) begin
          bf_mem[bf_wptr_q] <= in_data_i[8*ib +: 8];
          bf_wptr_q         <= bf_wptr_q + 1'b1;
          bf_count_q        <= bf_count_q + 1'b1;
        end
      end
    end
  end

  // Byte pop helper
  function automatic logic [7:0] bf_peek;
    bf_peek = bf_mem[bf_rptr_q];
  endfunction

  // ==========================================================================
  // Packet parser & router
  // ==========================================================================
  typedef enum logic [2:0] { S_IDLE, S_HDR, S_HDR_CHECK, S_ROUTE_HDR, S_PAYLOAD, S_DROP } state_e;
  state_e state_q, state_d;

  // Header assembly (16 bytes)
  logic [127:0] hdr_q, hdr_d;
  logic [3:0]   hdr_bytes_col_q, hdr_bytes_col_d; // 0..15
  // Decoded header fields (latched when complete)
  logic [15:0]  h_magic_q;
  logic [7:0]   h_ver_q, h_hdrlen_q, h_dest_q, h_opcode_q;
  logic [15:0]  h_flags_q;
  logic [31:0]  h_paylen_q, h_crc_q;

  // Destination select & payload counters
  logic [$clog2((NDEST>1)?NDEST:2)-1:0] sel_dest_q, sel_dest_d;
  logic [31:0]  pay_bytes_rem_q, pay_bytes_rem_d;

  // Counters & errors
  logic [31:0]  pkt_cnt_q, pkt_cnt_d;
  logic [31:0]  drop_cnt_q, drop_cnt_d;
  logic         err_magic_q, err_magic_d;
  logic         err_dest_q,  err_dest_d;
  logic         err_hdrlen_q,err_hdrlen_d;
  logic         err_over_q,  err_over_d;

  // Busy
  assign busy_o = (state_q != S_IDLE) || (bf_count_q != 0);

  // Default all outputs to inactive
  always_comb begin
    out_valid_o  = '0;
    out_data_o   = '{default:'0};
    out_strb_o   = '{default:'0};
    out_last_o   = '0;

    hdr_valid_o  = '0;
    hdr_info_o   = '{default:'0};

    // status defaults
    pkt_cnt_d    = pkt_cnt_q;
    drop_cnt_d   = drop_cnt_q;
    err_magic_d  = err_magic_q;
    err_dest_d   = err_dest_q;
    err_hdrlen_d = err_hdrlen_q;
    err_over_d   = err_over_q;

    // FSM defaults
    state_d          = state_q;
    hdr_q            = hdr_d;           // hdr_d gets set below when collecting
    hdr_bytes_col_d  = hdr_bytes_col_q;

    sel_dest_d       = sel_dest_q;
    pay_bytes_rem_d  = pay_bytes_rem_q;
  end

  // ------------- Header byte collection & decode -------------
  always_comb begin
    // start from previous defaults
    hdr_d = hdr_q;

    if (state_q == S_HDR && !bf_empty) begin
      // place next byte into hdr_d at position hdr_bytes_col_q (little-endian)
      logic [7:0] b = bf_peek();
      hdr_d[8*hdr_bytes_col_q +: 8] = b;
    end
  end

  // ------------- Egress packer (OUT_W) -------------
  // Assemble OUT_BYTES from the byte FIFO and send to selected destination.
  // Cut-through: we only advance when the selected egress is ready (or we are accumulating).
  logic [OUT_W-1:0]  pk_acc_q, pk_acc_d;
  logic [7:0]        pk_bytes_q, pk_bytes_d; // 0..OUT_BYTES
  logic              pk_have_q,  pk_have_d;

  // Which destination is active this packet?
  wire [NDEST-1:0] sel_onehot = (NDEST==1) ? 1 : (logic[NDEST-1:0]) (1'b1 << sel_dest_q);

  // Egress drive: single selected destination gets the beat
  // (Combinational below when we finalize a packed word or the last partial)
  // We keep per-beat strobes contiguous from LSB.

  // ------------- Sequential: state, pointers, FIFO pops -------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      state_q         <= S_IDLE;

      hdr_q           <= '0;
      hdr_bytes_col_q <= '0;

      h_magic_q       <= 16'h0000;
      h_ver_q         <= 8'h00;
      h_hdrlen_q      <= 8'h00;
      h_dest_q        <= 8'h00;
      h_opcode_q      <= 8'h00;
      h_flags_q       <= 16'h0000;
      h_paylen_q      <= 32'd0;
      h_crc_q         <= 32'd0;

      sel_dest_q      <= '0;
      pay_bytes_rem_q <= 32'd0;

      pkt_cnt_q       <= 32'd0;
      drop_cnt_q      <= 32'd0;
      err_magic_q     <= 1'b0;
      err_dest_q      <= 1'b0;
      err_hdrlen_q    <= 1'b0;
      err_over_q      <= 1'b0;

      // byte FIFO read side
      // (bf_wptr_q/bf_rptr_q updated in ingress block)

      // packer
      pk_acc_q   <= '0;
      pk_bytes_q <= 8'd0;
      pk_have_q  <= 1'b0;
    end else begin
      state_q         <= state_d;
      hdr_q           <= hdr_q;          // hdr_d already applied when collecting
      hdr_bytes_col_q <= hdr_bytes_col_d;

      sel_dest_q      <= sel_dest_d;
      pay_bytes_rem_q <= pay_bytes_rem_d;

      pkt_cnt_q       <= pkt_cnt_d;
      drop_cnt_q      <= drop_cnt_d;
      err_magic_q     <= err_magic_d;
      err_dest_q      <= err_dest_d;
      err_hdrlen_q    <= err_hdrlen_d;
      err_over_q      <= err_over_d;

      pk_acc_q        <= pk_acc_d;
      pk_bytes_q      <= pk_bytes_d;
      pk_have_q       <= pk_have_d;

      // ---------------- State actions ----------------
      unique case (state_q)
        // ------------------------------------------------
        S_IDLE: begin
          // Reset packer
          pk_acc_q   <= '0;
          pk_bytes_q <= 8'd0;
          pk_have_q  <= 1'b0;

          if (bf_count_q != 0) begin
            // start header collection
            state_q         <= S_HDR;
            hdr_bytes_col_q <= 4'd0;
          end
        end

        // ------------------------------------------------
        S_HDR: begin
          if (!bf_empty) begin
            // consume one byte into header
            bf_rptr_q        <= bf_rptr_q + 1'b1;
            bf_count_q       <= bf_count_q - 1'b1;
            hdr_bytes_col_q  <= hdr_bytes_col_q + 4'd1;

            if (hdr_bytes_col_q == 4'd15) begin
              // header complete: decode fields
              h_magic_q   <= hdr_d[15:0];
              h_ver_q     <= hdr_d[23:16];
              h_hdrlen_q  <= hdr_d[31:24];
              h_dest_q    <= hdr_d[39:32];
              h_opcode_q  <= hdr_d[47:40];
              h_flags_q   <= hdr_d[63:48];
              h_paylen_q  <= hdr_d[95:64];
              h_crc_q     <= hdr_d[127:96];

              state_q     <= S_HDR_CHECK;
            end
          end
        end

        // ------------------------------------------------
        S_HDR_CHECK: begin
          // Validate basics
          logic ok = 1'b1;
          if (CHECK_MAGIC) begin
            if (h_magic_q != 16'hA151 || h_ver_q != 8'h01) begin
              ok          = 1'b0;
              err_magic_q <= 1'b1;
            end
            if (h_hdrlen_q != 8'd16) begin
              ok           = 1'b0;
              err_hdrlen_q <= 1'b1;
            end
          end
          if (h_paylen_q > MAX_PAYLOAD_BYTES) begin
            ok          = 1'b0;
            err_over_q  <= 1'b1;
          end
          if (h_dest_q >= NDEST[7:0]) begin
            ok         = 1'b0;
            err_dest_q <= 1'b1;
          end

          if (ok) begin
            sel_dest_q      <= h_dest_q[$clog2((NDEST>1)?NDEST:2)-1:0];
            pay_bytes_rem_q <= h_paylen_q;
            state_q         <= S_ROUTE_HDR;
          end else begin
            // drop payload bytes silently (using length); count drop
            drop_cnt_q      <= drop_cnt_q + 32'd1;
            pay_bytes_rem_q <= h_paylen_q;
            state_q         <= S_DROP;
          end
        end

        // ------------------------------------------------
        S_ROUTE_HDR: begin
          // Try to present header sideband to selected destination
          if (hdr_ready_i[sel_dest_q] || !hdr_valid_o[sel_dest_q]) begin
            // Will assert hdr_valid_o in comb this cycle with hdr_info
            // Next: emit payload
            state_q <= S_PAYLOAD;
            pkt_cnt_q <= pkt_cnt_q + 32'd1;
          end
        end

        // ------------------------------------------------
        S_PAYLOAD: begin
          // 1) Accumulate bytes from FIFO into pk_acc until we can emit OUT_BYTES or this is the final partial
          // 2) Emit to selected destination when either full word or last partial is ready and dest ready

          // Pull bytes into accumulator
          if (!bf_empty && (pk_bytes_q < OUT_BYTES) && (pay_bytes_rem_q != 0)) begin
            // consume one byte
            pk_acc_q[8*pk_bytes_q +: 8] <= bf_mem[bf_rptr_q];
            bf_rptr_q                   <= bf_rptr_q + 1'b1;
            bf_count_q                  <= bf_count_q - 1'b1;
            pk_bytes_q                  <= pk_bytes_q + 8'd1;
            pk_have_q                   <= 1'b1;
            pay_bytes_rem_q             <= pay_bytes_rem_q - 32'd1;
          end

          // Emit a full word when ready
          if ((pk_bytes_q == OUT_BYTES) && !out_valid_o[sel_dest_q]) begin
            // drive egress in comb; after handshake, clear accumulator
            if (out_ready_i[sel_dest_q]) begin
              pk_acc_q   <= '0;
              pk_bytes_q <= 8'd0;
              pk_have_q  <= 1'b0;
            end
          end

          // Emit final partial word (last) when no bytes remain and we have something
          if ((pay_bytes_rem_q == 0) && pk_have_q) begin
            // drive final with strobes=pk_bytes_q
            if (out_ready_i[sel_dest_q]) begin
              pk_acc_q   <= '0;
              pk_bytes_q <= 8'd0;
              pk_have_q  <= 1'b0;
              // Packet done
              state_q    <= S_IDLE;
            end
          end

          // If no payload at all (len=0), we still emit nothing to data channel; packet ends immediately
          if ((h_paylen_q == 0) && (pk_have_q == 1'b0)) begin
            state_q <= S_IDLE;
          end
        end

        // ------------------------------------------------
        S_DROP: begin
          // Discard h_paylen_q bytes from FIFO (no egress). If not enough buffered,
          // we'll continue discarding as they arrive.
          if (!bf_empty && (pay_bytes_rem_q != 0)) begin
            bf_rptr_q       <= bf_rptr_q + 1'b1;
            bf_count_q      <= bf_count_q - 1'b1;
            pay_bytes_rem_q <= pay_bytes_rem_q - 32'd1;
          end
          if (pay_bytes_rem_q == 0) begin
            state_q <= S_IDLE;
          end
        end

        default: state_q <= S_IDLE;
      endcase
    end
  end

  // ==========================================================================
  // Output drive (comb): header sideband + selected egress
  // ==========================================================================
  // Header sideband payload (packed 128 bits = header as-is)
  always_comb begin
    // default (cleared above)
    // hdr_info carries full header (fields already latched)
    logic [127:0] packed_hdr;
    packed_hdr = '0;
    packed_hdr[15:0]   = h_magic_q;
    packed_hdr[23:16]  = h_ver_q;
    packed_hdr[31:24]  = h_hdrlen_q;
    packed_hdr[39:32]  = h_dest_q;
    packed_hdr[47:40]  = h_opcode_q;
    packed_hdr[63:48]  = h_flags_q;
    packed_hdr[95:64]  = h_paylen_q;
    packed_hdr[127:96] = EXPOSE_CRC ? h_crc_q : 32'h0;

    if (state_q == S_ROUTE_HDR) begin
      hdr_valid_o[sel_dest_q] = 1'b1;
      hdr_info_o [sel_dest_q] = packed_hdr;
    end

    // Egress data path
    // Full word available
    if ((state_q == S_PAYLOAD) && (pk_bytes_q == OUT_BYTES)) begin
      out_valid_o[sel_dest_q] = 1'b1;
      out_data_o [sel_dest_q] = pk_acc_q;
      out_strb_o [sel_dest_q] = {OUT_BYTES{1'b1}};
      out_last_o [sel_dest_q] = 1'b0;
    end

    // Final partial word
    if ((state_q == S_PAYLOAD) && (pay_bytes_rem_q == 0) && pk_have_q && (pk_bytes_q != 0)) begin
      out_valid_o[sel_dest_q] = 1'b1;
      out_data_o [sel_dest_q] = pk_acc_q;

      // Contiguous LSB strobes up to pk_bytes_q
      out_strb_o [sel_dest_q] = '0;
      for (int sb=0; sb<OUT_BYTES; sb++) begin
        out_strb_o[sel_dest_q][sb] = (sb < pk_bytes_q);
      end

      out_last_o [sel_dest_q] = 1'b1;
    end
  end

  // ==========================================================================
  // Status outputs
  // ==========================================================================
  assign pkt_cnt_o      = pkt_cnt_q;
  assign drop_cnt_o     = drop_cnt_q;
  assign err_magic_o    = err_magic_q;
  assign err_dest_o     = err_dest_q;
  assign err_hdrlen_o   = err_hdrlen_q;
  assign err_overflow_o = err_over_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // in_strb must be contiguous from LSB (relaxed for non-final beats: allow all-ones)
  if (STRICT_STRB) begin : g_strb_assert
    function automatic logic is_contig(input logic [IN_BYTES-1:0] s);
      int i; logic seen0; begin
        seen0 = 1'b0;
        for (i=0; i<IN_BYTES; i++) begin
          if (!s[i]) seen0 = 1'b1;
          else if (seen0) return 1'b0;
        end
        return 1'b1;
      end
    endfunction
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      in_valid_i |-> is_contig(in_strb_i))
      else $warning("pkt_unpack: in_strb_i not LSB-contiguous");
  end

  // No byte-FIFO overflow if upstream obeys ready
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(in_valid_i && !in_ready_o))
    else $warning("pkt_unpack: upstream ignored ready");

  // Header length fixed to 16 in v1
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (CHECK_MAGIC && state_q==S_HDR_CHECK) |-> (h_hdrlen_q == 8'd16))
    else $error("pkt_unpack: header length != 16");

  // Destination bounds
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (state_q==S_HDR_CHECK) |-> (h_dest_q < NDEST))
    else $warning("pkt_unpack: dest out-of-range");

  // Do not assert two egress valids in the same cycle
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    $onehot0(out_valid_o))
    else $error("pkt_unpack: multiple egress valids in same cycle");
`endif

endmodule

`default_nettype wire
