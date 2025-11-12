// ============================================================================
//  ZRLE Activations Decompressor (ZRLE-A v1)
//  File: zrle_activ_decomp.sv
//  Description:
//    - Specialized decompressor for sparse activations (ReLU-heavy).
//    - Token set: ZERO_RUN, LITERAL_RUN, BITMASK_BLOCK, EOS.
//    - Streaming input (byte-granular), packed streaming output.
//    - Robust error handling, assertions, and parameterization.
//
//  Parameters:
//    IN_W            : input bus width (bits), multiple of 8 (default 64)
//    OUT_W           : output bus width (bits), multiple of 8 and of ELEM_W (default 512)
//    ELEM_W          : element width (bits), multiple of 8 (e.g., 8, 16)
//    BYTE_FIFO_DEPTH : ingress byte FIFO depth (bytes; power-of-two)
//    WORD_FIFO_DEPTH : element FIFO depth (elements)
//    STRICT_IN_STRB  : assert contiguity on in_strb_i for last beat
//
//  ZRLE-A v1 format:
//   header[7:5] TYPE, header[4:0] LENm1; if LENm1==31, COUNT=32+LE16(ext), else COUNT=LENm1+1.
//   TYPE:
//     000 ZERO_RUN:     emit COUNT zeros.
//     001 LITERAL_RUN:  read COUNT elements (ELEM_W bytes each), emit.
//     010 BITMASK_BLOCK: group size G encoded in header[4:3] (00:8, 01:16, 10:32, 11:64).
//                        Then read ceil(G/8) mask bytes (LSB-first). For each '1' bit, read one
//                        element and emit at that position; for '0' bit, emit zero. Emits G elems.
//     111 EOS: end-of-stream. out_last_o asserted on final packed beat.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module zrle_activ_decomp #(
  parameter int unsigned IN_W             = 64,
  parameter int unsigned OUT_W            = 512,
  parameter int unsigned ELEM_W           = 8,
  parameter int unsigned BYTE_FIFO_DEPTH  = 1024, // bytes (power-of-two)
  parameter int unsigned WORD_FIFO_DEPTH  = 64,   // elements
  parameter bit          STRICT_IN_STRB   = 1'b1
)(
  input  logic                  clk_i,
  input  logic                  rstn_i,         // synchronous, active-low

  // -------- Input (byte-stream) --------
  input  logic                  in_valid_i,
  output logic                  in_ready_o,
  input  logic [IN_W-1:0]       in_data_i,
  input  logic [IN_W/8-1:0]     in_strb_i,      // valid bytes in in_data_i
  input  logic                  in_last_i,

  // -------- Output (packed element stream) --------
  output logic                  out_valid_o,
  input  logic                  out_ready_i,
  output logic [OUT_W-1:0]      out_data_o,
  output logic [OUT_W/8-1:0]    out_strb_o,     // byte strobes (contiguous from LSB)
  output logic                  out_last_o,

  // -------- Status --------
  output logic                  busy_o,
  output logic                  eos_seen_o,
  output logic                  err_o,
  output logic [3:0]            err_code_o
);

  // ==========================================================================
  // Static / parameter checks
  // ==========================================================================
  initial begin
    if (IN_W % 8 != 0)   $error("zrle_activ_decomp: IN_W must be multiple of 8");
    if (OUT_W % 8 != 0)  $error("zrle_activ_decomp: OUT_W must be multiple of 8");
    if (ELEM_W % 8 != 0) $error("zrle_activ_decomp: ELEM_W must be multiple of 8");
    if ((OUT_W % ELEM_W) != 0) $error("zrle_activ_decomp: OUT_W must be multiple of ELEM_W");
    if ((BYTE_FIFO_DEPTH & (BYTE_FIFO_DEPTH-1)) != 0)
      $error("zrle_activ_decomp: BYTE_FIFO_DEPTH must be power-of-two");
  end

  localparam int unsigned IN_BYTES   = IN_W/8;
  localparam int unsigned OUT_BYTES  = OUT_W/8;
  localparam int unsigned ELEM_BYTES = ELEM_W/8;

  // ==========================================================================
  // Error codes
  // ==========================================================================
  localparam logic [3:0]
    EC_NONE         = 4'h0,
    EC_BAD_TOKEN    = 4'h1,
    EC_UNDERFLOW    = 4'h2, // needed bytes not present before EOS/in_last
    EC_EARLY_INLAST = 4'h3, // in_last before EOS observed
    EC_MASK_OVERRUN = 4'h4; // defensive (mask consumed beyond group)

  // ==========================================================================
  // Ingress Byte FIFO (push IN_BYTES at a time, pop one byte)
  // ==========================================================================
  localparam int unsigned BF_AW = $clog2(BYTE_FIFO_DEPTH);

  logic [7:0]          bf_mem   [0:BYTE_FIFO_DEPTH-1];
  logic [BF_AW-1:0]    bf_wptr_q, bf_rptr_q;
  logic [BF_AW:0]      bf_count_q;
  logic                bf_full, bf_empty;

  assign bf_full  = (bf_count_q == BYTE_FIFO_DEPTH[BF_AW:0]);
  assign bf_empty = (bf_count_q == '0);

  // Flatten input bytes for convenience
  logic [7:0] in_bytes [0:IN_BYTES-1];
  genvar ib;
  generate
    for (ib=0; ib<IN_BYTES; ib++) begin : G_INB
      always_comb in_bytes[ib] = in_data_i[8*ib +: 8];
    end
  endgenerate

  // popcount for in_strb_i (used in assertions)
  function automatic [7:0] popcountN(input logic [IN_BYTES-1:0] v);
    automatic int i; automatic [7:0] s;
    begin s = 8'd0; for (i=0;i<IN_BYTES;i++) s = s + v[i]; popcountN = s; end
  endfunction

  // Ready policy: require space for a full beat (simplifies backpressure)
  assign in_ready_o = !bf_full && (bf_count_q <= (BYTE_FIFO_DEPTH - IN_BYTES));

  // Push on handshake
  wire in_fire = in_valid_i && in_ready_o;

  // Byte FIFO push/pop
  logic bf_pop;

  // in_last before EOS error flagging (sticky)
  logic in_last_early_err_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      bf_wptr_q          <= '0;
      bf_rptr_q          <= '0;
      bf_count_q         <= '0;
      in_last_early_err_q<= 1'b0;
    end else begin
      // Push
      if (in_fire) begin
        for (int k=0; k<IN_BYTES; k++) begin
          if (in_strb_i[k]) begin
            bf_mem[bf_wptr_q] <= in_bytes[k];
            bf_wptr_q         <= bf_wptr_q + 1'b1;
            bf_count_q        <= bf_count_q + 1'b1;
          end
        end
      end
      // Pop
      if (bf_pop && !bf_empty) begin
        bf_rptr_q  <= bf_rptr_q + 1'b1;
        bf_count_q <= bf_count_q - 1'b1;
      end
    end
  end

`ifdef ASSERT_ON
  if (STRICT_IN_STRB) begin
    // On last beat, strobes must be contiguous from LSB
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      in_valid_i && in_last_i |-> (in_strb_i == {popcountN(in_strb_i){1'b1}}))
      else $warning("zrle_activ_decomp: in_strb_i not contiguous from LSB on last beat");
  end
`endif

  // ==========================================================================
  // Parser → Word FIFO (element width)
  // ==========================================================================
  localparam int unsigned WF_AW = (WORD_FIFO_DEPTH <= 2) ? 1 : $clog2(WORD_FIFO_DEPTH);
  logic [ELEM_W-1:0]     wf_mem [0:WORD_FIFO_DEPTH-1];
  logic [WF_AW:0]        wf_wptr_q, wf_rptr_q;
  wire                   wf_full  = (wf_wptr_q[WF_AW]     != wf_rptr_q[WF_AW]) &&
                                    (wf_wptr_q[WF_AW-1:0] == wf_rptr_q[WF_AW-1:0]);
  wire                   wf_empty = (wf_wptr_q == wf_rptr_q);
  wire [ELEM_W-1:0]      wf_rdata = wf_mem[wf_rptr_q[WF_AW-1:0]];
  logic                  wf_push, wf_pop;

  // Parser state & registers
  typedef enum logic [4:0] {
    PS_IDLE, PS_LEN_LO, PS_LEN_HI,
    PS_ZERO_EMIT,
    PS_LIT_ACCUM,
    PS_MASK_INIT, PS_MASK_LOAD, PS_MASK_EMIT, PS_MASK_GETVAL,
    PS_EOS, PS_ERROR
  } pstate_e;

  pstate_e            ps_q, ps_d;

  // Current token context
  logic [2:0]         tok_type_q, tok_type_d;   // TYPE
  logic [31:0]        tok_cnt_q,  tok_cnt_d;    // COUNT (remaining)
  logic               need_ext_q, need_ext_d;   // LEN extension pending

  // LITERAL assembly
  logic [ELEM_W-1:0]  lit_acc_q, lit_acc_d;
  logic [$clog2(ELEM_BYTES+1)-1:0] lit_bytes_q, lit_bytes_d;

  // BITMASK block context
  logic [6:0]         mask_group_q, mask_group_d;      // 8/16/32/64
  logic [6:0]         mask_left_q,  mask_left_d;       // positions left in block
  logic [7:0]         mask_byte_q,  mask_byte_d;       // current mask byte
  logic [2:0]         mask_bit_idx_q, mask_bit_idx_d;  // 0..7 bit pointer within mask_byte_q
  logic [7:0]         mask_bytes_to_read_q, mask_bytes_to_read_d; // ceil(G/8)
  logic [7:0]         mask_bytes_read_q, mask_bytes_read_d;

  // Value assembly for mask '1' positions
  logic [ELEM_W-1:0]  val_acc_q,  val_acc_d;
  logic [$clog2(ELEM_BYTES+1)-1:0] val_bytes_q, val_bytes_d;

  // EOS, errors
  logic               eos_seen_q, eos_seen_d;
  logic               err_q, err_d;
  logic [3:0]         errc_q, errc_d;

  // Busy
  assign busy_o     = (ps_q != PS_IDLE) || !wf_empty || eos_seen_q;
  assign eos_seen_o = eos_seen_q;
  assign err_o      = err_q;
  assign err_code_o = errc_q;

  // Helpers
  function automatic [7:0] bf_peek;
    bf_peek = bf_mem[bf_rptr_q];
  endfunction

  // Parser sequential
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      ps_q        <= PS_IDLE;
      tok_type_q  <= 3'b000;
      tok_cnt_q   <= '0;
      need_ext_q  <= 1'b0;

      lit_acc_q   <= '0;
      lit_bytes_q <= '0;

      mask_group_q<= 7'd0;
      mask_left_q <= 7'd0;
      mask_byte_q <= 8'h00;
      mask_bit_idx_q <= 3'd0;
      mask_bytes_to_read_q <= 8'd0;
      mask_bytes_read_q    <= 8'd0;

      val_acc_q   <= '0;
      val_bytes_q <= '0;

      wf_wptr_q   <= '0;
      wf_rptr_q   <= '0;

      eos_seen_q  <= 1'b0;
      err_q       <= 1'b0;
      errc_q      <= EC_NONE;
    end else begin
      ps_q        <= ps_d;
      tok_type_q  <= tok_type_d;
      tok_cnt_q   <= tok_cnt_d;
      need_ext_q  <= need_ext_d;

      lit_acc_q   <= lit_acc_d;
      lit_bytes_q <= lit_bytes_d;

      mask_group_q<= mask_group_d;
      mask_left_q <= mask_left_d;
      mask_byte_q <= mask_byte_d;
      mask_bit_idx_q <= mask_bit_idx_d;
      mask_bytes_to_read_q <= mask_bytes_to_read_d;
      mask_bytes_read_q    <= mask_bytes_read_d;

      val_acc_q   <= val_acc_d;
      val_bytes_q <= val_bytes_d;

      // Word FIFO push
      if (wf_push && !wf_full) begin
        wf_mem[wf_wptr_q[WF_AW-1:0]] <= (ps_q == PS_LIT_ACCUM) ? lit_acc_d : val_acc_d;
        wf_wptr_q <= wf_wptr_q + 1'b1;
      end
      // Pop by packer
      if (wf_pop && !wf_empty) begin
        wf_rptr_q <= wf_rptr_q + 1'b1;
      end

      // Byte FIFO pop (single)
      if (bf_pop && !bf_empty) begin
        bf_rptr_q  <= bf_rptr_q + 1'b1;
        bf_count_q <= bf_count_q - 1'b1;
      end

      // EOS / error (sticky)
      eos_seen_q <= eos_seen_d;
      err_q      <= err_q | err_d;
      if (err_d) errc_q <= errc_d;
    end
  end

  // Parser combinational
  always_comb begin
    // defaults
    ps_d        = ps_q;
    tok_type_d  = tok_type_q;
    tok_cnt_d   = tok_cnt_q;
    need_ext_d  = need_ext_q;

    lit_acc_d   = lit_acc_q;
    lit_bytes_d = lit_bytes_q;

    mask_group_d          = mask_group_q;
    mask_left_d           = mask_left_q;
    mask_byte_d           = mask_byte_q;
    mask_bit_idx_d        = mask_bit_idx_q;
    mask_bytes_to_read_d  = mask_bytes_to_read_q;
    mask_bytes_read_d     = mask_bytes_read_q;

    val_acc_d   = val_acc_q;
    val_bytes_d = val_bytes_q;

    wf_push     = 1'b0;
    bf_pop      = 1'b0;

    eos_seen_d  = eos_seen_q;
    err_d       = 1'b0;
    errc_d      = EC_NONE;

    // ------------------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------------------
    unique case (ps_q)
      // --------------------------------------------------------------
      PS_IDLE: begin
        // Need a header byte
        if (!bf_empty) begin
          logic [7:0] hdr = bf_peek();
          bf_pop      = 1'b1;
          tok_type_d  = hdr[7:5];

          // EOS (no payload)
          if (hdr[7:5] == 3'b111) begin
            eos_seen_d = 1'b1;
            ps_d       = PS_EOS; // flush handled by packer
          end else if (hdr[7:5] == 3'b010) begin
            // BITMASK_BLOCK: group size in hdr[4:3]
            automatic logic [1:0] gs = hdr[4:3];
            case (gs)
              2'b00: mask_group_d = 7'd8;
              2'b01: mask_group_d = 7'd16;
              2'b10: mask_group_d = 7'd32;
              2'b11: mask_group_d = 7'd64;
            endcase
            mask_left_d          = mask_group_d;
            mask_bytes_to_read_d = (mask_group_d + 7) >> 3; // ceil(G/8)
            mask_bytes_read_d    = 8'd0;
            mask_bit_idx_d       = 3'd0;
            ps_d = PS_MASK_LOAD;
          end else begin
            // ZERO_RUN or LITERAL_RUN with optional extended length
            if (hdr[4:0] == 5'd31) begin
              need_ext_d = 1'b1;
              tok_cnt_d  = 32'd0;
              ps_d       = PS_LEN_LO;
            } else begin
              need_ext_d = 1'b0;
              tok_cnt_d  = {27'd0, hdr[4:0]} + 32'd1;
              if (hdr[7:5] == 3'b000) begin
                // ZERO_RUN
                ps_d = PS_ZERO_EMIT;
              end else if (hdr[7:5] == 3'b001) begin
                // LITERAL_RUN
                lit_acc_d   = '0;
                lit_bytes_d = '0;
                ps_d = PS_LIT_ACCUM;
              end else begin
                // invalid type
                ps_d  = PS_ERROR; err_d = 1'b1; errc_d = EC_BAD_TOKEN;
              end
            end
          end
        end
      end

      // --------------------------------------------------------------
      // Extended length: COUNT = 32 + ext (LE16)
      PS_LEN_LO: begin
        if (!bf_empty) begin
          tok_cnt_d = {tok_cnt_q[31:8], bf_peek()};
          bf_pop    = 1'b1;
          ps_d      = PS_LEN_HI;
        end
      end
      PS_LEN_HI: begin
        if (!bf_empty) begin
          logic [15:0] ext = {bf_peek(), tok_cnt_q[7:0]};
          bf_pop    = 1'b1;
          tok_cnt_d = 32'(32 + ext);
          need_ext_d= 1'b0;
          // Dispatch to the right payload state
          case (tok_type_q)
            3'b000: ps_d = PS_ZERO_EMIT;
            3'b001: begin lit_acc_d='0; lit_bytes_d='0; ps_d = PS_LIT_ACCUM; end
            default: begin ps_d=PS_ERROR; err_d=1'b1; errc_d=EC_BAD_TOKEN; end
          endcase
        end
      end

      // --------------------------------------------------------------
      // ZERO_RUN: emit zeros
      PS_ZERO_EMIT: begin
        if (tok_cnt_q == 0) begin
          ps_d = PS_IDLE;
        end else if (!wf_full) begin
          // push zero element
          val_acc_d    = '0;
          wf_push      = 1'b1;
          tok_cnt_d    = tok_cnt_q - 32'd1;
        end
      end

      // --------------------------------------------------------------
      // LITERAL_RUN: read ELEM_BYTES per element, little-endian
      PS_LIT_ACCUM: begin
        if (tok_cnt_q == 0) begin
          ps_d = PS_IDLE;
        end else if (!bf_empty) begin
          // consume one byte
          lit_acc_d   = lit_acc_q | ( ELEM_W'({24'd0, bf_peek()}) << (8*lit_bytes_q) );
          bf_pop      = 1'b1;
          lit_bytes_d = lit_bytes_q + 1'b1;

          if (lit_bytes_q + 1 == ELEM_BYTES[$clog2(ELEM_BYTES+1)-1:0]) begin
            // have one element
            if (!wf_full) begin
              wf_push      = 1'b1;
              tok_cnt_d    = tok_cnt_q - 32'd1;
              lit_acc_d    = '0;
              lit_bytes_d  = '0;
            end
          end
        end
      end

      // --------------------------------------------------------------
      // BITMASK_BLOCK:
      //  - load mask bytes (ceil(G/8))
      //  - iterate bits; for 0 -> emit zero; for 1 -> read element bytes then emit
      PS_MASK_LOAD: begin
        if (mask_bytes_read_q == mask_bytes_to_read_q) begin
          // no more bytes to read? then start emitting per-bit.
          mask_bit_idx_d = 3'd0;
          ps_d           = PS_MASK_EMIT;
        end else if (!bf_empty) begin
          // load next mask byte
          mask_byte_d        = bf_peek();
          bf_pop             = 1'b1;
          mask_bytes_read_d  = mask_bytes_read_q + 8'd1;
          mask_bit_idx_d     = 3'd0;
          ps_d               = PS_MASK_EMIT;
        end
      end

      PS_MASK_EMIT: begin
        if (mask_left_q == 0) begin
          // block done
          ps_d = PS_IDLE;
        end else if (!wf_full) begin
          // If we exhausted current mask byte bits, fetch next
          if (mask_bit_idx_q == 3'd8) begin
            ps_d = PS_MASK_LOAD;
          end else begin
            logic this_bit = mask_byte_q[mask_bit_idx_q];
            if (!this_bit) begin
              // emit zero immediately
              val_acc_d      = '0;
              wf_push        = 1'b1;
              mask_bit_idx_d = mask_bit_idx_q + 3'd1;
              mask_left_d    = mask_left_q - 7'd1;
            end else begin
              // need to read an element value
              val_acc_d      = '0;
              val_bytes_d    = '0;
              ps_d           = PS_MASK_GETVAL;
            end
          end
        end
      end

      PS_MASK_GETVAL: begin
        // read ELEM_BYTES to assemble one value at this position
        if (!bf_empty) begin
          val_acc_d    = val_acc_q | ( ELEM_W'({24'd0, bf_peek()}) << (8*val_bytes_q) );
          bf_pop       = 1'b1;
          val_bytes_d  = val_bytes_q + 1'b1;
          if (val_bytes_q + 1 == ELEM_BYTES[$clog2(ELEM_BYTES+1)-1:0]) begin
            // push assembled value
            if (!wf_full) begin
              wf_push        = 1'b1;
              // advance mask position
              mask_bit_idx_d = mask_bit_idx_q + 3'd1;
              mask_left_d    = mask_left_q - 7'd1;
              // back to EMIT for next bit (or LOAD if end of byte)
              ps_d           = PS_MASK_EMIT;
              val_acc_d      = '0;
              val_bytes_d    = '0;
            end
          end
        end
      end

      // --------------------------------------------------------------
      PS_EOS: begin
        // Nothing to do here, packer handles flush & out_last. Return to IDLE.
        ps_d = PS_IDLE;
      end

      PS_ERROR: begin
        // Latch error and stall (sticky until reset)
        err_d  = 1'b1;
        errc_d = (errc_q != EC_NONE) ? errc_q : EC_BAD_TOKEN;
        ps_d   = PS_ERROR;
      end

      default: begin
        ps_d  = PS_ERROR; err_d = 1'b1; errc_d = EC_BAD_TOKEN;
      end
    endcase

    // Early in_last before EOS → flag error (does not halt)
    if (in_fire && in_last_i && !eos_seen_q) begin
      err_d  = 1'b1;
      errc_d = EC_EARLY_INLAST;
    end
  end

  // ==========================================================================
  // Output packer: ELEM_W → OUT_W with byte strobes and LAST
  // ==========================================================================
  localparam int unsigned PACK_RATIO = OUT_W / ELEM_W;

  logic                 pk_have_q, pk_have_d;           // partial present
  logic [OUT_W-1:0]     pk_acc_q, pk_acc_d;             // accumulator
  logic [OUT_BYTES:0]   pk_bytes_q, pk_bytes_d;         // bytes accumulated

  logic                 out_v_q, out_v_d;
  logic [OUT_W-1:0]     out_d_q, out_d_d;
  logic [OUT_BYTES-1:0] out_s_q, out_s_d;
  logic                 out_l_q, out_l_d;

  assign out_valid_o = out_v_q;
  assign out_data_o  = out_d_q;
  assign out_strb_o  = out_s_q;
  assign out_last_o  = out_l_q;

  // Pop from word FIFO when we can place into accumulator (and not holding a beat)
  assign wf_pop = !wf_empty &&
                  ( (pk_bytes_q + ELEM_BYTES <= OUT_BYTES) &&
                    (!out_v_q || (out_v_q && out_ready_i)) );

  // Packer comb
  always_comb begin
    pk_acc_d    = pk_acc_q;
    pk_bytes_d  = pk_bytes_q;
    pk_have_d   = pk_have_q;

    out_v_d     = out_v_q;
    out_d_d     = out_d_q;
    out_s_d     = out_s_q;
    out_l_d     = out_l_q;

    // Drop when consumed
    if (out_v_q && out_ready_i) begin
      out_v_d = 1'b0;
      out_l_d = 1'b0;
    end

    // Place next element
    if (wf_pop) begin
      automatic int bit_off = pk_bytes_q * 8;
      pk_acc_d[bit_off +: ELEM_W] = wf_rdata;
      pk_bytes_d = pk_bytes_q + ELEM_BYTES;
      pk_have_d  = 1'b1;

      // Full beat -> emit
      if (pk_bytes_d == OUT_BYTES) begin
        if (!out_v_q || (out_v_q && out_ready_i)) begin
          out_d_d = pk_acc_d;
          out_s_d = {OUT_BYTES{1'b1}};
          out_v_d = 1'b1;
          // Reset accumulator
          pk_acc_d   = '0;
          pk_bytes_d = '0;
          pk_have_d  = 1'b0;
        end
      end
    end

    // Flush on EOS when parser idle-ish and FIFOs drained
    logic parser_idle = (ps_q == PS_IDLE);
    logic can_emit    = (!out_v_q || (out_v_q && out_ready_i));

    if (eos_seen_q && wf_empty && parser_idle && can_emit) begin
      if (pk_bytes_q != 0) begin
        out_d_d = pk_acc_d;
        // Contiguous strobes
        out_s_d = '0;
        for (int sb=0; sb<OUT_BYTES; sb++) out_s_d[sb] = (sb < pk_bytes_q);
        out_v_d = 1'b1;
        out_l_d = 1'b1;
        // Clear accumulator
        pk_acc_d   = '0;
        pk_bytes_d = '0;
        pk_have_d  = 1'b0;
      end else if (!out_v_q) begin
        // No partial -> last already emitted on a full beat earlier.
        // Nothing to do here (we don't send zero-length beats).
      end
    end

    // Clear EOS after the LAST beat is taken
    if (out_v_q && out_ready_i && out_l_q) begin
      eos_seen_d = 1'b0;
    end
  end

  // Packer sequential
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      pk_acc_q   <= '0;
      pk_bytes_q <= '0;
      pk_have_q  <= 1'b0;
      out_v_q    <= 1'b0;
      out_d_q    <= '0;
      out_s_q    <= '0;
      out_l_q    <= 1'b0;
    end else begin
      pk_acc_q   <= pk_acc_d;
      pk_bytes_q <= pk_bytes_d;
      pk_have_q  <= pk_have_d;

      out_v_q    <= out_v_d;
      out_d_q    <= out_d_d;
      out_s_q    <= out_s_d;
      out_l_q    <= out_l_d;
    end
  end

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Word FIFO safety
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(wf_push && wf_full)) else $error("zrle_activ_decomp: word FIFO overflow");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(wf_pop && wf_empty)) else $error("zrle_activ_decomp: word FIFO underflow");

  // OUT_W multiple of ELEM_W (redundant with param check)
  initial begin
    if ((OUT_W % ELEM_W) != 0) $fatal("zrle_activ_decomp: OUT_W must be multiple of ELEM_W");
  end
`endif

endmodule

`default_nettype wire
