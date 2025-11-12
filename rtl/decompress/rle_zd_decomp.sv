// ============================================================================
//  RLE + Zero/Delta Decompressor
//  File: rle_zd_decomp.sv
//  Description:
//    Byte-stream decoder for compressed weights/activations.
//    See table above for opcode and delta definitions.
//
//  Typical config: ELEM_W=8, OUT_ELEMS=64 → 512‑bit output to GB bus.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module rle_zd_decomp #(
  parameter int unsigned IN_W       = 64,    // input bus bits
  parameter int unsigned ELEM_W     = 8,     // element bits (8/16)
  parameter int unsigned OUT_ELEMS  = 64,    // elements per output word
  parameter int unsigned FIFO_BYTES = 64     // internal byte buffer
)(
  input  logic                     clk_i,
  input  logic                     rstn_i,

  // Compressed input
  input  logic                     in_valid_i,
  output logic                     in_ready_o,
  input  logic [IN_W-1:0]          in_data_i,
  input  logic                     in_last_i,
  input  logic [$clog2(IN_W/8):0]  in_last_bytes_i,

  // Decompressed output
  output logic                     out_valid_o,
  input  logic                     out_ready_i,
  output logic [OUT_ELEMS*ELEM_W-1:0] out_data_o,
  output logic                     out_last_o,
  output logic [$clog2(OUT_ELEMS+1)-1:0] out_last_elems_o
);

  // --------------------------------------------------------------------------
  // Byte FIFO to stage input stream
  // --------------------------------------------------------------------------
  localparam int unsigned FIFO_AW = (FIFO_BYTES <= 2) ? 1 : $clog2(FIFO_BYTES);
  logic [7:0] fifo_mem [0:FIFO_BYTES-1];
  logic [FIFO_AW:0] wr_ptr_q, rd_ptr_q;
  wire  fifo_empty = (wr_ptr_q == rd_ptr_q);
  wire  fifo_full  = (wr_ptr_q[FIFO_AW]     != rd_ptr_q[FIFO_AW]) &&
                     (wr_ptr_q[FIFO_AW-1:0] == rd_ptr_q[FIFO_AW-1:0]);
  wire  [7:0] fifo_head = fifo_mem[rd_ptr_q[FIFO_AW-1:0]];

  // bytes in flight
  wire [FIFO_AW:0] fifo_count = wr_ptr_q - rd_ptr_q;
  assign in_ready_o = !fifo_full;

  integer b;
  always_ff @(posedge clk_i) begin
    if (!rstn_i) begin
      wr_ptr_q <= '0;
      rd_ptr_q <= '0;
    end else begin
      // write from input
      if (in_valid_i && in_ready_o) begin
        int unsigned bytes = (in_last_i) ? in_last_bytes_i : (IN_W/8);
        for (b=0; b<IN_W/8; b++) begin
          if (b < bytes) begin
            fifo_mem[wr_ptr_q[FIFO_AW-1:0]] <= in_data_i[8*b +: 8];
            wr_ptr_q <= wr_ptr_q + 1'b1;
          end
        end
      end
    end
  end

  // pop helper
  task automatic pop_byte(output logic [7:0] val);
    val = fifo_mem[rd_ptr_q[FIFO_AW-1:0]];
    rd_ptr_q <= rd_ptr_q + 1'b1;
  endtask

  // --------------------------------------------------------------------------
  // Output element FIFO (each element = ELEM_W bits)
  // --------------------------------------------------------------------------
  localparam int unsigned EFIFO_DEPTH = OUT_ELEMS*4;
  localparam int unsigned EFIFO_AW    = (EFIFO_DEPTH<=2)?1:$clog2(EFIFO_DEPTH);
  logic [ELEM_W-1:0] efifo_mem [0:EFIFO_DEPTH-1];
  logic [EFIFO_AW:0] e_wptr_q, e_rptr_q;
  wire  e_empty = (e_wptr_q == e_rptr_q);
  wire  e_full  = (e_wptr_q[EFIFO_AW]     != e_rptr_q[EFIFO_AW]) &&
                  (e_wptr_q[EFIFO_AW-1:0] == e_rptr_q[EFIFO_AW-1:0]);
  wire  [ELEM_W-1:0] e_head = efifo_mem[e_rptr_q[EFIFO_AW-1:0]];

  // helper
  task automatic push_elem(input logic [ELEM_W-1:0] val);
    efifo_mem[e_wptr_q[EFIFO_AW-1:0]] <= val;
    e_wptr_q <= e_wptr_q + 1'b1;
  endtask
  task automatic pop_elem(output logic [ELEM_W-1:0] val);
    val = efifo_mem[e_rptr_q[EFIFO_AW-1:0]];
    e_rptr_q <= e_rptr_q + 1'b1;
  endtask

  // --------------------------------------------------------------------------
  // Token parser FSM
  // --------------------------------------------------------------------------
  typedef enum logic [3:0] {
    S_IDLE, S_FETCH, S_EXTLEN, S_LIT, S_ZRUN, S_RVAL, S_RLE, S_DELTA_BASE,
    S_DELTA_CODES, S_DONE
  } state_e;

  state_e state_q, state_d;

  logic [7:0] token_q;
  logic [9:0] seg_len_q;          // up to 288 elements
  logic [9:0] seg_left_q;
  logic [ELEM_W-1:0] cur_val_q;   // last value (for delta)
  logic [7:0] delta_byte_q;       // current nibble source
  logic       delta_nib_sel_q;    // 0=low nibble next

  logic [31:0] elem_count_q;      // total elems emitted (for last calc)
  logic [31:0] total_elem_target_q;

  logic last_input_seen_q;        // got last_i
  logic stream_done_q;

  // FSM sequential
  always_ff @(posedge clk_i) begin
    if (!rstn_i) begin
      state_q <= S_IDLE;
      seg_len_q <= '0;
      seg_left_q <= '0;
      token_q <= '0;
      cur_val_q <= '0;
      delta_byte_q <= '0;
      delta_nib_sel_q <= 1'b0;
      elem_count_q <= '0;
      last_input_seen_q <= 1'b0;
      stream_done_q <= 1'b0;
      e_wptr_q <= '0;
      e_rptr_q <= '0;
    end else begin
      state_q <= state_d;
    end
  end

  // FSM next-state + actions
  always_comb begin
    state_d = state_q;
    case (state_q)
      S_IDLE: if (!fifo_empty) state_d = S_FETCH;
      S_FETCH: begin
        if (!fifo_empty) begin
          token_q = fifo_head;
          state_d = S_EXTLEN;
        end
      end
      S_EXTLEN: begin
        // compute length
        int unsigned L = (token_q[4:0]==0) ? fifo_head+32 : token_q[4:0];
        if (token_q[4:0]==0 && fifo_empty)
          state_d = S_EXTLEN; // wait for length byte
        else begin
          seg_len_q  = L;
          seg_left_q = L;
          unique case (token_q[7:5])
            3'b000: state_d = S_LIT;
            3'b001: state_d = S_ZRUN;
            3'b010: state_d = S_RVAL;
            3'b011: state_d = S_DELTA_BASE;
            default: state_d = S_DONE;
          endcase
        end
      end
      S_LIT: if (!fifo_empty && !e_full) begin
        // pop one element (ELEM_W bytes)
        if (ELEM_W==8) begin
          push_elem(fifo_head); seg_left_q--; rd_ptr_q++;
        end else begin
          if (fifo_count>=2) begin
            push_elem({fifo_mem[rd_ptr_q[FIFO_AW-1:0]+1], fifo_head});
            seg_left_q--; rd_ptr_q += 2;
          end
        end
        if (seg_left_q==0) state_d = S_FETCH;
      end
      S_ZRUN: if (!e_full) begin
        push_elem('0); seg_left_q--;
        if (seg_left_q==0) state_d = S_FETCH;
      end
      S_RVAL: if (fifo_count >= (ELEM_W/8)) begin
        // read value V then go output state
        if (ELEM_W==8) cur_val_q = fifo_head;
        else cur_val_q = {fifo_mem[rd_ptr_q[FIFO_AW-1:0]+1], fifo_head};
        rd_ptr_q += (ELEM_W/8);
        state_d = S_RLE;
      end
      S_RLE: if (!e_full) begin
        push_elem(cur_val_q); seg_left_q--;
        if (seg_left_q==0) state_d = S_FETCH;
      end
      S_DELTA_BASE: if (fifo_count >= (ELEM_W/8)) begin
        if (ELEM_W==8) cur_val_q = fifo_head;
        else cur_val_q = {fifo_mem[rd_ptr_q[FIFO_AW-1:0]+1], fifo_head};
        rd_ptr_q += (ELEM_W/8);
        delta_nib_sel_q = 1'b0;
        state_d = S_DELTA_CODES;
      end
      S_DELTA_CODES: begin
        if (seg_left_q==0) state_d = S_FETCH;
        else if (!fifo_empty && !e_full) begin
          if (!delta_nib_sel_q) begin
            delta_byte_q = fifo_head;
            rd_ptr_q++;
            delta_nib_sel_q = 1'b1;
          end else begin
            // process low and high nibble in one go
            int n;
            for (n=0; n<2; n++) begin
              logic [3:0] nib = (n==0)?delta_byte_q[3:0]:delta_byte_q[7:4];
              logic signed [ELEM_W:0] tmp = cur_val_q;
              unique case (nib)
                4'h0: tmp = cur_val_q;
                4'h1: tmp = cur_val_q + 1;
                4'h2: tmp = cur_val_q - 1;
                4'h3: tmp = cur_val_q + 2;
                4'h4: tmp = cur_val_q - 2;
                4'h5: tmp = cur_val_q + 3;
                4'h6: tmp = cur_val_q - 3;
                4'hF: begin
                  // literal base next
                  if (fifo_count >= (ELEM_W/8)) begin
                    if (ELEM_W==8) tmp = fifo_head;
                    else tmp = {fifo_mem[rd_ptr_q[FIFO_AW-1:0]+1], fifo_head};
                    rd_ptr_q += (ELEM_W/8);
                  end
                end
                default: tmp = cur_val_q;
              endcase
              if (!e_full && seg_left_q!=0) begin
                push_elem(tmp[ELEM_W-1:0]);
                cur_val_q = tmp[ELEM_W-1:0];
                seg_left_q--;
              end
            end
            delta_nib_sel_q = 1'b0;
          end
        end
      end
      default: state_d = S_IDLE;
    endcase
  end

  // --------------------------------------------------------------------------
  // Output packer: combine OUT_ELEMS elements per beat
  // --------------------------------------------------------------------------
  logic [OUT_ELEMS*ELEM_W-1:0] pack_reg;
  logic [$clog2(OUT_ELEMS+1)-1:0] pack_count_q;
  logic pack_valid_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      pack_count_q <= '0;
      pack_valid_q <= 1'b0;
      out_valid_o  <= 1'b0;
      out_last_o   <= 1'b0;
    end else begin
      out_valid_o <= pack_valid_q;
      if (pack_valid_q && out_ready_i) begin
        pack_valid_q <= 1'b0;
        pack_count_q <= '0;
      end

      if (!e_empty && !pack_valid_q) begin
        // fill packer
        int k;
        for (k=0; k<OUT_ELEMS; k++) begin
          if (!e_empty) begin
            pack_reg[k*ELEM_W +: ELEM_W] = efifo_mem[e_rptr_q[EFIFO_AW-1:0]];
            e_rptr_q <= e_rptr_q + 1'b1;
            pack_count_q++;
          end
        end
        pack_valid_q <= 1'b1;
      end
    end
  end

  assign out_data_o        = pack_reg;
  assign out_last_elems_o  = pack_count_q;
  assign out_last_o        = stream_done_q && (e_empty && !fifo_empty);

  // --------------------------------------------------------------------------
  // Assertions
  // --------------------------------------------------------------------------
`ifdef ASSERT_ON
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    ELEM_W%8==0) else $error("rle_zd_decomp: ELEM_W must be multiple of 8");

  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(in_valid_i && !in_ready_o))
    else $warning("rle_zd_decomp: input backpressure violation");
`endif

endmodule
`default_nettype wire
