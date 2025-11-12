// ============================================================================
//  SRAM Bank Wrapper with Optional SECDED ECC, RAW Bypass, and MBIST Bypass
//  File: sram_bank_wrapper.sv
//  Description:
//    - Front-end: decoupled write/read streams (valid/ready), one beat per op.
//    - Back-end: 1RW SRAM macro (write OR read per cycle), 1-cycle read latency.
//    - Optional ECC per 64-bit slice (8 parity bits -> SECDED (72,64)).
//    - RAW (read-after-write) bypass returns freshly written data.
//    - MBIST bypass: connect macro pins to an external BIST in test mode.
//    - Generic behavioral RAM for sim if USE_GENERIC_RAM=1.
//
//  Notes:
//    - Addresses are byte-based at the front-end; internally word indices.
//    - For ECC enabled, full-beat writes required (no partial strobes).
//    - Ready/valid FIFOs (small) decouple the scheduler & the front-end.
//    - Synthesis-friendly; SVAs under `ASSERT_ON`.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module sram_bank_wrapper #(
  // ---------------- Geometry ----------------
  parameter int unsigned DATA_W   = 512,   // front-end data width (bits)
  parameter int unsigned DEPTH    = 4096,  // number of words (DATA_W each)
  parameter int unsigned ADDR_W   = $clog2(DEPTH) + $clog2(DATA_W/8), // byte addr width

  // ---------------- ECC ----------------
  // ECC_MODE: "NONE" or "SECDED64" (per 64-bit slice -> +8 bits per slice)
  parameter string       ECC_MODE = "SECDED64",
  // If ECC enabled, require full-beat writes (no partial strobes)
  parameter bit          REQUIRE_FULL_STROBE = 1'b1,

  // ---------------- Queues / Arbitration ----------------
  parameter int unsigned WR_Q_DEPTH = 2,
  parameter int unsigned RD_Q_DEPTH = 2,
  parameter string       ARB_POLICY = "RR", // "W_FIRST", "R_FIRST", "RR"

  // ---------------- Macro & simulation ----------------
  // One-cycle read latency macro (typical synchronous 1RW)
  parameter bit          USE_GENERIC_RAM = 1'b1,

  // ---------------- Test / MBIST ----------------
  parameter bit          HAS_MBIST = 1'b1
)(
  input  logic                     clk_i,
  input  logic                     rstn_i,       // synchronous, active-low

  // ---------------- Write front-end ----------------
  input  logic                     wr_cmd_valid_i,
  output logic                     wr_cmd_ready_o,
  input  logic [ADDR_W-1:0]        wr_addr_i,    // byte address
  input  logic                     wr_data_valid_i,
  output logic                     wr_data_ready_o,
  input  logic [DATA_W-1:0]        wr_data_i,
  input  logic [DATA_W/8-1:0]      wr_strb_i,    // 1 bit per byte

  // ---------------- Read front-end ----------------
  input  logic                     rd_cmd_valid_i,
  output logic                     rd_cmd_ready_o,
  input  logic [ADDR_W-1:0]        rd_addr_i,    // byte address

  output logic                     rd_data_valid_o,
  input  logic                     rd_data_ready_i,
  output logic [DATA_W-1:0]        rd_data_o,
  output logic                     rd_sbe_o,     // single-bit corrected (pulse)
  output logic                     rd_dbe_o,     // double-bit detected (pulse)

  output logic [31:0]              sbe_count_o,
  output logic [31:0]              dbe_count_o,

  // ---------------- Sleep/Test ----------------
  input  logic                     sleep_i,      // passed to macro if available

  // MBIST raw macro bypass (active only when HAS_MBIST=1 and mbist_mode_i=1)
  input  logic                     mbist_mode_i,
  input  logic                     mbist_csb_i,  // macro chip-select (active low)
  input  logic                     mbist_web_i,  // macro write-enable (active low = write)
  input  logic [$clog2(DEPTH)-1:0] mbist_addr_i, // word address
  input  logic [/*PHY_W*/1023:0]   mbist_din_i,  // widened in bind section
  input  logic [/*PHY_W/8*/127:0]  mbist_wmask_i,// widened in bind section
  output logic [/*PHY_W*/1023:0]   mbist_dout_o  // widened in bind section
);
  // ==========================================================================
  // Derived constants
  // ==========================================================================
  localparam int unsigned BEAT_BYTES = DATA_W/8;
  localparam int unsigned BEAT_LSB   = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);

  // ECC geometry
  localparam bit ECC_EN        = (ECC_MODE == "SECDED64");
  localparam int unsigned ECC_SLICE = 64;
  localparam int unsigned ECC_BITS_PER_SLICE = ECC_EN ? 8 : 0;
  localparam int unsigned NUM_SLICES         = ECC_EN ? (DATA_W / ECC_SLICE) : 0;
  localparam int unsigned ECC_W              = ECC_EN ? (NUM_SLICES * ECC_BITS_PER_SLICE) : 0;

  // Physical macro width = data + ecc, stored together
  localparam int unsigned PHY_W = DATA_W + ECC_W;
  localparam int unsigned PHY_BYTES = PHY_W/8;

  // Guard
  initial begin
    if (DATA_W % 8 != 0) $error("sram_bank_wrapper: DATA_W must be multiple of 8.");
    if (ECC_EN && ((DATA_W % ECC_SLICE) != 0))
      $error("sram_bank_wrapper: For SECDED64, DATA_W must be multiple of 64.");
    if (ADDR_W < (BEAT_LSB + $clog2(DEPTH)))
      $warning("sram_bank_wrapper: ADDR_W appears smaller than required; using lower bits only.");
  end

  // ==========================================================================
  // Front-end acceptance FIFOs (write + read commands)
  //   - Write: combine cmd+data into one entry when both valid in same cycle
  //   - Read : queue word index
  // ==========================================================================
  typedef struct packed {
    logic [$clog2(DEPTH)-1:0]  waddr;
    logic [DATA_W-1:0]         wdata;
    logic [DATA_W/8-1:0]       wstrb;
  } wr_entry_t;

  localparam int unsigned WR_AW = (WR_Q_DEPTH <= 2) ? 1 : $clog2(WR_Q_DEPTH);
  wr_entry_t wr_q_mem   [0:WR_Q_DEPTH-1];
  logic [WR_AW:0] wr_q_wptr_q, wr_q_rptr_q;
  wire  wr_q_full  = (wr_q_wptr_q[WR_AW]     != wr_q_rptr_q[WR_AW]) &&
                     (wr_q_wptr_q[WR_AW-1:0] == wr_q_rptr_q[WR_AW-1:0]);
  wire  wr_q_empty = (wr_q_wptr_q == wr_q_rptr_q);
  wr_entry_t wr_q_head;

  assign wr_q_head = wr_q_mem[wr_q_rptr_q[WR_AW-1:0]];

  // Combine cmd+data handshakes atomically
  wire wr_accept = wr_cmd_valid_i && wr_data_valid_i && !wr_q_full;

  // Addresses are byte-based; convert to word index
  wire [$clog2(DEPTH)-1:0] wr_word_addr = wr_addr_i[BEAT_LSB +: $clog2(DEPTH)];

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wr_q_wptr_q <= '0;
      wr_q_rptr_q <= '0;
    end else begin
      if (wr_accept) begin
        wr_q_mem[wr_q_wptr_q[WR_AW-1:0]].waddr <= wr_word_addr;
        wr_q_mem[wr_q_wptr_q[WR_AW-1:0]].wdata <= wr_data_i;
        wr_q_mem[wr_q_wptr_q[WR_AW-1:0]].wstrb <= wr_strb_i;
        wr_q_wptr_q <= wr_q_wptr_q + 1'b1;
      end
      // Pop happens in the scheduler when a write is issued to macro
      if (!wr_q_empty && /*pop condition set later*/ 1'b0) begin
        // placeholder; real pop below
      end
    end
  end

  assign wr_cmd_ready_o  = !wr_q_full && wr_data_valid_i;
  assign wr_data_ready_o = !wr_q_full && wr_cmd_valid_i;

  // ---------------- Read queue ----------------
  localparam int unsigned RD_AW = (RD_Q_DEPTH <= 2) ? 1 : $clog2(RD_Q_DEPTH);
  logic [$clog2(DEPTH)-1:0] rd_q_mem [0:RD_Q_DEPTH-1];
  logic [RD_AW:0] rd_q_wptr_q, rd_q_rptr_q;
  wire  rd_q_full  = (rd_q_wptr_q[RD_AW]     != rd_q_rptr_q[RD_AW]) &&
                     (rd_q_wptr_q[RD_AW-1:0] == rd_q_rptr_q[RD_AW-1:0]);
  wire  rd_q_empty = (rd_q_wptr_q == rd_q_rptr_q);
  wire [$clog2(DEPTH)-1:0] rd_q_head = rd_q_mem[rd_q_rptr_q[RD_AW-1:0]];

  assign rd_cmd_ready_o = !rd_q_full;

  wire rd_accept = rd_cmd_valid_i && rd_cmd_ready_o;
  wire [$clog2(DEPTH)-1:0] rd_word_addr = rd_addr_i[BEAT_LSB +: $clog2(DEPTH)];

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rd_q_wptr_q <= '0;
      rd_q_rptr_q <= '0;
    end else begin
      if (rd_accept) begin
        rd_q_mem[rd_q_wptr_q[RD_AW-1:0]] <= rd_word_addr;
        rd_q_wptr_q <= rd_q_wptr_q + 1'b1;
      end
      // Pop happens in the scheduler when a read is issued to macro
      if (!rd_q_empty && /*pop cond*/ 1'b0) begin
        // placeholder; real pop below
      end
    end
  end

  // ==========================================================================
  // ECC encode/decode (per 64-bit slice) — generic SECDED (72,64)
  // ==========================================================================
  function automatic bit is_pow2(input int unsigned x);
    return (x & (x-1)) == 0;
  endfunction

  // Encode 64b -> 8b {p_overall, p[6:0]}
  function automatic logic [7:0] ecc64_encode(input logic [63:0] d);
    logic [6:0] p;
    logic       overall;
    int unsigned cwpos; // 1..71 (data bits occupy non-power-of-two positions)
    int unsigned di;
    p = '0; overall = 1'b0; di = 0;
    for (cwpos = 1; cwpos <= 71; cwpos++) begin
      if (!is_pow2(cwpos)) begin
        logic bitd = d[di];
        overall ^= bitd;
        for (int i=0; i<7; i++) begin
          if ((cwpos >> i) & 1) p[i] ^= bitd;
        end
        di++;
      end
    end
    overall ^= ^p; // include parity bits for extended (SECDED)
    ecc64_encode = {overall, p};
  endfunction

  // Given 64b data and stored ecc, return corrected data, sbe, dbe
  function automatic void ecc64_check_correct(
    input  logic [63:0] d_in,
    input  logic [7:0]  ecc_in,
    output logic [63:0] d_out,
    output logic        sbe,
    output logic        dbe
  );
    logic [7:0] ecc_exp;
    logic [6:0] synd;
    logic       ov, single, dbl;
    int unsigned cwpos;
    int unsigned di;
    ecc_exp = ecc64_encode(d_in);
    synd    = ecc_in[6:0] ^ ecc_exp[6:0];
    ov      = ecc_in[7] ^ ecc_exp[7];

    // Classification per extended Hamming:
    // synd==0 && ov==0 -> no error
    // synd!=0 && ov==1 -> single-bit error at codeword position 'synd'
    // synd==0 && ov==1 -> single-bit error in overall parity bit (treat as SBE, no data fix)
    // synd!=0 && ov==0 -> double-bit error
    single  = (synd != 7'd0) && ov;
    dbl     = (synd != 7'd0) && !ov;

    d_out   = d_in;
    sbe     = 1'b0;
    dbe     = 1'b0;

    if (single) begin
      // Map codeword position -> data index (or parity)
      if (is_pow2(synd)) begin
        // flipped a parity bit only: data unchanged
        sbe = 1'b1;
      end else begin
        // find data index for this cw position
        di = 0;
        for (cwpos = 1; cwpos <= 71; cwpos++) begin
          if (!is_pow2(cwpos)) begin
            if (cwpos == synd) begin
              d_out[di] = ~d_in[di];
            end
            di++;
          end
        end
        sbe = 1'b1;
      end
    end else if (synd == 7'd0 && ov) begin
      // overall parity bit flipped; treat as SBE (no data correction)
      sbe = 1'b1;
    end else if (dbl) begin
      dbe = 1'b1;
    end
  endfunction

  // Encode a full DATA_W word into PHY_W (concatenate ECC slices high)
  function automatic logic [PHY_W-1:0] ecc_pack_write(input logic [DATA_W-1:0] d);
    logic [PHY_W-1:0] res;
    res = '0;
    if (!ECC_EN) begin
      res[DATA_W-1:0] = d;
    end else begin
      // per-slice ECC, store as: {ecc[N-1],...,ecc[0], data}
      res[DATA_W-1:0] = d;
      for (int s=0; s<NUM_SLICES; s++) begin
        res[DATA_W + s*8 +: 8] = ecc64_encode(d[s*64 +: 64]);
      end
    end
    return res;
  endfunction

  // Decode a full PHY_W read word -> DATA_W + flags
  function automatic void ecc_unpack_read(
    input  logic [PHY_W-1:0] phy_r,
    output logic [DATA_W-1:0] d_fix,
    output logic               sbe_any,
    output logic               dbe_any
  );
    d_fix   = '0;
    sbe_any = 1'b0;
    dbe_any = 1'b0;
    if (!ECC_EN) begin
      d_fix = phy_r[DATA_W-1:0];
    end else begin
      d_fix = phy_r[DATA_W-1:0];
      for (int s=0; s<NUM_SLICES; s++) begin
        logic [63:0] di  = phy_r[s*64 +: 64];
        logic [7:0]  ei  = phy_r[DATA_W + s*8 +: 8];
        logic [63:0] do_;
        logic sbe_, dbe_;
        ecc64_check_correct(di, ei, do_, sbe_, dbe_);
        d_fix[s*64 +: 64] = do_;
        sbe_any |= sbe_;
        dbe_any |= dbe_;
      end
    end
  endfunction

  // ==========================================================================
  // Macro (1RW) interface nets
  // ==========================================================================
  // 1RW macro pins (generic)
  logic                      mac_csb_n;     // active-low chip select
  logic                      mac_web_n;     // active-low write enable (0=write,1=read)
  logic [$clog2(DEPTH)-1:0]  mac_addr;
  logic [PHY_W-1:0]          mac_din;
  logic [PHY_W-1:0]          mac_dout;
  logic [PHY_BYTES-1:0]      mac_wmask;     // active-high byte-write mask (if supported)

  // Sleep hint (passed through to macro if applicable)
  logic                      mac_sleep;

  assign mac_sleep = sleep_i;

  // ==========================================================================
  // Simple scheduler for 1RW macro + RAW bypass
  //   - One operation issued per cycle (if any queue non-empty).
  //   - Read data returns the next cycle (modeled), with ECC fixup & flags.
  //   - RAW bypass: if previous issued op was WRITE to same address, feed write data.
  // ==========================================================================
  typedef enum logic [1:0] { OP_NONE, OP_WRITE, OP_READ } op_e;
  op_e last_op_q;

  // Arbiter
  logic take_write, take_read;

  // Pop strobes for queues
  logic wr_q_pop, rd_q_pop;

  // Default arb policy
  logic rr_sel_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) rr_sel_q <= 1'b0;
    else if (!wr_q_empty || !rd_q_empty) rr_sel_q <= ~rr_sel_q;
  end

  always_comb begin
    take_write = 1'b0;
    take_read  = 1'b0;
    unique case (ARB_POLICY)
      "W_FIRST": begin
        if (!wr_q_empty)       take_write = 1'b1;
        else if (!rd_q_empty)  take_read  = 1'b1;
      end
      "R_FIRST": begin
        if (!rd_q_empty)       take_read  = 1'b1;
        else if (!wr_q_empty)  take_write = 1'b1;
      end
      default: begin // "RR"
        if (!wr_q_empty && !rd_q_empty) begin
          take_write = rr_sel_q ? 1'b0 : 1'b1;
          take_read  = rr_sel_q ? 1'b1 : 1'b0;
        end else if (!wr_q_empty) take_write = 1'b1;
        else if (!rd_q_empty)     take_read  = 1'b1;
      end
    endcase
  end

  // RAW bypass bookkeeping (last issued write)
  logic                      last_wr_v_q;
  logic [$clog2(DEPTH)-1:0]  last_wr_addr_q;
  logic [DATA_W-1:0]         last_wr_data_q;

  // ECC write data precompute (for full-beat only)
  logic [PHY_W-1:0]          wr_phydata_head;
  always_comb begin
    wr_phydata_head = ecc_pack_write(wr_q_head.wdata);
  end

  // Macro command drive + queue pops
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      mac_csb_n     <= 1'b1;
      mac_web_n     <= 1'b1;
      mac_addr      <= '0;
      mac_din       <= '0;
      mac_wmask     <= {PHY_BYTES{1'b1}}; // all bytes enabled by default
      wr_q_rptr_q   <= '0;
      rd_q_rptr_q   <= '0;
      last_op_q     <= OP_NONE;
      last_wr_v_q   <= 1'b0;
      last_wr_addr_q<= '0;
      last_wr_data_q<= '0;
    end else begin
      // Default: idle macro (no op)
      mac_csb_n <= 1'b1;
      mac_web_n <= 1'b1;
      mac_wmask <= {PHY_BYTES{1'b1}}; // all bytes write-enabled when writing
      wr_q_pop  <= 1'b0;
      rd_q_pop  <= 1'b0;

      if (take_write) begin
        // Optional strobe enforcement when ECC enabled
`ifdef ASSERT_ON
        if (ECC_EN && REQUIRE_FULL_STROBE) begin
          if (wr_q_head.wstrb != {BEAT_BYTES{1'b1}}) begin
            $error("sram_bank_wrapper: Partial write with ECC enabled. Set REQUIRE_FULL_STROBE=0 and add RMW sequencer.");
          end
        end
`endif
        mac_csb_n <= 1'b0;
        mac_web_n <= 1'b0; // write
        mac_addr  <= wr_q_head.waddr;
        mac_din   <= wr_phydata_head;
        // If you later add RMW, build mac_wmask from wr_q_head.wstrb expanded including ECC bytes.
        mac_wmask <= {PHY_BYTES{1'b1}}; // write all bytes for now

        // Queue pop
        wr_q_rptr_q <= wr_q_rptr_q + 1'b1;
        wr_q_pop    <= 1'b1;

        // Bypass bookeeping
        last_op_q       <= OP_WRITE;
        last_wr_v_q     <= 1'b1;
        last_wr_addr_q  <= wr_q_head.waddr;
        last_wr_data_q  <= wr_q_head.wdata;
      end else if (take_read) begin
        mac_csb_n <= 1'b0;
        mac_web_n <= 1'b1; // read
        mac_addr  <= rd_q_head;
        // No DIN/WMASK on read

        rd_q_rptr_q <= rd_q_rptr_q + 1'b1;
        rd_q_pop    <= 1'b1;

        last_op_q   <= OP_READ;
        // do not change last_wr_v (keep for one-cycle RAW compare)
      end else begin
        last_op_q <= OP_NONE;
        // decay the bypass flag (valid only for immediate next cycle RAW hit)
        if (last_wr_v_q && last_op_q != OP_WRITE) begin
          // keep one cycle; a read after this will still match
          // (we clear after the read pipeline below)
        end
      end
    end
  end

  // ==========================================================================
  // Macro instance (generic behavioral RAM or placeholder for foundry macro)
  //   - Modeled as synchronous 1RW with 1-cycle read latency.
  // ==========================================================================
  logic [PHY_W-1:0] mac_dout_q;

  generate
    if (USE_GENERIC_RAM) begin : g_genram
      // Byte-addressable 1RW synchronous array with 1-cycle read
      (* ram_style = "block" *) logic [PHY_W-1:0] mem [0:DEPTH-1];

      always_ff @(posedge clk_i) begin
        // Write on web_n=0
        if (!mac_csb_n && !mac_web_n) begin
          // For now, write all bytes
          mem[mac_addr] <= mac_din;
        end
        // Read on web_n=1
        if (!mac_csb_n && mac_web_n) begin
          mac_dout_q <= mem[mac_addr];
        end
      end
      assign mac_dout = mac_dout_q;
    end else begin : g_macro
      // ---- Replace with your foundry macro instantiation ----
      // Example (pseudocode):
      // sram_1rw_macro #(.WIDTH(PHY_W), .DEPTH(DEPTH)) u_mac (
      //   .CLK  (clk_i),
      //   .CSB  (mac_csb_n),
      //   .WEB  (mac_web_n),
      //   .A    (mac_addr),
      //   .D    (mac_din),
      //   .Q    (mac_dout),
      //   .SLEEP(mac_sleep),
      //   .WMASK(mac_wmask)
      // );
      // -------------------------------------------------------
    end
  endgenerate

  // ==========================================================================
  // Read return path: ECC check, RAW bypass, and output handshake
  // ==========================================================================
  // Pipeline register for "this cycle was a read to addr X"
  logic                     rd_pipe_v_q;
  logic [$clog2(DEPTH)-1:0] rd_pipe_addr_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rd_pipe_v_q    <= 1'b0;
      rd_pipe_addr_q <= '0;
    end else begin
      rd_pipe_v_q    <= (take_read);  // read issued this cycle -> data next cycle
      rd_pipe_addr_q <= rd_q_head;
      // Invalidate last_wr_v_q after one comparison window
      if (last_wr_v_q && take_read) begin
        // we'll use it below for bypass; clear after
      end
    end
  end

  // ECC + bypass stage (data available this cycle when rd_pipe_v_q was high last cycle)
  logic [DATA_W-1:0] rd_data_fix;
  logic              sbe_pulse, dbe_pulse;
  always_comb begin
    sbe_pulse = 1'b0;
    dbe_pulse = 1'b0;
    rd_data_fix = '0;

    if (rd_pipe_v_q) begin
      // RAW bypass: if last op was WRITE to same word, prefer the freshly written data
      if (last_wr_v_q && (last_wr_addr_q == rd_pipe_addr_q)) begin
        rd_data_fix = last_wr_data_q;
        sbe_pulse   = 1'b0;
        dbe_pulse   = 1'b0;
      end else begin
        // ECC fixup (or straight pass-through)
        if (!ECC_EN) begin
          rd_data_fix = mac_dout[DATA_W-1:0];
        end else begin
          logic [DATA_W-1:0] tmp;
          logic sbe_, dbe_;
          ecc_unpack_read(mac_dout, tmp, sbe_, dbe_);
          rd_data_fix = tmp;
          sbe_pulse   = sbe_;
          dbe_pulse   = dbe_;
        end
      end
    end
  end

  // Output handshake (single-beat)
  logic rd_out_v_q;
  logic [DATA_W-1:0] rd_out_data_q;
  logic sbe_q, dbe_q;

  assign rd_data_valid_o = rd_out_v_q;
  assign rd_data_o       = rd_out_data_q;
  assign rd_sbe_o        = rd_out_v_q && sbe_q;
  assign rd_dbe_o        = rd_out_v_q && dbe_q;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rd_out_v_q    <= 1'b0;
      rd_out_data_q <= '0;
      sbe_q         <= 1'b0;
      dbe_q         <= 1'b0;
      last_wr_v_q   <= 1'b0;
    end else begin
      // Present read result when available
      if (rd_pipe_v_q) begin
        rd_out_v_q    <= 1'b1;
        rd_out_data_q <= rd_data_fix;
        sbe_q         <= sbe_pulse;
        dbe_q         <= dbe_pulse;
        // RAW window expires after we used (or not) the bypass
        last_wr_v_q   <= 1'b0;
      end
      // Consume when downstream ready
      if (rd_out_v_q && rd_data_ready_i) begin
        rd_out_v_q <= 1'b0;
      end
    end
  end

  // Sticky counters
  logic [31:0] sbe_cnt_q, dbe_cnt_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      sbe_cnt_q <= 32'd0; dbe_cnt_q <= 32'd0;
    end else begin
      if (rd_out_v_q && rd_data_ready_i && sbe_q) sbe_cnt_q <= sbe_cnt_q + 32'd1;
      if (rd_out_v_q && rd_data_ready_i && dbe_q) dbe_cnt_q <= dbe_cnt_q + 32'd1;
    end
  end
  assign sbe_count_o = sbe_cnt_q;
  assign dbe_count_o = dbe_cnt_q;

  // ==========================================================================
  // MBIST macro bypass (optional)
  //   When mbist_mode_i=1, disconnect scheduler and drive macro pins from MBIST.
  // ==========================================================================
  generate
    if (HAS_MBIST) begin : g_mbist
      // Width adapters: since mbist_* ports are declared wide to avoid zero-size,
      // slice them down to PHY_W (synthesizer will trim).
      wire [PHY_W-1:0]      mbist_din_w   = mbist_din_i[PHY_W-1:0];
      wire [PHY_BYTES-1:0]  mbist_wmask_w = mbist_wmask_i[PHY_BYTES-1:0];
      wire [PHY_W-1:0]      mbist_dout_w;

      // Macro muxing
      always_comb begin
        if (mbist_mode_i) begin
          // MBIST drives macro
          // (we keep scheduler idle because queues backpressure naturally)
          // You may also add explicit gating to wr_cmd_ready/rd_cmd_ready if needed.
          // Drive macro pins
          // Note: we still clock the macro; test env controls CSB/WEB.
          // The generic RAM model maps to mac_* nets already.
        end
      end

      // If you have a physical macro, wire mbist_* directly in the instance above.
      // For generic RAM model, we can't simultaneously drive mac_*; so MBIST mode here
      // simply exposes mac_dout via mbist_dout_o and expects the testbench to drive
      // the wrapper front-end for memory screening. For real silicon integration,
      // replace g_macro with your vendor macro and mux mac_* with mbist_* when
      // mbist_mode_i=1.

      assign mbist_dout_o = { {(1024-PHY_W){1'b0}}, mac_dout }; // zero-extend
    end
    else begin : g_no_mbist
      assign mbist_dout_o = '0;
    end
  endgenerate

  // ==========================================================================
  // Busy/Ready backpressure refinement (queue pops)
  //   We delayed pop logic to know when the macro issued a command.
  // ==========================================================================
  // We already advanced rptrs inside the scheduler on issue; so nothing further here.

  // Drive ready signals (computed above)
  // (wr_cmd_ready_o / wr_data_ready_o / rd_cmd_ready_o already assigned)

  // ==========================================================================
  // Assertions
  // ==========================================================================
`ifdef ASSERT_ON
  // No over-accept
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(wr_cmd_valid_i && wr_data_valid_i && wr_q_full))
    else $error("sram_bank_wrapper: Write accepted while write queue full");

  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(rd_cmd_valid_i && rd_q_full))
    else $error("sram_bank_wrapper: Read accepted while read queue full");

  // Address alignment at beat granularity
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    wr_cmd_valid_i && wr_cmd_ready_o |-> (wr_addr_i[BEAT_LSB-1:0] == '0))
    else $error("sram_bank_wrapper: wr_addr_i not beat-aligned");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rd_cmd_valid_i && rd_cmd_ready_o |-> (rd_addr_i[BEAT_LSB-1:0] == '0))
    else $error("sram_bank_wrapper: rd_addr_i not beat-aligned");

  // Full-beat write when ECC enabled and REQUIRE_FULL_STROBE
  if (ECC_EN && REQUIRE_FULL_STROBE) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      wr_cmd_valid_i && wr_cmd_ready_o |-> (wr_strb_i == {BEAT_BYTES{1'b1}}))
      else $error("sram_bank_wrapper: Partial write with ECC enabled.");
  end
`endif

endmodule

`default_nettype wire
