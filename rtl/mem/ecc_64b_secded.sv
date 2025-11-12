// ============================================================================
//  ecc_64b_secded.sv
//  64-bit Single-Error-Correct, Double-Error-Detect (SECDED) ECC
//  (72,64) extended Hamming: 64 data + 7 Hamming parity + 1 overall parity
//
//  Features
//  --------
//  - Combinational encoder: d_i -> p_enc_o[7:0] = {overall, p[6:0]}
//  - Decoder/corrector: (d_i, ecc_i) -> corrected d_o, SBE/DBE flags, syndrome
//  - Re-encode after correction: ecc_o (for scrub-back)
//  - Optional output pipelining (REGISTER_OUTPUTS)
//  - Fault injection by codeword position (1..71) or overall bit
//
//  Interface summary
//  -----------------
//  Inputs:
//    d_i         : [63:0] data word
//    ecc_i       : [7:0]  stored ECC bits on read ({overall, p[6:0]})
//    test_inject_en, test_inject_code_pos[6:0], test_inject_overall
//  Outputs:
//    p_enc_o     : [7:0]  freshly encoded ECC for d_i
//    d_o         : [63:0] corrected data
//    ecc_o       : [7:0]  ECC of corrected data (scrub target)
//    sbe_o       :        single-bit error (corrected or in parity only)
//    dbe_o       :        double-bit error (detected)
//    pbit_err_o  :        parity-bit-only error (data unchanged)
//    overall_err_o :      overall parity-only error (treated as SBE)
//    syndrome_o  : [6:0]  Hamming syndrome
//
//  Notes
//  -----
//  - Codeword positions 1..71: parity at powers-of-two; overall parity is separate (ecc[7]).
//  - Fault injection uses codeword positions; powers-of-two flip a parity bit;
//    non-powers-of-two flip the corresponding data bit.
//  - Fully synthesizable; for-loops are static and tool-friendly.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module ecc_64b_secded #(
  parameter bit REGISTER_OUTPUTS = 1'b0  // 1: register outputs on clk_i, 0: purely combinational
)(
  input  logic         clk_i,
  input  logic         rstn_i,       // only used when REGISTER_OUTPUTS=1

  // Data & ECC inputs
  input  logic [63:0]  d_i,
  input  logic [7:0]   ecc_i,        // {overall, p[6:0]} from memory (read path)

  // Fault injection (single-bit) for bring-up
  input  logic         test_inject_en,
  input  logic [6:0]   test_inject_code_pos,  // 1..71; if power-of-two -> parity flip; else data flip
  input  logic         test_inject_overall,   // flip overall parity bit (ecc[7])

  // Encoder output (fresh ECC for d_i)
  output logic [7:0]   p_enc_o,

  // Decoder/Corrector outputs
  output logic [63:0]  d_o,           // corrected data
  output logic [7:0]   ecc_o,         // ECC for d_o (post-correction)
  output logic         sbe_o,         // single-bit error (corrected or parity-only)
  output logic         dbe_o,         // double-bit error (detected)
  output logic         pbit_err_o,    // single parity bit wrong (no data change)
  output logic         overall_err_o, // overall parity bit wrong (no data change)
  output logic [6:0]   syndrome_o     // Hamming syndrome (non-zero on error)
);

  // ---------------- Utility: power-of-two test ----------------
  function automatic bit is_pow2(input int unsigned x);
    return (x >= 1) && ((x & (x-1)) == 0);
  endfunction

  // ---------------------------------------------------------------------------
  // Encoder (64b -> 8b) : compute p[6:0] over codeword positions and overall
  // ---------------------------------------------------------------------------
  function automatic logic [7:0] ecc64_encode(input logic [63:0] d);
    logic [6:0] p;          // Hamming parities p[0]..p[6]
    logic       overall;    // overall parity (across data + p)
    int unsigned cwpos;     // codeword position 1..71
    int unsigned di;        // data index 0..63

    p       = '0;
    overall = 1'b0;
    di      = 0;

    // Iterate codeword positions; data occupy non-pow2 positions
    for (cwpos = 1; cwpos <= 71; cwpos++) begin
      if (!is_pow2(cwpos)) begin
        logic bitd = d[di];
        overall ^= bitd;
        // For each Hamming parity bit position i, include bit if cwpos has bit i set
        for (int i = 0; i < 7; i++) begin
          if ((cwpos >> i) & 1) p[i] ^= bitd;
        end
        di++;
      end
    end
    // extended parity includes the parity bits themselves
    overall ^= ^p;

    ecc64_encode = {overall, p}; // [7]=overall, [6:0]=p
  endfunction

  // Combinational encoder output for d_i
  wire [7:0] p_enc = ecc64_encode(d_i);

  // ---------------------------------------------------------------------------
  // Fault injection (single-bit) on the incoming codeword for decoder
  // ---------------------------------------------------------------------------
  // Build injected versions of d/ecc to decode
  function automatic int unsigned codepos_to_data_index(input int unsigned cwpos);
    // Returns 0..63 for data positions,  -1 for parity positions
    int unsigned di; int unsigned i;
    if (is_pow2(cwpos)) return -1; // parity position
    di = 0;
    for (i = 1; i <= 71; i++) begin
      if (!is_pow2(i)) begin
        if (i == cwpos) return di;
        di++;
      end
    end
    return -1; // should not hit
  endfunction

  logic [63:0] d_inj;
  logic [7:0]  ecc_inj;

  always_comb begin
    d_inj   = d_i;
    ecc_inj = ecc_i;
    if (test_inject_en) begin
      int unsigned cp = test_inject_code_pos;
      if (cp >= 1 && cp <= 71) begin
        if (is_pow2(cp)) begin
          // parity location (2^i) -> flip p[i]
          int i = 0;
          for (i=0; i<7; i++) begin
            if (cp == (1<<i)) ecc_inj[i] = ~ecc_inj[i];
          end
        end else begin
          // data location -> flip the mapped data bit
          int di = codepos_to_data_index(cp);
          if (di >= 0 && di < 64) d_inj[di] = ~d_inj[di];
        end
      end
      if (test_inject_overall) begin
        ecc_inj[7] = ~ecc_inj[7];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Decoder/Corrector
  // ---------------------------------------------------------------------------
  function automatic void ecc64_check_correct(
    input  logic [63:0] d_in,
    input  logic [7:0]  ecc_in,
    output logic [63:0] d_fix,
    output logic        sbe,
    output logic        dbe,
    output logic        pbit_only,
    output logic        overall_only,
    output logic [6:0]  synd
  );
    logic [7:0] enc;   // expected ECC for incoming data
    logic [6:0] syn;   // syndrome = ecc_in[6:0] ^ enc[6:0]
    logic       ov;    // overall parity mismatch (ecc_in[7] ^ enc[7])
    logic       sbe_hit, dbe_hit;

    enc = ecc64_encode(d_in);
    syn = ecc_in[6:0] ^ enc[6:0];
    ov  = ecc_in[7]   ^ enc[7];

    // Classification (extended Hamming):
    // syn==0 && ov==0 -> no error
    // syn!=0 && ov==1 -> single-bit error at codeword position 'syn'
    // syn==0 && ov==1 -> overall parity bit error
    // syn!=0 && ov==0 -> double-bit error
    sbe_hit = (syn != 7'd0) && ov;
    dbe_hit = (syn != 7'd0) && !ov;

    d_fix        = d_in;
    sbe          = 1'b0;
    dbe          = 1'b0;
    pbit_only    = 1'b0;
    overall_only = 1'b0;
    synd         = syn;

    if (sbe_hit) begin
      // single-bit at codeword position 'syn'
      if (is_pow2(syn)) begin
        // Parity bit flipped; data unchanged
        sbe       = 1'b1;
        pbit_only = 1'b1;
      end else begin
        // Correct the data bit at the mapped data index
        int di = codepos_to_data_index(syn);
        if (di >= 0 && di < 64) d_fix[di] = ~d_in[di];
        sbe = 1'b1;
      end
    end
    else if (syn == 7'd0 && ov) begin
      // overall parity only
      sbe          = 1'b1;
      overall_only = 1'b1;
      // data unchanged
    end
    else if (dbe_hit) begin
      dbe = 1'b1;
      // no correction (uncorrectable)
    end
    // else no error
  endfunction

  // Combinational decode/correct of injected inputs
  logic [63:0] d_fix_c;
  logic        sbe_c, dbe_c, pbit_only_c, overall_only_c;
  logic [6:0]  synd_c;

  always_comb begin
    ecc64_check_correct(d_inj, ecc_inj, d_fix_c, sbe_c, dbe_c, pbit_only_c, overall_only_c, synd_c);
  end

  // Re-encode corrected data for scrub-back
  wire [7:0] ecc_reenc_c = ecc64_encode(d_fix_c);

  // ---------------------------------------------------------------------------
  // Optional output pipelining
  // ---------------------------------------------------------------------------
  generate
    if (REGISTER_OUTPUTS) begin : g_regs
      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          p_enc_o        <= 8'h00;
          d_o            <= 64'h0;
          ecc_o          <= 8'h00;
          sbe_o          <= 1'b0;
          dbe_o          <= 1'b0;
          pbit_err_o     <= 1'b0;
          overall_err_o  <= 1'b0;
          syndrome_o     <= 7'h00;
        end else begin
          p_enc_o        <= ecc64_encode(d_i);
          d_o            <= d_fix_c;
          ecc_o          <= ecc_reenc_c;
          sbe_o          <= sbe_c;
          dbe_o          <= dbe_c;
          pbit_err_o     <= pbit_only_c;
          overall_err_o  <= overall_only_c;
          syndrome_o     <= synd_c;
        end
      end
    end else begin : g_comb
      always_comb begin
        p_enc_o        = ecc64_encode(d_i);
        d_o            = d_fix_c;
        ecc_o          = ecc_reenc_c;
        sbe_o          = sbe_c;
        dbe_o          = dbe_c;
        pbit_err_o     = pbit_only_c;
        overall_err_o  = overall_only_c;
        syndrome_o     = synd_c;
      end
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Assertions (simulation only)
  // ---------------------------------------------------------------------------
`ifdef ASSERT_ON
  // Encode must be stable when d_i stable (combinational)
  // (Tool will prune as this is trivial in a pure comb path)

  // Decoder sanity: if no error injected and ecc_i==encode(d_i) then no flags and d_o==d_i
  logic [7:0] _enc_chk;
  always_comb _enc_chk = ecc64_encode(d_i);

  // We only check in the combinational flavor or at clock edge in registered
  if (!REGISTER_OUTPUTS) begin
    assert property ( (ecc_i == _enc_chk) && !test_inject_en && !test_inject_overall
                     |-> (sbe_o==0 && dbe_o==0 && d_o==d_i) );
  end
`endif

endmodule

`default_nettype wire
