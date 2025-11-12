// ============================================================================
//  pe_cell.sv
//  Integer PE Cell: VEC-wide MAC per step, accumulate across K, emit C on tile end
//
//  Upstream (from pe_lane_int.sv):
//    - step_valid_i / step_ready_o
//    - a_vec_i   [VEC*ACT_W-1:0]
//    - w_vec_i   [VEC*WGT_W-1:0]
//    - mul_en_mask_i [VEC-1:0]  (1=en, 0=skip)
//    - step_last_i                (1 on final step of current tile)
//    - tile_start_i / acc_clear_i (pulse: clear accumulator at tile start)
//
//  Downstream (C-stream out):
//    - c_valid_o / c_ready_i
//    - c_data_o  [OUT_W-1:0]
//    - c_last_o  (1 with the only C beat per tile)
//
//  Notes:
//    - Integer path with saturating accumulate (signed/unsigned selectable).
//    - Quantization is "scale-by-power-of-two": arithmetic >> cfg_shift_i,
//      with optional rounding-to-nearest (symmetric).
//    - If you need affine output (scale * x + bias_out) or per-PE scale, add
//      a lightweight post-op here; wiring is already clean.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module pe_cell #(
  // Geometry
  parameter int unsigned VEC         = 16,  // parallel MACs per cycle
  parameter int unsigned ACT_W       = 8,   // activation element width
  parameter int unsigned WGT_W       = 8,   // weight element width
  parameter int unsigned ACC_W       = 32,  // accumulator width
  parameter int unsigned OUT_W       = 16,  // output width (post-quant/clip)

  // Signedness
  parameter bit          ACT_SIGNED  = 1'b1,
  parameter bit          WGT_SIGNED  = 1'b1,
  parameter bit          OUT_SIGNED  = 1'b1, // affects clamp endpoints

  // Behavior
  parameter bit          SATURATE    = 1'b1, // 1: saturating add; 0: wrap
  parameter bit          ZERO_SKIP   = 1'b0  // 1: treat all-zero/masked dot as a "skip" event (telemetry only)
)(
  input  logic                          clk_i,
  input  logic                          rstn_i,           // synchronous, active-low

  // ---------------- Step interface (from lane) ----------------
  input  logic                          step_valid_i,
  output logic                          step_ready_o,
  input  logic [VEC*ACT_W-1:0]          a_vec_i,
  input  logic [VEC*WGT_W-1:0]          w_vec_i,
  input  logic [VEC-1:0]                mul_en_mask_i,
  input  logic                          step_last_i,      // 1 on final step in tile

  // Tile control
  input  logic                          tile_start_i,     // pulse: start of tile (initialize accumulator)
  input  logic                          acc_clear_i,      // pulse: explicit accumulator clear (optional)
  input  logic                          stall_i,          // level: force backpressure upstream (optional, OR into ready)

  // ---------------- Quantize / post-op config ----------------
  input  logic                          cfg_bias_en_i,    // accumulator bias at tile start
  input  logic signed   [ACC_W-1:0]     cfg_bias_i,
  input  logic [5:0]                    cfg_shift_i,      // arithmetic right-shift (0..63)
  input  logic                          cfg_shift_rnd_i,  // 1: symmetric round-to-nearest, 0: truncate
  input  logic                          cfg_relu_en_i,    // 1: ReLU after shift (signed outputs only)
  input  logic signed   [OUT_W-1:0]     cfg_clip_min_i,   // clamp min (signed/unsigned aware)
  input  logic signed   [OUT_W-1:0]     cfg_clip_max_i,   // clamp max

  // ---------------- C stream out ----------------
  output logic                          c_valid_o,
  input  logic                           c_ready_i,
  output logic [OUT_W-1:0]              c_data_o,
  output logic                          c_last_o,

  // ---------------- Telemetry -------------------
  output logic [31:0]                   stat_steps_o,      // steps accepted
  output logic [31:0]                   stat_zero_steps_o, // dot==0 or all masked
  output logic [31:0]                   stat_saturations_o,// accum saturations (events)
  output logic signed [ACC_W-1:0]       dbg_acc_q_o        // live accumulator (for debug)
);

  // ---------------------------------------------------------------------------
  // Derived widths & helpers
  // ---------------------------------------------------------------------------
  localparam int unsigned MUL_W = ACT_W + WGT_W + 1;             // product width (signed-safe)
  localparam int unsigned SUM_W = MUL_W + ((VEC<=1)?1:$clog2(VEC)); // dot sum headroom

  function automatic int unsigned umin(input int unsigned a, input int unsigned b);
    return (a<b)?a:b;
  endfunction

  // ---------------------------------------------------------------------------
  // Unpack inputs
  // ---------------------------------------------------------------------------
  logic signed [ACT_W-1:0] a_e [VEC];
  logic signed [WGT_W-1:0] w_e [VEC];

  for (genvar i=0; i<VEC; i++) begin : g_unpack
    // Cast to signed vectors internally; if unsigned, MSB remains 0 at runtime.
    assign a_e[i] = ACT_SIGNED ? $signed(a_vec_i[i*ACT_W +: ACT_W])
                               : $signed({1'b0, a_vec_i[i*ACT_W +: ACT_W-1]});
    assign w_e[i] = WGT_SIGNED ? $signed(w_vec_i[i*WGT_W +: WGT_W])
                               : $signed({1'b0, w_vec_i[i*WGT_W +: WGT_W-1]});
  end

  // ---------------------------------------------------------------------------
  // Vector dot product (masked)
  // ---------------------------------------------------------------------------
  logic signed [MUL_W-1:0] prod [VEC];

  for (genvar i2=0; i2<VEC; i2++) begin : g_mul
    wire signed [MUL_W-1:0] p = $signed(a_e[i2]) * $signed(w_e[i2]);
    assign prod[i2] = mul_en_mask_i[i2] ? p : '0;
  end

  // Reduce to a single sum (tool will build a tree)
  logic signed [SUM_W-1:0] dot_sum_w;
  always_comb begin
    dot_sum_w = '0;
    for (int k=0; k<VEC; k++) begin
      dot_sum_w = dot_sum_w + {{(SUM_W-MUL_W){prod[k][MUL_W-1]}}, prod[k]};
    end
  end

  // Zero-step detection for telemetry (all masked OR arithmetic zero)
  wire zero_step_w = (dot_sum_w == '0);

  // ---------------------------------------------------------------------------
  // Accumulator with optional saturating add
  // ---------------------------------------------------------------------------
  logic signed [ACC_W-1:0] acc_q, acc_d;
  logic                    acc_sat_event;

  // Signed saturating add: acc + dot_sum_w
  function automatic logic signed [ACC_W-1:0]
    sat_add(input logic signed [ACC_W-1:0] acc,
            input logic signed [SUM_W-1:0] inc,
            output logic                   did_saturate);
    // Extend to ACC_W+1 to see overflow on add
    logic signed [ACC_W:0] acc_ext, inc_ext, sum_ext;
    acc_ext = {{1{acc[ACC_W-1]}}, acc};
    inc_ext = {{(ACC_W+1-SUM_W){inc[SUM_W-1]}}, inc};
    sum_ext = acc_ext + inc_ext;

    // Detect signed overflow
    logic ovf_pos = (acc_ext[ACC_W] == 0) && (inc_ext[ACC_W] == 0) && (sum_ext[ACC_W] == 1);
    logic ovf_neg = (acc_ext[ACC_W] == 1) && (inc_ext[ACC_W] == 1) && (sum_ext[ACC_W] == 0);

    if (SATURATE && (ovf_pos || ovf_neg)) begin
      did_saturate = 1'b1;
      if (ovf_pos) sat_add = {1'b0, {ACC_W-1{1'b1}}}; //  2^(ACC_W-1)-1
      else          sat_add = {1'b1, {ACC_W-1{1'b0}}}; // -2^(ACC_W-1)
    end else begin
      did_saturate = 1'b0;
      sat_add      = sum_ext[ACC_W-1:0]; // wrap or exact
    end
  endfunction

  // Accumulator update on step fire
  // Fire definition and step ready:
  //  - Normally always ready, except if the output skid is full AND this step
  //    would finish the tile (to avoid losing the result).
  logic out_full_q;
  wire  block_this_step = out_full_q && step_valid_i && step_last_i;

  assign step_ready_o = ~stall_i & ~block_this_step;

  wire step_fire = step_valid_i & step_ready_o;

  // Reset behavior for accumulator
  wire clear_acc = tile_start_i | acc_clear_i;

  // Next accumulator
  always_comb begin
    if (clear_acc) begin
      acc_d         = cfg_bias_en_i ? cfg_bias_i : '0;
      acc_sat_event = 1'b0;
    end else if (step_fire) begin
      acc_d = sat_add(acc_q, dot_sum_w, acc_sat_event);
    end else begin
      acc_d         = acc_q;
      acc_sat_event = 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // Output: quantize/clip on tile end, 1-entry skid to decouple consumer
  // ---------------------------------------------------------------------------
  logic               out_v_q, out_v_d;
  logic [OUT_W-1:0]   out_d_q, out_d_d;
  logic               out_last_q, out_last_d;

  // Quantization helpers
  function automatic logic signed [OUT_W-1:0]
    post_quant_clip (
      input logic signed [ACC_W-1:0] x,
      input logic [5:0]              sh,
      input logic                    rnd,
      input logic                    relu_en,
      input logic signed [OUT_W-1:0] cmin,
      input logic signed [OUT_W-1:0] cmax
    );
    logic signed [ACC_W-1:0] xrnd;
    if (sh == 6'd0) begin
      xrnd = x;
    end else if (rnd) begin
      // Symmetric round-to-nearest: add +2^(sh-1) for x>=0; add -2^(sh-1) for x<0
      logic [ACC_W-1:0] half = {{(ACC_W-1){1'b0}}, 1'b1} << (sh-1);
      xrnd = x + (x[ACC_W-1] ? -$signed(half) : $signed(half));
    end else begin
      xrnd = x;
    end
    // Arithmetic shift
    logic signed [ACC_W-1:0] xsh = (sh == 0) ? xrnd : (xrnd >>> sh);

    // ReLU (only meaningful for signed)
    logic signed [ACC_W-1:0] xact;
    if (relu_en && OUT_SIGNED) xact = xsh[ACC_W-1] ? '0 : xsh;
    else                       xact = xsh;

    // Clip to OUT_W range (signed/unsigned aware)
    logic signed [OUT_W-1:0] y_clip;
    // Saturation to bounds
    logic signed [OUT_W-1:0] y_cast = xact[OUT_W-1:0];
    if (OUT_SIGNED) begin
      // compute min/max for OUT_W signed
      logic signed [OUT_W-1:0] smin = cmin;
      logic signed [OUT_W-1:0] smax = cmax;
      if (y_cast < smin)      y_clip = smin;
      else if (y_cast > smax) y_clip = smax;
      else                    y_clip = y_cast;
    end else begin
      // Treat as unsigned: negative becomes 0, and clamp to max
      logic [OUT_W-1:0] umin = (cmin[OUT_W-1]) ? '0 : logic'(cmin);
      logic [OUT_W-1:0] umax = logic'(cmax);
      logic [OUT_W-1:0] u    = xact[ACC_W-1] ? '0 : logic'(y_cast);
      if (u < umin)      y_clip = logic'(umin);
      else if (u > umax) y_clip = logic'(umax);
      else               y_clip = logic'(u);
    end
    return y_clip;
  endfunction

  // Form the "current result" that should be emitted if this step ends the tile.
  // We must use the *updated* accumulator value (acc_d) on the firing step.
  wire step_finishes_tile = step_fire & step_last_i;
  wire [OUT_W-1:0] quantized_now =
      post_quant_clip(acc_d, cfg_shift_i, cfg_shift_rnd_i,
                      cfg_relu_en_i, cfg_clip_min_i, cfg_clip_max_i);

  // Skid buffer logic
  // Load on tile finish; hold until consumed.
  always_comb begin
    out_v_d    = out_v_q;
    out_d_d    = out_d_q;
    out_last_d = out_last_q;

    // Produce a new C beat at tile end
    if (step_finishes_tile) begin
      out_v_d    = 1'b1;
      out_d_d    = quantized_now;
      out_last_d = 1'b1;
    end

    // Pop on handshake
    if (out_v_q && c_ready_i) begin
      out_v_d    = 1'b0;
      out_last_d = 1'b0;
    end
  end

  // Output full detection for backpressure into step_ready_o
  assign c_valid_o = out_v_q;
  assign c_data_o  = out_d_q;
  assign c_last_o  = out_last_q;

  // ---------------------------------------------------------------------------
  // Telemetry & state
  // ---------------------------------------------------------------------------
  logic [31:0] steps_q, steps_d;
  logic [31:0] zeros_q, zeros_d;
  logic [31:0] sats_q,  sats_d;

  always_comb begin
    steps_d = steps_q + (step_fire ? 32'd1 : 32'd0);
    zeros_d = zeros_q + ((ZERO_SKIP && step_fire && zero_step_w) ? 32'd1 : 32'd0);
    sats_d  = sats_q  + ((step_fire && acc_sat_event) ? 32'd1 : 32'd0);
  end

  // ---------------------------------------------------------------------------
  // Sequential updates
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      acc_q      <= '0;
      out_v_q    <= 1'b0;
      out_d_q    <= '0;
      out_last_q <= 1'b0;
      steps_q    <= 32'd0;
      zeros_q    <= 32'd0;
      sats_q     <= 32'd0;
      out_full_q <= 1'b0;
    end else begin
      acc_q      <= acc_d;
      out_v_q    <= out_v_d;
      out_d_q    <= out_d_d;
      out_last_q <= out_last_d;
      steps_q    <= steps_d;
      zeros_q    <= zeros_d;
      sats_q     <= sats_d;
      out_full_q <= (out_v_d && !c_ready_i); // next-cycle "full" view for gating
    end
  end

  // Expose telemetry
  assign stat_steps_o       = steps_q;
  assign stat_zero_steps_o  = zeros_q;
  assign stat_saturations_o = sats_q;
  assign dbg_acc_q_o        = acc_q;

  // ---------------------------------------------------------------------------
  // Assertions (simulation only)
  // ---------------------------------------------------------------------------
`ifdef ASSERT_ON
  // When we would generate a C beat, ensure we don't overflow the 1-entry skid.
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (step_valid_i && step_last_i && step_ready_o) |-> (!out_v_q || c_ready_i))
    else $error("pe_cell: output skid overflow; consumer must accept previous C before consecutive tile finishes.");

  // If ReLU enabled and outputs are unsigned, negatives must not appear
  if (OUT_SIGNED == 1'b0 && cfg_relu_en_i) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      c_valid_o |-> (c_data_o[OUT_W-1] == 1'b0))
      else $error("pe_cell: unsigned output produced negative under ReLU.");
  end
`endif

endmodule

`default_nettype wire
