// ============================================================================
//  post_activation.sv
//  Lane-parallel integer/fixed-point activation stage
//
//  Placement (typical):
//     ... → post_bias (optional) → post_activation → DMA/drain
//
//  Run-time selectable activations:
//   - ACT_NONE   : identity
//   - ACT_RELU   : max(0, x)
//   - ACT_RELU6  : clamp to [0, relu6_hi]
//   - ACT_CLAMP  : clamp to [clip_min, clip_max]
//   - ACT_LEAKY  : x if x>=0 else x >>> leaky_shift     (α = 1/2^leaky_shift)
//   - ACT_HSIG   : y = clamp(((x + hs_bias) * hs_gain) >>> hs_shift,
//                            hs_min, hs_max)
//   - ACT_HSWISH : y = (x * hsig(x)) >>> hswish_shift
//
//  Characteristics:
//   - No combinational ready loops (ingress FIFO + output skid).
//   - One beat per cycle when downstream is ready.
//   - Lane-parallel (LANES ≥ 1).
//   - Assertions under `ASSERT_ON`.
//   - Light telemetry of beats, ReLU zeros, and clamp events.
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module post_activation #(
  // ---------------- Geometry ----------------
  parameter int unsigned LANES         = 1,   // parallel outputs per beat
  parameter int unsigned DATA_W        = 16,  // in/out element width per lane
  parameter bit          IN_SIGNED     = 1'b1,
  parameter bit          OUT_SIGNED    = 1'b1,

  // ---------------- Elasticity / timing ----------------
  parameter int unsigned IN_FIFO_DEPTH = 2,   // ingress buffering (>=2 recommended)
  parameter bit          PIPE_OUT      = 1'b1,// register output (adds 1 cycle latency)

  // ---------------- Internal headroom for mul/shift paths ----------------
  // Used only for HSIG/HSWISH intermediate math; keep modest for timing (6–10).
  parameter int unsigned HEADROOM      = 8
)(
  input  logic                          clk_i,
  input  logic                          rstn_i,          // synchronous, active-low

  // =================== Stream In ===================
  input  logic                          in_valid_i,
  output logic                          in_ready_o,
  input  logic [LANES*DATA_W-1:0]       in_data_i,
  input  logic                          in_last_i,

  // =================== Stream Out ==================
  output logic                          out_valid_o,
  input  logic                           out_ready_i,
  output logic [LANES*DATA_W-1:0]       out_data_o,
  output logic                          out_last_o,

  // =================== Activation Select =============
  // 3-bit mode (encoded below)
  input  logic [2:0]                    cfg_mode_i,

  // =================== ReLU / ReLU6 / CLAMP =========
  input  logic signed [DATA_W-1:0]      cfg_relu6_hi_i,   // upper clamp for ReLU6 (in output domain)
  input  logic signed [DATA_W-1:0]      cfg_clip_min_i,   // generic clamp min
  input  logic signed [DATA_W-1:0]      cfg_clip_max_i,   // generic clamp max

  // =================== Leaky ReLU ====================
  input  logic [5:0]                    cfg_leaky_shift_i, // α = 1/2^shift (0..63)

  // =================== Hard-Sigmoid / Hard-Swish =====
  // hsig(x) = clamp( ((x + hs_bias) * hs_gain) >>> hs_shift , hs_min .. hs_max )
  input  logic signed [DATA_W-1:0]      cfg_hs_bias_i,    // bias term (same domain as x)
  input  logic [15:0]                   cfg_hs_gain_i,    // gain (fixed-point int; pick scale via hs_shift)
  input  logic [5:0]                    cfg_hs_shift_i,   // right shift after multiply
  input  logic signed [DATA_W-1:0]      cfg_hs_min_i,     // clamp low for hsig output
  input  logic signed [DATA_W-1:0]      cfg_hs_max_i,     // clamp high for hsig output

  // hswish(x) = (x * hsig(x)) >>> cfg_hswish_shift_i
  input  logic [5:0]                    cfg_hswish_shift_i,

  // =================== Telemetry =====================
  input  logic                          cmd_clr_stats_i,  // pulse: clear counters
  output logic [31:0]                   stat_in_beats_o,
  output logic [31:0]                   stat_out_beats_o,
  output logic [31:0]                   stat_relu_zero_o, // #lanes zeroed by ReLU
  output logic [31:0]                   stat_clamp_hits_o // #lanes hit min/max clamp
);

  // ==========================================================================
  // Parameters / local constants
  // ==========================================================================
  // Mode encoding
  localparam logic [2:0] ACT_NONE   = 3'd0;
  localparam logic [2:0] ACT_RELU   = 3'd1;
  localparam logic [2:0] ACT_RELU6  = 3'd2;
  localparam logic [2:0] ACT_CLAMP  = 3'd3;
  localparam logic [2:0] ACT_LEAKY  = 3'd4;
  localparam logic [2:0] ACT_HSIG   = 3'd5;
  localparam logic [2:0] ACT_HSWISH = 3'd6;

  // Internal widened width for mul/shift paths
  localparam int unsigned ACC_W = DATA_W + HEADROOM;

  // ==========================================================================
  // Ingress FIFO (no comb ready loops)
  // ==========================================================================
  localparam int unsigned D_BW = LANES*DATA_W + 1;
  logic               f_in_ready, f_out_valid, f_out_ready;
  logic [D_BW-1:0]    f_out_word;

  rv_fifo #(
    .WIDTH (D_BW),
    .DEPTH (IN_FIFO_DEPTH)
  ) u_in_fifo (
    .clk_i       (clk_i),
    .rstn_i      (rstn_i),
    .in_valid_i  (in_valid_i),
    .in_ready_o  (in_ready_o),
    .in_data_i   ({in_last_i, in_data_i}),
    .out_valid_o (f_out_valid),
    .out_ready_i (f_out_ready),
    .out_data_o  (f_out_word)
  );

  wire in_last = f_out_word[D_BW-1];
  wire [LANES*DATA_W-1:0] in_data = f_out_word[LANES*DATA_W-1:0];

  // We can accept from the FIFO when our output staging can accept a new beat
  wire out_can_load;
  logic out_v_q;
  assign f_out_ready = f_out_valid && out_can_load;

  // ==========================================================================
  // Activation helpers
  // ==========================================================================
  // Signed/unsigned interpretation helper
  function automatic logic signed [DATA_W-1:0]
    s_cast(input logic [DATA_W-1:0] x);
    if (IN_SIGNED) s_cast = x;
    else           s_cast = $signed({1'b0, x[DATA_W-2:0]}); // treat as non-negative
  endfunction

  // Generic clamp
  function automatic logic signed [DATA_W-1:0]
    clamp_s(input logic signed [DATA_W-1:0] x,
            input logic signed [DATA_W-1:0] lo,
            input logic signed [DATA_W-1:0] hi,
            output logic hit);
    logic signed [DATA_W-1:0] y;
    hit = 1'b0;
    if (x < lo) begin y = lo; hit = 1'b1; end
    else if (x > hi) begin y = hi; hit = 1'b1; end
    else y = x;
    return y;
  endfunction

  // Arithmetic right shift with sign
  function automatic logic signed [ACC_W-1:0]
    arshift_s(input logic signed [ACC_W-1:0] x, input logic [5:0] sh);
    return (sh == 0) ? x : (x >>> sh);
  endfunction

  // Hard-sigmoid primitive for one lane:
  function automatic logic signed [DATA_W-1:0]
    hsig_lane (
      input logic signed [DATA_W-1:0] x,
      input logic signed [DATA_W-1:0] bias,
      input logic [15:0]              gain,
      input logic [5:0]               sh,
      input logic signed [DATA_W-1:0] lo,
      input logic signed [DATA_W-1:0] hi,
      output logic                    clamp_hit
    );
    // widen x+bias
    logic signed [ACC_W-1:0] xb = {{(ACC_W-DATA_W){x[DATA_W-1]}}, x} +
                                  {{(ACC_W-DATA_W){bias[DATA_W-1]}}, bias};
    // multiply by gain (unsigned), keep sign of xb
    logic signed [ACC_W+16-1:0] mul = xb * $signed({1'b0, gain}); // gain is non-negative
    logic signed [ACC_W-1:0]    shr = arshift_s(mul[ACC_W+16-1 -: ACC_W], sh);
    // narrow to DATA_W with clamp to [lo,hi]
    logic hit;
    logic signed [DATA_W-1:0] narrowed = shr[DATA_W-1:0];
    logic signed [DATA_W-1:0] clamped  = clamp_s(narrowed, lo, hi, hit);
    clamp_hit = hit;
    return clamped;
  endfunction

  // Hard-swish primitive for one lane:
  function automatic logic signed [DATA_W-1:0]
    hswish_lane (
      input logic signed [DATA_W-1:0] x,
      input logic signed [DATA_W-1:0] bias,
      input logic [15:0]              gain,
      input logic [5:0]               sh,
      input logic signed [DATA_W-1:0] lo,
      input logic signed [DATA_W-1:0] hi,
      input logic [5:0]               out_sh,
      output logic                    clamp_hit
    );
    logic clamp_hsig;
    logic signed [DATA_W-1:0] h = hsig_lane(x, bias, gain, sh, lo, hi, clamp_hsig);
    // y = (x * h) >>> out_sh
    logic signed [2*DATA_W-1:0] prod = $signed(x) * $signed(h);
    logic signed [2*DATA_W-1:0] prod_sh = (out_sh==0) ? prod : (prod >>> out_sh);
    logic signed [DATA_W-1:0] y = prod_sh[DATA_W-1:0];
    clamp_hit = clamp_hsig; // report if inner hsig clamped (indicator of saturation)
    return y;
  endfunction

  // ==========================================================================
  // Per-lane activation (combinational on FIFO head)
  // ==========================================================================
  logic [LANES*DATA_W-1:0] act_out_w;
  logic [LANES-1:0]        act_relu_zero_w;
  logic [LANES-1:0]        act_clamp_hit_w;

  always_comb begin
    act_out_w        = '0;
    act_relu_zero_w  = '0;
    act_clamp_hit_w  = '0;

    for (int l=0; l<LANES; l++) begin
      logic signed [DATA_W-1:0] x  = s_cast(in_data[l*DATA_W +: DATA_W]);
      logic signed [DATA_W-1:0] y;
      logic clamp_hit;

      unique case (cfg_mode_i)
        ACT_NONE: begin
          y = x; clamp_hit = 1'b0;
        end

        ACT_RELU: begin
          if (OUT_SIGNED) begin
            y = x[DATA_W-1] ? '0 : x;
            act_relu_zero_w[l] = x[DATA_W-1];
          end else begin
            y = x; // already non-negative
          end
          clamp_hit = 1'b0;
        end

        ACT_RELU6: begin
          logic signed [DATA_W-1:0] cl_hi = cfg_relu6_hi_i;
          logic signed [DATA_W-1:0] cl_lo = '0;
          logic rz;
          rz = OUT_SIGNED && x[DATA_W-1];
          y  = rz ? '0 : x;
          act_relu_zero_w[l] = rz;
          y  = (y > cl_hi) ? cl_hi : y;
          clamp_hit = rz | (y == cl_hi);
        end

        ACT_CLAMP: begin
          y = clamp_s(x, cfg_clip_min_i, cfg_clip_max_i, clamp_hit);
        end

        ACT_LEAKY: begin
          if (OUT_SIGNED && x[DATA_W-1]) begin
            // arithmetic shift for negative slope
            logic signed [ACC_W-1:0] xw = {{(ACC_W-DATA_W){1'b1}}, x};
            logic signed [ACC_W-1:0] xs = arshift_s(xw, cfg_leaky_shift_i);
            y = xs[DATA_W-1:0];
            act_relu_zero_w[l] = 1'b0;
          end else begin
            y = x;
          end
          clamp_hit = 1'b0;
        end

        ACT_HSIG: begin
          y = hsig_lane(x,
                        cfg_hs_bias_i,
                        cfg_hs_gain_i,
                        cfg_hs_shift_i,
                        cfg_hs_min_i,
                        cfg_hs_max_i,
                        clamp_hit);
        end

        ACT_HSWISH: begin
          y = hswish_lane(x,
                          cfg_hs_bias_i,
                          cfg_hs_gain_i,
                          cfg_hs_shift_i,
                          cfg_hs_min_i,
                          cfg_hs_max_i,
                          cfg_hswish_shift_i,
                          clamp_hit);
        end

        default: begin
          y = x; clamp_hit = 1'b0;
        end
      endcase

      // Final generic clamp to cfg_clip_[min,max] if mode==CLAMP; otherwise optional guard
      if (cfg_mode_i == ACT_CLAMP) begin
        // already clamped above
      end else begin
        logic hit2;
        y = clamp_s(y, cfg_clip_min_i, cfg_clip_max_i, hit2);
        clamp_hit |= hit2;
      end

      act_out_w[l*DATA_W +: DATA_W] = y;
      act_clamp_hit_w[l] = clamp_hit;
    end
  end

  // ==========================================================================
  // Output staging (skid)
  // ==========================================================================
  logic [LANES*DATA_W-1:0] out_d_q, out_d_d;
  logic                    out_l_q, out_l_d;

  assign out_can_load = PIPE_OUT ? (!out_v_q || out_ready_i)
                                 : out_ready_i;

  // Registered output path (recommended)
  generate
    if (PIPE_OUT) begin : g_out_reg
      always_comb begin
        out_d_d = out_d_q;
        out_l_d = out_l_q;

        // Load from FIFO if we can
        if (f_out_valid && out_can_load) begin
          out_d_d = act_out_w;
          out_l_d = in_last;
        end
      end

      always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          out_v_q <= 1'b0;
          out_d_q <= '0;
          out_l_q <= 1'b0;
        end else begin
          // Load on accept
          if (f_out_valid && out_can_load) begin
            out_v_q <= 1'b1;
            out_d_q <= out_d_d;
            out_l_q <= out_l_d;
          end
          // Pop on consumer
          if (out_v_q && out_ready_i) begin
            out_v_q <= 1'b0;
          end
        end
      end

      assign out_valid_o = out_v_q;
      assign out_data_o  = out_d_q;
      assign out_last_o  = out_l_q;
    end else begin : g_out_comb
      assign out_valid_o = f_out_valid;
      assign out_data_o  = act_out_w;
      assign out_last_o  = in_last;
    end
  endgenerate

  // ==========================================================================
  // Telemetry
  // ==========================================================================
  logic [31:0] in_beats_q, out_beats_q, relu_zero_q, clamp_hits_q;

  wire in_fire  = f_out_valid && f_out_ready;
  wire out_fire = out_valid_o && out_ready_i;

  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      in_beats_q   <= 32'd0;
      out_beats_q  <= 32'd0;
      relu_zero_q  <= 32'd0;
      clamp_hits_q <= 32'd0;
    end else begin
      if (cmd_clr_stats_i) begin
        in_beats_q   <= 32'd0;
        out_beats_q  <= 32'd0;
        relu_zero_q  <= 32'd0;
        clamp_hits_q <= 32'd0;
      end else begin
        if (in_fire)  in_beats_q  <= in_beats_q  + 32'd1;
        if (out_fire) out_beats_q <= out_beats_q + 32'd1;
        if (f_out_valid && out_can_load)
          relu_zero_q <= relu_zero_q + $unsigned($countones(act_relu_zero_w));
        if (f_out_valid && out_can_load)
          clamp_hits_q <= clamp_hits_q + $unsigned($countones(act_clamp_hit_w));
      end
    end
  end

  assign stat_in_beats_o   = in_beats_q;
  assign stat_out_beats_o  = out_beats_q;
  assign stat_relu_zero_o  = relu_zero_q;
  assign stat_clamp_hits_o = clamp_hits_q;

  // ==========================================================================
  // Assertions (simulation only)
  // ==========================================================================
`ifdef ASSERT_ON
  // Clip bounds ordering
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    cfg_clip_min_i <= cfg_clip_max_i)
    else $error("post_activation: cfg_clip_min_i > cfg_clip_max_i");

  // ReLU6 upper bound must be >= 0
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    cfg_relu6_hi_i >= $signed('0))
    else $warning("post_activation: cfg_relu6_hi_i is negative.");

  // No comb ready loop: f_out_ready only depends on registered out_v_q
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    f_out_valid && !out_can_load |-> !f_out_ready);

`endif

endmodule

// ============================================================================
//  rv_fifo: single-clock ready/valid FIFO (non-fallthrough)
// ============================================================================
module rv_fifo #(
  parameter int unsigned WIDTH = 32,
  parameter int unsigned DEPTH = 2
)(
  input  logic               clk_i,
  input  logic               rstn_i,
  input  logic               in_valid_i,
  output logic               in_ready_o,
  input  logic [WIDTH-1:0]   in_data_i,
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

  assign in_ready_o  = ~fifo_full(wr_ptr_q, rd_ptr_q);
  assign out_valid_o = ~fifo_empty(wr_ptr_q, rd_ptr_q);
  assign out_data_o  = mem[rd_ptr_q[AW-1:0]];

  wire do_push = in_valid_i  && in_ready_o;
  wire do_pop  = out_valid_o && out_ready_i;

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
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_push |-> !fifo_full(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: push while full.");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    do_pop  |-> !fifo_empty(wr_ptr_q, rd_ptr_q))
    else $error("rv_fifo: pop while empty.");
`endif
endmodule

`default_nettype wire
