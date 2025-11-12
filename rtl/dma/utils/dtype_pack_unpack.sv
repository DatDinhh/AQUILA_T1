// ============================================================================
//  dtype_pack_unpack.sv
//  Generic dtype converter + packer/unpacker between element stream and bus.
//
//  Direction (compile-time):
//    DIR=0 : PACK   (elem -> bus)
//    DIR=1 : UNPACK (bus  -> elem)
//
//  Endianness (run-time):
//    cfg_big_endian_i = 0 : element 0 placed at low bits/bytes of bus (LSB-first)
//    cfg_big_endian_i = 1 : element 0 placed at high bits/bytes of bus (MSB-first)
//
//  Constraints:
//    - BUS_W % 8 == 0
//    - ELEM_B_W % 8 == 0   (bus element granularity must be byte-multiple for bus_keep)
//    - BUS_W % ELEM_B_W == 0
//
//  Frame semantics:
//    - PACK: never mixes two frames in the same bus beat. If the next element had
//      last=1 before filling a full beat, emits a partial beat using bus_keep.
//    - UNPACK: derives per-element 'last' from bus_last_i & bus_keep_i. Last element
//      within a last beat is marked last=1 on the final element emitted.
//
//  Telemetry (counters):
//    - beats_in / beats_out / elems_in / elems_out / partial_beats
//
//  © 2025 Aquila Project. MIT-style license.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module dtype_pack_unpack #(
  // ---------------- Direction ----------------
  // 0 = PACK (elem->bus), 1 = UNPACK (bus->elem)
  parameter int unsigned DIR = 0,

  // ---------------- Bus geometry ----------------
  parameter int unsigned BUS_W    = 64,  // bus beat width (bits)
  parameter int unsigned ELEM_B_W = 8,   // element width as it appears on the bus

  // ---------------- Element-side geometry ----------------
  parameter int unsigned ELEM_E_W = 16,  // element width on element side

  // ---------------- Signedness ----------------
  parameter bit          ELEM_B_SIGNED = 1'b0, // bus element interpretation
  parameter bit          ELEM_E_SIGNED = 1'b1, // element-side interpretation

  // ---------------- Buffering ----------------
  // Internal element ring capacity (in bus elements). Must be >= ELS_PER_BEAT.
  parameter int unsigned BUF_ELEMS = (BUS_W / ELEM_B_W) * 4
)(
  input  logic                          clk_i,
  input  logic                          rstn_i,          // synchronous, active-low

  // =================== Element-side stream ===================
  //  Used as INPUT when DIR=0 (PACK). Used as OUTPUT when DIR=1 (UNPACK).
  input  logic                          elem_in_valid_i,
  output logic                          elem_in_ready_o,
  input  logic [ELEM_E_W-1:0]           elem_in_data_i,
  input  logic                          elem_in_last_i,

  output logic                          elem_out_valid_o,
  input  logic                          elem_out_ready_i,
  output logic [ELEM_E_W-1:0]           elem_out_data_o,
  output logic                          elem_out_last_o,

  // =================== Bus-side stream ======================
  //  Used as OUTPUT when DIR=0 (PACK). Used as INPUT when DIR=1 (UNPACK).
  input  logic                          bus_in_valid_i,
  output logic                          bus_in_ready_o,
  input  logic [BUS_W-1:0]              bus_in_data_i,
  input  logic [(BUS_W/8)-1:0]          bus_in_keep_i,   // byte-granular keep (AXI tkeep-style)
  input  logic                          bus_in_last_i,

  output logic                          bus_out_valid_o,
  input  logic                          bus_out_ready_i,
  output logic [BUS_W-1:0]              bus_out_data_o,
  output logic [(BUS_W/8)-1:0]          bus_out_keep_o,
  output logic                          bus_out_last_o,

  // =================== Run-time control =====================
  input  logic                          cfg_big_endian_i, // 0: little (LSB-first), 1: big (MSB-first)

  // =================== Telemetry ============================
  input  logic                          cmd_clr_stats_i,
  output logic [31:0]                   stat_beats_in_o,
  output logic [31:0]                   stat_beats_out_o,
  output logic [31:0]                   stat_elems_in_o,
  output logic [31:0]                   stat_elems_out_o,
  output logic [31:0]                   stat_partial_beats_o // #bus beats with keep != full
);

  // ==========================================================================
  // Static checks
  // ==========================================================================
  localparam int unsigned BYTES_PER_BUS     = BUS_W / 8;
  localparam int unsigned BYTES_PER_ELEM_B  = ELEM_B_W / 8;
  localparam int unsigned ELS_PER_BEAT      = (ELEM_B_W == 0) ? 0 : (BUS_W / ELEM_B_W);

  initial begin
    if (BUS_W == 0)             $error("dtype_pack_unpack: BUS_W must be > 0.");
    if (ELEM_B_W == 0)          $error("dtype_pack_unpack: ELEM_B_W must be > 0.");
    if (ELEM_E_W == 0)          $error("dtype_pack_unpack: ELEM_E_W must be > 0.");
    if ((BUS_W % 8) != 0)       $error("dtype_pack_unpack: BUS_W must be a multiple of 8.");
    if ((ELEM_B_W % 8) != 0)    $error("dtype_pack_unpack: ELEM_B_W must be a multiple of 8.");
    if ((BUS_W % ELEM_B_W) != 0)$error("dtype_pack_unpack: BUS_W must be a multiple of ELEM_B_W.");
    if (BUF_ELEMS < ELS_PER_BEAT)
      $error("dtype_pack_unpack: BUF_ELEMS must be >= ELS_PER_BEAT.");
  end

  // ==========================================================================
  // Helpers: dtype conversion (saturating) between ELEM_E_W <-> ELEM_B_W
  // ==========================================================================
  localparam int unsigned MAXW_E2B = (ELEM_E_W > ELEM_B_W ? ELEM_E_W : ELEM_B_W) + 1;
  localparam int unsigned MAXW_B2E = (ELEM_B_W > ELEM_E_W ? ELEM_B_W : ELEM_E_W) + 1;

  // Saturate an extended signed value to a signed/unsigned destination width.
  function automatic logic [ELEM_B_W-1:0]
    conv_e2b (input logic [ELEM_E_W-1:0] xin);
    logic signed [MAXW_E2B-1:0] xext;
    logic signed [MAXW_E2B-1:0] smin, smax, y;
    // Extend input according to ELEM_E_SIGNED
    if (ELEM_E_SIGNED) xext = {{(MAXW_E2B-ELEM_E_W){xin[ELEM_E_W-1]}}, xin};
    else               xext = {{(MAXW_E2B-ELEM_E_W){1'b0}}, xin};

    // Destination range depends on ELEM_B_SIGNED
    if (ELEM_B_SIGNED) begin
      smin = - (MAXW_E2B' (1) <<< (ELEM_B_W-1));
      smax =   (MAXW_E2B' (1) <<< (ELEM_B_W-1)) - MAXW_E2B'(1);
    end else begin
      smin = '0;
      smax = (MAXW_E2B' (1) <<< ELEM_B_W) - MAXW_E2B'(1);
    end

    if (xext < smin)      y = smin;
    else if (xext > smax) y = smax;
    else                  y = xext;

    return y[ELEM_B_W-1:0];
  endfunction

  function automatic logic [ELEM_E_W-1:0]
    conv_b2e (input logic [ELEM_B_W-1:0] xin);
    logic signed [MAXW_B2E-1:0] xext;
    logic signed [MAXW_B2E-1:0] smin, smax, y;
    // Extend from bus side interpretation
    if (ELEM_B_SIGNED) xext = {{(MAXW_B2E-ELEM_B_W){xin[ELEM_B_W-1]}}, xin};
    else               xext = {{(MAXW_B2E-ELEM_B_W){1'b0}}, xin};

    // Destination range depends on ELEM_E_SIGNED
    if (ELEM_E_SIGNED) begin
      smin = - (MAXW_B2E' (1) <<< (ELEM_E_W-1));
      smax =   (MAXW_B2E' (1) <<< (ELEM_E_W-1)) - MAXW_B2E'(1);
    end else begin
      smin = '0;
      smax = (MAXW_B2E' (1) <<< ELEM_E_W) - MAXW_B2E'(1);
    end

    if (xext < smin)      y = smin;
    else if (xext > smax) y = smax;
    else                  y = xext;

    return y[ELEM_E_W-1:0];
  endfunction

  // ==========================================================================
  // Internal element ring buffer (stores *bus elements* + per-element 'last')
  //   Used in PACK to accumulate until a bus beat is ready.
  //   Used in UNPACK to stage parsed elements for consumption.
  // ==========================================================================
  localparam int AW = (BUF_ELEMS <= 2) ? 1 : $clog2(BUF_ELEMS);

  logic [ELEM_B_W-1:0] ring_data [0:BUF_ELEMS-1];
  logic                ring_last [0:BUF_ELEMS-1];

  logic [AW:0] wr_ptr_q, rd_ptr_q; // extra bit for full/empty decode
  function automatic logic ring_empty(input logic [AW:0] w, input logic [AW:0] r);
    return (w == r);
  endfunction
  function automatic logic ring_full (input logic [AW:0] w, input logic [AW:0] r);
    return (w[AW] != r[AW]) && (w[AW-1:0] == r[AW-1:0]);
  endfunction
  function automatic logic [AW:0] ring_inc(input logic [AW:0] p);
    return p + {{AW{1'b0}},1'b1};
  endfunction

  // Live occupancy (combinational)
  function automatic int unsigned ring_count(input logic [AW:0] w, input logic [AW:0] r);
    int unsigned c;
    if (w[AW] == r[AW]) c = (w[AW-1:0] >= r[AW-1:0]) ? (w[AW-1:0] - r[AW-1:0])
                                                     : (BUF_ELEMS - (r[AW-1:0] - w[AW-1:0]));
    else                c = (BUF_ELEMS - r[AW-1:0]) + w[AW-1:0];
    return c;
  endfunction

  // ==========================================================================
  // PACK direction (elem -> bus)
  // ==========================================================================
  generate if (DIR == 0) begin : g_pack
    // Upstream element acceptance is disabled once we have a pending frame end (last)
    // so that two frames never cohabit a single bus beat.
    logic             last_pending_q;          // we have seen a last in the ring
    int unsigned      dist_to_last_q;          // #elements in ring *before* the last (0-based distance)

    // Forming logic
    logic                     have_full_beat;
    logic                     can_emit_partial_last;
    int unsigned              n_emit;          // #elements to emit on this beat (<=ELS_PER_BEAT)
    logic                     emit_last;       // bus_last for this beat
    logic [BUS_W-1:0]         beat_data_w;
    logic [(BUS_W/8)-1:0]     beat_keep_w;

    // ---------------- Element ingress (push into ring) ----------------
    wire elem_fire = elem_in_valid_i && elem_in_ready_o;
    assign elem_in_ready_o = !ring_full(wr_ptr_q, rd_ptr_q) && !last_pending_q;

    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        wr_ptr_q        <= '0;
        last_pending_q  <= 1'b0;
        dist_to_last_q  <= '0;
      end else begin
        if (elem_fire) begin
          // Convert to bus element and push
          ring_data[wr_ptr_q[AW-1:0]] <= conv_e2b(elem_in_data_i);
          ring_last[wr_ptr_q[AW-1:0]] <= elem_in_last_i;
          wr_ptr_q <= ring_inc(wr_ptr_q);

          // Record first pending last
          if (elem_in_last_i) begin
            last_pending_q <= 1'b1;
            dist_to_last_q <= ring_count(ring_inc(wr_ptr_q), rd_ptr_q) - 1; // elements before 'last'
          end
        end

        // Clear last_pending after we emit a beat that contains it (below)
        if (bus_out_valid_o && bus_out_ready_i && emit_last) begin
          last_pending_q <= 1'b0;
          dist_to_last_q <= '0;
        end
      end
    end

    // ---------------- Decide when a beat can be emitted ----------------
    always_comb begin
      int unsigned count = ring_count(wr_ptr_q, rd_ptr_q);
      have_full_beat      = (count >= ELS_PER_BEAT);
      can_emit_partial_last = last_pending_q && (count == (dist_to_last_q + 1));
      if (have_full_beat) begin
        n_emit    = ELS_PER_BEAT;
        emit_last = last_pending_q && (dist_to_last_q < ELS_PER_BEAT);
      end else if (can_emit_partial_last) begin
        n_emit    = dist_to_last_q + 1; // flush tail with last
        emit_last = 1'b1;
      end else begin
        n_emit    = 0;
        emit_last = 1'b0;
      end
    end

    // ---------------- Assemble beat data & keep ----------------
    always_comb begin
      beat_data_w = '0;
      beat_keep_w = '0;

      // Keep mask: contiguous bytes set either from LSB or MSB depending on endianness
      int unsigned valid_bytes = n_emit * BYTES_PER_ELEM_B;
      if (valid_bytes != 0) begin
        if (!cfg_big_endian_i) begin
          beat_keep_w = ((BYTES_PER_BUS' (1) << valid_bytes) - BYTES_PER_BUS'(1));
        end else begin
          beat_keep_w = ~(((BYTES_PER_BUS' (1) << (BYTES_PER_BUS - valid_bytes)) - BYTES_PER_BUS'(1)));
        end
      end

      // Populate bus payload with n_emit elements from ring head
      for (int i = 0; i < ELS_PER_BEAT; i++) begin
        logic [ELEM_B_W-1:0] d = '0;
        if (i < n_emit) begin
          int unsigned idx = (rd_ptr_q[AW-1:0] + i) % BUF_ELEMS;
          d = ring_data[idx];
        end
        int bitpos = (!cfg_big_endian_i)
                    ? (i * ELEM_B_W)
                    : (BUS_W - (i+1)*ELEM_B_W);
        beat_data_w[bitpos +: ELEM_B_W] = d;
      end
    end

    // ---------------- Bus egress (pop from ring) ----------------
    assign bus_out_valid_o = (n_emit != 0);
    assign bus_out_data_o  = beat_data_w;
    assign bus_out_keep_o  = beat_keep_w;
    assign bus_out_last_o  = emit_last;

    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        rd_ptr_q <= '0;
      end else if (bus_out_valid_o && bus_out_ready_i) begin
        // Pop n_emit elements
        for (int i=0; i<n_emit; i++) begin
          rd_ptr_q <= ring_inc(rd_ptr_q);
        end
        // If we didn't include last, update distance
        if (last_pending_q && !emit_last) begin
          dist_to_last_q <= dist_to_last_q - n_emit;
        end
      end
    end

    // ---------------- Tie off unused ports (directional) ----------------
    assign bus_in_ready_o   = 1'b0;
    assign elem_out_valid_o = 1'b0;
    assign elem_out_data_o  = '0;
    assign elem_out_last_o  = 1'b0;

    // ---------------- Telemetry ----------------
    logic [31:0] beats_in_q, beats_out_q, elems_in_q, elems_out_q, partial_q;
    wire         beat_out_fire = bus_out_valid_o && bus_out_ready_i;
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        beats_in_q  <= 32'd0;
        beats_out_q <= 32'd0;
        elems_in_q  <= 32'd0;
        elems_out_q <= 32'd0;
        partial_q   <= 32'd0;
      end else begin
        if (cmd_clr_stats_i) begin
          beats_in_q  <= 32'd0;
          beats_out_q <= 32'd0;
          elems_in_q  <= 32'd0;
          elems_out_q <= 32'd0;
          partial_q   <= 32'd0;
        end else begin
          if (elem_fire) elems_in_q <= elems_in_q + 32'd1;
          if (beat_out_fire) begin
            beats_out_q <= beats_out_q + 32'd1;
            elems_out_q <= elems_out_q + n_emit;
            if (bus_out_keep_o != {BYTES_PER_BUS{1'b1}})
              partial_q <= partial_q + 32'd1;
          end
        end
      end
    end

    assign stat_beats_in_o   = beats_in_q;   // not used in PACK
    assign stat_beats_out_o  = beats_out_q;
    assign stat_elems_in_o   = elems_in_q;
    assign stat_elems_out_o  = elems_out_q;
    assign stat_partial_beats_o = partial_q;

    // ---------------- Assertions ----------------
`ifdef ASSERT_ON
    // No mixed frames within a beat
    // (We enforce by stalling elem_in while last_pending)
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      last_pending_q |-> !elem_in_ready_o)
      else $error("dtype_pack_unpack(PACK): accepted elements while last pending.");

    // Bus inputs should be idle in PACK mode
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      !bus_in_valid_i) else $warning("PACK mode: bus_in_valid_i asserted (ignored).");
`endif

  end else begin : g_unpack
  // ==========================================================================
  // UNPACK direction (bus -> elem)
  // ==========================================================================
    // Ingress: accept a bus beat when there is enough room to store all *valid*
    // elements of this beat (derived from keep).
    function automatic int unsigned popcount_keep(input logic [(BUS_W/8)-1:0] k);
      int unsigned c; c=0;
      for (int i=0; i<BYTES_PER_BUS; i++) if (k[i]) c++;
      return c;
    endfunction

    // Determine #valid elements implied by keep mask
    function automatic int unsigned elems_from_keep(
      input logic [(BUS_W/8)-1:0] k,
      input logic big_end
    );
      // We expect contiguous ones then zeros, starting from LSB (little) or MSB (big).
      int unsigned vbytes = popcount_keep(k);
      if (!big_end) return vbytes / BYTES_PER_ELEM_B;
      else          return vbytes / BYTES_PER_ELEM_B;
    endfunction

    // Keep contiguity check (warn if malformed)
    function automatic logic is_contiguous_keep(
      input logic [(BUS_W/8)-1:0] k,
      input logic big_end
    );
      logic [(BUS_W/8)-1:0] ref;
      int unsigned vbytes = popcount_keep(k);
      if (!big_end) ref = (vbytes==0) ? '0 : ((BYTES_PER_BUS' (1) << vbytes) - BYTES_PER_BUS'(1));
      else begin
        ref = ~(((BYTES_PER_BUS' (1) << (BYTES_PER_BUS - vbytes)) - BYTES_PER_BUS'(1)));
      end
      return (k == ref);
    endfunction

    // Available slots in ring
    function automatic int unsigned ring_space(
      input logic [AW:0] w, input logic [AW:0] r
    );
      return BUF_ELEMS - ring_count(w,r);
    endfunction

    // Bus ingress -> push each valid element into ring (converted) with last flags
    wire bus_fire = bus_in_valid_i && bus_in_ready_o;
    // Ready when enough space for all valid elements in the beat
    logic [(BUS_W/8)-1:0] keep_cur;
    assign keep_cur = bus_in_keep_i;

    // Elements valid in this beat
    logic [15:0] v_elems_w;
    always_comb v_elems_w = elems_from_keep(keep_cur, cfg_big_endian_i);

    assign bus_in_ready_o =
      (ring_space(wr_ptr_q, rd_ptr_q) >= v_elems_w) &&
      // Disallow zero-length beats unless it's a last marker (defensive)
      ((v_elems_w != 0) || bus_in_last_i);

    // Push on accept
    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        wr_ptr_q <= '0;
      end else if (bus_fire) begin
        // Walk elements in bus beat
        for (int i=0; i<v_elems_w; i++) begin
          int bitpos = (!cfg_big_endian_i)
                        ? (i * ELEM_B_W)
                        : (BUS_W - (i+1)*ELEM_B_W);
          logic [ELEM_B_W-1:0] braw = bus_in_data_i[bitpos +: ELEM_B_W];
          ring_data[wr_ptr_q[AW-1:0]] <= conv_b2e(braw); // store as element-side width? No — ring stores bus elems!
          // NOTE: The ring stores *bus element width* consistently. We convert to
          // element width at pop time for symmetry with PACK path.
          // For UNPACK, store 'last' on the final valid element if bus_last_i is set.
          ring_last[wr_ptr_q[AW-1:0]] <= (bus_in_last_i && (i == (v_elems_w-1)));
          wr_ptr_q <= ring_inc(wr_ptr_q);
        end
      end
    end

    // Element egress -> pop one element per cycle
    assign elem_out_valid_o = !ring_empty(wr_ptr_q, rd_ptr_q);

    logic [ELEM_B_W-1:0]  raw_from_ring;
    assign raw_from_ring = ring_data[rd_ptr_q[AW-1:0]];
    assign elem_out_data_o = conv_b2e(raw_from_ring);
    assign elem_out_last_o = ring_last[rd_ptr_q[AW-1:0]];

    wire elem_fire = elem_out_valid_o && elem_out_ready_i;

    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        rd_ptr_q <= '0;
      end else if (elem_fire) begin
        rd_ptr_q <= ring_inc(rd_ptr_q);
      end
    end

    // ---------------- Tie off unused ports (directional) ----------------
    assign elem_in_ready_o  = 1'b0;
    assign bus_out_valid_o  = 1'b0;
    assign bus_out_data_o   = '0;
    assign bus_out_keep_o   = '0;
    assign bus_out_last_o   = 1'b0;

    // ---------------- Telemetry ----------------
    logic [31:0] beats_in_q, beats_out_q, elems_in_q, elems_out_q, partial_q;
    wire         elem_out_fire = elem_out_valid_o && elem_out_ready_i;

    always_ff @(posedge clk_i or negedge rstn_i) begin
      if (!rstn_i) begin
        beats_in_q  <= 32'd0;
        beats_out_q <= 32'd0;
        elems_in_q  <= 32'd0;
        elems_out_q <= 32'd0;
        partial_q   <= 32'd0;
      end else begin
        if (cmd_clr_stats_i) begin
          beats_in_q  <= 32'd0;
          beats_out_q <= 32'd0;
          elems_in_q  <= 32'd0;
          elems_out_q <= 32'd0;
          partial_q   <= 32'd0;
        end else begin
          if (bus_fire) begin
            beats_in_q <= beats_in_q + 32'd1;
            elems_in_q <= elems_in_q + v_elems_w;
            if (bus_in_keep_i != {BYTES_PER_BUS{1'b1}})
              partial_q <= partial_q + 32'd1;
          end
          if (elem_out_fire) begin
            elems_out_q <= elems_out_q + 32'd1;
          end
        end
      end
    end

    assign stat_beats_in_o      = beats_in_q;
    assign stat_beats_out_o     = beats_out_q; // not used in UNPACK
    assign stat_elems_in_o      = elems_in_q;
    assign stat_elems_out_o     = elems_out_q;
    assign stat_partial_beats_o = partial_q;

    // ---------------- Assertions ----------------
`ifdef ASSERT_ON
    // Keep contiguity guard on every accepted beat
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      bus_fire |-> is_contiguous_keep(bus_in_keep_i, cfg_big_endian_i))
      else $warning("dtype_pack_unpack(UNPACK): non-contiguous keep observed (accepted anyway).");

    // Element inputs must be idle in UNPACK mode
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      !elem_in_valid_i) else $warning("UNPACK mode: elem_in_valid_i asserted (ignored).");
`endif

  end endgenerate

endmodule

`default_nettype wire
