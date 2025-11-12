// ============================================================================
//  AXI Master Core (Read + Write) with Burst Chopping & Outstanding Control
//  File: axi_mst_core.sv
//  Description:
//    - Streaming command API for reads/writes
//    - Automatically chops long transfers into legal AXI bursts
//      (<=256 beats, no 4KiB boundary crossing)
//    - Limits outstanding bursts per channel (credits)
//    - Fixed AXI ID per instance (in-order completion)
//    - Elastic FIFOs on R and W data paths
//
//  Read API:
//    rd_cmd_valid_i, rd_cmd_ready_o, rd_cmd_addr_i, rd_cmd_bytes_i
//    -> outputs: rd_data_valid_o/ready_i, rd_data_o, rd_last_o, rd_resp_o
//
//  Write API:
//    wr_cmd_valid_i, wr_cmd_ready_o, wr_cmd_addr_i, wr_cmd_bytes_i
//    wr_data_valid_i/ready_o, wr_data_i, wr_strb_i (optional)
//    -> outputs: wr_done_o (pulse), wr_error_o (status for last cmd)
//
//  Parameters let you set QoS/CACHE/PROT, FIFO depths, and outstanding limits.
//
//  Notes:
//    - STRICT_ALIGN=1 enforces beat-aligned addr and byte-count.
//      Turn off only if you add a data realigner.
//    - One command at a time per direction keeps control simple and robust.
//      Burst pipelining + credits still keep the bus busy.
//
//  © 2025. MIT-style license (adjust per repo policy).
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi_mst_core #(
  // ---------------- AXI widths ----------------
  parameter int unsigned AXI_ADDR_W = 48,
  parameter int unsigned AXI_DATA_W = 512,
  parameter int unsigned AXI_ID_W   = 6,
  parameter int unsigned AXI_USER_W = 1,

  // ---------------- Attributes (constant per instance) ----------------
  parameter logic [AXI_ID_W-1:0] AXI_FIXED_ID = 'h01,

  parameter logic [3:0] AXI_ARQOS   = 4'h4,
  parameter logic [3:0] AXI_ARCACHE = 4'b1111,
  parameter logic [2:0] AXI_ARPROT  = 3'b000,

  parameter logic [3:0] AXI_AWQOS   = 4'h4,
  parameter logic [3:0] AXI_AWCACHE = 4'b1111,
  parameter logic [2:0] AXI_AWPROT  = 3'b000,

  // ---------------- Behavior knobs ----------------
  parameter int unsigned MAX_RD_OUTS = 4,   // max outstanding AR bursts
  parameter int unsigned MAX_WR_OUTS = 4,   // max outstanding AW bursts
  parameter int unsigned RD_FIFO_DEPTH = 32, // R-beat elastic FIFO
  parameter int unsigned WR_FIFO_DEPTH = 32, // W-beat elastic FIFO
  parameter bit          STRICT_ALIGN  = 1'b1, // require aligned addr/bytes
  parameter bit          USE_WR_STRB_I = 1'b0   // if 0, drive all-ones WSTRB
)(
  // --------------------------------------------------------------------
  // Clock / Reset
  // --------------------------------------------------------------------
  input  logic                         clk_i,
  input  logic                         rstn_i,  // synchronous active-low

  // --------------------------------------------------------------------
  // AXI4 Master Interface
  // --------------------------------------------------------------------
  // Write address
  output logic [AXI_ID_W-1:0]          m_awid_o,
  output logic [AXI_ADDR_W-1:0]        m_awaddr_o,
  output logic [7:0]                   m_awlen_o,
  output logic [2:0]                   m_awsize_o,
  output logic [1:0]                   m_awburst_o,
  output logic                         m_awlock_o,
  output logic [3:0]                   m_awcache_o,
  output logic [2:0]                   m_awprot_o,
  output logic [3:0]                   m_awqos_o,
  output logic [AXI_USER_W-1:0]        m_awuser_o,
  output logic                         m_awvalid_o,
  input  logic                         m_awready_i,

  // Write data
  output logic [AXI_DATA_W-1:0]        m_wdata_o,
  output logic [AXI_DATA_W/8-1:0]      m_wstrb_o,
  output logic                         m_wlast_o,
  output logic [AXI_USER_W-1:0]        m_wuser_o,
  output logic                         m_wvalid_o,
  input  logic                         m_wready_i,

  // Write response
  input  logic [AXI_ID_W-1:0]          m_bid_i,
  input  logic [1:0]                   m_bresp_i,
  input  logic [AXI_USER_W-1:0]        m_buser_i,
  input  logic                         m_bvalid_i,
  output logic                         m_bready_o,

  // Read address
  output logic [AXI_ID_W-1:0]          m_arid_o,
  output logic [AXI_ADDR_W-1:0]        m_araddr_o,
  output logic [7:0]                   m_arlen_o,
  output logic [2:0]                   m_arsize_o,
  output logic [1:0]                   m_arburst_o,
  output logic                         m_arlock_o,
  output logic [3:0]                   m_arcache_o,
  output logic [2:0]                   m_arprot_o,
  output logic [3:0]                   m_arqos_o,
  output logic [AXI_USER_W-1:0]        m_aruser_o,
  output logic                         m_arvalid_o,
  input  logic                         m_arready_i,

  // Read data
  input  logic [AXI_ID_W-1:0]          m_rid_i,
  input  logic [AXI_DATA_W-1:0]        m_rdata_i,
  input  logic [1:0]                   m_rresp_i,
  input  logic                         m_rlast_i,
  input  logic [AXI_USER_W-1:0]        m_ruser_i,
  input  logic                         m_rvalid_i,
  output logic                         m_rready_o,

  // --------------------------------------------------------------------
  // Read Command + Output Stream
  // --------------------------------------------------------------------
  input  logic                         rd_cmd_valid_i,
  output logic                         rd_cmd_ready_o,
  input  logic [AXI_ADDR_W-1:0]        rd_cmd_addr_i,      // start addr
  input  logic [31:0]                  rd_cmd_bytes_i,     // total bytes

  output logic                         rd_data_valid_o,
  input  logic                         rd_data_ready_i,
  output logic [AXI_DATA_W-1:0]        rd_data_o,
  output logic [1:0]                   rd_resp_o,          // RRESP per beat
  output logic                         rd_last_o,          // last beat of command
  output logic                         rd_busy_o,
  output logic                         rd_error_o,         // sticky for the last command
  output logic                         rd_done_o,          // pulse when last beat leaves

  // --------------------------------------------------------------------
  // Write Command + Input Stream
  // --------------------------------------------------------------------
  input  logic                         wr_cmd_valid_i,
  output logic                         wr_cmd_ready_o,
  input  logic [AXI_ADDR_W-1:0]        wr_cmd_addr_i,
  input  logic [31:0]                  wr_cmd_bytes_i,

  input  logic                         wr_data_valid_i,
  output logic                         wr_data_ready_o,
  input  logic [AXI_DATA_W-1:0]        wr_data_i,
  input  logic [AXI_DATA_W/8-1:0]      wr_strb_i,          // ignored if USE_WR_STRB_I=0

  output logic                         wr_busy_o,
  output logic                         wr_done_o,          // pulse when all B responses seen
  output logic                         wr_error_o          // status for the last command
);

  // --------------------------------------------------------------------------
  // Local constants / helpers
  // --------------------------------------------------------------------------
  localparam int unsigned BEAT_BYTES = AXI_DATA_W/8;
  localparam int unsigned SZ_LOG2    = (BEAT_BYTES <= 1) ? 0 : $clog2(BEAT_BYTES);

  // static checks
  initial begin
    if (AXI_DATA_W % 8 != 0) $error("axi_mst_core: AXI_DATA_W must be multiple of 8.");
    if ((BEAT_BYTES & (BEAT_BYTES-1)) != 0) $error("axi_mst_core: BEAT_BYTES must be power of two.");
    if (MAX_RD_OUTS < 1) $error("axi_mst_core: MAX_RD_OUTS must be >=1");
    if (MAX_WR_OUTS < 1) $error("axi_mst_core: MAX_WR_OUTS must be >=1");
  end

  // AXI attribute constants
  assign m_arid_o    = AXI_FIXED_ID;
  assign m_arsize_o  = SZ_LOG2[2:0];
  assign m_arburst_o = 2'b01; // INCR
  assign m_arlock_o  = 1'b0;
  assign m_arcache_o = AXI_ARCACHE;
  assign m_arprot_o  = AXI_ARPROT;
  assign m_arqos_o   = AXI_ARQOS;
  assign m_aruser_o  = '0;

  assign m_awid_o    = AXI_FIXED_ID;
  assign m_awsize_o  = SZ_LOG2[2:0];
  assign m_awburst_o = 2'b01; // INCR
  assign m_awlock_o  = 1'b0;
  assign m_awcache_o = AXI_AWCACHE;
  assign m_awprot_o  = AXI_AWPROT;
  assign m_awqos_o   = AXI_AWQOS;
  assign m_awuser_o  = '0;

  assign m_wuser_o   = '0;
  assign m_bready_o  = 1'b1; // always ready to take B

  // WSTRB policy
  localparam logic [AXI_DATA_W/8-1:0] ALL_BYTES = { (AXI_DATA_W/8){1'b1} };
  wire [AXI_DATA_W/8-1:0] wstrb_mux = USE_WR_STRB_I ? wr_strb_i : ALL_BYTES;

  // burst length function (beats) with 4KiB cap and 256-beat cap
  function automatic [7:0] calc_blen(input logic [AXI_ADDR_W-1:0] addr,
                                     input logic [31:0] beats_left);
    int unsigned bytes_to_4k  = 4096 - (addr[11:0]);
    int unsigned beats_to_4k  = bytes_to_4k / BEAT_BYTES;
    int unsigned cap_256      = 256;
    int unsigned m1           = (beats_left < cap_256) ? beats_left : cap_256;
    int unsigned m2           = (beats_to_4k < m1) ? beats_to_4k : m1;
    calc_blen = (m2 == 0) ? 8'd1 : 8'(m2); // never return 0
  endfunction

  // =============================================================================
  // Read Path
  // =============================================================================
  // Command registers
  logic                    rd_active_q;
  logic [AXI_ADDR_W-1:0]   rd_addr_q;
  logic [31:0]             rd_bytes_q;
  logic [31:0]             rd_beats_total_q;
  logic [31:0]             rd_beats_issued_q;   // beats requested on AR (sum of bursts)
  logic [31:0]             rd_beats_emitted_q;  // beats popped to client
  logic [15:0]             rd_out_bursts_q;     // AR bursts in flight
  logic                    rd_err_flag_q;

  // AR issue registers
  logic [AXI_ADDR_W-1:0]   ar_addr_q, ar_addr_d;
  logic [7:0]              ar_len_q,  ar_len_d;
  logic                    ar_valid_q, ar_valid_d;

  // R FIFO
  localparam int unsigned RF_AW = (RD_FIFO_DEPTH <= 2) ? 1 : $clog2(RD_FIFO_DEPTH);
  logic [AXI_DATA_W-1:0]   rf_mem [0:RD_FIFO_DEPTH-1];
  logic [1:0]              rf_resp_mem [0:RD_FIFO_DEPTH-1];
  logic [RF_AW:0]          rf_wptr_q, rf_rptr_q;
  logic                    rf_full, rf_empty;
  logic                    rf_push, rf_pop;
  logic [AXI_DATA_W-1:0]   rf_rdata;
  logic [1:0]              rf_rresp;

  assign rf_full  = (rf_wptr_q[RF_AW]     != rf_rptr_q[RF_AW]) &&
                    (rf_wptr_q[RF_AW-1:0] == rf_rptr_q[RF_AW-1:0]);
  assign rf_empty = (rf_wptr_q == rf_rptr_q);
  assign rf_rdata = rf_mem[rf_rptr_q[RF_AW-1:0]];
  assign rf_rresp = rf_resp_mem[rf_rptr_q[RF_AW-1:0]];

  // R ready when FIFO has space
  assign m_rready_o = !rf_full;

  // Drive output stream from FIFO
  assign rd_data_valid_o = !rf_empty;
  assign rd_data_o       = rf_rdata;
  assign rd_resp_o       = rf_rresp;
  assign rd_last_o       = rd_data_valid_o &&
                           (rd_beats_emitted_q == (rd_beats_total_q - 32'd1));

  // rd_busy & done
  assign rd_busy_o = rd_active_q;
  assign rd_done_o = rd_data_valid_o && rd_data_ready_i && rd_last_o;
  assign rd_error_o = rd_err_flag_q;

  // R FIFO push on R handshake
  assign rf_push = m_rvalid_i && m_rready_o;

  // R FIFO pop on client handshake
  assign rf_pop  = rd_data_valid_o && rd_data_ready_i;

  // AR channel drive
  assign m_araddr_o  = ar_addr_q;
  assign m_arlen_o   = ar_len_q;
  assign m_arvalid_o = ar_valid_q;

  // Read command ready when idle
  assign rd_cmd_ready_o = ~rd_active_q;

  // ---------------- Sequential (read)
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      rd_active_q         <= 1'b0;
      rd_addr_q           <= '0;
      rd_bytes_q          <= '0;
      rd_beats_total_q    <= '0;
      rd_beats_issued_q   <= '0;
      rd_beats_emitted_q  <= '0;
      rd_out_bursts_q     <= '0;
      rd_err_flag_q       <= 1'b0;

      ar_addr_q           <= '0;
      ar_len_q            <= '0;
      ar_valid_q          <= 1'b0;

      rf_wptr_q           <= '0;
      rf_rptr_q           <= '0;
    end else begin
      // Default holds
      ar_addr_q  <= ar_addr_d;
      ar_len_q   <= ar_len_d;
      ar_valid_q <= ar_valid_d;

      // Accept new read command
      if (rd_cmd_valid_i && rd_cmd_ready_o) begin
        rd_active_q        <= 1'b1;
        rd_addr_q          <= rd_cmd_addr_i;
        rd_bytes_q         <= rd_cmd_bytes_i;
        rd_beats_total_q   <= (STRICT_ALIGN) ? (rd_cmd_bytes_i/BEAT_BYTES)
                                             : ((rd_cmd_bytes_i + BEAT_BYTES - 1) >> SZ_LOG2);
        rd_beats_issued_q  <= 32'd0;
        rd_beats_emitted_q <= 32'd0;
        rd_out_bursts_q    <= 16'd0;
        rd_err_flag_q      <= 1'b0;
      end

      // AR handshake: credit ++, update issued beats & next addr
      if (m_arvalid_o && m_arready_i) begin
        rd_out_bursts_q    <= rd_out_bursts_q + 16'd1;
        rd_beats_issued_q  <= rd_beats_issued_q + {24'd0, (ar_len_q + 8'd1)};
        rd_addr_q          <= rd_addr_q + ( (ar_len_q + 8'd1) * BEAT_BYTES );
      end

      // R push (data return)
      if (rf_push) begin
        rf_mem      [rf_wptr_q[RF_AW-1:0]] <= m_rdata_i;
        rf_resp_mem [rf_wptr_q[RF_AW-1:0]] <= m_rresp_i;
        rf_wptr_q   <= rf_wptr_q + 1'b1;
        if (m_rlast_i && (rd_out_bursts_q != 0)) begin
          rd_out_bursts_q <= rd_out_bursts_q - 16'd1;
        end
        if (m_rresp_i != 2'b00) rd_err_flag_q <= 1'b1;
      end

      // R pop (to client)
      if (rf_pop) begin
        rf_rptr_q          <= rf_rptr_q + 1'b1;
        rd_beats_emitted_q <= rd_beats_emitted_q + 32'd1;
      end

      // Complete command when last beat emitted
      if (rd_done_o) begin
        rd_active_q <= 1'b0;
      end
    end
  end

  // ---------------- Combinational (AR issue control)
  always_comb begin
    // defaults
    ar_addr_d  = ar_addr_q;
    ar_len_d   = ar_len_q;
    ar_valid_d = 1'b0;

    if (rd_active_q) begin
      // still beats to request?
      logic [31:0] beats_remaining = rd_beats_total_q - rd_beats_issued_q;
      if (beats_remaining != 0 && (rd_out_bursts_q < MAX_RD_OUTS)) begin
        // plan next burst
        logic [7:0] blen = calc_blen(rd_addr_q, beats_remaining);
        ar_addr_d  = rd_addr_q;
        ar_len_d   = blen - 8'd1;
        ar_valid_d = 1'b1;
      end
    end
  end

  // =============================================================================
  // Write Path
  // =============================================================================
  // Command registers
  logic                    wr_active_q;
  logic [AXI_ADDR_W-1:0]   wr_addr_q;
  logic [31:0]             wr_bytes_q;
  logic [31:0]             wr_beats_total_q;
  logic [31:0]             wr_beats_issued_q;   // by AW (sum of bursts)
  logic [31:0]             wr_beats_streamed_q; // on W channel
  logic [15:0]             wr_out_bursts_q;     // AW bursts in flight (awaiting B)
  logic                    wr_err_flag_q;

  // AW issue registers
  logic [AXI_ADDR_W-1:0]   aw_addr_q, aw_addr_d;
  logic [7:0]              aw_len_q,  aw_len_d;
  logic                    aw_valid_q, aw_valid_d;

  // FIFO of AW lengths (for W streaming sequencing)
  localparam int unsigned ALF_DEPTH = (MAX_WR_OUTS < 2) ? 2 : MAX_WR_OUTS;
  localparam int unsigned ALF_AW    = (ALF_DEPTH <= 2) ? 1 : $clog2(ALF_DEPTH);
  logic [7:0]              alf_mem [0:ALF_DEPTH-1];
  logic [ALF_AW:0]         alf_wptr_q, alf_rptr_q;
  wire                     alf_empty = (alf_wptr_q == alf_rptr_q);
  wire                     alf_full  = (alf_wptr_q[ALF_AW]     != alf_rptr_q[ALF_AW]) &&
                                       (alf_wptr_q[ALF_AW-1:0] == alf_rptr_q[ALF_AW-1:0]);

  // Current burst streaming counter
  logic [31:0]             w_curr_beats_left_q;

  // W data FIFO (input stream → AXI W)
  localparam int unsigned WF_AW = (WR_FIFO_DEPTH <= 2) ? 1 : $clog2(WR_FIFO_DEPTH);
  logic [AXI_DATA_W-1:0]   wf_mem [0:WR_FIFO_DEPTH-1];
  logic [AXI_DATA_W/8-1:0] wf_strb_mem [0:WR_FIFO_DEPTH-1];
  logic [WF_AW:0]          wf_wptr_q, wf_rptr_q;
  wire                     wf_empty = (wf_wptr_q == wf_rptr_q);
  wire                     wf_full  = (wf_wptr_q[WF_AW]     != wf_rptr_q[WF_AW]) &&
                                      (wf_wptr_q[WF_AW-1:0] == wf_rptr_q[WF_AW-1:0]);
  wire [AXI_DATA_W-1:0]    wf_rdata = wf_mem[wf_rptr_q[WF_AW-1:0]];
  wire [AXI_DATA_W/8-1:0]  wf_rstrb = wf_strb_mem[wf_rptr_q[WF_AW-1:0]];

  // Stream to AXI W
  assign m_wdata_o  = wf_rdata;
  assign m_wstrb_o  = wstrb_mux ? wf_rstrb : ALL_BYTES; // USE_WR_STRB_I gate (elaborates away)
  assign m_wvalid_o = !wf_empty && (w_curr_beats_left_q != 0);
  assign m_wlast_o  = (w_curr_beats_left_q == 32'd1) && m_wvalid_o && m_wready_i;

  // Pop from W FIFO on W handshake
  wire wf_pop  = m_wvalid_o && m_wready_i;
  // Push into W FIFO from client
  wire wf_push = wr_data_valid_i && wr_data_ready_o;

  // Connect ready to client
  assign wr_data_ready_o = !wf_full;

  // AW drive
  assign m_awaddr_o  = aw_addr_q;
  assign m_awlen_o   = aw_len_q;
  assign m_awvalid_o = aw_valid_q;

  // Write cmd ready when idle
  assign wr_cmd_ready_o = ~wr_active_q;

  // busy/done/error
  assign wr_busy_o  = wr_active_q;
  assign wr_done_o  = (wr_active_q && (wr_beats_streamed_q == wr_beats_total_q) &&
                       (w_curr_beats_left_q == 0) && alf_empty &&
                       (wr_out_bursts_q == 0));
  assign wr_error_o = wr_err_flag_q;

  // ---------------- Sequential (write)
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      wr_active_q         <= 1'b0;
      wr_addr_q           <= '0;
      wr_bytes_q          <= '0;
      wr_beats_total_q    <= '0;
      wr_beats_issued_q   <= '0;
      wr_beats_streamed_q <= '0;
      wr_out_bursts_q     <= '0;
      wr_err_flag_q       <= 1'b0;

      aw_addr_q           <= '0;
      aw_len_q            <= '0;
      aw_valid_q          <= 1'b0;

      alf_wptr_q          <= '0;
      alf_rptr_q          <= '0;
      w_curr_beats_left_q <= 32'd0;

      wf_wptr_q           <= '0;
      wf_rptr_q           <= '0;
    end else begin
      // Default holds
      aw_addr_q  <= aw_addr_d;
      aw_len_q   <= aw_len_d;
      aw_valid_q <= aw_valid_d;

      // Accept new write command
      if (wr_cmd_valid_i && wr_cmd_ready_o) begin
        wr_active_q         <= 1'b1;
        wr_addr_q           <= wr_cmd_addr_i;
        wr_bytes_q          <= wr_cmd_bytes_i;
        wr_beats_total_q    <= (STRICT_ALIGN) ? (wr_cmd_bytes_i/BEAT_BYTES)
                                              : ((wr_cmd_bytes_i + BEAT_BYTES - 1) >> SZ_LOG2);
        wr_beats_issued_q   <= 32'd0;
        wr_beats_streamed_q <= 32'd0;
        wr_out_bursts_q     <= 16'd0;
        wr_err_flag_q       <= 1'b0;
        w_curr_beats_left_q <= 32'd0;
        // Clear AW len FIFO pointers
        alf_wptr_q          <= '0;
        alf_rptr_q          <= '0;
      end

      // AW handshake: track outstanding bursts and advance addr/issued beats
      if (m_awvalid_o && m_awready_i) begin
        wr_out_bursts_q   <= wr_out_bursts_q + 16'd1;
        wr_beats_issued_q <= wr_beats_issued_q + {24'd0, (aw_len_q + 8'd1)};
        wr_addr_q         <= wr_addr_q + ( (aw_len_q + 8'd1) * BEAT_BYTES );

        // Push length into AW-length FIFO
        if (!alf_full) begin
          alf_mem[alf_wptr_q[ALF_AW-1:0]] <= aw_len_q;
          alf_wptr_q <= alf_wptr_q + 1'b1;
        end
      end

      // Pull next burst length when current finished and queue non-empty
      if ((w_curr_beats_left_q == 0) && !alf_empty) begin
        w_curr_beats_left_q <= {24'd0, (alf_mem[alf_rptr_q[ALF_AW-1:0]] + 8'd1)};
        alf_rptr_q          <= alf_rptr_q + 1'b1;
      end

      // W FIFO push from client
      if (wf_push) begin
        wf_mem     [wf_wptr_q[WF_AW-1:0]] <= wr_data_i;
        wf_strb_mem[wf_wptr_q[WF_AW-1:0]] <= wr_strb_i;
        wf_wptr_q <= wf_wptr_q + 1'b1;
      end

      // W stream pop to AXI
      if (wf_pop) begin
        wf_rptr_q          <= wf_rptr_q + 1'b1;
        wr_beats_streamed_q<= wr_beats_streamed_q + 32'd1;
        if (w_curr_beats_left_q != 0) begin
          w_curr_beats_left_q <= w_curr_beats_left_q - 32'd1;
        end
      end

      // B response
      if (m_bvalid_i) begin
        if (wr_out_bursts_q != 0) wr_out_bursts_q <= wr_out_bursts_q - 16'd1;
        if (m_bresp_i != 2'b00) wr_err_flag_q <= 1'b1;
      end

      // Finish command when conditions met (wr_done_o is combinational pulse)
      if (wr_done_o) begin
        wr_active_q <= 1'b0;
      end
    end
  end

  // ---------------- Combinational (AW issue control)
  always_comb begin
    // defaults
    aw_addr_d  = aw_addr_q;
    aw_len_d   = aw_len_q;
    aw_valid_d = 1'b0;

    if (wr_active_q) begin
      logic [31:0] beats_remaining = wr_beats_total_q - wr_beats_issued_q;
      // Issue new AW if we have beats left and credits and room in the AW-length FIFO
      if (beats_remaining != 0 && (wr_out_bursts_q < MAX_WR_OUTS) && !alf_full) begin
        logic [7:0] blen = calc_blen(wr_addr_q, beats_remaining);
        aw_addr_d  = wr_addr_q;
        aw_len_d   = blen - 8'd1;
        aw_valid_d = 1'b1;
      end
    end
  end

  // =============================================================================
  // Assertions (simulation only)
  // =============================================================================
`ifdef ASSERT_ON
  // READ: alignment (when enabled)
  if (STRICT_ALIGN) begin
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      rd_cmd_valid_i && rd_cmd_ready_o |-> (rd_cmd_addr_i % BEAT_BYTES) == 0)
      else $error("axi_mst_core: RD addr not beat-aligned");
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      rd_cmd_valid_i && rd_cmd_ready_o |-> (rd_cmd_bytes_i % BEAT_BYTES) == 0)
      else $error("axi_mst_core: RD bytes not multiple of beat");
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      wr_cmd_valid_i && wr_cmd_ready_o |-> (wr_cmd_addr_i % BEAT_BYTES) == 0)
      else $error("axi_mst_core: WR addr not beat-aligned");
    assert property (@(posedge clk_i) disable iff(!rstn_i)
      wr_cmd_valid_i && wr_cmd_ready_o |-> (wr_cmd_bytes_i % BEAT_BYTES) == 0)
      else $error("axi_mst_core: WR bytes not multiple of beat");
  end

  // READ: do not overflow R FIFO
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    !(m_rvalid_i && !m_rready_o))
    else $error("axi_mst_core: R FIFO overflow (m_rready_o deasserted too late)");

  // WRITE: do not underflow W FIFO while streaming
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    (m_wvalid_o && !wf_empty))
    else $error("axi_mst_core: W underflow (m_wvalid_o but FIFO empty)");

  // WRITE: WLAST must coincide with end of current burst
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    m_wlast_o |-> (w_curr_beats_left_q == 1))
    else $error("axi_mst_core: WLAST not aligned to AW length");

  // Outstanding counters bound
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    rd_out_bursts_q <= MAX_RD_OUTS)
    else $error("axi_mst_core: rd_out_bursts exceeded MAX_RD_OUTS");
  assert property (@(posedge clk_i) disable iff(!rstn_i)
    wr_out_bursts_q <= MAX_WR_OUTS)
    else $error("axi_mst_core: wr_out_bursts exceeded MAX_WR_OUTS");

`endif

endmodule

`default_nettype wire
