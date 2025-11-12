// ============================================================================
//  Aquila-T1 CSR Block (APB3)
//  File: csr_block.sv
//  Description:
//    Implements the CSR map defined in csr_map.json (v1.0).
//    - APB3 slave, 32-bit data, byte addressing
//    - RW fields for ARRAY_CFG and partition masks
//    - DOORBELL with W1P pulse
//    - IRQ_MASK (RW)
//    - Read-only ID/REV/FEATURES
//
//  Compatibility:
//    Matches the port list used in tile_top.sv. Additional CSRs can be added
//    later without breaking existing named-port instantiation.
//
//  Parameters:
//    APB_ADDR_W : APB address bus width (bytes)
//    APB_DATA_W : must be 32
//    GB_BANKS   : number of SRAM banks (<= 32 in this revision)
//
//  Address map (byte addresses):
//    0x0000  TILE_ID         [RO] {DESIGN_ID[31:16]=0x4151 'AQ', REV[15:8]=1, FEATURES[7:0]=0}
//    0x0010  ARRAY_CFG       [RW] {SPARSE_2OF4[4], MODE[3:1], ENABLE[0]}
//    0x0020  PART_A_MASK0    [RW] bank select bits (0..GB_BANKS-1)
//    0x0040  PART_W_MASK0    [RW] bank select bits
//    0x0060  PART_C_MASK0    [RW] bank select bits
//    0x0080  DOORBELL        [W1P] write '1' to bit0 to emit 1-cycle pulse
//    0x0084  IRQ_MASK        [RW] {MASK[7:0]}
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module csr_block #(
  parameter int unsigned APB_ADDR_W = 20,
  parameter int unsigned APB_DATA_W = 32,
  parameter int unsigned GB_BANKS   = 16
)(
  // Clock / Reset
  input  logic                   clk_i,
  input  logic                   rstn_i,   // synchronous active-low

  // APB3 slave
  input  logic [APB_ADDR_W-1:0]  paddr_i,
  input  logic                   psel_i,
  input  logic                   penable_i,
  input  logic                   pwrite_i,
  input  logic [APB_DATA_W-1:0]  pwdata_i,
  output logic [APB_DATA_W-1:0]  prdata_o,
  output logic                   pready_o,
  output logic                   pslverr_o,

  // Controls used by tile_top.sv today
  output logic                   array_enable_o,
  output logic [2:0]             array_mode_o,
  output logic                   sparsity_2of4_en_o,

  output logic [GB_BANKS-1:0]    part_a_mask_o,
  output logic [GB_BANKS-1:0]    part_w_mask_o,
  output logic [GB_BANKS-1:0]    part_c_mask_o,

  output logic                   doorbell_o,     // 1-cycle pulse on write-1 to DOORBELL.RING
  output logic [7:0]             irq_mask_o
);

  // --------------------------------------------------------------------------
  // Static checks
  // --------------------------------------------------------------------------
  initial begin
    if (APB_DATA_W != 32) begin
      $error("csr_block: APB_DATA_W must be 32 (is %0d).", APB_DATA_W);
    end
    if (GB_BANKS > 32) begin
      $error("csr_block: GB_BANKS=%0d exceeds 32. This revision supports up to 32. Expand mask words for more.", GB_BANKS);
    end
  end

  // --------------------------------------------------------------------------
  // Address map constants (byte addresses)
  // --------------------------------------------------------------------------
  localparam logic [APB_ADDR_W-1:0] ADDR_TILE_ID      = 32'h0000;
  localparam logic [APB_ADDR_W-1:0] ADDR_ARRAY_CFG    = 32'h0010;
  localparam logic [APB_ADDR_W-1:0] ADDR_PART_A0      = 32'h0020;
  localparam logic [APB_ADDR_W-1:0] ADDR_PART_W0      = 32'h0040;
  localparam logic [APB_ADDR_W-1:0] ADDR_PART_C0      = 32'h0060;
  localparam logic [APB_ADDR_W-1:0] ADDR_DOORBELL     = 32'h0080;
  localparam logic [APB_ADDR_W-1:0] ADDR_IRQ_MASK     = 32'h0084;

  // Word-aligned assumption
  wire [APB_ADDR_W-1:0] addr_aligned = {paddr_i[APB_ADDR_W-1:2], 2'b00};

  // APB handshake
  wire wr_en = psel_i && penable_i && pwrite_i;
  wire rd_en = psel_i && !pwrite_i; // combinational read

  // --------------------------------------------------------------------------
  // Registers (with resets per csr_map.json)
  // --------------------------------------------------------------------------
  // ARRAY_CFG
  logic        array_enable_q;
  logic [2:0]  array_mode_q;
  logic        sparse_2of4_q;

  // Partition masks (stored as 32b regs, truncated to GB_BANKS)
  logic [31:0] part_a_mask_reg_q, part_w_mask_reg_q, part_c_mask_reg_q;

  // DOORBELL pulse
  logic doorbell_pulse_q;

  // IRQ mask (8-bit)
  logic [7:0] irq_mask_q;

  // Reset
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      array_enable_q       <= 1'b0;
      array_mode_q         <= 3'd1;       // default = INT8 per spec note
      sparse_2of4_q        <= 1'b0;

      part_a_mask_reg_q    <= 32'h0000_0000;
      part_w_mask_reg_q    <= 32'h0000_0000;
      part_c_mask_reg_q    <= 32'h0000_0000;

      doorbell_pulse_q     <= 1'b0;

      irq_mask_q           <= 8'hFF;      // enable all by default
    end else begin
      // Default pulses low
      doorbell_pulse_q     <= 1'b0;

      // Writes
      if (wr_en) begin
        unique case (addr_aligned)
          ADDR_ARRAY_CFG: begin
            array_enable_q <= pwdata_i[0];
            array_mode_q   <= pwdata_i[3:1];
            sparse_2of4_q  <= pwdata_i[4];
          end

          ADDR_PART_A0: begin
            part_a_mask_reg_q <= pwdata_i;
          end

          ADDR_PART_W0: begin
            part_w_mask_reg_q <= pwdata_i;
          end

          ADDR_PART_C0: begin
            part_c_mask_reg_q <= pwdata_i;
          end

          ADDR_DOORBELL: begin
            // W1P: writing '1' to bit0 emits a single-cycle pulse
            if (pwdata_i[0]) doorbell_pulse_q <= 1'b1;
          end

          ADDR_IRQ_MASK: begin
            irq_mask_q <= pwdata_i[7:0];
          end

          default: begin
            // No side-effects for unmapped addresses
          end
        endcase
      end
    end
  end

  // --------------------------------------------------------------------------
  // Read mux (combinational, no wait states)
  // --------------------------------------------------------------------------
  logic [31:0] rdata;

  always_comb begin
    rdata = 32'h0000_0000;

    unique case (addr_aligned)
      ADDR_TILE_ID: begin
        rdata[31:16] = 16'h4151;  // 'A' 'Q'
        rdata[15:8]  = 8'h01;     // REV
        rdata[7:0]   = 8'h00;     // FEATURES (reserved)
      end

      ADDR_ARRAY_CFG: begin
        rdata[0]     = array_enable_q;
        rdata[3:1]   = array_mode_q;
        rdata[4]     = sparse_2of4_q;
      end

      ADDR_PART_A0: begin
        rdata = part_a_mask_reg_q;
      end

      ADDR_PART_W0: begin
        rdata = part_w_mask_reg_q;
      end

      ADDR_PART_C0: begin
        rdata = part_c_mask_reg_q;
      end

      ADDR_DOORBELL: begin
        rdata = 32'h0; // reads-as-zero
      end

      ADDR_IRQ_MASK: begin
        rdata[7:0] = irq_mask_q;
      end

      default: begin
        rdata = 32'h0000_0000;
      end
    endcase
  end

  // --------------------------------------------------------------------------
  // APB response signals
  //  - Zero wait state (pready=1)
  //  - pslverr set for writes to unmapped addresses (optional)
  // --------------------------------------------------------------------------
  assign prdata_o  = rdata;
  assign pready_o  = 1'b1;

  // Simple error reporting on illegal writes; reads are allowed (return 0)
  logic addr_is_mapped_wr;
  always_comb begin
    addr_is_mapped_wr = 1'b0;
    unique case (addr_aligned)
      ADDR_ARRAY_CFG,
      ADDR_PART_A0,
      ADDR_PART_W0,
      ADDR_PART_C0,
      ADDR_DOORBELL,
      ADDR_IRQ_MASK: addr_is_mapped_wr = 1'b1;
      default:       addr_is_mapped_wr = 1'b0;
    endcase
  end

  assign pslverr_o = (psel_i && penable_i && pwrite_i && !addr_is_mapped_wr);

  // --------------------------------------------------------------------------
  // Outputs to the rest of the tile
  // --------------------------------------------------------------------------
  assign array_enable_o      = array_enable_q;
  assign array_mode_o        = array_mode_q;
  assign sparsity_2of4_en_o  = sparse_2of4_q;

  // Truncate to GB_BANKS
  assign part_a_mask_o       = part_a_mask_reg_q[GB_BANKS-1:0];
  assign part_w_mask_o       = part_w_mask_reg_q[GB_BANKS-1:0];
  assign part_c_mask_o       = part_c_mask_reg_q[GB_BANKS-1:0];

  assign doorbell_o          = doorbell_pulse_q;
  assign irq_mask_o          = irq_mask_q;

`ifdef ASSERT_ON
  // Ensure addresses are word-aligned (APB masters typically ensure this)
  assert property (@(posedge clk_i) disable iff (!rstn_i)
    psel_i |-> (paddr_i[1:0] == 2'b00))
    else $warning("csr_block: unaligned APB access (ignored alignment in decode).");

  // Writes to DOORBELL should be W1P semantics
  // (We don't store a sticky bit; just a pulse. Coverage wise, we can cover a pulse.)
`endif

endmodule

`default_nettype wire
