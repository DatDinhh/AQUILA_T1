// ============================================================================
//  Aquila-T1 RISC-V Microcontroller Subsystem
//  File: riscv_subsys.sv
//  Description:
//    Core-agnostic RV32IMC microcontroller wrapper with:
//      - Boot ROM (hex-init), TCM SRAM (dual-port), local WDT
//      - Data-bus decode: TCM/ROM/local-APB vs external APB master window
//      - APB3 MASTER with RMW for sub-word writes
//      - JTAG pins (pass-through) and external interrupt line
//
//    External APB window (default): 0x4000_0000 .. 0xFFFF_FFFF
//    Local WDT window (default):    0x1FFF_0000 .. 0x1FFF_0FFF
//    TCM SRAM window (default):     0x2000_0000 .. 0x2000_0000+RAM_BYTES-1
//    Boot ROM window (default):     0x0000_0000 .. 0x0000_0000+ROM_BYTES-1
//
//  Notes:
//    - This wrapper compiles with a lightweight stub core by default.
//    - For a real RV32 core (Ibex/CV32E40P), define USE_IBEX / USE_CV32E40P
//      and wire the core to the imem/dmem interfaces indicated below.
//
//  © 2025. MIT-style license (adjust for your repo).
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module riscv_subsys #(
  // Memory map configuration
  parameter logic [31:0] ROM_BASE       = 32'h0000_0000,
  parameter int unsigned ROM_BYTES      = 16*1024,
  parameter string       BOOTROM_HEX    = "boot_rom.hex",

  parameter logic [31:0] RAM_BASE       = 32'h2000_0000,
  parameter int unsigned RAM_BYTES      = 64*1024,

  parameter logic [31:0] WDT_BASE       = 32'h1FFF_0000, // local WDT registers
  parameter int unsigned WDT_BYTES      = 4*1024,

  parameter logic [31:0] APB_EXT_BASE   = 32'h4000_0000, // external APB window
  // APB master data width
  parameter int unsigned APB_DATA_W     = 32,
  parameter int unsigned APB_ADDR_W     = 32
)(
  // Clock / Reset (control domain)
  input  logic                   clk_i,
  input  logic                   rstn_i,         // synchronous active-low

  // -------- APB3 Master (to tile CSR/INTC fabric) --------
  output logic [APB_ADDR_W-1:0]  paddr_o,
  output logic                   psel_o,
  output logic                   penable_o,
  output logic                   pwrite_o,
  output logic [APB_DATA_W-1:0]  pwdata_o,
  input  logic [APB_DATA_W-1:0]  prdata_i,
  input  logic                   pready_i,
  input  logic                   pslverr_i,

  // -------- JTAG (pass-through to real debug module/core) --------
  input  logic                   jtag_tck_i,
  input  logic                   jtag_trst_ni,
  input  logic                   jtag_tms_i,
  input  logic                   jtag_tdi_i,
  output logic                   jtag_tdo_o,

  // -------- Interrupts --------
  input  logic [31:0]            intr_i,         // aggregated lines from intc_tile
  output logic                   wdog_irq_o      // local WDT fired (level)
);

  // ==========================================================================
  // Parameter checks
  // ==========================================================================
  initial begin
    if (APB_DATA_W != 32) $error("riscv_subsys: APB_DATA_W must be 32.");
    if ((ROM_BYTES & (ROM_BYTES-1)) != 0)  $error("ROM_BYTES must be power-of-two.");
    if ((RAM_BYTES & (RAM_BYTES-1)) != 0)  $error("RAM_BYTES must be power-of-two.");
    if ((WDT_BYTES & (WDT_BYTES-1)) != 0)  $error("WDT_BYTES must be power-of-two.");
  end

  localparam int unsigned ROM_AW = (ROM_BYTES <= 4) ? 2 : $clog2(ROM_BYTES);
  localparam int unsigned RAM_AW = (RAM_BYTES <= 4) ? 2 : $clog2(RAM_BYTES);

  // ==========================================================================
  // Core IF — Ibex-like simple handshake (works well for many small RV cores)
  //   Instruction fetch
  // ==========================================================================
  logic        imem_req,  imem_gnt,  imem_rvalid;
  logic [31:0] imem_addr;
  logic [31:0] imem_rdata;
  logic        imem_err;

  //   Data access
  logic        dmem_req,  dmem_gnt,  dmem_rvalid;
  logic        dmem_we;
  logic [3:0]  dmem_be;
  logic [31:0] dmem_addr, dmem_wdata, dmem_rdata;
  logic        dmem_err;

  // External interrupt line to core (OR reduction)
  logic        ext_irq;
  assign ext_irq = |intr_i;

  // ==========================================================================
  // Core instantiation hooks
  //   By default, use a stub that issues no requests (safe). Replace with
  //   a real core under USE_IBEX / USE_CV32E40P.
  // ==========================================================================
`ifdef USE_IBEX
  // ---- Example hook for Ibex core (adjust port names per your version) ----
  ibex_core u_core (
    .clk_i               (clk_i),
    .rst_ni              (rstn_i),

    .test_en_i           (1'b0),
    .hart_id_i           (32'd0),

    // Instruction bus
    .instr_req_o         (imem_req),
    .instr_gnt_i         (imem_gnt),
    .instr_rvalid_i      (imem_rvalid),
    .instr_addr_o        (imem_addr),
    .instr_rdata_i       (imem_rdata),
    .instr_err_i         (imem_err),

    // Data bus
    .data_req_o          (dmem_req),
    .data_gnt_i          (dmem_gnt),
    .data_rvalid_i       (dmem_rvalid),
    .data_we_o           (dmem_we),
    .data_be_o           (dmem_be),
    .data_addr_o         (dmem_addr),
    .data_wdata_o        (dmem_wdata),
    .data_rdata_i        (dmem_rdata),
    .data_err_i          (dmem_err),

    // Interrupts (use external only; SW/TIMER optional via intc_tile)
    .irq_software_i      (1'b0),
    .irq_timer_i         (1'b0),
    .irq_external_i      (ext_irq),
    .irq_fast_i          (15'd0),
    .irq_nm_i            (1'b0),

    // Debug
    .debug_req_i         (1'b0) // JTAG debug module can drive this if you integrate RV_DM
  );
  assign jtag_tdo_o = 1'b0; // no direct JTAG in core; hook RV_DM separately
`elsif USE_CV32E40P
  // ---- Example hook for CV32E40P (adjust per your core release) ----
  cv32e40p_core u_core (
    .clk_i               (clk_i),
    .rst_ni              (rstn_i),

    .instr_req_o         (imem_req),
    .instr_gnt_i         (imem_gnt),
    .instr_rvalid_i      (imem_rvalid),
    .instr_addr_o        (imem_addr),
    .instr_rdata_i       (imem_rdata),
    .instr_err_i         (imem_err),

    .data_req_o          (dmem_req),
    .data_gnt_i          (dmem_gnt),
    .data_rvalid_i       (dmem_rvalid),
    .data_we_o           (dmem_we),
    .data_be_o           (dmem_be),
    .data_addr_o         (dmem_addr),
    .data_wdata_o        (dmem_wdata),
    .data_rdata_i        (dmem_rdata),
    .data_err_i          (dmem_err),

    .irq_i               ({31'd0, ext_irq}),

    .debug_req_i         (1'b0)
  );
  assign jtag_tdo_o = 1'b0;
`else
  // ---- Safe default: stub core (no bus traffic) ----
  rv32_core_stub u_core_stub (
    .clk_i      (clk_i),
    .rstn_i     (rstn_i),
    .imem_req_o (imem_req),
    .imem_gnt_i (imem_gnt),
    .imem_rvalid_i(imem_rvalid),
    .imem_addr_o(imem_addr),
    .imem_rdata_i(imem_rdata),
    .imem_err_i (imem_err),

    .dmem_req_o (dmem_req),
    .dmem_gnt_i (dmem_gnt),
    .dmem_rvalid_i(dmem_rvalid),
    .dmem_we_o  (dmem_we),
    .dmem_be_o  (dmem_be),
    .dmem_addr_o(dmem_addr),
    .dmem_wdata_o(dmem_wdata),
    .dmem_rdata_i(dmem_rdata),
    .dmem_err_i (dmem_err),

    .ext_irq_i  (ext_irq)
  );
  // Pass-through JTAG pins to an unimplemented debug module (tdo low)
  assign jtag_tdo_o = 1'b0;
`endif

  // Consume JTAG inputs to avoid unused warnings in the default build
  logic unused_jtag;
  assign unused_jtag = jtag_tck_i ^ jtag_trst_ni ^ jtag_tms_i ^ jtag_tdi_i;

  // ==========================================================================
  // Boot ROM (synchronous read, 1-cycle latency)
  // ==========================================================================
  logic                 rom_en;
  logic [ROM_AW-1:2]    rom_addr_w; // word index
  logic [31:0]          rom_rdata_q;

  // ROM array
  logic [31:0] rom_mem [0:(ROM_BYTES/4)-1];
  initial begin
    // If file missing, tools will warn; memory defaults to X/0 per simulator.
    $readmemh(BOOTROM_HEX, rom_mem);
  end

  always_ff @(posedge clk_i) begin
    if (rom_en) rom_rdata_q <= rom_mem[rom_addr_w];
  end

  // ==========================================================================
  // TCM SRAM (dual-port: Port A = IFetch read, Port B = Data R/W)
  //   Simple 1-cycle read latency; write-first behavior not required here.
  // ==========================================================================
  logic                 ram_a_en;
  logic [RAM_AW-1:2]    ram_a_addr_w;
  logic [31:0]          ram_a_rdata_q;

  logic                 ram_b_en, ram_b_we;
  logic [3:0]           ram_b_be;
  logic [RAM_AW-1:2]    ram_b_addr_w;
  logic [31:0]          ram_b_wdata;
  logic [31:0]          ram_b_rdata_q;

  logic [31:0]          ram_mem [0:(RAM_BYTES/4)-1];

  // Port A (IFetch read)
  always_ff @(posedge clk_i) begin
    if (ram_a_en) ram_a_rdata_q <= ram_mem[ram_a_addr_w];
  end

  // Port B (Data RW)
  integer bi;
  always_ff @(posedge clk_i) begin
    if (ram_b_en) begin
      if (ram_b_we) begin
        // Byte enable writes
        logic [31:0] w = ram_mem[ram_b_addr_w];
        if (ram_b_be[0]) w[7:0]   = ram_b_wdata[7:0];
        if (ram_b_be[1]) w[15:8]  = ram_b_wdata[15:8];
        if (ram_b_be[2]) w[23:16] = ram_b_wdata[23:16];
        if (ram_b_be[3]) w[31:24] = ram_b_wdata[31:24];
        ram_mem[ram_b_addr_w] <= w;
      end
      ram_b_rdata_q <= ram_mem[ram_b_addr_w];
    end
  end

  // ==========================================================================
  // Local Watchdog (memory-mapped @ WDT_BASE)
  // ==========================================================================
  // Reg map:
  //   0x00 CTRL:   [0]EN [1]AUTORELOAD
  //   0x04 RELOAD: reload value
  //   0x08 COUNT:  current count (RW)
  //   0x0C KICK:   W1P
  //   0x10 STATUS: [0]FIRED (W1C)
  // irq line (level) asserted while STATUS[0]=1
  logic        wdg_en_q, wdg_autorl_q, wdg_kick_pulse;
  logic [31:0] wdg_reload_q, wdg_count_q;
  logic        wdg_fired_q; // sticky until W1C
  assign wdog_irq_o = wdg_fired_q;

  // ==========================================================================
  // APB Master FSM for external window (+ internal RMW for sub-word stores)
  // ==========================================================================
  typedef enum logic [2:0] {
    A_IDLE, A_SETUP, A_ACCESS, A_RMW_READ, A_RMW_SETUPW, A_RMW_ACCESSW, A_RESP
  } apb_state_e;
  apb_state_e astate_q, astate_d;

  logic [APB_ADDR_W-1:0] apb_addr_q, apb_addr_d;
  logic                  apb_we_q,   apb_we_d;
  logic [3:0]            apb_be_q,   apb_be_d;
  logic [31:0]           apb_wdata_q, apb_wdata_d;
  logic [31:0]           apb_rdata_q;

  logic                  apb_do_rmw;       // need read-modify-write
  logic [31:0]           apb_rmw_mask;
  logic [31:0]           apb_rmw_wdata;

  // Drive APB default
  assign paddr_o   = apb_addr_q;
  assign pwrite_o  = apb_we_q;
  assign pwdata_o  = apb_wdata_q;

  // PSEL/PENABLE sequencing
  assign psel_o    = (astate_q == A_SETUP)  || (astate_q == A_ACCESS) ||
                     (astate_q == A_RMW_SETUPW) || (astate_q == A_RMW_ACCESSW);
  assign penable_o = (astate_q == A_ACCESS) || (astate_q == A_RMW_ACCESSW);

  // ==========================================================================
  // Instruction path decode (ROM or SRAM) — 1-cycle response
  // ==========================================================================
  logic imem_hit_rom, imem_hit_ram, imem_hit_ext, imem_hit_wdt;

  assign imem_hit_rom = (imem_addr >= ROM_BASE) &&
                        (imem_addr < (ROM_BASE + ROM_BYTES));
  assign imem_hit_ram = (imem_addr >= RAM_BASE) &&
                        (imem_addr < (RAM_BASE + RAM_BYTES));
  assign imem_hit_wdt = (imem_addr >= WDT_BASE) &&
                        (imem_addr < (WDT_BASE + WDT_BYTES));
  assign imem_hit_ext = (imem_addr >= APB_EXT_BASE);

  // We grant immediately; return rvalid/data next cycle for ROM/RAM; error for others
  assign imem_gnt   = imem_req;

  logic imem_valid_q;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      imem_valid_q <= 1'b0;
    end else begin
      imem_valid_q <= imem_req;
    end
  end

  // Register read enables/addresses for ROM/RAM
  always_ff @(posedge clk_i) begin
    if (imem_req && imem_hit_rom) begin
      rom_en      <= 1'b1;
      rom_addr_w  <= (imem_addr - ROM_BASE) [ROM_AW-1:2];
    end else begin
      rom_en      <= 1'b0;
    end

    if (imem_req && imem_hit_ram) begin
      ram_a_en     <= 1'b1;
      ram_a_addr_w <= (imem_addr - RAM_BASE)[RAM_AW-1:2];
    end else begin
      ram_a_en     <= 1'b0;
    end
  end

  assign imem_rdata = imem_hit_rom ? rom_rdata_q :
                      imem_hit_ram ? ram_a_rdata_q : 32'h0000_0000;

  assign imem_err   = imem_valid_q && (imem_hit_ext || imem_hit_wdt) ? 1'b1 : 1'b0;
  assign imem_rvalid= imem_valid_q && (imem_hit_rom || imem_hit_ram || imem_err);

  // ==========================================================================
  // Data path decode and execution (single outstanding)
  // ==========================================================================
  logic d_hit_rom, d_hit_ram, d_hit_wdt, d_hit_ext;

  assign d_hit_rom = (dmem_addr >= ROM_BASE) && (dmem_addr < (ROM_BASE + ROM_BYTES));
  assign d_hit_ram = (dmem_addr >= RAM_BASE) && (dmem_addr < (RAM_BASE + RAM_BYTES));
  assign d_hit_wdt = (dmem_addr >= WDT_BASE) && (dmem_addr < (WDT_BASE + WDT_BYTES));
  assign d_hit_ext = (dmem_addr >= APB_EXT_BASE);

  // Data FSM
  typedef enum logic [2:0] {
    D_IDLE, D_ROM, D_RAM, D_WDT_RD, D_WDT_WR, D_APB, D_RESP
  } d_state_e;
  d_state_e dstate_q, dstate_d;

  // Latched request info
  logic        d_we_q;
  logic [3:0]  d_be_q;
  logic [31:0] d_addr_q, d_wdata_q;

  // Default gnt: accept in IDLE only
  assign dmem_gnt = (dstate_q == D_IDLE) && dmem_req;

  // Capture request when granted
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      dstate_q   <= D_IDLE;
      d_we_q     <= 1'b0;
      d_be_q     <= 4'h0;
      d_addr_q   <= 32'h0;
      d_wdata_q  <= 32'h0;

      astate_q   <= A_IDLE;
      apb_addr_q <= '0; apb_we_q <= 1'b0; apb_be_q <= 4'h0; apb_wdata_q <= '0;

      wdg_en_q   <= 1'b0; wdg_autorl_q <= 1'b0; wdg_kick_pulse <= 1'b0;
      wdg_reload_q <= 32'd0; wdg_count_q <= 32'd0; wdg_fired_q <= 1'b0;
    end else begin
      // Default single-cycle pulses
      wdg_kick_pulse <= 1'b0;

      // ---------------- Data FSM ----------------
      dstate_q <= dstate_d;

      // ---------------- APB FSM ----------------
      astate_q  <= astate_d;
      apb_addr_q<= apb_addr_d;
      apb_we_q  <= apb_we_d;
      apb_be_q  <= apb_be_d;
      apb_wdata_q <= apb_wdata_d;

      if (dmem_gnt) begin
        d_we_q    <= dmem_we;
        d_be_q    <= dmem_be;
        d_addr_q  <= dmem_addr;
        d_wdata_q <= dmem_wdata;
      end

      // Local WDT free-running
      if (wdg_kick_pulse) begin
        wdg_count_q <= wdg_reload_q;
      end else if (wdg_en_q && (wdg_count_q != 32'd0)) begin
        wdg_count_q <= wdg_count_q - 32'd1;
      end
      if (wdg_en_q && (wdg_count_q == 32'd1)) begin
        wdg_fired_q <= 1'b1;
        if (wdg_autorl_q) wdg_count_q <= wdg_reload_q;
      end

      // APB read data latch
      if ((astate_q == A_ACCESS) && pready_i && !apb_we_q) begin
        apb_rdata_q <= prdata_i;
      end
      if ((astate_q == A_RMW_ACCESSW) && pready_i) begin
        // complete RMW write
      end
    end
  end

  // dmem_rvalid / rdata / err
  logic d_rvalid_d, d_err_d;
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      dmem_rvalid <= 1'b0;
      dmem_rdata  <= 32'h0;
      dmem_err    <= 1'b0;
    end else begin
      dmem_rvalid <= d_rvalid_d;
      dmem_err    <= d_err_d;
      // dmem_rdata assigned in comb below for TCM/APB/WDT
    end
  end

  // Combinational next-state logic
  always_comb begin
    // Defaults
    dstate_d   = dstate_q;
    d_rvalid_d = 1'b0;
    d_err_d    = 1'b0;

    // Default RAM/ROM enables off
    ram_b_en   = 1'b0; ram_b_we = 1'b0; ram_b_be = 4'h0;
    ram_b_addr_w = '0; ram_b_wdata = '0;

    // APB defaults
    astate_d   = astate_q;
    apb_addr_d = apb_addr_q; apb_we_d = apb_we_q; apb_be_d = apb_be_q; apb_wdata_d = apb_wdata_q;
    apb_do_rmw = 1'b0; apb_rmw_mask = 32'h0; apb_rmw_wdata = 32'h0;

    // WDT side-effects
    logic wdt_wr = 1'b0; logic wdt_rd = 1'b0;
    logic [31:0] wdt_rdata = 32'h0;

    case (dstate_q)
      D_IDLE: begin
        if (dmem_req) begin
          if (d_hit_ram) begin
            dstate_d       = D_RAM;
          end else if (d_hit_rom) begin
            dstate_d       = D_ROM;
          end else if (d_hit_wdt) begin
            if (dmem_we)   dstate_d = D_WDT_WR;
            else           dstate_d = D_WDT_RD;
          end else if (d_hit_ext) begin
            // Prepare APB op
            apb_addr_d   = dmem_addr;
            apb_we_d     = dmem_we;
            apb_be_d     = dmem_be;
            apb_wdata_d  = dmem_wdata;

            // RMW if sub-word store
            if (dmem_we && dmem_be != 4'hF) begin
              apb_do_rmw = 1'b1;
              astate_d   = A_RMW_READ; // do a read, then write merged
            end else begin
              astate_d   = A_SETUP;
            end
            dstate_d     = D_APB;
          end else begin
            // Decode miss
            d_rvalid_d   = 1'b1;
            d_err_d      = 1'b1;
            dstate_d     = D_RESP;
          end
        end
      end

      // --- ROM read (1-cycle latency) ---
      D_ROM: begin
        // emulate 1-cycle read latency using rom via instruction port?
        // We do not share ROM port B; reuse ROM array combinationally is avoided.
        // For data reads to ROM, just return 0 for simplicity or mirror IF port.
        dmem_rdata  = rom_mem[(d_addr_q - ROM_BASE) >> 2];
        d_rvalid_d  = 1'b1;
        d_err_d     = d_we_q; // writes to ROM are error
        dstate_d    = D_RESP;
      end

      // --- RAM read/write (1-cycle latency) ---
      D_RAM: begin
        ram_b_en     = 1'b1;
        ram_b_we     = d_we_q;
        ram_b_be     = d_be_q;
        ram_b_addr_w = (d_addr_q - RAM_BASE) [RAM_AW-1:2];
        ram_b_wdata  = d_wdata_q;

        dmem_rdata   = ram_b_rdata_q;
        d_rvalid_d   = 1'b1;
        d_err_d      = 1'b0;
        dstate_d     = D_RESP;
      end

      // --- Local WDT read ---
      D_WDT_RD: begin
        wdt_rd = 1'b1;
        unique case (d_addr_q - WDT_BASE)
          32'h00: wdt_rdata = {30'h0, wdg_autorl_q, wdg_en_q};
          32'h04: wdt_rdata = wdg_reload_q;
          32'h08: wdt_rdata = wdg_count_q;
          32'h0C: wdt_rdata = 32'h0;
          32'h10: wdt_rdata = {31'h0, wdg_fired_q};
          default: begin wdt_rdata = 32'h0; d_err_d = 1'b1; end
        endcase
        dmem_rdata = wdt_rdata;
        d_rvalid_d = 1'b1;
        dstate_d   = D_RESP;
      end

      // --- Local WDT write ---
      D_WDT_WR: begin
        wdt_wr = 1'b1;
        unique case (d_addr_q - WDT_BASE)
          32'h00: begin wdg_en_q <= d_wdata_q[0]; wdg_autorl_q <= d_wdata_q[1]; end
          32'h04: begin wdg_reload_q <= d_wdata_q; end
          32'h08: begin wdg_count_q  <= d_wdata_q; end
          32'h0C: begin if (|d_wdata_q) wdg_kick_pulse <= 1'b1; end
          32'h10: begin if (d_wdata_q[0]) wdg_fired_q <= 1'b0; end // W1C
          default: d_err_d = 1'b1;
        endcase
        dmem_rdata = 32'h0;
        d_rvalid_d = 1'b1;
        dstate_d   = D_RESP;
      end

      // --- External APB access (delegated to APB FSM) ---
      D_APB: begin
        // Wait until APB FSM reaches RESP
        if (astate_q == A_RESP) begin
          dmem_rdata = apb_rdata_q;
          d_rvalid_d = 1'b1;
          d_err_d    = pslverr_i; // from last access
          dstate_d   = D_RESP;
        end
      end

      D_RESP: begin
        // Single beat responses; drop back to IDLE
        dstate_d = D_IDLE;
      end

      default: dstate_d = D_IDLE;
    endcase

    // ---------------- APB FSM ----------------
    case (astate_q)
      A_IDLE: begin
        if (dstate_q == D_APB) begin
          if (apb_do_rmw) begin
            // First do READ
            apb_we_d   = 1'b0;
            astate_d   = A_SETUP;
          end else begin
            // Direct access
            astate_d   = A_SETUP;
          end
        end
      end

      A_SETUP: begin
        // setup phase (PSEL=1, PENABLE=0)
        astate_d = A_ACCESS;
      end

      A_ACCESS: begin
        // access phase (PSEL=1, PENABLE=1)
        if (pready_i) begin
          if (!apb_we_q && (dstate_q == D_APB) && (apb_be_q != 4'hF) && apb_do_rmw) begin
            // RMW path: merge PRDATA with write mask and go write
            apb_rmw_mask  = { {8{apb_be_q[3]}}, {8{apb_be_q[2]}}, {8{apb_be_q[1]}}, {8{apb_be_q[0]}} };
            apb_rmw_wdata = (apb_wdata_q & apb_rmw_mask) | (prdata_i & ~apb_rmw_mask);

            // Prepare write
            apb_we_d      = 1'b1;
            apb_wdata_d   = apb_rmw_wdata;
            astate_d      = A_RMW_SETUPW;
          end else begin
            astate_d = A_RESP;
          end
        end
      end

      A_RMW_SETUPW: begin
        // setup for RMW write
        astate_d = A_RMW_ACCESSW;
      end

      A_RMW_ACCESSW: begin
        if (pready_i) begin
          astate_d = A_RESP;
        end
      end

      A_RESP: begin
        // Single transfer complete, drop back
        astate_d = A_IDLE;
      end

      default: astate_d = A_IDLE;
    endcase
  end

  // ==========================================================================
  // Instruction/Data illegal address assertions (simulation)
  // ==========================================================================
`ifdef ASSERT_ON
  // Optional: warn on instruction fetch to external region
  assert property (@(posedge clk_i) disable iff (!rstn_i)
    !(imem_req && imem_hit_ext)) else
      $warning("riscv_subsys: IFetch to external APB region (unsupported).");

  // Data: only one outstanding transaction supported
  // Enforced by dmem_gnt in D_IDLE only.
`endif

endmodule

// ============================================================================
//  rv32_core_stub — safe no-op core for early integration
//  - Asserts no bus requests; ignores interrupts; keeps system quiescent.
// ============================================================================
module rv32_core_stub (
  input  logic        clk_i,
  input  logic        rstn_i,

  // IMEM
  output logic        imem_req_o,
  input  logic        imem_gnt_i,
  input  logic        imem_rvalid_i,
  output logic [31:0] imem_addr_o,
  input  logic [31:0] imem_rdata_i,
  input  logic        imem_err_i,

  // DMEM
  output logic        dmem_req_o,
  input  logic        dmem_gnt_i,
  input  logic        dmem_rvalid_i,
  output logic        dmem_we_o,
  output logic [3:0]  dmem_be_o,
  output logic [31:0] dmem_addr_o,
  output logic [31:0] dmem_wdata_o,
  input  logic [31:0] dmem_rdata_i,
  input  logic        dmem_err_i,

  input  logic        ext_irq_i
);
  always_ff @(posedge clk_i or negedge rstn_i) begin
    if (!rstn_i) begin
      imem_req_o  <= 1'b0;
      imem_addr_o <= 32'h0;
      dmem_req_o  <= 1'b0;
      dmem_we_o   <= 1'b0;
      dmem_be_o   <= 4'h0;
      dmem_addr_o <= 32'h0;
      dmem_wdata_o<= 32'h0;
    end else begin
      // remain idle
    end
  end
endmodule

`default_nettype wire
