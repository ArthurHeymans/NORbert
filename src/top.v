/*
 * Tang Primer 25K SPI Flash Emulator
 * 
 * Ported from Tang Nano 20K version. Uses external SDRAM module on dock
 * with two W9825G6KH chips (64MB total) via 16-bit shared bus.
 *
 * SPI wiring on dock PMOD connector (bank 6):
 *   A11 = CS
 *   E11 = CLK  
 *   C11 = IO0 (MOSI / bidirectional in dual/quad mode)
 *   D11 = IO1 (MISO / bidirectional in dual/quad mode)
 *   B11 = IO2 (/WP / bidirectional in quad mode)
 *   C10 = IO3 (/HOLD / bidirectional in quad mode)
 *   A10 = POWER detection (active high)
 *   E10 = Debug output
 *
 * Supported flash chips (selected via FLASH_CHIP parameter):
 *   0 = Winbond W25Q64FV (8MB, default)
 *   1 = Micron N25Q256A (32MB)
 */
`default_nettype none

`ifndef FLASH_CHIP
`define FLASH_CHIP 0
`endif

module top #(
    parameter FLASH_CHIP = `FLASH_CHIP
)(
    input wire clk_50mhz,         // 50MHz crystal oscillator
    
    // LED
    output wire led,              // Active low LED (single LED on core board)
    
    // User buttons
    input wire btn_s1,
    input wire btn_s2,
    
    // UART via FTDI debugger on dock
    input wire uart_rx,           // FPGA receives from FTDI
    output wire uart_tx,          // FPGA transmits to FTDI
    
    // SPI flash emulation interface (on dock GPIO header)
    input wire spi_cs_pin,        // Active low chip select
    input wire spi_clk_pin,       // SPI clock from master
    inout wire spi_mosi_pin,      // IO0: MOSI / bidirectional in dual/quad mode
    inout wire spi_miso_pin,      // IO1: MISO / bidirectional in dual/quad mode
    inout wire spi_io2_pin,       // IO2: /WP / bidirectional in quad mode
    inout wire spi_io3_pin,       // IO3: /HOLD / bidirectional in quad mode
    input wire spi_power_pin,     // Power detection
    output wire spi_debug_pin,    // Debug output
    
    // External SDRAM interface (via dock 40-pin connector)
    output wire        O_sdram_clk,
    output wire        O_sdram_cs_n,    // Single CS: LOW=chip0, HIGH=chip1
    output wire        O_sdram_cas_n,
    output wire        O_sdram_ras_n,
    output wire        O_sdram_wen_n,
    output wire [12:0] O_sdram_addr,
    output wire [1:0]  O_sdram_ba,
    output wire [1:0]  O_sdram_dqm,
    inout wire [15:0]  IO_sdram_dq
);

    // -----------------------------------------------------------
    // Clock and Reset
    // -----------------------------------------------------------
    
    wire clk_133;
    wire clk_133_sdram;  // Dedicated PLL output for SDRAM clock (phase-shifted)
    wire aux_clk;        // Phase-shifted clock for SDRAM read capture
    wire pll_locked;
    
    // Reset logic - active for ~1ms after PLL lock
    reg [16:0] reset_cnt = 0;
    wire reset = !reset_cnt[16];
    
    always @(posedge clk_133) begin
        if (!pll_locked)
            reset_cnt <= 0;
        else if (!reset_cnt[16])
            reset_cnt <= reset_cnt + 1;
    end
    
    // PLL: 50MHz -> 120MHz (main) + 120MHz (SDRAM clock) + 120MHz (aux/read capture)
    // CRITICAL: SDRAM clock uses a SEPARATE PLL output with PE_COARSE=9 phase shift.
    // IODELAY does not work for clock outputs on this device (GW5A), so all clock
    // phase adjustment is done via PLL PE_COARSE parameters instead.
    pll pll_i(
        .clkin(clk_50mhz),
        .clkout(clk_133),
        .clkout_sdram(clk_133_sdram),
        .clkoutp(aux_clk),
        .locked(pll_locked)
    );
    
    wire clk = clk_133;
    
    // SDRAM clock output: use PE_COARSE=9 on dedicated PLL output (no inversion)
    // This gives ~3.75ns delay which provides SDRAM write setup time.
    assign O_sdram_clk = clk_133_sdram;
    
    // -----------------------------------------------------------
    // SDRAM Data Bus Handling
    // -----------------------------------------------------------
    
    wire [63:0] sdram_read_buffer;
    wire sdram_read_busy;
    wire [63:0] sdram_write_buffer;
    
    // Internal signals for SDRAM controller
    wire [12:0] sdram_addr;
    wire [1:0] sdram_dqm;
    wire [1:0] sdram_ba;
    wire sdram_cs_n;
    wire sdram_ras_n;
    wire sdram_cas_n;
    wire sdram_we_n;
    
    // Connect to SDRAM interface signals
    assign O_sdram_cs_n = sdram_cs_n;
    assign O_sdram_cas_n = sdram_cas_n;
    assign O_sdram_ras_n = sdram_ras_n;
    assign O_sdram_wen_n = sdram_we_n;
    assign O_sdram_addr = sdram_addr;
    assign O_sdram_ba = sdram_ba;
    assign O_sdram_dqm = sdram_dqm;
    
    // -----------------------------------------------------------
    // SPI Interface (dual IO capable)
    // -----------------------------------------------------------
    
    // SPI input signals
    wire spi_cs_in = spi_cs_pin;
    wire spi_clk_in = spi_clk_pin;
    wire spi_power_in = spi_power_pin;
    
    // Bidirectional IO signals from spi_trx
    wire spi_io0_out;   // IO0 output data (active in dual/quad read)
    wire spi_io0_oe;    // IO0 output enable
    wire spi_io1_out;   // IO1 output data (MISO in single, IO1 in dual/quad)
    wire spi_io1_oe;    // IO1 output enable
    wire spi_io2_out;   // IO2 output data (active in quad read)
    wire spi_io2_oe;    // IO2 output enable
    wire spi_io3_out;   // IO3 output data (active in quad read)
    wire spi_io3_oe;    // IO3 output enable
    wire spi_debug_out;
    
    // IO0 (MOSI pin): input normally, output during dual/quad read data phase
    wire spi_active_out = !spi_cs_in && spi_power_in;
    assign spi_mosi_pin = (spi_io0_oe && spi_active_out) ? spi_io0_out : 1'bz;
    wire spi_io0_in = spi_mosi_pin;
    
    // IO1 (MISO pin): output during single-mode reads and dual/quad read data phase
    assign spi_miso_pin = (spi_io1_oe && spi_active_out) ? spi_io1_out : 1'bz;
    wire spi_io1_in = spi_miso_pin;
    
    // IO2 (/WP pin): input normally, output during quad read data phase
    assign spi_io2_pin = (spi_io2_oe && spi_active_out) ? spi_io2_out : 1'bz;
    wire spi_io2_in = spi_io2_pin;
    
    // IO3 (/HOLD pin): input normally, output during quad read data phase
    assign spi_io3_pin = (spi_io3_oe && spi_active_out) ? spi_io3_out : 1'bz;
    wire spi_io3_in = spi_io3_pin;
    
    assign spi_debug_pin = spi_debug_out;
    
    // SPI power detection and reset logic
    reg [1:0] spi_power_reg;
    reg spi_reset = 1;
    reg [16:0] spi_reset_count = 0;
    
    // Set to 1 to bypass power detection (for debugging without power pin connected)
    localparam BYPASS_POWER_DETECT = 1;
    
    wire power_ok = BYPASS_POWER_DETECT ? 1'b1 : spi_power_reg[1];
    
    always @(posedge clk) begin
        spi_power_reg <= {spi_power_reg[0], spi_power_in};
        
        if (!reset && power_ok) begin
            if (spi_reset && spi_reset_count[16]) begin
                spi_reset <= 0;
                spi_reset_count <= 0;
            end else
                spi_reset_count <= spi_reset_count + 1;
        end else begin
            spi_reset <= 1;
            spi_reset_count <= 0;
        end
    end
    
    // -----------------------------------------------------------
    // SPI Transceiver
    // -----------------------------------------------------------
    
    wire spi_active;
    wire spi_ram_inhibit_refresh;
    wire spi_ram_activate;
    wire spi_ram_read;
    wire [22:0] spi_ram_addr;    // 23-bit for 64MB addressing
    
    wire spi_write_cmd;
    wire spi_write_type;  // 0=page program, 1=erase (sector/block/chip)
    wire [22:0] spi_write_addr;  // 23-bit
    wire [19:0] spi_write_len;
    wire spi_write_done;
    
    wire spi_write_buf_strobe;
    wire [7:0] spi_write_buf_offset;
    wire [7:0] spi_write_buf_val;
    
    wire log_strobe;
    wire [7:0] log_val;
    
    spi_trx #(
        .FLASH_CHIP(FLASH_CHIP)
    ) spi_trx_i (
        .clk(clk),
        
        .spi_clk(spi_clk_in),
        .spi_reset(spi_reset),
        .spi_csel(spi_cs_in),
        .spi_io0_in(spi_io0_in),
        .spi_io0_out(spi_io0_out),
        .spi_io0_oe(spi_io0_oe),
        .spi_io1_in(spi_io1_in),
        .spi_io1_out(spi_io1_out),
        .spi_io1_oe(spi_io1_oe),
        .spi_io2_in(spi_io2_in),
        .spi_io2_out(spi_io2_out),
        .spi_io2_oe(spi_io2_oe),
        .spi_io3_in(spi_io3_in),
        .spi_io3_out(spi_io3_out),
        .spi_io3_oe(spi_io3_oe),
        .spi_debug(spi_debug_out),
        
        .spi_active(spi_active),
        
        .ram_inhibit_refresh(spi_ram_inhibit_refresh),
        .ram_activate(spi_ram_activate),
        .ram_read(spi_ram_read),
        
        .ram_addr(spi_ram_addr),
        .ram_read_buffer(sdram_read_buffer),
        .ram_read_busy(sdram_read_busy),
        
        .write_cmd(spi_write_cmd),
        .write_type(spi_write_type),
        .write_addr(spi_write_addr),
        .write_len(spi_write_len),
        .write_done(spi_write_done),
        
        .write_buf_strobe(spi_write_buf_strobe),
        .write_buf_offset(spi_write_buf_offset),
        .write_buf_val(spi_write_buf_val),
        
        .log_strobe(log_strobe),
        .log_val(log_val)
    );
    
    // -----------------------------------------------------------
    // SDRAM Controller
    // -----------------------------------------------------------
    
    wire [1:0] sdram_access_cmd;
    wire [24:0] sdram_access_addr;   // 25-bit for serial path
    wire sdram_inhibit_refresh;
    wire sdram_cmd_busy;
    
    sdram #(
        .CLK_FREQ_MHZ(120),
        .BURST_LEN(4)
    ) sdram_i (
        .clk(clk),
        .aux_clk(aux_clk),
        .reset(reset),
        
        // Physical interface
        .ba_o(sdram_ba),
        .a_o(sdram_addr),
        .cs_o(sdram_cs_n),
        .ras_o(sdram_ras_n),
        .cas_o(sdram_cas_n),
        .we_o(sdram_we_n),
        .dqm_o(sdram_dqm),
        .dq_io(IO_sdram_dq),
        
        // SPI fast-path control
        .spi_inhibit_refresh(spi_ram_inhibit_refresh),
        .spi_cmd_activate(spi_ram_activate),
        .spi_cmd_read(spi_ram_read),
        .spi_addr(spi_ram_addr),
        
        // Serial path control
        .access_cmd(sdram_access_cmd),
        .access_addr(sdram_access_addr),
        .inhibit_refresh(sdram_inhibit_refresh),
        .cmd_busy(sdram_cmd_busy),
        
        .read_buffer(sdram_read_buffer),
        .read_busy(sdram_read_busy),
        
        .write_buffer(sdram_write_buffer)
    );
    
    // -----------------------------------------------------------
    // UART Interface
    // -----------------------------------------------------------
    
    wire uart_txd_ready;
    wire [7:0] uart_txd;
    wire uart_txd_strobe;
    wire uart_rxd_strobe;
    wire [7:0] uart_rxd;
    
    // UART for 2 Mbaud at 120MHz: 120MHz / 2M = 60
    // DIVISOR must be divisible by 4 (uart_rx uses DIVISOR/4 for 4x oversampling)
    uart #(
        .DIVISOR(60),
        .FIFO(256),
        .FREESPACE(16)
    ) uart_i(
        .clk(clk),
        .reset(reset),
        .serial_txd(uart_tx),
        .serial_rxd(uart_rx),
        .txd(uart_txd),
        .txd_strobe(uart_txd_strobe),
        .txd_ready(uart_txd_ready),
        .rxd(uart_rxd),
        .rxd_strobe(uart_rxd_strobe)
    );
    
    // -----------------------------------------------------------
    // Glue Logic
    // -----------------------------------------------------------
    
    wire [7:0] led_out;
    
    glue glue_i(
        .clk(clk),
        .reset(reset),
        
        .rxd_strobe(uart_rxd_strobe),
        .rxd_data(uart_rxd),
        
        .txd_ready(uart_txd_ready),
        .txd_strobe(uart_txd_strobe),
        .txd_data(uart_txd),
        
        .sdram_access_cmd(sdram_access_cmd),
        .sdram_access_addr(sdram_access_addr),
        .sdram_inhibit_refresh(sdram_inhibit_refresh),
        .sdram_cmd_busy(sdram_cmd_busy),
        
        .sdram_read_buffer(sdram_read_buffer),
        .sdram_read_busy(sdram_read_busy),
        
        .sdram_write_buffer(sdram_write_buffer),
        
        .spi_reset(spi_reset),
        .spi_csel(spi_cs_in),
        
        .spi_cmd_write(spi_write_cmd),
        .spi_write_type(spi_write_type),
        .spi_write_addr(spi_write_addr),
        .spi_write_len(spi_write_len),
        .spi_write_done(spi_write_done),
        
        .spi_write_buf_strobe(spi_write_buf_strobe),
        .spi_write_buf_offset(spi_write_buf_offset),
        .spi_write_buf_val(spi_write_buf_val),
        
        .log_strobe(log_strobe),
        .log_val(log_val),
        
        .led(led_out)
    );
    
    // Single LED on Tang Primer 25K (active low)
    // Use heartbeat (bit 0) as primary indicator
    assign led = ~led_out[0];

endmodule
