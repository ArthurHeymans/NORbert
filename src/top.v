/*
 * Tang Primer 25K SPI Flash Emulator
 * 
 * Ported from Tang Nano 20K version. Uses external SDRAM module on dock
 * with two MT48LC16M16A2 chips (64MB total) via 16-bit shared bus.
 *
 * SPI wiring on dock GPIO header pins:
 *   A5 = CS
 *   B5 = CLK  
 *   A6 = MOSI
 *   B6 = MISO
 *   A7 = POWER detection (active high)
 *   B7 = Debug output
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
    input wire spi_mosi_pin,      // Master Out Slave In
    output wire spi_miso_pin,     // Master In Slave Out
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
    wire clk_133_sdram;  // Pre-IODELAY clock for SDRAM output
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
    
    // PLL: 50MHz -> ~132MHz (main) + phase-shifted (SDRAM clk) + aux (read capture)
    pll pll_i(
        .clkin(clk_50mhz),
        .clkout(clk_133),
        .clkoutp(aux_clk),
        .locked(pll_locked)
    );
    
    wire clk = clk_133;
    
    // SDRAM clock output via IODELAY for phase adjustment
    // The IODELAY adds a static delay (~0.8ns at C_STATIC_DLY=64) to align
    // the SDRAM clock edge with data setup/hold requirements.
    IODELAY sdram_clk_delay(
        .DO(O_sdram_clk),
        .DF(),
        .DI(clk_133),
        .SDTAP(1'b0),
        .VALUE(1'b0),
        .DLYSTEP(8'b0)
    );
    defparam sdram_clk_delay.C_STATIC_DLY = 64;
    defparam sdram_clk_delay.DYN_DLY_EN = "FALSE";
    defparam sdram_clk_delay.ADAPT_EN = "FALSE";
    
    // -----------------------------------------------------------
    // SDRAM Data Bus Handling
    // -----------------------------------------------------------
    
    wire [63:0] sdram_read_buffer;
    wire sdram_read_busy;
    wire [63:0] sdram_write_buffer;
    
    // Internal signals for SDRAM controller
    wire [12:0] sdram_addr;
    wire [15:0] sdram_dq_i;
    wire [15:0] sdram_dq_o;
    wire sdram_dq_oe;
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
    
    // Bidirectional data bus
    genvar gi;
    generate
        for (gi = 0; gi < 16; gi = gi + 1) begin : sdram_dq_gen
            assign IO_sdram_dq[gi] = sdram_dq_oe ? sdram_dq_o[gi] : 1'bz;
        end
    endgenerate
    assign sdram_dq_i = IO_sdram_dq;
    
    // -----------------------------------------------------------
    // SPI Interface
    // -----------------------------------------------------------
    
    // SPI input signals
    wire spi_cs_in = spi_cs_pin;
    wire spi_clk_in = spi_clk_pin;
    wire spi_mosi_in = spi_mosi_pin;
    wire spi_power_in = spi_power_pin;
    
    // SPI output signals
    wire spi_miso_out;
    wire spi_miso_enable;
    wire spi_debug_out;
    
    // MISO output - active when enabled and selected
    assign spi_miso_pin = (spi_miso_enable && !spi_cs_in && spi_power_in) ? spi_miso_out : 1'b0;
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
        .spi_mosi(spi_mosi_in),
        .spi_miso(spi_miso_out),
        .spi_miso_enable(spi_miso_enable),
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
        .CLK_FREQ_MHZ(132),
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
        .dq_i(sdram_dq_i),
        .dq_o(sdram_dq_o),
        .dqm_o(sdram_dqm),
        .dq_oe_o(sdram_dq_oe),
        
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
    
    // UART for 3 Mbaud at 132MHz: 132MHz / 3M = 44
    uart #(
        .DIVISOR(44),
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
