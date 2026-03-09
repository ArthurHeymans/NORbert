// Glue Logic Module
// Handles serial protocol, SPI erase/page program, and SDRAM coordination
//
// Ported for Tang Primer 25K with 64MB external SDRAM (23-bit burst addresses)
// Serial path uses 25-bit access_addr = {chip, row[12:0], bank[1:0], col[8:0]}

`default_nettype none

module glue(
    input wire clk,
    input wire reset,

    input wire rxd_strobe,
    input wire [7:0] rxd_data,

    input wire txd_ready,
    output reg txd_strobe,
    output reg [7:0] txd_data,

    // SDRAM control signals
    output reg [1:0] sdram_access_cmd, // 00=nop 01=read 10=write 11=activate
    output reg [24:0] sdram_access_addr,  // 25-bit: {chip, row, bank, col}
    output reg sdram_inhibit_refresh,
    input wire sdram_cmd_busy,

    input wire [63:0] sdram_read_buffer,
    input wire sdram_read_busy,

    output reg [63:0] sdram_write_buffer,
    
    // SPI signals
    input wire spi_reset,
    input wire spi_csel,
    
    input wire spi_cmd_write,
    input wire spi_write_type,  // 0=page program, 1=erase (sector/block/chip)
    input wire [22:0] spi_write_addr,   // 23-bit burst address
    input wire [19:0] spi_write_len,
    output reg spi_write_done,
    
    input wire spi_write_buf_strobe,
    input wire [7:0] spi_write_buf_offset,
    input wire [7:0] spi_write_buf_val,
    
    input wire log_strobe,
    input wire [7:0] log_val,
    
    output reg [7:0] led
);

    // Serial protocol commands
    localparam
        CMD_NOP          = 8'h00,
        CMD_VERSION      = 8'h30,
        CMD_RAMREAD      = 8'h31,
        CMD_RAMWRITE     = 8'h32;

    localparam VERSION = 8'h02;  // Version 2 for Tang Primer 25K port

    reg [7:0] cmd;
    reg [3:0] in_count;
    
    // Idle timeout: reset serial parser if no byte received within ~137us
    // while in middle of a multi-byte command. Handles spurious bytes
    // from USB-UART bridge on port open/close.
    // At 120MHz, 2^14 = 16384 cycles = ~137us
    // This is long enough for back-to-back UART bytes within a USB
    // transfer (~3.3us per byte at 3Mbaud) but short enough to recover
    // quickly from spurious bytes before the tool's real command arrives
    // via USB (~125us for high-speed, ~1ms for full-speed).
    // The tool should combine command headers and data into single
    // write() calls to avoid mid-command USB transfer gaps.
    reg [14:0] serial_idle_count;

    reg [22:0] addr;       // 23-bit burst address
    reg [7:0] len;

    reg [2:0] read_state;
    reg [2:0] read_pos;

    reg [2:0] write_state;
    reg [2:0] write_pos;

    reg txd_strobe_buf;
    reg [7:0] txd_data_buf;
    
    reg rxd_strobe_buf;
    reg [7:0] rxd_data_buf;

    wire sdram_busy = (sdram_access_cmd != 0) || sdram_cmd_busy;
    
    reg [63:0] write_buffer;
    reg write_strobe;
    
    reg [1:0] log_strobe_buf;
    always @(posedge clk) log_strobe_buf <= {log_strobe_buf[0], log_strobe};
    reg log_ack;
    
    reg [1:0] spi_csel_buf;
    
    // Heartbeat counter
    reg [25:0] heartbeat;
    
    reg spi_writing;
    reg spi_write_ack;
    reg [1:0] spi_cmd_write_buf;
    
    reg i_spi_write_type;
    reg [3:0] i_spi_write_state;
    reg [19:0] i_spi_len;
    
    // Page program buffer (256 bytes + write flags)
    reg [8:0] i_spi_write_data [0:255];
    reg [1:0] spi_write_buf_strobe_buf;
    reg spi_write_buf_ack;
    
    reg [7:0] spi_write_buf_offset_reg;
    reg [7:0] spi_write_buf_val_reg;

    // Convert 23-bit burst address to 25-bit access_addr for SDRAM controller
    // Burst address: [22]=chip, [21:9]=row, [8:7]=bank, [6:0]=col_burst
    // Access addr:   [24]=chip, [23:11]=row, [10:9]=bank, [8:0]=col
    // col = {col_burst[6:0], 2'b00} (burst-4 aligned)
    wire [24:0] addr_to_access = {addr[22], addr[21:9], addr[8:7], addr[6:0], 2'b00};

    integer i;

    always @(posedge clk) begin
        if (reset) begin
            cmd <= CMD_NOP;
            in_count <= 0;
            serial_idle_count <= 0;
            addr <= 0;
            len <= 0;

            read_state <= 0;
            read_pos <= 0;

            write_state <= 0;
            write_pos <= 0;

            sdram_access_cmd <= 0;
            sdram_inhibit_refresh <= 0;

            write_strobe <= 0;

            txd_strobe_buf <= 0;
            txd_data_buf <= 0;
            
            rxd_strobe_buf <= 0;
            rxd_data_buf <= 0;
            
            led <= 0;
            log_ack <= 0;
            
            spi_writing <= 0;
            spi_write_ack <= 0;
            spi_cmd_write_buf <= 0;
            spi_write_done <= 0;
            i_spi_write_type <= 0;
            i_spi_write_state <= 0;
            i_spi_len <= 0;
            
            spi_write_buf_strobe_buf <= 0;
            spi_write_buf_ack <= 0;
            spi_write_buf_offset_reg <= 0;
            spi_write_buf_val_reg <= 0;
            
            write_buffer <= 0;

            for (i = 0; i < 256; i = i + 1)
                i_spi_write_data[i][8] <= 0;
        end
        else begin
            txd_strobe_buf <= 0;
            txd_strobe <= txd_strobe_buf;
            txd_data <= txd_data_buf;
            
            rxd_strobe_buf <= rxd_strobe;
            rxd_data_buf <= rxd_data;
            
            sdram_access_addr <= addr_to_access;
            sdram_write_buffer <= write_buffer;
            sdram_inhibit_refresh <= 0;
    
            if (sdram_access_cmd)
                sdram_access_cmd <= 0;
                
            spi_csel_buf <= {spi_csel_buf[0], spi_csel};
            heartbeat <= heartbeat + 1;

            led[7] <= !spi_reset && !spi_csel_buf[1];  // SPI active
            led[6] <= sdram_cmd_busy;
            led[5] <= spi_writing;
            led[4] <= spi_reset;
            led[3] <= !spi_csel_buf[1];
            led[0] <= heartbeat[25];                    // Heartbeat ~2Hz at 132MHz
            
            // Log strobe handling
            if (log_strobe_buf[1] && !log_ack) begin
                txd_strobe_buf <= 1;
                txd_data_buf <= log_val;
                log_ack <= 1;
            end
            if (!log_strobe_buf[1]) log_ack <= 0;
                
            // SPI write buffer handling
            spi_write_buf_strobe_buf <= {spi_write_buf_strobe_buf[0], spi_write_buf_strobe};
            
            if (spi_write_buf_strobe_buf[0] && !spi_write_buf_strobe_buf[1]) begin
                spi_write_buf_offset_reg <= spi_write_buf_offset;
                spi_write_buf_val_reg <= spi_write_buf_val;
            end
            
            if (!spi_write_buf_strobe_buf[1])
                spi_write_buf_ack <= 0;
                
            if (spi_write_buf_strobe_buf[1] && !spi_write_buf_ack) begin
                i_spi_write_data[spi_write_buf_offset_reg] <= {1'b1, spi_write_buf_val_reg};
                spi_write_buf_ack <= 1;
            end
            
            // SPI write command handling    
            spi_cmd_write_buf <= {spi_cmd_write_buf[0], spi_cmd_write};
            
            if (!spi_cmd_write_buf[1])
                spi_write_ack <= 0;

            if (spi_cmd_write_buf[1] && !spi_write_ack && spi_csel_buf[1]) begin
                spi_writing <= 1;
                spi_write_ack <= 1;
                
                i_spi_write_type <= spi_write_type;
                i_spi_write_state <= (spi_write_type != 0) ? 2 : 0;
                
                addr <= spi_write_addr;
                i_spi_len <= spi_write_len;
                spi_write_done <= 0;
                
                if (spi_write_type != 0)
                    write_buffer <= 64'hFFFFFFFFFFFFFFFF;
            end
            
            // SPI write state machine
            if (spi_writing && !sdram_busy) begin
                if (i_spi_write_state == 0) begin
                    // Page program: prepare data from page buffer
                    for (i = 0; i < 8; i = i + 1) begin
                        write_buffer[i*8 +: 8] <= i_spi_write_data[{addr[4:0], 3'b000} + i][7:0];
                    end
                    i_spi_write_state <= 1;
                end
                else if (i_spi_write_state == 1) begin
                    i_spi_write_state <= 2;
                end
                else if (i_spi_write_state == 2) begin
                    // Activate for write
                    sdram_access_cmd <= 2'b11;
                    i_spi_write_state <= 3;
                end
                else if (i_spi_write_state == 3) begin
                    // Write burst to SDRAM
                    sdram_access_cmd <= 2'b10;
                    
                    if (i_spi_len == 0) begin
                        i_spi_write_state <= 5;
                    end
                    else begin
                        i_spi_write_state <= 4;
                    end
                end
                else if (i_spi_write_state == 4) begin
                    i_spi_write_state <= (i_spi_write_type != 0) ? 2 : 0;
                    addr <= addr + 1;
                    i_spi_len <= i_spi_len - 1;
                end
                else if (i_spi_write_state == 5) begin
                    spi_writing <= 0;
                    spi_write_done <= 1;
                end
            end
            
            // Serial protocol idle timeout: reset parser if stuck in
            // multi-byte command with no data for ~1ms
            if (in_count != 0 && !rxd_strobe_buf) begin
                serial_idle_count <= serial_idle_count + 1;
                if (serial_idle_count[14]) begin
                    in_count <= 0;
                    cmd <= CMD_NOP;
                    read_state <= 0;
                    write_state <= 0;
                    write_strobe <= 0;
                end
            end
            
            // Serial protocol handling
            if ((spi_reset || spi_csel_buf[1]) && !spi_writing) begin
                
                if (rxd_strobe_buf) begin
                    serial_idle_count <= 0;

                    if (in_count == 0) begin
                        if (rxd_data_buf == CMD_VERSION) begin
                            txd_strobe_buf <= 1;
                            txd_data_buf <= VERSION;
                            in_count <= 0;
                        end
                        else if (rxd_data_buf == CMD_RAMREAD ||
                                 rxd_data_buf == CMD_RAMWRITE) begin
                            cmd <= rxd_data_buf;
                            in_count <= 1;

                            read_state <= 0;
                            read_pos <= 0;

                            write_state <= 0;
                            write_pos <= 0;
                        end
                    end
                    else begin
                        // Address bytes: 3 bytes for 23-bit burst address
                        // (shifted in MSB first)
                        if (in_count <= 3)
                            addr <= {addr[14:0], rxd_data_buf};
                        else if (in_count == 4)
                            len <= rxd_data_buf;

                        if (cmd == CMD_RAMREAD && in_count == 4) begin
                            read_state <= 1;
                        end
                        if (cmd == CMD_RAMWRITE && in_count > 4) begin
                            write_buffer[write_pos*8 +: 8] <= rxd_data_buf;
                            
                            if (write_pos == 7) begin
                                write_strobe <= 1;
                            end
                            write_pos <= write_pos + 1;
                        end

                        if (in_count <= 4)
                            in_count <= in_count + 1;
                    end

                end
                else begin

                    if (write_strobe && !sdram_busy)
                        write_state <= 1;

                    if (read_state) begin
                        if ((read_state == 1) && !sdram_busy) begin
                            // Activate
                            sdram_access_cmd <= 2'b11;
                            read_state <= 2;
                        end
                        else if ((read_state == 2) && !sdram_busy) begin
                            // Read
                            sdram_access_cmd <= 2'b01;
                            read_state <= 3;
                        end
                        else if ((read_state == 3) && !sdram_busy && txd_ready) begin
                            txd_strobe_buf <= 1;
                            txd_data_buf <= sdram_read_buffer[read_pos*8 +: 8];

                            if (read_pos == 7) begin
                                if (len == 1) begin
                                    read_state <= 0;
                                    in_count <= 0;
                                    cmd <= CMD_NOP;
                                end
                                else begin
                                    addr <= addr + 1;
                                    len <= len - 1;
                                    read_state <= 1;
                                    read_pos <= 0;
                                end
                            end
                            else
                                read_pos <= read_pos + 1;
                        end
                    end
                    else if (write_state) begin
                        if ((write_state == 1) && !sdram_busy) begin
                            // Activate
                            sdram_access_cmd <= 2'b11;
                            write_strobe <= 0;
                            write_state <= 2;
                        end
                        else if ((write_state == 2) && !sdram_busy) begin
                            // Write
                            sdram_access_cmd <= 2'b10;
                            write_state <= 3;
                        end
                        else if ((write_state == 3) && !sdram_busy) begin
                            if (len == 1) begin
                                if (txd_ready) begin
                                    txd_strobe_buf <= 1;
                                    txd_data_buf <= 8'h01;
                                    write_state <= 0;
                                    in_count <= 0;
                                    cmd <= CMD_NOP;
                                end
                            end
                            else begin
                                write_state <= 0;
                                addr <= addr + 1;
                                len <= len - 1;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule
