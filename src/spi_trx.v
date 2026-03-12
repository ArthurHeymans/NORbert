// SPI Flash Transceiver Module
// Emulates a configurable SPI NOR flash chip.
// Chip identity (JEDEC ID, size, SFDP) is configured at runtime
// via the serial CHIPCONFIG command through glue.v.
//
// Ported for Tang Primer 25K with 64MB SDRAM (23-bit burst addresses)

`default_nettype none

module spi_trx(
    input wire clk,

    input wire spi_clk,
    input wire spi_reset,    // Active high
    input wire spi_csel,     // Active low chip select
    input wire spi_io0_in,   // IO0 input (directly from MOSI pin)
    output reg spi_io0_out = 0,  // IO0 output (driven in dual read)
    output wire spi_io0_oe,      // IO0 output enable (gated by reset_cs)
    input wire spi_io1_in,   // IO1 input (directly from MISO pin, used in 1-2-2 addr)
    output reg spi_io1_out = 0,  // IO1 output (MISO in single, IO1 in dual)
    output wire spi_io1_oe,      // IO1 output enable (gated by reset_cs)
    input wire spi_io2_in,   // IO2 input (/WP pin, used in 1-4-4 addr)
    output reg spi_io2_out = 0,  // IO2 output (driven in quad read)
    output wire spi_io2_oe,      // IO2 output enable (gated by reset_cs)
    input wire spi_io3_in,   // IO3 input (/HOLD pin, used in 1-4-4 addr)
    output reg spi_io3_out = 0,  // IO3 output (driven in quad read)
    output wire spi_io3_oe,      // IO3 output enable (gated by reset_cs)
    output reg spi_debug = 0,
    
    output wire spi_active,
    
    // SDRAM control signals
    output reg ram_inhibit_refresh = 0,
    output reg ram_activate = 0,
    output reg ram_read = 0,
    
    output reg [22:0] ram_addr,       // 23-bit burst address for 64MB
    input wire [63:0] ram_read_buffer,
    input wire ram_read_busy,
    
    // For writing
    output reg write_cmd,
    output reg write_type,  // 0=page program, 1=erase (sector/block/chip)
    output reg [22:0] write_addr,     // 23-bit burst address
    output reg [22:0] write_len,
    input wire write_done,
    
    output reg write_buf_strobe,
    output reg [7:0] write_buf_offset,
    output reg [7:0] write_buf_val,
    
    // Configuration inputs (from glue, updated via serial CHIPCONFIG command).
    // These are stable during SPI transactions (config only applied when CS high).
    input wire [23:0] cfg_jedec_id,
    input wire cfg_4byte,
    input wire [22:0] cfg_chip_erase_bursts,
    
    // SFDP table read interface (memory in glue.v)
    output reg [6:0] sfdp_raddr,
    input wire [7:0] sfdp_rdata,
    
    output reg log_strobe = 0,
    output reg [7:0] log_val = 0
);

    wire is_selected = !spi_reset && !spi_csel;
    
    assign spi_active = is_selected;
    
    // Internal OE registers — gated with reset_cs to prevent bus contention.
    // When CS deasserts (reset_cs goes high asynchronously), the output enables
    // are immediately deasserted combinationally, preventing the FPGA from
    // driving IO0/IO1 when the master starts the next SPI transaction.
    reg spi_io0_oe_ff = 0;
    reg spi_io1_oe_ff = 0;
    reg spi_io2_oe_ff = 0;
    reg spi_io3_oe_ff = 0;
    assign spi_io0_oe = spi_io0_oe_ff && !reset_cs && !reset_power;
    assign spi_io1_oe = spi_io1_oe_ff && !reset_cs && !reset_power;
    assign spi_io2_oe = spi_io2_oe_ff && !reset_cs && !reset_power;
    assign spi_io3_oe = spi_io3_oe_ff && !reset_cs && !reset_power;
    
    // Reset detection using async set flip-flops
    reg reset_cs = 1;
    reg reset_power = 1;
    
    // CS reset detection
    always @(posedge spi_clk or posedge spi_csel) begin
        if (spi_csel)
            reset_cs <= 1;
        else if (is_selected)
            reset_cs <= 0;
    end
    
    // Power reset detection
    always @(posedge spi_clk or posedge spi_reset) begin
        if (spi_reset)
            reset_power <= 1;
        else if (is_selected)
            reset_power <= 0;
    end

    reg [2:0] bit_count_in;
    reg [7:0] mosi_byte;
    reg [7:0] miso_byte;
    
    // JEDEC ID and 4-byte addressing are driven by cfg_jedec_id and cfg_4byte
    // from glue.v (configured at runtime via serial CHIPCONFIG command).
    
    // SPI Flash command definitions
    localparam
        CMD_WRITESTATUS     = 8'h01,  // Write Status Register (1-2 data bytes)
        CMD_PAGEPROGRAM     = 8'h02,
        CMD_READ            = 8'h03,
        CMD_WRITEDISABLE    = 8'h04,
        CMD_READSTATUS      = 8'h05,
        CMD_WRITEENABLE     = 8'h06,
        CMD_FASTREAD        = 8'h0B,
        CMD_FASTREAD_4B     = 8'h0C,
        CMD_READ_4B         = 8'h13,
        CMD_SECTORERASE_4K  = 8'h20,
        CMD_READSTATUS2     = 8'h35,  // Read Status Register 2
        CMD_DUALREAD        = 8'h3B,  // Dual Output Read (1-1-2)
        CMD_BLOCKERASE_32K  = 8'h52,
        CMD_CHIPERASE1      = 8'h60,
        CMD_QUADREAD        = 8'h6B,  // Quad Output Read (1-1-4)
        CMD_READID1         = 8'h9E,
        CMD_READSFDP        = 8'h5A,  // Read SFDP table
        CMD_READID2         = 8'h9F,
        CMD_4BYTEENABLE     = 8'hB7,
        CMD_DUALIOREAD      = 8'hBB,  // Dual I/O Read (1-2-2)
        CMD_CHIPERASE2      = 8'hC7,
        CMD_BLOCKERASE_64K  = 8'hD8,
        CMD_4BYTEDISABLE    = 8'hE9,
        CMD_QUADIOREAD      = 8'hEB,  // Quad I/O Read (1-4-4)
        CMD_LOG             = 8'hF2;
    
    // State machine states
    localparam
        STA_CMD             = 0,
        STA_READSTATUS      = 1,
        STA_ADDR_READ       = 2,
        STA_READ            = 3,
        STA_READID          = 4,
        STA_ADDR_WRITE      = 5,
        STA_WRITE           = 6,
        STA_ADDR_ERASE      = 7,
        STA_ERASE           = 8,
        STA_LOG             = 9,
        STA_DUMMY           = 10,
        STA_ADDR_READ_DUAL  = 11,  // Dual I/O address phase (1-2-2)
        STA_READ_DUAL       = 12,  // Dual data output phase (1-1-2 & 1-2-2)
        STA_MODE_MULTI      = 13,  // Mode+dummy phase for 0xBB/0xEB
        STA_ADDR_READ_QUAD  = 14,  // Quad I/O address phase (1-4-4)
        STA_READ_QUAD       = 15,  // Quad data output phase (1-1-4 & 1-4-4)
        STA_WRITESTATUS     = 16,  // Receive status register write data
        STA_READSFDP        = 17;  // SFDP data output phase
    
    reg [4:0] state;
    reg [2:0] dummy_count = 0;
    reg is_fast_read = 0;
    reg is_dual_read = 0;       // Set for both 0x3B and 0xBB
    reg is_quad_read = 0;       // Set for both 0x6B and 0xEB
    reg is_sfdp_read = 0;       // Set for CMD 0x5A (Read SFDP)
    reg [2:0] mode_count = 0;   // Mode+dummy counter (4 for dual, 6 for quad)
    
    reg [31:0] addr;
    reg [4:0] addr_count;
    reg addr_4byte;
    
    reg fresh_read = 0;
    reg [7:0] saved_last_byte;  // Byte 7 saved before SDRAM read overwrites buffer
    
    // Status registers
    reg [7:0] status_reg = 8'b00000000;
    reg [7:0] status_reg2 = 8'h02;     // QE bit (bit 1) default enabled
    
    // Synchronize write_done from system clock domain
    reg [2:0] write_done_sync;
    
    always @(posedge clk) begin
        write_done_sync <= {write_done_sync[1:0], write_done};
    end
    
    wire write_busy_clr = write_done_sync[2];

    // Main SPI state machine
    always @(posedge spi_clk) begin
        if (is_selected) begin
            fresh_read <= 0;
            
            if (status_reg[0] && write_busy_clr)
                status_reg[0] <= 0;
            
            if (reset_cs || reset_power) begin
                bit_count_in <= 6;
                mosi_byte <= {spi_io0_in, 7'b0};
                miso_byte <= 0;
                
                spi_io0_oe_ff <= 0;
                spi_io1_oe_ff <= 0;
                spi_io2_oe_ff <= 0;
                spi_io3_oe_ff <= 0;
                
                state <= STA_CMD;
                
                addr <= 0;
                addr_count <= 0;
                dummy_count <= 0;
                is_fast_read <= 0;
                is_dual_read <= 0;
                is_quad_read <= 0;
                is_sfdp_read <= 0;
                mode_count <= 0;
                
                log_strobe <= 0;
                log_val <= 0;
                
                ram_inhibit_refresh <= 0;
                ram_activate <= 0;
                ram_read <= 0;
                
                write_cmd <= 0;
                
                write_buf_strobe <= 0;
                
                if (reset_power) begin
                    status_reg[1:0] <= 2'b00;
                    addr_4byte <= 0;
                    
                    log_strobe <= 1;
                    log_val <= 8'hE2;
                end
            end
            else begin
                log_strobe <= 0;
                    
                write_buf_strobe <= 0;
                
                mosi_byte[bit_count_in] <= spi_io0_in;

                if ((state == STA_CMD) && (bit_count_in == 0)) begin
                    
                    case ({mosi_byte[7:1], spi_io0_in})
                        
                    CMD_READSTATUS: begin
                        state <= STA_READSTATUS;
                        spi_io1_oe_ff <= 1;
                        miso_byte <= status_reg;
                    end
                    
                    CMD_READSTATUS2: begin
                        state <= STA_READSTATUS;
                        spi_io1_oe_ff <= 1;
                        miso_byte <= status_reg2;
                    end
                    
                    CMD_WRITESTATUS: begin
                        if (status_reg[1]) begin
                            state <= STA_WRITESTATUS;
                            addr_count <= 0;
                        end
                    end
                    
                    CMD_WRITEDISABLE: begin
                        status_reg[1] <= 0;
                    end
                    
                    CMD_WRITEENABLE: begin
                        status_reg[1] <= 1;
                    end
                    
                    CMD_4BYTEENABLE: begin
                        if (status_reg[1] && cfg_4byte) addr_4byte <= 1;
                    end
                    
                    CMD_4BYTEDISABLE: begin
                        if (status_reg[1] && cfg_4byte) addr_4byte <= 0;
                    end
                        
                    CMD_READ: begin
                        state <= STA_ADDR_READ;
                        addr_count <= addr_4byte ? 31 : 23;
                    end
                    
                    CMD_READ_4B: begin
                        if (cfg_4byte) begin
                            state <= STA_ADDR_READ;
                            addr_count <= 31;
                        end
                    end
                    
                    CMD_FASTREAD: begin
                        state <= STA_ADDR_READ;
                        addr_count <= addr_4byte ? 31 : 23;
                        is_fast_read <= 1;
                    end
                    
                    CMD_FASTREAD_4B: begin
                        if (cfg_4byte) begin
                            state <= STA_ADDR_READ;
                            addr_count <= 31;
                            is_fast_read <= 1;
                        end
                    end
                    
                    // Dual Output Read (1-1-2): cmd(1), addr(1), 8 dummy, data(2)
                    CMD_DUALREAD: begin
                        state <= STA_ADDR_READ;
                        addr_count <= addr_4byte ? 31 : 23;
                        is_fast_read <= 1;   // Uses same dummy phase
                        is_dual_read <= 1;
                    end
                    
                    // Dual I/O Read (1-2-2): cmd(1), addr(2), mode+dummy(2), data(2)
                    CMD_DUALIOREAD: begin
                        state <= STA_ADDR_READ_DUAL;
                        addr_count <= addr_4byte ? 31 : 23;
                        is_dual_read <= 1;
                    end
                    
                    // Quad Output Read (1-1-4): cmd(1), addr(1), 8 dummy, data(4)
                    CMD_QUADREAD: begin
                        if (status_reg2[1]) begin  // QE required
                            state <= STA_ADDR_READ;
                            addr_count <= addr_4byte ? 31 : 23;
                            is_fast_read <= 1;   // Uses same dummy phase
                            is_quad_read <= 1;
                        end
                    end
                    
                    // Quad I/O Read (1-4-4): cmd(1), addr(4), mode+dummy(4), data(4)
                    CMD_QUADIOREAD: begin
                        if (status_reg2[1]) begin  // QE required
                            state <= STA_ADDR_READ_QUAD;
                            addr_count <= addr_4byte ? 31 : 23;
                            is_quad_read <= 1;
                        end
                    end

                    CMD_SECTORERASE_4K: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_ERASE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 23'h001FF; // 4KB = 512 × 8-byte bursts - 1
                        end
                    end
                    
                    CMD_BLOCKERASE_32K: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_ERASE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 23'h00FFF; // 32KB = 4096 × 8-byte bursts - 1
                        end
                    end
                    
                    CMD_BLOCKERASE_64K: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_ERASE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 23'h01FFF; // 64KB = 8192 × 8-byte bursts - 1
                        end
                    end
                    
                    CMD_CHIPERASE1,
                    CMD_CHIPERASE2: begin
                        if (status_reg[1]) begin
                            state <= STA_ERASE;
                            write_cmd <= 1;
                            write_type <= 1'd1;
                            write_addr <= 23'b0;
                            write_len <= cfg_chip_erase_bursts;
                            status_reg[1] <= 0;
                            status_reg[0] <= 1;
                        end
                    end
                    
                    CMD_PAGEPROGRAM: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR_WRITE;
                            addr_count <= addr_4byte ? 31 : 23;
                            write_len <= 23'h0001F; // 256 byte page = 32 × 8-byte bursts - 1
                        end
                    end
                    
                    CMD_READSFDP: begin
                        state <= STA_ADDR_READ;
                        addr_count <= 23;        // Always 3-byte address for SFDP
                        is_fast_read <= 1;       // 8 dummy clocks after address
                        is_sfdp_read <= 1;
                    end
                    
                    CMD_READID1,
                    CMD_READID2: begin
                        state <= STA_READID;
                        spi_io1_oe_ff <= 1;
                        miso_byte <= cfg_jedec_id[7:0];
                        addr_count <= 1;
                    end
                    
                    CMD_LOG: begin
                        state <= STA_LOG;
                    end
                        
                    endcase
                    
                    log_strobe <= 1;
                    log_val <= {mosi_byte[7:1], spi_io0_in};
                end
                else if ((state == STA_READSTATUS) && (bit_count_in == 0)) begin
                    miso_byte <= status_reg;
                end
                else if (state == STA_ADDR_READ) begin
                    // Receiving address bytes for read (single-bit, 1 bit/clock)
                    // ram_addr is 23-bit burst address = byte_addr[25:3]
                    // addr_count counts down from 23 (or 31 for 4-byte) to 0
                    
                    // SDRAM pipeline (skipped for SFDP reads which use internal table)
                    if (!is_sfdp_read) begin
                        if (addr_count == 7) begin
                            ram_inhibit_refresh <= 1;
                        end
                        else if (addr_count == 4) begin
                            ram_activate <= 1;
                            ram_addr[22:4] <= addr[25:7];
                        end
                        else if (addr_count == 3) begin
                            ram_read <= 1;
                            ram_addr[3:0] <= {addr[6:4], spi_io0_in};
                        end
                    end
                    
                    if (addr_count == 0) begin
                        if (!is_sfdp_read) begin
                            ram_inhibit_refresh <= 0;
                            ram_activate <= 0;
                            ram_read <= 0;
                        end

                        if (is_fast_read) begin
                            state <= STA_DUMMY;
                            dummy_count <= 7;
                        end
                        else begin
                            state <= STA_READ;
                            spi_io1_oe_ff <= 1;
                            fresh_read <= 1;
                        end
                    end
                    
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_io0_in};
                    end
                    
                    addr[addr_count] <= spi_io0_in;
                    addr_count <= addr_count - 1;
                end
                else if (state == STA_DUMMY) begin
                    if (dummy_count == 0) begin
                        if (is_sfdp_read) begin
                            // SFDP read: output from internal SFDP table.
                            // sfdp_raddr was preloaded at dummy_count==1,
                            // so sfdp_rdata is valid now.
                            state <= STA_READSFDP;
                            spi_io1_oe_ff <= 1;
                            miso_byte <= sfdp_rdata;
                        end
                        else if (is_quad_read) begin
                            state <= STA_READ_QUAD;
                            spi_io0_oe_ff <= 1;
                            spi_io1_oe_ff <= 1;
                            spi_io2_oe_ff <= 1;
                            spi_io3_oe_ff <= 1;
                            fresh_read <= 1;
                        end
                        else if (is_dual_read) begin
                            state <= STA_READ_DUAL;
                            spi_io0_oe_ff <= 1;
                            spi_io1_oe_ff <= 1;
                            fresh_read <= 1;
                        end
                        else begin
                            state <= STA_READ;
                            spi_io1_oe_ff <= 1;
                            fresh_read <= 1;
                        end
                    end
                    else begin
                        dummy_count <= dummy_count - 1;
                        // Preload SFDP address one cycle before transition
                        // so sfdp_rdata is valid when we enter STA_READSFDP.
                        if (dummy_count == 1 && is_sfdp_read)
                            sfdp_raddr <= addr[6:0];
                    end
                end
                else if (state == STA_READ) begin
                    // Advance the read
                    
                    if (addr[2:0] == 7) begin
                        // Reaching end of burst, start reading new one
                        
                        if (bit_count_in == 7) begin
                            ram_inhibit_refresh <= 1;
                        end
                        else if (bit_count_in == 4) begin
                            ram_activate <= 1;
                            ram_addr <= ram_addr + 1;
                        end
                        else if (bit_count_in == 3) begin
                            ram_read <= 1;
                        end
                        else if (bit_count_in == 0) begin
                            ram_inhibit_refresh <= 0;
                            ram_activate <= 0;
                            ram_read <= 0;
                            
                            fresh_read <= 1;
                        end
                    end
                    
                    if (bit_count_in == 0) begin
                        miso_byte <= ram_read_buffer[(addr[2:0]+1)*8 +: 8];
                        addr <= addr + 1;
                    end
                    
                    if (fresh_read) 
                        miso_byte <= ram_read_buffer[addr[2:0]*8 +: 8];
                end
                else if (state == STA_READID) begin
                    if (bit_count_in == 0) begin
                        if (addr_count < 3) begin
                            miso_byte <= cfg_jedec_id[addr_count*8 +: 8];
                            addr_count <= addr_count + 1;
                        end
                        else
                            miso_byte <= 0;
                    end
                end
                else if (state == STA_ADDR_ERASE) begin
                    if (addr_count == 0) begin
                        state <= STA_ERASE;
                        write_cmd <= 1;
                        write_type <= 1'd1;
                        
                        // Align address based on erase size
                        // write_addr is 23-bit burst address = byte_addr[25:3]
                        if (write_len == 20'h01FFF)
                            write_addr <= {addr[25:16], 13'b0};  // 64KB aligned
                        else if (write_len == 20'h00FFF)
                            write_addr <= {addr[25:15], 12'b0};  // 32KB aligned
                        else
                            write_addr <= {addr[25:12], 9'b0};   // 4KB aligned
                            
                        status_reg[1] <= 0;
                        status_reg[0] <= 1;
                    end
                    
                    addr[addr_count] <= spi_io0_in;
                    addr_count <= addr_count - 1;
                    
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_io0_in};
                    end
                end
                else if (state == STA_ADDR_WRITE) begin
                    if (addr_count == 0) begin
                        state <= STA_WRITE;
                        write_cmd <= 1;
                        write_type <= 0;
                        
                        // Page-aligned address in 8-byte burst units
                        // byte_addr[25:3] -> burst address, page = 256 bytes = 32 bursts
                        write_addr <= {addr[25:8], 5'b0};
                            
                        status_reg[1] <= 0;
                        status_reg[0] <= 1;
                    end
                    
                    addr[addr_count] <= spi_io0_in;
                    addr_count <= addr_count - 1;
                    
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_io0_in};
                    end
                end
                else if ((state == STA_WRITE) && (bit_count_in == 0)) begin
                    write_buf_strobe <= 1;
                    write_buf_offset <= addr[7:0];
                    write_buf_val <= {mosi_byte[7:1], spi_io0_in};
                    
                    addr[7:0] <= addr[7:0] + 1;
                end
                else if (state == STA_LOG) begin
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_io0_in};
                    end
                end
                // ---------------------------------------------------------
                // Dual I/O address phase (1-2-2 mode, CMD 0xBB)
                // 2 address bits per clock: IO1=high bit, IO0=low bit
                // ---------------------------------------------------------
                else if (state == STA_ADDR_READ_DUAL) begin
                    
                    // SDRAM pipeline during last address clocks
                    if (addr_count == 7) begin
                        ram_inhibit_refresh <= 1;
                    end
                    else if (addr_count == 3) begin
                        ram_activate <= 1;
                        ram_addr[22:4] <= addr[25:7];
                    end
                    else if (addr_count == 1) begin
                        ram_read <= 1;
                        // addr[6:3] all received by prior clocks
                        ram_addr[3:0] <= addr[6:3];
                    end
                    
                    // Receive 2 address bits per clock
                    addr[addr_count]     <= spi_io1_in;   // High bit
                    addr[addr_count - 1] <= spi_io0_in;   // Low bit
                    addr_count <= addr_count - 2;
                    
                    // Transition when last 2 bits received (addr_count was 1)
                    if (addr_count == 1) begin
                        // Enter mode+dummy phase (4 dual clocks for 0xBB)
                        state <= STA_MODE_MULTI;
                        mode_count <= 3;
                    end
                end
                // ---------------------------------------------------------
                // Mode+dummy phase for multi-IO reads (0xBB and 0xEB)
                // Dual 0xBB: 4 clocks (mode byte M[7:0])
                // Quad 0xEB: 6 clocks (2 mode + 4 dummy)
                // SDRAM read signals remain asserted; deassert at end.
                // ---------------------------------------------------------
                else if (state == STA_MODE_MULTI) begin
                    if (mode_count == 0) begin
                        ram_inhibit_refresh <= 0;
                        ram_activate <= 0;
                        ram_read <= 0;
                        
                        if (is_quad_read) begin
                            state <= STA_READ_QUAD;
                            spi_io0_oe_ff <= 1;
                            spi_io1_oe_ff <= 1;
                            spi_io2_oe_ff <= 1;
                            spi_io3_oe_ff <= 1;
                        end
                        else begin
                            state <= STA_READ_DUAL;
                            spi_io0_oe_ff <= 1;
                            spi_io1_oe_ff <= 1;
                        end
                        fresh_read <= 1;
                    end
                    else begin
                        mode_count <= mode_count - 1;
                    end
                end
                // ---------------------------------------------------------
                // Dual data output phase (1-1-2 and 1-2-2)
                // 2 data bits per clock on IO0+IO1, 4 clocks per byte.
                // bit_count_in cycles 3->2->1->0->3->... in this state.
                //
                // SDRAM pipeline timing (dual, 4 clocks/byte):
                //   Byte 5, bit 3: inhibit_refresh + save byte 7
                //   Byte 6, bit 3: activate + ram_read + new ram_addr
                //   Byte 7, bit 0: deassert all + fresh_read
                //
                // ram_read is asserted at byte 6 (with activate) to give 8
                // SPI clocks (~32 sys clocks @120MHz) before fresh_read.
                // The SDRAM controller dispatches ACTIVATE first (priority),
                // waits tRCD, then READ — total ~14 sys clocks, leaving
                // ~18 sys clocks of margin.
                //
                // Byte 7 is saved at byte 5 because ram_read_buffer is a
                // live register: the SDRAM read completes within ~14 sys
                // clocks, overwriting the buffer before byte 7 can be
                // loaded into miso_byte at byte 6 bit 0.
                // ---------------------------------------------------------
                else if (state == STA_READ_DUAL) begin
                    
                    // Pipeline phase 1: inhibit refresh + save byte 7
                    // Save byte 7 from ram_read_buffer BEFORE the SDRAM read
                    // (phase 2) overwrites it with the next burst's data.
                    // ram_read_buffer is a live register updated by the SDRAM
                    // controller; the read completes within ~14 sys clocks,
                    // which is faster than the 3 SPI clocks between phase 2
                    // and the byte 7 preload at byte 6 bit 0.
                    if (addr[2:0] == 5 && bit_count_in == 3) begin
                        ram_inhibit_refresh <= 1;
                        saved_last_byte <= ram_read_buffer[7*8 +: 8];
                    end
                    
                    // Pipeline phase 2: activate + read 1 byte early
                    // Both signals are asserted simultaneously. The SDRAM
                    // controller's priority dispatch processes ACTIVATE first,
                    // waits tRCD, then dispatches READ. This gives 8 SPI
                    // clocks (~32 sys clocks) of margin before fresh_read,
                    // vs the ~14 sys clocks the SDRAM path needs.
                    if (addr[2:0] == 6 && bit_count_in == 3) begin
                        ram_inhibit_refresh <= 1;  // Also here for short first bursts
                        ram_activate <= 1;
                        ram_read <= 1;
                        ram_addr <= ram_addr + 1;
                    end
                    
                    // Pipeline phase 3: deassert during last byte
                    if (addr[2:0] == 7) begin
                        if (bit_count_in == 3 && !ram_activate) begin
                            // Fallback for very short first bursts (addr starts at 7)
                            // Only fires if byte 6 activate didn't happen
                            ram_inhibit_refresh <= 1;
                            ram_activate <= 1;
                            ram_read <= 1;
                            ram_addr <= ram_addr + 1;
                            saved_last_byte <= ram_read_buffer[7*8 +: 8];
                        end
                        else if (bit_count_in == 0) begin
                            ram_inhibit_refresh <= 0;
                            ram_activate <= 0;
                            ram_read <= 0;
                            
                            fresh_read <= 1;
                        end
                    end
                    
                    // Byte advance every 4 clocks
                    // Use saved_last_byte for byte 7 since ram_read_buffer
                    // may already contain the next burst's data by this point.
                    if (bit_count_in == 0) begin
                        if (addr[2:0] == 6)
                            miso_byte <= saved_last_byte;
                        else
                            miso_byte <= ram_read_buffer[(addr[2:0]+1)*8 +: 8];
                        addr <= addr + 1;
                    end
                    
                    if (fresh_read)
                        miso_byte <= ram_read_buffer[addr[2:0]*8 +: 8];
                end
                // ---------------------------------------------------------
                // Quad I/O address phase (1-4-4 mode, CMD 0xEB)
                // 4 address bits per clock: IO3=MSB, IO0=LSB
                // ---------------------------------------------------------
                else if (state == STA_ADDR_READ_QUAD) begin
                    
                    // SDRAM pipeline during last address clocks
                    if (addr_count == 11) begin
                        ram_inhibit_refresh <= 1;
                    end
                    else if (addr_count == 7) begin
                        // addr[25:8] available from prior clocks; addr[7] = IO3 now
                        ram_addr[22:4] <= {addr[25:8], spi_io3_in};
                    end
                    else if (addr_count == 3) begin
                        ram_activate <= 1;
                        ram_read <= 1;
                        // addr[6:4] available from prior clock; addr[3] = IO3 now
                        ram_addr[3:0] <= {addr[6:4], spi_io3_in};
                    end
                    
                    // Receive 4 address bits per clock
                    addr[addr_count]     <= spi_io3_in;   // MSB
                    addr[addr_count - 1] <= spi_io2_in;
                    addr[addr_count - 2] <= spi_io1_in;
                    addr[addr_count - 3] <= spi_io0_in;   // LSB
                    addr_count <= addr_count - 4;
                    
                    // Transition when last 4 bits received (addr_count was 3)
                    if (addr_count == 3) begin
                        // Enter mode+dummy phase (6 quad clocks: 2 mode + 4 dummy)
                        state <= STA_MODE_MULTI;
                        mode_count <= 5;
                    end
                end
                // ---------------------------------------------------------
                // Quad data output phase (1-1-4 and 1-4-4)
                // 4 data bits per clock on IO[3:0], 2 clocks per byte.
                // bit_count_in cycles 1->0->1->0->... in this state.
                //
                // SDRAM pipeline timing (quad, 2 clocks/byte):
                //   Byte 5, bit 1: inhibit_refresh + save byte 7
                //   Byte 6, bit 1: activate + ram_read + new ram_addr
                //   Byte 7, bit 0: deassert all + fresh_read
                //
                // From activate to fresh_read: 4 SPI clocks = 48 sys clocks
                // SDRAM path needs ~14 sys clocks, leaving ~34 of margin.
                // ---------------------------------------------------------
                else if (state == STA_READ_QUAD) begin
                    
                    // Pipeline phase 1: inhibit refresh + save byte 7
                    if (addr[2:0] == 5 && bit_count_in == 1) begin
                        ram_inhibit_refresh <= 1;
                        saved_last_byte <= ram_read_buffer[7*8 +: 8];
                    end
                    
                    // Pipeline phase 2: activate + read
                    if (addr[2:0] == 6 && bit_count_in == 1) begin
                        ram_inhibit_refresh <= 1;
                        ram_activate <= 1;
                        ram_read <= 1;
                        ram_addr <= ram_addr + 1;
                    end
                    
                    // Pipeline phase 3: deassert during last byte
                    if (addr[2:0] == 7) begin
                        if (bit_count_in == 1 && !ram_activate) begin
                            // Fallback for very short first bursts (addr starts at 7)
                            ram_inhibit_refresh <= 1;
                            ram_activate <= 1;
                            ram_read <= 1;
                            ram_addr <= ram_addr + 1;
                            saved_last_byte <= ram_read_buffer[7*8 +: 8];
                        end
                        else if (bit_count_in == 0) begin
                            ram_inhibit_refresh <= 0;
                            ram_activate <= 0;
                            ram_read <= 0;
                            
                            fresh_read <= 1;
                        end
                    end
                    
                    // Byte advance every 2 clocks
                    if (bit_count_in == 0) begin
                        if (addr[2:0] == 6)
                            miso_byte <= saved_last_byte;
                        else
                            miso_byte <= ram_read_buffer[(addr[2:0]+1)*8 +: 8];
                        addr <= addr + 1;
                    end
                    
                    if (fresh_read)
                        miso_byte <= ram_read_buffer[addr[2:0]*8 +: 8];
                end
                // ---------------------------------------------------------
                // Write Status Register data reception (CMD 0x01)
                // Receives 1 byte (SR1) or 2 bytes (SR1 + SR2).
                // ---------------------------------------------------------
                // ---------------------------------------------------------
                // SFDP data output phase (CMD 0x5A)
                // One byte per 8 SPI clocks, like normal single read.
                // Data comes from SFDP table in glue.v via sfdp_rdata.
                // Address preloaded 1 cycle ahead via sfdp_raddr.
                // ---------------------------------------------------------
                else if (state == STA_READSFDP) begin
                    if (bit_count_in == 1) begin
                        // Preload next byte address for sfdp_rdata
                        sfdp_raddr <= addr[6:0] + 1;
                    end
                    else if (bit_count_in == 0) begin
                        miso_byte <= sfdp_rdata;
                        addr <= addr + 1;
                    end
                end
                else if (state == STA_WRITESTATUS && bit_count_in == 0) begin
                    if (addr_count == 0) begin
                        // First byte: SR1 (preserve WEL/BUSY)
                        status_reg[7:2] <= mosi_byte[7:2];
                        status_reg[1] <= 0;  // Clear WEL after write
                        addr_count <= 1;
                    end
                    else if (addr_count == 1) begin
                        // Second byte: SR2
                        status_reg2 <= {mosi_byte[7:1], spi_io0_in};
                        addr_count <= 2;
                    end
                end
                
                // ---------------------------------------------------------
                // bit_count_in management
                // Single mode: cycles 7->0 (8 clocks/byte), wraps naturally
                // Dual read:   cycles 3->0 (4 clocks/byte), override wrap
                // ---------------------------------------------------------
                if (state == STA_READ_QUAD && bit_count_in == 0)
                    bit_count_in <= 1;
                else if (state == STA_READ_DUAL && bit_count_in == 0)
                    bit_count_in <= 3;
                else if (state == STA_DUMMY && dummy_count == 0 && is_quad_read)
                    bit_count_in <= 1;   // Entry into STA_READ_QUAD
                else if (state == STA_DUMMY && dummy_count == 0 && is_dual_read)
                    bit_count_in <= 3;   // Entry into STA_READ_DUAL
                else if (state == STA_MODE_MULTI && mode_count == 0 && is_quad_read)
                    bit_count_in <= 1;   // Entry into STA_READ_QUAD
                else if (state == STA_MODE_MULTI && mode_count == 0)
                    bit_count_in <= 3;   // Entry into STA_READ_DUAL
                else
                    bit_count_in <= bit_count_in - 1;
            end
        end
    end
    
    // Data output on falling edge of SPI clock
    always @(negedge spi_clk) begin
        if (state == STA_READ_QUAD) begin
            // Quad output: 4 bits per clock (2 clocks per byte)
            // IO3 = MSB of nibble, IO0 = LSB of nibble
            // bit_count_in=1: high nibble [7:4], bit_count_in=0: low nibble [3:0]
            if (fresh_read) begin
                spi_io3_out <= ram_read_buffer[addr[2:0]*8 + 7];
                spi_io2_out <= ram_read_buffer[addr[2:0]*8 + 6];
                spi_io1_out <= ram_read_buffer[addr[2:0]*8 + 5];
                spi_io0_out <= ram_read_buffer[addr[2:0]*8 + 4];
            end
            else begin
                spi_io3_out <= miso_byte[{bit_count_in[0], 2'b11}];
                spi_io2_out <= miso_byte[{bit_count_in[0], 2'b10}];
                spi_io1_out <= miso_byte[{bit_count_in[0], 2'b01}];
                spi_io0_out <= miso_byte[{bit_count_in[0], 2'b00}];
            end
        end
        else if (state == STA_READ_DUAL) begin
            // Dual output: 2 bits per clock
            // IO1 = high bit of pair, IO0 = low bit of pair
            if (fresh_read) begin
                spi_io1_out <= ram_read_buffer[addr[2:0]*8 + 7];
                spi_io0_out <= ram_read_buffer[addr[2:0]*8 + 6];
            end
            else begin
                spi_io1_out <= miso_byte[{bit_count_in[1:0], 1'b1}];
                spi_io0_out <= miso_byte[{bit_count_in[1:0], 1'b0}];
            end
        end
        else begin
            // Single output on IO1 (MISO)
            if (fresh_read)
                spi_io1_out <= ram_read_buffer[addr[2:0]*8 + 7];
            else
                spi_io1_out <= miso_byte[bit_count_in];
        end
    end

endmodule
