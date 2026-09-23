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
    
    // SDRAM read requests and buffers, driven by spi_prefetch (see there)
    output wire ram_inhibit_refresh,
    output wire ram_activate,
    output wire ram_read,
    output wire ram_continuation,
    output wire ram_post_toggle,
    output wire [22:0] ram_addr,       // 23-bit burst address for 64MB
    input wire [63:0] ram_read_buffer,
    input wire [63:0] ram_read_buffer_b,
    input wire ram_read_valid_a,
    input wire ram_read_valid_b,
    input wire ram_read_busy,
    // Sticky per-transaction diagnostics: the consumed burst never filled
    // (underrun) or the upcoming burst was not yet complete at a burst end
    // (thin margin, data may still be correct).
    output wire prefetch_underrun,
    output wire prefetch_thin,

    // For writing
    output reg write_cmd,
    output reg [1:0] write_type,  // 00=page program, 01=erase, 10=AAI RMW
    output reg [22:0] write_addr,     // 23-bit burst address
    output reg [22:0] write_len,
    input wire write_done,       // Toggles once per completed write/erase
    
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
    output reg [7:0] log_val = 0,
    
    // Structured logging outputs (directly driven from SPI state machine).
    // All signals are in the SPI clock domain; the logger module synchronizes.
    output reg log_cmd_valid = 0,       // Pulse: command byte decoded
    output reg [7:0] log_cmd_opcode = 0,// The opcode that was decoded
    output reg log_addr_valid = 0,      // Pulse: address phase complete
    output reg log_addr_toggle = 0,     // Changes on the final address bit
    // Held across CS/reset until the next address event, so the system
    // domain can capture this payload using the synchronized toggle.
    output reg [31:0] log_addr_out = 0, // Full flash byte address
    output reg [23:0] log_byte_count = 0// Running count of bytes read in current transaction
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
    //
    // cfg_chip_erase_bursts is both the chip-erase length and the burst-address
    // mask. SPI NOR capacities are powers of two, and the configured value is
    // (capacity_bytes / 8) - 1. Mask every SDRAM-facing address so unimplemented
    // high address bits alias into the configured flash array like real chips.
    function [22:0] wrap_burst_addr;
        input [22:0] burst_addr;
        begin
            wrap_burst_addr = burst_addr & cfg_chip_erase_bursts;
        end
    endfunction
    
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
        CMD_PAGEPROGRAM_4B  = 8'h12,
        CMD_READ_4B         = 8'h13,
        CMD_DUALREAD_4B     = 8'h3C,  // Dual Output Read with 4-byte address
        CMD_SECTORERASE_4K  = 8'h20,
        CMD_SECTORERASE_4K_4B = 8'h21,
        CMD_READSTATUS2     = 8'h35,  // Read Status Register 2
        CMD_DUALREAD        = 8'h3B,  // Dual Output Read (1-1-2)
        CMD_BLOCKERASE_32K  = 8'h52,
        CMD_BLOCKERASE_32K_4B = 8'h5C,
        CMD_CHIPERASE1      = 8'h60,
        CMD_QUADREAD        = 8'h6B,  // Quad Output Read (1-1-4)
        CMD_QUADREAD_4B     = 8'h6C,  // Quad Output Read with 4-byte address
        CMD_READID1         = 8'h9E,
        CMD_READSFDP        = 8'h5A,  // Read SFDP table
        CMD_READID2         = 8'h9F,
        CMD_4BYTEENABLE     = 8'hB7,
        CMD_DUALIOREAD      = 8'hBB,  // Dual I/O Read (1-2-2)
        CMD_DUALIOREAD_4B   = 8'hBC,  // Dual I/O Read with 4-byte address
        CMD_CHIPERASE2      = 8'hC7,
        CMD_BLOCKERASE_64K  = 8'hD8,
        CMD_BLOCKERASE_64K_4B = 8'hDC,
        CMD_EWSR            = 8'h50,  // Enable Write Status Register (SST)
        CMD_AAI_WORD        = 8'hAD,  // AAI Word Program (SST)
        CMD_4BYTEDISABLE    = 8'hE9,
        CMD_QUADIOREAD      = 8'hEB,  // Quad I/O Read (1-4-4)
        CMD_QUADIOREAD_4B   = 8'hEC,  // Quad I/O Read with 4-byte address
        CMD_LOG             = 8'hF2;
    
    // State machine states
    localparam
        STA_CMD             = 0,
        STA_READSTATUS      = 1,
        STA_ADDR            = 2,   // Address phase, 1/2/4 lanes (addr_lanes)
        STA_READ            = 3,   // Data output phase, 1/2/4 lanes (read_byte_top)
        STA_READID          = 4,
        STA_WRITE           = 6,
        STA_ERASE           = 8,
        STA_LOG             = 9,
        STA_DUMMY           = 10,
        STA_MODE_MULTI      = 13,  // Mode+dummy phase for 0xBB/0xEB
        STA_WRITESTATUS     = 16,  // Receive status register write data
        STA_READSFDP        = 17,  // SFDP data output phase
        STA_AAI_DATA        = 18;  // AAI data reception (2 bytes)
    
    reg [4:0] state;
    reg [2:0] dummy_count = 0;
    reg is_fast_read = 0;
    reg is_dual_read = 0;       // Set for both 0x3B and 0xBB
    reg is_quad_read = 0;       // Set for both 0x6B and 0xEB
    // Last bit_count_in value of a data byte in STA_READ: 7 for single,
    // 3 for dual and 1 for quad output (SPI clocks per byte - 1).
    reg [2:0] read_byte_top = 7;
    reg is_sfdp_read = 0;       // Set for CMD 0x5A (Read SFDP)
    reg [2:0] mode_count = 0;   // Mode+dummy counter (4 for dual, 6 for quad)
    
    // AAI (Auto Address Increment) state -- persists across CS cycles
    reg aai_active = 0;         // In AAI word program mode
    reg is_aai = 0;             // Current transaction is AAI (per-CS flag)
    reg [1:0] aai_bytes_left;   // Ignore data beyond the two-byte AAI word
    
    reg [31:0] addr;
    reg [4:0] addr_count;       // Index of the next address MSB to arrive
    reg addr_4byte;

    // STA_ADDR receives addr_lanes (1, 2 or 4) bits per clock, MSB first,
    // then continues according to addr_kind.
    localparam [1:0]
        ADDR_KIND_READ  = 2'd0,
        ADDR_KIND_WRITE = 2'd1,
        ADDR_KIND_ERASE = 2'd2;
    reg [2:0] addr_lanes = 1;
    reg [1:0] addr_kind = ADDR_KIND_READ;
    wire addr_quad = addr_lanes[2];
    wire addr_dual = addr_lanes[1];
    // Address including the bits arriving on this clock. Commands clear
    // addr when entering STA_ADDR, so before the phase ends addr holds only
    // the bits received so far, right-aligned.
    wire [31:0] addr_next = addr_quad ? {addr[27:0], spi_io3_in, spi_io2_in, spi_io1_in, spi_io0_in} :
                            addr_dual ? {addr[29:0], spi_io1_in, spi_io0_in} :
                                        {addr[30:0], spi_io0_in};
    // Most significant address bit arriving on this clock.
    wire addr_lane_msb = addr_quad ? spi_io3_in : addr_dual ? spi_io1_in : spi_io0_in;
    wire addr_last = addr_count == addr_lanes - 1'b1;
    
    // Set on the clock that starts a new burst in STA_READ; the first byte
    // of the burst comes straight from live_buffer.
    reg fresh_read = 0;
    wire [63:0] live_buffer;

    // Status registers
    reg [7:0] status_reg = 8'b00000000;
    reg [7:0] status_reg2 = 8'h02;     // QE bit (bit 1) default enabled
    
    // Synchronize write_done from the system clock domain into spi_clk.
    // glue.v toggles write_done once per completed program/erase, so the
    // event is preserved even if spi_clk is stopped while the write runs.
    reg [2:0] write_done_sync = 0;
    wire write_busy_clr = write_done_sync[1] ^ write_done_sync[2];

    reg status_read_sel2 = 0;

    // SDRAM prefetch. Each strobe marks the clock on which the SPI
    // transaction reaches the corresponding point; spi_prefetch documents
    // what it does with it.
    wire addr_read = state == STA_ADDR && addr_kind == ADDR_KIND_READ && !is_sfdp_read;
    wire in_read = state == STA_READ;

    spi_prefetch prefetch(
        .spi_clk(spi_clk),
        .active(is_selected),
        .restart(reset_cs || reset_power),
        .addr_mask(cfg_chip_erase_bursts),
        .fresh_read(fresh_read),
        .in_read(in_read),
        .first_inhibit(addr_read && addr_count == 15),
        // Quad: IO3/IO2 carry byte-address bits 11/10 on this clock.
        .first_activate(addr_read && addr_count == (addr_quad ? 11 : 9)),
        .first_row(addr_quad ? {addr[13:0], spi_io3_in, spi_io2_in} : addr[15:0]),
        .first_read(addr_read && addr_count == 3),
        .first_col({addr[5:0], addr_lane_msb}),
        .first_done(addr_read && addr_last && !addr_dual && !addr_quad),
        .hold_inhibit(is_fast_read),
        .release_inhibit(state == STA_DUMMY && dummy_count == 0),
        .post_lookahead((state == STA_DUMMY && dummy_count == 5 && !is_sfdp_read) ||
                        (state == STA_MODE_MULTI && mode_count == 1)),
        .drop((state == STA_MODE_MULTI && mode_count == 3) ||
              (in_read && addr[2:0] == 6 && bit_count_in == 0)),
        .mode_end(state == STA_MODE_MULTI && mode_count == 0),
        .fallback(in_read && addr[2:0] == 7 && bit_count_in == read_byte_top),
        .burst_end(in_read && addr[2:0] == 7 && bit_count_in == 0),
        .ram_inhibit_refresh(ram_inhibit_refresh),
        .ram_activate(ram_activate),
        .ram_read(ram_read),
        .ram_continuation(ram_continuation),
        .ram_post_toggle(ram_post_toggle),
        .ram_addr(ram_addr),
        .ram_read_buffer(ram_read_buffer),
        .ram_read_buffer_b(ram_read_buffer_b),
        .ram_read_valid_a(ram_read_valid_a),
        .ram_read_valid_b(ram_read_valid_b),
        .live_buffer(live_buffer),
        .prefetch_underrun(prefetch_underrun),
        .prefetch_thin(prefetch_thin)
    );

    // Main SPI state machine
    always @(posedge spi_clk) begin
        write_done_sync <= {write_done_sync[1:0], write_done};

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
                
                // Preserve addr during AAI mode (auto-increment persists across CS)
                if (!aai_active)
                    addr <= 0;
                addr_count <= 0;
                addr_lanes <= 1;
                addr_kind <= ADDR_KIND_READ;
                dummy_count <= 0;
                is_fast_read <= 0;
                is_dual_read <= 0;
                is_quad_read <= 0;
                read_byte_top <= 7;
                is_sfdp_read <= 0;
                is_aai <= 0;
                aai_bytes_left <= 0;
                status_read_sel2 <= 0;
                mode_count <= 0;
                
                log_strobe <= 0;
                log_val <= 0;
                log_cmd_valid <= 0;
                log_addr_valid <= 0;
                log_byte_count <= 0;
                
                write_cmd <= 0;
                
                write_buf_strobe <= 0;
                
                if (reset_power) begin
                    status_reg[1:0] <= 2'b00;
                    status_reg[6] <= 0;     // AAI bit
                    addr_4byte <= 0;
                    aai_active <= 0;
                    addr <= 0;
                    
                    log_strobe <= 1;
                    log_val <= 8'hE2;
                end
            end
            else begin
                log_strobe <= 0;
                log_cmd_valid <= 0;
                log_addr_valid <= 0;
                    
                write_buf_strobe <= 0;
                
                mosi_byte[bit_count_in] <= spi_io0_in;


                if ((state == STA_CMD) && (bit_count_in == 0)) begin
                    
                    case ({mosi_byte[7:1], spi_io0_in})
                        
                    CMD_READSTATUS: begin
                        state <= STA_READSTATUS;
                        status_read_sel2 <= 0;
                        spi_io1_oe_ff <= 1;
                        miso_byte <= status_reg;
                    end
                    
                    CMD_READSTATUS2: begin
                        state <= STA_READSTATUS;
                        status_read_sel2 <= 1;
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
                        status_reg[6] <= 0;  // Clear AAI bit
                        aai_active <= 0;
                    end
                    
                    CMD_WRITEENABLE: begin
                        status_reg[1] <= 1;
                    end
                    
                    CMD_EWSR: begin
                        // SST Enable-Write-Status-Register: acts like WREN
                        // for status register writes.  For emulation we just
                        // set WEL; the distinction only matters for hardware
                        // write protection which the emulator doesn't enforce.
                        status_reg[1] <= 1;
                    end
                    
                    CMD_4BYTEENABLE: begin
                        if (cfg_4byte) addr_4byte <= 1;
                    end
                    
                    CMD_4BYTEDISABLE: begin
                        if (cfg_4byte) addr_4byte <= 0;
                    end
                        
                    CMD_READ: begin
                        state <= STA_ADDR;
                        addr <= 0;
                        addr_count <= addr_4byte ? 31 : 23;
                    end
                    
                    CMD_READ_4B: begin
                        if (cfg_4byte) begin
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_count <= 31;
                        end
                    end
                    
                    CMD_FASTREAD: begin
                        state <= STA_ADDR;
                        addr <= 0;
                        addr_count <= addr_4byte ? 31 : 23;
                        is_fast_read <= 1;
                    end
                    
                    CMD_FASTREAD_4B: begin
                        if (cfg_4byte) begin
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_count <= 31;
                            is_fast_read <= 1;
                        end
                    end
                    
                    // Dual Output Read (1-1-2): cmd(1), addr(1), 8 dummy, data(2)
                    CMD_DUALREAD,
                    CMD_DUALREAD_4B: begin
                        state <= STA_ADDR;
                        addr <= 0;
                        addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_DUALREAD_4B) ? 31 : (addr_4byte ? 31 : 23);
                        is_fast_read <= 1;   // Uses same dummy phase
                        is_dual_read <= 1;
                        read_byte_top <= 3;
                    end
                    
                    // Dual I/O Read (1-2-2): cmd(1), addr(2), mode+dummy(2), data(2)
                    CMD_DUALIOREAD,
                    CMD_DUALIOREAD_4B: begin
                        state <= STA_ADDR;
                        addr <= 0;
                        addr_lanes <= 2;
                        addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_DUALIOREAD_4B) ? 31 : (addr_4byte ? 31 : 23);
                        is_dual_read <= 1;
                        read_byte_top <= 3;
                    end
                    
                    // Quad Output Read (1-1-4): cmd(1), addr(1), 8 dummy, data(4)
                    CMD_QUADREAD,
                    CMD_QUADREAD_4B: begin
                        if (status_reg2[1]) begin  // QE required
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_QUADREAD_4B) ? 31 : (addr_4byte ? 31 : 23);
                            is_fast_read <= 1;   // Uses same dummy phase
                            is_quad_read <= 1;
                            read_byte_top <= 1;
                        end
                    end
                    
                    // Quad I/O Read (1-4-4): cmd(1), addr(4), mode+dummy(4), data(4)
                    CMD_QUADIOREAD,
                    CMD_QUADIOREAD_4B: begin
                        if (status_reg2[1]) begin  // QE required
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_lanes <= 4;
                            addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_QUADIOREAD_4B) ? 31 : (addr_4byte ? 31 : 23);
                            is_quad_read <= 1;
                            read_byte_top <= 1;
                        end
                    end

                    CMD_SECTORERASE_4K,
                    CMD_SECTORERASE_4K_4B: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_kind <= ADDR_KIND_ERASE;
                            addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_SECTORERASE_4K_4B) ? 31 : (addr_4byte ? 31 : 23);
                            write_len <= 23'h001FF; // 4KB = 512 × 8-byte bursts - 1
                        end
                    end
                    
                    CMD_BLOCKERASE_32K,
                    CMD_BLOCKERASE_32K_4B: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_kind <= ADDR_KIND_ERASE;
                            addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_BLOCKERASE_32K_4B) ? 31 : (addr_4byte ? 31 : 23);
                            write_len <= 23'h00FFF; // 32KB = 4096 × 8-byte bursts - 1
                        end
                    end
                    
                    CMD_BLOCKERASE_64K,
                    CMD_BLOCKERASE_64K_4B: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_kind <= ADDR_KIND_ERASE;
                            addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_BLOCKERASE_64K_4B) ? 31 : (addr_4byte ? 31 : 23);
                            write_len <= 23'h01FFF; // 64KB = 8192 × 8-byte bursts - 1
                        end
                    end
                    
                    CMD_CHIPERASE1,
                    CMD_CHIPERASE2: begin
                        if (status_reg[1]) begin
                            state <= STA_ERASE;
                            write_cmd <= 1;
                            write_type <= 2'd1;
                            write_addr <= 23'b0;
                            write_len <= cfg_chip_erase_bursts;
                            status_reg[1] <= 0;
                            status_reg[0] <= 1;
                        end
                    end
                    
                    CMD_PAGEPROGRAM,
                    CMD_PAGEPROGRAM_4B: begin
                        if (status_reg[1]) begin
                            state <= STA_ADDR;
                            addr <= 0;
                            addr_kind <= ADDR_KIND_WRITE;
                            addr_count <= ({mosi_byte[7:1], spi_io0_in} == CMD_PAGEPROGRAM_4B) ? 31 : (addr_4byte ? 31 : 23);
                            write_len <= 23'h0001F; // 256 byte page = 32 × 8-byte bursts - 1
                        end
                    end
                    
                    // AAI Word Program (SST, 0xAD):
                    //   First:      0xAD + addr(3) + byte0 + byte1
                    //   Subsequent: 0xAD + byte0 + byte1
                    // Address auto-increments by 2. Mode persists across
                    // CS cycles until WRDI (0x04).  The host is expected
                    // to send an even starting address per the SST spec;
                    // A0 of the transmitted address is accepted as-is
                    // (matches real SST chips which ignore A0 internally
                    // because writes are word-aligned).
                    //
                    // SR bit 6 (AAI flag) is set on entry and cleared on
                    // WRDI, matching the SST25VFxxx datasheet and what
                    // flashprog's spi_prettyprint_status_register_sst25
                    // reports.
                    CMD_AAI_WORD: begin
                        if (status_reg[1] || aai_active) begin
                            if (!aai_active) begin
                                // First AAI: need address phase
                                state <= STA_ADDR;
                                addr <= 0;
                                addr_kind <= ADDR_KIND_WRITE;
                                addr_count <= addr_4byte ? 31 : 23;
                                is_aai <= 1;
                                write_len <= 0;
                                aai_active <= 1;
                                status_reg[6] <= 1;  // Set AAI bit
                            end else begin
                                // Subsequent AAI: skip address, go to data
                                state <= STA_AAI_DATA;
                                aai_bytes_left <= 2;
                                write_cmd <= 1;
                                write_type <= 2'd2;
                                write_addr <= wrap_burst_addr(addr[25:3]);
                                write_len <= 0;
                                status_reg[0] <= 1;
                            end
                        end
                    end
                    
                    CMD_READSFDP: begin
                        state <= STA_ADDR;
                        addr <= 0;
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
                    log_cmd_valid <= 1;
                    log_cmd_opcode <= {mosi_byte[7:1], spi_io0_in};
                    log_byte_count <= 0;
                end
                else if ((state == STA_READSTATUS) && (bit_count_in == 0)) begin
                    miso_byte <= status_read_sel2 ? status_reg2 : status_reg;
                end
                // ---------------------------------------------------------
                // Address phase for reads, programs and erases. Receives
                // addr_lanes bits per clock MSB first: IO0 for 1-x-x
                // commands, IO1:IO0 for 1-2-2 and IO3:IO0 for 1-4-4.
                // ---------------------------------------------------------
                else if (state == STA_ADDR) begin
                    // SST AAI ignores transmitted A0: the two bytes occupy
                    // an aligned word within one SDRAM burst.
                    addr <= (is_aai && addr_last) ? {addr_next[31:1], 1'b0} : addr_next;
                    addr_count <= addr_count - addr_lanes;

                    if (!addr_dual && !addr_quad && bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_io0_in};
                    end

                    if (addr_last) begin
                        log_addr_valid <= 1;
                        log_addr_toggle <= !log_addr_toggle;
                        log_addr_out <= addr_next;

                        case (addr_kind)
                        ADDR_KIND_READ: begin
                            if (addr_dual || addr_quad) begin
                                // Mode+dummy phase: 4 clocks for 0xBB,
                                // 6 (2 mode + 4 dummy) for 0xEB.
                                state <= STA_MODE_MULTI;
                                mode_count <= addr_quad ? 3'd5 : 3'd3;
                            end
                            else begin
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
                        end

                        ADDR_KIND_ERASE: begin
                            state <= STA_ERASE;
                            write_cmd <= 1;
                            write_type <= 2'd1;

                            // Align address based on erase size
                            // write_addr is 23-bit burst address = byte_addr[25:3]
                            if (write_len == 20'h01FFF)
                                write_addr <= wrap_burst_addr({addr_next[25:16], 13'b0});  // 64KB aligned
                            else if (write_len == 20'h00FFF)
                                write_addr <= wrap_burst_addr({addr_next[25:15], 12'b0});  // 32KB aligned
                            else
                                write_addr <= wrap_burst_addr({addr_next[25:12], 9'b0});   // 4KB aligned

                            status_reg[1] <= 0;
                            status_reg[0] <= 1;
                        end

                        default: begin // ADDR_KIND_WRITE
                            if (is_aai) begin
                                // Don't clear WEL between AAI words.
                                state <= STA_AAI_DATA;
                                aai_bytes_left <= 2;
                                write_cmd <= 1;
                                write_type <= 2'd2;
                                write_addr <= wrap_burst_addr(addr_next[25:3]);
                                write_len <= 0;
                                status_reg[0] <= 1;
                            end else begin
                                state <= STA_WRITE;
                                write_cmd <= 1;
                                write_type <= 2'd0;

                                // Page-aligned address in 8-byte burst units
                                // byte_addr[25:3] -> burst address, page = 256 bytes = 32 bursts
                                write_addr <= wrap_burst_addr({addr_next[25:8], 5'b0});

                                status_reg[1] <= 0;
                                status_reg[0] <= 1;
                            end
                        end
                        endcase
                    end
                end
                else if (state == STA_DUMMY) begin
                    if (dummy_count == 0) begin
                        if (is_sfdp_read) begin
                            // SFDP read: output from internal SFDP table.
                            // sfdp_raddr was preloaded at dummy_count==2,
                            // so sfdp_rdata is valid now.
                            state <= STA_READSFDP;
                            spi_io1_oe_ff <= 1;
                            miso_byte <= sfdp_rdata;
                        end
                        else begin
                            state <= STA_READ;
                            spi_io0_oe_ff <= is_dual_read || is_quad_read;
                            spi_io1_oe_ff <= 1;
                            spi_io2_oe_ff <= is_quad_read;
                            spi_io3_oe_ff <= is_quad_read;
                            fresh_read <= 1;
                        end
                    end
                    else begin
                        dummy_count <= dummy_count - 1;
                        // Preload SFDP address two cycles before transition:
                        // the table is a sync BSRAM, so the address is
                        // sampled on the next SPI clock and data is valid
                        // the clock after that, just in time for the
                        // STA_READSFDP entry below.
                        if (dummy_count == 2 && is_sfdp_read)
                            sfdp_raddr <= addr[6:0];
                    end
                end
                // ---------------------------------------------------------
                // Data output phase for all read modes. read_byte_top sets
                // the SPI clocks per byte: bit_count_in cycles 7..0 for
                // single, 3..0 for dual (IO1:IO0) and 1..0 for quad
                // (IO3:IO0) output.
                // ---------------------------------------------------------
                else if (state == STA_READ) begin
                    // The next burst starts after byte 7. spi_prefetch has
                    // it in the other ping-pong half by then.
                    if (addr[2:0] == 7 && bit_count_in == 0)
                        fresh_read <= 1;

                    if (bit_count_in == 0) begin
                        miso_byte <= live_buffer[(addr[2:0]+1)*8 +: 8];
                        addr <= addr + 1;
                        log_byte_count <= log_byte_count + 1;
                    end

                    if (fresh_read)
                        miso_byte <= live_buffer[addr[2:0]*8 +: 8];
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
                else if ((state == STA_WRITE) && (bit_count_in == 0)) begin
                    write_buf_strobe <= 1;
                    write_buf_offset <= addr[7:0];
                    write_buf_val <= {mosi_byte[7:1], spi_io0_in};
                    
                    addr[7:0] <= addr[7:0] + 1;
                end
                // ---------------------------------------------------------
                // AAI data reception (2 bytes per CS cycle)
                // Keep addr word-aligned until both bytes arrive, then
                // advance the full address (including across pages).
                // A short transaction must not misalign the next AAI word.
                // Only two bytes are accepted, even if CS remains low:
                // flags outside this burst must not leak into a later PP.
                // ---------------------------------------------------------
                else if ((state == STA_AAI_DATA) && (bit_count_in == 0) &&
                         (aai_bytes_left != 0)) begin
                    aai_bytes_left <= aai_bytes_left - 1'b1;
                    write_buf_strobe <= 1;
                    write_buf_offset <= {addr[7:1], (aai_bytes_left == 1)};
                    write_buf_val <= {mosi_byte[7:1], spi_io0_in};
                    if (aai_bytes_left == 1)
                        addr <= addr + 2;
                end
                else if (state == STA_LOG) begin
                    if (bit_count_in == 0) begin
                        log_strobe <= 1;
                        log_val <= {mosi_byte[7:1], spi_io0_in};
                    end
                end
                // ---------------------------------------------------------
                // Mode+dummy phase for multi-IO reads (0xBB and 0xEB)
                // Dual 0xBB: 4 clocks (mode byte M[7:0])
                // Quad 0xEB: 6 clocks (2 mode + 4 dummy)
                // spi_prefetch posts the second burst during this phase.
                // ---------------------------------------------------------
                else if (state == STA_MODE_MULTI) begin
                    if (mode_count == 0) begin
                        state <= STA_READ;
                        spi_io0_oe_ff <= 1;
                        spi_io1_oe_ff <= 1;
                        spi_io2_oe_ff <= is_quad_read;
                        spi_io3_oe_ff <= is_quad_read;
                        fresh_read <= 1;
                    end
                    else begin
                        mode_count <= mode_count - 1;
                    end
                end
                // ---------------------------------------------------------
                // SFDP data output phase (CMD 0x5A)
                // One byte per 8 SPI clocks, like normal single read.
                // Data comes from SFDP table in glue.v via sfdp_rdata.
                // Address preloaded 2 SPI clocks ahead via sfdp_raddr
                // (synchronous BSRAM read latency).
                // ---------------------------------------------------------
                else if (state == STA_READSFDP) begin
                    if (bit_count_in == 2) begin
                        // Preload next byte address two SPI clocks ahead:
                        // BSRAM samples it at bit 1, data is stable a full
                        // clock before miso_byte loads it at bit 0.
                        sfdp_raddr <= addr[6:0] + 1;
                    end
                    else if (bit_count_in == 0) begin
                        miso_byte <= sfdp_rdata;
                        addr <= addr + 1;
                        log_byte_count <= log_byte_count + 1;
                    end
                end
                // ---------------------------------------------------------
                // Write Status Register data reception (CMD 0x01)
                // Receives 1 byte (SR1) or 2 bytes (SR1 + SR2).
                // ---------------------------------------------------------
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
                // Counts down to 0 once per byte; outside STA_READ it wraps
                // naturally (8 clocks/byte). STA_READ, and the dummy/mode
                // phases entering it, reload read_byte_top instead.
                // ---------------------------------------------------------
                if ((state == STA_READ && bit_count_in == 0) ||
                    (state == STA_DUMMY && dummy_count == 0) ||
                    (state == STA_MODE_MULTI && mode_count == 0))
                    bit_count_in <= read_byte_top;
                else
                    bit_count_in <= bit_count_in - 1;
            end
        end
    end
    
    // Data output on falling edge of SPI clock.
    //
    // IO0 and IO2 are only enabled on entry to STA_READ (IO0 for dual and
    // quad, IO2 for quad) and cleared at every CS drop, so their enable
    // flops double as registered lane selects. Decoding state here instead
    // would lengthen the half-cycle posedge-to-negedge output path.
    always @(negedge spi_clk) begin
        if (spi_io2_oe_ff) begin
            // Quad output: 4 bits per clock (2 clocks per byte)
            // IO3 = MSB of nibble, IO0 = LSB of nibble
            // bit_count_in=1: high nibble [7:4], bit_count_in=0: low nibble [3:0]
            if (fresh_read) begin
                spi_io3_out <= live_buffer[addr[2:0]*8 + 7];
                spi_io2_out <= live_buffer[addr[2:0]*8 + 6];
                spi_io1_out <= live_buffer[addr[2:0]*8 + 5];
                spi_io0_out <= live_buffer[addr[2:0]*8 + 4];
            end
            else begin
                spi_io3_out <= miso_byte[{bit_count_in[0], 2'b11}];
                spi_io2_out <= miso_byte[{bit_count_in[0], 2'b10}];
                spi_io1_out <= miso_byte[{bit_count_in[0], 2'b01}];
                spi_io0_out <= miso_byte[{bit_count_in[0], 2'b00}];
            end
        end
        else if (spi_io0_oe_ff) begin
            // Dual output: 2 bits per clock
            // IO1 = high bit of pair, IO0 = low bit of pair
            if (fresh_read) begin
                spi_io1_out <= live_buffer[addr[2:0]*8 + 7];
                spi_io0_out <= live_buffer[addr[2:0]*8 + 6];
            end
            else begin
                spi_io1_out <= miso_byte[{bit_count_in[1:0], 1'b1}];
                spi_io0_out <= miso_byte[{bit_count_in[1:0], 1'b0}];
            end
        end
        else begin
            // Single output on IO1 (MISO)
            if (fresh_read)
                spi_io1_out <= live_buffer[addr[2:0]*8 + 7];
            else
                spi_io1_out <= miso_byte[bit_count_in];
        end
    end

endmodule
