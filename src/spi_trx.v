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
    input wire [3:0] ram_read_beats_a,
    input wire [3:0] ram_read_beats_b,
    input wire ram_read_busy,
    // Sticky per-transaction diagnostics: an output sample used an unready
    // beat (underrun), or the upcoming first beat was not ready at a burst
    // boundary (thin margin, data may still be correct).
    output wire prefetch_underrun,
    output wire prefetch_thin,

    // For writing. write_cmd rises when a program/erase is issued and is
    // held until its completion clears WIP, so the system side cannot miss
    // it however long it waits for CS high and SDRAM ownership. The other
    // write_* outputs are held as long. write_partial is frozen at the end
    // of the issuing transaction: set if CS rose mid-byte, in which case a
    // real NOR flash does not execute the command.
    output reg write_cmd = 0,
    output reg write_partial = 0,
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
    input wire [459:0] cfg_profile,
    input wire cfg_erase_wren,
    input wire [4:0] cfg_erase_waddr,
    input wire [49:0] cfg_erase_wdata,
    input wire cfg_profile_valid,
    input wire [31:0] cfg_profile_epoch,
    
    // SFDP table read interface (memory in glue.v)
    output reg [6:0] sfdp_raddr,
    input wire [7:0] sfdp_rdata,
    
    
    // Structured logging outputs (directly driven from SPI state machine).
    // All signals are in the SPI clock domain; the logger module synchronizes.
    output reg log_cmd_valid = 0,       // Pulse: command byte decoded
    output reg [7:0] log_cmd_opcode = 0,// The opcode that was decoded
    output reg log_addr_valid = 0,      // Pulse: address phase complete
    // Changes on the final address bit of array reads only (not SFDP,
    // program or erase): the TOCTOU traps count those accesses.
    output reg log_addr_toggle = 0,
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
    reg miso_byte_valid;
    reg sample_missing = 0;
    
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
    
    `include "spi_flash_cmds.vh"
    
    // State machine states
    localparam
        STA_CMD             = 0,
        STA_READSTATUS      = 1,
        STA_ADDR            = 2,   // Address phase, 1/2/4 lanes (addr_lanes)
        STA_READ            = 3,   // Data output phase, 1/2/4 lanes (read_byte_top)
        STA_READID          = 4,
        STA_WRITE           = 6,
        STA_ERASE           = 8,
        // 9 (STA_LOG) retired with the 0xF2 logging hook: unlisted opcodes
        // are ignored by the decoder, so 9 stays unused rather than reused.
        STA_DUMMY           = 10,
        STA_MODE_MULTI      = 13,  // Mode+dummy phase for 0xBB/0xEB
        STA_WRITESTATUS     = 16,  // Receive status register write data
        STA_READSFDP        = 17,  // SFDP data output phase
        STA_AAI_DATA        = 18,  // AAI data reception (2 bytes)
        STA_SECTORREAD      = 19,
        STA_BPRREAD         = 20,
        STA_BPRWRITE        = 21,
        STA_REGWRITE        = 22,
        STA_UID_DUMMY       = 23,
        STA_UID_READ        = 24,
        STA_CONFIGREAD      = 25,
        STA_CONFIGWRITE     = 26,
        STA_ANYWRITE        = 27,
        STA_LEGACYID        = 28,
        STA_IGNORE          = 31;
    
    reg [4:0] state;
    reg [4:0] dummy_count = 0;
    reg [4:0] transaction_wait = 0;
    // dummy_count is loaded with the last dummy clock and counts down to 0,
    // so fast reads support 1..8 wait clocks. Verilog-2001 has no cast to
    // narrow the parameter expression, so the truncation is waived here and
    // the range is enforced by the generate guard below instead.
    /* verilator lint_off WIDTHTRUNC */
    localparam [2:0] FAST_READ_DUMMY_LAST = FAST_READ_DUMMY_CLKS - 1;
    /* verilator lint_on WIDTHTRUNC */
    generate
        if (FAST_READ_DUMMY_CLKS < 1 || FAST_READ_DUMMY_CLKS > 8) begin : g_dummy_range
            // Deliberately undefined: elaboration fails if dummy_count
            // violates the legacy generic fast-read parameter contract.
            FAST_READ_DUMMY_CLKS_must_be_1_to_8 unsupported_dummy_clks();
        end
    endgenerate
    reg is_fast_read = 0;
    reg is_dual_read = 0;       // Set for both 0x3B and 0xBB
    reg is_quad_read = 0;       // Set for both 0x6B and 0xEB
    // Last bit_count_in value of a data byte in STA_READ: 7 for single,
    // 3 for dual and 1 for quad output (SPI clocks per byte - 1).
    reg [2:0] read_byte_top = 7;
    reg is_sfdp_read = 0;       // Set for CMD 0x5A (Read SFDP)
    reg [4:0] mode_count = 0;   // Includes configured multi-IO latency.
    
    // AAI (Auto Address Increment) state -- persists across CS cycles
    reg aai_active = 0;         // In AAI word program mode
    reg is_aai = 0;             // Current transaction is AAI (per-CS flag)
    reg [1:0] aai_bytes_left;   // Ignore data beyond the two-byte AAI word

    // Mode-byte continuous reads: M5-4 = 10 skips the next opcode.
    // Micron profile BB/EB waits are dummy-only, not a mode byte.
    // Persist the accepted address/latency framing across CS; reseed/reset
    // clears the mode so it cannot leak into a different chip profile.
    reg cont_read = 0;
    reg cont_quad = 0;          // ...for 1-4-4 (else 1-2-2)
    reg cont_4b = 0;            // ...with a four-byte address
    reg [4:0] cont_wait = 0;
    reg io_read_4b = 0;         // Current 1-2-2/1-4-4 read has a 4-byte address
    
    reg [31:0] addr;
    reg [4:0] addr_count;       // Index of the next address MSB to arrive
    reg addr_4byte;

    // STA_ADDR receives addr_lanes (1, 2 or 4) bits per clock, MSB first,
    // then continues according to addr_kind.
    localparam [2:0]
        ADDR_KIND_READ=0, ADDR_KIND_WRITE=1, ADDR_KIND_ERASE=2,
        ADDR_KIND_PROTECT=3, ADDR_KIND_CONFIG=4, ADDR_KIND_CONFIGWRITE=5, ADDR_KIND_ID=6;
    reg [2:0] addr_lanes = 1;
    reg [2:0] addr_kind = ADDR_KIND_READ;
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
    // addr_count is 5 bits and counts down by addr_lanes (1, 2 or 4) per
    // address clock; the subtraction is done at addr_count's width.
    wire addr_last = addr_count == ({2'b00, addr_lanes} - 5'd1);
    
    // Set on the clock that starts a new burst in STA_READ; the first byte
    // of the burst comes straight from live_buffer.
    reg fresh_read = 0;
    wire [63:0] live_buffer;
    wire [3:0] live_beats;
    wire [2:0] next_byte = addr[2:0] + 3'd1;

    // Status registers
    reg [7:0] status_reg = 8'b00000000;
    reg [7:0] status_reg2 = 8'h02;     // Legacy generic default; profiles reseed it.
    
    // Synchronize write_done from the system clock domain into spi_clk.
    // glue.v toggles write_done once per completed program/erase, so the
    // event is preserved even if spi_clk is stopped while the write runs.
    reg [2:0] write_done_sync = 0;
    wire write_busy_clr = write_done_sync[1] ^ write_done_sync[2];

    `include "spi_chip_profile.vh"

    // SDRAM prefetch. Each strobe marks the clock on which the SPI
    // transaction reaches the corresponding point; spi_prefetch documents
    // what it does with it.
    // The issuing transaction of a pending write command is still selected.
    reg write_open = 0;

    // Share descriptor admission with OP_READ, including completion on this
    // edge, profile erase priority and the Atmel 3C register-read alias.
    wire read_enabled = (!command_native || cfg_profile_valid || cfg_4byte) &&
                        (command_data_lanes != 2 || quad_enabled);
    wire array_read_accepted = can_mutate && operation == OP_READ && read_enabled &&
                              !(cfg_profile_valid && profile_erase_match) &&
                              !(is_atmel && received_byte == 8'h3c);

    wire addr_read = state == STA_ADDR && addr_kind == ADDR_KIND_READ && !is_sfdp_read;
    wire in_read = state == STA_READ;

    spi_prefetch prefetch(
        .spi_clk(spi_clk),
        .active(is_selected),
        .restart(reset_cs || reset_power),
        .addr_mask(cfg_chip_erase_bursts),
        .fresh_read(fresh_read),
        .in_read(in_read),
        .sample_missing(sample_missing),
        .release_inhibit(state == STA_CMD && bit_count_in == 0 &&
                         !array_read_accepted),
        // Quad: IO3/IO2 carry byte-address bits 11/10 on this clock.
        .first_activate(addr_read && addr_count == (addr_quad ? 11 : 9)),
        .first_row(addr_quad ? {addr[13:0], spi_io3_in, spi_io2_in} : addr[15:0]),
        .first_read(addr_read && addr_count == 3),
        .first_col({addr[5:0], addr_lane_msb}),
        .first_done(addr_read && addr_last && !addr_dual && !addr_quad),
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
        .ram_read_beats_a(ram_read_beats_a),
        .ram_read_beats_b(ram_read_beats_b),
        .live_buffer(live_buffer),
        .live_beats(live_beats),
        .prefetch_underrun(prefetch_underrun),
        .prefetch_thin(prefetch_thin)
    );

    // Main SPI state machine
    always @(posedge spi_clk) begin
        write_done_sync <= {write_done_sync[1:0], write_done};

        // SCK can keep running for another slave while this CS is high.
        // Consume completion independently of selection, before the
        // selected state machine (which may start a new operation).
        if (write_busy_clr) begin
            status_reg[0] <= 0;
            write_cmd <= 0;
        end

        if (is_selected) begin
            fresh_read <= 0;
            
            if (reset_cs || reset_power) begin
                bit_count_in <= 6;
                mosi_byte <= {spi_io0_in, 7'b0};
                miso_byte <= 0;
                miso_byte_valid <= 0;
                
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
                status_read_index <= 0;
                status_write_index <= 0;
                erase_custom <= 0;
                program_quad <= 0;
                transaction_wait <= 0;
                byte_programmed <= 0;
                mode_count <= 0;
                transaction_opcode <= 0;
                register_index <= 0;
                
                log_cmd_valid <= 0;
                log_addr_valid <= 0;
                log_byte_count <= 0;
                
                write_open <= 0;
                write_buf_strobe <= 0;
                
                if (reset_power) begin
                    write_cmd <= 0;
                    status_reg[1:0] <= 2'b00;
                    if (!cfg_profile_valid || command_flags[3]) status_reg[6] <= 0;
                    addr_4byte <= 0;
                    extended_address <= 0;
                    reset_enabled <= 0;
                    aai_active <= 0;
                    cont_read <= 0;
                    addr <= 0;
                end
                if (cfg_profile_valid && (!profile_loaded || profile_epoch_seen != cfg_profile_epoch)) begin
                    profile_loaded <= 1;
                    profile_epoch_seen <= cfg_profile_epoch;
                    status_reg <= profile_byte(12) & 8'hfc;
                    status_reg2 <= profile_byte(13);
                    status_reg3 <= profile_byte(14);
                    nvcr <= 16'hffff; vcr <= 8'hfb; evcr <= 8'hff;
                    config_reg3 <= 0; config_reg4 <= 0; config_reg5 <= 0;
                    addr_4byte <= 0;
                    extended_address <= 0;
                    reset_enabled <= 0;
                    aai_active <= 0;
                    cont_read <= 0;
                    addr <= 0;
                end else if (!cfg_profile_valid) begin
                    profile_loaded <= 0;
                end
                if (!reset_power &&
                    !(cfg_profile_valid && (!profile_loaded || profile_epoch_seen != cfg_profile_epoch)) &&
                    cont_read && can_mutate) begin
                    // Continuous read: this first clock already carries
                    // the address MSBs of an implied 0xBB/0xEB (or 4B).
                    state <= STA_ADDR;
                    addr_kind <= ADDR_KIND_READ;
                    transaction_wait <= cont_wait;
                    io_read_4b <= cont_4b;
                    log_cmd_valid <= 1;
                    if (cont_quad) begin
                        addr_lanes <= 4;
                        is_quad_read <= 1;
                        read_byte_top <= 1;
                        addr <= {28'b0, spi_io3_in, spi_io2_in, spi_io1_in, spi_io0_in};
                        addr_count <= cont_4b ? 5'd27 : 5'd19;
                        log_cmd_opcode <= cont_4b ? CMD_QUADIOREAD_4B : CMD_QUADIOREAD;
                    end
                    else begin
                        addr_lanes <= 2;
                        is_dual_read <= 1;
                        read_byte_top <= 3;
                        addr <= {30'b0, spi_io1_in, spi_io0_in};
                        addr_count <= cont_4b ? 5'd29 : 5'd21;
                        log_cmd_opcode <= cont_4b ? CMD_DUALIOREAD_4B : CMD_DUALIOREAD;
                    end
                end
            end
            else begin
                log_cmd_valid <= 0;
                log_addr_valid <= 0;
                    
                write_buf_strobe <= 0;

                // A completed byte leaves the transaction at a boundary.
                if (write_open)
                    write_partial <= bit_count_in != 0;
                
                if (state == STA_WRITE && program_quad)
                    mosi_byte <= {mosi_byte[3:0], spi_io3_in, spi_io2_in, spi_io1_in, spi_io0_in};
                else mosi_byte[bit_count_in] <= spi_io0_in;


                if ((state == STA_CMD) && (bit_count_in == 0)) begin
                    
                    transaction_opcode <= received_byte;
                    state <= STA_IGNORE; // extra bytes cannot become new commands within CS
                    if (cfg_profile_valid && profile_erase_match) begin
                        if (status_reg[1] && can_mutate && !aai_active) begin
                            state <= STA_ADDR;
                            addr <= {extended_address, 24'd0};
                            addr_kind <= ADDR_KIND_ERASE;
                            addr_count <= (profile_erase_native || addr_4byte) ? 31 : 23;
                            erase_descriptor <= profile_erase_index;
                            erase_custom <= 1;
                        end
                    end else if (can_mutate || operation == OP_SR1 || operation == OP_SR2 ||
                                 operation == OP_SR3 || operation == OP_FLAG || operation == OP_CONFIGREAD) case (operation)
                        
                    OP_SR1, OP_SR2, OP_SR3, OP_BANKREAD, OP_FLAG: begin
                        if (operation == OP_SR3 && cfg_profile_valid && command_flags[6]) begin
                            state <= STA_READID; miso_byte <= cfg_jedec_id[7:0]; addr_count <= 1;
                        end else begin
                            state <= STA_READSTATUS;
                            status_read_index <= operation == OP_SR2 || (operation == OP_SR3 && is_macronix) ? 1 : operation == OP_SR3 ? 2 : 0;
                            miso_byte <= operation == OP_SR2 || (operation == OP_SR3 && is_macronix) ? status_reg2_value : operation == OP_SR3 ? status_reg3 :
                                operation == OP_BANKREAD ? extended_address : operation == OP_FLAG ?
                                (status_reg[0] ? 8'h00 : 8'h80) : status_value;
                        end
                        spi_io1_oe_ff <= 1;
                    end
                    OP_WRSR, OP_WRSR2, OP_WRSR3: begin
                        if ((cfg_profile_valid || status_reg[1]) && !aai_active) begin
                            status_write_index <= operation == OP_WRSR2 ? 1 : operation == OP_WRSR3 ? 2 : 0;
                            state <= STA_WRITESTATUS; addr_count <= 0;
                        end
                    end
                    
                    OP_WRDI: begin
                        status_reg[1] <= 0;
                        if (!cfg_profile_valid || command_flags[3]) status_reg[6] <= 0;
                        aai_active <= 0;
                    end
                    
                    OP_WREN: begin
                        if (can_mutate && !aai_active) status_reg[1] <= 1;
                    end
                    
                    OP_ENTER4, OP_EXIT4: begin
                        if (cfg_profile_valid || cfg_4byte) addr_4byte <= operation == OP_ENTER4;
                    end
                        
                    OP_READ: begin
                        if (read_enabled) begin
                            state <= STA_ADDR;
                            addr <= {extended_address,24'd0};
                            addr_count <= (command_native || addr_4byte) ? 31 : 23;
                            addr_lanes <= command_address_lanes == 2 ? 4 : command_address_lanes == 1 ? 2 : 1;
                            is_dual_read <= command_data_lanes == 1;
                            is_quad_read <= command_data_lanes == 2;
                            read_byte_top <= command_data_lanes == 2 ? 1 : command_data_lanes == 1 ? 3 : 7;
                            is_fast_read <= command_wait != 0 && command_address_lanes == 0;
                            transaction_wait <= selected_wait;
                            io_read_4b <= command_native || addr_4byte;
                            if (is_atmel && received_byte == 8'h3c) begin
                                addr_kind <= ADDR_KIND_PROTECT; addr_count <= 23;
                                is_fast_read <= 0; is_dual_read <= 0;
                            end
                        end
                    end
                    OP_ERASE: begin
                        if (status_reg[1] && !aai_active) begin
                            if (received_byte == 8'hc7 || received_byte == 8'h60 || received_byte == 8'h62) begin
                                state <= STA_ERASE; write_cmd <= 1; write_type <= 2'd1;
                                write_open <= 1; write_partial <= 0;
                                write_addr <= 0; write_len <= cfg_chip_erase_bursts;
                                status_reg[1:0] <= 2'b01;
                            end else begin
                                state <= STA_ADDR; addr <= {extended_address,24'd0};
                                addr_kind <= ADDR_KIND_ERASE;
                                addr_count <= (command_native || addr_4byte) ? 31 : 23;
                                write_len <= command_erase_size == 0 ? 23'h001ff :
                                    command_erase_size == 1 ? 23'h00fff : 23'h01fff;
                            end
                        end
                    end
                    OP_PROGRAM: begin
                        if (status_reg[1] && !aai_active &&
                            (received_byte != 8'h38 || cfg_jedec_id[7:0] == 8'hc2)) begin
                            state <= STA_ADDR; addr <= {extended_address,24'd0};
                            addr_kind <= ADDR_KIND_WRITE;
                            addr_count <= (command_native || addr_4byte) ? 31 : 23;
                            addr_lanes <= command_address_lanes == 2 ? 4 : 1;
                            program_quad <= command_data_lanes == 2;
                            write_len <= 23'h0001f;
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
                    OP_AAI: begin
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
                                write_open <= 1;
                                write_partial <= 0;
                                write_type <= 2'd2;
                                write_addr <= wrap_burst_addr(addr[25:3]);
                                write_len <= 0;
                                status_reg[0] <= 1;
                            end
                        end
                    end
                    
                    OP_SFDP: begin
                        state <= STA_ADDR;
                        addr <= 0;
                        addr_count <= 23;        // Always 3-byte address for SFDP
                        is_fast_read <= 1;       // 8 dummy clocks after address
                        is_sfdp_read <= 1;
                    end
                    
                    OP_SECTOR, OP_LOCKFF: begin
                        state <= STA_ADDR; addr <= 0; addr_kind <= ADDR_KIND_PROTECT;
                        addr_count <= addr_4byte ? 31 : 23;
                        status_reg[1] <= 0;
                    end
                    OP_ZERO: begin
                        state <= STA_BPRREAD; spi_io1_oe_ff <= 1; miso_byte <= 0;
                    end
                    OP_UNLOCK: status_reg[1] <= 0;
                    OP_EWSR: if (!cfg_profile_valid) status_reg[1] <= 1;
                    OP_BANKWRITE: state <= STA_REGWRITE;
                    OP_CONFIGREAD: begin
                        if (is_spansion && received_byte == 8'h65) begin
                            state <= STA_ADDR; addr <= 0; addr_kind <= ADDR_KIND_CONFIG;
                            addr_count <= addr_4byte ? 31 : 23;
                        end else begin
                            state <= STA_CONFIGREAD; register_index <= 0; spi_io1_oe_ff <= 1;
                            miso_byte <= received_byte == 8'hb5 ? nvcr[7:0] : received_byte == 8'h85 ? vcr : evcr;
                        end
                    end
                    OP_ANYWRITE: if (is_spansion) begin
                        state <= STA_ADDR; addr <= 0; addr_kind <= ADDR_KIND_CONFIGWRITE;
                        addr_count <= addr_4byte ? 31 : 23;
                    end
                    OP_LEGACYID: begin
                        state <= STA_ADDR; addr <= 0; addr_kind <= ADDR_KIND_ID; addr_count <= 23;
                    end
                    OP_CONFIGWRITE: begin
                        state <= STA_CONFIGWRITE; register_index <= 0;
                    end
                    OP_UID: begin
                        state <= STA_UID_DUMMY;
                        uid_dummy <= 31;
                        register_index <= 0;
                    end
                    OP_RESET_ENABLE: reset_enabled <= 1;
                    OP_RESET: begin
                        if (reset_enabled) begin
                            reset_enabled <= 0;
                            extended_address <= 0;
                            // NVCR survives soft reset and determines subsequent
                            // address/dummy framing; no protection state exists.
                            vcr <= is_micron ? {nvcr[15:12],4'hb} : 8'hfb; evcr <= 8'hff;
                            addr_4byte <= is_micron ? !nvcr[0] : is_spansion ? status_reg3[7] : 1'b0;
                            if (is_micron && !nvcr[1]) extended_address <= {6'b0,cfg_chip_erase_bursts[22:21]};
                            aai_active <= 0;
                            cont_read <= 0;
                            status_reg[1:0] <= 0;
                            if (command_flags[3]) status_reg[6] <= 0;
                        end
                    end
                    OP_ID: begin
                        state <= STA_READID;
                        spi_io1_oe_ff <= 1;
                        miso_byte <= cfg_jedec_id[7:0];
                        addr_count <= 1;
                    end
                    
                    endcase
                    // Seed EAR before shifting in the 24-bit address. In a
                    // four-byte transaction it is shifted out naturally.
                    if (!aai_active && (profile_erase_match || operation == OP_READ ||
                        operation == OP_PROGRAM || operation == OP_ERASE || operation == OP_AAI))
                        addr <= {24'd0, extended_address};
                    if (received_byte != 8'h66 && received_byte != 8'h99) reset_enabled <= 0;
                    log_cmd_valid <= 1;
                    log_cmd_opcode <= {mosi_byte[7:1], spi_io0_in};
                    log_byte_count <= 0;
                end
                else if ((state == STA_READSTATUS) && (bit_count_in == 0)) begin
                    miso_byte <= transaction_opcode == 8'h70 ? (status_reg[0] ? 8'h00 : 8'h80) :
                        (transaction_opcode == 8'hc8 || transaction_opcode == 8'h16) ? extended_address :
                        status_read_index == 1 ? status_reg2_value : status_read_index == 2 ? status_reg3 : status_value;
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
                    addr_count <= addr_count - {2'b00, addr_lanes};

                    if (addr_last) begin
                        log_addr_valid <= 1;
                        if (addr_kind == ADDR_KIND_READ && !is_sfdp_read)
                            log_addr_toggle <= !log_addr_toggle;
                        log_addr_out <= addr_next;

                        case (addr_kind)
                        ADDR_KIND_READ: begin
                            if (addr_dual || addr_quad) begin
                                // ROM default or register-selected latency;
                                // the paired descriptor already chose lanes.
                                state <= STA_MODE_MULTI;
                                mode_count <= transaction_wait - 5'd1;
                            end
                            else begin
                                if (is_fast_read) begin
                                    state <= STA_DUMMY;
                                    dummy_count <= is_sfdp_read || !cfg_profile_valid ? {2'b0,FAST_READ_DUMMY_LAST} : transaction_wait - 5'd1;
                                end
                                else begin
                                    state <= STA_READ;
                                    spi_io1_oe_ff <= 1;
                                    fresh_read <= 1;
                                end
                            end
                        end

                        ADDR_KIND_ERASE: begin
                            status_reg[1] <= 0;
                            if (!erase_custom || erase_region_found) begin
                                state <= STA_ERASE;
                                write_cmd <= 1;
                                write_open <= 1;
                                write_partial <= 0;
                                write_type <= 2'd1;
                                write_addr <= wrap_burst_addr(selected_erase_base[25:3]);
                                write_len <= selected_erase_mask;
                                status_reg <= (status_reg & ~profile_byte(7) & 8'hfd) | 8'h01;
                            end else begin
                                state <= STA_IGNORE;
                                status_reg <= (status_reg & 8'hfd) | profile_byte(7);
                            end
                        end
                        ADDR_KIND_PROTECT: begin
                            if (transaction_opcode == 8'h3c) begin
                                state <= STA_SECTORREAD;
                                spi_io1_oe_ff <= 1;
                                miso_byte <= 8'h00;
                            end else if (transaction_opcode == 8'he0 || transaction_opcode == 8'he2) begin
                                state <= STA_SECTORREAD; spi_io1_oe_ff <= 1; miso_byte <= 8'hff;
                            end else begin
                                state <= STA_IGNORE; status_reg[1] <= 0;
                            end
                        end

                        ADDR_KIND_CONFIG: begin
                            state <= STA_CONFIGREAD; register_index <= addr_next[4:0];
                            spi_io1_oe_ff <= 1; miso_byte <= addressed_config(addr_next[4:0]);
                        end
                        ADDR_KIND_CONFIGWRITE: begin
                            state <= STA_ANYWRITE; register_index <= addr_next[4:0];
                        end
                        ADDR_KIND_ID: begin
                            state <= STA_LEGACYID; spi_io1_oe_ff <= 1;
                            miso_byte <= transaction_opcode == 8'hab || addr_next[0] ?
                                cfg_jedec_id[23:16]-8'd1 : cfg_jedec_id[7:0];
                            register_index[0] <= !addr_next[0];
                        end
                        default: begin // ADDR_KIND_WRITE
                            if (is_aai) begin
                                // Don't clear WEL between AAI words.
                                state <= STA_AAI_DATA;
                                aai_bytes_left <= 2;
                                write_cmd <= 1;
                                write_open <= 1;
                                write_partial <= 0;
                                write_type <= 2'd2;
                                write_addr <= wrap_burst_addr(addr_next[25:3]);
                                write_len <= 0;
                                status_reg[0] <= 1;
                            end else begin
                                state <= STA_WRITE;
                                write_cmd <= 1;
                                write_open <= 1;
                                write_partial <= 0;
                                write_type <= 2'd0;

                                // Page-aligned address in 8-byte burst units
                                // byte_addr[25:3] -> burst address, page = 256 bytes = 32 bursts
                                write_addr <= wrap_burst_addr({addr_next[25:8], 5'b0});

                                status_reg <= (status_reg & ~profile_byte(7) & 8'hfd) | 8'h01;
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
                            miso_byte <= (!cfg_profile_valid || addr < 32'd128) ? sfdp_rdata : 8'hff;
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
                        miso_byte <= live_buffer[next_byte*8 +: 8];
                        miso_byte_valid <= live_beats[next_byte[2:1]];
                        addr <= addr + 1;
                        log_byte_count <= log_byte_count + 1;
                    end

                    if (fresh_read) begin
                        miso_byte <= live_buffer[addr[2:0]*8 +: 8];
                        miso_byte_valid <= live_beats[addr[2:1]];
                    end
                end
                else if (state == STA_READID) begin
                    if (bit_count_in == 0) begin
                        if (addr_count < ((cfg_profile_valid && command_flags[6]) ? 2 : 3)) begin
                            miso_byte <= cfg_jedec_id[addr_count*8 +: 8];
                            addr_count <= addr_count + 1;
                        end
                        else
                            miso_byte <= 0;
                    end
                end
                else if ((state == STA_WRITE) && (bit_count_in == 0)) begin
                    if (!cfg_profile_valid || !command_flags[4] || !byte_programmed) begin
                        write_buf_strobe <= 1;
                        write_buf_offset <= addr[7:0];
                        write_buf_val <= program_byte;
                        byte_programmed <= 1;
                        addr[7:0] <= cfg_profile_valid ?
                            ((addr[7:0] & ~page_mask) | ((addr[7:0] + 8'd1) & page_mask)) : addr[7:0] + 8'd1;
                    end
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
                // ---------------------------------------------------------
                // Mode/dummy phase for multi-IO reads. Legacy defaults:
                // BB has 4 mode clocks, EB has 2 mode + 4 dummy clocks.
                // Profile registers can change the total wait; Micron has
                // dummy-only waits. Prefetch posts during the final clocks.
                // ---------------------------------------------------------
                else if (state == STA_MODE_MULTI) begin
                    // M5-4 arrive on IO1:IO0: in the first mode clock for
                    // quad (M7-4 on IO3:IO0), the second for dual.
                    if (!is_micron && mode_count == transaction_wait - (is_quad_read ? 5'd1 : 5'd2)) begin
                        cont_read <= {spi_io1_in, spi_io0_in} == 2'b10;
                        cont_quad <= is_quad_read;
                        cont_4b <= io_read_4b;
                        cont_wait <= transaction_wait;
                    end
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
                        // Compatibility profiles must not alias the header when
                        // the 7-bit table RAM address wraps at byte 128.
                        miso_byte <= (!cfg_profile_valid || addr < 32'd127) ? sfdp_rdata : 8'hff;
                        addr <= addr + 1;
                        log_byte_count <= log_byte_count + 1;
                    end
                end
                // ---------------------------------------------------------
                // Write Status Register data reception (CMD 0x01)
                // Receives 1 byte (SR1) or 2 bytes (SR1 + SR2).
                // ---------------------------------------------------------
                else if (state == STA_WRITESTATUS && bit_count_in == 0) begin
                    status_reg[1] <= 0;
                    if (addr_count == 0) begin
                        case (status_write_index)
                            0: status_reg[7:2] <= received_byte[7:2];
                            1: status_reg2 <= received_byte;
                            2: status_reg3 <= received_byte;
                            default: ;
                        endcase
                        addr_count <= 1;
                        if (status_write_index != 0) state <= STA_IGNORE;
                    end else if (addr_count == 1) begin
                        status_reg2 <= received_byte; addr_count <= 2;
                    end else if (addr_count == 2) begin
                        status_reg3 <= received_byte; state <= STA_IGNORE;
                    end
                end
                else if (state == STA_ANYWRITE && bit_count_in == 0) begin
                    case (register_index)
                        0: status_reg[7:2] <= received_byte[7:2];
                        2: status_reg2 <= received_byte;
                        3: begin status_reg3 <= received_byte; addr_4byte <= received_byte[7]; end
                        4: config_reg3 <= received_byte;
                        5: config_reg4 <= received_byte;
                        6: config_reg5 <= received_byte;
                        default: ;
                    endcase
                    status_reg[1] <= 0; state <= STA_IGNORE;
                end
                else if (state == STA_LEGACYID && bit_count_in == 0) begin
                    miso_byte <= transaction_opcode == 8'hab || register_index[0] ?
                        cfg_jedec_id[23:16]-8'd1 : cfg_jedec_id[7:0];
                    register_index[0] <= !register_index[0];
                end
                else if (state == STA_CONFIGREAD && bit_count_in == 0) begin
                    if (transaction_opcode == 8'hb5) begin
                        miso_byte <= register_index[0] ? nvcr[7:0] : nvcr[15:8];
                        register_index[0] <= !register_index[0];
                    end else miso_byte <= config_byte;
                end
                else if (state == STA_CONFIGWRITE && bit_count_in == 0) begin
                    status_reg[1] <= 0;
                    if (transaction_opcode == 8'hb1) begin
                        if (register_index[0]) begin
                            nvcr[15:8] <= received_byte; state <= STA_IGNORE;
                        end else begin nvcr[7:0] <= received_byte; register_index[0] <= 1; end
                    end else begin
                        if (transaction_opcode == 8'h81) vcr <= received_byte;
                        else evcr <= received_byte;
                        state <= STA_IGNORE;
                    end
                end
                else if (state == STA_REGWRITE && bit_count_in == 0) begin
                    extended_address <= received_byte;
                    if (address_flags[2] && ((transaction_opcode == 8'hc5 && address_flags[3]) ||
                        (transaction_opcode == 8'h17 && address_flags[4]))) addr_4byte <= received_byte[7];
                    status_reg[1] <= 0;
                    state <= STA_IGNORE;
                end
                else if (state == STA_UID_DUMMY) begin
                    if (uid_dummy == 0) begin
                        state <= STA_UID_READ;
                        spi_io1_oe_ff <= 1;
                        miso_byte <= profile_byte(16);
                        register_index <= 1;
                    end else uid_dummy <= uid_dummy - 1'b1;
                end
                else if (state == STA_UID_READ && bit_count_in == 0) begin
                    if (register_index < 8) begin
                        miso_byte <= profile_uid[register_index[2:0]*8 +: 8];
                        register_index <= register_index + 1'b1;
                    end else miso_byte <= 8'hff;
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
                else if ((state == STA_WRITE && bit_count_in == 0) ||
                         (state == STA_ADDR && addr_last && addr_kind == ADDR_KIND_WRITE && !is_aai))
                    bit_count_in <= program_byte_top;
                else bit_count_in <= bit_count_in - 1;
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
        // Record readiness on exactly the path used to drive the next
        // sample: first bits use live SDRAM, later bits use the byte latch.
        // The following posedge is the master's sampling edge. Checking
        // only at burst end would miss short reads and late-arriving fills.
        sample_missing <= is_selected && in_read &&
            !(fresh_read ? live_beats[addr[2:1]] : miso_byte_valid);
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
