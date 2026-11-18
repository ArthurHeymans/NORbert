// Controller protocol hints, not a model of flash protection circuitry.
//
// profile_byte reads the held system-domain cfg_profile bundle. It is
// used only at the per-transaction capture edge below and for the
// status reseed values read at that same edge in spi_trx; all runtime
// decode below consumes the captured SCK-domain shadow fields, so no
// decode path crosses clock domains. Configuration must be stable from
// CS falling through that first clock (the host only reconfigures while
// emulation is stopped, which is stronger).
function [7:0] profile_byte;
    input integer index;
    begin profile_byte = cfg_profile[index*8 +: 8]; end
endfunction
always @(posedge spi_clk) begin
    if (is_selected && (reset_cs || reset_power)) begin
        sh_valid <= cfg_profile_valid;
        sh_jedec <= cfg_jedec_id;
        sh_4byte <= cfg_4byte;
        sh_erase_bursts <= cfg_chip_erase_bursts;
        sh_cmd_flags <= profile_byte(11);
        sh_addr_flags <= profile_byte(10);
        sh_p2 <= profile_byte(2);
        sh_p7 <= profile_byte(7);
        sh_is_atmel <= cfg_profile_valid && profile_byte(1) == 2;
        sh_uid <= cfg_profile[191:128];
        sh_page_mask <= 8'hff >> (4'd8 - {1'b0, cfg_profile[227:224]});
        sh_erase_ops <= cfg_profile[319:256];
        sh_erase_cnts <= cfg_profile[331:320];
    end
end
wire [7:0] received_byte = {mosi_byte[7:1], spi_io0_in};
wire [7:0] command_flags = sh_cmd_flags;
wire [7:0] address_flags = sh_addr_flags;
wire [25:0] capacity_mask = {sh_erase_bursts, 3'b111};
wire is_atmel = sh_is_atmel;
wire [63:0] profile_uid = sh_uid;
wire is_macronix = sh_valid && sh_jedec[7:0] == 8'hc2;
wire is_spansion = sh_valid && sh_jedec[7:0] == 8'h01;
wire [7:0] status_value = sh_valid ?
    ((status_reg & ~(sh_p2 | sh_p7 | 8'h80) & 8'hfc) |
     {6'b0,status_reg[1:0]} | (is_atmel ? 8'h10 : 8'h00)) : status_reg;
wire [7:0] status_reg2_value = is_macronix ?
    {status_reg2[7:6],addr_4byte,status_reg2[4:0]} : sh_valid ? status_reg2 & 8'hbe : status_reg2;
wire can_mutate = !status_reg[0] || write_busy_clr;
wire quad_enabled = sh_valid || status_reg2[1];
reg [7:0] transaction_opcode;
reg [31:0] profile_epoch_seen = 0;
reg profile_loaded = 0;
reg [7:0] status_reg3 = 0;
reg [15:0] nvcr = 16'hffff;
reg [7:0] vcr = 8'hfb, evcr = 8'hff;
reg [7:0] config_reg3 = 0, config_reg4 = 0, config_reg5 = 0;
wire is_micron = sh_valid && sh_jedec[7:0] == 8'h20;
function [7:0] addressed_config;
    input [4:0] index;
    begin
        case (index)
            0: addressed_config = status_value;
            2: addressed_config = status_reg2_value;
            3: addressed_config = {addr_4byte,status_reg3[6:0]};
            4: addressed_config = config_reg3 & 8'hef; // Hardware page buffer <=256.
            5: addressed_config = config_reg4;
            6: addressed_config = config_reg5;
            default: addressed_config = 0;
        endcase
    end
endfunction
wire [7:0] config_byte = is_spansion && transaction_opcode == 8'h65 ? addressed_config(register_index) :
    transaction_opcode == 8'hb5 ? (register_index[0] ? nvcr[15:8] : nvcr[7:0]) :
    transaction_opcode == 8'h85 ? vcr : evcr;
// Default waits are descriptor fields, except Micron's 8/10-clock multi-IO
// defaults. Configuration writes change transaction timing, not protection.
wire [4:0] micron_default_wait = command_address_lanes == 2 ? 5'd10 : 5'd8;
wire [4:0] spansion_wait = status_reg3[3:0] == 4'hb ? 5'd24 : {1'b0,status_reg3[3:0]};
wire [4:0] macronix_wait = command_address_lanes == 2 ?
    (status_reg2[7:6] == 1 ? 5'd4 : status_reg2[7:6] == 2 ? 5'd8 : 5'd6) :
    command_address_lanes == 1 ?
    (status_reg2[7:6] == 1 ? 5'd6 : status_reg2[7:6] == 2 ? 5'd8 : 5'd4) :
    (status_reg2[7:6] == 1 ? 5'd6 : 5'd8);
wire [4:0] selected_wait = is_micron ?
    (vcr[7:4] == 0 || vcr[7:4] == 15 ? micron_default_wait : {1'b0,vcr[7:4]}) :
    is_spansion && status_reg3[3:0] != 0 ? spansion_wait :
    is_macronix ? macronix_wait : {1'b0,command_wait};
reg [1:0] status_read_index = 0;
reg [1:0] status_write_index = 0;
reg reset_enabled = 0;
reg [7:0] extended_address = 0;
reg [4:0] register_index;
reg [5:0] uid_dummy;
reg [1:0] erase_descriptor;
reg erase_custom = 0;
reg byte_programmed;
reg program_quad = 0;
wire [2:0] program_byte_top = program_quad ? 3'd1 : 3'd7;
wire [7:0] program_byte = program_quad ?
    {mosi_byte[3:0], spi_io3_in, spi_io2_in, spi_io1_in, spi_io0_in} : received_byte;
wire [7:0] page_mask = sh_page_mask;

// Paired opcode descriptors: read both possible final-bit commands on the
// penultimate command clock. The last bit selects one on the decode edge,
// without requiring an extra master clock or an asynchronous RAM read.
localparam [4:0] OP_IGNORE=0, OP_READ=1, OP_PROGRAM=2, OP_ERASE=3,
    OP_SR1=4, OP_SR2=5, OP_SR3=6, OP_WRSR=7, OP_WRSR2=8,
    OP_WREN=9, OP_WRDI=10, OP_ID=11, OP_SFDP=12, OP_AAI=13,
    OP_ENTER4=14, OP_EXIT4=15, OP_BANKWRITE=16, OP_BANKREAD=17,
    OP_UID=18, OP_RESET_ENABLE=19, OP_RESET=20, OP_UNLOCK=21,
    OP_SECTOR=22, OP_ZERO=23, OP_FLAG=24, OP_WRSR3=25,
    OP_CONFIGREAD=26, OP_CONFIGWRITE=27, OP_LOCKFF=28, OP_EWSR=29,
    OP_ANYWRITE=30, OP_LEGACYID=31;
// {unused, erase-size/descriptor, wait clocks, data lanes, address lanes,
//  native-four-byte address, operation}. Lane codes: 0 single, 1 dual, 2 quad.
function [16:0] opcode_descriptor;
    input [7:0] op;
    begin
        opcode_descriptor = 0;
        case (op)
            8'h03: opcode_descriptor = {1'b0,2'd0,4'd0,2'd0,2'd0,1'b0,OP_READ};
            8'h13: opcode_descriptor = {1'b0,2'd0,4'd0,2'd0,2'd0,1'b1,OP_READ};
            8'h0b: opcode_descriptor = {1'b0,2'd0,4'd8,2'd0,2'd0,1'b0,OP_READ};
            8'h0c: opcode_descriptor = {1'b0,2'd0,4'd8,2'd0,2'd0,1'b1,OP_READ};
            8'h3b: opcode_descriptor = {1'b0,2'd0,4'd8,2'd1,2'd0,1'b0,OP_READ};
            8'h3c: opcode_descriptor = {1'b0,2'd0,4'd8,2'd1,2'd0,1'b1,OP_READ};
            8'h6b: opcode_descriptor = {1'b0,2'd0,4'd8,2'd2,2'd0,1'b0,OP_READ};
            8'h6c: opcode_descriptor = {1'b0,2'd0,4'd8,2'd2,2'd0,1'b1,OP_READ};
            8'hbb: opcode_descriptor = {1'b0,2'd0,4'd4,2'd1,2'd1,1'b0,OP_READ};
            8'hbc: opcode_descriptor = {1'b0,2'd0,4'd4,2'd1,2'd1,1'b1,OP_READ};
            8'heb: opcode_descriptor = {1'b0,2'd0,4'd6,2'd2,2'd2,1'b0,OP_READ};
            8'hec: opcode_descriptor = {1'b0,2'd0,4'd6,2'd2,2'd2,1'b1,OP_READ};
            8'h02: opcode_descriptor = {1'b0,2'd0,4'd0,2'd0,2'd0,1'b0,OP_PROGRAM};
            8'h12: opcode_descriptor = {1'b0,2'd0,4'd0,2'd0,2'd0,1'b1,OP_PROGRAM};
            8'h32: opcode_descriptor = {1'b0,2'd0,4'd0,2'd2,2'd0,1'b0,OP_PROGRAM};
            8'h34: opcode_descriptor = {1'b0,2'd0,4'd0,2'd2,2'd0,1'b1,OP_PROGRAM};
            8'h38: opcode_descriptor = {1'b0,2'd0,4'd0,2'd2,2'd2,1'b0,OP_PROGRAM};
            8'h20: opcode_descriptor = {1'b0,2'd0,4'd0,2'd0,2'd0,1'b0,OP_ERASE};
            8'h21: opcode_descriptor = {1'b0,2'd0,4'd0,2'd0,2'd0,1'b1,OP_ERASE};
            8'h52: opcode_descriptor = {1'b0,2'd1,4'd0,2'd0,2'd0,1'b0,OP_ERASE};
            8'h5c: opcode_descriptor = {1'b0,2'd1,4'd0,2'd0,2'd0,1'b1,OP_ERASE};
            8'hd8: opcode_descriptor = {1'b0,2'd2,4'd0,2'd0,2'd0,1'b0,OP_ERASE};
            8'hdc: opcode_descriptor = {1'b0,2'd2,4'd0,2'd0,2'd0,1'b1,OP_ERASE};
            8'hc7,8'h60,8'h62: opcode_descriptor = {12'b0,OP_ERASE};
            8'h05: opcode_descriptor = {12'b0,OP_SR1};
            8'h35: opcode_descriptor = {12'b0,OP_SR2};
            8'h15: opcode_descriptor = {12'b0,OP_SR3};
            8'h01: opcode_descriptor = {12'b0,OP_WRSR};
            8'h31: opcode_descriptor = {12'b0,OP_WRSR2};
            8'h11: opcode_descriptor = {12'b0,OP_WRSR3};
            8'h06: opcode_descriptor = {12'b0,OP_WREN};
            8'h04: opcode_descriptor = {12'b0,OP_WRDI};
            8'h9f,8'h9e: opcode_descriptor = {12'b0,OP_ID};
            8'h90,8'hab: opcode_descriptor = {12'b0,OP_LEGACYID};
            8'h5a: opcode_descriptor = {12'b0,OP_SFDP};
            8'had: opcode_descriptor = {12'b0,OP_AAI};
            8'hb7: opcode_descriptor = {12'b0,OP_ENTER4};
            8'he9,8'hb8: opcode_descriptor = {12'b0,OP_EXIT4};
            8'hc5,8'h17: opcode_descriptor = {12'b0,OP_BANKWRITE};
            8'hc8,8'h16: opcode_descriptor = {12'b0,OP_BANKREAD};
            8'h4b: opcode_descriptor = {12'b0,OP_UID};
            8'h66: opcode_descriptor = {12'b0,OP_RESET_ENABLE};
            8'h99: opcode_descriptor = {12'b0,OP_RESET};
            8'h42,8'h98: opcode_descriptor = {12'b0,OP_UNLOCK};
            8'h36,8'h39: opcode_descriptor = {12'b0,OP_SECTOR};
            8'h72: opcode_descriptor = {12'b0,OP_ZERO};
            8'h70: opcode_descriptor = {12'b0,OP_FLAG};
            8'hb5,8'h85,8'h65: opcode_descriptor = {12'b0,OP_CONFIGREAD};
            8'hb1,8'h81,8'h61: opcode_descriptor = {12'b0,OP_CONFIGWRITE};
            8'he0,8'he2: opcode_descriptor = {12'b0,OP_LOCKFF};
            8'h50: opcode_descriptor = {12'b0,OP_EWSR};
            8'h71: opcode_descriptor = {12'b0,OP_ANYWRITE};
            default: ; // EWSR/clear-error/power setup need no state or gating.
        endcase
    end
endfunction
reg [35:0] opcode_table [0:127];
reg [35:0] opcode_pair;
integer opcode_i;
initial for (opcode_i=0; opcode_i<128; opcode_i=opcode_i+1)
    opcode_table[opcode_i] = {1'b0,opcode_descriptor({opcode_i[6:0],1'b1}),
                             1'b0,opcode_descriptor({opcode_i[6:0],1'b0})};
always @(posedge spi_clk) opcode_pair <= opcode_table[{mosi_byte[7:2],spi_io0_in}];
wire [17:0] decoded_command = spi_io0_in ? opcode_pair[35:18] : opcode_pair[17:0];
wire [4:0] operation = decoded_command[4:0];
wire command_native = decoded_command[5];
wire [1:0] command_address_lanes = decoded_command[7:6];
wire [1:0] command_data_lanes = decoded_command[9:8];
wire [3:0] command_wait = decoded_command[13:10];
wire [1:0] command_erase_size = decoded_command[15:14];

// Only erase geometry and opcode collisions need chip-dependent tables.
reg profile_erase_match, profile_erase_native;
reg [1:0] profile_erase_index;
integer erase_i;
always @* begin
    profile_erase_match = 0; profile_erase_native = 0; profile_erase_index = 0;
    for (erase_i=0; erase_i<4; erase_i=erase_i+1)
        if (sh_erase_cnts[erase_i*3 +:3] != 0 &&
            (received_byte == sh_erase_ops[erase_i*16 +:8] ||
             (sh_erase_ops[erase_i*16+8 +:8] != 0 && received_byte == sh_erase_ops[erase_i*16+8 +:8]))) begin
            profile_erase_match = 1;
            profile_erase_native = received_byte == sh_erase_ops[erase_i*16+8 +:8];
            profile_erase_index = erase_i[1:0];
        end
end
reg [49:0] erase_table [0:31];
reg [49:0] erase_read_data;
reg [17:0] erase_probe_address;
reg [2:0] erase_fetch_count = 0;
reg erase_fetch_active = 0, erase_region_found = 0;
reg [22:0] erase_burst_mask;
wire [4:0] erase_read_address = {erase_descriptor, erase_fetch_count};
always @(posedge clk)
    if (cfg_erase_wren) erase_table[cfg_erase_waddr] <= cfg_erase_wdata;
always @(posedge spi_clk) begin
    erase_read_data <= erase_table[erase_read_address];
    if (reset_cs || reset_power) begin
        erase_fetch_active <= 0; erase_region_found <= 0; erase_fetch_count <= 0;
    end else if (erase_custom && state == STA_ADDR && addr_kind == ADDR_KIND_ERASE && addr_count == 8) begin
        erase_probe_address <= addr_next[17:0] & capacity_mask[25:8];
        erase_fetch_active <= 1; erase_region_found <= 0; erase_fetch_count <= 0;
    end else if (erase_fetch_active) begin
        erase_fetch_count <= erase_fetch_count + 1'b1;
        if (erase_fetch_count != 0 && !erase_region_found &&
            {1'b0,erase_probe_address} < erase_read_data[49:31]) begin
            erase_burst_mask <= erase_read_data[22:0]; erase_region_found <= 1;
        end
        if (erase_fetch_count == 5) erase_fetch_active <= 0;
    end
end
wire [22:0] selected_erase_mask = erase_custom ? erase_burst_mask : write_len;
wire [25:0] selected_erase_base = (addr_next[25:0] & capacity_mask) & ~{selected_erase_mask,3'b111};
