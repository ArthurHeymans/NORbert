// Compile the 128-byte compatibility packet while SPI is stopped.
// Geometry arithmetic is pipelined; SPI sees only synchronous RAM rows
// {exclusive byte end, burst mask}, never shifts/counts/protection maps.
`default_nettype none
module chip_profile_store(
    input wire clk, reset, start, byte_valid,
    input wire [7:0] byte_data,
    input wire [26:0] capacity,
    output wire [459:0] profile,
    output reg done, bad,
    output reg erase_wren,
    output reg [4:0] erase_waddr,
    output reg [49:0] erase_wdata
);
    reg [255:0] header;
    reg [63:0] opcodes;
    reg [11:0] counts;
    // Keep the existing internal port width; unused sector-map bits are
    // constants, not registers or hardware state.
    assign profile = {128'b0, counts, opcodes, header};
    reg [6:0] offset;
    reg [4:0] position;
    reg [1:0] descriptor;
    reg [2:0] region_count;
    reg [23:0] word_low;
    reg [27:0] covered;
    reg [3:0] pending;
    reg finishing;
    reg [31:0] word;
    reg [27:0] word_start;
    reg [4:0] word_address;
    reg word_active, word_last_region, word_last_packet;
    reg [27:0] span, limit;
    reg [26:0] byte_mask, count_limit, capacity_snapshot;
    reg shift_bad, zero_bad, count_bad, alignment_bad;
    reg end_bad, coverage_bad, inactive_bad;
    integer i;
    always @(posedge clk) begin
        erase_wren <= 0;
        if (reset || start) begin
            header <= 0; opcodes <= 0; counts <= 0;
            done <= 0; bad <= 0; offset <= 0; position <= 0; descriptor <= 0;
            region_count <= 0; covered <= 0; word_low <= 0;
            pending <= 0; finishing <= 0;
            word <= 0; word_start <= 0; word_address <= 0;
            word_active <= 0; word_last_region <= 0; word_last_packet <= 0;
            span <= 0; limit <= 0; byte_mask <= 0; count_limit <= 0;
            capacity_snapshot <= capacity;
            shift_bad <= 0; zero_bad <= 0; count_bad <= 0; alignment_bad <= 0;
            end_bad <= 0; coverage_bad <= 0; inactive_bad <= 0;
            erase_waddr <= 0; erase_wdata <= 0;
        end else begin
            pending <= {pending[2:0],1'b0};
            if (finishing) begin done <= 1; finishing <= 0; end
            if (pending[0]) begin
                // Narrow shift controls; full-byte bounds are independently
                // rejected, so malformed shifts cannot alias valid records.
                span <= {4'b0,word[31:8]} << word[4:0];
                byte_mask <= (27'd1 << word[4:0]) - 27'd1;
                count_limit <= capacity_snapshot >> word[4:0];
                shift_bad <= word[7:0] < 8 || word[7:0] > 26;
            end
            if (pending[1]) begin
                limit <= word_start + span;
                zero_bad <= word[31:8] == 0;
                count_bad <= {3'b0,word[31:8]} > count_limit;
                alignment_bad <= (word_start[26:0] & byte_mask) != 0;
            end
            if (pending[2]) begin
                erase_wren <= 1; erase_waddr <= word_address;
                erase_wdata <= word_active ? {limit[26:0],byte_mask[25:3]} : 50'b0;
                end_bad <= limit > {1'b0,capacity_snapshot};
                coverage_bad <= word_last_region && limit != {1'b0,capacity_snapshot};
                inactive_bad <= word != 0;
                if (word_active) covered <= limit;
            end
            if (pending[3]) begin
                // Comparisons land in separate registers before reduction.
                // This edge also follows the committed erase-RAM write.
                if (word_active ? (shift_bad || zero_bad || count_bad ||
                    alignment_bad || end_bad || coverage_bad) : inactive_bad) bad <= 1;
                if (word_last_packet) finishing <= 1;
            end
            if (byte_valid) begin
                offset <= offset + 1'b1;
                if (offset < 32) begin
                    for (i=0; i<32; i=i+1)
                        if (offset == i[6:0]) header[i*8 +:8] <= byte_data;
                end else begin
                    if (position == 23) begin position <= 0; descriptor <= descriptor + 1'b1; end
                    else position <= position + 1'b1;
                    case (position)
                        0: for (i=0; i<4; i=i+1)
                            if (descriptor == i[1:0]) opcodes[i*16 +:8] <= byte_data;
                        1: for (i=0; i<4; i=i+1)
                            if (descriptor == i[1:0]) opcodes[i*16+8 +:8] <= byte_data;
                        2: begin
                            if (byte_data > 5) bad <= 1;
                            region_count <= byte_data[2:0];
                            for (i=0; i<4; i=i+1)
                                if (descriptor == i[1:0]) counts[i*3 +:3] <= byte_data[2:0];
                        end
                        3: if (byte_data != 0) bad <= 1;
                        default: case (position[1:0])
                            0: word_low[7:0] <= byte_data;
                            1: word_low[15:8] <= byte_data;
                            2: word_low[23:16] <= byte_data;
                            3: begin
                                word <= {byte_data,word_low};
                                // First row resets coverage even when the
                                // preceding descriptor's final row is active.
                                word_start <= position == 7 ? 28'd0 : covered;
                                word_address <= {descriptor,(position[4:2]-3'd1)};
                                word_active <= (position[4:2]-3'd1) < region_count;
                                word_last_region <= position[4:2] == region_count;
                                word_last_packet <= offset == 127;
                                pending[0] <= 1;
                            end
                        endcase
                    endcase
                end
            end
        end
    end
endmodule
`default_nettype wire
