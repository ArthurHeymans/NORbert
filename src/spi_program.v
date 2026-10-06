// System-clock SPI program/erase engine. Owns the page-buffer BSRAM,
// consumed flags, NOR read-modify-write and write-completion toggle.
// The SDRAM request and payload stay stable until explicit acceptance;
// requests are issued only after the previous access completes.
`default_nettype none
module spi_program(
    input wire clk, reset, deselected,
    input wire command,
    input wire [1:0] command_type,
    input wire [22:0] command_addr, command_len,
    input wire byte_strobe,
    input wire [7:0] byte_offset, byte_value,
    input wire busy, accept,
    input wire [63:0] read_data,
    output reg writing, done, init_done,
    output wire init_finishing,
    output reg [1:0] access_cmd,
    output reg [24:0] access_addr,
    output wire inhibit,
    output reg [63:0] write_data
);
    localparam [3:0] WRITE_ACT=2, WRITE=3, NEXT=4, DONE=5,
                     READ_ACT=6, READ=7, MERGE=8;
    reg [3:0] state;
    reg [1:0] kind;
    reg [22:0] addr, remaining;
    reg [1:0] command_sync, byte_sync;
    reg command_ack, byte_ack;
    reg [7:0] byte_addr, byte_data;
    reg [63:0] merged;
    reg [8:0] mem [0:255];
    reg [7:0] waddr, raddr;
    reg [8:0] wdata, rdata;
    reg wren;
    reg prefetch, capture_valid, burst_ready;
    reg [2:0] capture_idx;
    reg [71:0] burst;
    reg [8:0] init_count;
    assign init_finishing = init_count == 9'd256;
    assign inhibit = writing && (state == WRITE_ACT || state == WRITE ||
                                  state == READ_ACT || state == READ);
    always @(posedge clk) rdata <= mem[raddr];
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            writing <= 0; done <= 0; init_done <= 0;
            access_cmd <= 0; access_addr <= 0; write_data <= 0;
            state <= READ_ACT; kind <= 0; addr <= 0; remaining <= 0;
            command_sync <= 0; byte_sync <= 0; command_ack <= 0; byte_ack <= 0;
            byte_addr <= 0; byte_data <= 0; merged <= 0;
            waddr <= 0; raddr <= 0; wdata <= 0; wren <= 0;
            prefetch <= 0; capture_valid <= 0; capture_idx <= 0;
            burst <= 0; burst_ready <= 0; init_count <= 0;
        end else begin
            wren <= 0;
            if (wren) mem[waddr] <= wdata;
            access_addr <= {addr, 2'b00};
            write_data <= merged;
            if (accept) access_cmd <= 0;
            if (!init_done) begin
                if (init_finishing) init_done <= 1;
                else begin
                    waddr <= init_count[7:0]; wdata <= 0; wren <= 1;
                    init_count <= init_count + 1'b1;
                end
            end
            byte_sync <= {byte_sync[0], byte_strobe};
            if (byte_sync[0] && !byte_sync[1]) begin
                byte_addr <= byte_offset; byte_data <= byte_value;
            end
            if (!byte_sync[1]) byte_ack <= 0;
            if (byte_sync[1] && !byte_ack && init_done) begin
                waddr <= byte_addr; wdata <= {1'b1,byte_data}; wren <= 1;
                byte_ack <= 1;
            end
            command_sync <= {command_sync[0], command};
            if (!command_sync[1]) command_ack <= 0;
            if (command_sync[1] && !command_ack && deselected && init_done && !writing) begin
                writing <= 1; command_ack <= 1;
                kind <= command_type; addr <= command_addr; remaining <= command_len;
                state <= command_type == 2'd1 ? WRITE_ACT : READ_ACT;
                if (command_type == 2'd1) merged <= 64'hffffffffffffffff;
            end
            // Registered read latency is hidden by SDRAM activation/read.
            // Clear consumed flags behind the advancing read address.
            if (prefetch) begin
                capture_valid <= 1;
                if (raddr[2:0] != 7) raddr <= raddr + 1'b1;
                if (capture_valid) begin
                    burst[capture_idx*9 +: 9] <= rdata;
                    waddr <= {addr[4:0],capture_idx}; wdata <= 0; wren <= 1;
                    if (capture_idx == 7) begin prefetch <= 0; burst_ready <= 1; end
                    else capture_idx <= capture_idx + 1'b1;
                end
            end
            if (writing && !busy) case (state)
                WRITE_ACT: begin access_cmd <= 2'b11; state <= WRITE; end
                WRITE: begin access_cmd <= 2'b10; state <= remaining == 0 ? DONE : NEXT; end
                NEXT: begin
                    state <= kind == 2'd1 ? WRITE_ACT : READ_ACT;
                    addr <= addr + 1'b1; remaining <= remaining - 1'b1;
                end
                DONE: begin writing <= 0; done <= ~done; end
                READ_ACT: begin
                    raddr <= {addr[4:0],3'b000}; prefetch <= 1;
                    capture_valid <= 0; capture_idx <= 0; burst_ready <= 0;
                    access_cmd <= 2'b11; state <= READ;
                end
                READ: begin access_cmd <= 2'b01; state <= MERGE; end
                MERGE: if (burst_ready) begin
                    for (i=0;i<8;i=i+1)
                        merged[i*8 +: 8] <= burst[i*9+8] ?
                            read_data[i*8 +: 8] & burst[i*9 +: 8] : read_data[i*8 +: 8];
                    state <= WRITE_ACT;
                end
                default: state <= DONE;
            endcase
        end
    end
endmodule
