// SPI event logger. Queue capture frames before byte serialization so
// backpressure cannot reorder packet types or overwrite older payloads.
// Events captured on the same system edge use CMD, ADDR, TRAP, END order.
// Overflow drops entire new frames; a saturating count is inserted before
// the first later frame accepted, preserving the position of stream gaps.
`default_nettype none

module logger(
    input wire clk, reset, enable,
    input wire spi_log_cmd_valid,
    input wire [7:0] spi_log_cmd_opcode,
    input wire spi_log_addr_valid,
    input wire [31:0] spi_log_addr,
    input wire spi_active,
    input wire [23:0] spi_log_byte_count,
    input wire trap_notify_strobe,
    input wire [1:0] trap_notify_index,
    input wire [23:0] trap_notify_addr,
    output wire out_data_available,
    output wire [7:0] out_read_data,
    input wire out_read_strobe
);
    `include "host_protocol.vh"
    reg [2:0] cmd_valid_sync, addr_valid_sync, active_sync;
    always @(posedge clk) begin
        cmd_valid_sync <= {cmd_valid_sync[1:0], spi_log_cmd_valid};
        addr_valid_sync <= {addr_valid_sync[1:0], spi_log_addr_valid};
        active_sync <= {active_sync[1:0], spi_active};
    end
    wire [3:0] events = {trap_notify_strobe,
                         !active_sync[1] && active_sync[2],
                         addr_valid_sync[1] && !addr_valid_sync[2],
                         cmd_valid_sync[1] && !cmd_valid_sync[2]} & {4{enable}};
    wire [2:0] event_count = {2'b0, events[0]} + {2'b0, events[1]} +
                             {2'b0, events[2]} + {2'b0, events[3]};
    reg [15:0] lost_pending;
    wire [16:0] lost_sum = {1'b0, lost_pending} + {14'b0, event_count};
    wire frame_space, frame_available;
    wire [110:0] frame_data;
    wire frame_write = frame_space && (events != 0 || lost_pending != 0);
    reg frame_pop;
    fifo #(.WIDTH(111), .NUM(8), .FREESPACE(1)) event_fifo(
        .clk(clk), .reset(reset), .space_available(frame_space),
        .write_data({lost_pending != 0, events, lost_pending,
                     spi_log_cmd_opcode, spi_log_addr, spi_log_byte_count,
                     trap_notify_index, trap_notify_addr}),
        .write_strobe(frame_write), .data_available(frame_available),
        .more_available(), .read_data(frame_data), .read_strobe(frame_pop));
    always @(posedge clk) begin
        if (reset || frame_write) lost_pending <= 0;
        else if (events != 0) lost_pending <= lost_sum[16] ? 16'hffff : lost_sum[15:0];
    end

    wire byte_space;
    reg byte_write;
    reg [7:0] byte_data;
    fifo #(.WIDTH(8), .NUM(512), .FREESPACE(8)) byte_fifo(
        .clk(clk), .reset(reset), .space_available(byte_space),
        .write_data(byte_data), .write_strobe(byte_write),
        .data_available(out_data_available), .more_available(),
        .read_data(out_read_data), .read_strobe(out_read_strobe));

    localparam [1:0] IDLE=0, SELECT=1, EMIT=2;
    reg [1:0] state;
    reg [110:0] frame;
    reg [4:0] pending;
    reg [39:0] payload;
    reg [2:0] remain;
    always @(posedge clk) begin
        byte_write <= 0;
        frame_pop <= 0;
        if (reset) begin
            state <= IDLE;
            frame <= 0;
            pending <= 0;
            payload <= 0;
            remain <= 0;
            byte_data <= 0;
        end else case(state)
            IDLE: if (frame_available) begin
                frame <= frame_data;
                pending <= frame_data[110:106];
                frame_pop <= 1;
                state <= SELECT;
            end
            SELECT: begin
                if (pending == 0) state <= IDLE;
                else if (byte_space) begin
                    byte_write <= 1;
                    state <= EMIT;
                    if (pending[4]) begin
                        pending[4] <= 0;
                        byte_data <= LOG_PKT_LOST;
                        payload <= {frame[105:90], 24'b0};
                        remain <= 2;
                    end else if (pending[0]) begin
                        pending[0] <= 0;
                        byte_data <= LOG_PKT_CMD;
                        payload <= {frame[89:82], 32'b0};
                        remain <= 1;
                    end else if (pending[1]) begin
                        pending[1] <= 0;
                        byte_data <= LOG_PKT_ADDR;
                        payload <= {frame[81:50], 8'b0};
                        remain <= 4;
                    end else if (pending[3]) begin
                        pending[3] <= 0;
                        byte_data <= LOG_PKT_TRAP;
                        payload <= {6'b0, frame[25:24], frame[23:0], 8'b0};
                        remain <= 5;
                    end else begin
                        pending[2] <= 0;
                        byte_data <= LOG_PKT_END;
                        payload <= {frame[49:26], 16'b0};
                        remain <= 3;
                    end
                end
            end
            EMIT: if (byte_space) begin
                byte_write <= 1;
                byte_data <= payload[39:32];
                payload <= {payload[31:0], 8'b0};
                remain <= remain - 1'b1;
                if (remain == 1) state <= SELECT;
            end
            default: state <= IDLE;
        endcase
    end
endmodule
