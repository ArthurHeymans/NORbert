// Logger Module
// Captures SPI transaction events, formats log packets, and streams
// them to the FT245 TX interface for real-time monitoring.
//
// Log packet format (type byte 0xA0-0xAF to avoid collision with
// normal protocol responses):
//
//   0xA1 <opcode>                                - SPI command decoded (2 bytes)
//   0xA2 <addr3> <addr2> <addr1> <addr0>         - Address phase complete (5 bytes)
//   0xA3 <count2> <count1> <count0>              - Transaction end + byte count (4 bytes)
//   0xA4 <index> <addr3> <addr2> <addr1> <addr0> - TOCTOU trap triggered (6 bytes)
//
// SPI-domain inputs are synchronized internally via 3-FF synchronizers
// with rising-edge detection.

`default_nettype none

module logger(
    input wire clk,
    input wire reset,
    input wire enable,               // Logging active (from glue LOGCTL)

    // SPI-domain logging inputs (from spi_trx)
    input wire spi_log_cmd_valid,    // Pulse: command decoded
    input wire [7:0] spi_log_cmd_opcode,
    input wire spi_log_addr_valid,   // Pulse: address phase complete
    input wire [31:0] spi_log_addr,
    input wire spi_active,           // Level: SPI CS active
    input wire [23:0] spi_log_byte_count,

    // TOCTOU trap notification (system clock domain, from glue)
    input wire trap_notify_strobe,
    input wire [1:0] trap_notify_index,
    input wire [23:0] trap_notify_addr,

    // FT245 TX interface
    input wire txd_ready,
    output reg txd_strobe = 0,
    output reg [7:0] txd_data = 0
);

    // -----------------------------------------------------------------
    // Synchronize SPI-domain signals into system clock domain
    // -----------------------------------------------------------------
    reg [2:0] cmd_valid_sync;
    reg [2:0] addr_valid_sync;
    reg [2:0] active_sync;

    always @(posedge clk) begin
        cmd_valid_sync  <= {cmd_valid_sync[1:0],  spi_log_cmd_valid};
        addr_valid_sync <= {addr_valid_sync[1:0], spi_log_addr_valid};
        active_sync     <= {active_sync[1:0],     spi_active};
    end

    wire cmd_event      = cmd_valid_sync[1]  && !cmd_valid_sync[2];
    wire addr_event     = addr_valid_sync[1] && !addr_valid_sync[2];
    wire deselect_event = !active_sync[1]    && active_sync[2];

    // -----------------------------------------------------------------
    // Byte FIFO for log packet data
    // -----------------------------------------------------------------
    wire fifo_space;
    wire fifo_data_available;
    wire [7:0] fifo_read_data;
    reg fifo_write_strobe;
    reg [7:0] fifo_write_data;
    reg fifo_read_strobe;

    fifo #(.WIDTH(8), .NUM(256), .FREESPACE(8)) log_fifo(
        .clk(clk),
        .reset(reset),
        .write_data(fifo_write_data),
        .write_strobe(fifo_write_strobe),
        .space_available(fifo_space),
        .data_available(fifo_data_available),
        .more_available(),
        .read_data(fifo_read_data),
        .read_strobe(fifo_read_strobe)
    );

    // -----------------------------------------------------------------
    // Event capture and packet emission state machine
    //
    // Events are detected as edges.  When an event fires, the packet
    // bytes are written into the FIFO one byte per clock.  If multiple
    // events arrive while a packet is being emitted, they are queued
    // via pending flags and processed in priority order.
    // -----------------------------------------------------------------

    localparam [2:0]
        PKT_IDLE   = 3'd0,
        PKT_EMIT   = 3'd1;

    reg [2:0] pkt_state;
    reg [47:0] pkt_shift;   // Shift register: up to 6 bytes
    reg [2:0] pkt_remain;   // Bytes remaining to emit

    // Pending event flags (set on edge detection, cleared when serviced)
    reg evt_cmd_pending;
    reg evt_addr_pending;
    reg evt_end_pending;
    reg evt_trap_pending;

    // Latched event data (captured when event edge detected)
    reg [7:0] evt_cmd_opcode;
    reg [31:0] evt_addr;
    reg [23:0] evt_byte_count;
    reg [1:0] evt_trap_index;
    reg [23:0] evt_trap_addr;

    always @(posedge clk) begin
        fifo_write_strobe <= 0;

        if (reset) begin
            pkt_state       <= PKT_IDLE;
            pkt_shift       <= 0;
            pkt_remain      <= 0;
            evt_cmd_pending  <= 0;
            evt_addr_pending <= 0;
            evt_end_pending  <= 0;
            evt_trap_pending <= 0;
        end
        else begin
            // Capture events
            if (cmd_event && enable) begin
                evt_cmd_pending <= 1;
                evt_cmd_opcode  <= spi_log_cmd_opcode;
            end
            if (addr_event && enable) begin
                evt_addr_pending <= 1;
                evt_addr         <= spi_log_addr;
            end
            if (deselect_event && enable) begin
                evt_end_pending  <= 1;
                evt_byte_count   <= spi_log_byte_count;
            end
            if (trap_notify_strobe && enable) begin
                evt_trap_pending <= 1;
                evt_trap_index   <= trap_notify_index;
                evt_trap_addr    <= trap_notify_addr;
            end

            // Packet emission
            if (pkt_state == PKT_IDLE) begin
                // Service pending events in priority order
                if (evt_cmd_pending && fifo_space) begin
                    evt_cmd_pending   <= 0;
                    fifo_write_strobe <= 1;
                    fifo_write_data   <= 8'hA1;
                    pkt_shift         <= {evt_cmd_opcode, 40'b0};
                    pkt_remain        <= 1;
                    pkt_state         <= PKT_EMIT;
                end
                else if (evt_addr_pending && fifo_space) begin
                    evt_addr_pending  <= 0;
                    fifo_write_strobe <= 1;
                    fifo_write_data   <= 8'hA2;
                    pkt_shift         <= {evt_addr[31:0], 16'b0};
                    pkt_remain        <= 4;
                    pkt_state         <= PKT_EMIT;
                end
                else if (evt_end_pending && fifo_space) begin
                    evt_end_pending   <= 0;
                    fifo_write_strobe <= 1;
                    fifo_write_data   <= 8'hA3;
                    pkt_shift         <= {evt_byte_count, 24'b0};
                    pkt_remain        <= 3;
                    pkt_state         <= PKT_EMIT;
                end
                else if (evt_trap_pending && fifo_space) begin
                    evt_trap_pending  <= 0;
                    fifo_write_strobe <= 1;
                    fifo_write_data   <= 8'hA4;
                    pkt_shift         <= {6'b0, evt_trap_index, evt_trap_addr, 16'b0};
                    pkt_remain        <= 5;
                    pkt_state         <= PKT_EMIT;
                end
            end
            else if (pkt_state == PKT_EMIT) begin
                if (fifo_space) begin
                    fifo_write_strobe <= 1;
                    fifo_write_data   <= pkt_shift[47:40];
                    pkt_shift         <= {pkt_shift[39:0], 8'b0};
                    pkt_remain        <= pkt_remain - 1;
                    if (pkt_remain == 1)
                        pkt_state <= PKT_IDLE;
                end
            end
        end
    end

    // -----------------------------------------------------------------
    // FIFO drain: send buffered log bytes to FT245 TX
    //
    // Single-byte cadence: assert txd_strobe for 1 cycle, then wait
    // for txd_ready before sending the next.  drain_hold prevents
    // double-send during the ready propagation delay.
    // -----------------------------------------------------------------
    reg drain_hold;

    always @(posedge clk) begin
        txd_strobe      <= 0;
        fifo_read_strobe <= 0;

        if (reset) begin
            drain_hold <= 0;
        end
        else if (enable && fifo_data_available && txd_ready && !drain_hold) begin
            txd_strobe       <= 1;
            txd_data         <= fifo_read_data;
            fifo_read_strobe <= 1;
            drain_hold       <= 1;
        end
        else begin
            if (drain_hold)
                drain_hold <= 0;
        end
    end

endmodule
