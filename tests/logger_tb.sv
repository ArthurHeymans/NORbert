`timescale 1ns/1ps

// logger.v packet stream against the format the host decodes
// (tool/src/spi_log.rs): exact byte order, event priority when several
// events land together, the 3-FF synchronization delay, the enable
// gate, back-pressure when the host stops draining, and wrap of the
// 512-byte ring.
module logger_tb;
    reg clk = 0;
    always #4.167 clk = ~clk; // 120 MHz
    reg reset = 1;
    reg enable = 1;

    // SPI-domain inputs
    reg spi_log_cmd_valid = 0;
    reg [7:0] spi_log_cmd_opcode = 0;
    reg spi_log_addr_valid = 0;
    reg [31:0] spi_log_addr = 0;
    reg spi_active = 0;
    reg [23:0] spi_log_byte_count = 0;

    // Trap notification (system clock domain)
    reg trap_notify_strobe = 0;
    reg [1:0] trap_notify_index = 0;
    reg [23:0] trap_notify_addr = 0;

    wire out_data_available;
    wire [7:0] out_read_data;
    reg out_read_strobe = 0;

    logger dut (
        .clk(clk), .reset(reset), .enable(enable),
        .spi_log_cmd_valid(spi_log_cmd_valid),
        .spi_log_cmd_opcode(spi_log_cmd_opcode),
        .spi_log_addr_valid(spi_log_addr_valid),
        .spi_log_addr(spi_log_addr),
        .spi_active(spi_active),
        .spi_log_byte_count(spi_log_byte_count),
        .trap_notify_strobe(trap_notify_strobe),
        .trap_notify_index(trap_notify_index),
        .trap_notify_addr(trap_notify_addr),
        .out_data_available(out_data_available),
        .out_read_data(out_read_data),
        .out_read_strobe(out_read_strobe)
    );

    localparam LOG_PKT_CMD  = 8'hA1;
    localparam LOG_PKT_ADDR = 8'hA2;
    localparam LOG_PKT_END  = 8'hA3;
    localparam LOG_PKT_TRAP = 8'hA4;

    // -----------------------------------------------------------------
    // Capture: a byte queue fed by draining, compared against what the
    // test expected. Everything the DUT emits is checked; nothing is
    // pattern-matched away.
    // -----------------------------------------------------------------
    reg [7:0] got [0:8191];
    integer got_len = 0;
    reg [7:0] want [0:8191];
    integer want_len = 0;
    integer pos, decoded, last_index, last_end;
    // Usable logger ring depth: logger.v's 512-entry FIFO keeps FREESPACE=8
    // entries of headroom and one more slot, see fifo.v.
    localparam integer RING_BYTES = 512 - 1 - 8;
    reg [31:0] addr_value;
    reg [23:0] count_value;
    reg [7:0] opcode_seen;

    // Byte `shift` of a computed value, sized exactly.
    function automatic [7:0] byte_of(input integer v, input integer shift);
        byte_of = 8'((v >> shift) & 32'hff);
    endfunction

    task expect_byte(input [7:0] b);
        want[want_len] = b;
        want_len = want_len + 1;
    endtask

    // Drain the logger FIFO into got[].
    //
    // The FIFO is first-word-fall-through and advances on every edge that
    // carries read_strobe, so an entry is sampled while the strobe is still
    // low and consumed by a one-cycle pulse. Checking data_available with
    // the strobe low is what keeps this safe: the FIFO computes its count
    // as count + write - read, so a read while empty wraps the count and
    // makes data_available lie for the rest of the run.
    //
    // The strobe is lowered a whole negedge after the consuming posedge, not
    // immediately at it: this test's initial block resumes before the DUT's
    // always-block at a clock edge, so clearing it at the posedge would drop
    // the pulse before the DUT sampled it.
    //
    // `idle_limit` bounds how long to wait for the next byte, so a drain of
    // a partly-full ring terminates instead of blocking.
    task automatic drain(input integer max_bytes, input integer idle_limit = 4000);
        integer n = 0;
        integer idle = 0;
        out_read_strobe = 0;
        while (n < max_bytes && idle < idle_limit) begin
            @(negedge clk);
            if (out_data_available) begin
                got[got_len] = out_read_data;
                got_len = got_len + 1;
                n = n + 1;
                idle = 0;
                out_read_strobe = 1;
                @(posedge clk);
                @(negedge clk);
                out_read_strobe = 0;
            end
            else begin
                idle = idle + 1;
            end
        end
    endtask

    // Drain the ring empty, discarding the bytes. Same safety rule as drain:
    // the strobe is only ever pulsed with data genuinely present.
    task automatic flush(input integer idle_limit = 200);
        integer idle = 0;
        out_read_strobe = 0;
        while (idle < idle_limit) begin
            @(negedge clk);
            if (out_data_available) begin
                idle = 0;
                out_read_strobe = 1;
                @(posedge clk);
                @(negedge clk);
                out_read_strobe = 0;
            end
            else begin
                idle = idle + 1;
            end
        end
    endtask

    // Bring the logger to a known-empty state: let any queued events finish
    // emitting, drain the ring, let the emitter settle again, and require
    // that no further bytes appear. Without the two settle passes the
    // emitter keeps writing after the flush and the next phase starts
    // mid-stream.
    task automatic quiesce;
        settle(400);
        flush();
        settle(400);
        flush();
        if (out_data_available !== 1'b0)
            $fatal(1, "logger is still producing after quiesce");
    endtask

    // Let the emitter run without draining, so the ring fills up.
    task automatic settle(input integer cycles);
        out_read_strobe = 0;
        repeat (cycles) @(posedge clk);
    endtask

    task automatic compare_stream(input [511:0] where);
        if (got_len != want_len)
            $fatal(1, "%0s: got %0d bytes, expected %0d", where, got_len, want_len);
        for (integer i = 0; i < want_len; i++)
            if (got[i] !== want[i])
                $fatal(1, "%0s: byte %0d is %h, expected %h", where, i, got[i],
                       want[i]);
    endtask

    // SPI-domain events are level-plus-pulse inputs seen through three
    // synchronizer stages; the payload must be held while the pulse is
    // high for the synchronizer to see a consistent value.
    task spi_cmd(input [7:0] opcode);
        spi_log_cmd_opcode = opcode;
        spi_log_cmd_valid = 1;
        repeat (3) @(posedge clk);
        spi_log_cmd_valid = 0;
        repeat (3) @(posedge clk);
    endtask

    task spi_addr(input [31:0] addr);
        spi_log_addr = addr;
        spi_log_addr_valid = 1;
        repeat (3) @(posedge clk);
        spi_log_addr_valid = 0;
        repeat (3) @(posedge clk);
    endtask

    // A transaction ends when CS deasserts; the byte count is latched with
    // the deselect, so hold it steady across the edge.
    task spi_deselect(input [23:0] count);
        spi_log_byte_count = count;
        spi_active = 1;
        repeat (3) @(posedge clk);
        spi_active = 0;
        repeat (3) @(posedge clk);
    endtask

    task trap(input [1:0] index, input [23:0] addr);
        trap_notify_index = index;
        trap_notify_addr = addr;
        trap_notify_strobe = 1;
        @(posedge clk);
        #1;
        trap_notify_strobe = 0;
        repeat (2) @(posedge clk);
    endtask

    integer total_packets;

    initial begin
        repeat (4) @(negedge clk);
        reset = 0;
        repeat (4) @(negedge clk);

        // ---- one complete transaction, in order -------------------
        spi_cmd(8'hBB);
        spi_addr(32'h0010_0000);
        spi_deselect(24'h04);
        expect_byte(LOG_PKT_CMD);  expect_byte(8'hBB);
        expect_byte(LOG_PKT_ADDR); expect_byte(8'h00); expect_byte(8'h10);
        expect_byte(8'h00);          expect_byte(8'h00);
        expect_byte(LOG_PKT_END);  expect_byte(8'h00); expect_byte(8'h00); expect_byte(8'h04);
        drain(11);
        compare_stream("one transaction");
        total_packets = 1;

        // ---- trap notification: index + 4 address bytes + pad ------
        trap(2, 24'h12_3456);
        expect_byte(LOG_PKT_TRAP); expect_byte(8'h02);
        expect_byte(8'h12); expect_byte(8'h34); expect_byte(8'h56); expect_byte(8'h00);
        drain(6);
        compare_stream("trap");
        total_packets = total_packets + 1;

        // ---- enable gate: nothing may be captured while disabled --
        enable = 0;
        spi_cmd(8'h03);
        spi_addr(32'h0000_1000);
        trap(1, 24'h00_2000);
        spi_deselect(24'h08);
        drain(1);
        if (got_len != want_len)
            $fatal(1, "disabled logger emitted %0d extra bytes",
                   got_len - want_len);
        enable = 1;

        // ---- priority when several events are pending together ----
        // The three SPI events are aligned so their synchronized edges land
        // in the same cycle, which puts all three flags pending at once.
        // The emitter must then service them CMD, ADDR, END in that order.
        //
        // The trap notification is a system-clock strobe with no
        // synchronizer, so it is captured immediately. Aligning it with the
        // SPI events would therefore place it *first*, not last: it is
        // fired separately here, after the three have been captured.
        out_read_strobe = 0;
        spi_active = 1;
        repeat (3) @(posedge clk);       // let active_sync see CS asserted
        spi_log_cmd_opcode = 8'hEB;     spi_log_cmd_valid = 1;
        spi_log_addr = 32'h0000_ABCD;    spi_log_addr_valid = 1;
        spi_log_byte_count = 24'h00_1000;
        spi_active = 0;                 // deassert in the same window
        repeat (3) @(posedge clk);
        spi_log_cmd_valid = 0;
        spi_log_addr_valid = 0;
        repeat (3) @(posedge clk);
        expect_byte(LOG_PKT_CMD);  expect_byte(8'hEB);
        expect_byte(LOG_PKT_ADDR); expect_byte(8'h00); expect_byte(8'h00);
        expect_byte(8'hAB);          expect_byte(8'hCD);
        expect_byte(LOG_PKT_END);  expect_byte(8'h00); expect_byte(8'h10); expect_byte(8'h00);
        drain(11);
        compare_stream("event priority");

        // A trap arriving afterwards is appended, not reordered.
        trap(3, 24'h00_00FF);
        expect_byte(LOG_PKT_TRAP); expect_byte(8'h03);
        expect_byte(8'h00); expect_byte(8'h00); expect_byte(8'hFF); expect_byte(8'h00);
        drain(6);
        compare_stream("trap after transaction");
        total_packets = total_packets + 4;

        // ---- back-pressure: stop draining, nothing torn or reordered ----
        // Stop draining entirely and push far more traffic than the ring
        // holds. The ring is a byte FIFO with one pending slot per event
        // type, so under sustained back-pressure it is *lossy*: an event
        // that arrives while one of its type is still pending is dropped,
        // and a transaction can keep some of its packets but lose others.
        // What must hold is that every packet that does come out is whole,
        // belongs to a transaction that was sent, and is in order.
        out_read_strobe = 0;
        for (integer i = 0; i < 200; i++) begin
            spi_cmd(8'h03);
            spi_addr(32'h0000_0000 + 32'(i) * 32'h1000);
            spi_deselect(24'(i) & 24'h00FFFF);
        end
        // Give the emitter time to stall against the full ring.
        repeat (200) @(posedge clk);
        if (out_data_available !== 1'b1)
            $fatal(1, "ring not reporting data after back-pressure");
        // Drain until the logger goes idle: the ring's contents first, then
        // the packets that were pending behind it.
        got_len = 0;
        drain(8192, 200);
        if (got_len == 0)
            $fatal(1, "back-pressure destroyed the whole ring");
        // At most the ring's usable depth plus one pending packet per type.
        if (got_len > RING_BYTES + 2 + 5 + 4 + 6)
            $fatal(1, "ring produced %0d bytes, more than it can hold", got_len);
        pos = 0; decoded = 0; last_index = -1; last_end = -1;
        while (pos < got_len) begin
            case (got[pos])
                LOG_PKT_CMD: begin
                    if (got_len - pos < 2)
                        $fatal(1, "command packet %0d at byte %0d is cut short",
                               decoded, pos);
                    opcode_seen = got[pos + 1];
                    if (opcode_seen !== 8'h03)
                        $fatal(1, "packet %0d has opcode %h, expected 03",
                               decoded, opcode_seen);
                    pos = pos + 2;
                end
                LOG_PKT_ADDR: begin
                    if (got_len - pos < 5)
                        $fatal(1, "address packet %0d at byte %0d is cut short",
                               decoded, pos);
                    addr_value = {got[pos + 1], got[pos + 2], got[pos + 3],
                                  got[pos + 4]};
                    // Must be i * 0x1000 for one of the transactions sent.
                    // Dropped packets make neighbours adjacent, so the
                    // guarantee is that survivors are not reordered, not
                    // that they alternate CMD/ADDR/END.
                    if ((addr_value & 32'hfff) != 0 ||
                        (addr_value >> 12) >= 200)
                        $fatal(1, "packet %0d has address %h, not a transaction address",
                               decoded, addr_value);
                    if ($signed(addr_value >> 12) < last_index)
                        $fatal(1, "packet %0d address %h precedes the previous one (%0d): reordered",
                               decoded, addr_value, last_index);
                    last_index = addr_value >> 12;
                    pos = pos + 5;
                end
                LOG_PKT_END: begin
                    if (got_len - pos < 4)
                        $fatal(1, "end packet %0d at byte %0d is cut short",
                               decoded, pos);
                    // The byte count is the transaction index.
                    count_value = {got[pos + 1], got[pos + 2], got[pos + 3]};
                    if (count_value >= 200)
                        $fatal(1, "packet %0d has byte count %h, not a transaction index",
                               decoded, count_value);
                    if ($signed({8'h00, count_value}) < last_end)
                        $fatal(1, "packet %0d count %0d precedes the previous one (%0d): reordered",
                               decoded, count_value, last_end);
                    last_end = {8'h00, count_value};
                    pos = pos + 4;
                end
                default:
                    $fatal(1, "packet %0d at byte %0d has type %h, not a packet header",
                           decoded, pos, got[pos]);
            endcase
            decoded = decoded + 1;
        end
        $display("LOGGER: 200 transactions pushed with no draining; %0d packets survived (%0d bytes), none torn, corrupted or reordered",
                 decoded, got_len);

        // ---- a full ring followed by a drain, for the wrap path ----
        // Push more traffic than the 512-byte ring holds while draining at
        // the emitter's own rate, so every packet survives and the ring's
        // read and write pointers both wrap.
        quiesce();
        got_len = 0;
        want_len = 0;
        fork
            begin : wrap_pusher
                for (integer i = 0; i < 300; i++) begin
                    spi_cmd(8'h6B);
                    spi_addr(32'h0040_0000 + 32'(i) * 32'h0800);
                    spi_deselect(24'(i) & 24'h0FFF);
                    // Leave the drainer room: it moves one byte every two
                    // clocks, this transaction emits 11. Overflow is what the
                    // back-pressure phase above is for; this phase is about
                    // the ring's pointers wrapping with nothing lost.
                    repeat (16) @(posedge clk);
                    expect_byte(LOG_PKT_CMD);  expect_byte(8'h6B);
                    // Address is 0x0040_0000 + i * 0x800, big-endian.
                    expect_byte(LOG_PKT_ADDR);
                    expect_byte(byte_of(32'h0040_0000 + 32'(i) * 32'h0800, 24));
                    expect_byte(byte_of(32'h0040_0000 + 32'(i) * 32'h0800, 16));
                    expect_byte(byte_of(32'h0040_0000 + 32'(i) * 32'h0800, 8));
                    expect_byte(byte_of(32'h0040_0000 + 32'(i) * 32'h0800, 0));
                    expect_byte(LOG_PKT_END);  expect_byte(8'h00);
                    expect_byte(byte_of(i, 8)); expect_byte(byte_of(i, 0));
                end
            end
            begin : wrap_drainer
                drain(300 * 11, 20000);
            end
        join
        compare_stream("wrap: 300 transactions through the ring");
        total_packets = total_packets + 300;

        // A quiet logger must keep reporting an empty ring. Do not pulse
        // read_strobe here: reading an empty FIFO is outside its contract.
        settle(64);
        repeat (16) begin
            @(negedge clk);
            if (out_data_available !== 1'b0)
                $fatal(1, "empty logger reports data");
        end

        $display("PASS LOGGER: %0d packets, byte-exact stream, event priority, enable gate, back-pressure, ring wrap",
                 total_packets);
        $finish;
    end

    initial begin
        #4000000;
        $fatal(1, "LOGGER test timeout");
    end
endmodule
