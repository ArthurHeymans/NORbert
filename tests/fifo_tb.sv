`timescale 1ns/1ps

// fifo.v against a queue model: first-word-fall-through data, the
// more_available/space_available contract, pointer wrap, and the
// simultaneous write+read case that only exercises the FWFT bypass.
//
// Reads are only issued when data_available is high, which is the
// contract both consumers in the design rely on. The empty-read
// behaviour is not "undefined" in this implementation (it wraps count
// and reports data_available), so the test records it once instead of
// asserting on it.
module fifo_tb;
    localparam WIDTH = 8;
    localparam NUM = 16;      // As used for the FT245 RX FIFO in top.v
    localparam FREESPACE = 1;

    reg clk = 0;
    always #4.167 clk = ~clk;  // 120 MHz
    reg reset = 1;

    reg [WIDTH-1:0] write_data = 0;
    reg write_strobe = 0;
    wire space_available;
    wire data_available, more_available;
    wire [WIDTH-1:0] read_data;
    reg read_strobe = 0;

    fifo #(.WIDTH(WIDTH), .NUM(NUM), .FREESPACE(FREESPACE)) dut (
        .clk(clk), .reset(reset),
        .space_available(space_available),
        .write_data(write_data), .write_strobe(write_strobe),
        .data_available(data_available), .more_available(more_available),
        .read_data(read_data), .read_strobe(read_strobe)
    );

    // Queue model: same contents, written and read in the same order.
    reg [WIDTH-1:0] model [0:NUM-1];
    integer head = 0, tail = 0, depth = 0;
    integer writes = 0, reads = 0;
    integer max_depth = 0;
    reg [WIDTH-1:0] next_byte = 0;

    // The depth at which space_available drops. Derived from the RTL's
    // expression, (NUM - 1 - count) > FREESPACE, and asserted below so a
    // change to that expression is a test failure rather than a silent
    // change in how deep this FIFO can be used.
    localparam integer EXPECTED_DEPTH = NUM - 1 - FREESPACE;

    task model_push(input [WIDTH-1:0] d);
        model[tail] = d;
        tail = (tail + 1) % NUM;
        depth = depth + 1;
        if (depth > max_depth) max_depth = depth;
    endtask

    task model_pop;
        head = (head + 1) % NUM;
        depth = depth - 1;
    endtask

    // The model and the DUT must agree on what is available before every
    // edge, and on the FWFT byte after every edge.
    task check_state(input [255:0] where);
        if (data_available !== (depth != 0))
            $fatal(1, "%0s: data_available=%b but model depth=%0d", where,
                   data_available, depth);
        if (more_available !== (depth > 1))
            $fatal(1, "%0s: more_available=%b but model depth=%0d", where,
                   more_available, depth);
        if (depth != 0 && read_data !== model[head])
            $fatal(1, "%0s: read_data=%h but model head=%h (depth %0d)", where,
                   read_data, model[head], depth);
    endtask

    // A randomised stream of writes and reads, only ever issuing a
    // transfer the DUT advertises room/data for. Writes and reads are
    // biased to collide so the FWFT bypass gets hit often.
    task random_stream(input integer steps, input integer seed);
        integer rnd;
        bit do_write, do_read;
        rnd = seed;
        for (integer step = 0; step < steps; step++) begin
            rnd = (rnd * 1103515245 + 12345) & 32'h7fffffff;
            do_write = (rnd % 100) < 60;
            do_read = (rnd % 100) < 70;
            // Both are 1-bit, so the strobe expressions stay exact.
            if (!do_write && !do_read) do_write = 1;

            // Strobe combinationally, then let one edge happen.
            @(negedge clk);
            check_state("random pre-edge");
            write_strobe = do_write && space_available;
            read_strobe = do_read && data_available;
            if (write_strobe) begin
                write_data = next_byte;
                model_push(write_data);
                next_byte = next_byte + 8'h1f;
                writes = writes + 1;
            end
            if (read_strobe) begin
                model_pop;
                reads = reads + 1;
            end
            @(posedge clk);
            #1;
            check_state("random post-edge");
            @(negedge clk);
            write_strobe = 0;
            read_strobe = 0;
        end
    endtask

    integer empty_read_count;
    reg empty_read_available;

    initial begin
        repeat (4) @(negedge clk);
        reset = 0;
        repeat (2) @(negedge clk);

        // Empty after reset.
        check_state("after reset");
        if (data_available) $fatal(1, "FIFO not empty after reset");

        // Fill to the advertised depth and confirm it stops there.
        while (space_available) begin
            @(negedge clk);
            write_strobe = 1;
            write_data = next_byte;
            model_push(next_byte);
            next_byte = next_byte + 8'h1f;
            @(posedge clk);
            #1;
            check_state("fill");
        end
        @(negedge clk);
        write_strobe = 0;
        if (max_depth != EXPECTED_DEPTH)
            $fatal(1, "space_available dropped at depth %0d, expected %0d",
                   max_depth, EXPECTED_DEPTH);
        if (depth != EXPECTED_DEPTH)
            $fatal(1, "model depth %0d != %0d after fill", depth, EXPECTED_DEPTH);

        // Drain it; the FWFT byte must follow the head pointer.
        while (depth != 0) begin
            @(negedge clk);
            read_strobe = 1;
            model_pop;
            @(posedge clk);
            #1;
            check_state("drain");
            @(negedge clk);
            read_strobe = 0;
        end
        check_state("drained");

        // Wrap both pointers: far more traffic than NUM entries, in a
        // pattern that interleaves writes and reads every cycle.
        next_byte = 8'h00;
        random_stream(600, 12345);

        // Drain back to empty: the directed cases below reason about an
        // empty FIFO, and the random stream deliberately leaves it full.
        while (depth != 0) begin
            @(negedge clk);
            read_strobe = 1;
            model_pop;
            @(posedge clk);
            #1;
            check_state("drain after random");
            @(negedge clk);
            read_strobe = 0;
        end
        if (data_available) $fatal(1, "FIFO not empty after final drain");

        // Simultaneous write and read on a single entry: the FWFT bypass
        // must present the new byte, not the one just consumed.
        begin
            reg [WIDTH-1:0] first, second;
            first = 8'ha5; second = 8'h5a;
            @(negedge clk);
            write_strobe = 1; write_data = first; model_push(first);
            @(posedge clk); #1;
            @(negedge clk);
            write_strobe = 0;
            if (read_data !== first) $fatal(1, "FWFT: got %h expected %h", read_data, first);
            // Same cycle: consume the only entry and push a new one.
            write_strobe = 1; write_data = second; read_strobe = 1;
            model_pop; model_push(second);
            @(posedge clk); #1;
            if (data_available !== 1'b1)
                $fatal(1, "FWFT bypass lost the entry");
            if (read_data !== second)
                $fatal(1, "FWFT bypass: got %h expected %h", read_data, second);
            @(negedge clk);
            write_strobe = 0; read_strobe = 0;
            // Consume the entry the bypass produced.
            read_strobe = 1;
            model_pop;
            @(posedge clk); #1;
            if (data_available !== 1'b0)
                $fatal(1, "FIFO not empty after consuming the bypassed entry");
            @(negedge clk);
            read_strobe = 0;
        end
        check_state("after bypass");

        if (writes == 0 || reads == 0)
            $fatal(1, "random stream did no work: %0d writes, %0d reads",
                   writes, reads);

        // Record, do not assert: reading an empty FIFO is outside the
        // contract, and both consumers gate on data_available.
        @(negedge clk);
        read_strobe = 1;
        @(posedge clk); #1;
        empty_read_count = 32'(dut.count);
        empty_read_available = data_available;
        @(negedge clk);
        read_strobe = 0;

        $display("PASS FIFO: FWFT, more_available, space depth %0d, %0d writes, %0d reads",
                 max_depth, writes, reads);
        $display("      empty read (outside contract): count field became %0d, data_available=%b",
                 empty_read_count, empty_read_available);
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "FIFO test timeout");
    end
endmodule
