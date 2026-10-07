`timescale 1ns/1ps

// Log-only (sniff) mode at the top level.
//
// - Mutual exclusion with #HOLD: each refuses while the other is active,
//   and the mode can only change while emulation is stopped.
// - While a master talks to the (unmodelled) real flash, NORbert never
//   drives an SPI pad, never issues an SDRAM ACTIVATE/READ/WRITE and never
//   starts the program engine, whatever the commands.
// - The log still decodes every transaction, including commands our own
//   emulation would have refused: programs/erases without WREN, reads
//   right after an erase (no WIP), quad reads and 4-byte forms on a chip
//   configured without 4-byte support.
module log_only_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;  // The PLL stub passes its input through
    reg sck = 0, cs = 1, mosi = 0;
    reg host_strobe = 0;
    reg [7:0] host_data = 0;
    reg quad_drive = 0;
    reg [3:0] quad_value = 0;
    wire io0, io1, io2, io3;
    assign io0 = quad_drive ? quad_value[0] : mosi;
    assign io1 = quad_drive ? quad_value[1] : 1'bz;
    assign io2 = quad_drive ? quad_value[2] : 1'bz;
    assign io3 = quad_drive ? quad_value[3] : 1'bz;
    pullup (io1);
    pullup (io2);
    pullup (io3);

    top dut (
        .clk_50mhz(clk), .uart_rx(1'b1), .spi_cs_pin(cs), .spi_clk_pin(sck),
        .spi_mosi_pin(io0), .spi_miso_pin(io1), .spi_io2_pin(io2), .spi_io3_pin(io3),
        .spi_power_pin(1'b1), .ft_rxf_n(1'b1), .ft_txe_n(1'b1)
    );

    // Host replies, in order.
    reg [7:0] replies [$];
    always @(posedge clk) if (dut.uart_txd_strobe) replies.push_back(dut.uart_txd);

    // Nothing may drive the bus or touch SDRAM for the SPI side while
    // observing (refresh is the only command allowed).
    reg observing = 0;
    always @(negedge clk) if (observing) begin
        if (dut.spi_io0_oe || dut.spi_io1_oe || dut.spi_io2_oe || dut.spi_io3_oe)
            $fatal(1, "log-only mode enabled an SPI output");
        if (dut.hold_drive) $fatal(1, "log-only mode drove #HOLD");
        if (!dut.O_sdram_ras_n && dut.O_sdram_cas_n && dut.O_sdram_wen_n)
            $fatal(1, "log-only mode activated an SDRAM row");
        if (dut.O_sdram_ras_n && !dut.O_sdram_cas_n)
            $fatal(1, "log-only mode issued an SDRAM read/write");
        if (dut.glue_i.spi_writing) $fatal(1, "log-only mode started the program engine");
    end

    task tick(input integer n); repeat (n) @(negedge clk); endtask

    task host_byte(input [7:0] b);
        @(negedge clk); host_data = b; host_strobe = 1;
        @(negedge clk); host_strobe = 0;
        tick(10);
    endtask

    task automatic expect_reply(input [7:0] want, input string what);
        integer waited = 0;
        while (replies.size() == 0 && waited < 2000) begin tick(1); waited++; end
        if (replies.size() == 0) $fatal(1, "%s: no reply", what);
        if (replies[0] !== want) $fatal(1, "%s: reply %h, expected %h", what, replies[0], want);
        void'(replies.pop_front());
    endtask

    task command2(input [7:0] opcode, input [7:0] arg, input [7:0] want, input string what);
        host_byte(opcode); host_byte(arg); expect_reply(want, what);
    endtask

    /* verilator lint_off ZERODLY */
    task spi_byte(input [7:0] b);
        for (integer i = 7; i >= 0; i--) begin
            mosi = b[i]; #16.667; sck = 1; #16.667; sck = 0;
        end
    endtask
    /* verilator lint_on ZERODLY */

    task spi_quad(input [3:0] n);
        quad_drive = 1; quad_value = n;
        #16.667; sck = 1; #16.667; sck = 0;
        quad_drive = 0;
    endtask

    task select_spi; cs = 0; #35; endtask
    task deselect_spi; #35; cs = 1; #200; endtask

    // Expected log: packet type, value, in order.
    typedef struct { byte kind; int value; } packet_t;
    packet_t expected [$];
    task want(input byte kind, input int value);
        packet_t p; p.kind = kind; p.value = value; expected.push_back(p);
    endtask

    initial begin
        reg [7:0] raw [$];
        reg [7:0] stream [$];
        force dut.uart_rxd_strobe = host_strobe;
        force dut.uart_rxd = host_data;
        wait (!dut.reset);
        tick(300);

        // Exclusivity and mode-change rules.
        command2(8'h37, 8'h01, 8'h01, "HOLDCTL on");
        command2(8'h3c, 8'h01, 8'h02, "SNIFFCTL on with hold asserted");
        if (dut.log_only) $fatal(1, "log-only entered with hold asserted");
        command2(8'h37, 8'h00, 8'h01, "HOLDCTL off");
        command2(8'h3c, 8'h01, 8'h01, "SNIFFCTL on");
        command2(8'h37, 8'h01, 8'h02, "HOLDCTL on in log-only mode");
        if (dut.hold_out) $fatal(1, "hold asserted in log-only mode");
        command2(8'h38, 8'h01, 8'h01, "LOGCTL on");
        host_byte(8'h34); expect_reply(8'h01, "START");
        wait (!dut.spi_reset_effective);
        host_byte(8'h36); expect_reply(8'h03, "STATUS in log-only mode");
        command2(8'h3c, 8'h00, 8'h02, "SNIFFCTL off while running");
        if (!dut.log_only) $fatal(1, "log-only mode changed while running");

        observing = 1;

        // JEDEC ID and status read: the real flash answers.
        select_spi; spi_byte(8'h9f); repeat (3) spi_byte(0); deselect_spi;
        want(8'ha1, 'h9f); want(8'ha3, 0);
        select_spi; spi_byte(8'h05); spi_byte(0); deselect_spi;
        want(8'ha1, 'h05); want(8'ha3, 0);

        // A 16-byte read.
        select_spi; spi_byte(8'h03); spi_byte(8'h12); spi_byte(8'h34); spi_byte(8'h56);
        repeat (16) spi_byte(0); deselect_spi;
        want(8'ha1, 'h03); want(8'ha2, 32'h00123456); want(8'ha3, 16);

        // Page program and sector erase without WREN: still logged, and
        // neither reaches SDRAM nor sets WIP in our decoder.
        select_spi; spi_byte(8'h02); spi_byte(8'h00); spi_byte(8'h10); spi_byte(8'h00);
        repeat (4) spi_byte(8'ha5); deselect_spi;
        want(8'ha1, 'h02); want(8'ha2, 32'h00001000); want(8'ha3, 0);
        select_spi; spi_byte(8'h20); spi_byte(8'h00); spi_byte(8'h20); spi_byte(8'h00);
        deselect_spi;
        want(8'ha1, 'h20); want(8'ha2, 32'h00002000); want(8'ha3, 0);

        // Quad I/O read straight after the erase (our WIP must not mask it).
        select_spi; spi_byte(8'heb);
        spi_quad(4'h0); spi_quad(4'h0); spi_quad(4'h3); spi_quad(4'h0);
        spi_quad(4'h0); spi_quad(4'h8);
        repeat (6) spi_quad(4'hf);           // mode FFh + 4 dummy
        repeat (8) begin #16.667; sck = 1; #16.667; sck = 0; end  // 4 bytes
        deselect_spi;
        want(8'ha1, 'heb); want(8'ha2, 32'h00003008); want(8'ha3, 4);

        // 4-byte read, although the configured chip lacks 4-byte support.
        select_spi; spi_byte(8'h13); spi_byte(8'h01); spi_byte(8'h23); spi_byte(8'h45);
        spi_byte(8'h67); repeat (2) spi_byte(0); deselect_spi;
        want(8'ha1, 'h13); want(8'ha2, 32'h01234567); want(8'ha3, 2);

        tick(100);
        observing = 0;

        // Drain the log.
        host_byte(8'h3a);
        begin
            automatic integer waited = 0;
            while ((raw.size() == 0 || raw[raw.size()-1] !== 8'ha0) && waited < 200000) begin
                tick(1); waited++;
                while (replies.size() != 0) raw.push_back(replies.pop_front());
            end
        end
        for (integer i = 0; i < raw.size() - 1; i++) begin
            if (raw[i] == 8'ha5) begin
                stream.push_back(raw[i+1] == 8'h00 ? 8'ha0 : 8'ha5); i++;
            end else stream.push_back(raw[i]);
        end
        foreach (expected[k]) begin
            integer len;
            int value;
            if (stream.size() == 0) $fatal(1, "log ended before packet %0d (%h)", k, expected[k].kind);
            if (stream[0] !== expected[k].kind)
                $fatal(1, "log packet %0d: type %h, expected %h", k, stream[0], expected[k].kind);
            void'(stream.pop_front());
            len = expected[k].kind == 8'ha1 ? 1 : expected[k].kind == 8'ha2 ? 4 : 3;
            value = 0;
            repeat (len) begin value = (value << 8) | int'(stream[0]); void'(stream.pop_front()); end
            if (value != expected[k].value)
                $fatal(1, "log packet %0d (%h): %h, expected %h", k, expected[k].kind, value, expected[k].value);
        end
        if (stream.size() != 0) $fatal(1, "%0d unexpected log bytes", stream.size());

        // Leaving the mode needs emulation stopped first.
        host_byte(8'h35); expect_reply(8'h01, "STOP");
        command2(8'h3c, 8'h00, 8'h01, "SNIFFCTL off while stopped");
        command2(8'h37, 8'h01, 8'h01, "HOLDCTL on after log-only mode");
        $display("PASS LOG_ONLY: hold exclusion, no bus/SDRAM/program activity, every command decoded into the log");
        $finish;
    end

    initial begin #20000000; $fatal(1, "log-only test timeout"); end
endmodule
