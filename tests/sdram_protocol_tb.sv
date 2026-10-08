`timescale 1ns/1ps

// SDRAM bank protocol at the controller's client boundary.
//
// The SPI fast path and the serial (host/program) path share the banks.
// Either may hold a row between its ACTIVATE and auto-precharged access;
// the other must not open a bank, refresh or access under it. An SPI post
// can also be withdrawn after its ACTIVATE but before its READ (a refresh
// delayed the pair), and the row must still be closed afterwards.
module sdram_protocol_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;
    reg reset = 1;
    reg spi_active = 0, spi_inhibit = 0, spi_activate = 0, spi_read = 0;
    reg [22:0] spi_addr = 0;
    reg [1:0] access_cmd = 0;
    reg [24:0] access_addr = 0;
    wire ras, cas, we, chip, busy, accept, read_busy;
    wire [1:0] bank;
    wire [12:0] a;
    wire [15:0] dq = 16'h1234;
    wire [63:0] read_buffer;

    sdram #(.CLK_FREQ_MHZ(120)) ram(.clk(clk), .aux_clk(clk), .reset(reset), .dq_io(dq),
        .spi_active(spi_active), .spi_inhibit_refresh(spi_inhibit),
        .spi_cmd_activate(spi_activate), .spi_cmd_read(spi_read),
        .spi_cmd_post_toggle(1'b0), .spi_addr(spi_addr),
        .access_cmd(access_cmd), .access_addr(access_addr), .inhibit_refresh(1'b0),
        .cmd_busy(busy), .access_accept(accept), .read_buffer(read_buffer),
        .read_busy(read_busy), .write_buffer(64'h0),
        .ras_o(ras), .cas_o(cas), .we_o(we), .cs_o(chip), .ba_o(bank), .a_o(a));

    reg checking = 0;
    sdram_bank_checker banks(.clk(clk), .enable(checking), .cs(chip), .ras(ras),
        .cas(cas), .we(we), .ba(bank), .a(a));

    // Command trace, so ordering between the two clients can be asserted.
    integer cycle = 0, last_spi_activate = -1, last_spi_read = -1;
    integer last_serial_access = -1, last_precharge = -1;
    always @(negedge clk) begin
        cycle++;
        case ({ras, cas, we})
            3'b011: if (a == spi_addr[21:9] && bank == spi_addr[8:7]) last_spi_activate = cycle;
            // A serial READ sets serial_read_active at dispatch; an SPI
            // READ clears it. WRITEs are serial only.
            3'b101: if (ram.serial_read_active) last_serial_access = cycle;
                    else last_spi_read = cycle;
            3'b100: last_serial_access = cycle;
            3'b010: last_precharge = cycle;
            default: ;
        endcase
    end

    task tick(input integer n); repeat (n) @(negedge clk); endtask

    task serial_post(input [1:0] cmd, input [24:0] addr);
        @(negedge clk); access_cmd = cmd; access_addr = addr;
        while (!accept) @(negedge clk);
        access_cmd = 0;
    endtask

    // A transaction starts with the request levels low; the controller
    // arms SPI dispatch only after observing that while selected.
    task spi_select;
        spi_active = 1; tick(5);
    endtask

    task spi_idle;
        spi_activate = 0; spi_read = 0; spi_inhibit = 0; spi_active = 0;
        tick(40);
    endtask

    // Wait for the controller to settle and for the refresh counter to be
    // far from due, so the scenarios below are not interleaved with one.
    task quiet;
        while (ram.refreshcount > 100 || busy) @(negedge clk);
    endtask

    initial begin
        tick(4); reset = 0;
        tick(14000);
        checking = 1;

        // 1. SPI post withdrawn between ACTIVATE and READ. The READ level
        //    never arrives; the controller must precharge the bank itself.
        quiet;
        spi_addr = {1'b0, 13'h0123, 2'd2, 7'h11};
        spi_select; spi_inhibit = 1; spi_activate = 1;
        while (last_spi_activate < 0) @(negedge clk);
        spi_activate = 0; spi_inhibit = 0;  // re-arm drop before READ
        tick(30);
        if (last_precharge < last_spi_activate)
            $fatal(1, "withdrawn SPI ACTIVATE left its row open");
        if (ram.spi_activate_done) $fatal(1, "open-row tracking not cleared after close");
        // The next post to the same bank must be legal and complete.
        spi_addr[6:0] = 7'h12;
        spi_inhibit = 1; spi_activate = 1; spi_read = 1;
        tick(30);
        if (last_spi_read < last_spi_activate) $fatal(1, "SPI READ after close missing");
        spi_activate = 0; spi_read = 0; spi_inhibit = 0;
        // Let refresh run with the bank state known to be clean.
        tick(1200);
        spi_idle;

        // 2. The same withdrawal, but by CS release.
        quiet;
        spi_addr = {1'b1, 13'h0456, 2'd1, 7'h20};
        last_spi_activate = -1;
        spi_select; spi_inhibit = 1; spi_activate = 1;
        while (last_spi_activate < 0) @(negedge clk);
        spi_active = 0;
        tick(30);
        if (last_precharge < last_spi_activate)
            $fatal(1, "deselected SPI ACTIVATE left its row open");
        spi_idle;

        // 3. A serial pair holds a row while an SPI post arrives for the
        //    same bank. The SPI ACTIVATE must wait for the serial access.
        quiet;
        spi_addr = {1'b0, 13'h0777, 2'd3, 7'h05};
        last_spi_activate = -1; last_spi_read = -1;
        serial_post(2'b11, {1'b0, 13'h0100, 2'd3, 9'h040});
        spi_select; spi_inhibit = 1; spi_activate = 1; spi_read = 1;
        tick(25);  // a slow serial client still owns the row
        if (last_spi_activate >= 0) $fatal(1, "SPI ACTIVATE opened a bank under a serial pair");
        serial_post(2'b01, {1'b0, 13'h0100, 2'd3, 9'h040});
        tick(30);
        if (last_spi_activate < last_serial_access || last_spi_read < last_spi_activate)
            $fatal(1, "SPI post not serviced after the serial access");
        spi_idle;

        // 4. An SPI row waits for its READ (SCK slow) while a serial
        //    request arrives. The serial ACTIVATE must wait for the READ.
        quiet;
        spi_addr = {1'b0, 13'h0888, 2'd0, 7'h33};
        last_spi_activate = -1; last_spi_read = -1; last_serial_access = -1;
        spi_select; spi_inhibit = 1; spi_activate = 1;
        while (last_spi_activate < 0) @(negedge clk);
        fork
            serial_post(2'b11, {1'b0, 13'h0200, 2'd0, 9'h000});
            begin
                tick(20);
                if (ram.serial_row_open) $fatal(1, "serial ACTIVATE under an open SPI row");
                spi_read = 1;
            end
        join
        serial_post(2'b10, {1'b0, 13'h0200, 2'd0, 9'h000});
        tick(20);
        if (last_spi_read < 0 || last_serial_access < last_spi_read)
            $fatal(1, "serial access was not ordered after the SPI READ");
        spi_idle;

        $display("PASS SDRAM PROTOCOL: withdrawn SPI rows closed, SPI/serial row interlock (%0d ACT, %0d PRE, %0d REF)",
                 banks.activates, banks.precharges, banks.refreshes);
        $finish;
    end

    initial begin #2000000; $fatal(1, "sdram protocol test timeout"); end
endmodule
