`timescale 1ns/1ps

// Top-level SPI address CDC, trap priority and accepted SDRAM addresses.
module toctou_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;
    reg sck = 0, cs = 1, mosi = 0;
    reg host_strobe = 0, injected_addr_event = 0;
    reg [7:0] host_data = 0;
    reg quad_address = 0;
    reg [3:0] quad_input = 0;
    wire io0, io1, io2, io3;
    assign io0 = quad_address ? quad_input[0] : mosi;
    assign io1 = quad_address ? quad_input[1] : 1'bz;
    assign io2 = quad_address ? quad_input[2] : 1'bz;
    assign io3 = quad_address ? quad_input[3] : 1'bz;
    top dut (
        .clk_50mhz(clk), .uart_rx(1'b1), .spi_cs_pin(cs), .spi_clk_pin(sck),
        .spi_mosi_pin(io0), .spi_miso_pin(io1), .spi_io2_pin(io2), .spi_io3_pin(io3),
        .spi_power_pin(1'b1), .ft_rxf_n(1'b1), .ft_txe_n(1'b1)
    );

    integer notifications = 0, accepted_reads = 0, delayed_initial_reads = 0;
    reg watch_transaction = 0, want_redirect = 0, previous_read_ack = 0;
    reg [23:0] current_address = 0, expected_mask = 0, expected_base = 0;
    reg [1:0] expected_index = 0;
    reg [22:0] wanted_burst, actual_burst;
    always @(negedge clk) begin
        if (watch_transaction && dut.trap_notify_strobe) begin
            notifications++;
            if (!want_redirect || dut.trap_notify_addr !== current_address ||
                dut.trap_notify_index !== expected_index)
                $fatal(1, "Trap notification has wrong address, priority or access count");
        end
        // Check what the SDRAM controller actually accepted, not merely
        // the live address mux. A late redirect could mix an OLD latched
        // row/bank with a NEW column, even though the mux now looks right.
        if (watch_transaction && dut.sdram_i.spi_cmd_read_ack && !previous_read_ack) begin
            if (accepted_reads == 0 && want_redirect && dut.redirect_active)
                delayed_initial_reads++;
            wanted_burst = {2'b0, current_address[23:3]} + 23'(accepted_reads);
            if (want_redirect && accepted_reads != 0)
                wanted_burst = (wanted_burst & ~{2'b0, expected_mask[23:3]}) |
                    {2'b0, (expected_base[23:3] & expected_mask[23:3])};
            actual_burst = {dut.O_sdram_cs_n, dut.sdram_i.spi_addr_latched[21:9],
                            dut.O_sdram_ba, dut.O_sdram_addr[8:2]};
            if (actual_burst !== wanted_burst)
                $fatal(1, "Read %h burst %0d: SDRAM accepted %h, expected %h",
                       current_address, accepted_reads, actual_burst, wanted_burst);
            accepted_reads++;
        end
        previous_read_ack = dut.sdram_i.spi_cmd_read_ack;
    end

    task host_byte(input [7:0] b);
        @(negedge clk); host_data = b; host_strobe = 1;
        @(negedge clk); host_strobe = 0;
        repeat (10) @(negedge clk);
    endtask

    // Align an address event with the parser consuming a final host byte.
    task host_collision_byte(input [7:0] b);
        @(negedge clk); host_data = b; host_strobe = 1;
        @(negedge clk); host_strobe = 0; injected_addr_event = 1;
        @(negedge clk); injected_addr_event = 0;
        repeat (3) @(negedge clk);
    endtask

    task host_address(input [23:0] a);
        host_byte(a[23:16]); host_byte(a[15:8]); host_byte(a[7:0]);
    endtask

    task set_trap(input [1:0] index, input [23:0] start,
                  input [23:0] mask, input [23:0] replacement);
        host_byte(8'h39); host_byte(1); host_byte({6'b0, index});
        host_address(start); host_address(mask); host_address(replacement);
        host_byte(8'h39); host_byte(2); host_byte({6'b0, index});
    endtask

    task spi_byte(input [7:0] b);
        for (integer i = 7; i >= 0; i--) begin
            mosi = b[i]; #16.667; sck = 1; #16.667; sck = 0;
        end
    endtask

    task read_header(input [23:0] a, input [7:0] opcode = 8'h6b);
        cs = 0; #35;
        spi_byte(opcode);
        if (opcode == 8'heb) begin
            quad_address = 1;
            for (integer bit_index = 20; bit_index >= 0; bit_index -= 4) begin
                quad_input = a[bit_index +: 4];
                #16.667; sck = 1; #16.667; sck = 0;
            end
            quad_address = 0;
        end else begin
            if (opcode == 8'h13) spi_byte(0);
            spi_byte(a[23:16]); spi_byte(a[15:8]); spi_byte(a[7:0]);
        end
    endtask

    task read_flash(input [23:0] a, input [3:0] triggered, input bit redirect,
                    input [1:0] index, input [23:0] mask, input [23:0] replacement,
                    input [7:0] opcode = 8'h6b);
        current_address = a; want_redirect = redirect;
        expected_index = index; expected_mask = mask; expected_base = replacement;
        notifications = 0; accepted_reads = 0; watch_transaction = 1;
        read_header(a, opcode);
        if (opcode == 8'h6b) spi_byte(0); // Eight dummy clocks
        if (opcode == 8'heb)
            repeat (6) begin #16.667; sck = 1; #16.667; sck = 0; end
        repeat (10) spi_byte(0); // Forty quad or ten single data bytes
        if (dut.glue_i.trap_triggered !== triggered || dut.redirect_active !== redirect ||
            notifications != (redirect ? 1 : 0) || accepted_reads < 2)
            $fatal(1, "Wrong trap/redirect state for address %h", a);
        #35; cs = 1;
        repeat (20) @(negedge clk);
        if (dut.redirect_active) $fatal(1, "Redirect survived CS deassertion");
        watch_transaction = 0;
    endtask

    initial begin
        // Bypass only the UART transport. Host parser, SPI, address toggle
        // synchronizer, trap pipeline, logger and SDRAM wiring stay real.
        force dut.uart_rxd_strobe = host_strobe;
        force dut.uart_rxd = host_data;
        wait (!dut.reset);
        repeat (300) @(negedge clk);
        host_byte(8'h33); host_byte(8'hef); host_byte(8'h40); host_byte(8'h17);
        host_byte(1); host_address(24'h7fffff); host_byte(0); // Enable 0x13
        // Non-contiguous mask, ignored start bits, and overlapping entries.
        set_trap(0, 24'h0010ff, 24'h00f0f0, 24'h003080);
        set_trap(1, 24'h0010a5, 24'h00f000, 24'h002000);
        host_byte(8'h34);
        wait (!dut.spi_reset_effective);
        read_flash(24'h002000, 4'b0000, 0, 0, 0, 0); // Non-match first
        read_flash(24'h0015f2, 4'b0011, 0, 0, 0, 0); // Must use THIS address
        read_flash(24'h0015f2, 4'b0011, 1, 1, 24'h00f000, 24'h002000);
        read_flash(24'h0015f2, 4'b0011, 1, 1, 24'h00f000, 24'h002000);
        // No-dummy reads at offsets 6/7 prefetch immediately. Sweep the
        // SPI/system phase, checking the accepted row/bank AND column.
        for (integer phase = 0; phase < 9; phase++) begin
            #(phase+1);
            read_flash(24'h0015f6, 4'b0011, 1, 1, 24'h00f000, 24'h002000, 8'h03);
            #(phase+1);
            read_flash(24'h0015f7, 4'b0011, 1, 1, 24'h00f000, 24'h002000, 8'h13);
        end

        // End CS immediately after the last address bit, while the system
        // comparison is still pending. No extra SPI clock is needed for
        // the address event, and a late redirect must not survive CS.
        read_header(24'h0015f2);
        cs = 1;
        repeat (20) @(negedge clk);
        if (dut.redirect_active) $fatal(1, "Late pending trap leaked past CS");
        read_flash(24'h002000, 4'b0011, 0, 0, 0, 0);

        host_byte(8'h39); host_byte(5); // Reset/disarm all
        set_trap(2, 24'hffffff, 24'h000000, 24'h005000);
        read_flash(24'h123456, 4'b0100, 0, 0, 0, 0);
        read_flash(24'h654320, 4'b0100, 1, 2, 0, 24'h005000);
        host_byte(8'h39); host_byte(5);
        set_trap(3, 24'h001234, 24'hffffff, 24'h006000);
        read_flash(24'h001235, 4'b0000, 0, 0, 0, 0);
        read_flash(24'h001234, 4'b1000, 0, 0, 0, 0);
        read_flash(24'h001234, 4'b1000, 1, 3, 24'hffffff, 24'h006000);
        // 0xEB posts its initial READ and address event together. Sweep
        // refresh phases so the trap can finish between ACTIVATE and READ.
        // The first burst must stay original (both row AND column).
        host_byte(8'h39); host_byte(5);
        set_trap(0, 24'h001238, 24'hfffff8, 24'h007458);
        read_flash(24'h001238, 4'b0001, 0, 0, 0, 0, 8'heb);
        delayed_initial_reads = 0;
        for (integer phase = 0; phase < 40; phase++) begin
            while (dut.sdram_i.refreshcount != 392+phase) @(negedge clk);
            read_flash(24'h001238, 4'b0001, 1, 0, 24'hfffff8, 24'h007458, 8'heb);
        end
        if (delayed_initial_reads == 0) $fatal(1, "Missing initial-READ/redirect overlap coverage");

        // Unit-check the trap pipeline's command priority at its input
        // interface: a pending comparison cannot undo a concurrent reset.
        force dut.glue_i.log_addr_sync = 24'h001238;
        force dut.glue_i.log_addr_valid_sync = injected_addr_event;
        host_byte(8'h39); host_collision_byte(5); // RESET_ALL
        if (dut.glue_i.trap_triggered != 0) $fatal(1, "Pending comparison undid RESET_ALL");
        host_byte(8'h39); host_byte(2); host_byte(0); // ARM entry 0 again
        host_byte(8'h39); host_byte(4); host_collision_byte(0); // RESET entry 0
        if (dut.glue_i.trap_triggered != 0) $fatal(1, "Pending comparison undid RESET");
        release dut.glue_i.log_addr_sync;
        release dut.glue_i.log_addr_valid_sync;
        $display("PASS TOCTOU: address CDC, masks, priority, single/quad requests, refresh overlap and CS cleanup");
        $finish;
    end

    initial begin
        #5000000;
        $fatal(1, "TOCTOU test timeout");
    end
endmodule
