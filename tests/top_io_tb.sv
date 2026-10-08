`timescale 1ns/1ps

// Top-level SPI pad behaviour with the power-detect pin left unconnected
// (pulled low). With BYPASS_POWER_DETECT the emulator must still drive
// MISO while selected, and release it as soon as CS rises.
module top_io_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;  // The PLL stub passes its input through
    reg sck = 0, cs = 1, mosi = 0;
    reg host_strobe = 0;
    reg [7:0] host_data = 0;
    wire io0, io1, io2, io3;
    assign io0 = mosi;
    // Released pads read high, so a disabled driver shows up as FF bytes.
    pullup (io1);
    top dut (
        .clk_50mhz(clk), .uart_rx(1'b1), .spi_cs_pin(cs), .spi_clk_pin(sck),
        .spi_mosi_pin(io0), .spi_miso_pin(io1), .spi_io2_pin(io2), .spi_io3_pin(io3),
        .spi_power_pin(1'b0), .ft_rxf_n(1'b1), .ft_txe_n(1'b1)
    );

    task host_byte(input [7:0] b);
        @(negedge dut.clk); host_data = b; host_strobe = 1;
        @(negedge dut.clk); host_strobe = 0;
        repeat (10) @(negedge dut.clk);
    endtask

    task spi_send(input [7:0] b);
        for (integer i = 7; i >= 0; i--) begin
            mosi = b[i]; #16.667; sck = 1; #16.667; sck = 0;
        end
    endtask

    task spi_xfer(input [7:0] b, output [7:0] r);
        for (integer i = 7; i >= 0; i--) begin
            mosi = b[i]; #16.667; sck = 1;
            if (!(dut.spi_io1_oe && dut.spi_active_out))
                $fatal(1, "MISO not driven while selected (power pin low)");
            r[i] = io1; #16.667; sck = 0;
        end
    endtask

    initial begin
        reg [7:0] id [0:2];
        force dut.uart_rxd_strobe = host_strobe;
        force dut.uart_rxd = host_data;
        wait (!dut.reset);
        repeat (300) @(negedge dut.clk);
        host_byte(8'h34);  // START
        wait (!dut.spi_reset_effective);
        cs = 0; #35;
        spi_send(8'h9f);
        for (integer i = 0; i < 3; i++) spi_xfer(0, id[i]);
        #35; cs = 1; #1;
        if (io1 !== 1'b1) $fatal(1, "MISO not released after CS rose");
        if (id[0] !== 8'hef || id[1] !== 8'h40 || id[2] !== 8'h17)
            $fatal(1, "JEDEC ID %h %h %h, expected ef 40 17", id[0], id[1], id[2]);
        // The idle status register's MSB is zero. Deselect while that bit
        // is driven so a stuck output enable cannot hide behind the pull-up.
        #35; cs = 0; #35;
        spi_send(8'h05);
        #16.667; sck = 1; #1;
        if (io1 !== 1'b0) $fatal(1, "MISO not low before deselection");
        cs = 1; #1;
        if (io1 !== 1'b1) $fatal(1, "MISO not released after CS rose");
        sck = 0;
        $display("PASS TOP_IO: outputs driven with the power pin unconnected, released at CS high");
        $finish;
    end

    initial begin #3000000; $fatal(1, "top_io test timeout"); end
endmodule
