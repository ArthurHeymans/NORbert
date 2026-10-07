`timescale 1ns/1ps

// Observe status over the pins, including SCK clocks for a different slave.
module write_completion_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;
    reg sck = 0, cs = 1, reset = 1, mosi = 0, done = 0;
    wire miso, oe;
    spi_trx dut(.clk(clk), .spi_clk(sck), .spi_reset(reset), .spi_csel(cs),
        .spi_io0_in(mosi), .spi_io1_in(1'b0), .spi_io2_in(1'b0), .spi_io3_in(1'b0),
        .spi_io1_out(miso), .spi_io1_oe(oe), .write_done(done),
        .cfg_jedec_id(24'h1740ef), .cfg_4byte(1'b0), .cfg_chip_erase_bursts(23'h0fffff),
        .sfdp_rdata(8'hff), .ram_read_buffer(64'b0), .ram_read_buffer_b(64'b0),
        .ram_read_valid_a(1'b0), .ram_read_valid_b(1'b0), .ram_read_busy(1'b0));
    task transfer(input [7:0] b, output [7:0] r);
        for (integer i=7; i>=0; i--) begin
            mosi = b[i]; #20; sck = 1; #10; r[i] = miso; #10; sck = 0;
        end
    endtask
    task send(input [7:0] b); reg [7:0] r; transfer(b,r); endtask
    task select_spi; cs=0; #40; endtask
    task deselect_spi;
        cs=1; #1;
        if (oe) $fatal(1,"output enable survived CS high");
        #79;
    endtask
    task command(input [7:0] b); select_spi; send(b); deselect_spi; endtask
    task status(input bit busy);
        reg [7:0] r;
        select_spi; send(8'h05); transfer(0,r); deselect_spi;
        if (r[0] !== busy) $fatal(1,"WIP=%b, expected %b (status %h)",r[0],busy,r);
    endtask
    task start_write(input bit erase);
        command(8'h06);
        select_spi; send(erase ? 8'h20 : 8'h02);
        send(0); send(0); send(0);
        if (!erase) send(8'h39);
        deselect_spi; status(1);
    endtask
    initial begin
        #40; reset=0;
        for (integer kind=0; kind<2; kind++) begin
            start_write(kind != 0);
            done = !done;
            // Other devices clock the common SCK with this chip deselected.
            repeat(10) begin #20;sck=1;#20;sck=0;end
            status(0); status(0);
            start_write(kind != 0);
            done = !done;
            #5000; // No SCK: completion must survive until the next status read.
            status(0);
        end
        start_write(1);
        reset=1; #100; reset=0;
        status(0);
        $display("PASS WRITE_COMPLETION: shared-bus clocks, stopped SCK, program/erase and reset");
        $finish;
    end
endmodule
