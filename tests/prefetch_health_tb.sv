`timescale 1ns/1ps

// Short reads must check the actual beat, not eventual burst completion.
module prefetch_health_tb;
    reg clk=0;
    always #4.167 clk=~clk;
    reg sck=0, cs=1, reset=1, io0=0;
    reg [3:0] beats=0;
    wire underrun, thin;
    spi_trx dut(.clk(clk), .spi_clk(sck), .spi_reset(reset), .spi_csel(cs),
        .spi_io0_in(io0), .spi_io1_in(1'b0), .spi_io2_in(1'b0), .spi_io3_in(1'b0),
        .write_done(1'b0), .cfg_jedec_id(24'h1740ef), .cfg_4byte(1'b0),
        .cfg_chip_erase_bursts(23'h0fffff), .sfdp_rdata(8'hff),
        .ram_read_buffer(64'h0123456789abcdef), .ram_read_buffer_b(64'hfedcba9876543210),
        .ram_read_valid_a(beats[0]), .ram_read_valid_b(1'b0),
        .ram_read_beats_a(beats), .ram_read_beats_b(4'b0), .ram_read_busy(1'b1),
        .prefetch_underrun(underrun), .prefetch_thin(thin));
    task bit_spi(input bit b); io0=b; #20; sck=1; #20; sck=0; endtask
    task byte_spi(input [7:0] b); for(integer i=7;i>=0;i--) bit_spi(b[i]); endtask
    task start_read(input [7:0] opcode, input [7:0] offset);
        cs=0;#40;byte_spi(opcode);byte_spi(0);byte_spi(0);byte_spi(offset);
        if(opcode==8'h0b || opcode==8'h3b || opcode==8'h6b) byte_spi(0);
    endtask
    task finish_read(input bit missing);
        cs=1;#80;
        if(underrun!==missing) $fatal(1,"underrun=%b expected %b",underrun,missing);
        // Flags survive deselection even without another SPI clock.
        #200; if(underrun!==missing) $fatal(1,"fault lost after CS high");
    endtask
    initial begin
        #40;reset=0;
        for(integer lanes=1;lanes<=4;lanes*=2)
            for(integer off=0;off<8;off++) begin
                beats=0;
                start_read(lanes==1?8'h0b:lanes==2?8'h3b:8'h6b,8'(off));
                repeat(8/lanes) bit_spi(0);
                finish_read(1);
                beats=4'hf;
                start_read(lanes==1?8'h0b:lanes==2?8'h3b:8'h6b,8'(off));
                repeat(8/lanes) bit_spi(0);
                finish_read(0);
            end
        // Beat zero ready does not imply bytes in beats 1..3 are ready.
        beats=1;start_read(8'h03,6);byte_spi(0);finish_read(1);
        beats=1;start_read(8'h03,0);byte_spi(0);finish_read(0);
        // Filling after the first output bit must not clear that failure.
        beats=0;start_read(8'h03,0);
        #10;beats=4'hf;
        byte_spi(0);finish_read(1);
        // Non-array commands do not inspect SDRAM readiness; a new
        // transaction resets an earlier fault without clocks during CS high.
        beats=0;cs=0;#40;byte_spi(8'h9f);byte_spi(0);finish_read(0);
        $display("PASS PREFETCH_HEALTH: short single/dual/quad reads, all offsets, beat validity and late fills");
        $finish;
    end
endmodule
