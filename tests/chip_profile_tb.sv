`timescale 1ns/1ps
// Controller compatibility framing and geometry. The completion model records
// accepted mutations; test-rflasher exercises the actual array engine.
module chip_profile_tb;
    reg clk=0; always #5 clk=~clk;
    reg sck=0, cs=1, reset=1, mosi=0, io1=0, io2=1, io3=1;
    reg [23:0] jedec=24'h00471f;
    wire miso, oe, command, byte_strobe, q0oe, q2oe, q3oe;
    wire [1:0] kind;
    wire [22:0] write_addr, write_len;
    wire [7:0] byte_value, byte_offset;
    reg done=0, seen=0, valid=1;
    reg [31:0] epoch=0;
    reg [1023:0] profile=0, compiled_snapshot=0;
    reg [22:0] capacity=23'h07ffff, compiled_capacity=0;
    reg [7:0] sfdp_byte=8'hff;
    reg store_reset=1, store_start=0, store_byte_valid=0;
    reg [7:0] store_byte=0;
    wire [459:0] compiled_profile;
    wire store_done, store_bad, erase_wren;
    wire [4:0] erase_waddr;
    wire [49:0] erase_wdata;
    initial begin #20; store_reset=0; end
    chip_profile_store store(
        .clk(clk),.reset(store_reset),.start(store_start),
        .byte_valid(store_byte_valid),.byte_data(store_byte),
        .capacity({1'b0,capacity,3'b000}+27'd8),.profile(compiled_profile),
        .done(store_done),.bad(store_bad),.erase_wren(erase_wren),
        .erase_waddr(erase_waddr),.erase_wdata(erase_wdata)
    );
    integer mutations=0, bytes_written=0;
    reg [22:0] last_addr, last_len;
    reg [7:0] last_byte;
    spi_trx dut(
        .clk(clk),.spi_clk(sck),.spi_reset(reset),.spi_csel(cs),
        .spi_io0_in(mosi),.spi_io1_in(io1),.spi_io2_in(io2),.spi_io3_in(io3),
        .spi_io1_out(miso),.spi_io1_oe(oe),.spi_io0_oe(q0oe),.spi_io2_oe(q2oe),.spi_io3_oe(q3oe),
        .write_cmd(command),.write_type(kind),.write_addr(write_addr),.write_len(write_len),
        .write_done(done),.write_buf_strobe(byte_strobe),.write_buf_val(byte_value),.write_buf_offset(byte_offset),
        .cfg_jedec_id(jedec),.cfg_4byte(1'b1),.cfg_chip_erase_bursts(capacity),
        .cfg_profile(compiled_profile),.cfg_profile_valid(valid),.cfg_profile_epoch(epoch),
        .cfg_erase_wren(erase_wren),.cfg_erase_waddr(erase_waddr),.cfg_erase_wdata(erase_wdata),
        .sfdp_rdata(sfdp_byte),.ram_read_buffer(64'hffffffffffffffff),
        .ram_read_buffer_b(64'hffffffffffffffff),.ram_read_beats_a(4'hf),.ram_read_beats_b(4'hf),
        .ram_read_valid_a(1'b1),.ram_read_valid_b(1'b1),.ram_read_busy(1'b0)
    );
    always @(posedge clk) begin
        if (!command) seen<=0;
        if (command && cs && !seen) begin
            seen<=1; done<=~done; mutations<=mutations+1;
            last_addr<=write_addr; last_len<=write_len;
        end
    end
    always @(negedge sck) if (byte_strobe) begin bytes_written<=bytes_written+1; last_byte<=byte_value; end
    task select_spi;
        if (profile != compiled_snapshot || capacity != compiled_capacity) begin
            if (!cs) $fatal(1,"configuration during transaction");
            @(negedge clk); store_start=1;
            @(negedge clk); store_start=0;
            for (integer i=0;i<128;i++) begin
                store_byte_valid=1; store_byte=profile[i*8 +:8]; @(negedge clk);
            end
            store_byte_valid=0; wait(store_done); @(negedge clk);
            if (store_bad) $fatal(1,"invalid test geometry");
            compiled_snapshot=profile; compiled_capacity=capacity;
        end
        cs=0; #100;
    endtask
    task deselect_spi; #100; cs=1; #500; endtask
    task automatic transfer(input [7:0] value,output [7:0] response);
        for (integer b=7;b>=0;b--) begin
            mosi=value[b]; #50; sck=1; #50; response[b]=oe?miso:1'b1; sck=0; #50;
        end
    endtask
    task automatic send(input [7:0] value); reg [7:0] ignored; transfer(value,ignored); endtask
    task automatic quad_send(input [7:0] value);
        for (integer n=1;n>=0;n--) begin
            {io3,io2,io1,mosi}=value[n*4 +:4]; #50; sck=1; #50; sck=0; #50;
        end
    endtask
    task automatic dual_send(input [7:0] value);
        for (integer n=3;n>=0;n--) begin
            {io1,mosi}=value[n*2 +:2]; #50; sck=1; #50; sck=0; #50;
        end
    endtask
    task automatic wait_phase(input integer cycles);
        for (integer i=0;i<cycles;i++) begin
            #50;sck=1;#50;sck=0;#50;
            if(oe !== (i==cycles-1))$fatal(1,"read latency %0d/%0d",i,cycles);
        end
    endtask
    task automatic simple(input [7:0] op); select_spi; send(op); deselect_spi; endtask
    task automatic address(input [23:0] a); send(a[23:16]);send(a[15:8]);send(a[7:0]); endtask
    task automatic read_register(input [7:0] op,output [7:0] value);
        select_spi;send(op);transfer(0,value);deselect_spi;
    endtask
    task automatic sector(input [7:0] op,input [23:0] a);
        simple(8'h06);select_spi;send(op);address(a);deselect_spi;
    endtask
    task automatic pp(input [23:0] a);
        simple(8'h06);select_spi;send(8'h02);address(a);send(8'h5a);deselect_spi;
    endtask
    task automatic base_profile(input [7:0] model);
        cs=1;reset=1;#100; profile=0;
        profile[7:0]=2;profile[15:8]=model;profile[24*8 +:8]=3;profile[28*8 +:8]=8;
        epoch=epoch+1'b1;#100;reset=0;
    endtask
    task automatic region(input integer d,input integer r,input [7:0] shift,input [23:0] count);
        profile[(36+d*24+r*4)*8 +:8]=shift;
        profile[(37+d*24+r*4)*8 +:24]=count;
    endtask
    reg [7:0] value;
    integer before_count;
    initial begin
        #500;
        base_profile(2);profile[2*8 +:8]=8'h0c;profile[7*8 +:8]=8'h20;
        profile[32*8 +:8]=8'h20;profile[34*8 +:8]=1;region(0,0,12,1024);
        read_register(8'h05,value);if(value!==8'h10)$fatal(1,"Atmel ready status");
        sector(8'h36,0);io2=0;
        select_spi;send(8'h01);send(8'hfc);deselect_spi;
        read_register(8'h05,value);if((value&8'hac)!==0)$fatal(1,"protection/error/WP gate");
        select_spi;send(8'h3c);address(0);transfer(0,value);deselect_spi;
        if(value!==0)$fatal(1,"Atmel sector indication");
        before_count=mutations;pp(0);sector(8'h20,0);
        if(mutations!=before_count+2)$fatal(1,"permissive mutation");io2=1;

        for(integer model=3;model<=5;model++)begin
            base_profile(8'(model));read_register(8'h05,value);
            simple(8'h06);select_spi;send(8'h42);
            repeat(18)send(8'hff);deselect_spi;
            select_spi;send(8'h72);repeat(20)begin
                transfer(0,value);if(value!==0)$fatal(1,"BPR indication");end
            deselect_spi;
        end
        // Pipelined compiler must retain all five rows, even when the final
        // row is active and bytes arrive on every system clock.
        base_profile(3);capacity=23'h03ffff;
        profile[32*8 +:8]=8'hd8;profile[34*8 +:8]=5;
        region(0,0,13,4);region(0,1,15,1);region(0,2,16,30);region(0,3,15,1);region(0,4,13,4);
        profile[56*8 +:8]=8'h20;profile[58*8 +:8]=1;region(1,0,12,512);
        sector(8'hd8,24'h002345);
        if(last_addr!==23'h400||last_len!==23'd1023)$fatal(1,"8K geometry");
        sector(8'hd8,24'h00a345);
        if(last_addr!==23'h1000||last_len!==23'd4095)$fatal(1,"32K geometry");
        sector(8'hd8,24'h01a345);
        if(last_addr!==23'h2000||last_len!==23'd8191)$fatal(1,"64K geometry");
        sector(8'h20,24'h001234);if(last_addr!==23'h200||last_len!==23'd511)$fatal(1,"next descriptor coverage");
        base_profile(0);capacity=23'h00ffff;
        profile[32*8 +:8]=8'h50;profile[34*8 +:8]=1;region(0,0,11,256);
        sector(8'h50,24'ha00);if(last_addr!==23'h100||last_len!==23'd255)$fatal(1,"50 erase priority");

        base_profile(0);sfdp_byte=8'h42;
        select_spi;send(8'h5a);address(127);send(0);transfer(0,value);
        if(value!==8'h42)$fatal(1,"SFDP end byte");transfer(0,value);
        if(value!==8'hff)$fatal(1,"SFDP wrap");deselect_spi;
        select_spi;send(8'h5a);address(24'h1000);send(0);transfer(0,value);
        if(value!==8'hff)$fatal(1,"SFDP high address");deselect_spi;

        base_profile(0);capacity=23'h7fffff;profile[10*8 +:8]=8'h08;
        profile[16*8 +:64]=64'h8070605040302010;
        select_spi;send(8'h4b);repeat(4)send(0);
        for(integer i=0;i<8;i++)begin transfer(0,value);
            if(value!==8'(16*(i+1)))$fatal(1,"UID framing");end
        transfer(0,value);if(value!==8'hff)$fatal(1,"UID wraps");deselect_spi;
        select_spi;send(8'hc5);send(2);deselect_spi;
        pp(24'h1234);if(last_addr!==23'h400240)$fatal(1,"EAR program");
        simple(8'h66);simple(8'h99);read_register(8'hc8,value);if(value!==0)$fatal(1,"reset EAR");
        profile[104*8 +:8]=8'h20;profile[105*8 +:8]=8'h21;profile[106*8 +:8]=1;
        region(3,0,8,24'h040000);epoch=epoch+1'b1;
        select_spi;send(8'hc5);send(2);deselect_spi;
        sector(8'h20,24'h0123ff);
        if(last_addr!==23'h402460||last_len!==23'd31)$fatal(1,"fourth descriptor EAR");
        simple(8'h06);select_spi;send(8'h21);send(2);send(1);send(8'h23);#1000;send(8'hff);deselect_spi;
        if(last_addr!==23'h402460||last_len!==23'd31)$fatal(1,"native paused lookup");

        // Quad PP and quad read work even after clearing QE. Program bytes
        // are latched in nibble order; the command remains single-lane.
        base_profile(0);jedec=24'h1840ef;profile[28*8 +:8]=7;
        select_spi;send(8'h31);send(0);deselect_spi;before_count=bytes_written;
        simple(8'h06);select_spi;send(8'h32);address(24'h7f);quad_send(8'ha5);quad_send(8'h3c);deselect_spi;
        if(bytes_written!=before_count+2||last_byte!==8'h3c||byte_offset!==0)$fatal(1,"quad PP/subpage framing");
        read_register(8'h05,value);
        select_spi;send(8'h6b);address(0);send(0);
        if(!q0oe||!q2oe||!q3oe)$fatal(1,"QE gating remains");deselect_spi;
        // Native 1-1-4 PP and Macronix 1-4-4 PP share the page engine.
        simple(8'h06);select_spi;send(8'h34);send(2);address(24'h0123ff);
        quad_send(8'ha5);quad_send(8'h3c);deselect_spi;
        if(last_addr!==23'h402460||last_byte!==8'h3c)$fatal(1,"native quad PP");
        base_profile(0);jedec=24'h1820c2;
        simple(8'h06);select_spi;send(8'h38);quad_send(1);quad_send(2);quad_send(8'h7f);
        quad_send(8'ha5);quad_send(8'h3c);deselect_spi;
        if(last_addr!==23'h2040||last_byte!==8'h3c)$fatal(1,"Macronix quad address/data PP");
        select_spi;send(8'h01);send(8'h40);send(8'h47);deselect_spi;
        read_register(8'h15,value);if(value!==8'h47)$fatal(1,"Macronix Config alias");
        simple(8'hb7);read_register(8'h15,value);if(value!==8'h67)$fatal(1,"Macronix mode view");
        simple(8'he9);select_spi;send(8'heb);quad_send(0);quad_send(0);quad_send(0);
        wait_phase(4);if(!q2oe)$fatal(1,"Macronix dynamic quad wait");deselect_spi;

        // Micron configured multi-IO waits can exceed eight clocks, and
        // zero/F retain opcode-specific defaults rather than fifteen clocks.
        base_profile(0);jedec=24'h19ba20;
        select_spi;send(8'h81);send(8'hab);deselect_spi;
        select_spi;send(8'hbb);dual_send(0);dual_send(0);dual_send(0);
        wait_phase(10);if(!q0oe)$fatal(1,"Micron dual wait");deselect_spi;
        select_spi;send(8'h81);send(8'hfb);deselect_spi;
        select_spi;send(8'heb);quad_send(0);quad_send(0);quad_send(0);
        wait_phase(10);deselect_spi;
        select_spi;send(8'hb1);send(8'hfe);send(8'h6f);deselect_spi;
        simple(8'h66);simple(8'h99);
        select_spi;send(8'hb5);transfer(0,value);if(value!==8'hfe)$fatal(1,"NVCR reset low");
        transfer(0,value);if(value!==8'h6f)$fatal(1,"NVCR reset high");deselect_spi;
        select_spi;send(8'h0b);send(0);address(0);wait_phase(6);deselect_spi;
        base_profile(0);jedec=24'h196001;
        select_spi;send(8'h71);address(24'h800003);send(8'h0b);deselect_spi;
        select_spi;send(8'heb);quad_send(0);quad_send(0);quad_send(0);
        wait_phase(24);deselect_spi;
        $display("PASS chip compatibility: permissive initialization, opcode RAM, geometry, EAR/native/paused SCK, UID/SFDP, quad program and vendor-configured latency");
        $finish;
    end
    initial begin #10000000;$fatal(1,"profile test timeout");end
endmodule
