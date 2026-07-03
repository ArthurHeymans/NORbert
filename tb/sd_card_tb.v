// Testbench for sd_card_1bit against the real sdram.v controller plus a
// minimal SDRAM chip model.  Loads an image via the serial access path
// (like norbert-tool load), then exercises the SD protocol end to end:
// init sequence, CSD/SCR content, CMD17 single-block read with data
// compare, out-of-range rejection, CMD18 multi-block + CMD12 stop.
//
// Run: make test
`timescale 1ns/1ps
`default_nettype none

// ---------------------------------------------------------------------------
// Minimal SDRAM chip model: ACT/RD/WR, CAS latency 3, burst 4, 16-bit bus.
// Only models chip 0 (cs low).  Storage is flat: one 16-bit word per
// {row[3:0], bank, col[8:2], word-in-burst}.
// ---------------------------------------------------------------------------
module sdram_model(
    input wire clk,
    input wire cs,
    input wire ras,
    input wire cas,
    input wire we,
    input wire [1:0] ba,
    input wire [12:0] a,
    inout wire [15:0] dq
);
    reg [15:0] mem [0:32767];
    reg [12:0] open_row [0:3];

    reg [15:0] dq_o;
    reg dq_oe = 0;
    assign dq = dq_oe ? dq_o : 16'hzzzz;

    wire cmd_act = (cs == 0) && (ras == 0) && (cas == 1) && (we == 1);
    wire cmd_rd  = (cs == 0) && (ras == 1) && (cas == 0) && (we == 1);
    wire cmd_wr  = (cs == 0) && (ras == 1) && (cas == 0) && (we == 0);

    function [14:0] base;
        input [12:0] row;
        input [1:0] bank;
        input [8:0] col;
        base = {row[3:0], bank, col[8:2], 2'b00};
    endfunction

    reg [14:0] rd_ptr;
    reg [1:0] rd_wait;
    reg [2:0] rd_wcnt;
    reg rd_run = 0;
    reg [14:0] wr_ptr;
    reg [1:0] wr_left;
    reg wr_run = 0;

    always @(posedge clk) begin
        if (cmd_act)
            open_row[ba] <= a;

        // read: after CL=3 drive 4 words
        if (cmd_rd) begin
            rd_ptr <= base(open_row[ba], ba, a[8:0]);
            rd_wait <= 2;
            rd_wcnt <= 4;
            rd_run <= 1;
            dq_oe <= 0;
        end else if (rd_run) begin
            if (rd_wait != 0) begin
                rd_wait <= rd_wait - 1;
                dq_oe <= 0;
            end else begin
                dq_oe <= 1;
                dq_o <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1;
                rd_wcnt <= rd_wcnt - 1;
                if (rd_wcnt == 1)
                    rd_run <= 0;
            end
        end else begin
            dq_oe <= 0;
        end

        // write: capture 4 words starting this cycle
        if (cmd_wr) begin
            mem[base(open_row[ba], ba, a[8:0])] <= dq;
            wr_ptr <= base(open_row[ba], ba, a[8:0]) + 1;
            wr_left <= 3;
            wr_run <= 1;
        end else if (wr_run) begin
            mem[wr_ptr] <= dq;
            wr_ptr <= wr_ptr + 1;
            if (wr_left == 1)
                wr_run <= 0;
            wr_left <= wr_left - 1;
        end
    end
endmodule

// ---------------------------------------------------------------------------
// Testbench
// ---------------------------------------------------------------------------
module sd_card_tb;
    localparam real SD_NS = 33.328;    // sd_clk half period in ns (15 MHz @ 120MHz sysclk)

    reg clk = 0;
    reg aux_clk = 1;
    reg reset = 1;
    always #4.166 clk = ~clk;
    always #4.166 aux_clk = ~aux_clk;

    reg sd_clk = 1;
    reg host_cmd = 1;

    wire sd_cmd_out, sd_cmd_oe, sd_dat0_out, sd_dat0_oe;
    wire ram_activate, ram_read;
    wire [22:0] ram_addr;
    wire [63:0] read_buffer;
    wire read_busy;

    reg [1:0] access_cmd = 0;
    reg [24:0] access_addr = 0;
    reg [63:0] write_buffer = 0;
    wire cmd_busy;

    wire cmd_line = sd_cmd_oe ? sd_cmd_out : 1'b1;

    sd_card_1bit dut (
        .clk(clk), .reset(reset), .sd_reset(1'b0),
        .sd_clk(sd_clk), .sd_cmd_in(host_cmd),
        .sd_cmd_out(sd_cmd_out), .sd_cmd_oe(sd_cmd_oe),
        .sd_dat0_out(sd_dat0_out), .sd_dat0_oe(sd_dat0_oe),
        .image_sectors(32'd65536),   // 32 MB card
        .ram_activate(ram_activate), .ram_read(ram_read),
        .ram_addr(ram_addr),
        .ram_read_buffer(read_buffer), .ram_read_busy(read_busy),
        .log_cmd_valid(), .log_cmd_opcode(),
        .log_addr_valid(), .log_addr_out(), .log_byte_count()
    );

    wire [1:0] ba_o;
    wire [12:0] a_o;
    wire cs_o, ras_o, cas_o, we_o;
    wire [1:0] dqm_o;
    wire [15:0] dq;

    sdram #(.CLK_FREQ_MHZ(120), .BURST_LEN(4)) ctrl (
        .clk(clk), .aux_clk(aux_clk), .reset(reset),
        .ba_o(ba_o), .a_o(a_o), .cs_o(cs_o), .ras_o(ras_o),
        .cas_o(cas_o), .we_o(we_o), .dqm_o(dqm_o), .dq_io(dq),
        .spi_inhibit_refresh(1'b0),
        .spi_cmd_activate(ram_activate),
        .spi_cmd_read(ram_read),
        .spi_addr(ram_addr),
        .access_cmd(access_cmd), .access_addr(access_addr),
        .inhibit_refresh(1'b0), .cmd_busy(cmd_busy),
        .read_buffer(read_buffer), .read_busy(read_busy),
        .write_buffer(write_buffer)
    );

    sdram_model chip (
        .clk(clk), .cs(cs_o), .ras(ras_o), .cas(cas_o), .we(we_o),
        .ba(ba_o), .a(a_o), .dq(dq)
    );

    // ---------------- monitors ----------------
    integer errors = 0;
    integer act_count = 0, rd_count = 0, busy_pulses = 0;
    reg sd_phase = 0;
    reg busy_prev = 0;
    always @(posedge clk) begin
        if (sd_phase) begin
            if (ras_o == 0 && cas_o == 1 && we_o == 1) act_count <= act_count + 1;
            if (ras_o == 1 && cas_o == 0 && we_o == 1) rd_count <= rd_count + 1;
            if (read_busy && !busy_prev) busy_pulses <= busy_pulses + 1;
            busy_prev <= read_busy;
        end
    end

    task check(input cond, input [1023:0] msg);
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL: %0s", msg);
        end
    endtask

    // ---------------- CRC helpers ----------------
    function [6:0] crc7_next(input [6:0] crc, input b);
        reg inv;
        begin
            inv = b ^ crc[6];
            crc7_next = {crc[5:3], crc[2] ^ inv, crc[1:0], inv};
        end
    endfunction
    function [6:0] crc7_40(input [39:0] d);
        integer i;
        reg [6:0] c;
        begin
            c = 0;
            for (i = 39; i >= 0; i = i - 1) c = crc7_next(c, d[i]);
            crc7_40 = c;
        end
    endfunction
    function [15:0] crc16_next(input [15:0] crc, input b);
        reg inv;
        begin
            inv = b ^ crc[15];
            crc16_next = {crc[14:0], 1'b0} ^ (inv ? 16'h1021 : 16'h0);
        end
    endfunction

    // image pattern: byte at absolute address a
    function [7:0] img_byte(input [31:0] a);
        img_byte = a[7:0] ^ a[15:8] ^ 8'hA5;
    endfunction

    // ---------------- serial-path SDRAM access ----------------
    task sdram_wait;
        begin
            @(posedge clk); @(posedge clk);
            while (cmd_busy) @(posedge clk);
        end
    endtask

    task sdram_activate(input [22:0] burst);
        begin
            @(negedge clk);
            access_addr = {burst, 2'b00};
            access_cmd = 2'b11;
            @(negedge clk);
            access_cmd = 2'b00;
            sdram_wait;
        end
    endtask

    task sdram_write_burst(input [22:0] burst, input [63:0] data);
        begin
            write_buffer = data;
            @(negedge clk);
            access_addr = {burst, 2'b00};
            access_cmd = 2'b10;
            @(negedge clk);
            access_cmd = 2'b00;
            sdram_wait;
        end
    endtask

    task sdram_read_burst(input [22:0] burst);
        begin
            @(negedge clk);
            access_addr = {burst, 2'b00};
            access_cmd = 2'b01;
            @(negedge clk);
            access_cmd = 2'b00;
            sdram_wait;
        end
    endtask

    // ---------------- SD host model ----------------
    task sd_bit(input b);
        begin
            sd_clk = 0; host_cmd = b; #(SD_NS);
            sd_clk = 1;               #(SD_NS);
        end
    endtask

    task sd_send(input [5:0] idx, input [31:0] arg);
        integer i;
        reg [6:0] c;
        reg [39:0] f;
        begin
            f = {2'b01, idx, arg};
            c = crc7_40(f);
            sd_bit(0); sd_bit(1);
            for (i = 5; i >= 0; i = i - 1) sd_bit(idx[i]);
            for (i = 31; i >= 0; i = i - 1) sd_bit(arg[i]);
            for (i = 6; i >= 0; i = i - 1) sd_bit(c[i]);
            sd_bit(1);
            host_cmd = 1;
        end
    endtask

    // one clock with CMD sampled mid-high-half
    reg sample_v;
    task sd_sample_cmd;
        begin
            sd_clk = 0; #(SD_NS);
            sd_clk = 1; #(SD_NS/2);
            sample_v = cmd_line;
            #(SD_NS/2);
        end
    endtask
    task sd_sample_dat;
        begin
            sd_clk = 0; #(SD_NS);
            sd_clk = 1; #(SD_NS/2);
            sample_v = sd_dat0_out;
            #(SD_NS/2);
        end
    endtask

    reg [47:0] resp;
    reg [135:0] lresp;
    integer i, j;

    // read a 48-bit response; timeout=1 if none arrives
    task sd_read_r1(output timeout);
        integer t;
        begin
            timeout = 0;
            t = 0;
            while (t < 32) begin
                sd_sample_cmd;
                if (sample_v == 0) t = 100; else t = t + 1;
            end
            if (t != 100) begin timeout = 1; end
            else begin
                resp = 48'd0;
                for (i = 46; i >= 0; i = i - 1) begin
                    sd_sample_cmd;
                    resp[i] = sample_v;
                end
                if (resp[0] != 1) begin
                    errors = errors + 1;
                    $display("FAIL: response end bit not 1");
                end
                if (resp[7:1] != crc7_40(resp[47:8])) begin
                    errors = errors + 1;
                    $display("FAIL: response CRC7 mismatch: got %02x want %02x",
                             resp[7:1], crc7_40(resp[47:8]));
                end
            end
        end
    endtask

    task sd_read_r2;
        integer t;
        begin
            t = 0;
            while (t < 32) begin
                sd_sample_cmd;
                if (sample_v == 0) t = 100; else t = t + 1;
            end
            if (t != 100) begin
                errors = errors + 1;
                $display("FAIL: no R2 response");
            end else begin
                lresp = 136'd0;
                for (i = 134; i >= 0; i = i - 1) begin
                    sd_sample_cmd;
                    lresp[i] = sample_v;
                end
            end
        end
    endtask

    // read one data block of nbytes into blk[], check CRC16; timeout=1 on no start bit
    reg [7:0] blk [0:511];
    task sd_read_block(input integer nbytes, output timeout);
        integer t;
        reg [15:0] c;
        reg [15:0] got_crc;
        begin
            timeout = 0;
            t = 0;
            while (t < 2000) begin
                sd_sample_dat;
                if (sample_v == 0) t = 20000; else t = t + 1;
            end
            if (t != 20000) begin
                timeout = 1;
                errors = errors + 1;
                $display("FAIL: no data start bit");
            end else begin
                c = 0;
                for (i = 0; i < nbytes; i = i + 1) begin
                    blk[i] = 8'd0;
                    for (j = 7; j >= 0; j = j - 1) begin
                        sd_sample_dat;
                        blk[i][j] = sample_v;
                        c = crc16_next(c, sample_v);
                    end
                end
                got_crc = 0;
                for (i = 15; i >= 0; i = i - 1) begin
                    sd_sample_dat;
                    got_crc[i] = sample_v;
                end
                if (got_crc != c) begin
                    errors = errors + 1;
                    $display("FAIL: data CRC16 mismatch: got %04x want %04x", got_crc, c);
                end
                sd_sample_dat;   // end bit
                if (sample_v != 1) begin
                    errors = errors + 1;
                    $display("FAIL: data end bit not 1");
                end
            end
        end
    endtask

    // ---------------- image load via serial path ----------------
    task load_image;
        integer k;
        reg [63:0] pat;
        begin
            $display("--- loading 8 sectors via serial path ---");
            for (k = 0; k < 8 * 64; k = k + 1) begin
                pat = {img_byte(k*8+7), img_byte(k*8+6), img_byte(k*8+5), img_byte(k*8+4),
                       img_byte(k*8+3), img_byte(k*8+2), img_byte(k*8+1), img_byte(k*8+0)};
                sdram_activate(k[22:0]);
                sdram_write_burst(k[22:0], pat);
                sdram_activate(k[22:0]);
                sdram_read_burst(k[22:0]);
                if (read_buffer !== pat) begin
                    errors = errors + 1;
                    $display("FAIL: serial readback burst %0d: got %016x want %016x",
                             k, read_buffer, pat);
                end
            end
            $display("--- image loaded and verified ---");
        end
    endtask

    task check_block(input integer sector);
        begin
            for (i = 0; i < 512; i = i + 1) begin
                if (blk[i] !== img_byte(sector * 512 + i)) begin
                    errors = errors + 1;
                    $display("FAIL: sector %0d byte %0d: got %02x want %02x",
                             sector, i, blk[i], img_byte(sector * 512 + i));
                    i = 512;
                end
            end
        end
    endtask

    reg timeout_flag;

    // ---------------- test sequence ----------------
    initial begin
        #100;
        reset = 0;
        #140000;   // sdram init (100us) + margin
        sd_phase = 1;

        load_image;

        // 1. CMD0: expect NO response
        $display("--- CMD0 (expect silence) ---");
        sd_send(0, 0);
        timeout_flag = 0;
        for (i = 0; i < 24; i = i + 1) begin
            sd_sample_cmd;
            if (sample_v == 0) timeout_flag = 1;
        end
        check(timeout_flag == 0, "CMD0 produced a response (spec: none)");

        // 2. CMD8
        $display("--- CMD8 ---");
        sd_send(8, 32'h000001AA);
        sd_read_r1(timeout_flag);
        check(timeout_flag == 0, "CMD8 no response");
        check(resp[45:40] == 8, "CMD8 response index wrong");
        check(resp[39:8] == 32'h000001AA, "CMD8 echo wrong");

        // 3. ACMD41
        $display("--- ACMD41 ---");
        sd_send(55, 0);
        sd_read_r1(timeout_flag);
        sd_send(41, 32'h40300000);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'hc0ff8000, "ACMD41 OCR wrong");

        // 4. CMD2 (CID)
        $display("--- CMD2 ---");
        sd_send(2, 0);
        sd_read_r2;
        check(lresp[127:120] == 8'h1d, "CID MID wrong");

        // 5. CMD3
        $display("--- CMD3 ---");
        sd_send(3, 0);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'h00010000, "CMD3 RCA response wrong");

        // 6. CMD9 (CSD): C_SIZE must be 63 (32MB)
        $display("--- CMD9 ---");
        sd_send(9, 0);
        sd_read_r2;
        check(lresp[127:120] == 8'h40, "CSD structure byte wrong");
        check({lresp[69:48]} == 22'd63, "CSD C_SIZE wrong (expect 63 for 32MB)");

        // 7. CMD7 select
        $display("--- CMD7 ---");
        sd_send(7, 32'h00010000);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'h00000900, "CMD7 status wrong");

        // 8. ACMD51 (SCR)
        $display("--- ACMD51 ---");
        sd_send(55, 0);
        sd_read_r1(timeout_flag);
        sd_send(51, 0);
        sd_read_r1(timeout_flag);
        sd_read_block(8, timeout_flag);
        check(blk[0] == 8'h02, "SCR byte0 wrong");
        check(blk[1] == 8'h01, "SCR byte1 must advertise 1-bit only");

        // 9. CMD13
        $display("--- CMD13 ---");
        sd_send(13, 0);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'h00000900, "CMD13 status wrong");

        // 10. CMD16
        $display("--- CMD16 ---");
        sd_send(16, 512);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'h00000900, "CMD16 status wrong");

        // 11. CMD17 single block read, data compare + handshake counts
        $display("--- CMD17 sector 2 ---");
        act_count = 0; rd_count = 0; busy_pulses = 0;
        sd_send(17, 2);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'h00000900, "CMD17 status wrong");
        sd_read_block(512, timeout_flag);
        check_block(2);
        check(rd_count == 64, "CMD17 must issue exactly 64 SDRAM READ commands");
        check(busy_pulses == 64, "CMD17 must pulse read_busy exactly 64 times");

        // 12. CMD17 out of range
        $display("--- CMD17 out of range ---");
        sd_send(17, 32'd65536);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'h80000000, "CMD17 out-of-range must set bit31");

        // 13. CMD18 multi-block: read 3 blocks, CMD12 during 4th
        $display("--- CMD18 sectors 1..3 + CMD12 ---");
        sd_send(18, 1);
        sd_read_r1(timeout_flag);
        check(resp[39:8] == 32'h00000900, "CMD18 status wrong");
        sd_read_block(512, timeout_flag);
        check_block(1);
        sd_read_block(512, timeout_flag);
        check_block(2);
        sd_read_block(512, timeout_flag);
        check_block(3);
        // 4th block starts; let a few bits stream, then CMD12
        for (i = 0; i < 32; i = i + 1) sd_sample_dat;
        sd_send(12, 0);
        sd_read_r1(timeout_flag);
        check(resp[45:40] == 12, "CMD12 response index wrong");
        check(resp[39:8] == 32'h00000900, "CMD12 status wrong");
        // DAT0 must be idle now
        timeout_flag = 0;
        for (i = 0; i < 100; i = i + 1) begin
            sd_sample_dat;
            if (sample_v == 0) timeout_flag = 1;
        end
        check(timeout_flag == 0, "DAT0 not idle after CMD12 stop");

        // 14. recovery: CMD17 again after the abort
        $display("--- CMD17 sector 0 after CMD12 abort ---");
        sd_send(17, 0);
        sd_read_r1(timeout_flag);
        sd_read_block(512, timeout_flag);
        check_block(0);

        #(2000);
        if (errors == 0) $display("TEST PASSED");
        else $display("TEST FAILED: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #50000000;
        $display("TEST FAILED: global timeout");
        $finish;
    end
endmodule
