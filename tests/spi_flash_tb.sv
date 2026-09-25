`timescale 1ns/1ps

// Real SPI and host protocol engines, with a burst-level SDRAM model.
module spi_flash_tb;
    reg clk = 0;
    always #4.167 clk = ~clk; // 120 MHz
    reg reset = 1, sck = 0, cs = 1, mosi = 0;
    reg rx_strobe = 0;
    reg [7:0] rx_data = 0;
    wire running, miso, miso_oe, tx_strobe;
    wire [7:0] tx_data;
    reg check_early_start = 0, early_start_ack = 0;
    wire spi_reset = reset || !running;
    wire write_cmd, write_done, write_strobe;
    wire [1:0] write_type;
    wire [22:0] write_addr, write_len;
    wire [7:0] write_offset, write_value;
    wire [6:0] sfdp_addr;
    wire [7:0] sfdp_data;
    wire [1:0] access;
    wire [24:0] access_addr;
    wire [63:0] write_buffer;
    reg [63:0] read_buffer = 0;
    reg busy = 0;
    integer busy_cycles = 0;
    reg [63:0] memory [0:1023];
    reg [7:0] expected [0:8191];
    // 10-bit model index: (row 0, bank, burst-in-row).
    wire [9:0] mem_index = {access_addr[11], access_addr[10:9], access_addr[8:2]};

    spi_trx spi (
        .clk(clk), .spi_clk(sck), .spi_reset(spi_reset), .spi_csel(cs),
        .spi_io0_in(mosi), .spi_io1_in(1'b0), .spi_io2_in(1'b1), .spi_io3_in(1'b1),
        .spi_io1_out(miso), .spi_io1_oe(miso_oe),
        .ram_read_buffer(read_buffer), .ram_read_busy(busy),
        .write_cmd(write_cmd), .write_type(write_type), .write_addr(write_addr),
        .write_len(write_len), .write_done(write_done),
        .write_buf_strobe(write_strobe), .write_buf_offset(write_offset),
        .write_buf_val(write_value), .cfg_jedec_id(24'h4125bf), .cfg_4byte(1'b0),
        .cfg_chip_erase_bursts(23'h7fffff), .sfdp_raddr(sfdp_addr), .sfdp_rdata(sfdp_data)
    );
    glue dut (
        .clk(clk), .reset(reset), .rxd_strobe(rx_strobe), .rxd_data(rx_data),
        .txd_ready(1'b1), .txd_strobe(tx_strobe), .txd_data(tx_data),
        .ft_rx_data_available(1'b0), .ft_rx_data(8'b0), .ft_txd_ready(1'b1),
        .sdram_access_cmd(access), .sdram_access_addr(access_addr), .sdram_cmd_busy(busy),
        .sdram_read_busy(1'b0), .sdram_read_buffer(read_buffer), .sdram_write_buffer(write_buffer),
        .spi_reset(spi_reset), .spi_csel(cs), .spi_cmd_write(write_cmd),
        .spi_write_type(write_type), .spi_write_addr(write_addr), .spi_write_len(write_len),
        .spi_write_done(write_done), .spi_write_buf_strobe(write_strobe),
        .spi_write_buf_offset(write_offset), .spi_write_buf_val(write_value),
        .log_strobe(1'b0), .log_val(8'b0), .spi_clk(sck),
        .sfdp_raddr(sfdp_addr), .sfdp_rdata(sfdp_data), .spi_running(running),
        .log_fifo_data_available(1'b0), .log_fifo_read_data(8'b0),
        .log_addr_valid_sync(1'b0), .log_addr_sync(24'b0), .spi_active_sync(1'b0)
    );

    always @(posedge clk) begin
        if (check_early_start && tx_strobe) begin
            if (!running || tx_data !== 8'h01) $fatal(1, "START acknowledged before SPI was ready");
            early_start_ack <= 1;
        end
    end

    // Model only the glue-facing interface, not SDRAM electrical timings.
    // sdram_controller_tb separately exercises the real controller.
    always @(posedge clk) begin
        if (busy_cycles > 0) begin
            busy_cycles <= busy_cycles - 1;
            if (busy_cycles == 1) busy <= 0;
        end
        if (access != 0) begin
            if (busy) $fatal(1, "SDRAM request while busy");
            // The model flattens (row 0, bank, column) into 8 KiB, so a
            // second row would alias onto the first. The guard keeps tests
            // in row 0 rather than letting a truncated index make two
            // addresses share a byte.
            if (access_addr[24:12] != 0)
                $fatal(1, "SDRAM access 0x%0h is outside the test model", access_addr);
            busy <= 1;
            busy_cycles <= access == 3 ? 3 : 10;
            if (access == 1) read_buffer <= memory[mem_index];
            if (access == 2) memory[mem_index] <= write_buffer;
        end
    end

    task host_byte(input [7:0] b);
        @(negedge clk); rx_data = b; rx_strobe = 1;
        @(negedge clk); rx_strobe = 0;
        repeat (10) @(negedge clk); // One byte per 12 system clocks
    endtask

    task configure_sfdp(input integer length, input [7:0] seed,
                        input integer payload_count = -1);
        host_byte(8'h33);
        host_byte(8'hbf); host_byte(8'h25); host_byte(8'h41);
        host_byte(0); host_byte(0); host_byte(8'hff); host_byte(8'hff);
        host_byte(8'(length));
        for (integer i = 0; i < (payload_count < 0 ? length : payload_count); i++)
            host_byte(8'(i) ^ seed);
    endtask

    task spi_byte(input [7:0] b, output [7:0] result);
        for (integer bit_index = 7; bit_index >= 0; bit_index--) begin
            mosi = b[bit_index];
            #16.667; result[bit_index] = miso; sck = 1;
            #16.667; sck = 0;
        end
    endtask

    task send(input [7:0] b);
        reg [7:0] unused_byte;
        spi_byte(b, unused_byte);
    endtask

    task select_spi;
        cs = 0; #35;
    endtask

    task deselect_spi;
        #35; cs = 1; #70;
    endtask

    task command(input [7:0] opcode);
        select_spi; send(opcode); deselect_spi;
    endtask

    task address(input integer a);
        send(8'(a >> 16)); send(8'(a >> 8)); send(8'(a));
    endtask

    task poll_done;
        reg [7:0] status;
        integer polls;
        polls = 0;
        do begin
            select_spi; send(8'h05); spi_byte(0, status); deselect_spi;
            polls++;
            if (polls > 100) $fatal(1, "WIP failed to clear");
        end while (status[0]);
    endtask

    task check_memory;
        for (integer i = 0; i < 8192; i++)
            if (memory[i/8][(i%8)*8 +: 8] !== expected[i])
                $fatal(1, "Memory[%h]: got %h expected %h", i,
                       memory[i/8][(i%8)*8 +: 8], expected[i]);
    endtask

    task page_program(input integer a, input integer length, input [7:0] seed);
        reg [7:0] page [0:255];
        reg [255:0] valid;
        reg [7:0] b;
        valid = 0;
        command(8'h06);
        select_spi; send(8'h02); address(a);
        for (integer i = 0; i < length; i++) begin
            b = 8'(i * 17) ^ seed;
            send(b);
            // On page wrap, the last received value wins in the buffer.
            page[(a+i)%256] = b;
            valid[(a+i)%256] = 1;
        end
        deselect_spi; poll_done;
        for (integer i = 0; i < 256; i++)
            if (valid[i]) expected[(a & ~255)+i] &= page[i];
        check_memory;
    endtask

    task aai_word(input bit first, input integer a, input [7:0] lo,
                  input [7:0] hi, input integer extra_bytes);
        select_spi; send(8'had);
        if (first) address(a);
        send(lo); send(hi);
        for (integer i = 0; i < extra_bytes; i++) send(8'h00);
        deselect_spi; poll_done;
        expected[a & ~1] &= lo;
        expected[(a & ~1)+1] &= hi;
        check_memory;
    endtask

    task read_sfdp(input integer start, input integer count,
                   input integer length, input [7:0] seed);
        reg [7:0] actual, wanted;
        select_spi; send(8'h5a); address(start); send(0);
        for (integer i = 0; i < count; i++) begin
            spi_byte(0, actual);
            wanted = (start+i)%128 < length ? 8'((start+i)%128) ^ seed : 8'hff;
            if (!miso_oe || actual !== wanted)
                $fatal(1, "SFDP start=%0d byte=%0d: got %h expected %h", start, i, actual, wanted);
        end
        deselect_spi;
    endtask

    initial begin
        for (integer i = 0; i < 1024; i++) memory[i] = '1;
        for (integer i = 0; i < 8192; i++) expected[i] = '1;
        repeat (4) @(negedge clk); reset = 0;

        // Deliberately configure before a 128-cycle RAM scrub could finish.
        configure_sfdp(1, 8'ha5);
        check_early_start = 1;
        host_byte(8'h34);
        if (running) $fatal(1, "Early START enabled SPI before page-buffer initialization");
        wait (early_start_ack); check_early_start = 0;
        read_sfdp(0, 260, 1, 8'ha5);

        host_byte(8'h35); configure_sfdp(128, 8'ha5); host_byte(8'h34);
        for (integer start = 0; start < 128; start++) read_sfdp(start, 260, 128, 8'ha5);
        // Reconfiguration must not expose a previous longer table.
        host_byte(8'h35); configure_sfdp(3, 8'h39); host_byte(8'h34);
        read_sfdp(0, 130, 3, 8'h39);
        host_byte(8'h35); configure_sfdp(0, 0); host_byte(8'h34);
        read_sfdp(0, 130, 0, 0);
        host_byte(8'h35); configure_sfdp(128, 8'h84, 2);
        repeat (70000) @(negedge clk); // Incomplete host command times out
        host_byte(8'h34); read_sfdp(0, 130, 2, 8'h84);

        page_program('h1ff, 2, 8'ha5);  // Partial page wrapping at byte 255
        page_program('h106, 1, 8'h55);  // Preserve untouched bytes; NOR AND
        page_program('h2f5, 300, 8'h63);
        // Odd A0 must be ignored at every burst offset, especially offset 7.
        for (integer a = 'h300; a < 'h308; a++) begin
            command(8'h06); aai_word(1, a, 8'h12, 8'h34, 0); command(8'h04);
        end
        command(8'h06);
        aai_word(1, 'h7ff, 8'h61, 8'hb2, 8); // Extra bytes must not set flags
        for (integer a = 'h800; a < 'h904; a += 2) aai_word(0, a, 8'h61, 8'hb2, 0);
        command(8'h04);
        command(8'h06);
        select_spi; send(8'had); address('h706); send(8'ha5); deselect_spi; poll_done;
        expected['h706] &= 8'ha5; check_memory;
        aai_word(0, 'h706, 8'h12, 8'h34, 0); // Short prior word cannot misalign this one
        command(8'h04);
        page_program('h600, 1, 8'h56); // No stale AAI bytes may follow this PP
        page_program('ha00, 0, 0);    // Empty PP must also preserve all bytes

        // Reset invalidates SFDP, even though the BSRAM itself retains data.
        host_byte(8'h35); configure_sfdp(128, 8'h72);
        @(negedge clk); reset = 1;
        repeat (4) @(negedge clk); reset = 0;
        host_byte(8'h34); wait (running);
        read_sfdp(0, 130, 0, 0);
        $display("PASS SPI: SFDP/startup/reconfigure/reset, PP/wrap/AND, AAI/alignment/bounds/cleanup");
        $finish;
    end

    initial begin
        #50000000;
        $fatal(1, "SPI test timeout");
    end
endmodule
