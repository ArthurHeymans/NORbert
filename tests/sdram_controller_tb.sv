`timescale 1ns/1ps

// Exercise real glue/SDRAM command timing, refresh and write completion.
// Data semantics are covered by spi_flash_tb's memory model.
module sdram_controller_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;
    reg reset = 1, cs = 1, command = 0, strobe = 0;
    reg [1:0] kind = 0;
    reg [22:0] address = 0, length = 0;
    reg [7:0] offset = 0, value = 0;
    wire done, busy, read_busy, inhibit;
    wire [1:0] access;
    wire [24:0] access_addr;
    wire [63:0] write_buffer, read_buffer;
    wire ras, cas, we;
    wire [15:0] dq;
    integer cycles = 0, requests = 0, accepted = 0, writes = 0;
    integer refreshes = 0, last_refresh = 0, max_refresh_gap = 0;
    reg [1:0] previous_access = 0;
    reg checking = 0;

    glue dut (
        .clk(clk), .reset(reset), .rxd_strobe(1'b0), .rxd_data(8'b0), .txd_ready(1'b1),
        .ft_rx_data_available(1'b0), .ft_rx_data(8'b0), .ft_txd_ready(1'b1),
        .sdram_access_cmd(access), .sdram_access_addr(access_addr), .sdram_cmd_busy(busy),
        .sdram_read_busy(read_busy), .sdram_inhibit_refresh(inhibit),
        .sdram_read_buffer(read_buffer), .sdram_write_buffer(write_buffer),
        .spi_reset(1'b0), .spi_csel(cs), .spi_cmd_write(command), .spi_write_type(kind),
        .spi_write_addr(address), .spi_write_len(length), .spi_write_done(done),
        .spi_write_buf_strobe(strobe), .spi_write_buf_offset(offset), .spi_write_buf_val(value),
        .log_strobe(1'b0), .log_val(8'b0), .spi_clk(clk), .sfdp_raddr(7'b0),
        .log_fifo_data_available(1'b0), .log_fifo_read_data(8'b0),
        .log_addr_valid_sync(1'b0), .log_addr_sync(24'b0), .spi_active_sync(1'b0)
    );
    sdram ram (
        .clk(clk), .aux_clk(clk), .reset(reset), .dq_io(dq),
        .spi_active(1'b0), .spi_inhibit_refresh(1'b0), .spi_cmd_activate(1'b0),
        .spi_cmd_read(1'b0), .spi_addr(23'b0), .access_cmd(access), .access_addr(access_addr),
        .inhibit_refresh(inhibit), .cmd_busy(busy), .read_buffer(read_buffer),
        .read_busy(read_busy), .write_buffer(write_buffer), .ras_o(ras), .cas_o(cas), .we_o(we)
    );

    always @(posedge clk) begin
        cycles <= cycles + 1;
        previous_access <= access;
        if (checking && access != 0) requests <= requests + 1;
    end
    always @(negedge clk) begin
        if (checking) begin
            if (!ras && !cas && we) begin
                refreshes++;
                if (last_refresh != 0 && cycles-last_refresh > max_refresh_gap)
                    max_refresh_gap = cycles-last_refresh;
                last_refresh = cycles;
            end
            case (previous_access)
                3: begin
                    if (!(!ras && cas && we)) $fatal(1, "ACTIVATE was not accepted");
                    accepted++;
                end
                1: begin
                    if (!(ras && !cas && we)) $fatal(1, "READ was not accepted");
                    accepted++;
                end
                2: begin
                    if (!(ras && !cas && !we)) $fatal(1, "WRITE was not accepted");
                    accepted++; writes++;
                end
                default: ;
            endcase
        end
    end

    task tick(input integer count);
        repeat (count) @(negedge clk);
    endtask

    task program_burst(input bit aai, input integer a);
        reg old_done;
        integer start_cycle, elapsed;
        cs = 0; command = 0; tick(5);
        kind = aai ? 2 : 0; address = 23'(a); length = aai ? 0 : 31;
        command = 1; tick(5);
        for (integer i = 0; i < 2; i++) begin
            offset = 8'((a%32)*8+i); value = 8'h39; strobe = 1;
            tick(2); strobe = 0; tick(6);
        end
        old_done = done; cs = 1; start_cycle = cycles;
        while (done == old_done) begin
            tick(1);
            if (cycles-start_cycle > 2000) $fatal(1, "Program completion timeout");
        end
        elapsed = cycles-start_cycle;
        // Include refresh variation, but not serialized BSRAM merge latency.
        if (elapsed > (aai ? 60 : 1225)) $fatal(1, "Program latency regressed: %0d clocks", elapsed);
        command = 0; tick(6);
    endtask

    initial begin
        tick(4); reset = 0; tick(14000); checking = 1;
        for (integer i = 0; i < 5; i++) program_burst(0, i*32);
        for (integer i = 0; i < 130; i++) program_burst(1, 192+i);
        tick(600);
        if (requests != accepted || writes != 290 || refreshes == 0 || max_refresh_gap > 600)
            $fatal(1, "Controller: requests=%0d accepted=%0d writes=%0d refreshes=%0d gap=%0d",
                   requests, accepted, writes, refreshes, max_refresh_gap);
        $display("PASS SDRAM: %0d commands, %0d writes, bounded program latency and refresh", accepted, writes);
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "Controller test timeout");
    end
endmodule
