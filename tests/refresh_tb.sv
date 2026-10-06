`timescale 1ns/1ps

// Refresh must be independent of host TX progress and external SCK progress.
module refresh_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;
    reg reset = 1, rx_strobe = 0, tx_ready = 0;
    reg [7:0] rx_data = 0;
    wire tx_strobe;
    wire [7:0] tx_data;
    wire [1:0] access;
    wire [24:0] address;
    wire busy, read_busy, inhibit;
    wire [63:0] read_buffer, write_buffer;
    reg spi_active = 0, spi_inhibit = 0, spi_activate = 0, spi_read = 0;
    wire ras, cas, we, chip;
    wire [1:0] bank;
    wire [15:0] dq;
    assign dq = 16'ha55a;

    glue host(.clk(clk), .reset(reset), .rxd_strobe(rx_strobe), .rxd_data(rx_data),
        .txd_ready(tx_ready), .txd_strobe(tx_strobe), .txd_data(tx_data),
        .ft_rx_data_available(1'b0), .ft_rx_data(8'b0), .ft_txd_ready(1'b0),
        .sdram_access_cmd(access), .sdram_access_addr(address), .sdram_cmd_busy(busy),
        .sdram_read_busy(read_busy), .sdram_inhibit_refresh(inhibit),
        .sdram_read_buffer(read_buffer), .sdram_write_buffer(write_buffer),
        .spi_reset(1'b1), .spi_csel(1'b1), .spi_cmd_write(1'b0), .spi_write_type(2'b0),
        .spi_write_addr(23'b0), .spi_write_len(23'b0), .spi_write_buf_strobe(1'b0),
        .spi_write_buf_offset(8'b0), .spi_write_buf_val(8'b0),
        .spi_clk(clk), .sfdp_raddr(7'b0), .log_fifo_data_available(1'b0),
        .log_fifo_read_data(8'b0), .log_addr_valid_sync(1'b0), .log_addr_sync(24'b0),
        .spi_active_sync(1'b0), .prefetch_underrun(1'b0), .prefetch_thin(1'b0));
    sdram #(.CLK_FREQ_MHZ(120)) ram(.clk(clk), .aux_clk(clk), .reset(reset), .dq_io(dq),
        .spi_active(spi_active), .spi_inhibit_refresh(spi_inhibit),
        .spi_cmd_activate(spi_activate), .spi_cmd_read(spi_read),
        .spi_cmd_post_toggle(1'b0), .spi_addr(23'h001234),
        .access_cmd(access), .access_addr(address), .inhibit_refresh(inhibit),
        .cmd_busy(busy), .read_buffer(read_buffer), .read_busy(read_busy),
        .write_buffer(write_buffer), .ras_o(ras), .cas_o(cas), .we_o(we),
        .cs_o(chip), .ba_o(bank));

    integer cycles = 0, refreshes = 0, last_refresh [0:1];
    integer replies = 0, activates = 0, reads = 0;
    bit checking = 0;
    bit row_open [0:1][0:3];
    always @(negedge clk) begin
        cycles++;
        if (checking) begin
            // Auto-precharged accesses complete before the next dispatch.
            case ({ras, cas, we})
                3'b011: begin
                    if (row_open[chip][bank]) $fatal(1, "ACTIVATE to open bank");
                    row_open[chip][bank] = 1;
                    activates++;
                end
                3'b101, 3'b100: begin
                    if (!row_open[chip][bank]) $fatal(1, "Access without ACTIVATE");
                    row_open[chip][bank] = 0;
                    reads++;
                end
                3'b010: row_open[chip][bank] = 0;
                3'b001: begin
                    for (integer b = 0; b < 4; b++)
                        if (row_open[chip][b]) $fatal(1, "REFRESH with open bank");
                    if (cycles - last_refresh[chip] > 1100)
                        $fatal(1, "refresh deadline missed: %0d clocks", cycles-last_refresh[chip]);
                    last_refresh[chip] = cycles;
                    refreshes++;
                end
                default: ;
            endcase
            for (integer c = 0; c < 2; c++)
                if (cycles - last_refresh[c] > 1100) $fatal(1, "refresh starvation");
        end
    end
    always @(posedge clk) if (tx_strobe) begin
        if (tx_data !== (replies % 2 == 0 ? 8'h5a : 8'ha5))
            $fatal(1, "RAMREAD changed across refresh: %h", tx_data);
        replies++;
    end
    task tick(input integer n); repeat(n) @(negedge clk); endtask
    task host_byte(input [7:0] b);
        @(negedge clk); rx_data = b; rx_strobe = 1;
        @(negedge clk); rx_strobe = 0; tick(10);
    endtask
    initial begin
        for (integer c=0; c<2; c++) begin
            last_refresh[c] = 0;
            for (integer b=0; b<4; b++) row_open[c][b] = 0;
        end
        tick(4); reset = 0; tick(14000);
        last_refresh[0] = cycles; last_refresh[1] = cycles; checking = 1;
        host_byte(8'h31); host_byte(0); host_byte(0); host_byte(0); host_byte(0); host_byte(2);
        tick(50000); // >400us of TX backpressure, with a burst held for TX
        if (replies != 0 || refreshes < 100) $fatal(1, "stalled host test failed");
        tx_ready = 1; tick(500);
        if (replies != 16) $fatal(1, "RAMREAD failed to resume: %0d bytes", replies);
        spi_active = 1; spi_inhibit = 1; tick(5000); // SCK stopped before ACTIVATE
        spi_activate = 1; tick(5000); // SCK stopped with a row open
        if (activates < 4) $fatal(1, "held ACTIVATE was not replayed after refresh");
        spi_read = 1; tick(100);
        if (reads != 3) $fatal(1, "resumed SPI READ was lost or duplicated: %0d", reads);
        spi_active = 0; spi_activate = 0; spi_read = 0; tick(1200);
        $display("PASS REFRESH: TX stalls, stopped SCK, row closure/replay, resumed data (%0d refreshes)", refreshes);
        $finish;
    end
    initial begin #2000000; $fatal(1, "refresh test timeout"); end
endmodule
