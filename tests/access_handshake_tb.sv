`timescale 1ns/1ps

// Model a controller that looks idle but arbitrates a request later.
// The caller must retain its request/payload until explicit acceptance.
module access_handshake_tb;
    reg clk=0; always #4.167 clk=~clk;
    reg reset=1, rx_strobe=0, accept=0, busy=0;
    reg [7:0] rx_data=0;
    wire [1:0] request;
    wire [24:0] address;
    wire tx_strobe;
    wire [7:0] tx_data;
    localparam [63:0] DATA=64'h876543210fedcba9;
    glue dut(.clk(clk), .reset(reset), .rxd_strobe(rx_strobe), .rxd_data(rx_data),
        .txd_ready(1'b1), .txd_strobe(tx_strobe), .txd_data(tx_data),
        .ft_rx_data_available(1'b0), .ft_rx_data(8'b0), .ft_txd_ready(1'b1),
        .sdram_access_cmd(request), .sdram_access_addr(address), .sdram_cmd_busy(busy),
        .sdram_access_accept(accept), .sdram_read_busy(1'b0), .sdram_read_buffer(DATA),
        .spi_reset(1'b1), .spi_csel(1'b1), .spi_cmd_write(1'b0), .spi_write_type(2'b0),
        .spi_write_addr(23'b0), .spi_write_len(23'b0), .spi_write_buf_strobe(1'b0),
        .spi_write_buf_offset(8'b0), .spi_write_buf_val(8'b0), .spi_clk(clk),
        .sfdp_raddr(7'b0), .log_fifo_data_available(1'b0), .log_fifo_read_data(8'b0),
        .log_addr_valid_sync(1'b0), .log_addr_sync(24'b0), .spi_active_sync(1'b0),
        .prefetch_underrun(1'b0), .prefetch_thin(1'b0));
    integer wait_cycles=0, busy_cycles=0, grants=0, replies=0;
    reg [1:0] held_cmd=0;
    reg [24:0] held_address=0;
    always @(posedge clk) begin
        accept <= 0;
        if (!reset) begin
            if (busy_cycles!=0) begin
                busy_cycles <= busy_cycles-1;
                if (busy_cycles==1) busy <= 0;
            end else if (request!=0 && !accept) begin
                if (wait_cycles==0) begin
                    held_cmd <= request; held_address <= address;
                    wait_cycles <= 1;
                end else begin
                    if (request!==held_cmd || address!==held_address)
                        $fatal(1,"pending request/payload changed before acceptance");
                    if (wait_cycles==25+grants*12) begin
                        if (request !== (grants==0?2'd3:2'd1)) $fatal(1,"wrong request order");
                        accept <= 1; grants++; busy <= 1; busy_cycles <= 8; wait_cycles <= 0;
                    end else wait_cycles <= wait_cycles+1;
                end
            end else if (wait_cycles!=0 && request==0)
                $fatal(1,"request disappeared without acceptance");
            if (tx_strobe) begin
                if (grants!=2 || tx_data!==DATA[replies*8 +: 8])
                    $fatal(1,"data returned before acceptance or changed: %h",tx_data);
                replies++;
            end
        end
    end
    task byte_host(input [7:0] b);
        @(negedge clk);rx_data=b;rx_strobe=1;
        @(negedge clk);rx_strobe=0;repeat(10) @(negedge clk);
    endtask
    initial begin
        repeat(4) @(negedge clk);reset=0;repeat(300) @(negedge clk);
        byte_host(8'h31);byte_host(0);byte_host(0);byte_host(8'h12);byte_host(0);byte_host(1);
        repeat(500) @(negedge clk);
        if(grants!=2 || replies!=8) $fatal(1,"handshake failed: grants=%0d bytes=%0d",grants,replies);
        $display("PASS ACCESS_HANDSHAKE: deferred grants, held command/address and ordered data");
        $finish;
    end
endmodule
