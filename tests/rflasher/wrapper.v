// Real protocol and SPI engines; only external SDRAM is modeled by bridge.cpp.
`default_nettype none
module wrapper(
    input wire clk, reset,
    input wire host_strobe,
    input wire [7:0] host_data,
    output wire host_tx_strobe,
    output wire [7:0] host_tx_data,
    input wire sck, cs, mosi, wp_n, hold_n,
    output wire miso, miso_oe,
    output wire running,
    output wire [1:0] access_cmd,
    output wire [24:0] access_addr,
    output wire [63:0] write_data,
    input wire access_accept, cmd_busy, read_busy,
    input wire [63:0] read_data,
    output wire ram_read, ram_post_toggle, ram_pair,
    output wire [22:0] ram_addr, ram_addr2,
    input wire [63:0] ram_a, ram_b,
    input wire valid_a, valid_b,
    input wire [3:0] beats_a, beats_b,
    output wire underrun, thin
);
    wire spi_reset = reset || !running;
    wire command, done, byte_strobe, partial;
    wire [1:0] kind;
    wire [22:0] address, length, erase_bursts;
    wire [7:0] byte_offset, byte_value;
    wire [23:0] jedec;
    wire fourbyte, profile_valid;
    wire [459:0] profile;
    wire erase_wren;
    wire [4:0] erase_waddr;
    wire [49:0] erase_wdata;
    wire [31:0] profile_epoch;
    wire [6:0] sfdp_addr;
    wire [7:0] sfdp_data;
    glue glue_i(
        .clk(clk), .reset(reset), .rxd_strobe(host_strobe), .rxd_data(host_data),
        .txd_ready(1'b1), .txd_strobe(host_tx_strobe), .txd_data(host_tx_data),
        .ft_rx_data_available(1'b0), .ft_rx_data(8'b0), .ft_txd_ready(1'b1),
        .sdram_access_cmd(access_cmd), .sdram_access_addr(access_addr),
        .sdram_access_accept(access_accept), .sdram_cmd_busy(cmd_busy),
        .sdram_read_buffer(read_data), .sdram_read_busy(read_busy), .sdram_write_buffer(write_data),
        .spi_reset(spi_reset), .spi_csel(cs), .spi_clk(sck),
        .spi_cmd_write(command), .spi_write_type(kind), .spi_write_addr(address), .spi_write_len(length),
        .spi_write_partial(partial), .spi_write_done(done), .spi_write_buf_strobe(byte_strobe),
        .spi_write_buf_offset(byte_offset), .spi_write_buf_val(byte_value),
        .cfg_erase_wren(erase_wren), .cfg_erase_waddr(erase_waddr), .cfg_erase_wdata(erase_wdata),
        .cfg_jedec_id(jedec), .cfg_4byte(fourbyte), .cfg_chip_erase_bursts(erase_bursts),
        .cfg_profile(profile), .cfg_profile_valid(profile_valid), .cfg_profile_epoch(profile_epoch),
        .sfdp_raddr(sfdp_addr), .sfdp_rdata(sfdp_data), .spi_running(running),
        .log_fifo_data_available(1'b0), .log_fifo_read_data(8'b0),
        .log_addr_valid_sync(1'b0), .log_addr_sync(24'b0), .spi_active_sync(!cs),
        .prefetch_underrun(underrun), .prefetch_thin(thin));
    spi_trx spi_i(
        .clk(clk), .spi_clk(sck), .spi_reset(spi_reset), .spi_csel(cs),
        .spi_io0_in(mosi), .spi_io1_in(1'b0), .spi_io2_in(wp_n), .spi_io3_in(hold_n),
        .spi_io1_out(miso), .spi_io1_oe(miso_oe),
        .ram_read(ram_read), .ram_post_toggle(ram_post_toggle), .ram_addr(ram_addr),
        .ram_pair(ram_pair), .ram_addr2(ram_addr2),
        .ram_read_buffer(ram_a), .ram_read_buffer_b(ram_b),
        .ram_read_valid_a(valid_a), .ram_read_valid_b(valid_b),
        .ram_read_beats_a(beats_a), .ram_read_beats_b(beats_b), .ram_read_busy(1'b0),
        .prefetch_underrun(underrun), .prefetch_thin(thin),
        .write_cmd(command), .write_type(kind), .write_addr(address), .write_len(length), .write_done(done),
        .write_partial(partial),
        .write_buf_strobe(byte_strobe), .write_buf_offset(byte_offset), .write_buf_val(byte_value),
        .cfg_erase_wren(erase_wren), .cfg_erase_waddr(erase_waddr), .cfg_erase_wdata(erase_wdata),
        .cfg_jedec_id(jedec), .cfg_4byte(fourbyte), .cfg_chip_erase_bursts(erase_bursts),
        .cfg_profile(profile), .cfg_profile_valid(profile_valid), .cfg_profile_epoch(profile_epoch),
        .sfdp_raddr(sfdp_addr), .sfdp_rdata(sfdp_data));
endmodule
