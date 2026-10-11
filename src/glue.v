// Composition of independent host protocol and SPI program/RMW clients.
// SDRAM client ownership and acceptance routing live in sdram_client_mux.
`default_nettype none
module glue(
    input wire clk, reset,
    input wire rxd_strobe,
    input wire [7:0] rxd_data,
    input wire txd_ready,
    output wire txd_strobe,
    output wire [7:0] txd_data,
    input wire ft_rx_data_available,
    input wire [7:0] ft_rx_data,
    output wire ft_rx_pop,
    input wire ft_txd_ready,
    output wire ft_txd_strobe,
    output wire [7:0] ft_txd_data,
    output wire [1:0] sdram_access_cmd,
    output wire [24:0] sdram_access_addr,
    output wire sdram_inhibit_refresh,
    input wire sdram_cmd_busy, sdram_access_accept,
    input wire [63:0] sdram_read_buffer,
    input wire sdram_read_busy,
    output wire [63:0] sdram_write_buffer,
    input wire spi_reset, spi_csel, spi_cmd_write,
    input wire [1:0] spi_write_type,
    input wire [22:0] spi_write_addr, spi_write_len,
    input wire spi_write_partial,
    output wire spi_write_done,
    input wire spi_write_buf_strobe,
    input wire [7:0] spi_write_buf_offset, spi_write_buf_val,
    output wire [23:0] cfg_jedec_id,
    output wire cfg_4byte,
    output wire [22:0] cfg_chip_erase_bursts,
    output wire [459:0] cfg_profile,
    output wire cfg_erase_wren,
    output wire [4:0] cfg_erase_waddr,
    output wire [49:0] cfg_erase_wdata,
    output wire cfg_profile_valid,
    output wire [31:0] cfg_profile_epoch,
    input wire spi_clk,
    input wire [6:0] sfdp_raddr,
    output wire [7:0] sfdp_rdata,
    output wire spi_running, hold_out, log_active,
    input wire log_fifo_data_available,
    input wire [7:0] log_fifo_read_data,
    output wire log_fifo_read_strobe,
    input wire log_addr_valid_sync,
    input wire [23:0] log_addr_sync,
    input wire spi_active_sync, prefetch_underrun, prefetch_thin,
    output wire redirect_active, trap_check_pending,
    output wire [22:0] redirect_mask, redirect_base,
    output wire trap_notify_strobe,
    output wire [1:0] trap_notify_index,
    output wire [23:0] trap_notify_addr,
    output wire [7:0] led
);
    wire spi_writing, pp_init_done, pp_init_finishing, program_allowed;
    // STOP cancels commands that have not crossed CS. Already accepted
    // array work drains first; serial_gate excludes it from host ownership.
    // Re-arm only after observing the producer's command low, so a stale
    // SPI-domain level cannot be replayed when START releases reset.
    reg stop_seen, cancel_pending, program_armed;
    reg [1:0] program_command_sync;
    wire cancel_program = ((spi_reset && !stop_seen) || cancel_pending) && !spi_writing;
    always @(posedge clk) begin
        if (reset) begin
            stop_seen <= 1; cancel_pending <= 0; program_armed <= 0;
            program_command_sync <= 0;
        end else begin
            stop_seen <= spi_reset;
            program_command_sync <= {program_command_sync[0], spi_cmd_write};
            if (spi_reset && !stop_seen) cancel_pending <= 1;
            if (cancel_program) cancel_pending <= 0;
            if (spi_reset || cancel_pending || cancel_program) program_armed <= 0;
            else if (!program_command_sync[1] && !spi_cmd_write) program_armed <= 1;
        end
    end
    wire [3:0] trap_triggered; // Observable TOCTOU state for integration benches
    wire [1:0] host_cmd, program_cmd;
    wire [24:0] host_addr, program_addr;
    wire [63:0] host_data, program_data;
    wire host_inhibit, program_inhibit, host_accept, program_accept;
    wire busy = sdram_access_cmd != 0 || sdram_cmd_busy || sdram_read_busy;

    host_protocol host_i(
        .clk(clk), .reset(reset), .rxd_strobe(rxd_strobe), .rxd_data(rxd_data),
        .txd_ready(txd_ready), .txd_strobe(txd_strobe), .txd_data(txd_data),
        .ft_rx_data_available(ft_rx_data_available), .ft_rx_data(ft_rx_data),
        .ft_rx_pop(ft_rx_pop), .ft_txd_ready(ft_txd_ready),
        .ft_txd_strobe(ft_txd_strobe), .ft_txd_data(ft_txd_data),
        .sdram_access_cmd(host_cmd), .sdram_access_addr(host_addr),
        .sdram_write_buffer(host_data), .sdram_inhibit_refresh(host_inhibit),
        .sdram_access_accept(host_accept), .sdram_cmd_busy(sdram_cmd_busy),
        .sdram_read_busy(sdram_read_busy), .sdram_read_buffer(sdram_read_buffer),
        .spi_reset(spi_reset), .spi_csel(spi_csel), .spi_writing(spi_writing),
        .pp_init_done(pp_init_done), .pp_init_finishing(pp_init_finishing),
        .program_allowed(program_allowed), .trap_triggered(trap_triggered),
        .cfg_jedec_id(cfg_jedec_id), .cfg_4byte(cfg_4byte),
        .cfg_chip_erase_bursts(cfg_chip_erase_bursts), .spi_clk(spi_clk),
        .cfg_profile(cfg_profile), .cfg_profile_valid(cfg_profile_valid),
        .cfg_profile_epoch(cfg_profile_epoch),
        .cfg_erase_wren(cfg_erase_wren), .cfg_erase_waddr(cfg_erase_waddr),
        .cfg_erase_wdata(cfg_erase_wdata),
        .sfdp_raddr(sfdp_raddr), .sfdp_rdata(sfdp_rdata), .spi_running(spi_running),
        .hold_out(hold_out), .log_active(log_active),
        .log_fifo_data_available(log_fifo_data_available), .log_fifo_read_data(log_fifo_read_data),
        .log_fifo_read_strobe(log_fifo_read_strobe), .log_addr_valid_sync(log_addr_valid_sync),
        .log_addr_sync(log_addr_sync), .spi_active_sync(spi_active_sync),
        .prefetch_underrun(prefetch_underrun), .prefetch_thin(prefetch_thin),
        .redirect_active(redirect_active), .trap_check_pending(trap_check_pending),
        .redirect_mask(redirect_mask), .redirect_base(redirect_base),
        .trap_notify_strobe(trap_notify_strobe), .trap_notify_index(trap_notify_index),
        .trap_notify_addr(trap_notify_addr), .led(led));
    spi_program program_i(
        .clk(clk), .reset(reset || cancel_program), .deselected(program_allowed),
        .command(spi_cmd_write && program_armed && !spi_reset && !cancel_pending), .command_type(spi_write_type),
        .command_addr(spi_write_addr), .command_len(spi_write_len),
        .command_partial(spi_write_partial),
        .byte_strobe(spi_write_buf_strobe && program_armed && !spi_reset && !cancel_pending), .byte_offset(spi_write_buf_offset),
        .byte_value(spi_write_buf_val), .busy(busy), .accept(program_accept),
        .read_data(sdram_read_buffer), .writing(spi_writing), .done(spi_write_done),
        .init_done(pp_init_done), .init_finishing(pp_init_finishing),
        .inhibit(program_inhibit), .access_cmd(program_cmd),
        .access_addr(program_addr), .write_data(program_data));
    sdram_client_mux mux_i(
        .program_owner(spi_writing), .accept(sdram_access_accept),
        .host_cmd(host_cmd), .host_addr(host_addr), .host_data(host_data),
        .program_cmd(program_cmd), .program_addr(program_addr), .program_data(program_data),
        .host_inhibit(host_inhibit), .program_inhibit(program_inhibit),
        .access_cmd(sdram_access_cmd), .access_addr(sdram_access_addr),
        .write_data(sdram_write_buffer), .inhibit(sdram_inhibit_refresh),
        .host_accept(host_accept), .program_accept(program_accept));
endmodule
