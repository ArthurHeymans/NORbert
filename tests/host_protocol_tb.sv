`timescale 1ns/1ps

// Host transport arbitration and parser robustness (glue + host_protocol).
//
// - Commands that never touch SDRAM (HOLDCTL, LOGCTL, TOCTOU) are processed
//   over either transport while the SPI master holds CS low or the program
//   engine owns SDRAM; their bytes are never dropped.
// - SDRAM commands wait for the gate without losing bytes, UART included.
// - A command's bytes come only from the port that started it; the other
//   port's bytes wait until it ends.
// - A RAMWRITE payload survives a USB stall longer than the header idle
//   timeout, so payload bytes are never reinterpreted as commands.
module host_protocol_tb;
    reg clk = 0;
    always #4.167 clk = ~clk;
    reg reset = 1, uart_strobe = 0, cs = 1, program_command = 0;
    reg [7:0] uart_data = 0;
    wire uart_tx_strobe, ft_tx_strobe, ft_pop, hold, log_active, running;
    wire [7:0] uart_tx_data, ft_tx_data;
    wire [1:0] access;
    wire [24:0] access_addr;
    wire [63:0] write_buffer;
    wire write_done;
    reg [63:0] read_buffer = 0;
    reg busy = 0, accept = 0;
    integer busy_cycles = 0;
    reg [63:0] memory [0:255];

    reg [7:0] ft_queue [0:255];
    integer ft_head = 0, ft_tail = 0;
    wire ft_available = ft_head != ft_tail;

    glue dut(.clk(clk), .reset(reset), .rxd_strobe(uart_strobe), .rxd_data(uart_data),
        .txd_ready(1'b1), .txd_strobe(uart_tx_strobe), .txd_data(uart_tx_data),
        .ft_rx_data_available(ft_available), .ft_rx_data(ft_queue[ft_head % 256]),
        .ft_rx_pop(ft_pop), .ft_txd_ready(1'b1), .ft_txd_strobe(ft_tx_strobe),
        .ft_txd_data(ft_tx_data),
        .sdram_access_cmd(access), .sdram_access_addr(access_addr), .sdram_cmd_busy(busy),
        .sdram_access_accept(accept), .sdram_read_busy(1'b0),
        .sdram_read_buffer(read_buffer), .sdram_write_buffer(write_buffer),
        .spi_reset(!running), .spi_csel(cs), .spi_cmd_write(program_command),
        .spi_write_type(2'd1), .spi_write_addr(23'h4000), .spi_write_len(23'd400),
        .spi_write_done(write_done), .spi_write_buf_strobe(1'b0),
        .spi_write_buf_offset(8'b0), .spi_write_buf_val(8'b0), .spi_clk(clk),
        .sfdp_raddr(7'b0), .spi_running(running), .hold_out(hold), .log_active(log_active),
        .log_fifo_data_available(1'b0), .log_fifo_read_data(8'b0),
        .log_addr_valid_sync(1'b0), .log_addr_sync(24'b0), .spi_active_sync(1'b0),
        .prefetch_underrun(1'b0), .prefetch_thin(1'b0));

    // SDRAM model: 256 bursts of row 0, bank 0; everything else is ignored.
    always @(posedge clk) begin
        accept <= 0;
        if (busy_cycles > 0) begin
            busy_cycles <= busy_cycles - 1;
            if (busy_cycles == 1) busy <= 0;
        end
        if (access != 0 && !busy && !accept) begin
            accept <= 1; busy <= 1; busy_cycles <= access == 3 ? 3 : 10;
            if (access_addr[24:10] == 0) begin
                if (access == 1) read_buffer <= memory[access_addr[9:2]];
                if (access == 2) memory[access_addr[9:2]] <= write_buffer;
            end
        end
    end

    always @(posedge clk) if (ft_pop) ft_head <= ft_head + 1;

    // Replies per port, in order.
    reg [7:0] uart_replies [$];
    reg [7:0] ft_replies [$];
    always @(posedge clk) begin
        if (uart_tx_strobe) uart_replies.push_back(uart_tx_data);
        if (ft_tx_strobe) ft_replies.push_back(ft_tx_data);
    end

    task tick(input integer n); repeat (n) @(negedge clk); endtask

    // 2 Mbaud: one byte every 600 system clocks.
    task uart_byte(input [7:0] b);
        @(negedge clk); uart_data = b; uart_strobe = 1;
        @(negedge clk); uart_strobe = 0;
        tick(598);
    endtask

    task ft_byte(input [7:0] b);
        @(negedge clk); ft_queue[ft_tail % 256] = b; ft_tail = ft_tail + 1;
    endtask

    task automatic expect_reply(ref reg [7:0] q [$], input [7:0] want, input string what);
        integer waited = 0;
        while (q.size() == 0 && waited < 20000) begin tick(1); waited++; end
        if (q.size() == 0) $fatal(1, "%s: no reply", what);
        if (q[0] !== want) $fatal(1, "%s: reply %h, expected %h", what, q[0], want);
        void'(q.pop_front());
    endtask

    task automatic expect_quiet(ref reg [7:0] q [$], input string what);
        if (q.size() != 0) $fatal(1, "%s: unexpected reply %h", what, q[0]);
    endtask

    initial begin
        tick(4); reset = 0; tick(400);

        // Emulation running; the SPI master holds CS low throughout.
        uart_byte(8'h34); expect_reply(uart_replies, 8'h01, "START");
        cs = 0;

        // 1. Ungated commands over UART with CS low.
        uart_byte(8'h37); uart_byte(8'h01);
        expect_reply(uart_replies, 8'h01, "UART HOLDCTL with CS low");
        if (!hold) $fatal(1, "HOLDCTL argument dropped while CS low");
        uart_byte(8'h38); uart_byte(8'h01);
        expect_reply(uart_replies, 8'h01, "UART LOGCTL with CS low");
        if (!log_active) $fatal(1, "LOGCTL argument dropped while CS low");
        // A TOCTOU SET (12 bytes) and ARM over UART.
        begin
            reg [7:0] set_cmd [0:11];
            set_cmd = '{8'h39, 8'h01, 8'h02, 8'h00, 8'h12, 8'h00,
                        8'hff, 8'hf0, 8'h00, 8'h00, 8'h34, 8'h00};
            foreach (set_cmd[i]) uart_byte(set_cmd[i]);
        end
        expect_reply(uart_replies, 8'h01, "UART TOCTOU SET with CS low");
        if (dut.host_i.trap_start[2] !== 24'h001000 || dut.host_i.trap_mask[2] !== 24'hfff000 ||
            dut.host_i.trap_replace[2] !== 24'h003400)
            $fatal(1, "TOCTOU SET arguments corrupted with CS low");

        // 2. The same over FT245, back to back, still with CS low.
        ft_byte(8'h37); ft_byte(8'h00); ft_byte(8'h38); ft_byte(8'h00);
        expect_reply(ft_replies, 8'h01, "FT245 HOLDCTL with CS low");
        expect_reply(ft_replies, 8'h01, "FT245 LOGCTL with CS low");
        if (hold || log_active) $fatal(1, "FT245 arguments dropped while CS low");

        // 3. Ungated commands while the program engine owns SDRAM. CS goes
        //    high so the queued erase (400 bursts) can start.
        cs = 1;
        program_command = 1;
        while (!dut.spi_writing) tick(1);
        uart_byte(8'h37); uart_byte(8'h01);
        expect_reply(uart_replies, 8'h01, "UART HOLDCTL during program");
        if (!dut.spi_writing) $fatal(1, "program finished too early for coverage");

        // 4. Port isolation: an FT245 TOCTOU SET in progress must not take
        //    UART bytes; the UART VERSION is answered after it, on UART.
        ft_byte(8'h39); ft_byte(8'h01); ft_byte(8'h01);
        ft_byte(8'h00); ft_byte(8'h00); ft_byte(8'h00);
        tick(20);
        uart_byte(8'h30);  // arrives mid-command
        expect_quiet(uart_replies, "VERSION during another port's command");
        ft_byte(8'h00); ft_byte(8'h00); ft_byte(8'h00);
        ft_byte(8'h00); ft_byte(8'h56); ft_byte(8'h00);
        expect_reply(ft_replies, 8'h01, "FT245 TOCTOU SET across a UART byte");
        if (dut.host_i.trap_replace[1] !== 24'h005600)
            $fatal(1, "UART byte leaked into the FT245 command");
        expect_reply(uart_replies, 8'h07, "queued UART VERSION");

        // Wait for the program, then stop emulation for SDRAM commands.
        while (dut.spi_writing) tick(1);
        program_command = 0;
        uart_byte(8'h35); expect_reply(uart_replies, 8'h01, "STOP");

        // 5. RAMWRITE over UART with a 1 ms stall inside the payload, far
        //    beyond the 546 us header timeout. The block must land intact
        //    and no payload byte may be taken as a command (0x37 = HOLDCTL).
        begin
            reg hold_before;
            hold_before = hold;
            uart_byte(8'h32); uart_byte(0); uart_byte(0); uart_byte(8'h05);
            uart_byte(0); uart_byte(2);
            for (integer i = 0; i < 16; i++) begin
                uart_byte(i == 8 ? 8'h37 : 8'(8'h80 + i));
                if (i == 7) tick(120000);
            end
            expect_reply(uart_replies, 8'h01, "RAMWRITE across a payload stall");
            if (hold !== hold_before) $fatal(1, "payload byte was parsed as HOLDCTL");
            if (memory[5] !== 64'h8786858483828180 || memory[6] !== 64'h8f8e8d8c8b8a8937)
                $fatal(1, "stalled RAMWRITE stored %h %h", memory[5], memory[6]);
        end

        // 6. A gated command waits behind a program without losing bytes:
        //    queue RAMREAD over UART while the engine owns SDRAM.
        program_command = 1;
        while (!dut.spi_writing) tick(1);
        uart_byte(8'h31); uart_byte(0); uart_byte(0); uart_byte(8'h05);
        uart_byte(0); uart_byte(1);
        if (!dut.spi_writing) $fatal(1, "program finished too early for coverage");
        for (integer i = 0; i < 8; i++)
            expect_reply(uart_replies, 8'(8'h80 + i), "RAMREAD queued behind a program");

        $display("PASS HOST_PROTOCOL: ungated commands with CS low/program, no dropped UART bytes, port isolation, payload stall");
        $finish;
    end

    initial begin #60000000; $fatal(1, "host protocol test timeout"); end
endmodule
