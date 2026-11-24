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
    reg uart_tx_ready = 1, ft_tx_ready = 1;
    reg [7:0] uart_data = 0;
    reg [6:0] sfdp_raddr = 0;
    reg log_addr_valid = 0, spi_active = 0;
    reg [23:0] log_addr = 0;
    reg prefetch_underrun_in = 0, prefetch_thin_in = 0;
    wire uart_tx_strobe, ft_tx_strobe, ft_pop, hold, log_active, running;
    wire [7:0] uart_tx_data, ft_tx_data, sfdp_rdata;
    wire [23:0] cfg_jedec_id;
    wire cfg_4byte, cfg_profile_valid;
    wire [22:0] cfg_chip_erase_bursts;
    wire log_fifo_pop, redirect_active;
    wire [22:0] redirect_base;
    wire [1:0] access;
    wire [24:0] access_addr;
    wire [63:0] write_buffer;
    wire write_done;
    reg [63:0] read_buffer = 0;
    reg busy = 0, accept = 0, stall_sdram = 0;
    integer busy_cycles = 0;
    reg [63:0] memory [0:255];

    localparam integer FT_Q_SIZE = 1024;
    reg [7:0] ft_queue [0:FT_Q_SIZE-1];
    integer ft_head = 0, ft_tail = 0;
    wire ft_available = ft_head != ft_tail;

    localparam integer LOG_Q_SIZE = 1024;
    reg [7:0] log_queue [0:LOG_Q_SIZE-1];
    integer log_head = 0, log_tail = 0;
    wire log_available = log_head != log_tail;

    glue dut(.clk(clk), .reset(reset), .rxd_strobe(uart_strobe), .rxd_data(uart_data),
        .txd_ready(uart_tx_ready), .txd_strobe(uart_tx_strobe), .txd_data(uart_tx_data),
        .ft_rx_data_available(ft_available), .ft_rx_data(ft_queue[ft_head % FT_Q_SIZE]),
        .ft_rx_pop(ft_pop), .ft_txd_ready(ft_tx_ready), .ft_txd_strobe(ft_tx_strobe),
        .ft_txd_data(ft_tx_data),
        .sdram_access_cmd(access), .sdram_access_addr(access_addr), .sdram_cmd_busy(busy),
        .sdram_access_accept(accept), .sdram_read_busy(1'b0),
        .sdram_read_buffer(read_buffer), .sdram_write_buffer(write_buffer),
        .spi_reset(!running), .spi_csel(cs), .spi_cmd_write(program_command),
        .spi_write_type(2'd1), .spi_write_addr(23'h4000), .spi_write_len(23'd400),
        .spi_write_partial(1'b0),
        .spi_write_done(write_done), .spi_write_buf_strobe(1'b0),
        .spi_write_buf_offset(8'b0), .spi_write_buf_val(8'b0),
        .cfg_jedec_id(cfg_jedec_id), .cfg_4byte(cfg_4byte),
        .cfg_chip_erase_bursts(cfg_chip_erase_bursts),
        .cfg_profile_valid(cfg_profile_valid),
        .spi_clk(clk), .sfdp_raddr(sfdp_raddr), .sfdp_rdata(sfdp_rdata),
        .spi_running(running), .hold_out(hold), .log_active(log_active),
        .log_fifo_data_available(log_available),
        .log_fifo_read_data(log_queue[log_head % LOG_Q_SIZE]),
        .log_fifo_read_strobe(log_fifo_pop),
        .log_addr_valid_sync(log_addr_valid), .log_addr_sync(log_addr),
        .spi_active_sync(spi_active),
        .prefetch_underrun(prefetch_underrun_in), .prefetch_thin(prefetch_thin_in),
        .redirect_active(redirect_active), .redirect_base(redirect_base));

    // SDRAM model: 256 bursts of row 0, bank 0; everything else is ignored.
    always @(posedge clk) begin
        accept <= 0;
        if (busy_cycles > 0) begin
            busy_cycles <= busy_cycles - 1;
            if (busy_cycles == 1) busy <= 0;
        end
        if (access != 0 && !busy && !accept && !stall_sdram) begin
            accept <= 1; busy <= 1; busy_cycles <= access == 3 ? 3 : 10;
            if (access_addr[24:10] == 0) begin
                if (access == 1) read_buffer <= memory[access_addr[9:2]];
                if (access == 2) memory[access_addr[9:2]] <= write_buffer;
            end
        end
    end

    always @(posedge clk) begin
        if (ft_pop) ft_head <= ft_head + 1;
        if (log_fifo_pop) log_head <= log_head + 1;
    end

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
        @(negedge clk); ft_queue[ft_tail % FT_Q_SIZE] = b; ft_tail = ft_tail + 1;
    endtask

    task ft_push_now(input [7:0] b);
        ft_queue[ft_tail % FT_Q_SIZE] = b; ft_tail = ft_tail + 1;
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
        expect_reply(uart_replies, 8'h09, "queued UART VERSION");

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

        // 6. STOP drains accepted work before ACK; an SDRAM command on
        //    the other port waits behind it without losing any bytes.
        uart_byte(8'h34); expect_reply(uart_replies, 8'h01, "restart for program");
        program_command = 1;
        while (!dut.spi_writing) tick(1);
        uart_byte(8'h35);
        if (!dut.spi_writing) $fatal(1, "program finished too early for coverage");
        expect_quiet(uart_replies, "STOP before program drain");
        ft_byte(8'h31); ft_byte(0); ft_byte(0); ft_byte(8'h05);
        ft_byte(0); ft_byte(1);
        tick(20);
        expect_quiet(ft_replies, "RAMREAD before STOP drain");
        expect_reply(uart_replies, 8'h01, "STOP after program drain");
        program_command = 0;
        if (dut.spi_writing) $fatal(1, "STOP ACK preceded program drain");
        for (integer i = 0; i < 8; i++)
            expect_reply(ft_replies, 8'(8'h80 + i), "RAMREAD queued behind STOP/program");

        // 7. Prefilled multi-burst RAMWRITE with stalled SDRAM and stalled
        //    final TX ACK, immediately followed by a pipelined VERSION.
        //    None of the bursts may overwrite each other and VERSION must
        //    not be swallowed as payload.
        begin
            stall_sdram = 1;
            ft_tx_ready = 0;
            ft_push_now(8'h32); ft_push_now(0); ft_push_now(0); ft_push_now(8'h10);
            ft_push_now(0); ft_push_now(3);
            for (integer i = 0; i < 24; i++)
                ft_push_now(8'(8'h10 + i));
            ft_push_now(8'h30); // pipelined VERSION
            tick(120);
            stall_sdram = 0;
            tick(200);
            // Final burst retired, waiting on ft_tx_ready: VERSION must still wait.
            expect_quiet(ft_replies, "RAMWRITE final ACK while TX stalled");
            ft_tx_ready = 1;
            expect_reply(ft_replies, 8'h01, "prefilled stalled RAMWRITE ACK");
            expect_reply(ft_replies, 8'h09, "pipelined VERSION after RAMWRITE");
            if (memory[16] !== 64'h1716151413121110 ||
                memory[17] !== 64'h1f1e1d1c1b1a1918 ||
                memory[18] !== 64'h2726252423222120)
                $fatal(1, "prefilled RAMWRITE corrupted: %h %h %h",
                       memory[16], memory[17], memory[18]);
        end

        // 8. Zero-length RAMREAD and RAMWRITE rejected cleanly without
        //    wrapping to 65536 bursts or consuming a following VERSION.
        begin
            reg [63:0] mem5_before;
            mem5_before = memory[5];
            ft_push_now(8'h31); ft_push_now(0); ft_push_now(0); ft_push_now(8'h05);
            ft_push_now(0); ft_push_now(0);
            ft_push_now(8'h30); // pipelined VERSION
            expect_reply(ft_replies, 8'h09, "zero-length RAMREAD emits no data before VERSION");

            ft_push_now(8'h32); ft_push_now(0); ft_push_now(0); ft_push_now(8'h05);
            ft_push_now(0); ft_push_now(0);
            ft_push_now(8'h30); // pipelined VERSION
            expect_reply(ft_replies, 8'h02, "zero-length RAMWRITE rejected");
            expect_reply(ft_replies, 8'h09, "pipelined VERSION after zero-length RAMWRITE");
            if (memory[5] !== mem5_before) $fatal(1, "zero-length RAMWRITE modified SDRAM");
        end

        // 9. Unknown TOCTOU subcommand is rejected with 0x02 and does not
        //    fall through into TOCTOU_SET or clobber trap registers.
        begin
            reg [23:0] start2_before, mask2_before, repl2_before;
            start2_before = dut.host_i.trap_start[2];
            mask2_before  = dut.host_i.trap_mask[2];
            repl2_before  = dut.host_i.trap_replace[2];
            ft_push_now(8'h39); ft_push_now(8'h99);
            ft_push_now(8'h30); // pipelined VERSION
            expect_reply(ft_replies, 8'h02, "unknown TOCTOU subcommand rejected");
            expect_reply(ft_replies, 8'h09, "pipelined VERSION after unknown TOCTOU");
            if (dut.host_i.trap_start[2] !== start2_before ||
                dut.host_i.trap_mask[2]  !== mask2_before ||
                dut.host_i.trap_replace[2] !== repl2_before)
                $fatal(1, "unknown TOCTOU subcommand modified trap registers");
        end

        // 10. CHIPCONFIG SFDP length boundary & malformed framing recovery:
        //     - 128-byte SFDP with 1 ms mid-payload stall succeeds and stores
        //       all 128 bytes cleanly.
        //     - Lengths 129, 247, 248, 255 drain without wrapping in_count,
        //       do not execute payload opcodes or overwrite cfg_jedec_id,
        //       reply 0x02, leave START blocked, and preserve following VERSION.
        begin
            reg hold_before;
            integer bad_lens [0:3];
            bad_lens = '{129, 247, 248, 255};

            // Valid 128-byte SFDP with a 1 ms mid-payload stall.
            ft_byte(8'h33);
            ft_byte(8'hef); ft_byte(8'h40); ft_byte(8'h17); // JEDEC EF 40 17
            ft_byte(8'h00); // flags (3-byte)
            ft_byte(8'h0f); ft_byte(8'hff); ft_byte(8'hff); // 8 MiB erase bursts
            ft_byte(8'd128);
            for (integer i = 0; i < 128; i++) begin
                ft_byte(8'(i ^ 8'h5a));
                if (i == 63) tick(120000);
            end
            ft_push_now(8'h30);
            expect_reply(ft_replies, 8'h01, "CHIPCONFIG 128-byte SFDP with stall");
            expect_reply(ft_replies, 8'h09, "VERSION after 128-byte CHIPCONFIG");
            if (cfg_jedec_id !== 24'h1740ef)
                $fatal(1, "valid CHIPCONFIG JEDEC %h", cfg_jedec_id);
            sfdp_raddr = 7'd0; tick(2);
            if (sfdp_rdata !== 8'h5a) $fatal(1, "SFDP[0] = %h", sfdp_rdata);
            sfdp_raddr = 7'd127; tick(2);
            if (sfdp_rdata !== 8'(8'd127 ^ 8'h5a)) $fatal(1, "SFDP[127] = %h", sfdp_rdata);

            // Malformed lengths 129, 247, 248, 255 with opcode bytes inside payload.
            hold_before = hold;
            foreach (bad_lens[k]) begin
                ft_push_now(8'h33);
                ft_push_now(8'h11); ft_push_now(8'h22); ft_push_now(8'h33);
                ft_push_now(8'h01);
                ft_push_now(8'h00); ft_push_now(8'h00); ft_push_now(8'h01);
                ft_push_now(8'(bad_lens[k]));
                for (integer i = 0; i < bad_lens[k]; i++)
                    ft_push_now(i[0] ? 8'h30 : 8'h37);
                ft_push_now(8'h30); // pipelined VERSION after frame
                expect_reply(ft_replies, 8'h02, "malformed CHIPCONFIG rejected");
                expect_reply(ft_replies, 8'h09, "VERSION after malformed CHIPCONFIG");
                if (cfg_jedec_id !== 24'h1740ef)
                    $fatal(1, "malformed CHIPCONFIG (%0d) overwrote JEDEC to %h",
                           bad_lens[k], cfg_jedec_id);
                if (hold !== hold_before)
                    $fatal(1, "malformed CHIPCONFIG (%0d) executed payload as HOLDCTL", bad_lens[k]);
                // START must stay blocked after rejected CHIPCONFIG.
                ft_push_now(8'h34);
                expect_reply(ft_replies, 8'h02, "START blocked after rejected CHIPCONFIG");
            end
        end

        // 11. CHIPPROFILE: 1 ms mid-payload stall survives; after byte 128,
        //     TX stall > 1 ms must neither time out validation nor swallow
        //     a pipelined VERSION sitting in the RX FIFO.
        begin
            reg [7:0] prof [0:127];
            for (integer i = 0; i < 128; i++) prof[i] = 8'h00;
            prof[0]  = 8'h02; // format version 2
            prof[1]  = 8'h00; // generic status model
            prof[24] = 8'h03; // 3-byte addressing
            prof[28] = 8'h08; // page shift 8 (256B)
            // Descriptor 0: opcode 0x20, 1 region: 4 KiB (shift 12) * 2048 = 8 MiB
            prof[32] = 8'h20;
            prof[34] = 8'h01;
            prof[36] = 8'd12;
            prof[37] = 8'h00; prof[38] = 8'h08; prof[39] = 8'h00;

            ft_byte(8'h3c);
            for (integer i = 0; i < 127; i++) begin
                ft_byte(prof[i]);
                if (i == 63) tick(120000); // 1 ms mid-payload stall
            end
            // Stall TX before final byte and pipeline VERSION right behind byte 128.
            ft_tx_ready = 0;
            ft_push_now(prof[127]);
            ft_push_now(8'h30); // pipelined VERSION
            tick(120000); // > 546 us header timeout while validating / waiting for TX
            expect_quiet(ft_replies, "CHIPPROFILE ACK while TX stalled");
            ft_tx_ready = 1;
            expect_reply(ft_replies, 8'h01, "CHIPPROFILE accepted after TX stall");
            if (!cfg_profile_valid) $fatal(1, "CHIPPROFILE did not publish valid profile");
            expect_reply(ft_replies, 8'h09, "pipelined VERSION after CHIPPROFILE");
        end

        // 12. Immediate replies and PREFETCH read-and-clear are retained while
        //     TX is stalled on both UART and FT245, without losing replies or
        //     double-reporting faults.
        begin
            // Trigger prefetch underrun edge.
            prefetch_underrun_in = 1; tick(4); prefetch_underrun_in = 0; tick(4);
            uart_tx_ready = 0;
            uart_byte(8'h3b); // PREFETCH on UART while TX stalled
            tick(40);
            expect_quiet(uart_replies, "UART PREFETCH reply while TX stalled");
            uart_tx_ready = 1;
            expect_reply(uart_replies, 8'h81, "UART PREFETCH retained fault reply");
            // Second PREFETCH must report clean (fault cleared once).
            uart_byte(8'h3b);
            expect_reply(uart_replies, 8'h80, "UART PREFETCH cleared after first read");

            // Same on FT245 with queued VERSION + STATUS behind stalled TX.
            prefetch_thin_in = 1; tick(4); prefetch_thin_in = 0; tick(4);
            ft_tx_ready = 0;
            ft_push_now(8'h3b); // PREFETCH
            ft_push_now(8'h30); // VERSION
            ft_push_now(8'h36); // STATUS
            tick(40);
            expect_quiet(ft_replies, "FT245 replies while TX stalled");
            ft_tx_ready = 1;
            expect_reply(ft_replies, 8'h82, "FT245 PREFETCH retained thin reply");
            expect_reply(ft_replies, 8'h09, "FT245 VERSION after TX stall");
            expect_reply(ft_replies, 8'h02, "FT245 STATUS after TX stall");
            expect_quiet(uart_replies, "no UART leak from FT245 stalled replies");
        end

        // 13. Cross-port deferred boot START and STOP drain retention.
        begin
            // 13a. UART sends early START before pp_init_done, with UART TX
            // stalled; FT245 runs VERSION/STATUS while START is deferred and
            // after pp_init_done completes. UART must still get its 0x01 ACK
            // once unstalled, and FT245 must never receive it.
            reset = 1; tick(4); reset = 0;
            uart_tx_ready = 0;
            @(negedge clk); uart_data = 8'h34; uart_strobe = 1;
            @(negedge clk); uart_strobe = 0;
            tick(12);
            if (running) $fatal(1, "early UART START enabled SPI before pp_init_done");
            ft_byte(8'h30); ft_byte(8'h36);
            expect_reply(ft_replies, 8'h09, "FT245 VERSION while UART START deferred");
            expect_reply(ft_replies, 8'h02, "FT245 STATUS=stopped while UART START deferred");
            while (!running) tick(1);
            tick(20);
            expect_quiet(uart_replies, "deferred UART START while UART TX stalled");
            ft_byte(8'h36);
            expect_reply(ft_replies, 8'h01, "FT245 STATUS=running after pp_init_done");
            expect_quiet(ft_replies, "FT245 must not receive UART deferred START ACK");
            uart_tx_ready = 1;
            expect_reply(uart_replies, 8'h01, "UART deferred START ACK after unstall");

            // 13b. Reverse direction: FT245 sends early START while FT245 TX
            // is stalled; UART runs VERSION before and after pp_init_done.
            reset = 1; tick(4); reset = 0;
            ft_tx_ready = 0;
            ft_byte(8'h34);
            tick(12);
            if (running) $fatal(1, "early FT245 START enabled SPI before pp_init_done");
            @(negedge clk); uart_data = 8'h30; uart_strobe = 1;
            @(negedge clk); uart_strobe = 0;
            expect_reply(uart_replies, 8'h09, "UART VERSION while FT245 START deferred");
            while (!running) tick(1);
            tick(20);
            expect_quiet(ft_replies, "deferred FT245 START while FT245 TX stalled");
            uart_byte(8'h30);
            expect_reply(uart_replies, 8'h09, "UART VERSION after pp_init_done");
            expect_quiet(uart_replies, "UART must not receive FT245 deferred START ACK");
            ft_tx_ready = 1;
            expect_reply(ft_replies, 8'h01, "FT245 deferred START ACK after unstall");

            // 13c. STOP drain ACK retained on FT245 while TX is stalled.
            ft_tx_ready = 0;
            ft_byte(8'h35);
            tick(40);
            expect_quiet(ft_replies, "FT245 STOP ACK while TX stalled");
            ft_tx_ready = 1;
            expect_reply(ft_replies, 8'h01, "FT245 STOP ACK after unstall");
        end

        // 14. Directed CMD_LOGPOLL: empty terminator, 0xA0/0xA5 escaping,
        //     LOG_POLL_MAX=255 cap, TX backpressure, and pipelined VERSION.
        begin
            reg [7:0] raw_log [0:259];
            reg [7:0] want_poll1 [$];
            reg [7:0] want_poll2 [$];
            integer idx, expected_len, stall_guard;

            // Empty poll under UART TX stall -> single 0xA0 terminator.
            uart_tx_ready = 0;
            uart_byte(8'h3a);
            tick(30);
            expect_quiet(uart_replies, "empty LOGPOLL while UART TX stalled");
            uart_tx_ready = 1;
            expect_reply(uart_replies, 8'ha0, "empty UART LOGPOLL terminator");

            // Prefill 260 raw bytes including 0xA0 and 0xA5 at start, cap
            // boundary (254, 255), and tail (259).
            for (idx = 0; idx < 260; idx++) begin
                if (idx == 0 || idx == 254 || idx == 259) raw_log[idx] = 8'ha0;
                else if (idx == 1 || idx == 127 || idx == 255) raw_log[idx] = 8'ha5;
                else raw_log[idx] = 8'(idx & 8'h7f);
                log_queue[log_tail % LOG_Q_SIZE] = raw_log[idx];
                log_tail = log_tail + 1;
            end

            for (idx = 0; idx < 255; idx++) begin
                if (raw_log[idx] == 8'ha0) begin
                    want_poll1.push_back(8'ha5);
                    want_poll1.push_back(8'h00);
                end else if (raw_log[idx] == 8'ha5) begin
                    want_poll1.push_back(8'ha5);
                    want_poll1.push_back(8'h05);
                end else begin
                    want_poll1.push_back(raw_log[idx]);
                end
            end
            want_poll1.push_back(8'ha0);

            for (idx = 255; idx < 260; idx++) begin
                if (raw_log[idx] == 8'ha0) begin
                    want_poll2.push_back(8'ha5);
                    want_poll2.push_back(8'h00);
                end else if (raw_log[idx] == 8'ha5) begin
                    want_poll2.push_back(8'ha5);
                    want_poll2.push_back(8'h05);
                end else begin
                    want_poll2.push_back(raw_log[idx]);
                end
            end
            want_poll2.push_back(8'ha0);

            // Queue LOGPOLL + pipelined VERSION on FT245 with periodic TX stalls.
            ft_push_now(8'h3a);
            ft_push_now(8'h30);
            expected_len = want_poll1.size() + 1;
            stall_guard = 0;
            while (ft_replies.size() < expected_len && stall_guard < 50000) begin
                ft_tx_ready = (stall_guard % 7) == 0;
                tick(1);
                stall_guard++;
            end
            ft_tx_ready = 1;
            if (ft_replies.size() != expected_len)
                $fatal(1, "LOGPOLL1+VERSION got %0d bytes, expected %0d", ft_replies.size(), expected_len);
            for (idx = 0; idx < want_poll1.size(); idx++)
                expect_reply(ft_replies, want_poll1[idx], "LOGPOLL1 byte");
            expect_reply(ft_replies, 8'h09, "pipelined VERSION after LOGPOLL1");
            if ((log_tail - log_head) != 5)
                $fatal(1, "LOGPOLL1 left %0d bytes in FIFO, expected 5", log_tail - log_head);

            // Second LOGPOLL on UART drains remaining 5 bytes under UART TX stalls.
            uart_byte(8'h3a);
            expected_len = want_poll2.size();
            stall_guard = 0;
            while (uart_replies.size() < expected_len && stall_guard < 20000) begin
                uart_tx_ready = (stall_guard % 5) == 0;
                tick(1);
                stall_guard++;
            end
            uart_tx_ready = 1;
            for (idx = 0; idx < want_poll2.size(); idx++)
                expect_reply(uart_replies, want_poll2[idx], "LOGPOLL2 byte");
            if (log_head != log_tail)
                $fatal(1, "LOGPOLL2 did not drain logger FIFO");
        end

        // 15. Atomic TOCTOU_SET: partial SET timeout leaves existing trap intact,
        //     and a match on the commit cycle does not pair with the new entry.
        begin
            reg [7:0] set_init [0:11];
            reg [7:0] set_new [0:11];
            set_init = '{8'h39, 8'h01, 8'h00, 8'h01, 8'h00, 8'h00,
                         8'hff, 8'hf0, 8'h00, 8'h02, 8'h00, 8'h00};
            set_new  = '{8'h39, 8'h01, 8'h00, 8'h03, 8'h00, 8'h00,
                         8'hff, 8'hf0, 8'h00, 8'h04, 8'h00, 8'h00};

            foreach (set_init[i]) ft_push_now(set_init[i]);
            expect_reply(ft_replies, 8'h01, "initial TOCTOU SET");
            // Arm trap 0.
            ft_push_now(8'h39); ft_push_now(8'h02); ft_push_now(8'h00);
            expect_reply(ft_replies, 8'h01, "TOCTOU ARM trap 0");

            // Send partial TOCTOU_SET (11 of 12 bytes, stopping after byte 10)
            // and let header idle timeout fire.
            for (integer i = 0; i < 11; i++) ft_push_now(set_new[i]);
            tick(70000);
            expect_quiet(ft_replies, "partial TOCTOU SET must not reply");
            if (dut.host_i.trap_start[0] !== 24'h010000 ||
                dut.host_i.trap_mask[0]  !== 24'hfff000 ||
                dut.host_i.trap_replace[0] !== 24'h020000)
                $fatal(1, "partial TOCTOU SET mutated trap 0: start=%h mask=%h replace=%h",
                       dut.host_i.trap_start[0], dut.host_i.trap_mask[0], dut.host_i.trap_replace[0]);

            // Trigger trap 0 once so a second old match would redirect if not cancelled.
            spi_active = 1;
            log_addr = 24'h010000;
            @(negedge clk); log_addr_valid = 1;
            @(negedge clk); log_addr_valid = 0;
            tick(4);
            if (!dut.host_i.trap_triggered[0]) $fatal(1, "trap 0 did not arm-trigger");
            spi_active = 0; tick(2);

            // Commit new TOCTOU_SET while raising an old-address log_addr_valid
            // edge on the exact byte-11 commit cycle.
            for (integer i = 0; i < 11; i++) ft_push_now(set_new[i]);
            tick(40);
            spi_active = 1;
            log_addr = 24'h010000;
            ft_byte(set_new[11]); // popped on next cycle, processed 1 cycle later
            @(negedge clk); log_addr_valid = 1; // aligns with rxd_strobe_buf for byte 11
            @(negedge clk); log_addr_valid = 0;
            expect_reply(ft_replies, 8'h01, "atomic TOCTOU SET commit");
            tick(4);
            if (redirect_active)
                $fatal(1, "stale TOCTOU match redirected across atomic SET commit (base=%h)", redirect_base);
            if (dut.host_i.trap_triggered[0])
                $fatal(1, "trap_triggered stayed set after atomic TOCTOU SET");
            if (dut.host_i.trap_start[0] !== 24'h030000 ||
                dut.host_i.trap_mask[0]  !== 24'hfff000 ||
                dut.host_i.trap_replace[0] !== 24'h040000)
                $fatal(1, "atomic TOCTOU SET did not commit new entry");
            spi_active = 0;
        end

        $display("PASS HOST_PROTOCOL: admission, framing, reply retention, cross-port START/STOP, LOGPOLL, and atomic TOCTOU");
        $finish;
    end

    initial begin #200000000; $fatal(1, "host protocol test timeout"); end
endmodule
