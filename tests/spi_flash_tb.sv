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
    // Drive glue's prefetch fault inputs directly: the real flags come from
    // spi_prefetch inside spi_trx, and a genuine underrun needs a timing
    // corner the functional testbench does not reach.
    reg prefetch_underrun = 0, prefetch_thin = 0;
    // Whether the emulated chip advertises 4-byte addressing at all.
    reg cfg_4byte = 1'b0;
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
    integer outside_model = 0;
    // 8 KiB of model: one SDRAM row of four banks, which is what the
    // existing page-program and AAI tests stay within.
    reg [63:0] memory [0:1023];
    reg [7:0] expected [0:8191];
    // 10-bit model index: (row 0, bank, burst-in-row).
    wire [9:0] mem_index = {access_addr[11], access_addr[10:9], access_addr[8:2]};

    // FT245 receive FIFO as glue sees it: first-word-fallthrough data and a
    // pop strobe.
    reg [7:0] ft_queue [0:15];
    integer ft_head = 0, ft_tail = 0;
    wire ft_rx_available = ft_head != ft_tail;
    wire [7:0] ft_rx_byte = ft_queue[ft_head % 16];
    wire ft_rx_pop, ft_tx_strobe;
    wire [7:0] ft_tx_data;
    integer ft_replies = 0;
    reg [7:0] ft_reply [0:3];

    spi_trx spi (
        .clk(clk), .spi_clk(sck), .spi_reset(spi_reset), .spi_csel(cs),
        .spi_io0_in(mosi), .spi_io1_in(1'b0), .spi_io2_in(1'b1), .spi_io3_in(1'b1),
        .spi_io1_out(miso), .spi_io1_oe(miso_oe),
        .ram_read_buffer(read_buffer), .ram_read_busy(busy),
        .write_cmd(write_cmd), .write_type(write_type), .write_addr(write_addr),
        .write_len(write_len), .write_done(write_done),
        .write_buf_strobe(write_strobe), .write_buf_offset(write_offset),
        .write_buf_val(write_value), .cfg_jedec_id(24'h4125bf), .cfg_4byte(cfg_4byte),
        .cfg_chip_erase_bursts(23'h7fffff), .sfdp_raddr(sfdp_addr), .sfdp_rdata(sfdp_data)
    );
    glue dut (
        .clk(clk), .reset(reset), .rxd_strobe(rx_strobe), .rxd_data(rx_data),
        .txd_ready(1'b1), .txd_strobe(tx_strobe), .txd_data(tx_data),
        .ft_rx_data_available(ft_rx_available), .ft_rx_data(ft_rx_byte),
        .ft_rx_pop(ft_rx_pop), .ft_txd_ready(1'b1),
        .ft_txd_strobe(ft_tx_strobe), .ft_txd_data(ft_tx_data),
        .sdram_access_cmd(access), .sdram_access_addr(access_addr), .sdram_cmd_busy(busy),
        .sdram_read_busy(1'b0), .sdram_read_buffer(read_buffer), .sdram_write_buffer(write_buffer),
        .spi_reset(spi_reset), .spi_csel(cs), .spi_cmd_write(write_cmd),
        .spi_write_type(write_type), .spi_write_addr(write_addr), .spi_write_len(write_len),
        .spi_write_done(write_done), .spi_write_buf_strobe(write_strobe),
        .spi_write_buf_offset(write_offset), .spi_write_buf_val(write_value),
        .spi_clk(sck),
        .sfdp_raddr(sfdp_addr), .sfdp_rdata(sfdp_data), .spi_running(running),
        .log_fifo_data_available(1'b0), .log_fifo_read_data(8'b0),
        .log_addr_valid_sync(1'b0), .log_addr_sync(24'b0), .spi_active_sync(1'b0),
        .prefetch_underrun(prefetch_underrun), .prefetch_thin(prefetch_thin)
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
            busy <= 1;
            busy_cycles <= access == 3 ? 3 : 10;
            // The model flattens (row 0, bank, column) into 8 KiB, so a
            // second row would alias onto the first. A truncated index would
            // make two addresses share a byte, so accesses outside the
            // modelled row are dropped and counted rather than aliased. The
            // long erases below are the only thing that goes out there.
            if (access_addr[24:12] == 0) begin
                if (access == 1) read_buffer <= memory[mem_index];
                if (access == 2) memory[mem_index] <= write_buffer;
            end
            else begin
                outside_model = outside_model + 1;
                if (access == 1) read_buffer <= 64'hffffffffffffffff;
            end
        end
    end

    task host_byte(input [7:0] b);
        @(negedge clk); rx_data = b; rx_strobe = 1;
        @(negedge clk); rx_strobe = 0;
        repeat (10) @(negedge clk); // One byte per 12 system clocks
    endtask

    // FT245 receive FIFO model (declared above): pop advances the head,
    // and FT245-side replies are recorded apart from UART ones.
    always @(posedge clk) begin
        if (ft_rx_pop) ft_head <= ft_head + 1;
        if (ft_tx_strobe) begin
            if (ft_replies < 4) ft_reply[ft_replies] <= ft_tx_data;
            ft_replies <= ft_replies + 1;
        end
    end

    task ft_push(input [7:0] b);
        @(negedge clk);
        ft_queue[ft_tail % 16] = b;
        ft_tail = ft_tail + 1;
    endtask

    // Record every host reply so a query task can inspect the byte that
    // came back, and fail if the command produced none.
    reg last_reply = 0;
    reg [7:0] last_reply_byte = 8'hxx;
    always @(posedge clk) begin
        if (tx_strobe) begin
            last_reply <= 1;
            last_reply_byte <= tx_data;
        end
    end

    // Send an argument-free host command and return its reply byte. Fails
    // if nothing comes back, so a missing handler cannot pass silently.
    task automatic host_query(input [7:0] opcode, output [7:0] reply);
        last_reply = 0;
        host_byte(opcode);
        if (!last_reply) $fatal(1, "command %h produced no reply", opcode);
        reply = last_reply_byte;
    endtask

    // A fault must be visible to the host after the transaction that raised
    // it ended, and one read must both report and clear it.
    task automatic check_prefetch_faults;
        reg [7:0] reply;
        prefetch_underrun = 0; prefetch_thin = 0;
        repeat (4) @(negedge clk);
        host_query(8'h3b, reply);
        if (reply !== 8'h80) $fatal(1, "clean prefetch reported %h", reply);

        // Underrun only: bit 0.
        prefetch_underrun = 1;
        repeat (4) @(negedge clk);
        host_query(8'h3b, reply);
        if (reply !== 8'h81) $fatal(1, "underrun reported %h, expected 81", reply);
        // The SPI flag stays high through the CS gap. A second host read
        // must not mistake that same level for a new fault.
        host_query(8'h3b, reply);
        if (reply !== 8'h80) $fatal(1, "underrun reported twice: %h", reply);
        prefetch_underrun = 0;
        repeat (4) @(negedge clk);
        prefetch_underrun = 1;
        repeat (4) @(negedge clk);
        host_query(8'h3b, reply);
        if (reply !== 8'h81) $fatal(1, "new underrun not reported: %h", reply);
        prefetch_underrun = 0;
        host_query(8'h3b, reply);
        if (reply !== 8'h80) $fatal(1, "new underrun not cleared: %h", reply);

        // Both flags at once: bits 0 and 1.
        prefetch_underrun = 1; prefetch_thin = 1;
        repeat (4) @(negedge clk);
        prefetch_underrun = 0; prefetch_thin = 0;
        host_query(8'h3b, reply);
        if (reply !== 8'h83) $fatal(1, "both faults reported %h, expected 83", reply);

        // Thin alone: bit 1.
        prefetch_thin = 1;
        repeat (4) @(negedge clk);
        prefetch_thin = 0;
        host_query(8'h3b, reply);
        if (reply !== 8'h82) $fatal(1, "thin reported %h, expected 82", reply);

        // Reset must drop a latched fault, and the SPI flag still being
        // high from before the reset is not a new fault.
        prefetch_underrun = 1;
        repeat (8) @(negedge clk);
        reset = 1;
        repeat (4) @(negedge clk);
        reset = 0;
        repeat (8) @(negedge clk);
        host_query(8'h3b, reply);
        if (reply !== 8'h80) $fatal(1, "reset left fault latched: %h", reply);
        // A fault after the reset is still reported.
        prefetch_underrun = 0;
        repeat (4) @(negedge clk);
        prefetch_underrun = 1;
        repeat (4) @(negedge clk);
        prefetch_underrun = 0;
        host_query(8'h3b, reply);
        if (reply !== 8'h81) $fatal(1, "fault after reset reported %h, expected 81", reply);
    endtask

    // PREFETCH is argument-free and must be answered over FT245 while the
    // target holds CS low, without leaving anything in the RX FIFO that
    // would block the next always-safe command.
    task automatic check_prefetch_ft245_while_selected;
        integer waited;
        ft_replies = 0;
        select_spi;
        repeat (4) @(negedge clk);
        ft_push(8'h3b);
        ft_push(8'h36);
        waited = 0;
        while (ft_replies < 2 && waited < 200) begin
            @(negedge clk);
            waited++;
        end
        if (ft_replies != 2)
            $fatal(1, "FT245 PREFETCH+STATUS with CS low got %0d replies", ft_replies);
        if (ft_reply[0] !== 8'h80) $fatal(1, "FT245 PREFETCH reported %h", ft_reply[0]);
        if (ft_reply[1] !== 8'h01) $fatal(1, "FT245 STATUS reported %h", ft_reply[1]);
        if (ft_head != ft_tail) $fatal(1, "FT245 RX FIFO not drained with CS low");
        deselect_spi;
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

    task address4(input integer a);
        send(8'(a >> 24)); send(8'(a >> 16));
        send(8'(a >> 8)); send(8'(a));
    endtask

    // Issue a program or erase command with a four-byte address and check
    // the burst address, burst count and type the SPI engine handed to
    // glue. The model only backs the first few KiB, so this checks the
    // decode rather than writing through the data path.
    //
    // write_cmd is a one-clock pulse that glue has already consumed by the
    // time the transaction ends, so the held registers are what to read.
    // An erase only arms with WEL set, as on a real part, so the enable and
    // disable bracket it here rather than relying on a neighbouring command.
    task erase_cmd4(input [7:0] opcode, input integer a,
                    input [22:0] want_addr, input [22:0] want_len);
        command(8'h06);
        write_cmd4(opcode, a, want_addr, want_len, 2'd1);
        command(8'h04);
    endtask

    // Same, for an erase large enough that the default WIP budget is
    // sized by the 32 KiB and 64 KiB forms below.
    task erase_cmd4_long(input [7:0] opcode, input integer a,
                         input [22:0] want_addr, input [22:0] want_len);
        erase_cmd4(opcode, a, want_addr, want_len);
    endtask

    // The same, with a three-byte address phase: used where the point is
    // that the engine went back to three bytes after 0xE9.
    task write_cmd3(input [7:0] opcode, input integer a,
                    input [22:0] want_addr, input [22:0] want_len,
                    input [1:0] want_type);
        command(8'h06);
        select_spi;
        send(opcode);
        address(a);
        deselect_spi;
        check_write(opcode, a, want_addr, want_len, want_type);
        command(8'h04);
    endtask

    task check_write(input [7:0] opcode, input integer a,
                     input [22:0] want_addr, input [22:0] want_len,
                     input [1:0] want_type);
        if (write_addr !== want_addr)
            $fatal(1, "opcode %h at %h latched burst %h, expected %h",
                   opcode, a, write_addr, want_addr);
        if (write_len !== want_len)
            $fatal(1, "opcode %h at %h latched %0d bursts, expected %0d (%h)",
                   opcode, a, write_len, want_len, want_len);
        if (write_type !== want_type)
            $fatal(1, "opcode %h at %h latched type %0d, expected %0d",
                   opcode, a, write_type, want_type);
        poll_done;
    endtask

    task write_cmd4(input [7:0] opcode, input integer a,
                    input [22:0] want_addr, input [22:0] want_len,
                    input [1:0] want_type);
        select_spi;
        send(opcode);
        address4(a);
        deselect_spi;
        check_write(opcode, a, want_addr, want_len, want_type);
    endtask

    // Poll WIP, budgeting in system clocks rather than polls: a 64 KiB erase
    // is 8192 SDRAM bursts, which is far more than any fixed poll count.
    task automatic poll_done(input integer max_cycles = 120000);
        reg [7:0] status;
        integer waited = 0;
        status = 8'hff;
        while (status[0] && waited < max_cycles) begin
            select_spi; send(8'h05); spi_byte(0, status); deselect_spi;
            waited = waited + 30;  // one poll costs about 30 system clocks
        end
        if (status[0])
            $fatal(1, "WIP failed to clear within %0d system clocks", max_cycles);
    endtask

    task check_memory;
        for (integer i = 0; i < 8192; i++)
            if (memory[i/8][(i%8)*8 +: 8] !== expected[i])
                $fatal(1, "Memory[%h]: got %h expected %h", i,
                       memory[i/8][(i%8)*8 +: 8], expected[i]);
    endtask

    // Page program with a four-byte address phase (opcode 0x12).
    task page_program4(input integer a, input integer length, input [7:0] seed);
        reg [7:0] page [0:255];
        reg [255:0] valid;
        reg [7:0] b;
        valid = 0;
        command(8'h06);
        select_spi; send(8'h12); address4(a);
        for (integer i = 0; i < length; i++) begin
            b = 8'(i * 31) ^ seed;
            send(b);
            page[(a+i)%256] = b;
            valid[(a+i)%256] = 1;
        end
        deselect_spi; poll_done;
        // expected[] is indexed by the real byte address, not by the page
        // base: this program deliberately starts mid-page.
        for (integer i = 0; i < length; i++)
            expected[a + i] &= page[(a + i) % 256];
        check_memory;
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

        // Prefetch diagnostics first: the reply must be correct before any
        // SPI traffic can raise a real flag.
        check_prefetch_faults;

        // Deliberately configure before a 128-cycle RAM scrub could finish.
        configure_sfdp(1, 8'ha5);
        check_early_start = 1;
        host_byte(8'h34);
        if (running) $fatal(1, "Early START enabled SPI before page-buffer initialization");
        wait (early_start_ack); check_early_start = 0;
        read_sfdp(0, 260, 1, 8'ha5);
        check_prefetch_ft245_while_selected;

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

        // ---- four-byte addressing -------------------------------------

        // The data path is checked for 0x12 here and for every read command
        // in quad_fast_tb; what is unique to this bench is the decode of a
        // four-byte address and the two mode commands.
        cfg_4byte = 1'b1;

        // 0x12 programs, and the bytes land where the four address bytes
        // said they would.
        command(8'h06);
        page_program4('h1a0, 3, 8'hc3);
        command(8'h04);

        // The 0xB7/0xE9 mode pair: in four-byte mode a plain 0x03 must take
        // four address bytes, and three after 0xE9.
        select_spi; send(8'hb7); deselect_spi;
        // 4 KiB sector erase at 0x2000: burst 0x400, 512 bursts.
        erase_cmd4(8'h21, 32'h0000_2000, 23'h00400, 23'h001ff);
        // Page program at 0x0a00 with a four-byte phase: page-aligned, so
        // burst 0x140, 32 bursts, type program.
        command(8'h06);
        write_cmd4(8'h02, 32'h0000_0a00, 23'h00140, 23'h0001f, 2'd0);
        command(8'h04);
        command(8'he9);
        // Back in three-byte mode the plain opcode takes three address
        // bytes again. Sending a fourth here would be a malformed command,
        // so this uses the three-byte form.
        write_cmd3(8'h20, 32'h0000_2000, 23'h00400, 23'h001ff, 2'd1);
        // The 32 KiB and 64 KiB erase forms, both in four-byte mode.
        select_spi; send(8'hb7); deselect_spi;
        erase_cmd4_long(8'h5c, 32'h0000_8000, 23'h01000, 23'h00fff);
        erase_cmd4_long(8'hdc, 32'h0001_0000, 23'h02000, 23'h01fff);
        command(8'he9);

        // A chip that does not support four-byte addressing must ignore both
        // mode commands: 0xB7 leaves the engine in three-byte mode.
        cfg_4byte = 1'b0;
        command(8'hb7);
        write_cmd3(8'h20, 32'h0000_2000, 23'h00400, 23'h001ff, 2'd1);

        // A real four-byte erase through the data path: a 4 KiB sector at
        // the start of the modelled window, after programming it.
        // page_program already folded the new data into expected[].
        check_memory;
        command(8'h06);
        select_spi; send(8'h21); address4('h0000); deselect_spi;
        poll_done();
        command(8'h04);
        for (integer i = 0; i < 4096; i++) expected[i] = 8'hff;
        check_memory;
        // Five erases above execute outside the 8 KiB model: three 4 KiB
        // sectors (512 bursts each) plus the 32 KiB (4096) and 64 KiB
        // (8192) forms. Each burst costs two SDRAM accesses (activate +
        // write), so the count is exact: a decode bug that strays outside
        // the model can no longer hide behind the long erases.
        $display("      %0d SDRAM accesses fell outside the modelled row (the long erases)",
                 outside_model);
        if (outside_model !== 2*(512+512+4096+8192+512))
            $fatal(1, "out-of-model accesses %0d, expected 27648", outside_model);

        // Reset invalidates SFDP, even though the BSRAM itself retains data.
        host_byte(8'h35); configure_sfdp(128, 8'h72);
        @(negedge clk); reset = 1;
        repeat (4) @(negedge clk); reset = 0;
        host_byte(8'h34); wait (running);
        read_sfdp(0, 130, 0, 0);
        $display("PASS SPI: prefetch fault latch incl. FT245 with CS low, SFDP/startup/reconfigure/reset, PP/wrap/AND, AAI/alignment/bounds/cleanup, 4-byte address decode and mode commands");
        $finish;
    end

    initial begin
        #50000000;
        $fatal(1, "SPI test timeout");
    end
endmodule
