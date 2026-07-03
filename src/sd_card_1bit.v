// Minimal native 1-bit SD card emulator, read-only SDHC.
//
// Data path: a prefetch engine keeps the SDRAM ~64 SD clocks ahead of the
// DAT0 stream, so blocks stream gap-free at any practical SD clock.
//
// SDRAM fast-path contract (same as spi_trx): activate+read are HELD
// until the controller accepts them (read_busy rises); the controller
// silently drops short pulses.  Holding also makes refresh preemption
// harmless, so no inhibit_refresh is needed.
`default_nettype none

module sd_card_1bit(
    input wire clk,
    input wire reset,
    input wire sd_reset,
    input wire sd_clk,
    input wire sd_cmd_in,
    output reg sd_cmd_out,
    output reg sd_cmd_oe,
    output reg sd_dat0_out,
    output reg sd_dat0_oe,
    input wire [31:0] image_sectors,

    output reg ram_activate,
    output reg ram_read,
    output reg [22:0] ram_addr,
    input wire [63:0] ram_read_buffer,
    input wire ram_read_busy,

    output reg log_cmd_valid,
    output reg [7:0] log_cmd_opcode,
    output reg log_addr_valid,
    output reg [31:0] log_addr_out,
    output reg [23:0] log_byte_count
);
    localparam ST_IDLE   = 0;
    localparam ST_RESP   = 1;
    localparam ST_STREAM = 2;

    // stream phases
    localparam SPH_START = 0;   // wait for first burst, load it
    localparam SPH_BIT0  = 1;   // drive start bit (0) on next clk_fall
    localparam SPH_DATA  = 2;   // stream block bytes MSB first
    localparam SPH_CRC   = 3;   // 16-bit CRC16
    localparam SPH_END   = 4;   // end bit (1)

    // prefetch engine
    localparam PF_IDLE = 0;
    localparam PF_REQ  = 1;
    localparam PF_WAIT = 2;
    localparam PF_CLR  = 3;     // settle so the controller's acks clear

    localparam RR_IDLE = 0;
    localparam RR_READ = 1;

    localparam [15:0] RCA = 16'h0001;
    localparam [31:0] OCR = 32'hc0ff8000;          // powered up, CCS, 3.2-3.6V
    localparam [31:0] CARD_STATUS = 32'h00000900;  // ready_for_data | state=tran
    localparam [31:0] ST_ILLEGAL  = 32'h00000004;
    localparam [31:0] ST_OUT_OF_RANGE = 32'h80000000;

    reg [1:0] state;
    reg resp_return;
    // 2-FF synchronizers; edges are detected combinationally, so outputs
    // land ~1.5 clk after the pin edge.  Needs clk >> sd_clk (25MHz SD
    // clock leaves ~5ns setup to the host at 120MHz clk).
    reg [1:0] sd_clk_sync;
    reg [1:0] sd_cmd_sync;
    wire clk_rise = sd_clk_sync == 2'b01;
    wire clk_fall = sd_clk_sync == 2'b10;
    wire cmd_i = sd_cmd_sync[1];

    // command receiver; frame layout after 47 bits: start/tx/idx/arg/crc
    reg [47:0] cmd_shift;
    reg [5:0] cmd_bits;
    wire [5:0] cmd_idx = cmd_shift[44:39];
    wire [31:0] cmd_arg = cmd_shift[38:7];
    reg app_cmd;

    reg [135:0] resp_shift;
    reg [7:0] resp_bits;

    // stream engine
    reg [2:0] sph;
    reg stream_multi;
    reg stream_scr;
    reg [9:0] block_len;
    reg [12:0] bits_left;      // bits left in current block
    reg [5:0] burst_bit;       // bit index within current 8-byte burst
    reg [5:0] block_burst;     // burst index within current block
    reg [63:0] data_buf;
    reg data_ready;            // data_buf holds the next block's first burst
    reg [15:0] data_crc;
    reg [4:0] crc_bit;

    // prefetch engine
    reg [1:0] pf_state;
    reg pf_busy_seen;
    reg [2:0] pf_clr_cnt;
    reg prefetch_valid;
    reg [22:0] fetch_addr;

    // sdram read_buffer streams lowest byte first (same order as spi_trx);
    // the shifter emits MSB first, so swap byte lanes on load.
    function [63:0] bswap64;
        input [63:0] d;
        integer i;
        begin
            for (i = 0; i < 8; i = i + 1)
                bswap64[i*8 +: 8] = d[(7-i)*8 +: 8];
        end
    endfunction

    function [6:0] crc7_next;
        input [6:0] crc;
        input bitval;
        reg inv;
        begin
            inv = bitval ^ crc[6];
            crc7_next = {crc[5:3], crc[2] ^ inv, crc[1:0], inv};
        end
    endfunction

    function [6:0] crc7_40;
        input [39:0] data;
        integer i;
        reg [6:0] crc;
        begin
            crc = 0;
            for (i = 39; i >= 0; i = i - 1)
                crc = crc7_next(crc, data[i]);
            crc7_40 = crc;
        end
    endfunction

    function [15:0] crc16_next;
        input [15:0] crc;
        input bitval;
        reg inv;
        begin
            inv = bitval ^ crc[15];
            crc16_next = {crc[14:0], 1'b0} ^ (inv ? 16'h1021 : 16'h0000);
        end
    endfunction

    function [7:0] cid_byte;
        input [3:0] idx;
        begin
            case (idx)
            0: cid_byte = 8'h1d; 1: cid_byte = 8'h4e; 2: cid_byte = 8'h4f; 3: cid_byte = 8'h52;
            4: cid_byte = 8'h42; 5: cid_byte = 8'h45; 6: cid_byte = 8'h52; 7: cid_byte = 8'h54;
            8: cid_byte = 8'h10; 9: cid_byte = 8'h00; 10: cid_byte = 8'h00; 11: cid_byte = 8'h00;
            12: cid_byte = 8'h01; 13: cid_byte = 8'h01; 14: cid_byte = 8'h01; default: cid_byte = 8'hff;
            endcase
        end
    endfunction

    // CSD v2.0: C_SIZE lives in byte7[5:0]/byte8/byte9.
    function [7:0] csd_byte;
        input [3:0] idx;
        reg [31:0] csize;
        begin
            csize = (image_sectors > 0) ? ((image_sectors - 1) >> 10) : 0;
            case (idx)
            0: csd_byte = 8'h40;              // CSD_STRUCTURE=1 (v2.0)
            1: csd_byte = 8'h0e;              // TAAC
            2: csd_byte = 8'h00;              // NSAC
            3: csd_byte = 8'h32;              // TRAN_SPEED 25MHz
            4: csd_byte = 8'h58;              // CCC: basic, block read, select, app, switch
            5: csd_byte = 8'h59;              // CCC low + READ_BL_LEN=9 (512B)
            6: csd_byte = 8'h00;              // no partial/misalign/DSR
            7: csd_byte = {2'b00, csize[21:16]};
            8: csd_byte = csize[15:8];
            9: csd_byte = csize[7:0];
            10: csd_byte = 8'h7f;             // erase_blk_en + sector_size
            11: csd_byte = 8'h80;
            12: csd_byte = 8'h0a;
            13: csd_byte = 8'h40;
            14: csd_byte = 8'h60;             // COPY | PERM_WRITE_PROTECT (read-only)
            default: csd_byte = 8'hff;
            endcase
        end
    endfunction

    function [7:0] scr_byte;
        input [2:0] idx;
        begin
            case (idx)
            0: scr_byte = 8'h02;              // SCR_STRUCTURE=0, SD_SPEC=2
            1: scr_byte = 8'h01;              // SD_BUS_WIDTHS: 1-bit only
            default: scr_byte = 8'h00;
            endcase
        end
    endfunction

    task short_resp;
        input [5:0] idx;
        input [31:0] body;
        begin
            resp_shift <= {{2'b00, idx, body}, crc7_40({2'b00, idx, body}), 1'b1, 88'd0};
            resp_bits <= 48;
            state <= ST_RESP;
            sd_cmd_oe <= 1;
        end
    endtask

    task long_resp;
        input kind;   // 0=CID, 1=CSD
        integer i;
        reg [119:0] body;
        begin
            body = 0;
            for (i = 0; i < 15; i = i + 1)
                body[(14-i)*8 +: 8] = (kind == 0) ? cid_byte(i[3:0]) : csd_byte(i[3:0]);
            resp_shift <= {8'h3f, body, 7'h7f, 1'b1};
            resp_bits <= 136;
            state <= ST_RESP;
            sd_cmd_oe <= 1;
        end
    endtask

    // Queue a data transfer after the command response.  sector is a block
    // address, len is block size in bytes (8 for SCR), multi keeps blocks
    // coming until CMD12, scr serves the SCR word instead of SDRAM.
    task start_stream;
        input [31:0] sector;
        input [9:0] len;
        input multi;
        input scr;
        begin
            resp_return <= RR_READ;
            block_len <= len;
            stream_multi <= multi;
            stream_scr <= scr;
            fetch_addr <= {sector[16:0], 6'b000000};  // sector * 64 bursts
            prefetch_valid <= 0;
            data_ready <= 0;
            pf_state <= PF_IDLE;
            log_addr_valid <= 1;
            log_addr_out <= sector;
            log_byte_count <= {14'b0, len};
        end
    endtask

    always @(posedge clk) begin
        sd_clk_sync <= {sd_clk_sync[0], sd_clk};
        sd_cmd_sync <= {sd_cmd_sync[0], sd_cmd_in};
        log_cmd_valid <= 0;
        log_addr_valid <= 0;
        ram_activate <= 0;
        ram_read <= 0;

        if (reset || sd_reset) begin
            state <= ST_IDLE;
            resp_return <= RR_IDLE;
            cmd_bits <= 0;
            cmd_shift <= 0;
            sd_cmd_out <= 1;
            sd_cmd_oe <= 0;
            sd_dat0_out <= 1;
            sd_dat0_oe <= 1;
            app_cmd <= 0;
            resp_bits <= 0;
            sph <= SPH_START;
            stream_multi <= 0;
            stream_scr <= 0;
            block_len <= 0;
            bits_left <= 0;
            burst_bit <= 0;
            data_crc <= 0;
            crc_bit <= 0;
            pf_state <= PF_IDLE;
            pf_busy_seen <= 0;
            pf_clr_cnt <= 0;
            prefetch_valid <= 0;
            data_ready <= 0;
            fetch_addr <= 0;
            ram_addr <= 0;
        end else begin
            // ---------------------------------------------------------
            // Prefetch engine: fetch_addr -> ram_read_buffer, one burst
            // ahead of the stream.  Holds activate+read until accepted.
            // ---------------------------------------------------------
            case (pf_state)
            PF_REQ: begin
                ram_addr <= fetch_addr;
                ram_activate <= 1;
                ram_read <= 1;
                pf_busy_seen <= 0;
                pf_state <= PF_WAIT;
            end
            PF_WAIT: begin
                if (!pf_busy_seen) begin
                    ram_activate <= 1;
                    ram_read <= 1;
                    if (ram_read_busy)
                        pf_busy_seen <= 1;
                end else if (!ram_read_busy) begin
                    prefetch_valid <= 1;
                    fetch_addr <= fetch_addr + 1;
                    pf_clr_cnt <= 4;
                    pf_state <= PF_CLR;
                end
            end
            PF_CLR: begin
                if (pf_clr_cnt <= 1)
                    pf_state <= PF_IDLE;
                else
                    pf_clr_cnt <= pf_clr_cnt - 1;
            end
            default: ;
            endcase

            // ---------------------------------------------------------
            // Command receiver.  Runs in ST_STREAM too so CMD12 stops.
            // ---------------------------------------------------------
            if ((state == ST_IDLE || state == ST_STREAM) && clk_rise) begin
                if (cmd_bits == 0 && cmd_i) begin
                    cmd_bits <= 0;   // line idle, wait for start bit
                end else begin
                    cmd_shift <= {cmd_shift[46:0], cmd_i};
                    if (cmd_bits != 47) begin
                        cmd_bits <= cmd_bits + 1;
                    end else begin
                        cmd_bits <= 0;
                        log_cmd_valid <= 1;
                        log_cmd_opcode <= {2'b00, cmd_idx};
                        if (state == ST_STREAM) begin
                            // any command stops the stream; CMD12 is the
                            // normal multi-block stop, CMD0 is silent
                            stream_scr <= 0;
                            stream_multi <= 0;
                            data_ready <= 0;
                            sph <= SPH_START;
                            sd_dat0_out <= 1;
                            app_cmd <= 0;
                            if (cmd_idx == 12)
                                short_resp(12, CARD_STATUS);
                            else if (cmd_idx == 13)
                                short_resp(13, CARD_STATUS);
                            else if (cmd_idx != 0)
                                short_resp(cmd_idx, ST_ILLEGAL);
                            else
                                state <= ST_IDLE;
                        end else begin
                            case (cmd_idx)
                            0:  app_cmd <= 0;                       // GO_IDLE: no response
                            8:  begin app_cmd <= 0; short_resp(8, {20'h00001, cmd_arg[7:0]}); end
                            55: begin app_cmd <= 1; short_resp(55, 32'h00000020); end
                            41: begin
                                if (app_cmd) begin app_cmd <= 0; short_resp(41, OCR); end
                                else short_resp(41, ST_ILLEGAL);
                            end
                            58: begin app_cmd <= 0; short_resp(58, OCR); end
                            2:  begin app_cmd <= 0; long_resp(0); end
                            3:  begin app_cmd <= 0; short_resp(3, {RCA, 16'h0000}); end
                            9:  begin app_cmd <= 0; long_resp(1); end
                            7:  begin
                                app_cmd <= 0;
                                if (cmd_arg[31:16] == RCA)
                                    short_resp(7, CARD_STATUS);
                                // deselect: no response
                            end
                            13: begin app_cmd <= 0; short_resp(13, CARD_STATUS); end
                            16: begin app_cmd <= 0; short_resp(16, (cmd_arg == 32'd512) ? CARD_STATUS : ST_ILLEGAL); end
                            51: begin
                                if (app_cmd) begin
                                    app_cmd <= 0;
                                    short_resp(51, CARD_STATUS);
                                    start_stream(32'd0, 10'd8, 1'b0, 1'b1);
                                end else short_resp(51, ST_ILLEGAL);
                            end
                            17, 18: begin
                                app_cmd <= 0;
                                if (cmd_arg < image_sectors) begin
                                    short_resp(cmd_idx, CARD_STATUS);
                                    start_stream(cmd_arg, 10'd512, cmd_idx == 18, 1'b0);
                                end else begin
                                    short_resp(cmd_idx, ST_OUT_OF_RANGE);
                                end
                            end
                            default: begin app_cmd <= 0; short_resp(cmd_idx, ST_ILLEGAL); end
                            endcase
                        end
                    end
                end
            end

            // ---------------------------------------------------------
            // Response engine (CMD line, changes on clk_fall)
            // ---------------------------------------------------------
            if (state == ST_RESP && clk_fall) begin
                sd_cmd_out <= resp_shift[135];
                resp_shift <= {resp_shift[134:0], 1'b1};
                if (resp_bits == 1) begin
                    resp_bits <= 0;
                    sd_cmd_oe <= 0;
                    if (resp_return == RR_READ) begin
                        resp_return <= RR_IDLE;
                        state <= ST_STREAM;
                        sph <= SPH_START;
                        if (stream_scr) begin
                            data_buf <= {scr_byte(0), scr_byte(1), scr_byte(2), scr_byte(3),
                                         scr_byte(4), scr_byte(5), scr_byte(6), scr_byte(7)};
                        end else begin
                            prefetch_valid <= 0;
                            pf_state <= PF_REQ;    // first burst fetch
                        end
                    end else begin
                        state <= ST_IDLE;
                    end
                end else begin
                    resp_bits <= resp_bits - 1;
                end
            end

            // ---------------------------------------------------------
            // Data stream engine (DAT0, changes on clk_fall)
            // ---------------------------------------------------------
            if (state == ST_STREAM) begin
                // Load next burst into the shifter once the prefetch has
                // landed; then drive the start bit on the next clk_fall.
                if (sph == SPH_START) begin
                    if (stream_scr || data_ready || (prefetch_valid && pf_state == PF_IDLE)) begin
                        if (!stream_scr && !data_ready) begin
                            data_buf <= bswap64(ram_read_buffer);
                            prefetch_valid <= 0;
                            pf_state <= PF_REQ;    // prefetch next burst
                        end
                        data_ready <= 0;
                        data_crc <= 0;
                        burst_bit <= 0;
                        block_burst <= 0;
                        bits_left <= {block_len, 3'b000};
                        sph <= SPH_BIT0;
                    end
                end else if (sph == SPH_BIT0 && clk_fall) begin
                    sd_dat0_out <= 0;              // start bit
                    sph <= SPH_DATA;
                end else if (sph == SPH_DATA && clk_fall) begin
                    sd_dat0_out <= data_buf[63];
                    data_crc <= crc16_next(data_crc, data_buf[63]);
                    burst_bit <= burst_bit + 1;
                    if (burst_bit == 63 && (stream_multi || bits_left > 1)) begin
                        // burst drained: swap in the prefetched one
                        data_buf <= bswap64(ram_read_buffer);
                        prefetch_valid <= 0;
                        block_burst <= block_burst + 1;
                        // block-end load is the next block's first burst
                        if (bits_left == 1)
                            data_ready <= 1;
                        // fetch further ahead only if more bursts are needed
                        if (stream_multi || block_burst < 62)
                            pf_state <= PF_REQ;
                    end else begin
                        data_buf <= {data_buf[62:0], 1'b0};
                    end
                    if (bits_left == 1) begin
                        sph <= SPH_CRC;
                        crc_bit <= 0;
                    end else begin
                        bits_left <= bits_left - 1;
                    end
                end else if (sph == SPH_CRC && clk_fall) begin
                    sd_dat0_out <= data_crc[15 - crc_bit];
                    if (crc_bit == 15) begin
                        sph <= SPH_END;
                    end else begin
                        crc_bit <= crc_bit + 1;
                    end
                end else if (sph == SPH_END && clk_fall) begin
                    sd_dat0_out <= 1;              // end bit
                    if (stream_multi)
                        sph <= SPH_START;          // next block follows
                    else
                        state <= ST_IDLE;
                end
            end
        end
    end
endmodule
