// SDRAM Controller for Tang Primer 25K External SDRAM Module
//
// The dock's SDRAM module has two W9825G6KH chips (32MB each = 64MB total)
// sharing a 16-bit data bus. A single CS pin selects between chips:
// CS=LOW selects chip 0, CS=HIGH selects chip 1 (module has onboard inverter).
//
// Per chip: 4 banks, 8192 rows (13-bit), 512 columns (9-bit), 16-bit data
// CAS latency = 2, burst length = 4 (4 x 16-bit = 64 bits = 8 bytes per burst)
//
// Memory data is stored byte-serially to minimize first-byte latency:
// SDRAM burst beat w (4 x 16-bit words per 8-byte burst) carries bytes
// 2w and 2w+1 whole (dq[7:0] = byte 2w, dq[15:8] = byte 2w+1). The first
// beat therefore already completes bytes 0-1, which the controller
// publishes immediately; bytes 2-7 follow with beats 1-3 and publish with
// the final beat as before. First-byte latency is CAS+1 instead of
// CAS+BL, which is what makes dummy-less reads (0x03) and tiny first
// bursts (high start offsets) closable at fast SCLK. Later bytes are only
// ever consumed long after their beats arrive, so progressive publish is
// race-free (beats of one READ command arrive back-to-back).
//
// Clock and timing architecture:
//   - SDRAM clock is a separate PLL output (CLKOUT1) with PE_COARSE=9
//     phase shift (~3.75ns delay from logic clock). This provides setup
//     time for commands and write data at the SDRAM pins.
//   - IODELAY does NOT work for clock outputs on this device; PLL phase
//     shift is used instead.
//   - Read data is captured using aux_clk (CLKOUT2 with PE_COARSE=6,
//     ~2.5ns phase shift) to sample DQ in the middle of the valid window.
//   - With the ~3.75ns SDRAM clock delay and CAS=2, read data arrives
//     ~3 logic clock cycles after the READ command. RD_PIPELINE_DELAY=0
//     with the `readcount > tCAS` capture condition matches this timing.
//
// Address mapping (23-bit burst address, 8 bytes per burst = 64MB):
//   spi_addr[22]    = chip select (0=chip0, 1=chip1)
//   spi_addr[21:9]  = row (13 bits)
//   spi_addr[8:7]   = bank (2 bits)
//   spi_addr[6:0]   = column / 4 (7 bits, col[8:2], burst-4 aligned)

`default_nettype none

module sdram(
    input wire clk,
    input wire aux_clk,      // Phase-shifted clock for read data capture
    input wire reset,

    // SDRAM physical interface (directly to pins)
    output reg [1:0] ba_o,
    output reg [12:0] a_o,       // 13-bit address for W9825G6KH
    output reg cs_o,             // Single CS: LOW=chip0, HIGH=chip1
    output reg ras_o,
    output reg cas_o,
    output reg we_o,
    output reg [1:0] dqm_o,
    inout wire [15:0] dq_io,     // Bidirectional data bus (directly to pins)
    
    // Control signals from spi_trx (directly from SPI clock domain)
    input wire spi_active,
    input wire spi_inhibit_refresh,
    input wire spi_cmd_activate,
    input wire spi_cmd_read,
    input wire [22:0] spi_addr,      // 23-bit burst address (64MB)
    input wire spi_cmd_post_toggle, // Toggles on every SPI prefetch post

    // Control signals from glue (serial path)
    input wire [1:0] access_cmd,     // 00=nop 01=read 10=write 11=activate
    input wire [24:0] access_addr,   // Access address (25-bit: chip + row + bank + col)
    input wire inhibit_refresh,
    output reg cmd_busy,

    output reg [63:0] read_buffer,
    output reg [63:0] read_buffer_b,
    output reg read_valid_a,
    output reg read_valid_b,
    output reg read_busy,

    input wire [63:0] write_buffer
);

    parameter CLK_FREQ_MHZ = 132;
    parameter BURST_LEN = 4;

    // Internal DQ bus handling - tristate managed in this module
    // (Gowin requires inout and tristate to be in the same module for proper IOBUF inference)
    reg [15:0] dq_o;
    reg dq_oe_o;
    wire [15:0] dq_i;
    assign dq_io = dq_oe_o ? dq_o : 16'hZZZZ;
    assign dq_i = dq_io;

    // Timing parameters (in clock cycles)
    // Based on W9825G6KH-6 datasheet (166MHz grade, tCK_min=6ns for CL=3)
    // At 120MHz: tCK = 8.33ns
    localparam integer tINIT        = 100 * CLK_FREQ_MHZ;   // 100us init
    localparam integer tREFRESH     = (CLK_FREQ_MHZ * 32000) / 8192;  // ~468 cycles
    localparam integer tRP          = 2;   // 16.7ns precharge (min 15ns for -6)
    localparam integer tRC          = 8;   // 66.7ns row cycle (min 60ns for -6)
    localparam integer tMRD         = 2;   // 2 cycles mode register set
    localparam integer tRCD         = 2;   // 16.7ns RAS to CAS delay (min 15ns for -6)
    localparam integer tDPL         = 2;   // Write recovery (min 2 tCK)
    localparam integer tRAS         = 6;   // 50ns row active time (min 42ns for -6)
    // CAS latency 2: the W9825G6KH-6 is rated CL2 to 133MHz, so CL2 at
    // 120MHz is in spec and saves a full cycle of first-byte latency
    // versus CL3. This is load-bearing for dummy-less reads (0x03)
    // and tiny first bursts at fast SCLK. The MRS below programs the
    // same value into both chips, and tREAD/capture track it.
    localparam integer tCAS         = 2;   // CAS latency = 2 for W9825G6KH

    // Read pipeline delay: compensates for SDRAM clock phase shift and capture pipeline.
    // With PE_COARSE=9 on SDRAM clock and aux_clk capture, RD_PIPELINE_DELAY=0 is correct.
    localparam integer RD_PIPELINE_DELAY = 0;

    // Derived timing
    localparam integer tREAD  = tCAS + RD_PIPELINE_DELAY + BURST_LEN + 1;
    localparam integer tWRITE = BURST_LEN + tDPL + tRP;

    // State machine states
    localparam
        STA_INIT            = 0,
        STA_INIT_PRECHARGE  = 1,
        STA_INIT_REFRESH    = 2,
        STA_IDLE            = 3,
        STA_SETMODE         = 4,
        STA_REFRESH         = 5,
        STA_ACTIVATE        = 6,
        STA_READ            = 7,
        STA_WRITE           = 8,
        STA_INIT_CHIP2      = 9,   // Init second chip
        STA_INIT_PRECHARGE2 = 10,
        STA_INIT_REFRESH2      = 11,
        STA_SETMODE2           = 12,
        STA_REFRESH2           = 13,  // Refresh second chip
        STA_SPI_ABORT_WAIT     = 14,  // Wait tRAS before closing an abandoned row
        STA_SPI_ABORT_PRECHARGE = 15; // Precharge row opened by an aborted SPI read

    // Burst mode encoding
    localparam [2:0] BURST_MODE =
        (BURST_LEN == 1) ? 3'b000 :
        (BURST_LEN == 2) ? 3'b001 :
        (BURST_LEN == 4) ? 3'b010 :
        (BURST_LEN == 8) ? 3'b011 :
        3'b111;

    reg [3:0] state;
    reg [$clog2(tINIT)-1:0] initcount;
    reg initrefreshcount;
    reg [4:0] cmdcount;
    reg [4:0] cmdtarget;
    reg [$clog2(tREFRESH):0] refreshcount;
    
    // Track which chip we're refreshing (alternate between chips)
    reg refresh_chip;
    
    // Buffer control signals from SPI domain
    reg [2:0] spi_active_buf;
    reg [1:0] spi_inhibit_refresh_buf;
    reg [1:0] spi_cmd_activate_buf;
    reg [1:0] spi_cmd_read_buf;
    
    reg spi_cmd_activate_ack;
    reg spi_cmd_read_ack;
    reg spi_abort_pending;

    // One-burst lookahead (ping-pong) for the SPI fast path. The SPI side
    // consumes one 64-bit buffer while the controller fills the other, so a
    // completing SDRAM read never overwrites data still shifted out.
    // fill_sel selects the target of the next SPI fill (0=A=read_buffer,
    // 1=B=read_buffer_b) and toggles on every SPI READ completion. The SPI
    // side keeps its own consume toggle; the two march in lockstep because
    // serial-path reads are impossible while SPI is active (glue
    // serial_gate) and both sides restart at buffer A on every CS drop.
    reg fill_sel;
    reg fill_reset_armed;
    reg serial_read_active;
    // Edge-arming for the SPI fast path. A dispatch additionally requires
    // the request level to have been observed LOW while selected since the
    // last dispatch (or reset). Stale-high levels left over from a previous
    // transaction therefore cannot dispatch spuriously at the next select
    // and consume the pairing reset; every legitimate post follows a drop
    // (reset clear, address-end, mode-count-2, or byte-6 gaps), so arming
    // is always satisfied before a real post arrives.
    reg spi_act_armed;
    reg spi_read_armed;
    // Post-toggle synchronizer. Each edge invalidates the upcoming fill
    // target, so a burst that never fills can never alias a stale buffer.
    // The toggle always wins the race against its own fill: sync settles
    // in ~2 sysclks, the fill needs ~12.
    reg [2:0] spi_post_sync;

    wire spi_deselect_event = !spi_active_buf[1] && spi_active_buf[2];
    
    // Track whether ACTIVATE has been dispatched for the current SPI burst.
    // READ must not dispatch until this is set, preventing a race where
    // independent 2-FF synchronizers cause read_buf[1] to appear before
    // activate_buf[1] (Bug #3 fix).
    reg spi_activate_done;
    
    wire do_inhibit_refresh = (spi_inhibit_refresh_buf[1] || inhibit_refresh);
    
    // Address decoding for MT48LC16M16A2 (2 × 32MB = 64MB)
    // SPI path: 23-bit burst address
    //   [22]    = chip select
    //   [21:9]  = row (13 bits)
    //   [8:7]   = bank (2 bits)
    //   [6:0]   = column burst index (col[8:2])
    // Latch the row/bank portion when ACTIVATE is accepted. The column bits
    // arrive later in the SPI address phase, so READ samples them from the
    // live SPI address bus after its synchronized request arrives. spi_trx
    // holds each portion stable across the corresponding handshake.
    reg [22:0] spi_addr_latched;
    wire spi_chip_sel  = spi_addr_latched[22];
    wire [12:0] spi_row  = spi_addr_latched[21:9];
    wire [1:0]  spi_bank = spi_addr_latched[8:7];
    wire [8:0]  spi_col  = {spi_addr[6:0], 2'b00};  // Burst-4 aligned

    // Serial/glue path: 25-bit address = {chip, row, bank, col}
    //   [24]     = chip select
    //   [23:11]  = row (13 bits)
    //   [10:9]   = bank (2 bits)
    //   [8:0]    = column (9 bits, low 2 bits = burst offset)
    wire access_chip_sel  = access_addr[24];
    wire [12:0] access_row  = access_addr[23:11];
    wire [1:0]  access_bank = access_addr[10:9];
    wire [8:0]  access_col  = access_addr[8:0];

    reg [4:0] readcount;
    reg [1:0] rdbuf_write_ptr;
    reg [2:0] wrbuf_read_ptr;

    // Read data capture: DQ sampled on aux_clk (phase-shifted for proper timing)
    // then used directly in the main clock domain read logic.
    reg [15:0] dq_captured;
    
    always @(posedge aux_clk) begin
        dq_captured <= dq_i;
    end

    integer i;

    always @(posedge clk) begin
        if (reset) begin
            state <= STA_INIT;
            cs_o <= 0;           // Select chip 0 for init (CS LOW = chip 0)
            ras_o <= 1;
            cas_o <= 1;
            we_o <= 1;
            ba_o <= 0;
            a_o <= 0;
            dq_oe_o <= 0;
            dq_o <= 0;
            dqm_o <= 2'b11;

            cmd_busy <= 1;

            initcount <= 0;
            initrefreshcount <= 0;
            cmdcount <= 0;
            cmdtarget <= 0;
            refreshcount <= 0;
            refresh_chip <= 0;
            
            read_buffer <= 0;
            read_buffer_b <= 0;
            read_valid_a <= 0;
            read_valid_b <= 0;
            read_busy <= 0;
            readcount <= 0;

            rdbuf_write_ptr <= 0;
            wrbuf_read_ptr <= 0;
            
            spi_active_buf <= 0;
            spi_inhibit_refresh_buf <= 0;
            spi_cmd_activate_buf <= 0;
            spi_cmd_read_buf <= 0;
            
            spi_cmd_activate_ack <= 0;
            spi_cmd_read_ack <= 0;
            spi_abort_pending <= 0;
            spi_activate_done <= 0;
            spi_addr_latched <= 0;
            fill_sel <= 0;
            fill_reset_armed <= 0;
            serial_read_active <= 0;
            spi_act_armed <= 1;
            spi_read_armed <= 1;
            spi_post_sync <= 0;
        end
        else begin
            refreshcount <= refreshcount + 1;
            
            // Synchronize SPI control signals. Gate requests with active CS so
            // a master that stops the clock while deasserting CS cannot leave
            // refresh inhibition or command requests asserted indefinitely.
            spi_active_buf <= {spi_active_buf[1:0], spi_active};
            spi_inhibit_refresh_buf <= {spi_inhibit_refresh_buf[0],
                                        spi_inhibit_refresh && spi_active};
            spi_cmd_activate_buf <= {spi_cmd_activate_buf[0],
                                     spi_cmd_activate && spi_active};
            spi_cmd_read_buf <= {spi_cmd_read_buf[0],
                                 spi_cmd_read && spi_active};

            // If CS drops after ACTIVATE was accepted but before READ was
            // dispatched, close the open row once tRAS has safely elapsed.
            if (spi_deselect_event && spi_activate_done && !spi_cmd_read_ack)
                spi_abort_pending <= 1;
            // A CS drop ends the burst chain. The reset itself is deferred
            // to the next ACTIVATE dispatch: a fill posted just before the
            // drop may still complete afterwards, and must land before the
            // pairing restarts (the controller is strictly sequential, so no
            // new-transaction fill can overtake it).
            if (spi_deselect_event) fill_reset_armed <= 1;
            // A transaction can end with request levels still asserted (the
            // over-fetch post after the last burst). The level/ack handshake
            // would then stay settled-high across the deselect, and the next
            // transaction's posts would find the acks set and never dispatch.
            // Clear both acks here so every fresh post dispatches; the abort
            // path below keeps working because it keys off activate_done
            // (deliberately not cleared) with the now-zero read ack.
            if (spi_deselect_event) begin
                spi_cmd_activate_ack <= 0;
                spi_cmd_read_ack <= 0;
            end
            // Deselect also disarms: the low levels during the idle gap must
            // not re-arm a stale-high post. The next transaction re-arms
            // when its reset clear propagates through the synchronizers,
            // strictly before any post can arrive (posts need a full command
            // byte first).
            if (spi_deselect_event) begin
                spi_act_armed <= 0;
                spi_read_armed <= 0;
            end
            else begin
                if (!spi_cmd_activate_buf[1] && spi_active_buf[1]) spi_act_armed <= 1;
                if (!spi_cmd_read_buf[1] && spi_active_buf[1]) spi_read_armed <= 1;
            end
            // A post always precedes its fill by ~10 sysclks, so
            // invalidating here can never clobber a completed fill.
            spi_post_sync <= {spi_post_sync[1:0], spi_cmd_post_toggle};
            if (spi_post_sync[1] ^ spi_post_sync[2]) begin
                if (fill_sel) read_valid_b <= 0;
                else read_valid_a <= 0;
            end
            
            if (spi_cmd_activate_ack && !spi_cmd_activate_buf[1]) begin
                spi_cmd_activate_ack <= 0;
                spi_activate_done <= 0;
            end
            if (spi_cmd_read_ack && !spi_cmd_read_buf[1]) spi_cmd_read_ack <= 0;

            // Update busy flag
            // Busy when: any command is in progress (state != IDLE),
            // a new command has been posted (access_cmd != 0), or a
            // refresh is imminent and not inhibited.
            cmd_busy <= (state != STA_IDLE) ||
                        (access_cmd != 2'b00) ||
                        ((refreshcount >= tREFRESH-1) && !do_inhibit_refresh);

            if (state == STA_INIT) begin
                // Wait for SDRAM power-up (100us) - chip 0 selected (CS LOW)
                if (initcount >= tINIT) begin
                    state <= STA_INIT_PRECHARGE;
                    cmdcount <= 1;
                    cmdtarget <= tRP;

                    cs_o <= 0;       // Chip 0 (CS LOW)
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;       // PRECHARGE
                    dqm_o <= 2'b11;
                    a_o[10] <= 1;    // All banks
                end
                else begin
                    initcount <= initcount + 1;
                    // NOP
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                end
            end
            else if ((state != STA_IDLE) && (cmdcount < cmdtarget)) begin
                // Waiting for command to complete
                // Issue NOP
                ras_o <= 1;
                cas_o <= 1;
                we_o <= 1;

                if (state == STA_WRITE) begin
                    if (cmdcount < BURST_LEN) begin
                        // Feed write data (16-bit at a time, byte-serial)
                        dq_oe_o <= 1;
                        
                        // Byte-serial layout: word w carries bytes 2w/2w+1.
                        // wrbuf_read_ptr counts 1..3 here (word 0 went out
                        // at dispatch above). Explicit lanes, see read path.
                        case (wrbuf_read_ptr)
                            2'd1: dq_o <= write_buffer[31:16];
                            2'd2: dq_o <= write_buffer[47:32];
                            2'd3: dq_o <= write_buffer[63:48];
                            default: dq_o <= write_buffer[15:0];
                        endcase
                        dqm_o <= 2'b00;

                        wrbuf_read_ptr <= wrbuf_read_ptr + 1;
                    end
                    else begin
                        dq_oe_o <= 0;
                        dqm_o <= 2'b11;
                    end
                end

                cmdcount <= cmdcount + 1;
            end
            else begin
                // No command running, determine next command
                cmdcount <= 1;

                if (state == STA_INIT_PRECHARGE) begin
                    state <= STA_INIT_REFRESH;
                    cmdtarget <= tRC;
                    initrefreshcount <= 0;

                    // REFRESH command (chip 0)
                    cs_o <= 0;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                end
                else if (state == STA_INIT_REFRESH) begin
                    if (initrefreshcount == 1) begin
                        state <= STA_SETMODE;
                        cmdtarget <= tMRD;
                        refreshcount <= 1;

                        // MODE REGISTER SET (chip 0)
                        cs_o <= 0;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 0;
                        dqm_o <= 2'b11;
                        ba_o <= 2'b00;
                        a_o <= 0;
                        a_o[9] <= 1'b0;         // Write burst: programmed length
                        a_o[8:7] <= 2'b00;      // Standard operation
                        a_o[6:4] <= tCAS;       // CAS latency = 2 (tCAS)
                        a_o[3] <= 1'b0;         // Burst type: sequential
                        a_o[2:0] <= BURST_MODE; // Burst length = 4
                    end
                    else begin
                        initrefreshcount <= 1;

                        // Another REFRESH (chip 0)
                        cs_o <= 0;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 1;
                        dqm_o <= 2'b11;
                    end
                end
                else if (state == STA_SETMODE) begin
                    // Chip 0 init done, now init chip 1
                    state <= STA_INIT_CHIP2;
                    cmdcount <= 1;
                    cmdtarget <= tRP;

                    // PRECHARGE all banks on chip 1
                    cs_o <= 1;       // Chip 1 (CS HIGH)
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    dqm_o <= 2'b11;
                    a_o <= 0;
                    a_o[10] <= 1;    // All banks
                end
                else if (state == STA_INIT_CHIP2) begin
                    // Precharge done for chip 1, do refresh
                    state <= STA_INIT_REFRESH2;
                    cmdtarget <= tRC;
                    initrefreshcount <= 0;

                    // REFRESH command (chip 1)
                    cs_o <= 1;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                end
                else if (state == STA_INIT_REFRESH2) begin
                    if (initrefreshcount == 1) begin
                        state <= STA_SETMODE2;
                        cmdtarget <= tMRD;

                        // MODE REGISTER SET (chip 1)
                        cs_o <= 1;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 0;
                        dqm_o <= 2'b11;
                        ba_o <= 2'b00;
                        a_o <= 0;
                        a_o[9] <= 1'b0;
                        a_o[8:7] <= 2'b00;
                        a_o[6:4] <= tCAS;
                        a_o[3] <= 1'b0;
                        a_o[2:0] <= BURST_MODE;
                    end
                    else begin
                        initrefreshcount <= 1;

                        // Another REFRESH (chip 1)
                        cs_o <= 1;
                        ras_o <= 0;
                        cas_o <= 0;
                        we_o <= 1;
                        dqm_o <= 2'b11;
                    end
                end
                else if (state == STA_SETMODE2) begin
                    // Both chips initialized, go to idle
                    state <= STA_IDLE;
                    cs_o <= 0;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                // Refresh sequence transitions.  Handled inside the main
                // dispatcher (rather than a separate always-block hunk
                // after it) so they take priority over any SPI fast-path
                // or serial command dispatch that might otherwise fire in
                // the same cycle.  If a post-dispatch hunk overrode the
                // state back to REFRESH2, the command-side flags
                // (spi_cmd_activate_ack / spi_activate_done) set by the
                // overridden dispatch would persist, and the next
                // cycle's READ would fire without a matching ACTIVATE --
                // reading from a bank with no open row and returning all
                // 0xff from the floating DQ bus for the rest of the SPI
                // burst.
                else if (state == STA_REFRESH) begin
                    // Chip 0 refreshed, now refresh chip 1.
                    state <= STA_REFRESH2;
                    cmdtarget <= tRC;

                    cs_o <= 1;       // Chip 1
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                    dqm_o <= 2'b11;

                    refresh_chip <= 1;
                end
                else if (state == STA_REFRESH2) begin
                    // Chip 1 refreshed, back to idle.
                    refresh_chip <= 0;
                    state <= STA_IDLE;
                    cmdtarget <= 1;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (state == STA_SPI_ABORT_WAIT) begin
                    // The aborted ACTIVATE has satisfied tRAS; close only the
                    // bank opened for this SPI request.
                    state <= STA_SPI_ABORT_PRECHARGE;
                    cmdtarget <= tRP;

                    cs_o <= spi_chip_sel;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    ba_o <= spi_bank;
                    a_o <= 0;
                    a_o[10] <= 0;
                    dqm_o <= 2'b11;
                end
                else if (state == STA_SPI_ABORT_PRECHARGE) begin
                    state <= STA_IDLE;
                    cmdtarget <= 1;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (spi_abort_pending) begin
                    // Wait a full tRAS from abort detection. This is slightly
                    // conservative but guarantees PRECHARGE is never early.
                    spi_abort_pending <= 0;
                    state <= STA_SPI_ABORT_WAIT;
                    cmdtarget <= tRAS;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (spi_cmd_activate_buf[1] && !spi_cmd_activate_ack && spi_act_armed) begin
                    // SPI fast-path activate
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;
                    spi_cmd_activate_ack <= 1;
                    spi_activate_done <= 1;
                    spi_addr_latched <= spi_addr;
                    spi_act_armed <= 0;
                    if (fill_reset_armed) begin
                        fill_sel <= 0;
                        read_valid_a <= 0;
                        read_valid_b <= 0;
                        fill_reset_armed <= 0;
                    end

                    // ACTIVATE command
                    cs_o <= spi_addr[22];
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                    ba_o <= spi_addr[8:7];
                    a_o <= spi_addr[21:9];
                end
                else if (spi_cmd_read_buf[1] && !spi_cmd_read_ack && spi_activate_done && spi_read_armed) begin
                    // SPI fast-path read
                    state <= STA_READ;
                    cmdtarget <= tREAD;
                    read_busy <= 1;
                    spi_cmd_read_ack <= 1;
                    spi_read_armed <= 0;
                    serial_read_active <= 0;
                    // Invalidate the fill target up front; set again below
                    // when the final beat publishes.
                    if (fill_sel) read_valid_b <= 0;
                    else read_valid_a <= 0;

                    // READ command with auto-precharge
                    cs_o <= spi_chip_sel;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= spi_bank;
                    a_o <= 0;
                    a_o[8:0] <= spi_col;     // 9-bit column, burst-4 aligned
                    a_o[10] <= 1;            // Auto precharge
                    dq_oe_o <= 0;
                    dqm_o <= 2'b00;
                end
                else if (access_cmd == 2'b11) begin
                    // Serial path activate
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;

                    // ACTIVATE command
                    cs_o <= access_chip_sel;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                    ba_o <= access_bank;
                    a_o <= access_row;
                end
                else if (access_cmd == 2'b01) begin
                    // Serial path read
                    state <= STA_READ;
                    cmdtarget <= tREAD + 2;
                    read_busy <= 1;
                    serial_read_active <= 1;
                    read_valid_a <= 0;

                    // READ command with auto-precharge
                    cs_o <= access_chip_sel;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= access_bank;
                    a_o <= 0;
                    a_o[8:0] <= access_col;  // 9-bit column
                    a_o[10] <= 1;            // Auto precharge
                    dq_oe_o <= 0;
                    dqm_o <= 2'b00;
                end
                else if (access_cmd == 2'b10) begin
                    // Serial path write
                    state <= STA_WRITE;
                    cmdtarget <= tWRITE;
                    wrbuf_read_ptr <= 1;

                    // WRITE command with auto-precharge
                    cs_o <= access_chip_sel;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 0;
                    ba_o <= access_bank;
                    a_o <= 0;
                    a_o[8:0] <= access_col;  // 9-bit column
                    a_o[10] <= 1;            // Auto precharge
                    dq_oe_o <= 1;
                    
                    // First write data word (byte-serial: bytes 0-1)
                    dq_o <= write_buffer[15:0];
                    dqm_o <= 2'b00;
                end
                else if ((refreshcount >= tREFRESH) && !do_inhibit_refresh) begin
                    // Auto refresh - alternate between chips
                    state <= STA_REFRESH;
                    cmdtarget <= tRC;
                    refreshcount <= 1;

                    // REFRESH command for current chip
                    cs_o <= refresh_chip;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else begin
                    state <= STA_IDLE;
                    cmdtarget <= 1;  // Dispatch runs every cycle when idle
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
            end

            // REFRESH -> REFRESH2 -> IDLE transitions are now handled
            // inside the main dispatcher (as STA_REFRESH / STA_REFRESH2
            // cases) so they take priority over SPI and serial command
            // dispatches in the same cycle.
            
            // Read data capture: each beat's bytes publish straight into
            // the target buffer (progressive publish, see layout note at the
            // top of this file). Beats of one READ arrive back-to-back, and
            // the ping-pong target is always the idle half, so the SPI side
            // never observes a torn burst.
            if ((readcount > tCAS + RD_PIPELINE_DELAY) && (readcount <= tCAS + RD_PIPELINE_DELAY + BURST_LEN)) begin
                // rdbuf_write_ptr counts 0..3 (4 words in burst).
                // Byte-serial progressive publish: beat w completes bytes
                // 2w/2w+1 straight into the target buffer (serial: A; SPI:
                // ping-pong fill target). Beats of one READ arrive
                // back-to-back, so later bytes are always ready long before
                // the SPI side shifts them out; only the first byte(s) of a
                // burst are timing-critical. Validity (bytes 0-1 ready) is
                // therefore set with beat 0; the underrun check only ever
                // inspects a buffer whose full fill completed a whole burst
                // earlier, so early-valid is conservative-safe.
                // (Explicit lanes rather than variable part-selects: safest
                // across Yosys, Verilator, and Gowin synthesis.)
                case (rdbuf_write_ptr)
                    2'd0: begin
                        if (serial_read_active) begin
                            read_buffer[15:0] <= dq_captured;
                            read_valid_a <= 1;
                        end
                        else if (fill_sel) begin
                            read_buffer_b[15:0] <= dq_captured;
                            read_valid_b <= 1;
                        end
                        else begin
                            read_buffer[15:0] <= dq_captured;
                            read_valid_a <= 1;
                        end
                    end
                    2'd1: begin
                        if (serial_read_active) read_buffer[31:16] <= dq_captured;
                        else if (fill_sel) read_buffer_b[31:16] <= dq_captured;
                        else read_buffer[31:16] <= dq_captured;
                    end
                    2'd2: begin
                        if (serial_read_active) read_buffer[47:32] <= dq_captured;
                        else if (fill_sel) read_buffer_b[47:32] <= dq_captured;
                        else read_buffer[47:32] <= dq_captured;
                    end
                    2'd3: begin
                        if (serial_read_active) begin
                            read_buffer[63:48] <= dq_captured;
                            read_valid_a <= 1;
                            read_valid_b <= 0;
                            fill_sel <= 0;
                            serial_read_active <= 0;
                        end
                        else if (fill_sel) begin
                            read_buffer_b[63:48] <= dq_captured;
                            fill_sel <= 0;
                        end
                        else begin
                            read_buffer[63:48] <= dq_captured;
                            fill_sel <= 1;
                        end
                        read_busy <= 0;
                    end
                endcase

                rdbuf_write_ptr <= rdbuf_write_ptr + 1;
            end
            else
                rdbuf_write_ptr <= 0;
            
            readcount <= (state == STA_READ) ? cmdcount : 0;
        end
    end

endmodule
