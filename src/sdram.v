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
//     with beats publishing from read pipeline stage tCAS+1 matches this
//     timing.
//
// Address mapping (23-bit burst address, 8 bytes per burst = 64MB):
//   spi_addr[22]    = chip select (0=chip0, 1=chip1)
//   spi_addr[21:9]  = row (13 bits)
//   spi_addr[8:7]   = bank (2 bits)
//   spi_addr[6:0]   = column / 4 (7 bits, col[8:2]); the READ's col[1:0]
//                     is the start word (spi_word, critical word first)

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
    // First SDRAM word of the burst (critical word first): the READ
    // starts there and the sequential burst wraps, so the word the SPI
    // side needs first arrives first. Bundled with spi_addr's column.
    input wire [1:0] spi_word,
    input wire spi_cmd_post_toggle, // Toggles on every SPI prefetch post
    input wire spi_cmd_continuation, // Post is a lookahead (bundled with spi_addr)
    // Second burst of an array read. The SPI side raises this level once
    // the address phase is complete; the controller then fetches spi_addr2
    // itself right behind the first burst (see the burst pair below).
    // spi_addr2 is bundled with the level and held until the next
    // transaction's address phase.
    input wire spi_cmd_pair,
    input wire [22:0] spi_addr2,
    // TOCTOU decision for this transaction's address not final yet. Holds
    // SPI ACTIVATEs: lookahead addresses pass through the redirect mux.
    // (The initial ACTIVATE is posted before the address ends, so it is
    // not delayed in practice.)
    input wire spi_redirect_pending,

    // Control signals from glue (serial path)
    input wire [1:0] access_cmd,     // 00=nop 01=read 10=write 11=activate
    input wire [24:0] access_addr,   // Access address (25-bit: chip + row + bank + col)
    input wire inhibit_refresh,
    output reg cmd_busy,
    output reg access_accept,   // One cycle at actual serial-path dispatch

    output reg [63:0] read_buffer,
    output reg [63:0] read_buffer_b,
    output reg read_valid_a,
    output reg read_valid_b,
    // One bit per published 16-bit beat. Cleared before reuse and set
    // with its data, so high-offset/short reads can check their own bytes.
    output reg [3:0] read_beats_a,
    output reg [3:0] read_beats_b,
    output reg read_busy,

    input wire [63:0] write_buffer
);

    parameter CLK_FREQ_MHZ = 132;
    parameter [4:0] BURST_LEN = 5'd4;

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
    //
    // The values consumed by cmdtarget are sized to it (5 bits), so a
    // timing constant that stops fitting is an elaboration error rather
    // than a silent truncation. tINIT and tREFRESH stay plain integers:
    // they size the counters; sized copies drive refresh arithmetic.
    localparam integer tINIT        = 100 * CLK_FREQ_MHZ;   // 100us init
    localparam integer tREFRESH     = (CLK_FREQ_MHZ * 32000) / 8192;  // ~468 cycles
    // Refresh events alternate between the two chips, one chip per event,
    // so an event is due every tREFRESH/2 cycles and each chip is still
    // refreshed every tREFRESH. A single event stalls dispatch for one
    // tRC instead of a back-to-back pair. If tREFRESH is odd the truncated
    // event period refreshes slightly early, never late.
    localparam integer tREFRESH_EVENT = tREFRESH / 2;
    // Inhibit is a latency hint, not permission to lose memory retention.
    // Allow one per-chip interval of deferral past an event's due point,
    // then service refresh at a safe boundary.
    localparam integer tREFRESH_MAX = 2 * tREFRESH;
    // Keep service-latency headroom above the hard deadline so elapsed
    // time is not lost while a row is closing or a burst is completing.
    localparam integer REFRESH_BITS = $clog2(tREFRESH_MAX + 64);
    localparam [REFRESH_BITS-1:0] REFRESH_PERIOD = tREFRESH[REFRESH_BITS-1:0];
    localparam [REFRESH_BITS-1:0] REFRESH_EVENT = tREFRESH_EVENT[REFRESH_BITS-1:0];
    // Increment-predicted thresholds: refresh_due rises on the edge where
    // refreshcount reaches one event period, hard_refresh_due where it
    // reaches an event period plus a full per-chip deferral interval.
    localparam [REFRESH_BITS-1:0] REFRESH_SOFT_PRE = REFRESH_EVENT - 1'b1;
    localparam [REFRESH_BITS-1:0] REFRESH_HARD_PRE = REFRESH_EVENT + REFRESH_PERIOD - 1'b1;
    // Post-payment predicts, compared against the pre-dispatch count: after
    // paying one event period the next event is already due / hard-due.
    localparam [REFRESH_BITS-1:0] REFRESH_AGAIN_PRE = (REFRESH_EVENT << 1) - 1'b1;
    localparam [REFRESH_BITS:0] REFRESH_BACKLOG_PRE =
        {1'b0, REFRESH_EVENT} * 2'd2 + {1'b0, REFRESH_PERIOD} - 1'b1;
    localparam [4:0] tRP            = 5'd2;   // 16.7ns precharge (min 15ns for -6)
    localparam [4:0] tRC            = 5'd8;   // 66.7ns row cycle (min 60ns for -6)
    localparam [4:0] tMRD           = 5'd2;   // 2 cycles mode register set
    localparam [4:0] tRCD           = 5'd2;   // 16.7ns RAS to CAS delay (min 15ns for -6)
    localparam [4:0] tDPL           = 5'd2;   // Write recovery (min 2 tCK)
    localparam [4:0] tRAS           = 5'd6;   // 50ns row active time (min 42ns for -6)
    // CAS latency 2: the W9825G6KH-6 is rated CL2 to 133MHz, so CL2 at
    // 120MHz is in spec and saves a full cycle of first-byte latency
    // versus CL3. This is load-bearing for dummy-less reads (0x03)
    // and tiny first bursts at fast SCLK. The MRS below programs the
    // same value into both chips, and tREAD/capture track it.
    localparam [4:0] tCAS           = 5'd2;   // CAS latency = 2 for W9825G6KH

    // Read pipeline delay: compensates for SDRAM clock phase shift and capture pipeline.
    // With PE_COARSE=9 on SDRAM clock and aux_clk capture, RD_PIPELINE_DELAY=0 is correct.
    localparam [4:0] RD_PIPELINE_DELAY = 5'd0;

    // Derived timing
    // READ to next command when it is not a streamed SPI READ.
    localparam [4:0] tREAD  = tCAS + RD_PIPELINE_DELAY + BURST_LEN + 1;
    localparam [4:0] tWRITE = BURST_LEN + tDPL + tRP;

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
        STA_ROW_PRECHARGE      = 13,  // Wait tRP after closing an open row
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
    // Predict the wait predicate when count/target change. The registered
    // flag keeps the variable comparison out of the DQ output-register CE.
    reg cmd_wait;
    reg [REFRESH_BITS-1:0] refreshcount;
    
    // Chip refreshed by the next event; toggles at every refresh dispatch
    // so events alternate strictly between the chips.
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
    // 1=B=read_buffer_b) and toggles on every SPI READ dispatch (fills
    // complete in dispatch order, see the read data pipeline). The SPI
    // side keeps its own consume toggle; the two march in lockstep because
    // serial-path reads are impossible while SPI is active (glue
    // serial_gate) and both sides restart at buffer A on every CS drop.
    reg fill_sel;
    // Posts describe future buffers, not the fill currently in flight.
    // Track their order separately so a post during progressive publish
    // cannot invalidate already-published beats of the current burst.
    // A transaction's first post toggle targets A: B goes to the pair.
    reg post_fill_sel;
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
    assign spi_row_idle = !spi_activate_done || spi_cmd_read_ack;
    
    // SPI refresh window: while an array read inhibits refresh, a refresh
    // may still start after a lookahead READ was dispatched and until the
    // SPI side re-arms that post (see spi_prefetch). The next request is
    // then at least the post's remaining burst away, so the refresh
    // cannot delay it by more than the clocks between re-arm and post.
    // The burst pair opens the same window: from its READ until the SPI
    // side drops spi_cmd_pair at the first burst's re-arm point. Reads
    // too short for a lookahead post have no other window, and without
    // one the refresh backlog would end in a hard refresh between the
    // first burst's ACTIVATE and READ. The pair window is registered
    // (it closes a clock after the drop is seen) to keep it out of the
    // dispatch priority logic.
    reg spi_read_continuation;
    reg [2:0] spi_pair_buf;  // spi_cmd_pair synchronizer (see the burst pair)
    reg spi_pair_read_done;  // Pair READ of this transaction dispatched
    reg spi_pair_window;
    wire spi_refresh_window = (spi_cmd_read_ack && spi_cmd_read_buf[1] && spi_read_continuation) ||
                              spi_pair_window;
    wire do_inhibit_refresh = (spi_inhibit_refresh_buf[1] && !spi_refresh_window) ||
                              inhibit_refresh;
    // Registered deadline flags keep counter arithmetic/comparison out of
    // the command/read-buffer dispatch path. Predict the increment so the
    // flags describe the same elapsed time as refreshcount after each edge.
    reg refresh_due, hard_refresh_due;
    // At most one client may hold a row between its ACTIVATE and its
    // READ/WRITE dispatch. The SPI fast path outranks serial requests,
    // so both directions are interlocked explicitly: an SPI post cannot
    // open a bank under a serial pair, and a serial request or refresh
    // cannot run while an SPI row waits for its READ.
    reg serial_row_open;
    wire spi_row_idle;

    // Open-row tracking: SPI READs leave their row open (no
    // auto-precharge), so a following burst in the same row skips
    // ACTIVATE and tRCD entirely. One row per bank is tracked, indexed
    // {chip, bank}. With the address map below a sequential stream stays
    // in one bank row for 1KB, so it misses once per bank switch.
    // Serial accesses and refresh close what they need first (one bank,
    // or precharge-all for refresh). Serial READ/WRITE keep
    // auto-precharge and clear their bank's record at dispatch.
    reg [12:0] open_rows [0:7];
    reg [7:0] open_valid;
    wire [2:0] spi_open_idx = {spi_addr[22], spi_addr[8:7]};
    // The SPI ACTIVATE decision uses registered row lookups, keeping the
    // redirect mux and the row compare out of the dispatch path. spi_addr
    // is valid one system clock before its synchronized post arrives
    // (bundled-data contract). A redirect decision changes it, so posts
    // also wait out the clock after redirect_pending; and every command
    // that changes the open-row records waits >= 2 clocks, so the lookup
    // is current whenever dispatch evaluates.
    reg spi_row_hit, spi_bank_open, spi_redirect_settle;
    wire [3:0] refresh_open = refresh_chip ? open_valid[7:4] : open_valid[3:0];
    // Burst pair. Posting the second burst from the SPI side costs a
    // second synchronized handshake, and for a read starting at byte 7
    // that request could only go out during the mode/dummy clocks, a few
    // SPI clocks before its data was due. Instead the SPI side raises
    // spi_cmd_pair at the end of the address phase and the controller
    // fetches spi_addr2 itself, BURST_LEN clocks behind the first READ
    // when the row is already open. spi_addr2 is a continuation address
    // (redirected like any lookahead), so the fetch also waits for the
    // redirect decision. The pair level uses three synchronizer stages so
    // it cannot overtake the address event that starts that decision.
    // The SPI side counts the pair as the transaction's first lookahead
    // post without toggling spi_cmd_post_toggle; the fill reset below
    // accounts for it. (spi_pair_buf is declared with the refresh window.)
    reg spi_pair_armed;
    reg spi_txn_read_done;   // First SPI READ of this transaction dispatched
    reg auto_pending;
    // The redirected address is captured once the redirect decision is
    // final, and its open-row hit is registered, so neither the redirect
    // mux nor the row compare sits in front of the dispatch logic. The
    // registered hit is current whenever dispatch evaluates: every
    // command that changes the open-row records waits >= 2 clocks.
    reg auto_ready;
    reg [22:0] auto_addr;
    reg auto_hit;
    reg auto_hit_fresh;      // auto_hit already reflects auto_addr
    reg auto_opened;         // Pair ACTIVATE issued, its READ not yet
    wire [2:0] auto_idx = {auto_addr[22], auto_addr[8:7]};
    wire auto_go = auto_ready && auto_hit_fresh && spi_txn_read_done;

    // A refresh may dispatch (modulo closing open rows first) under the
    // same conditions as the hard/soft dispatch branches below, except
    // that a pair burst between its ACTIVATE and READ keeps its row.
    // An open serial row is never closed for refresh, independent of the
    // client's inhibit hint (its READ/WRITE auto-precharges it).
    wire refresh_wanted = !auto_opened && !serial_row_open && (
        (hard_refresh_due && access_cmd == 0 &&
         (!spi_activate_done || spi_cmd_read_ack)) ||
        (refresh_due && !do_inhibit_refresh && spi_row_idle));
    
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
    wire [8:0]  spi_col  = {spi_addr[6:0], 2'b00};  // Burst-4 aligned base

    // Serial/glue path: 25-bit address = {chip, row, bank, col}
    //   [24]     = chip select
    //   [23:11]  = row (13 bits)
    //   [10:9]   = bank (2 bits)
    //   [8:0]    = column (9 bits, low 2 bits = burst offset)
    wire access_chip_sel  = access_addr[24];
    wire [12:0] access_row  = access_addr[23:11];
    wire [1:0]  access_bank = access_addr[10:9];
    wire [8:0]  access_col  = access_addr[8:0];

    // Read data pipeline. A READ dispatch enters stage 0; its beats
    // publish from stages RD_BEAT0 to RD_BEAT0+3 (CAS latency plus the
    // aux_clk capture register), one beat per clock (BL4 only). Consecutive SPI READs to the same chip dispatch
    // BURST_LEN clocks apart, so their beats stream back to back and at
    // most one READ is ever in stages 3-6. Each stage carries its READ's
    // target buffer and client so publishing needs no further state.
    localparam integer RD_BEAT0 = {27'd0, tCAS} + {27'd0, RD_PIPELINE_DELAY} + 1;
    localparam integer RD_STAGES = RD_BEAT0 + 4;
    reg [RD_STAGES-1:0] rd_pipe;
    reg [RD_STAGES-1:0] rd_tag_b;      // SPI fill into read_buffer_b
    reg [RD_STAGES-1:0] rd_tag_serial; // Serial-path read (always read_buffer)
    reg [RD_STAGES-1:0] rd_tag_w0_lo, rd_tag_w0_hi; // Start word of the READ
    // A READ is on its way to or on the DQ bus. Serial READ/WRITE, a READ
    // to the other chip (shared DQ bus) and the SPI fill reset wait for
    // this to clear, keeping the old READ-to-access spacing and bus
    // turnaround. ACTIVATE, PRECHARGE and REFRESH may follow BURST_LEN
    // after a READ: a PRECHARGE there never truncates the burst.
    wire rd_inflight = |rd_pipe[RD_STAGES-2:0];
    reg last_read_chip;
    wire [3:0] beat_stage = rd_pipe[RD_BEAT0 +: 4];
    wire beat_any = |beat_stage;
    wire [1:0] beat_k = beat_stage[0] ? 2'd0 : beat_stage[1] ? 2'd1 :
                        beat_stage[2] ? 2'd2 : 2'd3;
    wire beat_b = |(rd_tag_b[RD_BEAT0 +: 4] & beat_stage);
    wire beat_serial = |(rd_tag_serial[RD_BEAT0 +: 4] & beat_stage);
    // Buffer word of this beat: the burst wraps from its start word.
    wire [1:0] beat_w = beat_k + {|(rd_tag_w0_hi[RD_BEAT0 +: 4] & beat_stage),
                                  |(rd_tag_w0_lo[RD_BEAT0 +: 4] & beat_stage)};
    reg [2:0] wrbuf_read_ptr;

    // Read data capture: DQ sampled on aux_clk (phase-shifted for proper timing)
    // then used directly in the main clock domain read logic.
    reg [15:0] dq_captured;
    
    always @(posedge aux_clk) begin
        dq_captured <= dq_i;
    end

    integer i;

    always @(posedge clk) begin
        access_accept <= 0;
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
            cmd_wait <= 0;
            refreshcount <= 0;
            refresh_due <= 0;
            hard_refresh_due <= 0;
            refresh_chip <= 0;
            serial_row_open <= 0;
            open_valid <= 0;
            
            read_buffer <= 0;
            read_buffer_b <= 0;
            read_valid_a <= 0;
            read_valid_b <= 0;
            read_beats_a <= 0;
            read_beats_b <= 0;
            read_busy <= 0;
            rd_pipe <= 0;
            rd_tag_b <= 0;
            rd_tag_serial <= 0;
            rd_tag_w0_lo <= 0;
            rd_tag_w0_hi <= 0;
            last_read_chip <= 0;

            wrbuf_read_ptr <= 0;
            
            spi_active_buf <= 0;
            spi_inhibit_refresh_buf <= 0;
            spi_cmd_activate_buf <= 0;
            spi_cmd_read_buf <= 0;
            
            spi_cmd_activate_ack <= 0;
            spi_cmd_read_ack <= 0;
            spi_abort_pending <= 0;
            spi_activate_done <= 0;
            spi_read_continuation <= 0;
            spi_addr_latched <= 0;
            fill_sel <= 0;
            post_fill_sel <= 0;     // B goes to the pair fetch (see fill reset)
            fill_reset_armed <= 0;
            serial_read_active <= 0;
            spi_act_armed <= 1;
            spi_read_armed <= 1;
            spi_post_sync <= 0;
            spi_pair_buf <= 0;
            spi_pair_armed <= 1;
            spi_txn_read_done <= 0;
            auto_pending <= 0;
            auto_ready <= 0;
            auto_addr <= 0;
            auto_hit <= 0;
            spi_row_hit <= 0;
            spi_bank_open <= 0;
            spi_redirect_settle <= 0;
            auto_hit_fresh <= 0;
            auto_opened <= 0;
            spi_pair_read_done <= 0;
            spi_pair_window <= 0;
        end
        else begin
            // Read pipeline advances every clock; a READ dispatch below
            // enters stage 0.
            rd_pipe <= {rd_pipe[RD_STAGES-2:0], 1'b0};
            rd_tag_b <= {rd_tag_b[RD_STAGES-2:0], 1'b0};
            rd_tag_serial <= {rd_tag_serial[RD_STAGES-2:0], 1'b0};
            rd_tag_w0_lo <= {rd_tag_w0_lo[RD_STAGES-2:0], 1'b0};
            rd_tag_w0_hi <= {rd_tag_w0_hi[RD_STAGES-2:0], 1'b0};

            // Saturate at capacity, not the dispatch deadline: elapsed
            // service/deferral time remains owed until actually refreshed.
            if (!(&refreshcount))
                refreshcount <= refreshcount + 1'b1;
            refresh_due <= refreshcount >= REFRESH_SOFT_PRE;
            hard_refresh_due <= refreshcount >= REFRESH_HARD_PRE;
            
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
                spi_pair_armed <= 0;
            end
            else begin
                if (!spi_cmd_activate_buf[1] && spi_active_buf[1]) spi_act_armed <= 1;
                if (!spi_cmd_read_buf[1] && spi_active_buf[1]) spi_read_armed <= 1;
                // Both pair stages low: the third stage lags spi_active_buf
                // by a clock, so a stale-high level must not arm through it.
                if (spi_pair_buf[2:1] == 2'b00 && spi_active_buf[1]) spi_pair_armed <= 1;
            end
            // Burst pair: one per transaction, armed like the posts above.
            // A deselect drops a pair that has not opened a row yet; one
            // that has completes its READ (harmless, and it keeps tRAS).
            spi_pair_buf <= {spi_pair_buf[1:0], spi_cmd_pair && spi_active};
            spi_pair_window <= spi_pair_read_done && spi_pair_buf[1];
            if (spi_deselect_event) begin
                spi_txn_read_done <= 0;
                spi_pair_read_done <= 0;
                if (!auto_opened) begin
                    auto_pending <= 0;
                    auto_ready <= 0;
                end
            end
            else if (spi_pair_buf[2] && spi_pair_armed && spi_active_buf[1]) begin
                spi_pair_armed <= 0;
                auto_pending <= 1;
            end
            else if (auto_pending && !auto_ready && !spi_redirect_pending) begin
                auto_addr <= spi_addr2;
                auto_ready <= 1;
            end
            auto_hit <= open_valid[auto_idx] && open_rows[auto_idx] == auto_addr[21:9];
            spi_row_hit <= open_valid[spi_open_idx] &&
                           open_rows[spi_open_idx] == spi_addr[21:9];
            spi_bank_open <= open_valid[spi_open_idx];
            spi_redirect_settle <= spi_redirect_pending;
            auto_hit_fresh <= auto_ready;
            // A post always precedes its fill by ~10 sysclks, so
            // invalidating here can never clobber a completed fill.
            spi_post_sync <= {spi_post_sync[1:0], spi_cmd_post_toggle};
            if (spi_post_sync[1] ^ spi_post_sync[2]) begin
                if (post_fill_sel) begin read_valid_b <= 0; read_beats_b <= 0; end
                else begin read_valid_a <= 0; read_beats_a <= 0; end
                post_fill_sel <= ~post_fill_sel;
            end
            
            if (spi_cmd_activate_ack && !spi_cmd_activate_buf[1]) begin
                spi_cmd_activate_ack <= 0;
                // The SPI side can withdraw a post (re-arm drop or CS
                // release) after its ACTIVATE but before its READ was
                // dispatched, e.g. when a refresh delayed the pair. The row
                // is then still open: keep tracking it and close it.
                if (spi_cmd_read_ack || !spi_activate_done)
                    spi_activate_done <= 0;
                else
                    spi_abort_pending <= 1;
            end
            if (spi_cmd_read_ack && !spi_cmd_read_buf[1]) spi_cmd_read_ack <= 0;

            // Update busy flag
            // Busy when: any command is in progress (state != IDLE),
            // a new command has been posted (access_cmd != 0), or a
            // refresh is imminent and not inhibited.
            //
            // Explicit acceptance makes a one-cycle lookahead unnecessary:
            // requests that race refresh remain posted until dispatched.
            cmd_busy <= (state != STA_IDLE) ||
                        (access_cmd != 2'b00) ||
                        (refresh_due && !do_inhibit_refresh) ||
                        (hard_refresh_due && !serial_row_open);

            if (state == STA_INIT) begin
                // Wait for SDRAM power-up (100us) - chip 0 selected (CS LOW)
                /* verilator lint_off WIDTHEXPAND */
                if (initcount >= tINIT) begin
                /* verilator lint_on WIDTHEXPAND */
                    state <= STA_INIT_PRECHARGE;
                    cmdcount <= 1;
                    cmdtarget <= tRP;
                    cmd_wait <= tRP > 1;

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
            else if ((state != STA_IDLE) && cmd_wait) begin
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
                            3'd1: dq_o <= write_buffer[31:16];
                            3'd2: dq_o <= write_buffer[47:32];
                            3'd3: dq_o <= write_buffer[63:48];
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
                cmd_wait <= (cmdcount + 5'd1) < cmdtarget;
            end
            else begin
                // No command running, determine next command. Repeated init
                // refreshes reuse the target; new commands override below.
                cmdcount <= 1;
                cmd_wait <= cmdtarget > 1;

                if (state == STA_INIT_PRECHARGE) begin
                    state <= STA_INIT_REFRESH;
                    cmdtarget <= tRC;
                    cmd_wait <= tRC > 1;
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
                        cmd_wait <= tMRD > 1;
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
                        a_o[6:4] <= tCAS[2:0];  // CAS latency = 2 (tCAS)
                        a_o[3] <= 1'b0;         // Burst type: sequential (spi_word wraps)
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
                    cmd_wait <= tRP > 1;

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
                    cmd_wait <= tRC > 1;
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
                        cmd_wait <= tMRD > 1;

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
                        a_o[6:4] <= tCAS[2:0];
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
                // Refresh completion.  Handled inside the main dispatcher
                // (rather than a separate always-block hunk after it) so it
                // takes priority over any SPI fast-path or serial command
                // dispatch that might otherwise fire in the same cycle.  If
                // a post-dispatch hunk overrode the state back to a refresh
                // state, the command-side flags (spi_cmd_activate_ack /
                // spi_activate_done) set by the overridden dispatch would
                // persist, and the next cycle's READ would fire without a
                // matching ACTIVATE -- reading from a bank with no open row
                // and returning all 0xff from the floating DQ bus for the
                // rest of the SPI burst.
                else if (state == STA_REFRESH) begin
                    // One chip refreshed, back to idle. The other chip's
                    // event is due one event period later.
                    state <= STA_IDLE;
                    cmdtarget <= 1;
                    cmd_wait <= 0;
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
                    cmd_wait <= tRP > 1;
                    spi_activate_done <= 0;
                    open_valid[{spi_chip_sel, spi_bank}] <= 0;

                    cs_o <= spi_chip_sel;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    ba_o <= spi_bank;
                    a_o <= 0;
                    a_o[10] <= 0;
                    dqm_o <= 2'b11;
                end
                else if (state == STA_SPI_ABORT_PRECHARGE || state == STA_ROW_PRECHARGE) begin
                    state <= STA_IDLE;
                    cmdtarget <= 1;
                    cmd_wait <= 0;
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
                    cmd_wait <= tRAS > 1;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (hard_refresh_due && !serial_row_open && access_cmd == 0 &&
                         spi_activate_done && !spi_cmd_read_ack && !spi_cmd_read_buf[1]) begin
                    // SCK may stop after ACTIVATE while CS remains low.
                    // Close the row, refresh, then replay the held ACTIVATE.
                    state <= STA_SPI_ABORT_WAIT;
                    cmdtarget <= tRAS;
                    cmd_wait <= tRAS > 1;
                    spi_cmd_activate_ack <= 0;
                    spi_act_armed <= 1;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (auto_go && auto_hit &&
                         (!rd_inflight || last_read_chip == auto_addr[22])) begin
                    // Burst pair READ, row open: streams right behind the
                    // first burst (no auto-precharge, like any SPI READ).
                    state <= STA_READ;
                    cmdtarget <= BURST_LEN;
                    cmd_wait <= BURST_LEN > 1;
                    read_busy <= 1;
                    auto_pending <= 0;
                    auto_ready <= 0;
                    auto_opened <= 0;
                    spi_pair_read_done <= 1;
                    rd_pipe[0] <= 1;
                    rd_tag_b[0] <= fill_sel;
                    rd_tag_serial[0] <= 0;
                    last_read_chip <= auto_addr[22];
                    fill_sel <= ~fill_sel;
                    if (fill_sel) begin read_valid_b <= 0; read_beats_b <= 0; end
                    else begin read_valid_a <= 0; read_beats_a <= 0; end

                    cs_o <= auto_addr[22];
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= auto_addr[8:7];
                    a_o <= 0;
                    a_o[8:0] <= {auto_addr[6:0], 2'b00};
                    a_o[10] <= 0;
                    dq_oe_o <= 0;
                    dqm_o <= 2'b00;
                end
                else if (auto_go && open_valid[auto_idx]) begin
                    // Pair burst in a bank holding another row: close it.
                    // That row was opened before the first burst's READ
                    // dispatched, so tRAS is met, and a PRECHARGE BURST_LEN
                    // after a READ never truncates its data.
                    state <= STA_ROW_PRECHARGE;
                    cmdtarget <= tRP;
                    cmd_wait <= tRP > 1;
                    open_valid[auto_idx] <= 0;
                    cs_o <= auto_addr[22];
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    ba_o <= auto_addr[8:7];
                    a_o <= 0;
                    a_o[10] <= 0;
                    dqm_o <= 2'b11;
                end
                else if (auto_go) begin
                    // Pair burst crosses into a closed bank.
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;
                    cmd_wait <= tRCD > 1;
                    auto_opened <= 1;
                    open_rows[auto_idx] <= auto_addr[21:9];
                    open_valid[auto_idx] <= 1;
                    cs_o <= auto_addr[22];
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                    ba_o <= auto_addr[8:7];
                    a_o <= auto_addr[21:9];
                end
                else if (refresh_wanted && refresh_open != 0) begin
                    // Refresh needs every bank of its chip closed; close
                    // them with one precharge-all and let refresh dispatch
                    // after tRP. refresh_wanted excludes a row still
                    // between ACTIVATE and READ (the replay branch above
                    // owns that case), and dispatch only runs after the
                    // previous burst completed, so tRAS is satisfied.
                    state <= STA_ROW_PRECHARGE;
                    cmdtarget <= tRP;
                    cmd_wait <= tRP > 1;
                    if (refresh_chip) open_valid[7:4] <= 0;
                    else open_valid[3:0] <= 0;
                    cs_o <= refresh_chip;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    ba_o <= 0;
                    a_o <= 0;
                    a_o[10] <= 1;
                    dqm_o <= 2'b11;
                end
                else if (hard_refresh_due && !serial_row_open && access_cmd == 0 &&
                         (!spi_activate_done || spi_cmd_read_ack) && refresh_open == 0) begin
                    state <= STA_REFRESH;
                    cmdtarget <= tRC;
                    cmd_wait <= tRC > 1;
                    refreshcount <= refreshcount - REFRESH_EVENT + 1'b1;
                    refresh_due <= refreshcount >= REFRESH_AGAIN_PRE;
                    hard_refresh_due <= {1'b0, refreshcount} >= REFRESH_BACKLOG_PRE;
                    cs_o <= refresh_chip;
                    refresh_chip <= ~refresh_chip;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (spi_cmd_activate_buf[1] && !spi_cmd_activate_ack && spi_act_armed &&
                         !serial_row_open && !spi_redirect_pending && !spi_redirect_settle &&
                         !auto_opened && !(fill_reset_armed && rd_inflight) && spi_row_hit) begin
                    // SPI fast-path activate, row already open: acknowledge
                    // without issuing ACTIVATE; READ may dispatch next cycle
                    // (saves the ACTIVATE command and tRCD).
                    state <= STA_IDLE;
                    cmdtarget <= 1;
                    cmd_wait <= 0;
                    spi_cmd_activate_ack <= 1;
                    spi_activate_done <= 1;
                    spi_addr_latched <= spi_addr;
                    spi_act_armed <= 0;
                    if (fill_reset_armed) begin
                        // A fill still in flight from the previous
                        // transaction must land first (see the guard).
                        // The pair fetch takes buffer B without a post
                        // toggle, so the first toggle targets A again.
                        fill_sel <= 0;
                        post_fill_sel <= 0;
                        read_valid_a <= 0;
                        read_valid_b <= 0;
                        read_beats_a <= 0;
                        read_beats_b <= 0;
                        fill_reset_armed <= 0;
                    end
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else if (spi_cmd_activate_buf[1] && !spi_cmd_activate_ack && spi_act_armed &&
                         !serial_row_open && !spi_redirect_pending && !spi_redirect_settle &&
                         !auto_opened && spi_bank_open) begin
                    // A different row is open in this bank: close it first.
                    // Not an acceptance: the post levels stay up and
                    // ACTIVATE dispatches after tRP. The previous burst
                    // completed before this dispatch point, so tRAS is met.
                    state <= STA_ROW_PRECHARGE;
                    cmdtarget <= tRP;
                    cmd_wait <= tRP > 1;
                    open_valid[spi_open_idx] <= 0;
                    cs_o <= spi_addr[22];
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    ba_o <= spi_addr[8:7];
                    a_o <= 0;
                    a_o[10] <= 0;
                    dqm_o <= 2'b11;
                end
                else if (spi_cmd_activate_buf[1] && !spi_cmd_activate_ack && spi_act_armed &&
                         !serial_row_open && !spi_redirect_pending && !spi_redirect_settle &&
                         !auto_opened && !(fill_reset_armed && rd_inflight)) begin
                    // SPI fast-path activate
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;
                    cmd_wait <= tRCD > 1;
                    spi_cmd_activate_ack <= 1;
                    spi_activate_done <= 1;
                    spi_addr_latched <= spi_addr;
                    spi_act_armed <= 0;
                    open_rows[spi_open_idx] <= spi_addr[21:9];
                    open_valid[spi_open_idx] <= 1;
                    if (fill_reset_armed) begin
                        // A fill still in flight from the previous
                        // transaction must land first (see the guard).
                        // The pair fetch takes buffer B without a post
                        // toggle, so the first toggle targets A again.
                        fill_sel <= 0;
                        post_fill_sel <= 0;
                        read_valid_a <= 0;
                        read_valid_b <= 0;
                        read_beats_a <= 0;
                        read_beats_b <= 0;
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
                else if (spi_cmd_read_buf[1] && !spi_cmd_read_ack && spi_activate_done && spi_read_armed &&
                         !serial_row_open && !auto_opened &&
                         (!rd_inflight || last_read_chip == spi_chip_sel)) begin
                    // SPI fast-path read. Same-chip READs stream BURST_LEN
                    // clocks apart (see the read data pipeline).
                    state <= STA_READ;
                    cmdtarget <= BURST_LEN;
                    cmd_wait <= BURST_LEN > 1;
                    read_busy <= 1;
                    spi_cmd_read_ack <= 1;
                    spi_read_armed <= 0;
                    spi_read_continuation <= spi_cmd_continuation;
                    spi_txn_read_done <= 1;
                    rd_pipe[0] <= 1;
                    rd_tag_b[0] <= fill_sel;
                    rd_tag_serial[0] <= 0;
                    rd_tag_w0_lo[0] <= spi_word[0];
                    rd_tag_w0_hi[0] <= spi_word[1];
                    last_read_chip <= spi_chip_sel;
                    // Fills alternate in dispatch order. Invalidate the
                    // target up front; its beats set the flags again.
                    fill_sel <= ~fill_sel;
                    if (fill_sel) begin read_valid_b <= 0; read_beats_b <= 0; end
                    else begin read_valid_a <= 0; read_beats_a <= 0; end

                    // READ command, no auto-precharge: the row stays open
                    // for the next burst (see open-row tracking above).
                    cs_o <= spi_chip_sel;
                    ras_o <= 1;
                    cas_o <= 0;
                    we_o <= 1;
                    ba_o <= spi_bank;
                    a_o <= 0;
                    a_o[8:0] <= {spi_col[8:2], spi_word};
                    a_o[10] <= 0;
                    dq_oe_o <= 0;
                    dqm_o <= 2'b00;
                end
                else if (access_cmd == 2'b11 && spi_row_idle && !auto_opened &&
                         open_valid[{access_chip_sel, access_bank}]) begin
                    // Close the row left open in the serial target bank
                    // first; not an acceptance (the client holds its
                    // request until the ACTIVATE below dispatches).
                    state <= STA_ROW_PRECHARGE;
                    cmdtarget <= tRP;
                    cmd_wait <= tRP > 1;
                    open_valid[{access_chip_sel, access_bank}] <= 0;
                    cs_o <= access_chip_sel;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 0;
                    ba_o <= access_bank;
                    a_o <= 0;
                    a_o[10] <= 0;
                    dqm_o <= 2'b11;
                end
                else if (access_cmd == 2'b11 && spi_row_idle && !auto_opened) begin
                    // Serial path activate
                    access_accept <= 1;
                    serial_row_open <= 1;
                    open_rows[{access_chip_sel, access_bank}] <= access_row;
                    open_valid[{access_chip_sel, access_bank}] <= 1;
                    state <= STA_ACTIVATE;
                    cmdtarget <= tRCD;
                    cmd_wait <= tRCD > 1;

                    // ACTIVATE command
                    cs_o <= access_chip_sel;
                    ras_o <= 0;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                    ba_o <= access_bank;
                    a_o <= access_row;
                end
                else if (access_cmd == 2'b01 && spi_row_idle && !auto_opened && !rd_inflight) begin
                    // Serial path read (auto-precharge closes its row)
                    access_accept <= 1;
                    serial_row_open <= 0;
                    open_valid[{access_chip_sel, access_bank}] <= 0;
                    state <= STA_READ;
                    cmdtarget <= tREAD + 2;
                    cmd_wait <= (tREAD + 5'd2) > 1;
                    read_busy <= 1;
                    serial_read_active <= 1;
                    rd_pipe[0] <= 1;
                    rd_tag_b[0] <= 0;
                    rd_tag_serial[0] <= 1;
                    last_read_chip <= access_chip_sel;
                    read_valid_a <= 0;
                    read_beats_a <= 0;

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
                else if (access_cmd == 2'b10 && spi_row_idle && !auto_opened && !rd_inflight) begin
                    // Serial path write (auto-precharge closes its row)
                    access_accept <= 1;
                    serial_row_open <= 0;
                    open_valid[{access_chip_sel, access_bank}] <= 0;
                    state <= STA_WRITE;
                    cmdtarget <= tWRITE;
                    cmd_wait <= tWRITE > 1;
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
                else if (refresh_due && !do_inhibit_refresh && spi_row_idle && refresh_open == 0) begin
                    // Auto refresh - alternate between chips
                    state <= STA_REFRESH;
                    cmdtarget <= tRC;
                    cmd_wait <= tRC > 1;
                    // Pay one event period, retaining overdue time.
                    // Resetting here halves the nominal rate under
                    // sustained inhibit.
                    refreshcount <= refreshcount - REFRESH_EVENT + 1'b1;
                    refresh_due <= refreshcount >= REFRESH_AGAIN_PRE;
                    hard_refresh_due <= {1'b0, refreshcount} >= REFRESH_BACKLOG_PRE;

                    // REFRESH command for current chip
                    cs_o <= refresh_chip;
                    refresh_chip <= ~refresh_chip;
                    ras_o <= 0;
                    cas_o <= 0;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
                else begin
                    state <= STA_IDLE;
                    cmdtarget <= 1;  // Dispatch runs every cycle when idle
                    cmd_wait <= 0;
                    ras_o <= 1;
                    cas_o <= 1;
                    we_o <= 1;
                    dqm_o <= 2'b11;
                end
            end

            // The REFRESH -> IDLE transition is handled inside the main
            // dispatcher (as the STA_REFRESH case) so it takes priority
            // over SPI and serial command dispatches in the same cycle.
            
            // Keep read data unmasked until the last beat has left the
            // SDRAM (DQM read latency 2), whatever dispatches meanwhile.
            if (rd_inflight) dqm_o <= 2'b00;

            // Read data capture: each beat's bytes publish straight into
            // the target buffer (progressive publish, see layout note at the
            // top of this file). Byte-serial: beat w completes bytes 2w/2w+1
            // (serial: A; SPI: the fill target tagged at dispatch). Only
            // the first byte(s) of a burst are timing-critical; later bytes
            // are ready long before the SPI side shifts them out, and the
            // target is never the half being shifted out, so the SPI side
            // never observes a torn burst. read_valid marks beat 0 for the
            // thin-margin check; read_beats marks each beat for the actual
            // output checks, including high-offset short reads. (Explicit
            // lanes rather than variable part-selects: safest across
            // Yosys, Verilator, and Gowin synthesis.)
            if (beat_any) begin
                if (beat_serial || !beat_b) begin
                    read_beats_a[beat_w] <= 1;
                    case (beat_w)
                        2'd0: read_buffer[15:0] <= dq_captured;
                        2'd1: read_buffer[31:16] <= dq_captured;
                        2'd2: read_buffer[47:32] <= dq_captured;
                        2'd3: read_buffer[63:48] <= dq_captured;
                    endcase
                    if (beat_k == 2'd0) read_valid_a <= 1;
                end
                else begin
                    read_beats_b[beat_w] <= 1;
                    case (beat_w)
                        2'd0: read_buffer_b[15:0] <= dq_captured;
                        2'd1: read_buffer_b[31:16] <= dq_captured;
                        2'd2: read_buffer_b[47:32] <= dq_captured;
                        2'd3: read_buffer_b[63:48] <= dq_captured;
                    endcase
                    if (beat_k == 2'd0) read_valid_b <= 1;
                end
                if (beat_k == 2'd3) begin
                    // Busy until the last streamed READ has published.
                    read_busy <= |rd_pipe[RD_BEAT0-1:0];
                    if (beat_serial) begin
                        // A serial read restarts the SPI pairing at A.
                        read_valid_b <= 0;
                        read_beats_b <= 0;
                        fill_sel <= 0;
                        serial_read_active <= 0;
                    end
                end
            end
        end
    end

endmodule
