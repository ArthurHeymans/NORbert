// SDRAM burst prefetch for SPI reads (spi_clk domain).
//
// Owns the request handshake to the SDRAM controller and the ping-pong
// buffer selection, so spi_trx only reports *when* things happen in the
// SPI transaction. All inputs are single-clock event strobes computed from
// spi_trx state; they are applied in a fixed order, so when several fire on
// the same clock the later one wins exactly as listed below.
//
// Handshake: ram_activate/ram_read are levels. The controller acts on a
// rising request, so every post must be preceded by a drop ("re-arm") at
// least one SPI clock earlier. ram_addr is the 23-bit burst address
// (byte_addr[25:3]) wrapped to the configured flash size.
//
// Refresh: ram_inhibit_refresh rises on the first clock of every
// transaction and stays up for the whole of an array read; it drops at the
// opcode only for other commands. During a read the controller may refresh
// only in the window between dispatching a lookahead (continuation) READ
// and the re-arm drop of that post. A refresh started there can at most
// overlap the few clocks before the next post, while one started in the
// gap before a post could delay that post by a whole refresh event.
//
// One-burst lookahead: the first burst is requested during the address
// phase (ACTIVATE once row/bank are known, READ once the column is known).
// The second burst is not posted: ram_pair rises when the address phase
// ends and the controller fetches ram_addr2 right behind the first burst,
// without a second synchronized handshake. That is what a read starting
// at byte 7 needs, whose second burst is due a few clocks after the
// address. From the second burst on, the next burst is posted two clocks
// into each burst, so it is always in flight while the current one is
// shifted out: the controller fills the idle half of
// ram_read_buffer/ram_read_buffer_b and consume_sel tracks the half
// currently being shifted out.

`default_nettype none

module spi_prefetch(
    input wire spi_clk,
    input wire active,               // Selected and out of reset
    input wire restart,              // First clock of a transaction
    input wire [22:0] addr_mask,     // Burst-address mask (flash size - 1)

    input wire fresh_read,           // spi_trx starts a new burst
    input wire in_read,              // spi_trx is in its data output phase
    input wire sample_missing,       // Previous falling-edge output lacked its beat
    // One-hot {buffer B byte 7..0, buffer A byte 7..0} of the first byte
    // of the burst being started, registered by spi_trx on the clock that
    // sets fresh_read. Only meaningful while fresh_read is set.
    input wire [15:0] fresh_sel,

    input wire release_inhibit,      // Opcode decoded: not an array read

    // First burst, from the address phase of array reads
    input wire first_activate,       // Row/bank known
    input wire [15:0] first_row,     // Burst address bits [22:7]
    input wire first_read,           // Column known
    input wire [6:0] first_col,      // Burst address bits [6:0]
    // Start word of the first burst, from the byte-address bits known
    // together with the column (all of [2:1] for quad, [2] for dual).
    input wire [1:0] first_word,
    input wire first_done,           // Single-lane address phase ended
    input wire addr_end,             // Array-read address phase ended

    input wire drop,                 // Re-arm the handshake
    input wire burst_end,            // Last clock of byte 7 in a burst

    output reg ram_inhibit_refresh = 0,
    output reg ram_activate = 0,
    output reg ram_read = 0,
    output reg ram_continuation = 0, // Only subsequent bursts may be redirected
    // Toggles on every lookahead post. The controller invalidates the
    // upcoming fill target on each edge, so a burst that never fills
    // (blocked post, lost race) is always flagged by the underrun check
    // instead of aliasing a stale buffer. The pair fetch does not toggle;
    // the controller accounts for it when a transaction starts.
    output reg ram_post_toggle = 0,
    output reg [22:0] ram_addr,
    // SDRAM word the READ starts at; the burst wraps (critical word
    // first). Bundled with the column bits of ram_addr.
    output reg [1:0] ram_word = 0,
    // Second burst, fetched by the controller once ram_pair is seen.
    // ram_addr2 is set with the first burst's column and held until the
    // next transaction's; ram_pair is one level per transaction. It drops
    // at the first burst's re-arm point, closing the refresh window the
    // pair READ opens (see sdram.v), like the re-arm of a post.
    output reg ram_pair = 0,
    output reg [22:0] ram_addr2 = 0,

    input wire [63:0] ram_read_buffer,
    input wire [63:0] ram_read_buffer_b,
    input wire ram_read_valid_a,
    input wire ram_read_valid_b,
    input wire [3:0] ram_read_beats_a,
    input wire [3:0] ram_read_beats_b,

    // Burst buffer currently shifted out. The controller only ever writes
    // the idle half, so this view is stable for the whole burst.
    output wire [63:0] live_buffer,
    output wire [3:0] live_beats,
    // First byte of the burst being started and the readiness of its
    // beat. The falling-edge output samples them half an SCK period after
    // fresh_read is set, against beats that may publish in that same half
    // period, so they are a plain AND-OR of the two buffers with the
    // registered one-hot fresh_sel: no buffer or offset decode and no
    // high-fanout select in that half period.
    output wire [7:0] fresh_byte,
    output wire fresh_ready,
    // Half being shifted out (B when set), for spi_trx's fresh_sel.
    output wire consume_b,

    // Sticky diagnostics, cleared at the start of every transaction.
    // Underrun is checked on each master's data sampling edge, including
    // short reads; thin still checks first-beat readiness at a burst boundary.
    output reg prefetch_underrun = 0,
    output reg prefetch_thin = 0
);

    // Half of the ping-pong pair being shifted out. Toggles at every burst
    // end in lockstep with the controller's fill toggle; both restart at A
    // on every CS drop, so pairing cannot drift across transactions.
    reg consume_sel = 0;
    // Set whenever the next burst has been requested during the current
    // one and cleared at every burst end, so each burst posts once. The
    // pair counts as the first burst's post.
    reg posted_this_burst = 0;
    // The pair fetched the burst after ram_addr, so the next post skips it.
    reg pair_skip = 0;
    // Two-cycle delayed fresh_read: the lookahead post fires here so the
    // burst-end drop always precedes it by >= 1 SPI clock, even for
    // 2-clock offset-7 first bursts that have no byte 6 to drop on.
    reg post_arm1 = 0;
    reg post_arm2 = 0;

    assign live_buffer = consume_sel ? ram_read_buffer_b : ram_read_buffer;
    assign live_beats = consume_sel ? ram_read_beats_b : ram_read_beats_a;

    assign consume_b = consume_sel;

    wire [127:0] both_buffers = {ram_read_buffer_b, ram_read_buffer};
    // Beat w carries bytes 2w and 2w+1.
    wire [15:0] byte_ready = {ram_read_beats_b[3], ram_read_beats_b[3], ram_read_beats_b[2],
                              ram_read_beats_b[2], ram_read_beats_b[1], ram_read_beats_b[1],
                              ram_read_beats_b[0], ram_read_beats_b[0],
                              ram_read_beats_a[3], ram_read_beats_a[3], ram_read_beats_a[2],
                              ram_read_beats_a[2], ram_read_beats_a[1], ram_read_beats_a[1],
                              ram_read_beats_a[0], ram_read_beats_a[0]};
    reg [7:0] fresh_or;
    integer lane;
    always @* begin
        fresh_or = 0;
        for (lane = 0; lane < 16; lane = lane + 1)
            fresh_or = fresh_or | (both_buffers[lane*8 +: 8] & {8{fresh_sel[lane]}});
    end
    assign fresh_byte = fresh_or;
    assign fresh_ready = |(byte_ready & fresh_sel);

    wire [22:0] next_burst = (ram_addr + (pair_skip ? 23'd2 : 23'd1)) & addr_mask;

    always @(posedge spi_clk) begin
        if (active) begin
            if (restart) begin
                // Assume an array read until the opcode says otherwise, so
                // no refresh can start late enough to delay the first burst.
                ram_inhibit_refresh <= 1;
                ram_activate <= 0;
                ram_read <= 0;
                ram_continuation <= 0;
                ram_post_toggle <= 0;
                consume_sel <= 0;
                posted_this_burst <= 0;
                pair_skip <= 0;
                ram_pair <= 0;
                post_arm1 <= 0;
                post_arm2 <= 0;
                prefetch_underrun <= 0;
                prefetch_thin <= 0;
            end
            else begin
                // The SPI output path samples beat validity alongside its
                // data. Do not let a later fill erase an earlier missing byte.
                if (in_read && sample_missing)
                    prefetch_underrun <= 1;
                // Lookahead post two clocks into every burst (the first
                // burst's was the pair).
                post_arm1 <= fresh_read;
                post_arm2 <= post_arm1;
                if (post_arm2 && in_read && !posted_this_burst) begin
                    ram_continuation <= 1;
                    ram_activate <= 1;
                    ram_read <= 1;
                    ram_addr <= next_burst;
                    ram_word <= 0;
                    pair_skip <= 0;
                    posted_this_burst <= 1;
                    ram_post_toggle <= ~ram_post_toggle;
                end

                if (release_inhibit)
                    ram_inhibit_refresh <= 0;

                // First burst: ACTIVATE as soon as row and bank are known,
                // READ when the column arrives.
                if (first_activate) begin
                    ram_activate <= 1;
                    ram_addr[22:7] <= first_row & addr_mask[22:7];
                end
                if (first_read) begin
                    ram_read <= 1;
                    ram_addr[6:0] <= first_col & addr_mask[6:0];
                    ram_word <= first_word;
                    ram_addr2 <= ({ram_addr[22:7], first_col & addr_mask[6:0]} + 1'b1) &
                                 addr_mask;
                end
                if (first_done) begin
                    ram_activate <= 0;
                    ram_read <= 0;
                end
                // The whole address is known: the controller may fetch the
                // second burst (after the address event, see sdram.v).
                if (addr_end) begin
                    ram_pair <= 1;
                    pair_skip <= 1;
                    posted_this_burst <= 1;
                end

                // Re-arm: mid mode phase, and on the last clock of byte 6.
                // This also closes the refresh window of the post.
                if (drop) begin
                    ram_activate <= 0;
                    ram_read <= 0;
                    if (in_read) ram_pair <= 0;
                end

                if (burst_end) begin
                    consume_sel <= ~consume_sel;
                    // First-beat readiness is a margin diagnostic only.
                    // The consumed-byte check above uses individual beats;
                    // a fill after consumption cannot hide a stale output.
                    // These bundled-data flags cross with their data and
                    // require the same bundled-data timing contract.
                    if (consume_sel ? !ram_read_valid_a : !ram_read_valid_b)
                        prefetch_thin <= 1;
                    // Burst-end drop re-arms the handshake for the delayed
                    // post two clocks later and restarts the posted flag.
                    // (Offset-7 first bursts have no byte 6; this is their
                    // only drop.)
                    ram_activate <= 0;
                    ram_read <= 0;
                    ram_pair <= 0;
                    posted_this_burst <= 0;
                end
            end
        end
    end

endmodule
