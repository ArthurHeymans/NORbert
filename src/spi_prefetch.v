// SDRAM burst prefetch for SPI reads (spi_clk domain).
//
// Owns the request handshake to the SDRAM controller and the ping-pong
// buffer selection, so spi_trx only reports *when* things happen in the
// SPI transaction. All inputs are single-clock event strobes computed from
// spi_trx state; they are applied in a fixed order, so when several fire on
// the same clock the later one wins exactly as listed below.
//
// Handshake: ram_inhibit_refresh/ram_activate/ram_read are levels. The
// controller acts on a rising request, so every post must be preceded by a
// drop ("re-arm") at least one SPI clock earlier. ram_addr is the 23-bit
// burst address (byte_addr[25:3]) wrapped to the configured flash size.
//
// One-burst lookahead: the first burst is requested during the address
// phase (ACTIVATE once row/bank are known, READ once the column is known).
// The second burst is posted during the dummy/mode phase when there is
// one, otherwise two clocks after the first data clock. From then on the
// next burst is always in flight while the current one is shifted out: the
// controller fills the idle half of ram_read_buffer/ram_read_buffer_b and
// consume_sel tracks the half currently being shifted out.

`default_nettype none

module spi_prefetch(
    input wire spi_clk,
    input wire active,               // Selected and out of reset
    input wire restart,              // First clock of a transaction
    input wire [22:0] addr_mask,     // Burst-address mask (flash size - 1)

    input wire fresh_read,           // spi_trx starts a new burst
    input wire in_read,              // spi_trx is in its data output phase

    // First burst, from the address phase of array reads
    input wire first_inhibit,        // Row/bank about to be known
    input wire first_activate,       // Row/bank known
    input wire [15:0] first_row,     // Burst address bits [22:7]
    input wire first_read,           // Column known
    input wire [6:0] first_col,      // Burst address bits [6:0]
    input wire first_done,           // Single-lane address phase ended
    input wire hold_inhibit,         // ...and a dummy phase follows

    input wire release_inhibit,      // Dummy phase ends
    input wire post_lookahead,       // Post the second burst mid dummy/mode
    input wire drop,                 // Re-arm the handshake
    input wire mode_end,             // Mode phase ends
    input wire fallback,             // First clock of byte 7 in a burst
    input wire burst_end,            // Last clock of byte 7 in a burst

    output reg ram_inhibit_refresh = 0,
    output reg ram_activate = 0,
    output reg ram_read = 0,
    output reg ram_continuation = 0, // Only subsequent bursts may be redirected
    // Toggles on every lookahead post (dummy, mode and first-clock posts).
    // The controller invalidates the upcoming fill target on each edge, so
    // a burst that never fills (blocked post, lost race) is always flagged
    // by the underrun check instead of aliasing a stale buffer.
    output reg ram_post_toggle = 0,
    output reg [22:0] ram_addr,

    input wire [63:0] ram_read_buffer,
    input wire [63:0] ram_read_buffer_b,
    input wire ram_read_valid_a,
    input wire ram_read_valid_b,

    // Burst buffer currently shifted out. The controller only ever writes
    // the idle half, so this view is stable for the whole burst.
    output wire [63:0] live_buffer,

    // Sticky diagnostics, cleared at the start of every transaction. See
    // the burst_end handling below for their exact meaning.
    output reg prefetch_underrun = 0,
    output reg prefetch_thin = 0
);

    // Half of the ping-pong pair being shifted out. Toggles at every burst
    // end in lockstep with the controller's fill toggle; both restart at A
    // on every CS drop, so pairing cannot drift across transactions.
    reg consume_sel = 0;
    // Set whenever the next burst has been requested during the current
    // one and cleared at every burst end. The byte-7 fallback may therefore
    // only fire for a first burst that started at offset 7 with no post
    // yet; otherwise it would bump ram_addr a second time without a
    // matching dispatch and walk the ping-pong pairing out of phase.
    reg posted_this_burst = 0;
    // Set when the second burst was already posted during a dummy/mode
    // phase: the first data clock then only clears the flag instead of
    // posting a duplicate request.
    reg prefetch_pending = 0;
    // Two-cycle delayed fresh_read: the lookahead post fires here so the
    // burst-end drop always precedes it by >= 2 SPI clocks, even for
    // 2-clock offset-7 first bursts that have no byte 6 to drop on.
    reg post_arm1 = 0;
    reg post_arm2 = 0;

    assign live_buffer = consume_sel ? ram_read_buffer_b : ram_read_buffer;

    wire [22:0] next_burst = (ram_addr + 1'b1) & addr_mask;

    always @(posedge spi_clk) begin
        if (active) begin
            if (restart) begin
                ram_inhibit_refresh <= 0;
                ram_activate <= 0;
                ram_read <= 0;
                ram_continuation <= 0;
                ram_post_toggle <= 0;
                consume_sel <= 0;
                posted_this_burst <= 0;
                prefetch_pending <= 0;
                post_arm1 <= 0;
                post_arm2 <= 0;
                prefetch_underrun <= 0;
                prefetch_thin <= 0;
            end
            else begin
                // Lookahead post two clocks into every burst. With a
                // dummy/mode-posted second burst there is nothing to post;
                // just clear the flag (the burst-end drop already re-armed).
                post_arm1 <= fresh_read;
                post_arm2 <= post_arm1;
                if (post_arm2 && in_read) begin
                    if (prefetch_pending)
                        prefetch_pending <= 0;
                    else if (!posted_this_burst) begin
                        ram_continuation <= 1;
                        ram_inhibit_refresh <= 1;
                        ram_activate <= 1;
                        ram_read <= 1;
                        ram_addr <= next_burst;
                        posted_this_burst <= 1;
                        ram_post_toggle <= ~ram_post_toggle;
                    end
                end

                // First burst: ACTIVATE as soon as row and bank are known,
                // READ when the column arrives.
                if (first_inhibit)
                    ram_inhibit_refresh <= 1;
                if (first_activate) begin
                    ram_activate <= 1;
                    ram_addr[22:7] <= first_row & addr_mask[22:7];
                end
                if (first_read) begin
                    ram_read <= 1;
                    ram_addr[6:0] <= first_col & addr_mask[6:0];
                end
                if (first_done) begin
                    ram_activate <= 0;
                    ram_read <= 0;
                    // Keep refresh inhibited across the dummy phase for fast
                    // reads: the second burst posts mid-dummy and a refresh
                    // starting in the gap would delay it past its need.
                    // (Slow 0x03 drops here, preserving the refresh-overlap
                    // window its first burst relies on for trap timing
                    // coverage.) Inhibit is released at the dummy end.
                    if (!hold_inhibit)
                        ram_inhibit_refresh <= 0;
                end

                // The second burst is already dispatched by the dummy end;
                // later bursts re-assert per burst with refresh gaps.
                if (release_inhibit)
                    ram_inhibit_refresh <= 0;

                // Mid dummy/mode post of the second burst, so short first
                // bursts (high start offsets) still meet SDRAM latency at
                // fast SCLK. The first burst's levels were dropped before,
                // so the handshake re-arms across a multi-clock gap.
                if (post_lookahead) begin
                    ram_continuation <= 1;
                    ram_inhibit_refresh <= 1;
                    ram_activate <= 1;
                    ram_read <= 1;
                    ram_addr <= next_burst;
                    prefetch_pending <= 1;
                    posted_this_burst <= 1;
                    ram_post_toggle <= ~ram_post_toggle;
                end

                // Re-arm: mid mode phase, and on the last clock of byte 6
                // (the gap across byte 7 also leaves room for a refresh).
                if (drop) begin
                    ram_inhibit_refresh <= 0;
                    ram_activate <= 0;
                    ram_read <= 0;
                end

                // Keep mode-posted levels up: the second burst is still
                // filling and byte 6 of the first burst drops them.
                if (mode_end && !prefetch_pending) begin
                    ram_inhibit_refresh <= 0;
                    ram_activate <= 0;
                    ram_read <= 0;
                end

                // Fallback when a read starts directly at byte 7 and no
                // lookahead post happened yet.
                if (fallback && !ram_activate && !posted_this_burst) begin
                    ram_continuation <= 1;
                    ram_inhibit_refresh <= 1;
                    ram_activate <= 1;
                    ram_read <= 1;
                    ram_addr <= next_burst;
                    posted_this_burst <= 1;
                end

                if (burst_end) begin
                    consume_sel <= ~consume_sel;
                    // Both flags sample the system-clock valids directly
                    // (async). Benign: these are sticky diagnostics, never
                    // data-path. A transitioning sample means its fill just
                    // completed (data is fine either way); only a stable 0
                    // flags.
                    //
                    // Underrun: the buffer just consumed (old consume_sel)
                    // was filled at least a full burst ago, so a clear bit
                    // means the fill never happened.
                    if (consume_sel ? !ram_read_valid_b : !ram_read_valid_a)
                        prefetch_underrun <= 1;
                    // Thin margin: the upcoming buffer (~old consume_sel)
                    // has not completed its fill yet. Data may still arrive
                    // in time (beats land progressively), so this flags
                    // thin margin, not corruption.
                    if (consume_sel ? !ram_read_valid_a : !ram_read_valid_b)
                        prefetch_thin <= 1;
                    // Burst-end drop re-arms the handshake for the delayed
                    // post two clocks later and restarts the posted flag.
                    // (Offset-7 first bursts have no byte 6; this is their
                    // only drop.)
                    ram_inhibit_refresh <= 0;
                    ram_activate <= 0;
                    ram_read <= 0;
                    posted_this_burst <= 0;
                end
            end
        end
    end

endmodule
