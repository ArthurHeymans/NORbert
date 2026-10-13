// SDRAM burst prefetch for SPI reads (spi_clk domain).
//
// Owns the request handshake to the SDRAM controller and the ring
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
// Lookahead over a four-slot ring: the first burst is requested during
// the address phase (ACTIVATE once row/bank are known, READ once the
// column is known). The second burst is posted during the dummy/mode
// phase when there is one, otherwise two clocks after the first data
// clock. When a dummy/mode phase posted the second burst, a third is
// primed during the first data burst: once the second burst's first
// beat publishes (proof its READ dispatched), its levels drop and the
// third burst posts two clocks later, holding off the next drop so its
// own dispatch cannot be cut short. From then on two bursts are in
// flight or filled ahead of the one being shifted out; without a dummy
// phase there is no early post to prime from and the depth stays one.
// ahead counts posts beyond the consume burst (capped at 2, leaving one
// slot spare so a fill can never target the slot being consumed).
// A post only fires while the request levels are down (!ram_activate):
// levels still up mean the previous post has not been re-armed, and
// raising them again would merge two posts into one dispatch.

`default_nettype none

module spi_prefetch(
    input wire spi_clk,
    input wire active,               // Selected and out of reset
    input wire restart,              // First clock of a transaction
    input wire [22:0] addr_mask,     // Burst-address mask (flash size - 1)

    input wire fresh_read,           // spi_trx starts a new burst
    input wire in_read,              // spi_trx is in its data output phase
    input wire sample_missing,       // Previous falling-edge output lacked its beat
    // Byte offset of the burst being started; only sampled while
    // fresh_read is set (the first byte after STA_READ entry has the
    // transaction start offset, later bursts always start at 0).
    input wire [2:0] burst_offset,

    input wire release_inhibit,      // Opcode decoded: not an array read

    // First burst, from the address phase of array reads
    input wire first_activate,       // Row/bank known
    input wire [15:0] first_row,     // Burst address bits [22:7]
    input wire first_read,           // Column known
    input wire [6:0] first_col,      // Burst address bits [6:0]
    input wire first_done,           // Single-lane address phase ended

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

    input wire [255:0] ram_read_buffers,
    input wire [3:0] ram_read_valid,
    input wire [15:0] ram_read_beats,

    // Burst buffer currently shifted out. The controller only ever writes
    // slots ahead of it, so this view is stable for the whole burst.
    output wire [63:0] live_buffer,
    output wire [3:0] live_beats,
    // First byte of the burst being started, selected from live_buffer by
    // burst_offset. The fresh-byte paths in spi_trx (falling-edge output
    // and the miso_byte load) close in half/one SCK period against beats
    // that may publish in the same half period, so they consume this one
    // shared byte mux instead of variable part-selects over 64 bits at
    // each use site. Only meaningful while fresh_read is set.
    output wire [7:0] live_first_byte,

    // Sticky diagnostics, cleared at the start of every transaction.
    // Underrun is checked on each master's data sampling edge, including
    // short reads; thin still checks first-beat readiness at a burst boundary.
    output reg prefetch_underrun = 0,
    output reg prefetch_thin = 0
);

    // Ring slot being shifted out. Advances at every burst end in
    // lockstep with the controller's fill index; both restart at slot 0
    // on every CS drop, so pairing cannot drift across transactions.
    reg [1:0] consume_idx = 0;
    // Bursts posted beyond the consume burst, capped at 2: at most one
    // post per burst period reaches the controller (each post here sets
    // it, every burst end clears one), which keeps the post/drop spacing
    // of the original one-burst design. A post may only fire while the
    // levels are down and ahead permits, otherwise it would bump
    // ram_addr without a matching dispatch and walk the ring pairing
    // out of phase.
    reg [1:0] ahead = 0;
    // Priming (see header): armed by a dummy/mode post, fires one extra
    // post during the first data burst once that post's first beat has
    // published. Posts that did not exist in the one-burst design (the
    // priming post and the depth-2 catch-up posts) can land a clock or
    // two before a scheduled drop, before the controller can have
    // dispatched their ACTIVATE+READ through its synchronizers. They
    // therefore hold their levels for at least MIN_HOLD SCK clocks:
    // drop events during the hold pend and execute when it expires.
    // Original-rhythm posts keep their proven drop timing.
    localparam [2:0] MIN_HOLD = 3'd6;
    // Set once priming first reaches ahead == 2. From then on the byte-7
    // fallback is the catch-up post for bursts whose delayed post found
    // the levels still up (a held priming post), so depth 2 sustains
    // itself on either post rhythm.
    reg depth2 = 0;
    reg prime_armed = 0;
    // The target slot's valid bit can still be stale-high from its last
    // use until the post invalidation syncs into the controller; only a
    // low-then-high observation proves this post's beat published.
    reg prime_cleared = 0;
    reg [1:0] prime_delay = 0;
    reg [2:0] hold_age = 0;
    reg hold_active = 0;
    reg drop_pending = 0;
    // Two-cycle delayed fresh_read: the lookahead post fires here so the
    // burst-end drop always precedes it by >= 2 SPI clocks, even for
    // 2-clock offset-7 first bursts that have no byte 6 to drop on.
    reg post_arm1 = 0;
    reg post_arm2 = 0;

    assign live_buffer = ram_read_buffers[{consume_idx, 6'b000000} +: 64];
    assign live_beats = ram_read_beats[{consume_idx, 2'b00} +: 4];

    // Explicit lanes rather than a variable part-select: safest across
    // Yosys, Verilator, and Gowin synthesis (as in sdram.v's read path).
    reg [7:0] first_byte_mux;
    always @* begin
        case (burst_offset)
            3'd0: first_byte_mux = live_buffer[7:0];
            3'd1: first_byte_mux = live_buffer[15:8];
            3'd2: first_byte_mux = live_buffer[23:16];
            3'd3: first_byte_mux = live_buffer[31:24];
            3'd4: first_byte_mux = live_buffer[39:32];
            3'd5: first_byte_mux = live_buffer[47:40];
            3'd6: first_byte_mux = live_buffer[55:48];
            3'd7: first_byte_mux = live_buffer[63:56];
        endcase
    end
    assign live_first_byte = first_byte_mux;

    wire [22:0] next_burst = (ram_addr + 1'b1) & addr_mask;

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
                consume_idx <= 0;
                ahead <= 0;
                depth2 <= 0;
                prime_armed <= 0;
                prime_cleared <= 0;
                prime_delay <= 0;
                hold_age <= 0;
                hold_active <= 0;
                drop_pending <= 0;
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
                // Lookahead post two clocks into every burst, while fewer
                // than two bursts are already ahead and the burst-end drop
                // re-armed the handshake.
                post_arm1 <= fresh_read;
                post_arm2 <= post_arm1;
                if (post_arm2 && in_read && ahead < 2 && !ram_activate) begin
                    ram_continuation <= 1;
                    ram_activate <= 1;
                    ram_read <= 1;
                    ram_addr <= next_burst;
                    ahead <= ahead + 1'b1;
                    // This post may be the priming post (levels already
                    // dropped by the prime logic, or never armed): the
                    // armed post is no longer the latest, so priming must
                    // not drop these levels from under its dispatch. As
                    // the priming post it can land a clock before the
                    // burst-end drop of a short first burst, so it holds
                    // that drop off exactly like a prime-logic post.
                    if (prime_armed) begin
                        depth2 <= 1;
                        hold_active <= 1;
                        hold_age <= 0;
                        drop_pending <= 0;
                    end
                    prime_armed <= 0;
                    prime_cleared <= 0;
                    prime_delay <= 0;
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
                end
                if (first_done) begin
                    ram_activate <= 0;
                    ram_read <= 0;
                end

                // Mid dummy/mode post of the second burst, so short first
                // bursts (high start offsets) still meet SDRAM latency at
                // fast SCLK. The first burst's levels were dropped before,
                // so the handshake re-arms across a multi-clock gap.
                if (post_lookahead && ahead < 2 && !ram_activate) begin
                    ram_continuation <= 1;
                    ram_activate <= 1;
                    ram_read <= 1;
                    ram_addr <= next_burst;
                    ahead <= ahead + 1'b1;
                    prime_armed <= 1;
                    prime_cleared <= 0;
                    ram_post_toggle <= ~ram_post_toggle;
                end

                // Minimum level hold for priming/catch-up posts (see the
                // MIN_HOLD declaration): age the hold and execute a
                // pended drop the clock it expires.
                if (hold_active) begin
                    if (hold_age == MIN_HOLD - 1'b1) begin
                        hold_active <= 0;
                        hold_age <= 0;
                        if (drop_pending) begin
                            drop_pending <= 0;
                            ram_activate <= 0;
                            ram_read <= 0;
                        end
                    end
                    else
                        hold_age <= hold_age + 1'b1;
                end

                // Re-arm: mid mode phase, and on the last clock of byte 6.
                // This also closes the refresh window of the post. During
                // a minimum hold the drop pends instead of cutting a
                // priming/catch-up dispatch short.
                if (drop) begin
                    if (hold_active)
                        drop_pending <= 1;
                    else begin
                        ram_activate <= 0;
                        ram_read <= 0;
                    end
                end

                // Keep dummy/mode-posted levels up at mode end: that burst
                // is still filling and priming (or byte 6) drops them.
                if (mode_end && ahead == 0) begin
                    ram_activate <= 0;
                    ram_read <= 0;
                end

                // Priming: the dummy/mode post's first beat proves its
                // READ dispatched, so its levels may drop; post one more
                // burst two clocks later and hold the following drop.
                // Ordered after the drop events above so a priming post
                // landing on a drop clock (short first bursts reach byte
                // 6 two clocks after the prime drop) wins over that drop
                // instead of being counted but never raised; the minimum
                // level hold then pends the following drops. It must not coincide
                // with burst_end, whose ahead/drop bookkeeping would
                // override the post the same way: priming gives up there
                // and the next burst's delayed post takes over.
                if (prime_delay != 0)
                    prime_delay <= prime_delay - 1'b1;
                if (prime_armed && !ram_read_valid[consume_idx + 1'b1])
                    prime_cleared <= 1;
                if (prime_armed && prime_cleared && prime_delay == 0 && ram_activate &&
                    ram_read_valid[consume_idx + 1'b1]) begin
                    ram_activate <= 0;
                    ram_read <= 0;
                    prime_delay <= 2;
                end
                if (prime_armed && prime_delay == 1 && in_read && ahead < 2 && !burst_end) begin
                    ram_continuation <= 1;
                    ram_activate <= 1;
                    ram_read <= 1;
                    ram_addr <= next_burst;
                    ahead <= ahead + 1'b1;
                    prime_armed <= 0;
                    depth2 <= 1;
                    hold_active <= 1;
                    hold_age <= 0;
                    drop_pending <= 0;
                    ram_post_toggle <= ~ram_post_toggle;
                end

                // Fallback when a read starts directly at byte 7 and no
                // lookahead post happened yet, and the catch-up post that
                // sustains depth 2 in bursts whose delayed post found the
                // levels held up. A depth-2 fallback post lands a clock
                // before the burst-end drop, so it holds that drop off;
                // the original ahead == 0 fallback keeps its old timing.
                if (fallback && !ram_activate &&
                    (ahead == 0 || (depth2 && ahead < 2))) begin
                    ram_continuation <= 1;
                    ram_activate <= 1;
                    ram_read <= 1;
                    ram_addr <= next_burst;
                    ahead <= ahead + 1'b1;
                    if (depth2) begin
                        hold_active <= 1;
                        hold_age <= 0;
                        drop_pending <= 0;
                    end
                    prime_armed <= 0;
                    prime_cleared <= 0;
                    prime_delay <= 0;
                    ram_post_toggle <= ~ram_post_toggle;
                end

                if (burst_end) begin
                    consume_idx <= consume_idx + 1'b1;
                    prime_armed <= 0;
                    prime_cleared <= 0;
                    prime_delay <= 0;
                    if (ahead != 0)
                        ahead <= ahead - 1'b1;
                    // First-beat readiness is a margin diagnostic only.
                    // The consumed-byte check above uses individual beats;
                    // a fill after consumption cannot hide a stale output.
                    // These bundled-data flags cross with their data and
                    // require the same bundled-data timing contract.
                    if (!ram_read_valid[consume_idx + 1'b1])
                        prefetch_thin <= 1;
                    // Burst-end drop re-arms the handshake for the delayed
                    // post two clocks later, unless a priming/catch-up post
                    // is still within its minimum hold, in which case it
                    // pends. (Offset-7 first bursts have no byte 6; this is
                    // their only drop.)
                    if (hold_active)
                        drop_pending <= 1;
                    else begin
                        ram_activate <= 0;
                        ram_read <= 0;
                    end
                end
            end
        end
    end

endmodule
