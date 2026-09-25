`timescale 1ns/1ps

// uart.v: the byte-level contract both transports rely on, exercised
// end to end -- all 256 byte values through the transmitter and receiver
// in both configurations, back-to-back reception, the receiver's 4x
// oversampling, and the transmitter FIFO's ordering and back-pressure.
//
// DIVISOR is small so a byte takes about a microsecond instead of the
// 50 us of the real 2 Mbaud setting. The framing under test is the same
// at any divisor: a start bit, eight data bits LSB first, one stop bit.
//
// Both instances are looped back to their own receive line, which is how
// the two halves see each other on the real board (one BL616 on the other
// end of the wire).
module uart_tb;
    // Must be divisible by 4: uart_rx is instantiated with DIVISOR/4.
    localparam DIVISOR = 12;
    // ns per bit at 120 MHz. DIVISOR=12 makes this exactly 100 ns, so the
    // parameter stays an integer and every delay below is exact.
    localparam integer BIT_NS = 1000 * DIVISOR / 120;

    reg clk = 0;
    always #4.167 clk = ~clk; // 120 MHz
    reg reset = 1;

    // ---- instance A: no transmitter FIFO ----
    wire serial_txd_a;
    wire serial_rxd_a = serial_txd_a;
    wire [7:0] rxd_a;
    wire rxd_strobe_a;
    reg [7:0] txd_a = 8'h00;
    reg txd_strobe_a = 0;
    wire txd_ready_a;

    uart #(.DIVISOR(DIVISOR), .FIFO(0)) dut_a (
        .clk(clk), .reset(reset),
        .serial_rxd(serial_rxd_a), .serial_txd(serial_txd_a),
        .rxd(rxd_a), .rxd_strobe(rxd_strobe_a),
        .txd(txd_a), .txd_strobe(txd_strobe_a), .txd_ready(txd_ready_a)
    );

    // ---- instance B: 16-deep transmitter FIFO ----
    wire serial_txd_b;
    wire serial_rxd_b = serial_txd_b;
    wire [7:0] rxd_b;
    wire rxd_strobe_b;
    reg [7:0] txd_b = 8'h00;
    reg txd_strobe_b = 0;
    wire txd_ready_b;

    uart #(.DIVISOR(DIVISOR), .FIFO(16), .FREESPACE(4)) dut_b (
        .clk(clk), .reset(reset),
        .serial_rxd(serial_rxd_b), .serial_txd(serial_txd_b),
        .rxd(rxd_b), .rxd_strobe(rxd_strobe_b),
        .txd(txd_b), .txd_strobe(txd_strobe_b), .txd_ready(txd_ready_b)
    );

    integer waited;
    integer accepted;

    // rxd_strobe is a single-cycle pulse, far shorter than the polling
    // interval below, so every received byte is latched by its own clocked
    // process rather than sampled by a waiting loop.
    //
    // The latch takes the data one clock *after* the strobe: uart_rx shifts
    // the final bit and raises data_strobe on the same edge, so rxd only
    // holds the complete byte from the following clock.
    reg rx_seen_a = 0, rx_seen_b = 0;
    reg rx_strobe_a = 0, rx_strobe_b = 0;
    reg [7:0] rx_byte_a = 8'hxx, rx_byte_b = 8'hxx;
    always @(posedge clk) begin
        rx_strobe_a <= rxd_strobe_a;
        rx_strobe_b <= rxd_strobe_b;
        if (rx_strobe_a) begin
            rx_seen_a <= 1'b1;
            rx_byte_a <= rxd_a;
        end
        if (rx_strobe_b) begin
            rx_seen_b <= 1'b1;
            rx_byte_b <= rxd_b;
        end
    end

    // Hand a byte to a transmitter and wait for the receiver to report it.
    //
    // The strobe must be a single cycle: uart_tx reloads its shifter on
    // every clock that data_strobe is high, so holding it would restart the
    // frame. It is raised at a negedge and lowered at the next one, so
    // exactly one posedge sees it.
    task automatic send(input bit fifo, input [7:0] value, input integer timeout_ns);
        if (fifo) begin
            wait (txd_ready_b);
            @(negedge clk);
            rx_seen_b = 0;
            txd_b = value;
            txd_strobe_b = 1;
        end
        else begin
            wait (txd_ready_a);
            @(negedge clk);
            rx_seen_a = 0;
            txd_a = value;
            txd_strobe_a = 1;
        end
        @(posedge clk);
        @(negedge clk);
        if (fifo) txd_strobe_b = 0;
        else txd_strobe_a = 0;
        waited = 0;
        while (waited < timeout_ns) begin
            #(BIT_NS / 4);
            waited = waited + BIT_NS / 4;
            if (fifo ? rx_seen_b : rx_seen_a) break;
        end
        if (fifo) begin
            if (!rx_seen_b) $fatal(1, "byte %h never arrived", value);
            if (rx_byte_b !== value)
                $fatal(1, "received %h, expected %h", rx_byte_b, value);
        end
        else begin
            if (!rx_seen_a) $fatal(1, "byte %h never arrived", value);
            if (rx_byte_a !== value)
                $fatal(1, "received %h, expected %h", rx_byte_a, value);
        end
    endtask

    // Wait for a byte the instance's own receiver has latched.
    task automatic await_rx(input bit fifo, input integer timeout_ns);
        waited = 0;
        while (waited < timeout_ns) begin
            #(BIT_NS / 4);
            waited = waited + BIT_NS / 4;
            if (fifo ? rx_seen_b : rx_seen_a) break;
        end
        if (fifo) begin
            if (!rx_seen_b) $fatal(1, "no byte arrived");
        end
        else begin
            if (!rx_seen_a) $fatal(1, "no byte arrived");
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        reset = 0;
        repeat (8) @(negedge clk);

        // ---- every byte value through the no-FIFO instance --------------
        // 0x00 and 0xff are the ones a receiver sampling an edge early or
        // late would lose, and the bytes are back to back with no idle gap,
        // so the receiver has to re-arm on each stop bit.
        for (integer v = 0; v < 256; v++)
            send(1'b0, 8'(v), 400 * BIT_NS);

        // ---- the same through the transmitter FIFO ----------------------
        for (integer v = 0; v < 16; v++)
            send(1'b1, 8'(v) + 8'h80, 400 * BIT_NS);

        // ---- FIFO queueing, order, and back-pressure -------------------
        // Fill the queue faster than it drains: txd_ready must go low. The
        // accepted bytes are then read back in order through the receiver.
        accepted = 0;
        txd_strobe_b = 0;
        for (integer i = 0; i < 24; i++) begin
            @(negedge clk);
            txd_strobe_b = 0;
            if (txd_ready_b) begin
                txd_b = 8'h40 + 8'(i);
                txd_strobe_b = 1;
                accepted = accepted + 1;
            end
            @(posedge clk);
            @(negedge clk);
            txd_strobe_b = 0;
        end
        if (accepted >= 24)
            $fatal(1, "fifo accepted all 24 bytes: back-pressure never engaged");
        if (accepted == 0) $fatal(1, "fifo accepted nothing");
        for (integer i = 0; i < accepted; i++) begin
            rx_seen_b = 0;
            await_rx(1'b1, 400 * BIT_NS);
            if (rx_byte_b !== 8'h40 + 8'(i))
                $fatal(1, "queued byte %0d came out as %h, expected %h", i,
                       rx_byte_b, 8'h40 + 8'(i));
        end

        // ---- after a full drain the transmitter is ready again ----------
        if (!txd_ready_b) $fatal(1, "txd_ready still low with the queue empty");
        send(1'b1, 8'h5a, 400 * BIT_NS);

        $display("PASS UART: all 256 byte values, back-to-back, 4x oversampling, FIFO=0 and FIFO=16, back-pressure accepted %0d of 24 in order",
                 accepted);
        $finish;
    end

    initial begin
        #40000000;
        $fatal(1, "UART test timeout");
    end
endmodule
