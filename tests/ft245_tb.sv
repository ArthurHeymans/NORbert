`timescale 1ns/1ps

// ft245.v against a behavioural FT2232H: the async 245 FIFO handshake in
// both directions, the timing the datasheet requires (read data valid at
// most t3 after RD# falls, write data set up and held around WR# falling),
// back-to-back reads, back-pressure when TXE# is high, and the
// read-over-write priority in IDLE.
//
// The model is deliberately strict about what the FT2232H guarantees and
// requires: read data is only valid from the latest time the datasheet
// allows, RD# and WR# pulses must be long enough, and a write is latched
// on the falling edge of WR# only if the data was already stable and stays
// stable for the hold time.
module ft245_tb;
    // The RTL's delays are in system clocks; this is the 120 MHz period.
    localparam real CLK_NS = 8.333;
    // FT2232H datasheet, FT245-style asynchronous FIFO timing:
    // t3: RD# active to data valid, 14 ns *maximum*. The model drives data
    // at that maximum, the latest the FPGA may see it.
    localparam real T3_NS = 14.0;
    // t4: RD# active pulse width, 30 ns minimum.
    localparam real T4_NS = 30.0;
    // t8/t9: data setup before and hold after WR# goes active, 5 ns each.
    localparam real T8_NS = 5.0;
    localparam real T9_NS = 5.0;
    // t10: WR# active pulse width, 30 ns minimum.
    localparam real T10_NS = 30.0;

    reg clk = 0;
    always #4.167 clk = ~clk; // 120 MHz
    reg reset = 1;

    // FT2232H side
    reg ft_rxf_n = 1;
    reg ft_txe_n = 0;
    wire ft_rd_n;
    wire ft_wr_n;

    // The bidirectional bus: the DUT drives it while writing, the model
    // drives it after RD# goes low. Neither drives while the other does --
    // that is checked below rather than being resolved by a resolution
    // strength.
    wire [7:0] bus;
    reg [7:0] model_data = 8'hxx;
    reg model_drive = 0;
    assign bus = model_drive ? model_data : 8'hz;

    // Byte-level interface
    wire [7:0] rxd;
    wire rxd_strobe;
    reg [7:0] txd = 8'h00;
    reg txd_strobe = 0;
    wire txd_ready;

    ft245 dut (
        .clk(clk), .reset(reset),
        .ft_data(bus), .ft_rxf_n(ft_rxf_n), .ft_txe_n(ft_txe_n),
        .ft_rd_n(ft_rd_n), .ft_wr_n(ft_wr_n),
        .rxd(rxd), .rxd_strobe(rxd_strobe),
        .txd(txd), .txd_strobe(txd_strobe), .txd_ready(txd_ready)
    );

    // -----------------------------------------------------------------
    // FT2232H model
    // -----------------------------------------------------------------
    reg [7:0] rx_queue [0:255];
    integer rx_head = 0, rx_tail = 0;

    task ft_push(input [7:0] value);
        rx_queue[rx_tail] = value;
        rx_tail = rx_tail + 1;
        ft_rxf_n = 1'b0;   // data available
    endtask

    // The FT2232H drives data t3 after RD# falls, on its own timing rather
    // than on this clock grid -- which is the point: a model that only
    // changed the bus on a clock edge would hide any margin the RTL has, or
    // manufacture margin it does not have.
    realtime rd_fell = 0.0;
    always @(negedge ft_rd_n) begin
        rd_fell = $realtime;
        #(T3_NS);
        if (!ft_rd_n && rx_head < rx_tail) begin
            model_data = rx_queue[rx_head];
            model_drive = 1'b1;
        end
    end

    always @(posedge ft_rd_n) begin
        if (!reset && $realtime - rd_fell < T4_NS)
            $fatal(1, "RD# low for %0.1f ns, t4 needs %0.1f", $realtime - rd_fell, T4_NS);
        model_drive = 1'b0;
        model_data = 8'hxx;
    end

    // Pop on the FPGA releasing RD# after having sampled.
    reg rd_seen_low = 0;
    always @(posedge clk) begin
        if (reset)
            rd_seen_low <= 0;
        else begin
            // Read completed: RD# went low then high.
            if (!ft_rd_n) rd_seen_low <= 1'b1;
            if (rd_seen_low && ft_rd_n) begin
                rd_seen_low <= 0;
                if (rx_head < rx_tail) rx_head = rx_head + 1;
                // rx_head has already advanced, so the test is against the
                // new value: RXF# stays low while a byte remains.
                ft_rxf_n <= (rx_head < rx_tail) ? 1'b0 : 1'b1;
            end
        end
    end

    // Writes: the FT2232H takes the byte as WR# falls. The data must have
    // been stable for t8 before that edge and must stay for t9 after it.
    // Both processes check, because a bus change and the WR# edge in the
    // same time step can run in either order: whichever runs second sees
    // the other's timestamp and a zero setup or hold time.
    integer write_count = 0;
    reg [7:0] written [0:255];
    realtime bus_changed = 0.0;
    realtime wr_fell = 0.0;
    always @(bus) begin
        bus_changed = $realtime;
        if (!reset && !ft_wr_n && bus_changed - wr_fell < T9_NS)
            $fatal(1, "write data changed %0.1f ns after WR# fell, t8/t9 need %0.1f",
                   bus_changed - wr_fell, T9_NS);
    end
    always @(negedge ft_wr_n) begin
        if (!reset) begin
            wr_fell = $realtime;
            if (!dut.data_oe)
                $fatal(1, "WR# fell with the FPGA not driving the bus");
            if (wr_fell - bus_changed < T8_NS)
                $fatal(1, "write data set up %0.1f ns before WR#, t8 needs %0.1f",
                       wr_fell - bus_changed, T8_NS);
            if (write_count < 256) begin
                written[write_count] = bus;
                write_count = write_count + 1;
            end
        end
    end

    always @(posedge ft_wr_n)
        if (!reset && $realtime - wr_fell < T10_NS)
            $fatal(1, "WR# low for %0.1f ns, t10 needs %0.1f", $realtime - wr_fell, T10_NS);

    // The FPGA must not drive the bus while the FT2232H is driving it.
    always @(posedge clk)
        if (!reset && model_drive && dut.data_oe)
            $fatal(1, "FPGA and FT2232H are both driving the bus");

    // t3 ordering: the clock edge on which the RTL samples the bus (its
    // ST_RD_SAMPLE state) must come after the model has driven valid data.
    always @(posedge clk)
        if (!reset && dut.state == dut.ST_RD_SAMPLE && !model_drive)
            $fatal(1, "FPGA sampled the bus before the FT2232H drove valid data");

    // -----------------------------------------------------------------
    // Latch received bytes: rxd_strobe is a single-cycle pulse.
    // -----------------------------------------------------------------
    reg rx_seen = 0;
    reg [7:0] rx_byte = 8'hxx;
    integer rx_total = 0;
    reg rx_strobe_d = 0;
    always @(posedge clk) begin
        rx_strobe_d <= rxd_strobe;
        if (rx_strobe_d) begin
            rx_seen <= 1'b1;
            rx_byte <= rxd;
            rx_total = rx_total + 1;
        end
    end

    task automatic await_rx(output reg [7:0] got, input integer timeout_ns);
        real w = 0.0;
        rx_seen = 0;
        while (!rx_seen && w < timeout_ns) begin
            #(CLK_NS);
            w = w + CLK_NS;
        end
        if (!rx_seen) $fatal(1, "no byte arrived within %0d ns", timeout_ns);
        got = rx_byte;
    endtask

    // Hand a byte to the transmitter and wait for it to be written out.
    task automatic send(input [7:0] value, input integer timeout_ns);
        real w;
        wait (txd_ready);
        @(negedge clk);
        txd = value;
        txd_strobe = 1;
        @(posedge clk);
        @(negedge clk);
        txd_strobe = 0;
        w = 0.0;
        while (write_count < 1 && w < timeout_ns) begin
            #(CLK_NS);
            w = w + CLK_NS;
        end
    endtask

    integer writes_before;
    initial begin
        repeat (4) @(negedge clk);
        reset = 0;
        repeat (4) @(negedge clk);

        // ---- write one byte ------------------------------------------
        send(8'h3c, 2000);
        if (write_count != 1) $fatal(1, "expected 1 write, saw %0d", write_count);
        if (written[0] !== 8'h3c) $fatal(1, "wrote %h, expected 3c", written[0]);

        // t8/t9/t10 are checked by the model on every write, this one
        // included: the RTL's ST_WR_SETUP state exists for t8.

        // ---- read one byte -------------------------------------------
        ft_push(8'ha5);
        await_rx(rx_byte, 2000);
        if (rx_byte !== 8'ha5) $fatal(1, "read %h, expected a5", rx_byte);

        // ---- back-to-back reads -------------------------------------
        // Three bytes queued at once: the controller must take them in
        // order, using the recovery state to re-assert RD#.
        ft_push(8'h11);
        ft_push(8'h22);
        ft_push(8'h33);
        await_rx(rx_byte, 2000);
        if (rx_byte !== 8'h11) $fatal(1, "read %h, expected 11", rx_byte);
        await_rx(rx_byte, 2000);
        if (rx_byte !== 8'h22) $fatal(1, "read %h, expected 22", rx_byte);
        await_rx(rx_byte, 2000);
        if (rx_byte !== 8'h33) $fatal(1, "read %h, expected 33", rx_byte);

        // ---- back-pressure: TXE# high --------------------------------
        // With no room in the FT2232H's transmit FIFO, the byte must be
        // held and not lost, then written once TXE# goes low.
        ft_txe_n = 1'b1;
        writes_before = write_count;
        txd = 8'h7e;
        txd_strobe = 1;
        @(posedge clk);
        @(negedge clk);
        txd_strobe = 0;
        repeat (20) @(posedge clk);
        if (write_count != writes_before) $fatal(1, "wrote while TXE# was high");
        if (txd_ready) $fatal(1, "txd_ready high with no room in the FIFO");
        ft_txe_n = 1'b0;
        repeat (60) @(posedge clk);
        if (write_count != writes_before + 1)
            $fatal(1, "held byte was not written after TXE# went low");
        if (written[write_count - 1] !== 8'h7e)
            $fatal(1, "held byte written as %h, expected 7e",
                   written[write_count - 1]);
        if (!txd_ready) $fatal(1, "txd_ready still low after the write");

        // ---- a long read burst ---------------------------------------
        // Many bytes back to back, which is the throughput case the host
        // tool depends on for bulk transfers.
        for (integer i = 0; i < 64; i++)
            ft_push(8'(i * 7 + 1));
        for (integer i = 0; i < 64; i++) begin
            await_rx(rx_byte, 4000);
            if (rx_byte !== 8'(i * 7 + 1))
                $fatal(1, "burst byte %0d read %h, expected %h", i, rx_byte,
                       8'(i * 7 + 1));
        end

        // ---- read and write offered together -------------------------
        // IDLE gives reads priority; a queued read must still be taken
        // before a pending write, and the write must not be lost.
        ft_push(8'h5f);
        txd = 8'ha5;
        txd_strobe = 1;
        @(posedge clk);
        @(negedge clk);
        txd_strobe = 0;
        writes_before = write_count;
        await_rx(rx_byte, 2000);
        if (rx_byte !== 8'h5f)
            $fatal(1, "concurrent read returned %h, expected 5f", rx_byte);
        repeat (60) @(posedge clk);
        if (write_count != writes_before + 1)
            $fatal(1, "write lost while a read was being serviced");
        if (written[write_count - 1] !== 8'ha5)
            $fatal(1, "concurrent write stored %h, expected a5",
                   written[write_count - 1]);

        $display("PASS FT245: write handshake and setup, read handshake, back-to-back reads, TXE# back-pressure, %0d-byte burst, read-over-write priority",
                 rx_total);
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "FT245 test timeout");
    end
endmodule
