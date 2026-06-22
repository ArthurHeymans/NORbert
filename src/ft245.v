// FT245 Asynchronous FIFO Interface for FT2232H
//
// Implements FT2232H async 245 FIFO mode.  The FT2232H EEPROM must be
// configured to set Channel A to "245 FIFO" mode (one-time setup with
// FT_PROG).  No CLKOUT or OE# signals are used -- in async mode the
// data bus direction is implicit from RD#.
//
// Provides the same byte-level interface as uart.v so glue.v can
// use either transport interchangeably.
//
// Async FIFO read protocol (FT2232H -> FPGA):
//   1. RXF# low indicates data available
//   2. Assert RD# low -- FT2232H drives D[7:0]
//   3. Wait >=50ns for data valid (t3 in FT2232H datasheet)
//   4. Sample D[7:0], deassert RD# high
//   5. Wait for recovery, check RXF# for next byte
//
// Async FIFO write protocol (FPGA -> FT2232H):
//   1. TXE# low indicates space available
//   2. Drive D[7:0] with data
//   3. Assert WR# low, hold >=50ns (t9 in FT2232H datasheet)
//   4. Deassert WR# high -- rising edge latches data

`default_nettype none

module ft245(
    input wire clk,           // System clock (120MHz)
    input wire reset,

    // FT2232H physical interface
    inout wire [7:0] ft_data, // Bidirectional data bus
    input wire ft_rxf_n,      // RX FIFO not empty (active low)
    input wire ft_txe_n,      // TX FIFO not full  (active low)
    output reg ft_rd_n,       // Read strobe  (active low, FPGA output)
    output reg ft_wr_n,       // Write strobe (active low, FPGA output)

    // Byte-level interface (system clock domain, matches uart.v)
    output reg [7:0] rxd,
    output reg rxd_strobe,
    input wire [7:0] txd,
    input wire txd_strobe,
    output wire txd_ready
);

    // -----------------------------------------------------------------
    // Timing parameters (120MHz clock, ~8.3ns per cycle)
    // All delays include margin above FT2232H datasheet minimums.
    // -----------------------------------------------------------------
    localparam [10:0]
        DELAY_RD_DATA  = 11'd40,  // RD# active to data valid (~333ns, Pico bridge margin)
        DELAY_RD_RECOV = 11'd10,  // RD# recovery + sync pipeline (~83ns)
        DELAY_WR_PULSE = 11'd40,  // WR# active pulse width (~333ns, Pico bridge margin)
        DELAY_WR_RECOV = 11'd20;  // WR# recovery + sync pipeline (~167ns)

    // -----------------------------------------------------------------
    // Synchronize FT2232H status inputs into system clock domain.
    // -----------------------------------------------------------------
    reg [1:0] rxf_sync, txe_sync;

    wire rxf_low = !rxf_sync[1];   // Data available from FT2232H
    wire txe_low = !txe_sync[1];   // FT2232H can accept data

    always @(posedge clk) begin
        rxf_sync <= {rxf_sync[0], ft_rxf_n};
        txe_sync <= {txe_sync[0], ft_txe_n};
    end

    // -----------------------------------------------------------------
    // Data bus tristate control
    // -----------------------------------------------------------------
    reg data_oe;
    reg [7:0] data_out;
    assign ft_data = data_oe ? data_out : 8'bz;

    // -----------------------------------------------------------------
    // TX pending buffer -- captures txd_strobe from glue
    // -----------------------------------------------------------------
    reg tx_pending;
    reg txd_strobe_d;
    reg [7:0] tx_byte;
    wire txd_strobe_rise = txd_strobe && !txd_strobe_d;
    assign txd_ready = !tx_pending;

    // -----------------------------------------------------------------
    // Delay counter
    // -----------------------------------------------------------------
    reg [10:0] delay_cnt;

    // -----------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------
    localparam [2:0]
        ST_IDLE       = 3'd0,
        ST_RD_STROBE  = 3'd1,   // RD# asserted, waiting for data valid
        ST_RD_SAMPLE  = 3'd2,   // Sample data, deassert RD#
        ST_RD_RECOVER = 3'd3,   // RD# recovery, check for more data
        ST_WR_SETUP   = 3'd4,   // Data driven, WR# not yet asserted (t8 setup)
        ST_WR_DRIVE   = 3'd5,   // Data driven, WR# asserted
        ST_WR_RECOVER = 3'd6;   // WR# deasserted, recovery

    reg [2:0] state;

    always @(posedge clk) begin
        rxd_strobe <= 0;

        if (reset) begin
            state      <= ST_IDLE;
            ft_rd_n    <= 1;
            ft_wr_n    <= 1;
            data_oe    <= 0;
            data_out   <= 0;
            tx_pending   <= 0;
            txd_strobe_d <= 0;
            tx_byte      <= 0;
            rxd          <= 0;
            delay_cnt    <= 0;
        end
        else begin
            txd_strobe_d <= txd_strobe;

            // Latch one TX request per logical strobe.  The Pico bridge can
            // hold a single FT245 write cycle much longer than a real
            // FT2232H.  If txd_strobe is treated as a level, the same glue
            // response can be re-latched after tx_pending clears and then be
            // delivered again on the next Pico read.  Edge-detecting keeps
            // one glue strobe == one FT245 byte.
            if (txd_strobe_rise && !tx_pending) begin
                tx_pending <= 1;
                tx_byte    <= txd;
            end

            case (state)
                // ---------------------------------------------------
                // IDLE: arbitrate between read (priority) and write
                // ---------------------------------------------------
                ST_IDLE: begin
                    if (rxf_low && !tx_pending) begin
                        // Start read: assert RD# (FT2232H drives bus)
                        ft_rd_n   <= 0;
                        data_oe   <= 0;
                        delay_cnt <= DELAY_RD_DATA - 1;
                        state     <= ST_RD_STROBE;
                    end
                    else if (tx_pending && txe_low) begin
                        // Start write: drive data FIRST, WR# stays high
                        // (satisfies t8: data setup before WR# active)
                        data_out  <= tx_byte;
                        data_oe   <= 1;
                        state     <= ST_WR_SETUP;
                    end
                end

                // ---------------------------------------------------
                // READ path: RD# -> delay -> sample -> recover
                // ---------------------------------------------------

                // RD# asserted, waiting for data to become valid
                ST_RD_STROBE: begin
                    if (delay_cnt != 0)
                        delay_cnt <= delay_cnt - 1;
                    else
                        state <= ST_RD_SAMPLE;
                end

                // Sample data, deassert RD#
                ST_RD_SAMPLE: begin
                    rxd        <= ft_data;   // Direct sample (data stable for ~50ns)
                    rxd_strobe <= 1;
                    ft_rd_n    <= 1;
                    delay_cnt  <= DELAY_RD_RECOV - 1;
                    state      <= ST_RD_RECOVER;
                end

                // Recovery: wait for RXF# to update, check for more data
                ST_RD_RECOVER: begin
                    if (delay_cnt != 0)
                        delay_cnt <= delay_cnt - 1;
                    else begin
                        if (rxf_low && !tx_pending) begin
                            // More data available: re-assert RD#
                            ft_rd_n   <= 0;
                            delay_cnt <= DELAY_RD_DATA - 1;
                            state     <= ST_RD_STROBE;
                        end
                        else begin
                            state <= ST_IDLE;
                        end
                    end
                end

                // ---------------------------------------------------
                // WRITE path: setup -> WR# pulse -> release
                // ---------------------------------------------------

                // Data is being driven on bus, now assert WR#.
                // One cycle of data setup (~8.3ns at 120MHz) satisfies
                // FT2232H t8 requirement (5ns data setup before WR#).
                ST_WR_SETUP: begin
                    ft_wr_n   <= 0;
                    delay_cnt <= DELAY_WR_PULSE - 1;
                    state     <= ST_WR_DRIVE;
                end

                // WR# asserted with data, holding for pulse width
                ST_WR_DRIVE: begin
                    if (delay_cnt != 0)
                        delay_cnt <= delay_cnt - 1;
                    else begin
                        // Deassert WR# (rising edge latches data in FT2232H)
                        ft_wr_n    <= 1;
                        tx_pending <= 0;
                        delay_cnt  <= DELAY_WR_RECOV - 1;
                        state      <= ST_WR_RECOVER;
                    end
                end

                // WR# recovery, release bus
                ST_WR_RECOVER: begin
                    if (delay_cnt != 0)
                        delay_cnt <= delay_cnt - 1;
                    else begin
                        data_oe <= 0;
                        state   <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
