// UART Module
// 3 Mbaud serial port for host communication
//
// Copyright (C) 2009 Micah Dowty
//           (C) 2018 Trammell Hudson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

`default_nettype none

module uart_tx(
    input wire clk,
    input wire reset,
    output wire serial,
    output reg ready,
    input wire [7:0] data,
    input wire data_strobe
);
    parameter DIVISOR = 100;
    wire baud_x1;
    divide_by_n #(.N(DIVISOR)) baud_x1_div(clk, reset, baud_x1);

    reg [9:0] shiftreg;
    reg serial_r;
    assign serial = !serial_r;

    always @(posedge clk) begin
        if (reset) begin
            shiftreg <= 0;
            serial_r <= 0;
        end
        else if (data_strobe) begin
            shiftreg <= {
                1'b1, // Stop bit
                data,
                1'b0  // Start bit (inverted)
            };
            ready <= 0;
        end
        else if (baud_x1) begin
            if (shiftreg == 0) begin
                // Idle state is idle high, serial_r is inverted
                serial_r <= 0;
                ready <= 1;
            end else begin
                serial_r <= !shiftreg[0];
            end
            // Shift the output register down
            shiftreg <= {1'b0, shiftreg[9:1]};
        end else
            ready <= (shiftreg == 0);
    end
endmodule


module uart_rx(
    input wire clk,
    input wire reset,
    input wire serial,
    output wire [7:0] data,
    output reg data_strobe
);
    parameter DIVISOR = 25; // Should be 1/4 the uart_tx divisor
    wire baud_x4;
    divide_by_n #(.N(DIVISOR)) baud_x4_div(clk, reset, baud_x4);

    // Clock crossing into clk domain
    reg [1:0] serial_buf;
    wire serial_sync = serial_buf[1];
    always @(posedge clk)
        serial_buf <= {serial_buf[0], serial};

    // State machine: Four clocks per bit, 10 total bits
    reg [8:0] shiftreg;
    reg [5:0] state;
    wire [3:0] bit_count = state[5:2];
    wire [1:0] bit_phase = state[1:0];

    wire sampling_phase = (bit_phase == 1);
    wire start_bit = (bit_count == 0 && sampling_phase);
    wire stop_bit = (bit_count == 9 && sampling_phase);

    wire waiting_for_start = (state == 0 && serial_sync == 1);

    wire error = ((start_bit && serial_sync == 1) ||
                  (stop_bit && serial_sync == 0));

    assign data = shiftreg[7:0];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= 0;
            data_strobe <= 0;
        end
        else if (baud_x4) begin
            if (waiting_for_start || error || stop_bit)
                state <= 0;
            else
                state <= state + 1;

            if (bit_phase == 1)
                shiftreg <= {serial_sync, shiftreg[8:1]};

            data_strobe <= stop_bit && !error;
        end
        else begin
            data_strobe <= 0;
        end
    end
endmodule


module uart(
    input wire clk,
    input wire reset,
    // Physical interface
    input wire serial_rxd,
    output wire serial_txd,

    // Logical interface
    output wire [7:0] rxd,
    output wire rxd_strobe,
    input wire [7:0] txd,
    input wire txd_strobe,
    output wire txd_ready
);
    parameter DIVISOR = 40; // Must be divisible by 4 for rx clock
    parameter FIFO = 0;
    parameter FREESPACE = 1;

    uart_rx #(.DIVISOR(DIVISOR/4)) rx(
        .clk(clk),
        .reset(reset),
        // Physical
        .serial(serial_rxd),
        // Logical
        .data_strobe(rxd_strobe),
        .data(rxd)
    );

    wire [7:0] serial_txd_data;
    wire serial_txd_ready;
    reg serial_txd_strobe;

    generate
    if (FIFO == 0) begin
        assign serial_txd_data = txd;
        assign txd_ready = serial_txd_ready;
        always @(posedge clk) serial_txd_strobe <= txd_strobe;
    end else begin
        wire fifo_available;
        fifo #(
            .FREESPACE(FREESPACE),
            .NUM(FIFO),
            .WIDTH(8)
        ) tx_fifo(
            .clk(clk),
            .reset(reset),
            // Input side to logic
            .space_available(txd_ready),
            .write_data(txd),
            .write_strobe(txd_strobe),
            // Output side to serial uart
            .data_available(fifo_available),
            .read_data(serial_txd_data),
            .read_strobe(serial_txd_strobe)
        );

        always @(posedge clk) begin
            if (fifo_available
            &&  serial_txd_ready
            && !serial_txd_strobe
            && !reset)
                serial_txd_strobe <= 1;
            else
                serial_txd_strobe <= 0;
        end
    end
    endgenerate

    uart_tx #(.DIVISOR(DIVISOR)) tx(
        .clk(clk),
        .reset(reset),
        // Physical
        .serial(serial_txd),
        // Logical
        .data(serial_txd_data),
        .data_strobe(serial_txd_strobe),
        .ready(serial_txd_ready)
    );
endmodule
