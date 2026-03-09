// Utility Modules

`default_nettype none

`ifndef CLOG2
`define CLOG2(x) \
   x <= 2     ? 1 : \
   x <= 4     ? 2 : \
   x <= 8     ? 3 : \
   x <= 16    ? 4 : \
   x <= 32    ? 5 : \
   x <= 64    ? 6 : \
   x <= 128   ? 7 : \
   x <= 256   ? 8 : \
   x <= 512   ? 9 : \
   x <= 1024  ? 10 : \
   x <= 2048  ? 11 : \
   x <= 4096  ? 12 : \
   x <= 8192  ? 13 : \
   x <= 16384 ? 14 : \
   x <= 32768 ? 15 : \
   x <= 65536 ? 16 : \
   x <= 131072 ? 17 : \
   x <= 262144 ? 18 : \
   -1
`endif

// Clock divider
module divide_by_n(
    input wire clk,
    input wire reset,
    output reg out
);
    parameter N = 2;

    reg [`CLOG2(N)-1:0] counter;

    always @(posedge clk) begin
        out <= 0;

        if (reset)
            counter <= 0;
        else if (counter == 0) begin
            out <= 1;
            counter <= N - 1;
        end else
            counter <= counter - 1;
    end
endmodule

// PWM generator
module pwm(
    input wire clk,
    input wire [BITS-1:0] bright,
    output wire out
);
    parameter BITS = 8;

    reg [BITS-1:0] counter;
    assign out = counter < bright;

    always @(posedge clk)
        counter <= counter + 1;
endmodule

// D flip-flop
module d_flipflop(
    input wire clk,
    input wire reset,
    input wire d_in,
    output reg d_out
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            d_out <= 0;
        else
            d_out <= d_in;
    end
endmodule

// Double D flip-flop (synchronizer)
module d_flipflop_pair(
    input wire clk,
    input wire reset,
    input wire d_in,
    output reg d_out
);
    wire intermediate;

    d_flipflop dff1(clk, reset, d_in, intermediate);
    d_flipflop dff2(clk, reset, intermediate, d_out);
endmodule

// Set/reset flipflop
module set_reset_flipflop(
    input wire clk,
    input wire reset,
    input wire sync_set,
    input wire sync_reset,
    output reg out
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            out <= 0;
        else if (sync_set)
            out <= 1;
        else if (sync_reset)
            out <= 0;
    end
endmodule

// Pulse stretcher
module pulse_stretcher(
    input wire clk,
    input wire reset,
    input wire in,
    output reg out
);
    parameter BITS = 20;

    reg [BITS-1:0] counter;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            out <= 0;
            counter <= 0;
        end
        else if (counter == 0) begin
            out <= in;
            counter <= in ? 1 : 0;
        end
        else if (&counter) begin
            if (in)
                out <= 1;
            else begin
                out <= 0;
                counter <= 0;
            end
        end
        else begin
            out <= 1;
            counter <= counter + 1;
        end
    end
endmodule

// Clock crossing strobe
module strobe_sync(
    input wire clk,
    input wire flop,
    output wire strobe
);
    parameter DELAY = 1;
    reg [DELAY:0] sync;
    assign strobe = sync[DELAY] != sync[DELAY-1];

    always @(posedge clk)
        sync <= {sync[DELAY-1:0], flop};
endmodule
