// Simulation-only clock source for top-level protocol/CDC tests.
// The test supplies the 120 MHz system clock directly. This does not model
// the board PLL phase shifts or validate SDRAM pin-level timing.
module pll (
    input wire clkin,
    output wire clkout,
    output wire clkout_sdram,
    output wire clkoutp,
    output wire locked
);
    assign clkout = clkin;
    assign clkout_sdram = clkin;
    assign clkoutp = clkin;
    assign locked = 1'b1;
endmodule
