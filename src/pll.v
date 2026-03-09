// PLL Configuration for Tang Primer 25K (GW5A-LV25MG121)
// Input: 50MHz crystal oscillator
// Output: clkout  = ~132MHz for SDRAM operation (main clock)
//         clkoutp = ~132MHz phase-shifted for SDRAM DQ read capture (aux clock)
//
// Gowin rPLL primitive for GW5A series
//
// Calculation:
// Fvco = Fclkin * FBDIV / IDIV = 50 * 58 / 11 = 263.636MHz
// Fout = Fvco / ODIV = 263.636 / 2 = 131.818MHz (~132MHz)
//
// Phase-shifted output (clkoutp) used for SDRAM DQ read capture timing.
// PSDA_SEL controls the phase shift amount for the auxiliary clock.

`default_nettype none

module pll(
    input wire clkin,       // 50MHz input
    output wire clkout,     // ~132MHz output (main SDRAM clock)
    output wire clkoutp,    // ~132MHz phase-shifted output (aux/read capture clock)
    output wire locked      // PLL lock indicator
);

    wire clkoutd;  // Divided output (unused)
    wire clkoutd3; // Divided by 3 output (unused)

    rPLL #(
        .FCLKIN("50"),           // Input clock frequency in MHz
        .IDIV_SEL(10),           // IDIV = 11 (0-63, actual = SEL+1)
        .FBDIV_SEL(57),          // FBDIV = 58 (0-63, actual = SEL+1)
        .ODIV_SEL(2),            // ODIV = 2 (2,4,8,16,32,48,64,80,96,112,128)
        .PSDA_SEL("0100"),       // Phase shift for clkoutp (~90 degrees)
        .DYN_SDIV_SEL(2),        // Dynamic SDIV select
        .DYN_DA_EN("false"),     // Disable dynamic adjustment
        .DYN_FBDIV_SEL("false"), // Disable dynamic FBDIV
        .DYN_IDIV_SEL("false"),  // Disable dynamic IDIV
        .DYN_ODIV_SEL("false"),  // Disable dynamic ODIV
        .DUTYDA_SEL("1000"),     // 50% duty cycle
        .CLKOUT_FT_DIR(1'b1),    // Fine tune direction
        .CLKOUTP_FT_DIR(1'b1),   // Fine tune direction
        .CLKOUT_DLY_STEP(0),     // Output delay steps
        .CLKOUTP_DLY_STEP(0),    // Phase output delay steps
        .CLKOUTD3_SRC("CLKOUT"), // CLKOUTD3 source
        .CLKOUTD_SRC("CLKOUT"),  // CLKOUTD source
        .CLKOUTD_BYPASS("false"), // Don't bypass divider
        .CLKOUTP_BYPASS("false"), // Don't bypass phase
        .CLKOUT_BYPASS("false"),  // Don't bypass output
        .DEVICE("GW5A-25")       // Device variant
    ) pll_inst (
        .CLKIN(clkin),           // Input clock
        .CLKFB(1'b0),           // Feedback clock (internal)
        .RESET(1'b0),           // Reset (active high)
        .RESET_P(1'b0),         // Reset for phase
        .FBDSEL(6'b000000),     // Dynamic FBDIV select
        .IDSEL(6'b000000),      // Dynamic IDIV select
        .ODSEL(6'b000000),      // Dynamic ODIV select
        .PSDA(4'b0000),         // Phase shift
        .FDLY(4'b0000),         // Fine delay
        .DUTYDA(4'b0000),       // Duty cycle adjust
        .LOCK(locked),          // Lock output
        .CLKOUT(clkout),        // Main output clock
        .CLKOUTP(clkoutp),      // Phase shifted output
        .CLKOUTD(clkoutd),      // Divided output
        .CLKOUTD3(clkoutd3)     // Divided by 3 output
    );

endmodule
