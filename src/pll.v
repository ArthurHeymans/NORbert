// PLL Configuration for Tang Primer 25K (GW5A-LV25MG121NC1/I0)
// Input: 50MHz crystal oscillator
// Output: clkout      = 120MHz for SDRAM operation (main clock, no phase shift)
//         clkout_sdram = 120MHz dedicated SDRAM clock output (fed through IODELAY)
//         clkoutp     = 120MHz phase-shifted for SDRAM DQ read capture (aux clock)
//
// IMPORTANT: The SDRAM clock MUST use a separate PLL output from the main
// logic clock. Routing the main clock through IODELAY causes synthesis issues.
// The reference design (LicheeTang25k_SDRAM) uses 3 PLL outputs:
//   clkout0 = main logic clock
//   clkout1 = SDRAM clock (through IODELAY)
//   clkout2 = aux clock (for read data capture)
//
// GW5A uses PLLA primitive (not rPLL - that's for GW1N/GW2A series)
//
// PLL formula: Fvco = Fclkin * MDIV / IDIV
//              Fout = Fvco / ODIV
//
// PFD frequency = FCLKIN / IDIV must be 19-87.5MHz
// VCO frequency must be 700-1400MHz
//
// Configuration:
//   IDIV = 1 -> PFD = 50/1 = 50MHz (valid: 19-87.5MHz)
//   MDIV = 24 -> VCO = 50 * 24 / 1 = 1200MHz (valid: 700-1400MHz)
//   ODIV0 = 10 -> CLKOUT0 = 1200 / 10 = 120MHz (main clock, PE_COARSE=0)
//   ODIV1 = 10 -> CLKOUT1 = 1200 / 10 = 120MHz (SDRAM clock, PE_COARSE=0)
//   ODIV2 = 10 -> CLKOUT2 = 1200 / 10 = 120MHz (aux clock, PE_COARSE=6)
//   UART: 120MHz / 60 = 2.0 Mbaud
//   CRITICAL: DIVISOR must be divisible by 4 (uart_rx uses DIVISOR/4)
//
// SDRAM read capture timing:
//   PE_COARSE step = T_VCO/2 = 0.833ns/2 = 0.417ns (for VCO=1200MHz)
//   PE_COARSE=6 -> 6 * 0.417ns = 2.5ns phase shift (108 degrees)
//   This centers the DQ sampling point within the SDRAM data valid window,
//   accounting for IODELAY (~0.8ns), round-trip PCB routing on the 40-pin
//   connector (~3ns), and W9825G6KH tAC (max 5.4ns) / tOH (min 2.5ns).
//
// Uses defparam style matching Gowin IDE-generated code (gowin_pll_27.v).
// Includes ICP_SEL, LPF_RES, LPF_CAP loop filter parameters that the
// Gowin toolchain requires for stable PLL lock.

`default_nettype none

module pll(
    input wire clkin,        // 50MHz input
    output wire clkout,      // 120MHz output (main logic clock, no phase shift)
    output wire clkout_sdram,// 120MHz output (dedicated SDRAM clock, phase-shifted)
    output wire clkoutp,     // 120MHz phase-shifted output (aux/read capture clock)
    output wire locked       // PLL lock indicator
);

    wire clkout3_o;
    wire clkout4_o;
    wire clkout5_o;
    wire clkout6_o;
    wire clkfbout_o;
    wire [7:0] mdrdo_o;
    wire gw_gnd;

    assign gw_gnd = 1'b0;

    PLLA pll_inst (
        .LOCK(locked),
        .CLKOUT0(clkout),
        .CLKOUT1(clkout_sdram),
        .CLKOUT2(clkoutp),
        .CLKOUT3(clkout3_o),
        .CLKOUT4(clkout4_o),
        .CLKOUT5(clkout5_o),
        .CLKOUT6(clkout6_o),
        .CLKFBOUT(clkfbout_o),
        .MDRDO(mdrdo_o),
        .CLKIN(clkin),
        .CLKFB(gw_gnd),
        .RESET(gw_gnd),
        .PLLPWD(gw_gnd),
        .RESET_I(gw_gnd),
        .RESET_O(gw_gnd),
        .PSSEL({gw_gnd,gw_gnd,gw_gnd}),
        .PSDIR(gw_gnd),
        .PSPULSE(gw_gnd),
        .SSCPOL(gw_gnd),
        .SSCON(gw_gnd),
        .SSCMDSEL({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd}),
        .SSCMDSEL_FRAC({gw_gnd,gw_gnd,gw_gnd}),
        .MDCLK(gw_gnd),
        .MDOPC({gw_gnd,gw_gnd}),
        .MDAINC(gw_gnd),
        .MDWDI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd})
    );

    defparam pll_inst.FCLKIN = "50";
    defparam pll_inst.IDIV_SEL = 1;
    defparam pll_inst.FBDIV_SEL = 1;
    defparam pll_inst.CLKFB_SEL = "INTERNAL";
    defparam pll_inst.ODIV0_SEL = 10;            // CLKOUT0 = 1200/10 = 120MHz (main clock)
    defparam pll_inst.ODIV0_FRAC_SEL = 0;
    defparam pll_inst.ODIV1_SEL = 10;            // CLKOUT1 = 1200/10 = 120MHz (SDRAM clock)
    defparam pll_inst.ODIV2_SEL = 10;            // CLKOUT2 = 1200/10 = 120MHz (aux clock)
    defparam pll_inst.ODIV3_SEL = 8;
    defparam pll_inst.ODIV4_SEL = 8;
    defparam pll_inst.ODIV5_SEL = 8;
    defparam pll_inst.ODIV6_SEL = 8;
    defparam pll_inst.MDIV_SEL = 24;             // Fvco = 50 * 24 / 1 = 1200MHz
    defparam pll_inst.MDIV_FRAC_SEL = 0;
    defparam pll_inst.CLKOUT0_EN = "TRUE";
    defparam pll_inst.CLKOUT1_EN = "TRUE";
    defparam pll_inst.CLKOUT2_EN = "TRUE";
    defparam pll_inst.CLKOUT3_EN = "FALSE";
    defparam pll_inst.CLKOUT4_EN = "FALSE";
    defparam pll_inst.CLKOUT5_EN = "FALSE";
    defparam pll_inst.CLKOUT6_EN = "FALSE";
    defparam pll_inst.CLKOUT0_DT_DIR = 1'b1;
    defparam pll_inst.CLKOUT1_DT_DIR = 1'b1;
    defparam pll_inst.CLKOUT2_DT_DIR = 1'b1;
    defparam pll_inst.CLKOUT3_DT_DIR = 1'b1;
    defparam pll_inst.CLK0_IN_SEL = 1'b0;
    defparam pll_inst.CLK0_OUT_SEL = 1'b0;
    defparam pll_inst.CLK1_IN_SEL = 1'b0;
    defparam pll_inst.CLK1_OUT_SEL = 1'b0;
    defparam pll_inst.CLK2_IN_SEL = 1'b0;
    defparam pll_inst.CLK2_OUT_SEL = 1'b0;
    defparam pll_inst.CLK3_IN_SEL = 1'b0;
    defparam pll_inst.CLK3_OUT_SEL = 1'b0;
    defparam pll_inst.CLK4_IN_SEL = 2'b00;
    defparam pll_inst.CLK4_OUT_SEL = 1'b0;
    defparam pll_inst.CLK5_IN_SEL = 1'b0;
    defparam pll_inst.CLK5_OUT_SEL = 1'b0;
    defparam pll_inst.CLK6_IN_SEL = 1'b0;
    defparam pll_inst.CLK6_OUT_SEL = 1'b0;
    defparam pll_inst.CLKOUT0_PE_COARSE = 0;      // Main clock: no phase shift
    defparam pll_inst.CLKOUT0_PE_FINE = 0;
    defparam pll_inst.CLKOUT1_PE_COARSE = 9;     // SDRAM clock: ~3.75ns delay (max for ODIV=10)
    defparam pll_inst.CLKOUT1_PE_FINE = 0;
    defparam pll_inst.CLKOUT2_PE_COARSE = 6;     // Aux clock: ~2.5ns phase shift for DQ read capture
    defparam pll_inst.CLKOUT2_PE_FINE = 0;
    defparam pll_inst.CLKOUT3_PE_COARSE = 0;
    defparam pll_inst.CLKOUT3_PE_FINE = 0;
    defparam pll_inst.CLKOUT4_PE_COARSE = 0;
    defparam pll_inst.CLKOUT4_PE_FINE = 0;
    defparam pll_inst.CLKOUT5_PE_COARSE = 0;
    defparam pll_inst.CLKOUT5_PE_FINE = 0;
    defparam pll_inst.CLKOUT6_PE_COARSE = 0;
    defparam pll_inst.CLKOUT6_PE_FINE = 0;
    defparam pll_inst.DE0_EN = "FALSE";
    defparam pll_inst.DE1_EN = "FALSE";
    defparam pll_inst.DE2_EN = "FALSE";
    defparam pll_inst.DE3_EN = "FALSE";
    defparam pll_inst.DE4_EN = "FALSE";
    defparam pll_inst.DE5_EN = "FALSE";
    defparam pll_inst.DE6_EN = "FALSE";
    defparam pll_inst.DYN_DPA_EN = "FALSE";
    defparam pll_inst.DYN_PE0_SEL = "FALSE";
    defparam pll_inst.DYN_PE1_SEL = "FALSE";
    defparam pll_inst.DYN_PE2_SEL = "FALSE";
    defparam pll_inst.DYN_PE3_SEL = "FALSE";
    defparam pll_inst.DYN_PE4_SEL = "FALSE";
    defparam pll_inst.DYN_PE5_SEL = "FALSE";
    defparam pll_inst.DYN_PE6_SEL = "FALSE";
    defparam pll_inst.RESET_I_EN = "FALSE";
    defparam pll_inst.RESET_O_EN = "FALSE";
    defparam pll_inst.ICP_SEL = 6'bXXXXXX;       // Charge pump current (auto-selected by toolchain)
    defparam pll_inst.LPF_RES = 3'bXXX;          // Loop filter resistance (auto-selected by toolchain)
    defparam pll_inst.LPF_CAP = 2'b00;           // Loop filter capacitance
    defparam pll_inst.SSC_EN = "FALSE";
    defparam pll_inst.CLKOUT0_DT_STEP = 0;
    defparam pll_inst.CLKOUT1_DT_STEP = 0;
    defparam pll_inst.CLKOUT2_DT_STEP = 0;
    defparam pll_inst.CLKOUT3_DT_STEP = 0;

endmodule
