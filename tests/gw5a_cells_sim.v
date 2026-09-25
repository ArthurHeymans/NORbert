// Simulation models for GW5A cells that Yosys ships no behaviour for,
// used by tests/gate.sh.
//
// DPX9B covers only the configuration synth_gowin emits for this design:
// 9-bit ports, bypass read mode, normal or write-through write mode, no
// initial contents. Collisions between the two ports are not modelled.
module DPX9B(CLKA, CEA, CLKB, CEB, OCEA, OCEB, RESETA, RESETB, WREA, WREB,
             ADA, ADB, DIA, DIB, BLKSELA, BLKSELB, DOA, DOB);
parameter READ_MODE0 = 1'b0;
parameter READ_MODE1 = 1'b0;
parameter WRITE_MODE0 = 2'b00;
parameter WRITE_MODE1 = 2'b00;
parameter BIT_WIDTH_0 = 18;
parameter BIT_WIDTH_1 = 18;
parameter BLK_SEL_0 = 3'b000;
parameter BLK_SEL_1 = 3'b000;
parameter RESET_MODE = "SYNC";
parameter [287:0]
    INIT_RAM_00 = 0, INIT_RAM_01 = 0, INIT_RAM_02 = 0, INIT_RAM_03 = 0,
    INIT_RAM_04 = 0, INIT_RAM_05 = 0, INIT_RAM_06 = 0, INIT_RAM_07 = 0,
    INIT_RAM_08 = 0, INIT_RAM_09 = 0, INIT_RAM_0A = 0, INIT_RAM_0B = 0,
    INIT_RAM_0C = 0, INIT_RAM_0D = 0, INIT_RAM_0E = 0, INIT_RAM_0F = 0,
    INIT_RAM_10 = 0, INIT_RAM_11 = 0, INIT_RAM_12 = 0, INIT_RAM_13 = 0,
    INIT_RAM_14 = 0, INIT_RAM_15 = 0, INIT_RAM_16 = 0, INIT_RAM_17 = 0,
    INIT_RAM_18 = 0, INIT_RAM_19 = 0, INIT_RAM_1A = 0, INIT_RAM_1B = 0,
    INIT_RAM_1C = 0, INIT_RAM_1D = 0, INIT_RAM_1E = 0, INIT_RAM_1F = 0,
    INIT_RAM_20 = 0, INIT_RAM_21 = 0, INIT_RAM_22 = 0, INIT_RAM_23 = 0,
    INIT_RAM_24 = 0, INIT_RAM_25 = 0, INIT_RAM_26 = 0, INIT_RAM_27 = 0,
    INIT_RAM_28 = 0, INIT_RAM_29 = 0, INIT_RAM_2A = 0, INIT_RAM_2B = 0,
    INIT_RAM_2C = 0, INIT_RAM_2D = 0, INIT_RAM_2E = 0, INIT_RAM_2F = 0,
    INIT_RAM_30 = 0, INIT_RAM_31 = 0, INIT_RAM_32 = 0, INIT_RAM_33 = 0,
    INIT_RAM_34 = 0, INIT_RAM_35 = 0, INIT_RAM_36 = 0, INIT_RAM_37 = 0,
    INIT_RAM_38 = 0, INIT_RAM_39 = 0, INIT_RAM_3A = 0, INIT_RAM_3B = 0,
    INIT_RAM_3C = 0, INIT_RAM_3D = 0, INIT_RAM_3E = 0, INIT_RAM_3F = 0;
input CLKA, CEA, CLKB, CEB, OCEA, OCEB, RESETA, RESETB, WREA, WREB;
input [13:0] ADA, ADB;
input [17:0] DIA, DIB;
input [2:0] BLKSELA, BLKSELB;
output reg [17:0] DOA = 0, DOB = 0;

// 2048 x 9: the word address is AD[13:3] at 9-bit width.
reg [8:0] mem [0:2047];
integer i;
initial begin
    if (BIT_WIDTH_0 != 9 || BIT_WIDTH_1 != 9 || READ_MODE0 != 0 || READ_MODE1 != 0 ||
        WRITE_MODE0 > 1 || WRITE_MODE1 > 1 || RESET_MODE != "ASYNC" ||
        {INIT_RAM_00, INIT_RAM_01, INIT_RAM_02, INIT_RAM_03, INIT_RAM_04, INIT_RAM_05, INIT_RAM_06, INIT_RAM_07,
    INIT_RAM_08, INIT_RAM_09, INIT_RAM_0A, INIT_RAM_0B, INIT_RAM_0C, INIT_RAM_0D, INIT_RAM_0E, INIT_RAM_0F,
    INIT_RAM_10, INIT_RAM_11, INIT_RAM_12, INIT_RAM_13, INIT_RAM_14, INIT_RAM_15, INIT_RAM_16, INIT_RAM_17,
    INIT_RAM_18, INIT_RAM_19, INIT_RAM_1A, INIT_RAM_1B, INIT_RAM_1C, INIT_RAM_1D, INIT_RAM_1E, INIT_RAM_1F,
    INIT_RAM_20, INIT_RAM_21, INIT_RAM_22, INIT_RAM_23, INIT_RAM_24, INIT_RAM_25, INIT_RAM_26, INIT_RAM_27,
    INIT_RAM_28, INIT_RAM_29, INIT_RAM_2A, INIT_RAM_2B, INIT_RAM_2C, INIT_RAM_2D, INIT_RAM_2E, INIT_RAM_2F,
    INIT_RAM_30, INIT_RAM_31, INIT_RAM_32, INIT_RAM_33, INIT_RAM_34, INIT_RAM_35, INIT_RAM_36, INIT_RAM_37,
    INIT_RAM_38, INIT_RAM_39, INIT_RAM_3A, INIT_RAM_3B, INIT_RAM_3C, INIT_RAM_3D, INIT_RAM_3E, INIT_RAM_3F} != 0)
        $fatal(1, "DPX9B model: unsupported configuration");
    for (i = 0; i < 2048; i = i + 1) mem[i] = 0;
end

always @(posedge CLKA or posedge RESETA)
    if (RESETA) DOA <= 0;
    else if (CEA) begin
        if (WREA) begin
            mem[ADA[13:3]] <= DIA[8:0];
            if (WRITE_MODE0 == 1) DOA <= {9'b0, DIA[8:0]};
        end else
            DOA <= {9'b0, mem[ADA[13:3]]};
    end

always @(posedge CLKB or posedge RESETB)
    if (RESETB) DOB <= 0;
    else if (CEB) begin
        if (WREB) begin
            mem[ADB[13:3]] <= DIB[8:0];
            if (WRITE_MODE1 == 1) DOB <= {9'b0, DIB[8:0]};
        end else
            DOB <= {9'b0, mem[ADB[13:3]]};
    end
endmodule

// Tristates that synth_gowin -noiopads leaves generic in submodules.
module \$_TBUF_ (input A, input E, output Y);
    assign Y = E ? A : 1'bz;
endmodule
