`timescale 1ns/1ps

// SDRAM bank-state protocol checker for the two-chip shared-bus module.
// Samples the command bus on the falling system edge (the controller
// drives it on the rising edge) and fails on any command that is illegal
// for the tracked bank state:
//   - ACTIVATE to a bank that already has an open row
//   - READ/WRITE to a bank without an open row
//   - AUTO REFRESH or MODE REGISTER SET with any bank open on that chip
// READ/WRITE always carry auto-precharge in this design, so they close
// the bank. PRECHARGE closes one bank, or all with A10.
module sdram_bank_checker(
    input wire clk,
    input wire enable,
    input wire cs,          // Chip select line: 0 = chip 0, 1 = chip 1
    input wire ras, cas, we,
    input wire [1:0] ba,
    input wire [12:0] a
);
    bit row_open [0:1][0:3];
    integer activates = 0, accesses = 0, precharges = 0, refreshes = 0;

    task automatic close_all;
        for (integer c = 0; c < 2; c++)
            for (integer b = 0; b < 4; b++) row_open[c][b] = 0;
    endtask

    initial close_all;

    always @(negedge clk) if (enable) begin
        case ({ras, cas, we})
            3'b011: begin
                if (row_open[cs][ba])
                    $fatal(1, "SDRAM: ACTIVATE to open bank (chip %0d bank %0d) at %0t", cs, ba, $time);
                row_open[cs][ba] = 1;
                activates++;
            end
            3'b101, 3'b100: begin
                if (!row_open[cs][ba])
                    $fatal(1, "SDRAM: %s without ACTIVATE (chip %0d bank %0d) at %0t",
                           we ? "READ" : "WRITE", cs, ba, $time);
                if (!a[10])
                    $fatal(1, "SDRAM: access without auto-precharge at %0t", $time);
                row_open[cs][ba] = 0;
                accesses++;
            end
            3'b010: begin
                if (a[10]) for (integer b = 0; b < 4; b++) row_open[cs][b] = 0;
                else row_open[cs][ba] = 0;
                precharges++;
            end
            3'b001, 3'b000: begin
                for (integer b = 0; b < 4; b++)
                    if (row_open[cs][b])
                        $fatal(1, "SDRAM: %s with open bank (chip %0d bank %0d) at %0t",
                               we ? "REFRESH" : "MODE SET", cs, b, $time);
                if (we) refreshes++;
            end
            default: ;
        endcase
    end
endmodule
