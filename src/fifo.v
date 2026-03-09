// FIFO Module
// Same clock domain FIFO with first-word-fallthrough
// Holds up to NUM entries (should be a power of two)

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
   -1
`endif

module fifo(
    input wire clk,
    input wire reset,
    // Write side
    output reg space_available,
    input wire [WIDTH-1:0] write_data,
    input wire write_strobe,
    // Read side
    output wire data_available,
    output wire more_available,
    output wire [WIDTH-1:0] read_data,
    input wire read_strobe
);
    parameter WIDTH = 8;
    parameter NUM = 256;
    parameter BITS = `CLOG2(NUM);
    parameter [BITS-1:0] FREESPACE = 1;

    reg [BITS:0] count;
    reg [BITS-1:0] rd_ptr;
    reg [BITS-1:0] wr_ptr;
    reg [WIDTH-1:0] read_data_ram;
    reg [WIDTH-1:0] read_data_fwft;
    reg [WIDTH-1:0] ram [0:NUM-1];

    wire [BITS:0] next_count = count + write_strobe - read_strobe;
    assign more_available = (count > 1);
    assign data_available = (count != 0);

    reg fwft;
    assign read_data = fwft ? read_data_fwft : read_data_ram;

    always @(posedge clk) begin
        if (reset) begin
            rd_ptr <= 0;
            wr_ptr <= 0;
            count <= 0;
            space_available <= 0;
        end else begin
            if (write_strobe)
                ram[wr_ptr] <= write_data;

            read_data_ram <= ram[rd_ptr + read_strobe];
            fwft <= 0;

            // First word fall through
            if ((write_strobe && count == 0)
            ||  (write_strobe && read_strobe && count == 1))
            begin
                read_data_fwft <= write_data;
                fwft <= 1;
            end

            rd_ptr <= rd_ptr + read_strobe;
            wr_ptr <= wr_ptr + write_strobe;
            count <= next_count;

            space_available <= (NUM - 1'b1 - next_count > FREESPACE);
        end
    end
endmodule
