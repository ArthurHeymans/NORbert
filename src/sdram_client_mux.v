// System-domain SDRAM client ownership. The program engine holds ownership
// from command capture through the final completed write. It may start only
// when host_protocol's program_allowed is true. Neither client can release
// a request before acceptance or change owners during an accepted burst.
`default_nettype none
module sdram_client_mux(
    input wire program_owner, accept,
    input wire [1:0] host_cmd, program_cmd,
    input wire [24:0] host_addr, program_addr,
    input wire [63:0] host_data, program_data,
    input wire host_inhibit, program_inhibit,
    output wire [1:0] access_cmd,
    output wire [24:0] access_addr,
    output wire [63:0] write_data,
    output wire inhibit, host_accept, program_accept
);
    assign access_cmd = program_owner ? program_cmd : host_cmd;
    assign access_addr = program_owner ? program_addr : host_addr;
    assign write_data = program_owner ? program_data : host_data;
    assign inhibit = program_owner ? program_inhibit : host_inhibit;
    assign host_accept = accept && !program_owner;
    assign program_accept = accept && program_owner;
endmodule
