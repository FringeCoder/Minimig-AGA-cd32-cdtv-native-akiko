`timescale 1ns/1ns

// SIM ONLY. Two stand-ins that let minimig.v elaborate under Icarus.
//
// Nothing here is on the chipset register bus, which is what the bench next
// door exists to check. Both stubs are deliberately minimal: a stub that
// modelled more than it needs to would be a second implementation to keep
// correct, and a wrong one would make the bench lie.

// ---------------------------------------------------------------- altsyncram
//
// Denise's colour table (denise_colortable_ram_mf.v) is the only megafunction
// instance minimig.v pulls in. It is configured DUAL_PORT: port A writes, port
// B reads, address registered on clock0, output unregistered, mixed-port
// read-during-write OLD_DATA. That is the whole of what is modelled here.
//
// The parameters arrive by `defparam` from the wrapper, so they are declared
// with the same names and only the ones that change behaviour are used.
module altsyncram
#(
	parameter width_a  = 32,
	parameter width_b  = 32,
	parameter widthad_a = 8,
	parameter widthad_b = 8,
	parameter numwords_a = 256,
	parameter numwords_b = 256,
	parameter width_byteena_a = 1,
	parameter byte_size = 8,
	parameter operation_mode = "DUAL_PORT",
	parameter address_aclr_b = "NONE",
	parameter address_reg_b = "CLOCK0",
	parameter clock_enable_input_a = "NORMAL",
	parameter clock_enable_input_b = "NORMAL",
	parameter clock_enable_output_b = "BYPASS",
	parameter intended_device_family = "Cyclone III",
	parameter lpm_type = "altsyncram",
	parameter outdata_aclr_b = "NONE",
	parameter outdata_reg_b = "UNREGISTERED",
	parameter power_up_uninitialized = "FALSE",
	parameter read_during_write_mode_mixed_ports = "OLD_DATA"
)
(
	input      [widthad_a-1:0] address_a,
	input      [widthad_b-1:0] address_b,
	input                      addressstall_a,
	input                      addressstall_b,
	input                      aclr0,
	input                      aclr1,
	input  [width_byteena_a-1:0] byteena_a,
	input                      byteena_b,
	input                      clock0,
	input                      clock1,
	input                      clocken0,
	input                      clocken1,
	input                      clocken2,
	input                      clocken3,
	input      [width_a-1:0]   data_a,
	input      [width_b-1:0]   data_b,
	output     [2:0]           eccstatus,
	output     [width_a-1:0]   q_a,
	output     [width_b-1:0]   q_b,
	input                      rden_a,
	input                      rden_b,
	input                      wren_a,
	input                      wren_b
);

reg [width_a-1:0] mem [0:numwords_a-1];
reg [widthad_b-1:0] addr_b_reg;

integer k;
initial begin
	for (k = 0; k < numwords_a; k = k + 1) mem[k] = {width_a{1'b0}};
	addr_b_reg = {widthad_b{1'b0}};
end

// Port A: byte-enabled write. Read port A is unused in this configuration.
always @(posedge clock0) begin
	if (clocken0 && wren_a) begin
		for (k = 0; k < width_byteena_a; k = k + 1)
			if (byteena_a[k])
				mem[address_a][k*byte_size +: byte_size]
					<= data_a[k*byte_size +: byte_size];
	end
end

// Port B: address registered on clock0, data out unregistered. OLD_DATA on a
// mixed-port collision falls out of reading `mem` in a separate always block
// after the write has been scheduled -- the read below is combinational on the
// registered address, so it sees the memory as of the previous clock edge.
always @(posedge clock0) if (clocken0) addr_b_reg <= address_b;

assign q_b = mem[addr_b_reg];
assign q_a = {width_a{1'b0}};
assign eccstatus = 3'b000;

endmodule

// ------------------------------------------------------------------- toccata
//
// The real toccata.sv uses unpacked structs, which Icarus does not support.
// The card is a Zorro II sound board hanging off the CPU bus; it touches the
// chipset register bus not at all, so a tie-off is a faithful enough stand-in
// for this bench.
module toccata
#(
	parameter CLK_FREQUENCY = 28_359_380
)
(
	input         clk,
	input         rst,
	input         hsync,
	input  [15:0] data_in,
	output [15:0] data_out,
	input  [15:1] addr,
	input         rd,
	input         hwr,
	input         lwr,
	input         sel,
	output        toc_int,
	output [15:0] out_left,
	output [15:0] out_right
);

assign data_out  = 16'h0000;
assign toc_int   = 1'b0;
assign out_left  = 16'h0000;
assign out_right = 16'h0000;

endmodule
