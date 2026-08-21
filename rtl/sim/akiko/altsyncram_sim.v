// SPDX-License-Identifier: GPL-3.0-or-later
//
// Minimal altsyncram model, for Icarus only.
//
// akiko_nvram.v instantiates Altera's altsyncram directly rather than
// inferring a RAM, so that Quartus emits a per-instance .mif binding for the
// M10K init_file. That is the right call for the build and it is why none of
// the akiko benches can be compiled by anything except ModelSim with
// altera_mf.v -- which is not installed on the dev machine, so in practice
// they were never run.
//
// This is the smallest thing that makes akiko.v elaborate under Icarus. It is
// NOT a general altsyncram model:
//
//   * DUAL_PORT only: write on port A, registered-address read on port B,
//     one clock (clock0). q_a is tied off.
//   * Fixed at the shape akiko_nvram uses -- 1024 words of 8 bits.
//   * init_file is accepted and IGNORED. A bench that depends on the EEPROM's
//     initial contents will see X, not the .mif, and must load what it needs
//     itself. tb_akiko_nvram is the bench that cares; it still wants ModelSim.
//
// The parameters are declared only so the defparam block in akiko_nvram.v
// resolves. Their values do nothing here.

`timescale 1ns / 1ps

module altsyncram
(
	input      [9:0] address_a,
	input      [9:0] address_b,
	input            clock0,
	input            clock1,
	input      [7:0] data_a,
	input      [7:0] data_b,
	input            wren_a,
	input            wren_b,
	output     [7:0] q_a,
	output     [7:0] q_b,
	input            aclr0,
	input            aclr1,
	input            addressstall_a,
	input            addressstall_b,
	input            byteena_a,
	input            byteena_b,
	input            clocken0,
	input            clocken1,
	input            clocken2,
	input            clocken3,
	output           eccstatus,
	input            rden_a,
	input            rden_b
);

parameter address_aclr_b                     = "NONE";
parameter address_reg_b                      = "CLOCK0";
parameter clock_enable_input_a               = "BYPASS";
parameter clock_enable_input_b               = "BYPASS";
parameter clock_enable_output_b              = "BYPASS";
parameter init_file                          = "";
parameter intended_device_family             = "Cyclone V";
parameter lpm_type                           = "altsyncram";
parameter numwords_a                         = 1024;
parameter numwords_b                         = 1024;
parameter operation_mode                     = "DUAL_PORT";
parameter outdata_aclr_b                     = "NONE";
parameter outdata_reg_b                      = "UNREGISTERED";
parameter power_up_uninitialized             = "FALSE";
parameter ram_block_type                     = "M10K";
parameter read_during_write_mode_mixed_ports = "OLD_DATA";
parameter widthad_a                          = 10;
parameter widthad_b                          = 10;
parameter width_a                            = 8;
parameter width_b                            = 8;
parameter width_byteena_a                    = 1;

reg [7:0] mem [0:1023];
reg [9:0] addr_b_q;

always @(posedge clock0) begin
	if (wren_a) mem[address_a] <= data_a;
	addr_b_q <= address_b;
end

assign q_b       = mem[addr_b_q];
assign q_a       = 8'h00;
assign eccstatus = 1'b0;

endmodule
