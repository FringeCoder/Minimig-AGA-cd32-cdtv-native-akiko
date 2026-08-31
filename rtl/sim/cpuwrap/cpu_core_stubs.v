// Simulation stubs for the two CPU cores cpu_wrapper instantiates.
//
// These are bench scaffolding, not copies of design logic. cpu_wrapper's own
// logic -- the save state park, the stock-speed throttle and the chip-slot
// guard -- is what tb_cpu_wrapper_park.sv exercises, and none of it needs a
// working 68000 underneath. What it needs is control over busstate and
// ss_at_boundary, which is exactly what the real kernel would be producing.
//
// TG68KdotC_Kernel is VHDL and fx68k is SystemVerilog with a rather larger
// footprint; standing in for both keeps this bench on Icarus alone.
//
// The tb drives the outputs by poking the regs below through the hierarchy,
// the same way rtl/sim/lightpen/tb_lightpen_latch.sv sets the beam counters.

`timescale 1ns/1ps

module TG68KdotC_Kernel
#(
	parameter sr_read = 2,
	parameter vbr_stackframe = 2,
	parameter extaddr_mode = 2,
	parameter mul_mode = 2,
	parameter div_mode = 2,
	parameter bitfield = 2
)
(
	input             clk,
	input             nreset,
	input             clkena_in,
	input      [15:0] data_in,
	input       [2:0] ipl,
	input             ipl_autovector,
	output     [31:0] regin_out,
	output     [31:0] addr_out,
	output     [15:0] data_write,
	output            nwr,
	output            nuds,
	output            nlds,
	output            nresetout,
	output            longword,

	input       [1:0] cpu,
	output      [1:0] busstate,
	output      [3:0] cacr_out,
	output            d_cache_out,
	output     [31:0] vbr_out,

	input       [3:0] ss_reg_index,
	output     [31:0] ss_reg_data,
	output     [31:0] ss_pc,
	output     [31:0] ss_exe_pc,
	output            ss_at_boundary,
	output      [9:0] ss_trap_vector,
	output            ss_trap_active,
	output     [15:0] ss_sr,
	output     [31:0] ss_usp,

	input       [3:0] ss_wr_index,
	input      [31:0] ss_wr_data,
	input             ss_wr_en,
	input             ss_pc_wr,
	input             ss_sr_wr,
	input             ss_usp_wr,
	input             ss_vbr_wr,
	input             ss_cacr_wr,
	input             ss_resume
);

	// What the bench drives.
	reg  [1:0]  r_busstate     = 2'd1;   // 1 = no memory access
	reg         r_at_boundary  = 1'b0;
	reg  [31:0] r_addr         = 32'h00BF0000;

	// Every clkena_in tick the wrapper lets through, counted here so the bench
	// can measure the stock-speed throttle without reaching into the wrapper.
	integer     ticks = 0;
	always @(posedge clk) if (clkena_in) ticks = ticks + 1;

	assign busstate       = r_busstate;
	assign ss_at_boundary = r_at_boundary;
	assign addr_out       = r_addr;

	assign regin_out      = 32'd0;
	assign data_write     = 16'd0;
	assign nwr            = 1'b1;
	assign nuds           = 1'b1;
	assign nlds           = 1'b1;
	assign nresetout      = nreset;
	assign longword       = 1'b0;
	assign cacr_out       = 4'd0;
	assign d_cache_out    = 1'b1;
	assign vbr_out        = 32'd0;
	assign ss_reg_data    = 32'd0;
	assign ss_pc          = 32'd0;
	assign ss_exe_pc      = 32'd0;
	assign ss_trap_vector = 10'd0;
	assign ss_trap_active = 1'b0;
	assign ss_sr          = 16'd0;
	assign ss_usp         = 32'd0;

endmodule


module fx68k
(
	input         clk,
	input         enPhi1,
	input         enPhi2,
	input         extReset,
	input         pwrUp,
	output        oRESETn,
	input         HALTn,
	output        eRWn,
	output        ASn,
	output        LDSn,
	output        UDSn,
	input         DTACKn,
	output        FC0,
	output        FC1,
	output        FC2,
	input         VPAn,
	input         BERRn,
	input         BRn,
	input         BGACKn,
	input         IPL0n,
	input         IPL1n,
	input         IPL2n,
	input  [15:0] iEdb,
	output [15:0] oEdb,
	output [23:1] eab
);

	// Never selected by this bench -- cpucfg[1:0] is non-zero throughout, which
	// is what puts the wrapper on the TG68K path. Idle levels only.
	assign oRESETn = 1'b1;
	assign eRWn    = 1'b1;
	assign ASn     = 1'b1;
	assign LDSn    = 1'b1;
	assign UDSn    = 1'b1;
	assign FC0     = 1'b0;
	assign FC1     = 1'b0;
	assign FC2     = 1'b0;
	assign oEdb    = 16'd0;
	assign eab     = 23'd0;

endmodule
