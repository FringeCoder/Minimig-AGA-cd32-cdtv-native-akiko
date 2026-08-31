// agnus_beamcounter's HHPOSR ($1DA).
//
// The register was not decoded at all. WinUAE custom.cpp:
//
//     case 0x1DA:
//         if (!ecs_agnus) goto writeonly;
//         v = HHPOSR();
//
//     static uae_u16 HHPOSR(void) {
//         uae_u16 v = islightpentriggered() ? hhpos_lpen : hhpos;
//         v &= 0xff;
//         return v;
//     }
//
// and hhpos is assigned agnus_hpos on every colour clock except in BEAMCON0
// DUAL mode, where it free-runs and HHPOSW ($1D8) can reseed it. This core has
// no DUAL mode, so outside it HHPOSR is the horizontal half of VHPOSR on its
// own, and that equality is what this bench holds -- including the light pen
// freeze, which HHPOSR honours in WinUAE exactly as VHPOSR does.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../agnus_beamcounter.v tb_beamcounter_hhposr.sv && vvp tb

`timescale 1ns/1ps

module tb_beamcounter_hhposr;

	localparam [8:1] A_VHPOSR  = 8'h03;   // 9'h006 >> 1
	localparam [8:1] A_HHPOSR  = 8'hED;   // 9'h1DA >> 1
	localparam [8:1] A_BPLCON0 = 8'h80;   // 9'h100 >> 1

	localparam [7:0] SAMPLE_COL = 8'd150;
	localparam [7:0] SAMPLE_RD  = SAMPLE_COL - 8'd1;   // the 06f30af decrement

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         reset = 1;
	reg         cck = 0;
	reg         ecs = 1;
	reg  [15:0] data_in = 0;
	reg  [8:1]  reg_address_in = A_VHPOSR;
	reg  [10:0] lpen_vpos = 11'h7FF;
	reg  [8:0]  lpen_hpos = 9'd0;

	wire [15:0] data_out;
	wire [8:0]  hpos;
	wire [10:0] vpos;

	integer errors = 0;

	always #1 clk = ~clk;

	reg [1:0] phase = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end
	always @(posedge clk) if (clk7_en) cck <= ~cck;

	agnus_beamcounter dut (
		.clk(clk), .clk7_en(clk7_en), .reset(reset), .cck(cck),
		.ntsc(1'b0), .aga(1'b1), .ecs(ecs), .a1k(1'b0),
		.data_in(data_in), .data_out(data_out), .reg_address_in(reg_address_in),
		.lpen_vpos(lpen_vpos), .lpen_hpos(lpen_hpos),
		.hpos(hpos), .vpos(vpos),
		._hsync(), ._vsync(), .field1(), .lace(), ._csync(),
		.hblank(), .vblank(), .vbl(), .vblend(),
		.eol(), .eof(), .vbl_int(),
		.htotal_out(), .harddis_out(), .varbeamen_out()
	);

	// Power-up state, as in the light pen bench next door.
	initial begin
		dut.vpos        = 11'd0;
		dut.hpos        = 9'd0;
		dut.end_of_line = 1'b0;
		dut.vpos_inc    = 1'b0;
		dut.long_line   = 1'b0;
		dut.long_frame  = 1'b0;
		dut.extra_line  = 1'b0;
		dut.vser        = 1'b0;
		dut.hblank      = 1'b0;
		dut.vblank      = 1'b0;
		dut.vbl_int     = 1'b0;
		dut._hsync      = 1'b1;
		dut._vsync      = 1'b1;
	end

	task wr(input [8:1] a, input [15:0] d);
		begin
			@(posedge clk); while (!clk7_en) @(posedge clk);
			reg_address_in = a; data_in = d;
			@(posedge clk); while (!clk7_en) @(posedge clk);
			@(posedge clk); while (!clk7_en) @(posedge clk);
			reg_address_in = A_VHPOSR; data_in = 16'h0000;
			@(posedge clk);
		end
	endtask

	// Park the beam on a known column, then settle. data_out is an always @(*)
	// reg, so a wait that unblocks on the hpos change itself would return the
	// previous column.
	task goto_line(input [10:0] n);
		begin
			wait (!(vpos == n && hpos[8:1] == SAMPLE_COL));
			wait (  vpos == n && hpos[8:1] == SAMPLE_COL);
			@(posedge clk);
			@(posedge clk);
		end
	endtask

	// Point the address at a register and let data_out settle.
	task sel(input [8:1] a);
		begin
			reg_address_in = a;
			@(posedge clk);
			@(posedge clk);
		end
	endtask

	task expect_eq(input [511:0] what, input [15:0] want);
		begin
			if (data_out !== want) begin
				$display("FAIL: %0s: got %04x want %04x (beam vpos=%0d hpos=%0d)",
				         what, data_out, want, vpos, hpos[8:1]);
				errors = errors + 1;
			end else begin
				$display("ok:   %0s = %04x", what, data_out);
			end
		end
	endtask

	reg [15:0] vh;

	initial begin
		repeat (40) @(posedge clk);
		reset = 0;

		// ---- 1. free-running: the low byte of VHPOSR, high byte clear --------
		goto_line(11'd60);
		sel(A_HHPOSR);
		expect_eq("HHPOSR tracks the beam", {8'h00, SAMPLE_RD});

		// ---- 2. and it agrees with VHPOSR, read on the same column -----------
		goto_line(11'd61);
		sel(A_VHPOSR);
		vh = data_out;
		sel(A_HHPOSR);
		if (data_out !== {8'h00, vh[7:0]}) begin
			$display("FAIL: HHPOSR %04x is not the low byte of VHPOSR %04x", data_out, vh);
			errors = errors + 1;
		end else begin
			$display("ok:   HHPOSR is the low byte of VHPOSR (%04x)", data_out);
		end

		// ---- 3. the light pen freeze applies to HHPOSR too -------------------
		// WinUAE returns hhpos_lpen when the pen is triggered. The frozen value
		// is not decremented, same asymmetry the light pen bench pins down.
		lpen_vpos = 11'd100;
		lpen_hpos = 9'd120;             // CCK 60
		wr(A_BPLCON0, 16'h0008);        // LPEN
		goto_line(11'd200);
		sel(A_HHPOSR);
		expect_eq("HHPOSR frozen at the pen", {8'h00, 8'd60});

		// ---- 4. released with the pen ----------------------------------------
		wr(A_BPLCON0, 16'h0000);
		goto_line(11'd220);
		sel(A_HHPOSR);
		expect_eq("HHPOSR released by clearing bit 3", {8'h00, SAMPLE_RD});

		// ---- 5. OCS Agnus does not decode it ---------------------------------
		// custom.cpp falls through to write-only when !ecs_agnus, so nothing
		// drives the bus. This core returns zero for an address it does not
		// decode, and HHPOSR must become one of those.
		ecs = 1'b0;
		goto_line(11'd230);
		sel(A_HHPOSR);
		expect_eq("HHPOSR is not decoded without ECS", 16'h0000);

		// VHPOSR is unaffected -- it is not an ECS register.
		sel(A_VHPOSR);
		expect_eq("VHPOSR still reads without ECS", {8'd230, SAMPLE_RD});

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#60000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
