// agnus_beamcounter's programmable horizontal blanking edges.
//
// What this pins down: HBSTRT and HBSTOP are the only two of the six
// programmable horizontal registers that carry sub-colour-clock position, and
// the core was throwing that away.
//
// The contract: bits 7:0 select the colour clock, and EVERYTHING ABOVE BIT 7 IS
// IGNORED. That is not obvious, it is the opposite of a first reading of
// WinUAE, and getting it wrong shipped a visible regression -- so most of this
// bench exists to hold the upper bits down.
//
// The trap. custom.cpp masks HBSTRT and HBSTOP to 0x7ff where the other four
// horizontal registers get 0xff, and drawing.cpp's update_hblank() builds a
// half-colour-clock position out of bit 10:
//
//     denise_phbstrt_lores = (denise_phbstrt << 1) |
//                            ((hbstrt_denise_reg >> 10) & 1);
//
// which reads like exactly what our hpos wants, since hpos counts half colour
// clocks. It is not. That block runs ONLY inside `if (exthblankon_aga)`, and
// its else branch sets every programmed position to -1: it is Denise's
// extended-HBLANK path, AGA only, and this core does not implement it.
//
// The Agnus-side programmed blanking that this register really drives compares
// against colour clocks and nothing finer:
//
//     hbstrt_cck = hbstrt & 0xff;
//     if (hhp == hbstrt_cck) { agnus_phblank = true; ... }
//
// Feeding bit 10 into the comparison instead shifts every programmed blanking
// edge by half a lores pixel. On hardware, 2026-09-01, that was a blurred
// picture and an OSD drawn twice with a horizontal offset.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../agnus_beamcounter.v tb_beamcounter_hblank.sv && vvp tb

`timescale 1ns/1ps

module tb_beamcounter_hblank;

	localparam [8:1] A_VHPOSR   = 8'h03;   // 9'h006 >> 1, read-only, safe to park on
	localparam [8:1] A_HBSTRT   = 8'hE2;   // 9'h1C4 >> 1
	localparam [8:1] A_HBSTOP   = 8'hE3;   // 9'h1C6 >> 1
	localparam [8:1] A_BEAMCON0 = 8'hEE;   // 9'h1DC >> 1

	localparam [15:0] VARBEAMEN = 16'h0080; // BEAMCON0 bit 7

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         reset = 1;
	reg         cck = 0;
	reg  [15:0] data_in = 0;
	reg  [8:1]  reg_address_in = A_VHPOSR;

	wire [8:0]  hpos;
	wire [10:0] vpos;
	wire        hblank;

	integer errors = 0;

	// Bus clock, clk7_en one in four, cck toggling on every clk7_en -- which is
	// what makes hpos step once per 140 ns half colour clock.
	always #1 clk = ~clk;

	reg [1:0] phase = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end
	always @(posedge clk) if (clk7_en) cck <= ~cck;

	agnus_beamcounter dut (
		.clk(clk), .clk7_en(clk7_en), .reset(reset), .cck(cck),
		.ntsc(1'b0), .aga(1'b1), .ecs(1'b1), .a1k(1'b0),
		.data_in(data_in), .data_out(), .reg_address_in(reg_address_in),
		.lpen_vpos(11'h7FF), .lpen_hpos(9'd0),
		.hpos(hpos), .vpos(vpos),
		._hsync(), ._vsync(), .field1(), .lace(), ._csync(),
		.hblank(hblank), .vblank(), .vbl(), .vblend(),
		.eol(), .eof(), .vbl_int(),
		.htotal_out(), .harddis_out(), .varbeamen_out()
	);

	// The beam counters have no reset -- on the real device they come up zero,
	// and in simulation they come up X and stay X, so say so explicitly. Same
	// power-up state the light pen bench next door assumes.
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

	// hblank is registered off the hpos==hbstrt / hpos==hbstop compares, so the
	// hpos that matched is the one sampled on the edge before the transition
	// becomes visible. Keep a one-clk7_en delayed copy and report that.
	//
	// The counters are free-running, so rather than clearing a flag (and racing
	// the sampler to do it) each transition bumps a counter. A test notes the
	// count, waits for it to move, and reads the position that went with it.
	reg  [8:0]  hpos_d    = 9'd0;
	reg         hblank_d  = 1'b0;
	reg  [8:0]  start_at  = 9'd0;
	reg  [8:0]  stop_at   = 9'd0;
	integer     starts    = 0;
	integer     stops     = 0;

	always @(posedge clk) begin
		if (clk7_en) begin
			hpos_d   <= hpos;
			hblank_d <= hblank;
			if ( hblank && !hblank_d) begin start_at <= hpos_d; starts <= starts + 1; end
			if (!hblank &&  hblank_d) begin stop_at  <= hpos_d; stops  <= stops  + 1; end
		end
	end

	task automatic next_start(output [8:0] at);
		integer mark;
		begin
			mark = starts;
			wait (starts != mark);
			@(posedge clk);
			at = start_at;
		end
	endtask

	task automatic next_stop(output [8:0] at);
		integer mark;
		begin
			mark = stops;
			wait (stops != mark);
			@(posedge clk);
			at = stop_at;
		end
	endtask

	task expect_pos(input [511:0] what, input [8:0] got, input [8:0] want);
		begin
			if (got !== want) begin
				$display("FAIL: %0s: blanking edge at hpos %0d, want %0d", what, got, want);
				errors = errors + 1;
			end else begin
				$display("ok:   %0s at hpos %0d", what, got);
			end
		end
	endtask

	reg [8:0] at;

	initial begin
		repeat (40) @(posedge clk);
		reset = 0;

		// Programmable beam mode, otherwise the hard-wired constants win and
		// nothing written below is looked at.
		wr(A_BEAMCON0, VARBEAMEN);

		// Blank from colour clock 100 to colour clock 20 of the next line, both
		// on a colour clock boundary. hpos counts half colour clocks, so those
		// are hpos 200 and hpos 40.
		wr(A_HBSTRT, 16'd100);
		wr(A_HBSTOP, 16'd20);

		next_start(at); next_start(at);   // discard the line the writes landed in
		expect_pos("HBSTRT=100, aligned", at, 9'd200);
		next_stop(at);
		expect_pos("HBSTOP=20, aligned", at, 9'd40);

		// Bit 10 must NOT move the edge. This is the regression guard: it is the
		// bit that looks like a half colour clock and belongs to a path this
		// core does not have.
		wr(A_HBSTRT, 16'd100 | 16'h0400);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=100 + bit10 ignored", at, 9'd200);

		// Unchanged HBSTOP has not moved either.
		next_stop(at);
		expect_pos("HBSTOP=20 still aligned", at, 9'd40);

		// Same for the stop edge on its own.
		wr(A_HBSTOP, 16'd20 | 16'h0400);
		next_stop(at); next_stop(at);
		expect_pos("HBSTOP=20 + bit10 ignored", at, 9'd40);

		// Bits 9:8 are the 35 ns part of the same field, equally not ours.
		wr(A_HBSTRT, 16'd100 | 16'h0300);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=100 + bits 9:8 ignored", at, 9'd200);

		// And everything above bit 10 is not part of the register at all.
		wr(A_HBSTRT, 16'd100 | 16'hF800);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=100 + bits 15:11 ignored", at, 9'd200);

		// The whole upper byte at once, which is what a caller passing the raw
		// register value would look like.
		wr(A_HBSTRT, 16'd100 | 16'hFF00);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=100 + whole upper byte ignored", at, 9'd200);

		// A high colour clock still works: 8 bits of CCK reach the whole line.
		wr(A_HBSTRT, 16'd220 | 16'h0400);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=220 + bit10 ignored", at, 9'd440);

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
