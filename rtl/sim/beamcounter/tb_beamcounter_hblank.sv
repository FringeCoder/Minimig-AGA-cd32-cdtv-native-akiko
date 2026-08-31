// agnus_beamcounter's programmable horizontal blanking edges.
//
// What this pins down: HBSTRT and HBSTOP are the only two of the six
// programmable horizontal registers that carry sub-colour-clock position, and
// the core was throwing that away.
//
// The register layout, from WinUAE. custom.cpp masks HBSTRT and HBSTOP to
// 0x7ff while HTOTAL, HSSTRT, HSSTOP and HCENTER are masked to 0xff, so only
// these two have anything above bit 7. drawing.cpp update_hblank() then says
// what the extra bits mean:
//
//     denise_phbstrt       = hbstrt_denise_reg & 0xff;                  // CCK
//     denise_phbstrt_lores = (denise_phbstrt << 1) |
//                            ((hbstrt_denise_reg >> 10) & 1);           // half CCK
//     denise_phbstrt     <<= 3;
//     denise_phbstrt      |= (hbstrt_denise_reg >> 8) & 7;              // 35 ns
//
// So bits 7:0 are the colour clock, and bits 10:8 place the edge within it at
// 35 ns resolution -- of which bit 10 alone is worth half a colour clock.
//
// Our hpos counts half colour clocks (140 ns), which is exactly the resolution
// bit 10 expresses and no more. Bits 9:8 are below what this counter can
// represent at all; they need the 35 ns comparators that the whole chipset
// lacks here, which is a separate and much larger job. So the contract this
// bench holds the RTL to is: bits 7:0 and bit 10, and no pretence about 9:8.
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

		// Bit 10 is half a colour clock. Same colour clock, edge one hpos later.
		wr(A_HBSTRT, 16'd100 | 16'h0400);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=100 + bit10, half a CCK later", at, 9'd201);

		// Unchanged HBSTOP must not have moved with it.
		next_stop(at);
		expect_pos("HBSTOP=20 still aligned", at, 9'd40);

		// And the same for the stop edge on its own.
		wr(A_HBSTOP, 16'd20 | 16'h0400);
		next_stop(at); next_stop(at);
		expect_pos("HBSTOP=20 + bit10, half a CCK later", at, 9'd41);

		// Bits 9:8 are below this counter's resolution. They must be ignored,
		// not folded into the comparison -- a core that treated the register as
		// a plain 11-bit position would land these somewhere far away.
		wr(A_HBSTRT, 16'd100 | 16'h0300);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=100 + bits 9:8 ignored", at, 9'd200);

		// Bits above 10 are not part of the register at all.
		wr(A_HBSTRT, 16'd100 | 16'hF800);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=100 + bits 15:11 ignored", at, 9'd200);

		// A high colour clock still works: 8 bits of CCK reach the whole line.
		wr(A_HBSTRT, 16'd220 | 16'h0400);
		next_start(at); next_start(at);
		expect_pos("HBSTRT=220 + bit10", at, 9'd441);

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
