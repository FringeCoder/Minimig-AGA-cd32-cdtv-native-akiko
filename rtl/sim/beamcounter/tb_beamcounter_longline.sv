// agnus_beamcounter: NTSC long lines.
//
// An NTSC line is 227.5 colour clocks, which the chipset produces by
// alternating 227 and 228. This core ran 227 flat: the long_line register
// existed and toggled, and nothing consumed it, so every NTSC title's raster
// timing was off by half a colour clock per line and a whole one every other
// line.
//
// WinUAE custom.cpp is the reference for both halves:
//
//     if (!(new_beamcon0 & BEAMCON0_PAL) && !(new_beamcon0 & BEAMCON0_LOLDIS)) {
//         lol = lol ? false : true;
//         linetoggle = true;
//     } else {
//         lol = false;
//         linetoggle = false;
//     }
//     ...
//     maxhpos = maxhpos_short + lol;
//
// and for VPOSW, custom.cpp's VPOSW() handler:
//
//     // LOL is always reset when VPOSW is written to.
//     if (lol) { lol = false; setmaxhpos(); }
//
// A write RESETS the alternation. It does not load it from a data bit. This
// bench originally asserted the opposite, taken from custom.cpp:7712 -- which
// is inside restore_custom(), a savestate blob reader, not the register
// handler. The implementation and the bench came from the same misreading, so
// they agreed with each other and CI passed while a PAL screen was wrong.
//
// What this measures is the last colour clock index of each line, which is one
// less than the line length. 226 is a 227-clock line, 227 a 228-clock one.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../agnus_beamcounter.v tb_beamcounter_longline.sv && vvp tb

`timescale 1ns/1ps

module tb_beamcounter_longline;

	localparam [8:1] A_VHPOSR   = 8'h03;   // 9'h006 >> 1
	localparam [8:1] A_VPOSW    = 8'h15;   // 9'h02A >> 1
	localparam [8:1] A_BEAMCON0 = 8'hEE;   // 9'h1DC >> 1

	localparam [15:0] VARBEAMEN = 16'h0080;  // BEAMCON0 bit 7
	localparam [15:0] LOLDIS    = 16'h0800;  // BEAMCON0 bit 11
	localparam [15:0] BC_PAL    = 16'h0020;  // BEAMCON0 bit 5

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         reset = 1;
	reg         cck = 0;
	reg         ntsc = 0;
	reg  [15:0] data_in = 0;
	reg  [8:1]  reg_address_in = A_VHPOSR;

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

	// LONG_LINES defaults OFF in the RTL -- the alternation is not currently
	// shipped, see the comment on the parameter. Turned on here so this bench
	// keeps testing the feature itself.
	agnus_beamcounter #(.LONG_LINES(1'b1)) dut (
		.clk(clk), .clk7_en(clk7_en), .reset(reset), .cck(cck),
		.ntsc(ntsc), .aga(1'b1), .ecs(1'b1), .a1k(1'b0),
		.data_in(data_in), .data_out(), .reg_address_in(reg_address_in),
		.lpen_vpos(11'h7FF), .lpen_hpos(9'd0),
		.hpos(hpos), .vpos(vpos),
		._hsync(), ._vsync(), .field1(), .lace(), ._csync(),
		.hblank(), .vblank(), .vbl(), .vblend(),
		.eol(), .eof(), .vbl_int(),
		.htotal_out(), .harddis_out(), .varbeamen_out()
	);

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
			@(negedge clk);
			reg_address_in = a; data_in = d;
			@(posedge clk); while (!clk7_en) @(posedge clk);
			@(negedge clk);
			reg_address_in = A_VHPOSR; data_in = 16'h0000;
		end
	endtask

	// The last colour clock of each line. Track the running maximum and latch it
	// when the line wraps -- end_of_line is registered, so hpos has already
	// moved on by the time it is visible.
	reg  [7:0]  cck_max  = 8'd0;
	reg  [7:0]  last_cck = 8'd0;
	integer     lines    = 0;

	always @(posedge clk) begin
		if (clk7_en) begin
			if (dut.end_of_line) begin
				last_cck <= cck_max;
				cck_max  <= 8'd0;
				lines    <= lines + 1;
			end else if (hpos[8:1] > cck_max) begin
				cck_max <= hpos[8:1];
			end
		end
	end

	// Collect the last-colour-clock value for n consecutive lines.
	task automatic sample_lines(input integer n, output integer lo, output integer hi,
	                            output integer total);
		integer i, mark;
		begin
			lo = 999; hi = 0; total = 0;
			for (i = 0; i < n; i = i + 1) begin
				mark = lines;
				wait (lines != mark);
				@(posedge clk);
				if (last_cck < lo) lo = last_cck;
				if (last_cck > hi) hi = last_cck;
				total = total + last_cck;
			end
		end
	endtask

	task check(input [511:0] what, input integer got, input integer want);
		begin
			if (got !== want) begin
				$display("FAIL: %0s: got %0d, want %0d", what, got, want);
				errors = errors + 1;
			end else begin
				$display("ok:   %0s = %0d", what, got);
			end
		end
	endtask

	integer lo, hi, total;

	initial begin
		// ---- 1. PAL: every line the same length ------------------------------
		ntsc = 1'b0;
		repeat (40) @(posedge clk);
		reset = 0;
		sample_lines(2, lo, hi, total);        // discard the first, settling
		sample_lines(8, lo, hi, total);
		check("PAL: shortest line", lo, 226);
		check("PAL: longest line",  hi, 226);

		// ---- 2. NTSC: 227 and 228 alternating --------------------------------
		reset = 1;
		ntsc  = 1'b1;
		repeat (40) @(posedge clk);
		reset = 0;
		sample_lines(2, lo, hi, total);
		sample_lines(8, lo, hi, total);
		check("NTSC: shortest line", lo, 226);
		check("NTSC: longest line",  hi, 227);
		// Eight lines, alternating, so four of each: 4*226 + 4*227 = 1812. That
		// is the 227.5 average the chipset is producing.
		check("NTSC: eight lines average 227.5", total, 1812);

		// ---- 3. LOLDIS pins it to the short line -----------------------------
		// BEAMCON0 bit 11. This core gates it on varbeamen, so set both.
		wr(A_BEAMCON0, VARBEAMEN | LOLDIS);
		sample_lines(3, lo, hi, total);
		sample_lines(8, lo, hi, total);
		check("LOLDIS: shortest line", lo, 226);
		check("LOLDIS: longest line",  hi, 226);

		// ---- 4. and clearing it starts the alternation again -----------------
		wr(A_BEAMCON0, VARBEAMEN);
		sample_lines(3, lo, hi, total);
		sample_lines(8, lo, hi, total);
		check("LOLDIS cleared: longest line", hi, 227);

		// ---- 5. BEAMCON0 PAL also stops it -----------------------------------
		wr(A_BEAMCON0, VARBEAMEN | BC_PAL);
		sample_lines(3, lo, hi, total);
		sample_lines(8, lo, hi, total);
		check("BEAMCON0 PAL: longest line", hi, 226);
		wr(A_BEAMCON0, VARBEAMEN);
		sample_lines(3, lo, hi, total);

		// ---- 6. VPOSW always RESETS the alternation -------------------------
		// Whatever the data carries. Bit 7 set is the case that matters: it is
		// what the first version of this wrongly loaded into long_line, and on
		// PAL that left a 228 colour clock line on screen.
		reset = 1; ntsc = 1'b1; repeat (40) @(posedge clk); reset = 0;
		sample_lines(3, lo, hi, total);
		force dut.long_line = 1'b1;      // pretend we are mid-alternation
		release dut.long_line;
		wr(A_VPOSW, 16'h0080);           // bit 7 SET
		if (dut.long_line !== 1'b0) begin
			$display("FAIL: VPOSW with bit 7 set did not reset long_line");
			errors = errors + 1;
		end else begin
			$display("ok:   VPOSW with bit 7 set resets long_line");
		end

		wr(A_VPOSW, 16'hFFFF);           // every bit set
		if (dut.long_line !== 1'b0) begin
			$display("FAIL: VPOSW with all bits set did not reset long_line");
			errors = errors + 1;
		end else begin
			$display("ok:   VPOSW with all bits set resets long_line");
		end

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#80000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
