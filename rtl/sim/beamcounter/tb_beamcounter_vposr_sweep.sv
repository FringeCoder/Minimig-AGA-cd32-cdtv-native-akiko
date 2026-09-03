// agnus_beamcounter: does VPOSR/VHPOSR report the line the beam is actually on,
// for every line of a frame?
//
// The siblings in this directory each pin one feature -- HHPOSR's decode, the
// programmable blanking edges, NTSC long lines. None of them sweeps a frame, so
// nothing here would notice a vertical readback that reports the same line
// forever, or one that loses the high bits above line 255. Both are silent
// failures: every value returned is a plausible line number.
//
// Written 2026-09-03 after the vAmigaTS VPOS suite was run on hardware for T7.
// Three tests -- probe1, probe2 and ersy1 -- each read VHPOSR from an interrupt
// handler and each reported vpos $31 (line 49), while the A500 reference photos
// that ship with the suite read $E0-$FE (lines 224-254). probe2 is the sharp
// one: its copper list is a single WAIT at $E881 followed by sixteen back-to-back
// INTREQ writes, and our sixteen results stepped by exactly $0A with the line
// wrap in the right place. So the horizontal half and the interrupt path are
// right, and the vertical half read 49 on a line the copper's own WAIT had just
// matched at 232.
//
// What this bench can and cannot settle. It exercises agnus_beamcounter
// standalone, so it covers the readback half of that observation only. If it
// PASSES, the vertical counter and its readback are correct in isolation and the
// hardware behaviour comes from integration or from the copper's compare -- which
// is the other half, lives in agnus_copper.v, and needs its own bench. A pass
// here is a real result, not a null one: it removes this module from suspicion.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../agnus_beamcounter.v tb_beamcounter_vposr_sweep.sv && vvp tb

`timescale 1ns/1ps

module tb_beamcounter_vposr_sweep;

	localparam [8:1] A_VPOSR   = 8'h02;   // 9'h004 >> 1
	localparam [8:1] A_VHPOSR  = 8'h03;   // 9'h006 >> 1
	localparam [8:1] A_BPLCON0 = 8'h80;   // 9'h100 >> 1

	localparam [7:0] SAMPLE_COL = 8'd150;
	localparam [7:0] SAMPLE_RD  = SAMPLE_COL - 8'd1;   // the 06f30af decrement

	localparam integer PAL_LAST  = 311;   // VTOTAL_PAL_VAL, lines 0..311
	localparam integer NTSC_LAST = 261;   // VTOTAL_NTSC_VAL, lines 0..261

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         reset = 1;
	reg         cck = 0;
	reg         ntsc = 0;
	reg  [15:0] data_in = 0;
	reg  [8:1]  reg_address_in = A_VHPOSR;
	reg  [10:0] lpen_vpos = 11'h7FF;
	reg  [8:0]  lpen_hpos = 9'd0;

	wire [15:0] data_out;
	wire [8:0]  hpos;
	wire [10:0] vpos;

	integer errors = 0;
	integer line;
	integer reported;

	always #1 clk = ~clk;

	reg [1:0] phase = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end
	always @(posedge clk) if (clk7_en) cck <= ~cck;

	agnus_beamcounter dut (
		.clk(clk), .clk7_en(clk7_en), .reset(reset), .cck(cck),
		.ntsc(ntsc), .aga(1'b1), .ecs(1'b1), .a1k(1'b0),
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

	task sel(input [8:1] a);
		begin
			reg_address_in = a;
			@(posedge clk);
			@(posedge clk);
		end
	endtask

	// One line, checked through both registers. VHPOSR carries the low eight
	// bits of the line, VPOSR the top three -- and a readback that drops the
	// top three is exactly what makes line 256 indistinguishable from line 0.
	// VPOSR's other bits are chipset flags and long_frame/long_line, which move
	// on their own, so only [2:0] is compared.
	task check_line(input integer n);
		begin
			goto_line(n[10:0]);

			sel(A_VHPOSR);
			if (data_out[15:8] !== n[7:0]) begin
				$display("FAIL: line %0d: VHPOSR high byte %02x, want %02x",
				         n, data_out[15:8], n[7:0]);
				errors = errors + 1;
			end
			if (data_out[7:0] !== SAMPLE_RD) begin
				$display("FAIL: line %0d: VHPOSR low byte %02x, want %02x (column moved)",
				         n, data_out[7:0], SAMPLE_RD);
				errors = errors + 1;
			end

			sel(A_VPOSR);
			if (data_out[2:0] !== n[10:8]) begin
				$display("FAIL: line %0d: VPOSR[2:0] = %0d, want %0d",
				         n, data_out[2:0], n[10:8]);
				errors = errors + 1;
			end
		end
	endtask

	initial begin
		repeat (40) @(posedge clk);
		reset = 0;

		// The light pen freezes both registers, and a frozen VPOSR is precisely
		// the "same line forever" shape this bench exists to catch -- so hold
		// BPLCON0 bit 3 clear throughout and make that explicit rather than
		// implicit. On hardware the vAmigaTS programs write BPLCON0 $2200, which
		// also has bit 3 clear, so the freeze was not what they hit either.
		wr(A_BPLCON0, 16'h0000);

		// ---- 1. every line of a PAL frame ------------------------------------
		// The whole frame, not a sample of it: a readback that tracks for a
		// while and then sticks is still broken, and so is one that only fails
		// above 255.
		for (line = 0; line <= PAL_LAST; line = line + 1)
			check_line(line);
		$display("ok:   PAL lines 0..%0d report their own line number", PAL_LAST);

		// ---- 2. the lines the hardware failure was measured on ----------------
		// probe1's copper list waits at $E051 $E253 ... $FE6F -- every second
		// line from 224 to 254. On hardware fifteen of those sixteen probes came
		// back empty and the one that landed read line 49. Called out separately
		// from the sweep so a failure names the tests it came from.
		for (line = 224; line <= 254; line = line + 2)
			check_line(line);
		$display("ok:   probe1's lines $E0-$FE ($%0h-$%0h) all report correctly", 224, 254);

		// probe2 waits once at $E881 and reads sixteen times from line 232.
		check_line(232);
		$display("ok:   probe2's line $E8 (232) reports correctly");

		// ---- 3. the same sweep in NTSC ---------------------------------------
		// A shorter frame, so a vertical wrap that is right for one line count
		// and wrong for the other shows up here rather than in a title.
		ntsc = 1'b1;
		repeat (4) @(posedge clk);
		// Let the current frame finish under the new total before sampling.
		@(negedge dut.vpos_inc);
		for (line = 0; line <= NTSC_LAST; line = line + 1)
			check_line(line);
		$display("ok:   NTSC lines 0..%0d report their own line number", NTSC_LAST);

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#600000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
