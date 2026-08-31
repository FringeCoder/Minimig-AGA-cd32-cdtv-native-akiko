// agnus_beamcounter's light pen latch.
//
// The bug this covers: the light pen used to be a combinational mux on VPOSR
// and VHPOSR, so setting BPLCON0 bit 3 held both registers at a constant for as
// long as the bit stayed set. Every beam-wait loop in every program then spins
// forever -- and with no gun connected the constant was zero, so it hung
// whether or not a light pen was involved at all.
//
// So most of what is asserted here is that the counters COME BACK. Frozen in
// the right place is one test; frozen forever has to be impossible.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../agnus_beamcounter.v tb_lightpen_latch.sv && vvp tb

`timescale 1ns/1ps

module tb_lightpen_latch;

	localparam [8:1] A_VHPOSR  = 8'h03;   // 9'h006 >> 1, read-only, safe to park on
	localparam [8:1] A_BPLCON0 = 8'h80;   // 9'h100 >> 1

	// Where in the line to sample. Past the pen's trigger column, so a read is
	// never taken in the cycle the latch is arming.
	localparam [7:0] SAMPLE_COL = 8'd150;

	// What VHPOSR reports for that column. 06f30af decrements the live readback,
	// because the internal hpos runs one colour clock ahead of what real Agnus
	// reports -- so a beam sitting at column 150 reads back as 149.
	//
	// The frozen readback is NOT decremented, and the expectations for the two
	// frozen cases below are deliberately left at their stored values: the light
	// pen latch holds a value already in reported-space, not a counter sample.
	// That asymmetry is the thing this bench now pins down.
	localparam [7:0] SAMPLE_RD = SAMPLE_COL - 8'd1;

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         reset = 1;
	reg         cck = 0;
	reg  [15:0] data_in = 0;
	reg  [8:1]  reg_address_in = A_VHPOSR;
	reg  [10:0] lpen_vpos = 11'h7FF;
	reg  [8:0]  lpen_hpos = 9'd0;

	wire [15:0] data_out;
	wire [8:0]  hpos;
	wire [10:0] vpos;
	wire        vbl, vblend, eol;

	integer errors = 0;

	// Bus clock, clk7_en one in four, cck toggling on every clk7_en -- which is
	// what makes hpos[8:1] step once per 280 ns CCK.
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
		.data_in(data_in), .data_out(data_out), .reg_address_in(reg_address_in),
		.lpen_vpos(lpen_vpos), .lpen_hpos(lpen_hpos),
		.hpos(hpos), .vpos(vpos),
		._hsync(), ._vsync(), .field1(), .lace(), ._csync(),
		.hblank(), .vblank(), .vbl(vbl), .vblend(vblend),
		.eol(eol), .eof(), .vbl_int(),
		.htotal_out(), .harddis_out(), .varbeamen_out()
	);

	// The beam counters have no reset -- on the real device they come up zero,
	// and in simulation they come up X and stay X, so say so explicitly. This
	// is the power-up state the RTL is entitled to assume, not a workaround.
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

	// The two clocks after the wait are not padding. data_out is an always @(*)
	// reg, so a wait that unblocks on the hpos change itself returns in the same
	// delta and would read the previous column. hpos[8:1] steps once per eight
	// bus clocks, so settling costs nothing.
	task goto_line(input [10:0] n);
		begin
			wait (!(vpos == n && hpos[8:1] == SAMPLE_COL));
			wait (  vpos == n && hpos[8:1] == SAMPLE_COL);
			@(posedge clk);
			@(posedge clk);
		end
	endtask

	task expect_eq(input [255:0] what, input [15:0] want);
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

	integer distinct;
	reg [15:0] seen_prev;
	integer i;

	initial begin
		repeat (40) @(posedge clk);
		reset = 0;

		// ---- 1. bit 3 clear: VHPOSR is the beam, untouched -------------------
		goto_line(11'd60);
		expect_eq("no pen, VHPOSR tracks beam", {8'd60, SAMPLE_RD});

		// ---- 2. bit 3 set, no pen: it must still advance ----------------------
		// The fallback freezes it through vblank; the unfreeze at the end of
		// vblank lets it go again. Without that unfreeze this counts 1.
		wr(A_BPLCON0, 16'h0008);
		goto_line(11'd300);            // let a frame boundary go by first
		distinct  = 0;
		seen_prev = 16'hFFFF;
		for (i = 40; i < 300; i = i + 1) begin
			goto_line(i[10:0]);
			if (data_out !== seen_prev) distinct = distinct + 1;
			seen_prev = data_out;
		end
		$display("info: %0d distinct VHPOSR values across 260 display lines", distinct);
		if (distinct < 200) begin
			$display("FAIL: VHPOSR is stuck with BPLCON0 bit 3 set and no pen");
			errors = errors + 1;
		end else begin
			$display("ok:   VHPOSR keeps advancing with bit 3 set and no pen");
		end

		// ---- 3. the fallback fires, and holds, through vblank -----------------
		// WinUAE freezes at the start of vblank at hpos 1, read back as CCK 1.
		goto_line(11'd10);
		expect_eq("fallback frozen in vblank", {8'd0, 8'd1});

		// ---- 4. a real pen freezes at the pen, then lets go -------------------
		lpen_vpos = 11'd100;
		lpen_hpos = 9'd120;            // CCK 60
		goto_line(11'd200);
		expect_eq("frozen at the pen", {8'd100, 8'd60});

		goto_line(11'd50);             // next frame, ahead of the pen's line
		expect_eq("unfrozen after vblank", {8'd50, SAMPLE_RD});

		// ---- 5. clearing bit 3 releases immediately ---------------------------
		goto_line(11'd200);
		expect_eq("frozen at the pen again", {8'd100, 8'd60});
		wr(A_BPLCON0, 16'h0000);
		goto_line(11'd220);
		expect_eq("released by clearing bit 3", {8'd220, SAMPLE_RD});

		// ---- 6. the off-screen sentinel is refused ----------------------------
		lpen_vpos = 11'h7FF;
		wr(A_BPLCON0, 16'h0008);
		goto_line(11'd150);
		expect_eq("sentinel is not a pen position", {8'd150, SAMPLE_RD});

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
