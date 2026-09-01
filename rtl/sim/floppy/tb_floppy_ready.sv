// paula_floppy: the drive _READY line.
//
// _ready was asserted on drive selection alone. A real DD drive asserts it when
// selected AND either the motor is off -- where the line carries the drive ID
// bit rather than a constant -- or the motor is running AND a disk is actually
// inserted. So a spinning empty drive reported ready, and software waiting for
// a disk was told it had one.
//
// WinUAE disk.cpp DISK_status(), bit 5 of CIA-A PRA, active low:
//
//     if (drive_running(drv)) {                       // motor on
//         if (drive_diskready(drv) && ...) st &= ~0x20;
//     } else {                                        // motor off
//         if (cs_df0idhw || dr > 0) { if (drv->idbit) st &= ~0x20; }
//         else                      { if (drive_diskready(drv)) st &= ~0x20; }
//     }
//
// The motor-off half is deliberately still unconditional here: the changelog at
// the top of paula_floppy.v records a 2008 incompatibility from making _READY
// motor-dependent, and with the DD drive ID being all ones the ID bit is 1
// anyway. Test 1 below is what pins that down, so a future change cannot
// quietly reintroduce the 2008 bug while "fixing" the motor gating.
//
// Motor state is driven through the real ports rather than poked, because the
// motor latch only samples _motor on the falling edge of _sel -- that sequence
// is part of what is under test. disk_present and the drive count are set
// directly: the former is normally written by a host command, and modelling
// that protocol would test the protocol rather than this.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb -I ../.. ../../paula_floppy.v ../../MiSTerFloppy*.v \
//     ../../paula_floppy_fifo.v tb_floppy_ready.sv
//   vvp tb

`timescale 1ns/1ps

module tb_floppy_ready;

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         reset = 1;
	reg  [3:0]  _sel = 4'b1111;      // active low, none selected
	reg         _motor = 1'b1;       // active low, motor off
	reg  [1:0]  floppy_drives = 2'd0;

	wire        _ready;

	integer errors = 0;

	always #1 clk = ~clk;

	reg [1:0] phase = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end

	paula_floppy dut (
		.clk(clk), .clk7_en(clk7_en), .clk7n_en(1'b0), .reset(reset),
		.ntsc(1'b0), .sof(1'b0),
		.enable(1'b0), .reg_address_in(8'h00), .data_in(16'h0000), .data_out(),
		.dmal(), .dmas(),
		._step(1'b1), .direc(1'b0), ._sel(_sel), .side(1'b1), ._motor(_motor),
		._track0(), ._change(), ._ready(_ready), ._wprot(),
		.index(), .blckint(), .syncint(), .wordsync(1'b0),
		.IO_ENA(1'b0), .IO_STROBE(1'b0), .IO_WAIT(), .IO_DIN(16'h0000), .IO_DOUT(),
		.fdd_led(),
		.floppy_drives(floppy_drives),
		.floppy_ext_drive(12'd0),
		.floppy_speed_allowed(1'b0), .floppy_speed(),
		.enable_mister_floppy(1'b0),          // keeps flux_inuse low, so _ready = _ready_adf
		.trackdisp(), .secdisp(), .floppy_fwr(), .floppy_frd(),
		.precomp(2'b00),
		.USER_IN(7'h7F), .USER_OUT(),
		.mister_floppy_status()
	);

	// Select one drive, latching the motor state as the real machine does: the
	// motor latch samples _motor on the falling edge of _sel.
	task automatic select_drive(input integer d, input integer motor_running);
		begin
			@(negedge clk);
			_sel = 4'b1111;                       // deselect everything first
			repeat (8) @(posedge clk);
			@(negedge clk);
			_motor = motor_running ? 1'b0 : 1'b1; // active low
			_sel[d] = 1'b0;                       // falling edge latches the motor
			repeat (8) @(posedge clk);
		end
	endtask

	// The drive count latches ONLY while reset is asserted -- paula_floppy.v's
	// "active floppy drive number, updated during reset". Changing
	// floppy_drives during a run does nothing at all, which cost this bench two
	// spurious failures before the cause was found. Adding a drive means
	// resetting the module.
	task automatic reset_with_drives(input [1:0] n);
		begin
			@(negedge clk);
			floppy_drives = n;
			reset = 1'b1;
			repeat (20) @(posedge clk);
			@(negedge clk);
			reset = 1'b0;
			repeat (20) @(posedge clk);
		end
	endtask

	task automatic deselect_all;
		begin
			@(negedge clk);
			_sel = 4'b1111;
			repeat (8) @(posedge clk);
		end
	endtask

	task check_ready(input [511:0] what, input expected_ready);
		begin
			// _ready is active low: 0 means ready.
			if (_ready !== ~expected_ready) begin
				$display("FAIL: %0s: _ready=%b, wanted %s",
				         what, _ready, expected_ready ? "READY (0)" : "NOT READY (1)");
				errors = errors + 1;
			end else begin
				$display("ok:   %0s -> %s", what, expected_ready ? "ready" : "not ready");
			end
		end
	endtask

	initial begin
		repeat (20) @(posedge clk);
		reset = 0;
		repeat (20) @(posedge clk);

		// One drive present, no disk in it.
		floppy_drives = 2'd0;                 // drives==0 means drive 0 only
		dut.disk_present = 4'b0000;
		repeat (8) @(posedge clk);

		// ---- 1. motor off, no disk: STILL READY ------------------------------
		// The 2008 fix. _READY must respond to _SEL with the motor stopped, and
		// the DD drive ID is all ones so the ID bit is 1. Breaking this is the
		// regression that fix was written for.
		select_drive(0, 0);
		check_ready("drive 0 selected, motor off, no disk", 1'b1);

		// ---- 2. motor on, no disk: NOT READY ---------------------------------
		// This is the bug. Before the fix a spinning empty drive said ready.
		select_drive(0, 1);
		check_ready("drive 0 selected, motor ON, no disk", 1'b0);

		// ---- 3. motor on, disk inserted: READY -------------------------------
		dut.disk_present = 4'b0001;
		repeat (8) @(posedge clk);
		select_drive(0, 1);
		check_ready("drive 0 selected, motor ON, disk present", 1'b1);

		// ---- 4. and the disk going away takes ready with it ------------------
		dut.disk_present = 4'b0000;
		repeat (8) @(posedge clk);
		check_ready("disk removed while spinning", 1'b0);

		// ---- 5. nothing selected is never ready ------------------------------
		dut.disk_present = 4'b1111;
		deselect_all;
		check_ready("no drive selected", 1'b0);

		// ---- 6. a drive that does not exist ----------------------------------
		// floppy_drives still says one drive, so drive 1 is absent and must not
		// answer even with a disk and a running motor.
		select_drive(1, 1);
		check_ready("absent drive 1 selected", 1'b0);

		// ---- 7. ...and does once it exists -----------------------------------
		// Needs a reset, per the note on reset_with_drives above.
		reset_with_drives(2'd1);              // two drives
		dut.disk_present = 4'b1111;           // reset cleared it
		repeat (8) @(posedge clk);
		select_drive(1, 1);
		check_ready("drive 1 present, motor ON, disk present", 1'b1);

		// Its disk is what matters, not drive 0's.
		dut.disk_present = 4'b0001;           // only drive 0 has a disk
		repeat (8) @(posedge clk);
		check_ready("drive 1 selected but only drive 0 has a disk", 1'b0);

		// ---- 8. motor off still ignores the disk on a present drive ----------
		select_drive(1, 0);
		check_ready("drive 1, motor off, no disk in it", 1'b1);

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#4000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
