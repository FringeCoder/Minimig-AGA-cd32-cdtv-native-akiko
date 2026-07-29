// SPDX-License-Identifier: GPL-3.0-or-later
//
// Akiko M6.2 CDDA bench — RTL-only acceptance for cdda.v.
//
// DUT  : cdda #(.CLK_RATE(441000)) standalone. CLK_RATE picked so the
//        44.1 kHz clock-enable pulses every 10 sim cycles — the bench
//        consumes ~588 CE pulses per sector, so this keeps a full-sector
//        drain at a few thousand cycles instead of millions.
//
// Sim gotchas applied (from feedback_sim_gotchas memory):
//   1. wait_* tasks lead with @(posedge clk).
//   2. n/a — bench drives WRITE pulses directly (no DMA BFM, no UIO bridge).
//
// What this bench actually verifies (the cdda.v contract):
//   A. Reset. WRITE_REQ asserts (buffer has >= 1 sector free). AUDIO_L/R = 0.
//   B. Empty-buffer underflow. AUDIO_CE keeps pulsing every CLK_RATE/44100
//      cycles; AUDIO_L/R stay 0 (silence, not stale data, not glitches).
//   C. Single L/R word push. WRITE pulse pair (LRCK toggles; pulse1 captures
//      DATA, pulse2 commits {DIN_pulse2, DATA_pulse1} into the buffer). On
//      the next AUDIO_CE the sample drains: AUDIO_L = pulse1, AUDIO_R = pulse2.
//   D. One full sector (588 L/R pairs = 1176 WRITE pulses). WRITE_REQ
//      stays asserted (1 sector consumed leaves >=1 sector free in the
//      2048-deep buffer). All 588 stereo samples drain in order.
//   E. Backpressure. Push enough pairs that AVAILABLE_COUNT drops below
//      SECTOR_SIZE → WRITE_REQ deasserts. Drain one sector → WRITE_REQ
//      reasserts.

`timescale 1ns / 1ps

module tb_akiko_cdda;

initial begin
	#5000000 $fatal(1, "tb_akiko_cdda: watchdog timeout");
end

logic clk = 0;
initial forever #5 clk = ~clk;

logic        reset_n = 0;

// DUT IO
logic        write_pulse = 0;
logic [15:0] din         = 16'h0000;
wire         write_req;
wire         audio_ce;
wire  [15:0] audio_l;
wire  [15:0] audio_r;

cdda #(.CLK_RATE(441000)) dut (
	.CLK    (clk),
	.nRESET (reset_n),
	.WRITE_REQ (write_req),
	.WRITE  (write_pulse),
	.DIN    (din),
	.AUDIO_CE (audio_ce),
	.AUDIO_L (audio_l),
	.AUDIO_R (audio_r)
);

// -----------------------------------------------------------------------
// Score keeping
// -----------------------------------------------------------------------
int checks = 0;
int errs   = 0;

task automatic check_eq16(string name, logic [15:0] expected, logic [15:0] actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected 0x%04h got 0x%04h (t=%0t)", name, expected, actual, $time);
		errs++;
	end
endtask

task automatic check_bit(string name, logic expected, logic actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected %0b got %0b (t=%0t)", name, expected, actual, $time);
		errs++;
	end
endtask

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

// One half-sample push. cdda.v uses LRCK to demux pulses into the
// {high, low} 32-bit buffer word. Each 32-bit word holds one stereo
// sample = {right16, left16}; first call writes the left half, second
// call writes the right half AND triggers the commit (WR_REQ=1).
//
// Driving rules:
//   - Hold WRITE high for 1 cycle, then drop (the DUT is rising-edge
//     sensitive: WR_REQ fires on ~OLD_WRITE & WRITE).
//   - DIN must be valid the cycle WRITE rises.
task automatic push_half(input [15:0] sample);
	@(posedge clk);
	din         <= sample;
	write_pulse <= 1'b1;
	@(posedge clk);
	write_pulse <= 1'b0;
endtask

// Push one full L/R stereo pair. After this, one 32-bit word lives in
// the buffer (WRITE_ADDR advanced by 1).
task automatic push_pair(input [15:0] left, input [15:0] right);
	push_half(left);
	push_half(right);
endtask

// Wait for the next AUDIO_CE rising-edge pulse, sample audio outputs the
// cycle of the pulse (cdda.v latches them on cen_44100).
task automatic wait_for_ce_and_capture(output [15:0] cap_l, output [15:0] cap_r);
	@(posedge clk);
	while (!audio_ce) @(posedge clk);
	// audio_ce is high this cycle; AUDIO_L/R have been latched non-blocking
	// in the same always block, so they are visible after posedge.
	cap_l = audio_l;
	cap_r = audio_r;
endtask

// Count CE pulses over a fixed cycle window (no other state change).
task automatic count_ce_for(input int cycles, output int n);
	int i;
	n = 0;
	for (i = 0; i < cycles; i++) begin
		@(posedge clk);
		if (audio_ce) n++;
	end
endtask

// -----------------------------------------------------------------------
// Main sequence
// -----------------------------------------------------------------------

initial begin
	$display("==== tb_akiko_cdda begin ====");
	// Sim-only seed for regs that cdda.v leaves uninitialized (Quartus
	// power-on-zeroes them on FPGA; ModelSim sees X otherwise, and the
	// X + 44100 add poisons cen_44100_cnt forever). Production RTL is
	// not modified — these are forces, released after reset deasserts.
	force dut.cen_44100_cnt = 32'd0;
	force dut.cen_44100     = 1'b0;
	force dut.AUDIO_CE      = 1'b0;
	force dut.AUDIO_L       = 16'h0000;
	force dut.AUDIO_R       = 16'h0000;
	force dut.DATA          = 16'h0000;

	// Hold reset for several cycles
	reset_n = 0;
	repeat (4) @(posedge clk);
	reset_n = 1;
	// Release sim-init forces; from here the DUT runs on its own logic.
	release dut.cen_44100_cnt;
	release dut.cen_44100;
	release dut.AUDIO_CE;
	release dut.AUDIO_L;
	release dut.AUDIO_R;
	release dut.DATA;

	// WRITE_REQ is set to 0 inside the reset block; it only goes high
	// one cycle after deassertion when the reg-update path runs and sees
	// AVAILABLE_COUNT >= SECTOR_SIZE. Wait one cycle before checking.
	@(posedge clk);
	@(posedge clk);

	// -------------------------------------------------------------------
	// Group A: reset state.
	// -------------------------------------------------------------------
	$display("[A] reset state");
	check_bit ("A.write_req asserted after reset", 1'b1, write_req);
	check_eq16("A.audio_l silent after reset",     16'h0000, audio_l);
	check_eq16("A.audio_r silent after reset",     16'h0000, audio_r);

	// -------------------------------------------------------------------
	// Group B: empty-buffer underflow over 200 cycles.
	// CLK_RATE=441000 → CE pulses every 10 cycles → ~20 CE pulses in 200.
	// All of them must show AUDIO_L=AUDIO_R=0 (not glitch, not stale).
	// -------------------------------------------------------------------
	$display("[B] empty-buffer underflow");
	begin
		int n_ce;
		int n_silent_ce = 0;
		int i;
		for (i = 0; i < 200; i++) begin
			@(posedge clk);
			if (audio_ce) begin
				if (audio_l === 16'h0000 && audio_r === 16'h0000) n_silent_ce++;
				n_ce++;
			end
		end
		// 200 cycles / 10 = 20 expected CE pulses, give or take phase.
		check_bit("B.saw multiple CE pulses while empty", 1'b1, (n_ce >= 15));
		// Every CE while empty must be silent.
		check_bit("B.all empty CE pulses silent",         1'b1, (n_silent_ce == n_ce));
	end

	// -------------------------------------------------------------------
	// Group C: single L/R pair drains in order.
	// Push (0xAAAA, 0x5555) and verify next CE delivers L=0xAAAA R=0x5555.
	// -------------------------------------------------------------------
	$display("[C] single L/R pair drain");
	push_pair(16'hAAAA, 16'h5555);
	begin
		logic [15:0] cap_l, cap_r;
		wait_for_ce_and_capture(cap_l, cap_r);
		check_eq16("C.audio_l = 0xAAAA",  16'hAAAA, cap_l);
		check_eq16("C.audio_r = 0x5555",  16'h5555, cap_r);
	end
	// Buffer should now be empty again — next CE must be silent.
	begin
		logic [15:0] cap_l, cap_r;
		wait_for_ce_and_capture(cap_l, cap_r);
		check_eq16("C.silent after drain (L)", 16'h0000, cap_l);
		check_eq16("C.silent after drain (R)", 16'h0000, cap_r);
	end

	// -------------------------------------------------------------------
	// Group D: one full sector (588 L/R pairs) drains in order.
	// Use a deterministic sample stream where left = i, right = ~i so we
	// can spot ordering mistakes. The 44.1 kHz pump runs free in real
	// hardware, but for an ordering test we need to stop it from
	// draining while we push — otherwise the bench reads sample[N+x]
	// where x = pairs the pump consumed during the push window. Force
	// cen_44100=0 for the duration of the fill, then release to drain.
	// -------------------------------------------------------------------
	$display("[D] full-sector drain (588 pairs)");
	force dut.cen_44100 = 1'b0;
	begin
		int i;
		for (i = 0; i < 588; i++) begin
			push_pair(16'(i), ~16'(i));
		end
		// WRITE_REQ should still be asserted (1 sector pushed; 2048-word
		// buffer has room for ~3.4 sectors).
		check_bit("D.write_req still asserted after 1 sector pushed", 1'b1, write_req);
	end
	release dut.cen_44100;

	begin
		int i;
		for (i = 0; i < 588; i++) begin
			logic [15:0] cap_l, cap_r;
			wait_for_ce_and_capture(cap_l, cap_r);
			if (cap_l !== 16'(i) || cap_r !== ~16'(i)) begin
				$display("FAIL D.pair[%0d]: expected (0x%04h,0x%04h) got (0x%04h,0x%04h) (t=%0t)",
					i, 16'(i), ~16'(i), cap_l, cap_r, $time);
				errs++;
				if (errs > 8) i = 588;
			end
			checks++;
		end
	end

	// -------------------------------------------------------------------
	// Group E: backpressure. Fill the buffer with 3+ sectors of pairs and
	// verify WRITE_REQ deasserts when AVAILABLE_COUNT drops below SECTOR_SIZE.
	// Then drain one sector and verify WRITE_REQ reasserts.
	// -------------------------------------------------------------------
	$display("[E] backpressure");
	force dut.cen_44100 = 1'b0;
	begin
		int i;
		// Buffer = 2048 words. Push 1800 pairs (slightly under full) so
		// AVAILABLE_COUNT drops to 248, well below SECTOR_SIZE=588.
		for (i = 0; i < 1800; i++) begin
			push_pair(16'h1000 + 16'(i[11:0]), 16'h2000 + 16'(i[11:0]));
		end
		check_bit("E.write_req deasserted when buffer near-full", 1'b0, write_req);
	end
	release dut.cen_44100;

	// Drain ~600 samples (one full sector) and verify write_req comes back.
	begin
		int i;
		for (i = 0; i < 600; i++) begin
			logic [15:0] cap_l, cap_r;
			wait_for_ce_and_capture(cap_l, cap_r);
		end
		check_bit("E.write_req reasserted after one-sector drain", 1'b1, write_req);
	end

	$display("==== tb_akiko_cdda end: checks=%0d errs=%0d ====", checks, errs);
	$finish;
end

endmodule
