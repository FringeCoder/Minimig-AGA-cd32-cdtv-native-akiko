// Paula audio channel modulation -- the ADKCON attach bits.
//
// Nothing to do with attaching hardware: this is one audio channel modulating
// the next. ADKCON bit n attaches channel n's VOLUME to channel n+1, bit 4+n
// attaches its PERIOD. The modulating channel goes silent and the words it
// fetches become the next channel's volume or period, so the modulation runs at
// the audio DMA rate rather than at whatever rate the CPU manages.
//
// paula_audio_channel.v said "attached modes are not supported" and ADKCON bits
// 0..7 were decoded nowhere in rtl/. Music that used it failed in both
// directions at once: the modulator was heard when it should have been silent,
// and no modulation happened.
//
// Reference is WinUAE audio.cpp loaddat() for what is written and where from,
// its two call sites for when, and audio_update_adkmasks() for the silencing.
// See the header of paula_audio_channel.v for the quoted extracts.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../paula_audio.v ../../paula_audio_channel.v \
//     ../../paula_audio_mixer.v ../../paula_audio_volume.v tb_paula_audio_attach.sv
//   vvp tb

`timescale 1ns/1ps

module tb_paula_audio_attach;

	// Register addresses, as reg_address_in[8:1] -- the byte address >> 1.
	localparam [8:1] A_AUD0LEN = 8'h52;   // 9'h0A4
	localparam [8:1] A_AUD0PER = 8'h53;   // 9'h0A6
	localparam [8:1] A_AUD0VOL = 8'h54;   // 9'h0A8
	localparam [8:1] A_AUD0DAT = 8'h55;   // 9'h0AA
	localparam [8:1] A_AUD1LEN = 8'h5A;   // 9'h0B4
	localparam [8:1] A_AUD1PER = 8'h5B;   // 9'h0B6
	localparam [8:1] A_AUD1VOL = 8'h5C;   // 9'h0B8
	localparam [8:1] A_AUD3LEN = 8'h6A;   // 9'h0D4
	localparam [8:1] A_AUD3PER = 8'h6B;   // 9'h0D6
	localparam [8:1] A_AUD3VOL = 8'h6C;   // 9'h0D8
	localparam [8:1] A_AUD3DAT = 8'h6D;   // 9'h0DA
	localparam [8:1] A_NONE    = 8'h00;

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         cck = 0;
	reg         rst = 1;
	reg         strhor = 0;
	reg  [8:1]  reg_address_in = A_NONE;
	reg  [15:0] data_in = 0;
	reg  [7:0]  adkcon = 0;
	reg  [3:0]  dmaena = 0;

	integer errors = 0;

	always #1 clk = ~clk;

	reg [1:0] phase = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end
	always @(posedge clk) if (clk7_en) cck <= ~cck;

	paula_audio dut (
		.clk(clk), .clk7_en(clk7_en), .cck(cck), .rst(rst), .strhor(strhor),
		.reg_address_in(reg_address_in), .data_in(data_in),
		.adkcon(adkcon), .dmaena(dmaena),
		.audint(), .audpen(4'b0000),
		.dmal(), .dmas(),
		.ldata(), .rdata(), .ldata_okk(), .rdata_okk()
	);

	// Drive on the falling edge. The design samples the address on posedge clk
	// with clk7_en high, so a task that assigns it at a posedge is racing the
	// sampler -- which shows up as writes that land or not depending on what the
	// previous write left the phase at.
	task wr(input [8:1] a, input [15:0] d);
		begin
			@(negedge clk);
			reg_address_in = a; data_in = d;
			@(posedge clk); while (!clk7_en) @(posedge clk);
			@(negedge clk);
			reg_address_in = A_NONE; data_in = 16'h0000;
		end
	endtask

	// A minimal Agnus: keep feeding channel 0 the same word so its state machine
	// keeps running. Real DMA would deliver on the channel's request; the FSM
	// only needs AUDxDAT to arrive, and an extra AUDxDAT in the sample states
	// costs nothing but a length count we have made unreachable.
	reg  [15:0] feed0_word = 16'h0000;
	reg  [15:0] feed3_word = 16'h0000;
	reg         feeding    = 0;

	initial begin
		forever begin
			@(posedge clk);
			if (feeding) begin
				wr(A_AUD0DAT, feed0_word);
				wr(A_AUD3DAT, feed3_word);
				repeat (24) @(posedge clk);
			end
		end
	end

	task expect_eq16(input [511:0] what, input [15:0] got, input [15:0] want);
		begin
			if (got !== want) begin
				$display("FAIL: %0s: got %04x want %04x", what, got, want);
				errors = errors + 1;
			end else begin
				$display("ok:   %0s = %04x", what, got);
			end
		end
	endtask

	// What await_change16 watches. Set before each use.
	reg [15:0] probe;
	reg [2:0]  probe_sel;
	always @(*) begin
		case (probe_sel)
			3'd0:    probe = {9'd0, dut.ach1.audvol};
			3'd1:    probe = dut.ach1.audper;
			default: probe = 16'h0000;
		endcase
	end

	// Wait for a modulation write to land, or give up. Returns 1 on success so a
	// missing strobe reports as its own failure rather than as a timeout.
	task automatic await_change16(input [511:0] what, input [15:0] was,
	                              output integer changed);
		integer guard;
		begin
			changed = 0;
			for (guard = 0; guard < 40000 && !changed; guard = guard + 1) begin
				@(posedge clk);
				if (probe !== was) changed = 1;
			end
			if (!changed) begin
				$display("FAIL: %0s: never changed from %04x", what, was);
				errors = errors + 1;
			end
		end
	endtask

	integer changed;
	reg [15:0] was;

	initial begin
		repeat (40) @(posedge clk);
		rst = 0;
		repeat (20) @(posedge clk);

		// Channel 0 modulates, channel 1 is modulated. Long lengths so the
		// length counter never reloads and nothing here depends on it. Channel 3
		// is set up the same way for the silencing check at the end.
		wr(A_AUD0LEN, 16'h4000);
		wr(A_AUD0PER, 16'd8);
		wr(A_AUD0VOL, 16'd40);
		wr(A_AUD1LEN, 16'h4000);
		wr(A_AUD1PER, 16'd200);
		wr(A_AUD1VOL, 16'd7);
		wr(A_AUD3LEN, 16'h4000);
		wr(A_AUD3PER, 16'd8);
		wr(A_AUD3VOL, 16'd40);

		dmaena     = 4'b1011;   // channels 0, 1 and 3
		feed0_word = 16'h1234;
		feed3_word = 16'h5678;
		feeding    = 1;

		// ---- 1. no attach: channel 1 keeps what it was given ----------------
		adkcon = 8'h00;
		repeat (4000) @(posedge clk);
		expect_eq16("no attach, AUD1VOL untouched", {9'd0, dut.ach1.audvol}, 16'd7);
		expect_eq16("no attach, AUD1PER untouched", dut.ach1.audper, 16'd200);

		// ...and channel 0 is heard.
		if (dut.sample0 === 8'h00) begin
			$display("FAIL: no attach, channel 0 is silent");
			errors = errors + 1;
		end else begin
			$display("ok:   no attach, channel 0 is heard (%02x)", dut.sample0);
		end

		// ---- 2. attach volume: channel 0's data becomes AUD1VOL -------------
		// Only the low 7 bits are a volume, same as a bus write to AUDxVOL.
		probe_sel  = 3'd0;
		feed0_word = 16'h0035;          // 53
		was        = {9'd0, dut.ach1.audvol};
		adkcon     = 8'h01;             // attach volume, channel 0
		await_change16("AUD1VOL follows channel 0", was, changed);
		if (changed)
			expect_eq16("attach volume, AUD1VOL", {9'd0, dut.ach1.audvol}, 16'd53);

		// The period must not have moved with it.
		expect_eq16("attach volume leaves AUD1PER", dut.ach1.audper, 16'd200);

		// ---- 3. and the modulator itself goes quiet -------------------------
		if (dut.sample0 !== 8'h00) begin
			$display("FAIL: attach volume: channel 0 still heard (%02x)", dut.sample0);
			errors = errors + 1;
		end else begin
			$display("ok:   attach volume: channel 0 is silent");
		end

		// A second value proves it keeps following rather than latching once.
		feed0_word = 16'h0011;          // 17
		was        = {9'd0, dut.ach1.audvol};
		await_change16("AUD1VOL follows again", was, changed);
		if (changed)
			expect_eq16("attach volume, second value", {9'd0, dut.ach1.audvol}, 16'd17);

		// ---- 4. attach period: channel 0's data becomes AUD1PER -------------
		probe_sel  = 3'd1;
		adkcon     = 8'h10;             // attach period, channel 0
		feed0_word = 16'h00C8;          // 200 -> pick something else to see it move
		feed0_word = 16'h0140;          // 320
		was        = dut.ach1.audper;
		await_change16("AUD1PER follows channel 0", was, changed);
		if (changed)
			expect_eq16("attach period, AUD1PER", dut.ach1.audper, 16'd320);

		// Channel 0 is silent for the period attach too -- the mask is either bit.
		if (dut.sample0 !== 8'h00) begin
			$display("FAIL: attach period: channel 0 still heard (%02x)", dut.sample0);
			errors = errors + 1;
		end else begin
			$display("ok:   attach period: channel 0 is silent");
		end

		// ---- 5. channel 3 has nothing to modulate but is still silenced -----
		// WinUAE's mask is built from adkcon | (adkcon >> 4) with no special
		// case for the last channel, and loaddat() returns early for nr >= 3.
		adkcon = 8'h08;                 // attach volume, channel 3
		repeat (2000) @(posedge clk);
		if (dut.sample3 !== 8'h00) begin
			$display("FAIL: channel 3 attached but still heard (%02x)", dut.sample3);
			errors = errors + 1;
		end else begin
			$display("ok:   channel 3 attached is silent");
		end

		// ---- 6. clearing the bits brings channel 0 back ---------------------
		adkcon = 8'h00;
		repeat (4000) @(posedge clk);
		if (dut.sample0 === 8'h00) begin
			$display("FAIL: attach cleared, channel 0 still silent");
			errors = errors + 1;
		end else begin
			$display("ok:   attach cleared, channel 0 is heard again (%02x)", dut.sample0);
		end

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#40000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
