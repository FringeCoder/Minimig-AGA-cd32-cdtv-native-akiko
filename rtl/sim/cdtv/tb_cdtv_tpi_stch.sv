// SPDX-License-Identifier: GPL-3.0-or-later
//
// CDTV TPI STCH-delivery bench — 2026-05-30 DotC investigation.
//
// DUT  : cdtv_bridge.v in isolation.
// Goal : Reproduce the hardware observation that an injected STCH never
//        manifests as AIR=0x04, even though STCH (bit2) is unmasked and TPI
//        is in mode 1, while SCOR (bit1, lower priority) is serviced fine.
//
//        Codex hypothesis (2026-05-30): the unconditional AIR read-falling
//        clear block clobbers a same-cycle priority-encoder raise when the
//        CPU reads AIR while tp_ilatch[5]==0 (a polling read with no IRQ in
//        service). This bench drives the exact mode/imask the HW trace shows
//        and checks whether reg7 (AIR) EVER reads 0x04.
//
// Scenarios:
//   A. Isolated STCH (mode 1, bit2 unmasked, nothing else): pulse stch, poll
//      AIR. A healthy encoder MUST return 0x04 on one of the reads. If this
//      fails, the encoder is fundamentally broken for STCH and an RTL fix is
//      warranted. If it PASSES, the encoder works in isolation and the HW
//      loss is a race or an upstream (inject-path) problem — do NOT spend a
//      14-min RBF on an encoder rewrite.
//   B. Realistic: SCOR ticks + INT2-handler-style AIR reads (0x02 then 0x00)
//      with periodic STCH injects, many phases. Assert 0x04 seen at least once.
//   C. Adversarial: STCH pulse timed to land its encoder-raise on an AIR
//      read-falling edge while tp_ilatch[5]==0. This is the Codex race; show
//      whether 0x04 is lost.
//
// Sim gotchas (feedback_sim_gotchas): pulse inputs driven exactly one clk;
// read task leaves a trailing @(posedge clk); wait_clocks leads with posedge.

`timescale 1ns / 1ps

module tb_cdtv_tpi_stch;

	initial begin
		#500000 $fatal(1, "tb_cdtv_tpi_stch: watchdog timeout");
	end

	int errs = 0;

	logic clk = 1'b0;
	initial forever #5 clk = ~clk;

	logic        reset = 1'b1;
	logic        sel   = 1'b0;
	logic [23:1] addr  = '0;
	logic [15:0] din   = '0;
	wire  [15:0] dout;
	wire         selack;
	logic        rd  = 1'b0;
	logic        hwr = 1'b0;
	logic        lwr = 1'b0;

	logic  [7:0] ac_rom_byte = 8'h00;
	wire   [5:0] ac_rom_addr;
	wire         cdtv_irq;
	wire   [9:0] cdda_volume;

	wire         cmd_in_pending;
	wire   [7:0] cmd_in_byte;
	logic        cmd_in_pop = 1'b0;

	logic        cmd_out_push = 1'b0;
	logic  [7:0] cmd_out_data = 8'h00;

	logic        sec_byte_push = 1'b0;
	logic  [7:0] sec_byte_data = 8'h00;

	logic        subq_push    = 1'b0;
	logic  [7:0] subq_byte    = 8'h00;

	logic        stch_pulse     = 1'b0;
	logic        sten_pulse_ext = 1'b0;
	logic        scor_pulse     = 1'b0;
	logic        sbcp_pulse     = 1'b0;

	wire         trace_we;
	wire  [63:0] trace_data;

	wire         cdtv_dma_req;
	wire         cdtv_dma_we;
	wire  [23:0] cdtv_dma_baddr;
	wire   [7:0] cdtv_dma_wbyte;

	cdtv_bridge u_dut (
		.clk(clk), .reset(reset),
		.sel(sel), .selack(selack),
		.addr(addr), .din(din), .dout(dout),
		.rd(rd), .hwr(hwr), .lwr(lwr),
		.ac_rom_byte(ac_rom_byte), .ac_rom_addr(ac_rom_addr),
		.cdtv_irq(cdtv_irq), .cdda_volume(cdda_volume),
		.cmd_in_pending(cmd_in_pending),
		.cmd_in_byte(cmd_in_byte),
		.cmd_in_pop(cmd_in_pop),
		.cmd_out_push(cmd_out_push),
		.cmd_out_data(cmd_out_data),
		.sec_byte_push(sec_byte_push),
		.sec_byte_data(sec_byte_data),
		.subq_push(subq_push), .subq_byte(subq_byte),
		.stch_pulse(stch_pulse),
		.sten_pulse_ext(sten_pulse_ext),
		.scor_pulse(scor_pulse),
		.sbcp_pulse(sbcp_pulse),
		.cdtv_dma_req(cdtv_dma_req),
		.cdtv_dma_we(cdtv_dma_we),
		.cdtv_dma_baddr(cdtv_dma_baddr),
		.cdtv_dma_wbyte(cdtv_dma_wbyte),
		.cdtv_dma_ack(1'b0),
		.trace_we(trace_we), .trace_data(trace_data)
	);

	//---------------------------------------------------------------------------
	// BFM helpers
	//---------------------------------------------------------------------------
	task automatic wait_clocks(input int n);
		for (int i = 0; i < n; i++) @(posedge clk);
	endtask

	task automatic cpu_wr_byte(input [15:0] boff, input [7:0] data);
		begin
			@(posedge clk);
			sel  <= 1'b1;
			addr <= boff[15:1];
			din  <= {8'h00, data};
			lwr  <= 1'b1; hwr <= 1'b0;
			@(posedge clk);
			sel <= 1'b0; lwr <= 1'b0; hwr <= 1'b0;
			din <= '0;
			@(posedge clk);
		end
	endtask

	// Read AIR (reg7, offset $BE). Holds rd for HOLD clocks to mimic the slow
	// 4-cycle CPU bus window the real driver produces (so in_air_rd asserts
	// long enough that air_rd_falling lands a known number of clocks later).
	task automatic cpu_rd_air(input int hold, output [7:0] data);
		begin
			@(posedge clk);
			sel  <= 1'b1;
			addr <= 16'h00BE >> 1;
			rd   <= 1'b1;
			for (int i = 0; i < hold; i++) @(posedge clk);
			data = dout[7:0];
			sel <= 1'b0; rd <= 1'b0;
			@(posedge clk);
		end
	endtask

	task automatic pulse_stch;
		begin
			@(posedge clk);
			stch_pulse <= 1'b1;
			@(posedge clk);
			stch_pulse <= 1'b0;
		end
	endtask

	task automatic pulse_scor;
		begin
			@(posedge clk);
			scor_pulse <= 1'b1;
			@(posedge clk);
			scor_pulse <= 1'b0;
		end
	endtask

	task automatic do_reset;
		begin
			reset <= 1'b1;
			wait_clocks(8);
			reset <= 1'b0;
			wait_clocks(4);
		end
	endtask

	// mode 1 + imask exactly as the HW trace: CR=0xf1, imask=0x2e.
	task automatic setup_mode1_imask;
		begin
			cpu_wr_byte(16'h00BD, 8'hF1);  // reg6 CR = 0xF1 (mode 1)
			cpu_wr_byte(16'h00BB, 8'h2E);  // reg5 imask = 0x2E -> [4:0]=0x0E
			wait_clocks(2);
		end
	endtask

	logic [7:0] v;
	bit         saw04;

	initial begin
		do_reset();

		//=====================================================================
		// Scenario A — isolated STCH. Does AIR ever read 0x04 at all?
		//=====================================================================
		setup_mode1_imask();
		pulse_stch();
		saw04 = 1'b0;
		for (int i = 0; i < 8; i++) begin
			cpu_rd_air(4, v);
			if (v == 8'h04) saw04 = 1'b1;
			$display("A: AIR read #%0d = 0x%02h%s", i, v, (v==8'h04)?"  <-- STCH":"");
		end
		if (!saw04) begin
			$display("FAIL [A] isolated STCH never produced AIR=0x04 -> encoder broken for STCH");
			errs++;
		end else
			$display("PASS [A] isolated STCH produced AIR=0x04");

		//=====================================================================
		// Scenario B — realistic: SCOR ticks + handler-style double AIR read,
		// periodic STCH inject across many phases. Assert 0x04 seen.
		//=====================================================================
		do_reset();
		setup_mode1_imask();
		saw04 = 1'b0;
		for (int it = 0; it < 60; it++) begin
			pulse_scor();
			wait_clocks(2);
			cpu_rd_air(4, v);                 // handler 1st read (expect 0x02)
			if (v == 8'h04) saw04 = 1'b1;
			cpu_rd_air(4, v);                 // handler 2nd read (expect 0x00)
			if (v == 8'h04) saw04 = 1'b1;
			// inject STCH on a sweeping sub-iteration phase
			if ((it % 5) == 2) begin
				wait_clocks(it % 7);          // vary phase vs the read loop
				pulse_stch();
				cpu_rd_air(4, v);
				if (v == 8'h04) saw04 = 1'b1;
				cpu_rd_air(4, v);
				if (v == 8'h04) saw04 = 1'b1;
			end
		end
		if (!saw04) begin
			$display("FAIL [B] STCH amid SCOR+polling never produced AIR=0x04");
			errs++;
		end else
			$display("PASS [B] STCH amid SCOR+polling produced AIR=0x04");

		//=====================================================================
		// Scenario C — adversarial: inject STCH so its encoder-raise lands on
		// an AIR read-falling edge while ilatch[5]==0. Sweep the alignment.
		//=====================================================================
		do_reset();
		setup_mode1_imask();
		saw04 = 1'b0;
		for (int ph = 0; ph < 12; ph++) begin
			// idle AIR poll (ilatch[5]==0, returns 0x00), then inject STCH at
			// offset `ph` clocks into the poll so the raise can collide.
			@(posedge clk);
			sel  <= 1'b1; addr <= 16'h00BE >> 1; rd <= 1'b1;
			for (int k = 0; k < 6; k++) begin
				if (k == ph % 6) begin stch_pulse <= 1'b1; end
				else             begin stch_pulse <= 1'b0; end
				@(posedge clk);
			end
			stch_pulse <= 1'b0;
			sel <= 1'b0; rd <= 1'b0;          // falling edge of AIR read here
			@(posedge clk);
			// now poll a few times; healthy encoder should surface 0x04
			for (int k = 0; k < 6; k++) begin
				cpu_rd_air(4, v);
				if (v == 8'h04) saw04 = 1'b1;
			end
			$display("C: phase %0d, saw04=%0d", ph, saw04);
		end
		if (!saw04) begin
			$display("FAIL [C] STCH lost when raise collides with AIR-read fall");
			errs++;
		end else
			$display("PASS [C] STCH survived the AIR-read-fall collision");

		//=====================================================================
		// Scenario D — FAITHFUL to HW: DotC polls AIR continuously (the trace
		// shows 4308 back-to-back reads of 0x00). air_rd_falling then fires
		// every read-period. SCOR ticks ~every 20ms; we inject STCH every so
		// often. Count how many of N STCH injects ever surface as a 0x04 read.
		// If ZERO -> the encoder race explains the HW 0/240 -> fix is warranted.
		// If >0  -> race alone can't explain HW -> suspect the inject path.
		//=====================================================================
		do_reset();
		setup_mode1_imask();
		begin
			int injects; int hits;
			injects = 0; hits = 0;
			for (int it = 0; it < 240; it++) begin
				// occasional SCOR tick (~every 12 reads, like 20ms vs the poll)
				if ((it % 12) == 0) pulse_scor();
				// inject an STCH every 8 reads
				if ((it % 8) == 4) begin pulse_stch(); injects++; end
				cpu_rd_air(4, v);                 // tight continuous AIR poll
				if (v == 8'h04) hits++;
			end
			$display("D: continuous-poll STCH: injects=%0d, 0x04 hits=%0d",
			         injects, hits);
			if (hits == 0) begin
				$display("FAIL [D] continuous-poll lost ALL STCH (race explains HW 0/240)");
				errs++;
			end else
				$display("PASS [D] continuous-poll still surfaced 0x04 (race alone != HW)");
		end

		//=====================================================================
		wait_clocks(4);
		if (errs == 0) $display("RUN: PASS (all STCH scenarios delivered 0x04)");
		else           $display("RUN: FAIL (%0d scenario failures)", errs);
		$finish;
	end

endmodule
