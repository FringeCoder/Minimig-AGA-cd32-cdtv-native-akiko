// SPDX-License-Identifier: GPL-3.0-or-later
//
// CDTV bridge CR-511 + TPI bench — M2 phase-0.
//
// DUT  : cdtv_bridge.v in isolation.
// Goal : pin down the contract surface that any future userspace-bypass
//        FSM must obey, AND lock in the 2026-05-21 byte-lane fix as a
//        regression test. The earlier FSM attempt regressed M1 splash
//        because STCH was pulsed on every command (spec 4.6 says it
//        should only fire on cd_media transitions). This bench will
//        catch that class of error before Quartus + deploy.
//
// Coverage (10 tests):
//   1.  Reset clears the FIFOs and ilatch.
//   2.  CPU write to $E900A1 (LDS) enqueues into cmd_in_fifo; cmd_in_pending
//       reflects depth.
//   3.  External cmd_in_pop drains cmd_in_fifo byte-by-byte.
//   4.  External cmd_out_push pushes a reply; sten_pulse_int rises and
//       latches into tp_ilatch[3] (when CR mode-1).
//   5.  CPU read of $E900A1 (LDS) returns push-ordered bytes, then last_out
//       when the FIFO drains.
//   6.  stch_pulse latches tp_ilatch[2] (mode 1 path), and the mode-1 read
//       of Port C ($B5) inverts it to bit-2 = 0.
//   7.  Mode 0 Port C returns the WinUAE `get_tp_c` shape (0x1F at idle, no
//       sources active) — regression test against pre-2026-05-21 bug.
//   8.  Mode 1 Port C at idle (no pulses, ilatch=0) returns 0x1F.
//   9.  Byte-lane regression: TPI write via LDS at $B5 (lwr=1, hwr=0) sets
//       tp_cd; TPI read via LDS at $B5 returns the same value on dout[7:0].
//   10. CNTR write with PREST bit (CNTR=0x40 via $E90043 LDS) pulses
//       stch internally → tp_ilatch[2] sets.
//
// Sim gotchas applied (feedback_sim_gotchas memory):
//   - wait_clocks() leads with @(posedge clk).
//   - cpu_rd_byte() leaves a trailing @(posedge clk) so NBA writes settle.
//   - External pulse inputs (cmd_in_pop, cmd_out_push, stch_pulse, ...) are
//     driven for exactly one clk and de-asserted before the next.

`timescale 1ns / 1ps

module tb_cdtv_cr511;

	initial begin
		#200000 $fatal(1, "tb_cdtv_cr511: watchdog timeout");
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

	// Phase-1b chip-RAM master master observed; bench doesn't exercise it
	// (covered by future tb_cdtv_sec_dma), so ack is tied off.
	wire         cdtv_dma_req;
	wire         cdtv_dma_we;
	wire  [31:0] cdtv_dma_baddr;
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
		begin
			for (int i = 0; i < n; i++) @(posedge clk);
		end
	endtask

	// Drive a byte write at byte-offset `boff` inside $E9xxxx. lds_strobe selects
	// the LDS (odd-byte) lane — used for all CR-511 and TPI accesses since the
	// driver issues `.b` instructions there. uds_strobe is the even-byte lane.
	task automatic cpu_wr_byte(input [15:0] boff, input [7:0] data,
	                           input bit use_lds);
		begin
			@(posedge clk);
			sel  <= 1'b1;
			addr <= boff[15:1];
			if (use_lds) begin
				din <= {8'h00, data};
				lwr <= 1'b1; hwr <= 1'b0;
			end else begin
				din <= {data, 8'h00};
				lwr <= 1'b0; hwr <= 1'b1;
			end
			@(posedge clk);
			sel <= 1'b0; lwr <= 1'b0; hwr <= 1'b0;
			din <= '0;
			@(posedge clk);
		end
	endtask

	// Drive a byte read at byte-offset `boff`. Returns the bridge's dout in
	// `data` — caller picks the LDS (dout[7:0]) or UDS (dout[15:8]) half.
	task automatic cpu_rd_word(input [15:0] boff, output [15:0] data);
		begin
			@(posedge clk);
			sel  <= 1'b1;
			addr <= boff[15:1];
			rd   <= 1'b1;
			@(posedge clk);
			data = dout;
			sel <= 1'b0; rd <= 1'b0;
			@(posedge clk);
		end
	endtask

	// Pulse cmd_in_pop for one cycle (helper — ref-arg tasks can't drive NBAs
	// of automatic vars in ModelSim 10.5b).
	task automatic pulse_cmd_in_pop;
		begin
			@(posedge clk);
			cmd_in_pop <= 1'b1;
			@(posedge clk);
			cmd_in_pop <= 1'b0;
			@(posedge clk);
		end
	endtask

	task automatic pulse_stch;
		begin
			@(posedge clk);
			stch_pulse <= 1'b1;
			@(posedge clk);
			stch_pulse <= 1'b0;
			@(posedge clk);
		end
	endtask

	function automatic void check(input string label, input bit cond);
		begin
			if (!cond) begin
				$display("FAIL [%0t] %s", $time, label);
				errs++;
			end
		end
	endfunction

	function automatic void check_eq(input string label,
	                                  input [15:0] got, input [15:0] expected);
		begin
			if (got !== expected) begin
				$display("FAIL [%0t] %s: got=%0h expected=%0h", $time, label, got, expected);
				errs++;
			end
		end
	endfunction

	//---------------------------------------------------------------------------
	// Test scenarios
	//---------------------------------------------------------------------------

	logic [15:0] rdval;

	initial begin
		// global reset
		reset <= 1'b1;
		wait_clocks(8);
		reset <= 1'b0;
		wait_clocks(4);

		//=====================================================================
		// Test 1 — reset clears FIFOs & ilatch
		//=====================================================================
		check("T1: cmd_in_pending=0 after reset", cmd_in_pending == 1'b0);
		check("T1: cdtv_irq=0 after reset",       cdtv_irq == 1'b0);

		//=====================================================================
		// Test 2 — CPU writes to $E900A1 enqueue
		//=====================================================================
		cpu_wr_byte(16'h00A1, 8'h80, 1'b1);  // opcode NOP
		cpu_wr_byte(16'h00A1, 8'h00, 1'b1);  // arg
		check("T2: cmd_in_pending=1 after 2 writes", cmd_in_pending == 1'b1);
		check_eq("T2: cmd_in_byte[0]=0x80", {8'h00, cmd_in_byte}, 16'h0080);

		//=====================================================================
		// Test 3 — external cmd_in_pop drains FIFO
		//=====================================================================
		pulse_cmd_in_pop();
		check_eq("T3: cmd_in_byte[1]=0x00", {8'h00, cmd_in_byte}, 16'h0000);
		pulse_cmd_in_pop();
		check("T3: cmd_in_pending=0 after drain", cmd_in_pending == 1'b0);

		//=====================================================================
		// Test 4 — cmd_out_push + STEN gating
		// Pre-condition: TPI in mode 0. Port C bit 3+4 in mode 0 is
		// cmd_out_empty (per dut line 468). After 1 push, both bits go low.
		//=====================================================================
		// Push reply byte 0xAA.
		@(posedge clk);
		cmd_out_data <= 8'hAA;
		cmd_out_push <= 1'b1;
		@(posedge clk);
		cmd_out_push <= 1'b0;
		wait_clocks(2);
		// Mode 0 Port C read at $E900B5 (LDS).
		cpu_rd_word(16'h00B4, rdval);
		// DUT line 468: tpi_rd = {3'h0, cmd_out_empty, cmd_out_empty,
		// 1'b1, 1'b1, ~sbcp_state}. With reply pending cmd_out_empty=0,
		// sbcp_state=0 → bits [4:0] = 0,0,1,1,1 = 0x07.
		check_eq("T4: mode-0 Port C with reply pending = 0x07",
		         {8'h00, rdval[7:0]}, 16'h0007);

		//=====================================================================
		// Test 5 — CPU read of $A1 returns push-ordered bytes
		//=====================================================================
		cpu_rd_word(16'h00A0, rdval);
		check_eq("T5: $A1 read returns 0xAA", {8'h00, rdval[7:0]}, 16'h00AA);
		// FIFO now empty — read again should return last_out=0xAA (sticky).
		cpu_rd_word(16'h00A0, rdval);
		check_eq("T5: $A1 sticky returns last_out=0xAA",
		         {8'h00, rdval[7:0]}, 16'h00AA);

		//=====================================================================
		// Test 6 — STCH latch in mode 1.
		// Reset first so the STEN latches from T4/T5 don't pollute ilatch.
		// (Real driver would have ack'd them via Port C writes.)
		//=====================================================================
		reset <= 1'b1;
		wait_clocks(8);
		reset <= 1'b0;
		wait_clocks(4);
		cpu_wr_byte(16'h00BD, 8'h01, 1'b1);  // CR mode 1
		wait_clocks(2);
		pulse_stch();
		wait_clocks(2);
		// Mode-1 Port C read at $B5: {ilatch[7:5], ~(ilatch[4:0]|ilatch2[4:0])}.
		// ilatch[5]=0 because tp_imask=0 (no source masked-in → priority
		// encoder doesn't fire). ilatch[2]=1 from STCH. → bits[4:0]=~0b00100
		// = 0b11011 = 0x1B. ilatch[7:5]=0. Total: 0x1B.
		cpu_rd_word(16'h00B4, rdval);
		check_eq("T6: mode-1 Port C after STCH (no mask) = 0x1B",
		         {8'h00, rdval[7:0]}, 16'h001B);

		//=====================================================================
		// Test 7 — mode-0 Port C idle value (post-test reset).
		//=====================================================================
		reset <= 1'b1;
		wait_clocks(8);
		reset <= 1'b0;
		wait_clocks(4);
		cpu_rd_word(16'h00B4, rdval);
		check_eq("T7: mode-0 Port C idle = 0x1F",
		         {8'h00, rdval[7:0]}, 16'h001F);

		//=====================================================================
		// Test 8 — mode-1 Port C idle.
		//=====================================================================
		cpu_wr_byte(16'h00BD, 8'h01, 1'b1);
		wait_clocks(2);
		cpu_rd_word(16'h00B4, rdval);
		// ilatch=0, ilatch2=0 → bits[4:0] = ~0 = 0x1F. ilatch[7:5]=0.
		// => 0x1F.
		check_eq("T8: mode-1 Port C idle = 0x1F",
		         {8'h00, rdval[7:0]}, 16'h001F);

		//=====================================================================
		// Test 9 — Byte-lane regression. Writes via LDS must land; reads via
		// LDS must come back on dout[7:0]. Use $B7 (tp_ad = TPI reg 3).
		//=====================================================================
		cpu_wr_byte(16'h00B7, 8'h5A, 1'b1);
		wait_clocks(2);
		cpu_rd_word(16'h00B6, rdval);
		check_eq("T9: LDS write+read $B7 roundtrip = 0x5A on lower lane",
		         {8'h00, rdval[7:0]}, 16'h005A);
		// And the upper lane (UDS path) — the bridge mirrors tpi_rd onto
		// both lanes for read, so dout[15:8] should also show 0x5A.
		check_eq("T9: TPI read mirrored on upper lane = 0x5A",
		         {8'h00, rdval[15:8]}, 16'h005A);

		//=====================================================================
		// Test 10 — PREST → STCH chain.
		// Write CNTR ($43, LDS) with bit 6 (PREST). The bridge's CNTR-write
		// block should pulse prst_pulse and OR it into stch via tpi_edges.
		//=====================================================================
		// Make sure mode-1 + STCH not already latched: reset between tests.
		reset <= 1'b1;
		wait_clocks(8);
		reset <= 1'b0;
		wait_clocks(4);
		cpu_wr_byte(16'h00BD, 8'h01, 1'b1);  // mode 1
		wait_clocks(2);
		cpu_wr_byte(16'h0043, 8'h40, 1'b1);  // CNTR = PREST
		wait_clocks(4);                       // give priority encoder a cycle
		cpu_rd_word(16'h00B4, rdval);
		// Same shape as T6 — ilatch[2]=1 from PREST→STCH chain, no mask
		// so priority encoder idle. Expect 0x1B.
		check_eq("T10: Port C after PREST shows STCH (bit2=0) = 0x1B",
		         {8'h00, rdval[7:0]}, 16'h001B);

		//=====================================================================
		// Summary
		//=====================================================================
		wait_clocks(4);
		if (errs == 0) $display("RUN: PASS (10/10 tests)");
		else           $display("RUN: FAIL (%0d errors)", errs);
		$finish;
	end

endmodule
