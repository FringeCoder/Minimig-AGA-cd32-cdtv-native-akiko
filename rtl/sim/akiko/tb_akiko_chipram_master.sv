// SPDX-License-Identifier: GPL-3.0-or-later
//
// Akiko M5 chip-RAM master bench.
//
// DUT  : chipdma_arb -- the wrapper that sits between minimig.v's chipset
//        DMA signals and sdram_ctrl's chipDMA port. Default forwards
//        minimig untouched; when minimig is idle and akiko_dma_req is
//        asserted, claims the slot for akiko, runs one byte access, and
//        pulses dma_ack with the read byte.
//
// BFM  : Behavioural SDRAM stub modelled on sdram_ctrl's chipDMA semantics
//        (active-low chipDMA/chipRW, byte enables chipL/chipU active-low,
//        word-addressed chipAddr[24:1], fixed N-cycle read latency).
//        A 64 KiB sparse byte memory backs both sides so we can preload
//        with $writememh-style backdoor writes via the `mem` array and
//        check writes by reading it back.
//
// What we verify:
//   1. Akiko byte read (even addr -> upper byte) returns preloaded value.
//   2. Akiko byte read (odd addr -> lower byte) returns preloaded value.
//   3. Akiko byte writes land in the correct byte half of memory.
//   4. Multi-byte sequential reads and writes (mirroring TX/RX bursts).
//   5. Minimig chipDMA is never preempted: when minimig pulses chipDMA,
//      the access reaches sdram_stub on the same cycle as it would have
//      without the arbiter, and the read data is unchanged.
//   6. rx_inflight handshake: between two back-to-back akiko acks there
//      is at least one cycle of !dma_ack while akiko keeps dma_req high.
//      Same-cycle ack would defeat akiko.v:522-528.

`timescale 1ns / 1ps

module tb_akiko_chipram_master;

// -----------------------------------------------------------------------
// Watchdog
// -----------------------------------------------------------------------
initial begin
	#500000 $fatal(1, "tb_akiko_chipram_master: watchdog timeout");
end

// -----------------------------------------------------------------------
// Clock
// -----------------------------------------------------------------------
logic clk = 0;
initial forever #5 clk = ~clk;  // 100 MHz nominal

// c_7m -- chipset slot clock. In real hardware c_7m = clk_sys / 4 and
// the arbiter runs on clk_sys. To model the same ratio, the bench's
// `clk` plays the role of clk_sys, so c_7m toggles every 2 cycles ->
// c_7m period = 4 cycles, matching real ratios.
logic c_7m = 0;
logic [1:0] c_7m_div = 0;
always @(posedge clk) begin
	c_7m_div <= c_7m_div + 2'd1;
	if (c_7m_div == 2'd1) begin
		c_7m <= ~c_7m;
		c_7m_div <= 2'd0;
	end
end

logic reset = 1;

// -----------------------------------------------------------------------
// Akiko-side stimulus signals (we play the role of akiko)
// -----------------------------------------------------------------------
logic        akiko_dma_req   = 0;
logic        akiko_dma_we    = 0;
logic [23:0] akiko_dma_baddr = 0;
logic  [7:0] akiko_dma_wbyte = 0;
wire   [7:0] akiko_dma_rbyte;
wire         akiko_dma_ack;

// -----------------------------------------------------------------------
// Minimig-side stimulus signals (we play the role of the chipset DMA)
// Active-low convention matches Minimig.sv wiring (_ram_oe, _ram_we,
// _ram_bhe, _ram_ble): 0 = active, 1 = idle.
// -----------------------------------------------------------------------
logic [24:1] chip_in_addr = 0;
logic        chip_in_l    = 1;  // idle (byte enable inactive)
logic        chip_in_u    = 1;
logic        chip_in_rw   = 1;  // idle (1=read which would be active LOW)
logic        chip_in_dma  = 1;  // idle
logic [15:0] chip_in_wr   = 0;

// -----------------------------------------------------------------------
// SDRAM-stub side (output of arbiter, input of stub)
// -----------------------------------------------------------------------
wire  [24:1] chip_out_addr;
wire         chip_out_l;
wire         chip_out_u;
wire         chip_out_rw;
wire         chip_out_dma;
wire  [15:0] chip_out_wr;
wire  [15:0] chip_in_rd;

// -----------------------------------------------------------------------
// DUT
// -----------------------------------------------------------------------
chipdma_arb u_dut (
	.clk             (clk             ),
	.reset           (reset           ),
	.c_7m            (c_7m            ),

	.chip_in_addr    (chip_in_addr    ),
	.chip_in_l       (chip_in_l       ),
	.chip_in_u       (chip_in_u       ),
	.chip_in_rw      (chip_in_rw      ),
	.chip_in_dma     (chip_in_dma     ),
	.chip_in_wr      (chip_in_wr      ),

	.akiko_dma_req   (akiko_dma_req   ),
	.akiko_dma_we    (akiko_dma_we    ),
	.akiko_dma_baddr (akiko_dma_baddr ),
	.akiko_dma_wbyte (akiko_dma_wbyte ),
	.akiko_dma_rbyte (akiko_dma_rbyte ),
	.akiko_dma_ack   (akiko_dma_ack   ),

	.chip_out_addr   (chip_out_addr   ),
	.chip_out_l      (chip_out_l      ),
	.chip_out_u      (chip_out_u      ),
	.chip_out_rw     (chip_out_rw     ),
	.chip_out_dma    (chip_out_dma    ),
	.chip_out_wr     (chip_out_wr     ),
	.chip_in_rd      (chip_in_rd      )
);

// -----------------------------------------------------------------------
// Behavioural SDRAM stub. Models sdram_ctrl's chipDMA semantics: latch
// the request the moment chipDMA OR chipRW goes active-low, present the
// read result LATENCY cycles later. Writes commit immediately. 64 KiB of
// byte memory backs the model; the upper bits of chip_out_addr select an
// 8-byte page modulo 8KiB so we can use small addresses.
// -----------------------------------------------------------------------
// In hardware, sdram_ctrl latches reads at its state 9 (= 9 clk_114
// cycles after slot start). With clk_sys = clk_114 / 4, that maps to
// ~2.25 clk_sys cycles. The bench's pipeline adds one extra clock for
// the chipRD_r register output, so LATENCY=1 here gives chipRD valid
// at clk_sys cycle 2 of the slot (matches hardware closely; arbiter
// samples at slot_cnt=3).
localparam int LATENCY = 1;

logic [7:0] mem [65536];

// Pipeline regs to model fixed read latency. Sized to 3 (LATENCY=2 + 1).
logic [15:0] rd_pipe [3];
logic        rd_valid_pipe [3];
logic [15:0] chipRD_r;

assign chip_in_rd = chipRD_r;

initial begin
	int ii;
	for (ii = 0; ii < 65536; ii++) mem[ii] = 8'h00;
	for (ii = 0; ii < 3; ii++) begin
		rd_pipe[ii] = 16'h0000;
		rd_valid_pipe[ii] = 1'b0;
	end
	chipRD_r = 16'h0000;
end

// Address into the byte memory: word index in [15:1], byte selector in [0]
// (functions inlined as wires below to avoid ModelSim ASE 10.5b issues
// with function-call LHS indexes inside non-blocking assignments).
wire [15:0] hi_idx = {chip_out_addr[15:1], 1'b0};
wire [15:0] lo_idx = {chip_out_addr[15:1], 1'b1};

always @(posedge clk) begin
	// Shift pipeline (unrolled -- ASE 10.5b is happier without the for loop).
	rd_pipe[2]       <= rd_pipe[1];
	rd_pipe[1]       <= rd_pipe[0];
	rd_valid_pipe[2] <= rd_valid_pipe[1];
	rd_valid_pipe[1] <= rd_valid_pipe[0];
	rd_pipe[0]       <= 16'h0000;
	rd_valid_pipe[0] <= 1'b0;

	// Latch new accesses.
	if (~chip_out_dma | ~chip_out_rw) begin
		if (chip_out_rw) begin
			// READ
			rd_pipe[0][15:8] <= chip_out_u ? 8'hxx : mem[hi_idx];
			rd_pipe[0][7:0]  <= chip_out_l ? 8'hxx : mem[lo_idx];
			rd_valid_pipe[0] <= 1'b1;
		end else begin
			// WRITE
			if (~chip_out_u) mem[hi_idx] <= chip_out_wr[15:8];
			if (~chip_out_l) mem[lo_idx] <= chip_out_wr[7:0];
		end
	end

	// Drive chipRD when the pipeline produces a valid read.
	if (rd_valid_pipe[LATENCY]) chipRD_r <= rd_pipe[LATENCY];
end

// -----------------------------------------------------------------------
// Score keeping
// -----------------------------------------------------------------------
int checks = 0;
int errs   = 0;

task automatic check8(string name, logic [7:0] expected, logic [7:0] actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected 0x%02h got 0x%02h (t=%0t)", name, expected, actual, $time);
		errs++;
	end
endtask

task automatic check_byte_at(string name, int unsigned idx, logic [7:0] expected);
	checks++;
	if (mem[idx] !== expected) begin
		$display("FAIL %s: mem[0x%0h] expected 0x%02h got 0x%02h (t=%0t)",
		         name, idx, expected, mem[idx], $time);
		errs++;
	end
endtask

// -----------------------------------------------------------------------
// Stimulus tasks
// -----------------------------------------------------------------------

// Drive akiko_dma_req for a single byte read; wait for ack; sample rbyte.
task automatic akiko_read_byte(input [23:0] baddr, input [7:0] expected, input string label);
	int timeout;
	@(posedge clk);
	akiko_dma_req   <= 1'b1;
	akiko_dma_we    <= 1'b0;
	akiko_dma_baddr <= baddr;
	akiko_dma_wbyte <= 8'h00;
	timeout = 200;
	while (!akiko_dma_ack && timeout > 0) begin
		@(posedge clk);
		timeout--;
	end
	if (timeout == 0) begin
		$display("FAIL %s: timeout waiting for ack (t=%0t)", label, $time);
		errs++;
	end else begin
		check8(label, expected, akiko_dma_rbyte);
	end
	akiko_dma_req <= 1'b0;
	@(posedge clk);
endtask

// Drive akiko_dma_req for a single byte write; wait for ack.
task automatic akiko_write_byte(input [23:0] baddr, input [7:0] value, input string label);
	int timeout;
	@(posedge clk);
	akiko_dma_req   <= 1'b1;
	akiko_dma_we    <= 1'b1;
	akiko_dma_baddr <= baddr;
	akiko_dma_wbyte <= value;
	timeout = 200;
	while (!akiko_dma_ack && timeout > 0) begin
		@(posedge clk);
		timeout--;
	end
	if (timeout == 0) begin
		$display("FAIL %s: write timeout (t=%0t)", label, $time);
		errs++;
	end
	akiko_dma_req <= 1'b0;
	@(posedge clk);
endtask

// Mimic an Agnus DMA slot: drive minimig's chipDMA inputs for one cycle.
// `do_write` selects read vs write. The arbiter MUST forward this to
// sdram_stub on the same cycle.
task automatic minimig_dma_slot(input [24:1] waddr, input do_write,
                                input [15:0] wr_data, input [1:0] be_lu);
	@(posedge clk);
	chip_in_addr <= waddr;
	chip_in_u    <= ~be_lu[1];   // be_lu[1]=1 -> upper enabled (chipU=0)
	chip_in_l    <= ~be_lu[0];   // be_lu[0]=1 -> lower enabled (chipL=0)
	chip_in_rw   <= ~do_write;   // active-low: 0 means write
	chip_in_dma  <= do_write ? 1'b1 : 1'b0;  // active-low for read
	chip_in_wr   <= wr_data;
	@(posedge clk);
	// Hold one cycle to mimic chipset slot, then idle.
	chip_in_dma  <= 1'b1;
	chip_in_rw   <= 1'b1;
	chip_in_l    <= 1'b1;
	chip_in_u    <= 1'b1;
endtask

// Backdoor preload of byte memory.
task automatic preload(input int unsigned idx, input [7:0] value);
	mem[idx] = value;
endtask

// -----------------------------------------------------------------------
// Main test body
// -----------------------------------------------------------------------
initial begin
	$display("tb_akiko_chipram_master: start (LATENCY=%0d)", LATENCY);

	// Reset
	reset = 1;
	repeat (4) @(posedge clk);
	reset = 0;
	repeat (2) @(posedge clk);

	// ---- Test 1: byte read, even addr (upper byte) ----
	preload(16'h0100, 8'hA5);   // byte at addr 0x100 -> word 0x80 upper half
	akiko_read_byte(24'h000100, 8'hA5, "test1: read upper byte");

	// ---- Test 2: byte read, odd addr (lower byte) ----
	preload(16'h0101, 8'h5A);   // byte at addr 0x101 -> word 0x80 lower half
	akiko_read_byte(24'h000101, 8'h5A, "test2: read lower byte");

	// ---- Test 3: byte writes land in correct half ----
	akiko_write_byte(24'h000200, 8'hDE, "test3a: write upper byte");
	akiko_write_byte(24'h000201, 8'hAD, "test3b: write lower byte");
	check_byte_at("test3a: mem[0x200]", 16'h0200, 8'hDE);
	check_byte_at("test3b: mem[0x201]", 16'h0201, 8'hAD);

	// ---- Test 4: 8-byte sequential burst (mirrors TX command ----
	for (int i = 0; i < 8; i++) preload(16'h0300 + i, 8'h10 + i[7:0]);
	for (int i = 0; i < 8; i++) begin
		akiko_read_byte(24'h000300 + i, 8'h10 + i[7:0], $sformatf("test4: read[%0d]", i));
	end

	// ---- Test 5: rx_inflight gap. Two back-to-back reads with akiko keeping
	//      dma_req high between them: dma_ack must drop for >=1 cycle ----
	preload(16'h0400, 8'h11);
	preload(16'h0401, 8'h22);
	begin : t5
		int low_cycles;
		int timeout;
		akiko_dma_req   <= 1'b1;
		akiko_dma_we    <= 1'b0;
		akiko_dma_baddr <= 24'h000400;
		// First ack
		timeout = 200;
		@(posedge clk);
		while (!akiko_dma_ack && timeout > 0) begin @(posedge clk); timeout--; end
		check8("test5: first byte", 8'h11, akiko_dma_rbyte);
		// Now bump address but keep dma_req asserted; arbiter should drop ack
		// for at least one cycle before the next ack.
		akiko_dma_baddr <= 24'h000401;
		low_cycles = 0;
		timeout = 200;
		@(posedge clk);
		while (!akiko_dma_ack && timeout > 0) begin
			low_cycles++;
			@(posedge clk);
			timeout--;
		end
		if (low_cycles < 1) begin
			$display("FAIL test5: ack stayed high without gap (low_cycles=%0d t=%0t)",
			         low_cycles, $time);
			errs++;
		end else begin
			checks++;
			$display("PASS test5: ack gap = %0d cycles", low_cycles);
		end
		check8("test5: second byte", 8'h22, akiko_dma_rbyte);
		akiko_dma_req <= 1'b0;
		@(posedge clk);
	end

	// ---- Test 6: minimig chipDMA priority. Pulse minimig's chipDMA every
	//      few cycles for a stretch; concurrently raise akiko_dma_req. The
	//      arbiter MUST forward chip_in_* straight through whenever they
	//      are active. ----
	preload(16'h0500, 8'hCC);
	akiko_dma_req   <= 1'b1;
	akiko_dma_we    <= 1'b0;
	akiko_dma_baddr <= 24'h000500;
	begin : t6
		automatic int slot;
		automatic int t6_timeout;
		for (slot = 0; slot < 4; slot++) begin
			minimig_dma_slot(24'h001000 + slot, /*do_write=*/0,
			                 16'h0000, 2'b11);
		end
		t6_timeout = 400;
		while (!akiko_dma_ack && t6_timeout > 0) begin
			@(posedge clk); t6_timeout--;
		end
		if (t6_timeout == 0) begin
			$display("FAIL test6: akiko ack never came");
			errs++;
		end else begin
			check8("test6: akiko got byte after minimig traffic",
			       8'hCC, akiko_dma_rbyte);
		end
		akiko_dma_req <= 1'b0;
	end

	// ---- Test 7: forwarding fidelity. Hold akiko_dma_req high. Drive a
	//      minimig slot. While that slot is active, chip_out_addr/chip_out_dma
	//      must equal chip_in_addr/chip_in_dma exactly (no arbiter
	//      interference). ----
	preload(16'h0600, 8'h77);
	preload(16'h0601, 8'h88);
	akiko_dma_req   <= 1'b1;
	akiko_dma_we    <= 1'b0;
	akiko_dma_baddr <= 24'h000600;
	begin : t7
		automatic int t7_mismatches = 0;
		// Set up a minimig read slot in the next cycle.
		@(posedge clk);
		chip_in_addr <= 25'h0001234;
		chip_in_u    <= 1'b0;        // upper enabled
		chip_in_l    <= 1'b0;        // lower enabled
		chip_in_rw   <= 1'b1;        // read
		chip_in_dma  <= 1'b0;        // active
		chip_in_wr   <= 16'h0000;
		// Sample over the next 4 cycles -- must be forwarded through.
		repeat (4) begin
			@(posedge clk);
			if (chip_out_dma !== chip_in_dma) t7_mismatches++;
			if (chip_out_addr !== chip_in_addr) t7_mismatches++;
			if (chip_out_rw !== chip_in_rw) t7_mismatches++;
		end
		// Idle minimig.
		chip_in_dma <= 1'b1;
		chip_in_rw  <= 1'b1;
		chip_in_l   <= 1'b1;
		chip_in_u   <= 1'b1;
		if (t7_mismatches > 0) begin
			$display("FAIL test7: %0d forwarding mismatches", t7_mismatches);
			errs++;
		end else begin
			checks++;
			$display("PASS test7: minimig forwarding fidelity (4 cycles)");
		end
		// Drain akiko's pending request.
		begin
			automatic int t7_timeout = 400;
			while (!akiko_dma_ack && t7_timeout > 0) begin
				@(posedge clk); t7_timeout--;
			end
			if (t7_timeout == 0) begin
				$display("FAIL test7: akiko ack timeout after minimig traffic");
				errs++;
			end
		end
		akiko_dma_req <= 1'b0;
	end

	// ---- Test 8: race -- minimig and akiko request the same c_7m slot.
	//      Minimig MUST win; akiko's request waits for the next idle slot. ----
	preload(16'h0700, 8'h99);
	akiko_dma_req   <= 1'b1;
	akiko_dma_we    <= 1'b0;
	akiko_dma_baddr <= 24'h000700;
	begin : t8
		automatic int t8_drove_minimig_addr = 0;
		// Wait for the next c_7m rising edge to align.
		@(negedge c_7m);
		@(posedge c_7m);
		// At this c_7m edge, drive minimig's request simultaneously with
		// akiko_dma_req still high. Check on the next sysclk edges that
		// chip_out_addr == minimig's address (not akiko's word addr).
		chip_in_addr <= 25'h0009999;
		chip_in_u    <= 1'b0;
		chip_in_l    <= 1'b0;
		chip_in_rw   <= 1'b0;        // write -- forces minimig active
		chip_in_dma  <= 1'b1;
		chip_in_wr   <= 16'hAAAA;
		repeat (3) begin
			@(posedge clk);
			if (chip_out_addr === 25'h0009999) t8_drove_minimig_addr++;
		end
		// Idle minimig.
		chip_in_rw <= 1'b1;
		chip_in_dma <= 1'b1;
		if (t8_drove_minimig_addr == 0) begin
			$display("FAIL test8: arbiter preempted minimig (akiko address won race)");
			errs++;
		end else begin
			checks++;
			$display("PASS test8: minimig won race (%0d cycles confirmed)",
			         t8_drove_minimig_addr);
		end
		// Drain akiko.
		begin
			automatic int t8_timeout = 400;
			while (!akiko_dma_ack && t8_timeout > 0) begin
				@(posedge clk); t8_timeout--;
			end
		end
		akiko_dma_req <= 1'b0;
	end

	// ---- Done ----
	repeat (10) @(posedge clk);
	$display("tb_akiko_chipram_master: %0d checks, %0d errs", checks, errs);
	$finish;
end

endmodule
