// SPDX-License-Identifier: GPL-3.0-or-later
//
// Akiko M4 PBX sector DMA bench.
//
// DUT  : akiko #(.NATIVE_CD32(1)) standalone (no bridge — sector channel
//        is driven by the bench directly through hps_sec_*).
// BFM  : 256 KiB chip-RAM behind dma_req/ack with the M2 !dma_ack re-arm
//        guard (so PBX writes don't double-count). Same BFM also serves the
//        RX engine for the RX-preempts-PBX test.
//
// Sim gotchas applied (from feedback_sim_gotchas memory):
//   1. wait_* tasks lead with @(posedge clk).
//   2. BFM re-arms with `dma_req && !dma_ack`.
//   3. n/a: this bench doesn't use the bridge, so no UIO reads.
//
// Per-sector cycles: one PBX slot = 2352 + 146 = 2498 chip-RAM byte writes,
// each taking 2 cycles via the BFM (req-ack, then re-arm). So ~5000 cycles
// per slot. The watchdog is sized to allow 4 slots + setup + RX preemption.

`timescale 1ns / 1ps

module tb_akiko_pbx_dma;

initial begin
	#5000000 $fatal(1, "tb_akiko_pbx_dma: watchdog timeout");
end

logic clk = 0;
initial forever #5 clk = ~clk;

logic        reset = 1;
logic        cs    = 0;
logic        rd    = 0;
logic        wr    = 0;
logic        lds   = 0;
logic        uds   = 0;
logic [5:1]  addr  = 0;
logic [15:0] din   = 0;
wire  [15:0] dout;
wire         irq;

// Master DMA port (akiko -> BFM)
wire        dma_req;
wire        dma_we;
wire [23:0] dma_baddr;
wire  [7:0] dma_wbyte;
logic [7:0] dma_rbyte;
logic       dma_ack;

// HPS sector channel (bench drives directly, mimicking the bridge)
wire        hps_sec_req;
wire  [7:0] hps_sec_status;
logic       hps_sec_push = 0;
logic [7:0] hps_sec_byte = 0;
logic       hps_sec_done = 0;

akiko #(.NATIVE_CD32(1)) u_dut (
	.clk(clk), .reset(reset),
	.cs(cs), .rd(rd), .wr(wr),
	.lds(lds), .uds(uds),
	.addr(addr), .din(din), .dout(dout),
	.akiko_irq(irq),
	.dma_req(dma_req), .dma_we(dma_we),
	.dma_baddr(dma_baddr), .dma_wbyte(dma_wbyte),
	.dma_rbyte(dma_rbyte), .dma_ack(dma_ack),
	// M3 cmd bridge: not exercised here — tied off.
	.hps_cmd_pending(), .hps_cmd_byte(),
	.hps_cmd_pop(1'b0), .hps_cmd_done(1'b0),
	.hps_result_push(1'b0), .hps_result_byte(8'h00), .hps_result_done(1'b0),
	// M4 sector channel: bench drives.
	.hps_sec_req(hps_sec_req),
	.hps_sec_status(hps_sec_status),
	.hps_sec_push(hps_sec_push),
	.hps_sec_byte(hps_sec_byte),
	.hps_sec_done(hps_sec_done)
);

// -----------------------------------------------------------------------
// Constants (mirrors of akiko.v / WinUAE)
// -----------------------------------------------------------------------
localparam [31:0] CDINT_PBX       = 32'h04000000; // bit 26
localparam [31:0] CDINT_RXDMADONE = 32'h10000000;
localparam [31:0] CDINT_TXDMADONE = 32'h08000000;

localparam [31:0] CFG_TXD    = 32'h40000000; // bit 30
localparam [31:0] CFG_RXD    = 32'h20000000; // bit 29
localparam [31:0] CFG_PBX    = 32'h08000000; // bit 27
localparam [31:0] CFG_ENABLE = 32'h04000000; // bit 26

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

task automatic check_bit(string name, logic expected, logic actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected %0b got %0b (t=%0t)", name, expected, actual, $time);
		errs++;
	end
endtask

// -----------------------------------------------------------------------
// CPU bus drivers
// -----------------------------------------------------------------------
task automatic bus_write_word(input [5:1] a, input [15:0] data);
	@(posedge clk);
	cs <= 1; wr <= 1; rd <= 0; addr <= a; din <= data; lds <= 1; uds <= 1;
	@(posedge clk);
	cs <= 0; wr <= 0; addr <= 0; din <= 0; lds <= 0; uds <= 0;
endtask

task automatic bus_write_byte_lo(input [5:1] a, input [7:0] data);
	@(posedge clk);
	cs <= 1; wr <= 1; rd <= 0; addr <= a; din <= {8'h0, data}; lds <= 1; uds <= 0;
	@(posedge clk);
	cs <= 0; wr <= 0; addr <= 0; din <= 0; lds <= 0; uds <= 0;
endtask

task automatic bus_write_long(input [5:1] a_hi, input [31:0] data);
	bus_write_word(a_hi,        data[31:16]);
	bus_write_word(a_hi + 5'd1, data[15:0]);
endtask

task automatic set_addressdata(input [23:0] base);
	bus_write_long(5'b01000, {8'h00, base});
endtask

task automatic set_misc_base(input [23:0] base);
	bus_write_long(5'b01010, {8'h00, base});
endtask

task automatic set_config(input [31:0] flags);
	bus_write_long(5'b10010, flags);
endtask

task automatic write_pbx(input [15:0] mask);
	bus_write_word(5'b10000, mask);
endtask

task automatic write_rxcmp(input [7:0] v);
	bus_write_byte_lo(5'b01111, v);
endtask

// -----------------------------------------------------------------------
// Chip-RAM BFM. 256 KiB, indexed by dma_baddr[17:0]. Addressdata is set
// to 0x010000 in tests so slots 0..15 land at 0x10000..0x1FFFF (slot N at
// 0x10000 + N*4096). Initial fill = 0x55 sentinel so we can spot un-written
// regions inside the 4 KiB slot.
// -----------------------------------------------------------------------
logic [7:0] mem [262144];
logic       bfm_in_xfer = 1'b0;

initial begin
	dma_ack   = 0;
	dma_rbyte = 8'h00;
	for (int i = 0; i < 262144; i++) mem[i] = 8'h55;
end

always @(posedge clk) begin
	dma_ack <= 0;
	if (bfm_in_xfer) begin
		if (dma_we) mem[dma_baddr[17:0]] <= dma_wbyte;
		else        dma_rbyte <= mem[dma_baddr[17:0]];
		dma_ack     <= 1'b1;
		bfm_in_xfer <= 1'b0;
	end else if (dma_req && !dma_ack) begin
		bfm_in_xfer <= 1'b1;
	end
end

// -----------------------------------------------------------------------
// Sector push helper. Drives hps_sec_push / hps_sec_byte for 2352 cycles,
// then pulses hps_sec_done one cycle after the last push (so push and done
// don't overlap, matching what the real bridge produces on deselect).
//
// `seed` controls the per-byte pattern (byte i = seed + i). Bytes 0..3 of
// the in-buffer data are present but ignored by the engine — bytes 0..2 are
// overridden to zero and byte 3 is overridden to sector_counter & 31.
// -----------------------------------------------------------------------
task automatic push_sector(input [7:0] seed);
	@(posedge clk);
	for (int i = 0; i < 2352; i++) begin
		hps_sec_byte <= seed + i[7:0];
		hps_sec_push <= 1'b1;
		@(posedge clk);
	end
	hps_sec_push <= 1'b0;
	hps_sec_byte <= 8'h00;
	@(posedge clk);
	hps_sec_done <= 1'b1;
	@(posedge clk);
	hps_sec_done <= 1'b0;
	@(posedge clk);
endtask

// -----------------------------------------------------------------------
// Wait helpers — leading @(posedge clk) per gotcha #1.
//
// wait_sector_inc is the reliable per-slot wait: it captures the current
// sector_counter, then waits for it to advance. The counter only bumps
// inside PBX_FIN, after a complete slot write. (Earlier draft used
// `wait_pbx_idle` checking pbx_state, but that's racy: the wait task can
// land its first @(posedge) before the engine has transitioned IDLE→DATA,
// and exit immediately because the loop condition is false. Sector counter
// monotonically advances and is safe to compare against.)
// -----------------------------------------------------------------------
task automatic wait_sector_inc(input [7:0] before_counter,
                                input int max_cycles,
                                output int cycles);
	int n;
begin
	n = 0;
	@(posedge clk);
	while (u_dut.g_cd.cdrom_sector_counter == before_counter && n < max_cycles) begin
		@(posedge clk);
		n = n + 1;
	end
	cycles = n;
end
endtask

task automatic wait_pbx_clear(input int max_cycles, output int cycles);
	int n;
begin
	n = 0;
	@(posedge clk);
	while (u_dut.g_cd.cdrom_pbx != 16'h0 && n < max_cycles) begin
		@(posedge clk);
		n = n + 1;
	end
	cycles = n;
end
endtask

task automatic wait_rx_done(input int max_cycles, output int cycles);
	int n;
begin
	n = 0;
	@(posedge clk);
	while ((u_dut.g_cd.rx_busy || u_dut.g_cd.cdrom_receive_length != 0) && n < max_cycles) begin
		@(posedge clk);
		n = n + 1;
	end
	cycles = n;
end
endtask

task automatic do_reset;
begin
	reset <= 1;
	@(posedge clk); @(posedge clk); @(posedge clk);
	reset <= 0;
	@(posedge clk);
end
endtask

// -----------------------------------------------------------------------
// Slot verification helper. Checks the WinUAE 4 KiB slot layout:
//   bytes 0..2 = 0
//   byte 3     = counter & 31
//   bytes 4..2351 = seed + idx
//   bytes 0xc00..0xc91 = 0
//   bytes 0x931..0xbff and 0xc92..0xfff = untouched (still sentinel 0x55)
// -----------------------------------------------------------------------
task automatic check_slot(string tag,
                          input int slot_base_addr,   // chip-RAM base of slot
                          input [7:0] seed,
                          input [7:0] counter);
	logic [7:0] expected;
	int errs0 = errs;
	begin
		check8({tag, ".byte0"}, 8'h00, mem[slot_base_addr + 0]);
		check8({tag, ".byte1"}, 8'h00, mem[slot_base_addr + 1]);
		check8({tag, ".byte2"}, 8'h00, mem[slot_base_addr + 2]);
		check8({tag, ".byte3"}, counter & 8'h1f, mem[slot_base_addr + 3]);
		// Spot-check pattern bytes (full 2348-byte sweep is needlessly slow).
		// Indices 4, 100, 1000, 2000, 2351 — pre-computed as (idx mod 256):
		//   1000 mod 256 = 232 = 0xE8
		//   2000 mod 256 = 208 = 0xD0
		//   2351 mod 256 =  47 = 0x2F
		check8({tag, ".byte4"},     seed + 8'd4,   mem[slot_base_addr + 4]);
		check8({tag, ".byte100"},   seed + 8'd100, mem[slot_base_addr + 100]);
		check8({tag, ".byte1000"},  seed + 8'd232, mem[slot_base_addr + 1000]);
		check8({tag, ".byte2000"},  seed + 8'd208, mem[slot_base_addr + 2000]);
		check8({tag, ".byte2351"},  seed + 8'd47,  mem[slot_base_addr + 2351]);
		// Zero region at +0xc00..+0xc91
		check8({tag, ".zero0"},     8'h00, mem[slot_base_addr + 'h0c00]);
		check8({tag, ".zero1"},     8'h00, mem[slot_base_addr + 'h0c45]);
		check8({tag, ".zero2"},     8'h00, mem[slot_base_addr + 'h0c91]);
		// Untouched sentinel regions — verify the engine didn't over-write
		check8({tag, ".gap_lo"},    8'h55, mem[slot_base_addr + 'h0931]);
		check8({tag, ".gap_hi"},    8'h55, mem[slot_base_addr + 'h0c92]);
		check8({tag, ".gap_end"},   8'h55, mem[slot_base_addr + 'h0fff]);
	end
endtask

// -----------------------------------------------------------------------
// Test sequence
// -----------------------------------------------------------------------
initial begin
	int cyc;

	$display("tb_akiko_pbx_dma starting");
	@(posedge clk);
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000); // INTENA upper byte all 1s

	// =====================================================================
	// Test A: single sector to slot 0.
	// addressdata = 0x010000, push pattern (seed=0x10), pbx=0x0001, run.
	// Expect mem[0x10000..0x10930] populated, mem[0x10c00..0x10c91] zero,
	// counter==1, pbx==0, CDINT_PBX set.
	// =====================================================================
	$display("--- Test A: single sector to slot 0 ---");
	set_addressdata(24'h010000);
	set_config(CFG_ENABLE | CFG_PBX);
	check8("A.counter_init", 8'd0, u_dut.g_cd.cdrom_sector_counter);
	check_bit("A.sec_req_no_pbx", 1'b0, hps_sec_req);
	push_sector(8'h10);
	check_bit("A.ready_after_push", 1'b1, u_dut.g_cd.sector_ready);
	write_pbx(16'h0001);
	check_bit("A.sec_req_low",  1'b0, hps_sec_req); // ready=1 means no req
	wait_pbx_clear(20000, cyc);
	$display("    A: pbx clear in %0d cycles", cyc);
	check_slot("A", 'h10000, 8'h10, 8'd0);
	check8 ("A.pbx_clear", 8'h00, u_dut.g_cd.cdrom_pbx[7:0]);
	check_bit("A.intpbx", 1'b1, u_dut.g_cd.cdrom_intreq[26]);
	check8 ("A.counter1", 8'd1, u_dut.g_cd.cdrom_sector_counter);
	check_bit("A.ready_clear", 1'b0, u_dut.g_cd.sector_ready);

	// =====================================================================
	// Test B: highest-slot-first selection.
	// pbx=0x8001 (slots 0 and 15). Push a sector, wait for slot 15 to drain;
	// slot 0 should still be set; push another sector; it goes to slot 0.
	// Verify byte 3 = 0 in slot 15 (counter==0 if reset), byte 3 = 1 in slot 0.
	// =====================================================================
	$display("--- Test B: highest-slot-first ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	for (int i = 'h10000; i < 'h20000; i++) mem[i] = 8'h55; // clear slot region
	set_addressdata(24'h010000);
	set_config(CFG_ENABLE | CFG_PBX);
	push_sector(8'h20);
	write_pbx(16'h8001);
	// First sector should land in slot 15 (highest set).
	wait_sector_inc(8'd0, 20000, cyc);
	$display("    B: slot15 done in %0d cycles, pbx=%04h", cyc, u_dut.g_cd.cdrom_pbx);
	check8 ("B.pbx_after_slot15", 8'h01, u_dut.g_cd.cdrom_pbx[7:0]);
	check8 ("B.pbx_hi_clear",     8'h00, u_dut.g_cd.cdrom_pbx[15:8]);
	check_slot("B.slot15", 'h10000 + 15*4096, 8'h20, 8'd0);
	check8 ("B.counter_after_one", 8'd1, u_dut.g_cd.cdrom_sector_counter);
	check_bit("B.sec_req_for_slot0", 1'b1, hps_sec_req);
	// Second sector for slot 0
	push_sector(8'h30);
	wait_pbx_clear(20000, cyc);
	$display("    B: slot0 done in %0d cycles", cyc);
	check_slot("B.slot0", 'h10000, 8'h30, 8'd1);
	check8 ("B.pbx_all_clear", 8'h00, u_dut.g_cd.cdrom_pbx[7:0]);
	check8 ("B.counter2", 8'd2, u_dut.g_cd.cdrom_sector_counter);

	// =====================================================================
	// Test C: 4 slots (bits 1, 4, 8, 14 → 0x4112). Verify ordering 14, 8, 4, 1
	// and counters 0, 1, 2, 3 → byte 3 in each slot equals 0/1/2/3.
	// =====================================================================
	$display("--- Test C: 4 slots, highest-first ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	for (int i = 'h10000; i < 'h20000; i++) mem[i] = 8'h55;
	set_addressdata(24'h010000);
	set_config(CFG_ENABLE | CFG_PBX);
	push_sector(8'h40);
	write_pbx(16'h4112); // bits 1, 4, 8, 14
	// Drain one slot at a time, pushing the next sector after each.
	wait_sector_inc(8'd0, 20000, cyc);
	$display("    C: slot14 done in %0d cycles, pbx=%04h", cyc, u_dut.g_cd.cdrom_pbx);
	check_slot("C.slot14", 'h10000 + 14*4096, 8'h40, 8'd0);
	check8("C.pbx_after14_lo", 8'h12, u_dut.g_cd.cdrom_pbx[7:0]);
	check8("C.pbx_after14_hi", 8'h01, u_dut.g_cd.cdrom_pbx[15:8]);
	push_sector(8'h50);
	wait_sector_inc(8'd1, 20000, cyc);
	$display("    C: slot8  done in %0d cycles, pbx=%04h", cyc, u_dut.g_cd.cdrom_pbx);
	check_slot("C.slot8",  'h10000 + 8*4096,  8'h50, 8'd1);
	check8("C.pbx_after8",  8'h12, u_dut.g_cd.cdrom_pbx[7:0]);
	push_sector(8'h60);
	wait_sector_inc(8'd2, 20000, cyc);
	$display("    C: slot4  done in %0d cycles, pbx=%04h", cyc, u_dut.g_cd.cdrom_pbx);
	check_slot("C.slot4",  'h10000 + 4*4096,  8'h60, 8'd2);
	check8("C.pbx_after4",  8'h02, u_dut.g_cd.cdrom_pbx[7:0]);
	push_sector(8'h70);
	wait_pbx_clear(20000, cyc);
	$display("    C: slot1  done in %0d cycles", cyc);
	check_slot("C.slot1",  'h10000 + 1*4096,  8'h70, 8'd3);
	check8("C.counter4", 8'd4, u_dut.g_cd.cdrom_sector_counter);

	// =====================================================================
	// Test D: CDFLAG_ENABLE rising-edge resets sector_counter.
	// After Test C, counter should be 4. Drop ENABLE (CONFIG = PBX only),
	// re-set ENABLE, verify counter = 0.
	// =====================================================================
	$display("--- Test D: ENABLE rising resets counter ---");
	check8("D.counter_pre", 8'd4, u_dut.g_cd.cdrom_sector_counter);
	set_config(CFG_PBX); // ENABLE -> 0
	@(posedge clk); @(posedge clk);
	check8("D.counter_after_disable", 8'd4, u_dut.g_cd.cdrom_sector_counter);
	set_config(CFG_ENABLE | CFG_PBX); // ENABLE 0->1 should reset counter
	@(posedge clk); @(posedge clk);
	check8("D.counter_after_enable", 8'd0, u_dut.g_cd.cdrom_sector_counter);

	// =====================================================================
	// Test E: sec_req protocol.
	// After D, counter=0, no pbx pending → sec_req = 0.
	// Set pbx=0x0008 → sec_req should rise (need a sector).
	// Push sector → sec_req drops (sector_ready=1).
	// Wait for engine done → sec_req drops (no more pbx pending).
	// =====================================================================
	$display("--- Test E: sec_req protocol ---");
	check_bit("E.sec_req_idle", 1'b0, hps_sec_req);
	write_pbx(16'h0008); // slot 3
	@(posedge clk); @(posedge clk);
	check_bit("E.sec_req_pending", 1'b1, hps_sec_req);
	push_sector(8'h80);
	check_bit("E.sec_req_after_push", 1'b0, hps_sec_req);
	wait_pbx_clear(20000, cyc);
	$display("    E: slot3 done in %0d cycles", cyc);
	check_slot("E.slot3", 'h10000 + 3*4096, 8'h80, 8'd0);
	check_bit("E.sec_req_after_done", 1'b0, hps_sec_req);
	check8("E.counter5", 8'd1, u_dut.g_cd.cdrom_sector_counter);

	// =====================================================================
	// Test F: RX preempts PBX. Set up an RX response, kick a PBX cycle,
	// then arm RX mid-burst. Verify RX bytes win the bus (mem at cdrx_address
	// gets the response), PBX completes after RX finishes.
	//
	// RX uses misc_base region (cdrx_address = misc_base | 0x000), which is
	// disjoint from the PBX slot region (0x10000+). misc_base = 0x002000.
	// =====================================================================
	$display("--- Test F: RX preempts PBX ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	for (int i = 'h10000; i < 'h20000; i++) mem[i] = 8'h55;
	for (int i = 'h02000; i < 'h02010; i++) mem[i] = 8'h00;
	set_addressdata(24'h010000);
	set_misc_base(24'h002000);
	set_config(CFG_ENABLE | CFG_PBX | CFG_RXD);
	// Pre-load a result for RX
	u_dut.g_cd.cdrom_result_buffer[0] = 8'hCA;
	u_dut.g_cd.cdrom_result_buffer[1] = 8'hFE;
	u_dut.g_cd.cdrom_result_buffer[2] = 8'hBA;
	u_dut.g_cd.cdrom_result_buffer[3] = 8'hBE;
	u_dut.g_cd.cdcomrxinx           = 8'd0;
	push_sector(8'hA0);
	write_pbx(16'h0001);
	// Let PBX run for a few hundred cycles so we know it's mid-burst
	repeat (300) @(posedge clk);
	check_bit("F.pbx_busy_pre", 1'b1, u_dut.g_cd.pbx_busy);
	$display("    F.pre: rb[0]=%02h rb[1]=%02h reclen=%0d rxinx=%0d rxcmp=%0d flags=%08h",
	         u_dut.g_cd.cdrom_result_buffer[0], u_dut.g_cd.cdrom_result_buffer[1],
	         u_dut.g_cd.cdrom_receive_length, u_dut.g_cd.cdcomrxinx,
	         u_dut.g_cd.cdcomrxcmp, u_dut.g_cd.cdrom_flags);
	// Now arm RX (will preempt)
	u_dut.g_cd.cdrom_receive_length = 6'd4;
	write_rxcmp(8'd4);
	$display("    F.post-rxcmp: reclen=%0d rxcmp=%0d delay=%0d",
	         u_dut.g_cd.cdrom_receive_length, u_dut.g_cd.cdcomrxcmp,
	         u_dut.g_cd.rx_dma_delay);
	wait_rx_done(2000, cyc);
	$display("    F: RX done in %0d cycles  rxinx=%0d reclen=%0d mem2000=%02h",
	         cyc, u_dut.g_cd.cdcomrxinx, u_dut.g_cd.cdrom_receive_length, mem['h02000]);
	check8("F.mem_rx0", 8'hCA, mem['h02000]);
	check8("F.mem_rx1", 8'hFE, mem['h02001]);
	check8("F.mem_rx2", 8'hBA, mem['h02002]);
	check8("F.mem_rx3", 8'hBE, mem['h02003]);
	// PBX should resume and complete.
	wait_pbx_clear(20000, cyc);
	$display("    F: pbx clear in %0d cycles", cyc);
	check_slot("F.slot0", 'h10000, 8'hA0, 8'd0);

	// =====================================================================
	// Test G: TX gated off during PBX.
	// Set CONFIG with TXD + ENABLE + PBX. Set txcmp != txinx so TX would
	// otherwise fetch. Verify cdrom_command_length stays 0 (no TX progress).
	// =====================================================================
	$display("--- Test G: TX gated off during PBX ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	for (int i = 'h10000; i < 'h20000; i++) mem[i] = 8'h55;
	set_addressdata(24'h010000);
	set_misc_base(24'h003000);
	// Put a TX byte in chip RAM just in case TX fires
	mem['h03200] = 8'h99;
	mem['h03201] = 8'h66; // checksum
	set_config(CFG_ENABLE | CFG_PBX | CFG_TXD);
	bus_write_byte_lo(5'b01110, 8'd2); // txcmp = 2 (would fetch 2 bytes if TX allowed)
	push_sector(8'hB0);
	write_pbx(16'h0001);
	wait_pbx_clear(20000, cyc);
	$display("    G: pbx clear in %0d cycles", cyc);
	check8("G.cmdlen_zero", 8'd0, {2'h0, u_dut.g_cd.cdrom_command_length});
	check8("G.txinx_zero",  8'd0, u_dut.g_cd.cdcomtxinx);
	check_slot("G.slot0", 'h10000, 8'hB0, 8'd0);

	// =====================================================================
	$display("------------------------------------");
	$display("checks=%0d errors=%0d", checks, errs);
	if (errs == 0) $display("tb_akiko_pbx_dma PASS");
	else           $display("tb_akiko_pbx_dma FAIL");
	$finish(errs == 0 ? 0 : 1);
end

endmodule
