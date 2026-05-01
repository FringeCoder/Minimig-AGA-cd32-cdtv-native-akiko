// SPDX-License-Identifier: GPL-3.0-or-later
//
// Akiko M2 TX/RX command DMA bench.
//
// DUT  : akiko #(.NATIVE_CD32(1)) (live edit-in-place RTL)
// BFM  : 64 KiB sparse byte RAM behind the dma_req/ack handshake. 0-latency
//        ack by default; one test injects a multi-cycle ack to stress the
//        held-request path. The BFM exposes write_byte/read_byte so the bench
//        can preload/check the chip-RAM image.
//
// Hierarchical backdoor writes (`u_dut.g_cd.cdrom_result_buffer[i] = ...`,
// `cdrom_receive_length = ...`) load synthetic responses for RX tests — M2
// has no command parser, so the bench supplies the result buffer the way M3
// will. `+acc` in run.do keeps the hierarchy visible to ModelSim ASE.
//
// Differential against akiko_legacy_ref is not useful here because legacy
// has no DMA semantics. The bench is a pure spec check against the WinUAE
// behavior captured in research/docs/akiko-mvp-implementation-strategy.md.

`timescale 1ns / 1ps

module tb_akiko_txrx_dma;

// -----------------------------------------------------------------------
// Watchdog
// -----------------------------------------------------------------------
initial begin
	#500000 $fatal(1, "tb_akiko_txrx_dma: watchdog timeout");
end

// -----------------------------------------------------------------------
// Clock + bus stimulus
// -----------------------------------------------------------------------
logic clk = 0;
initial forever #5 clk = ~clk;  // 100 MHz nominal

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

wire        dma_req;
wire        dma_we;
wire [23:0] dma_baddr;
wire  [7:0] dma_wbyte;
logic [7:0] dma_rbyte;
logic       dma_ack;

akiko #(.NATIVE_CD32(1)) u_dut (
	.clk(clk), .reset(reset),
	.cs(cs), .rd(rd), .wr(wr),
	.lds(lds), .uds(uds),
	.addr(addr), .din(din), .dout(dout),
	.akiko_irq(irq),
	.dma_req(dma_req), .dma_we(dma_we),
	.dma_baddr(dma_baddr), .dma_wbyte(dma_wbyte),
	.dma_rbyte(dma_rbyte), .dma_ack(dma_ack)
);

// -----------------------------------------------------------------------
// Constants (mirrors of akiko.v / WinUAE)
// -----------------------------------------------------------------------
localparam [31:0] CDINT_DRIVEXMIT = 32'h40000000;
localparam [31:0] CDINT_DRIVERECV = 32'h20000000;
localparam [31:0] CDINT_RXDMADONE = 32'h10000000;
localparam [31:0] CDINT_TXDMADONE = 32'h08000000;

localparam [31:0] CFG_TXD = 32'h40000000; // bit 30
localparam [31:0] CFG_RXD = 32'h20000000; // bit 29
localparam [31:0] CFG_ENA = 32'h04000000; // bit 26

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

task automatic check32(string name, logic [31:0] expected, logic [31:0] actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected 0x%08h got 0x%08h (t=%0t)", name, expected, actual, $time);
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
// Bus drivers (single-cycle byte/word writes & reads)
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

// -----------------------------------------------------------------------
// 64 KiB byte BFM (sparse — only the misc-base sub-region is touched).
// 0-latency by default. `bfm_extra_delay` adds N idle cycles between
// dma_req asserting and dma_ack pulsing; lets us check the engine holds
// its request stable.
// -----------------------------------------------------------------------
logic [7:0] mem [65536];

int    bfm_extra_delay = 0;
int    bfm_delay_cnt   = 0;
logic  bfm_in_xfer     = 1'b0;

initial begin
	dma_ack   = 0;
	dma_rbyte = 8'h00;
	for (int i = 0; i < 65536; i++) mem[i] = 8'h00;
end

// BFM RTL: when dma_req goes high, count extra_delay then pulse ack for one
// cycle, presenting the byte (read) or absorbing the byte (write).
//
// The `&& !dma_ack` guard on re-arming is essential: the engine drives
// dma_req combinationally off (tx_busy | rx_busy), and its busy NBA only
// drops one cycle AFTER it samples ack. So in the cycle the engine is
// consuming our ack pulse, dma_req is still high — we must NOT re-arm
// then, or the engine's next-cycle ack-sample will miscount one byte.
always @(posedge clk) begin
	dma_ack <= 0;
	if (bfm_in_xfer) begin
		if (bfm_delay_cnt != 0) begin
			bfm_delay_cnt <= bfm_delay_cnt - 1;
		end else begin
			if (dma_we) begin
				mem[dma_baddr[15:0]] <= dma_wbyte;
			end else begin
				dma_rbyte <= mem[dma_baddr[15:0]];
			end
			dma_ack     <= 1'b1;
			bfm_in_xfer <= 1'b0;
		end
	end else if (dma_req && !dma_ack) begin
		bfm_in_xfer  <= 1'b1;
		bfm_delay_cnt <= bfm_extra_delay;
	end
end

// -----------------------------------------------------------------------
// Helpers — enable native mode, set base, set CDFLAGS
// -----------------------------------------------------------------------
task automatic do_reset;
begin
	reset <= 1;
	@(posedge clk); @(posedge clk); @(posedge clk);
	reset <= 0;
	@(posedge clk);
end
endtask

task automatic set_misc_base(input [23:0] base);
begin
	// $14-$17 = misc DMA base
	bus_write_long(5'b01010, {8'h00, base});
end
endtask

task automatic set_config(input [31:0] flags);
begin
	bus_write_long(5'b10010, flags);
end
endtask

task automatic write_txcmp(input [7:0] v);
	bus_write_byte_lo(5'b01110, v);
endtask

task automatic write_rxcmp(input [7:0] v);
	bus_write_byte_lo(5'b01111, v);
endtask

// Wait until tx_busy/rx_busy both deassert AND the indices reach the
// compare values OR a watchdog count fires. Returns elapsed cycles.
//
// The leading @(posedge clk) is mandatory: when called immediately after a
// register write, the NBA that updates cdcomtxcmp has not committed yet —
// the active-region while-check would still see the old (often equal)
// value and exit before the engine has a chance to run.
task automatic wait_tx_done(input int max_cycles, output int cycles);
	int n;
begin
	n = 0;
	@(posedge clk);
	while ((u_dut.g_cd.tx_busy || u_dut.g_cd.cdcomtxinx != u_dut.g_cd.cdcomtxcmp) && n < max_cycles) begin
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

// -----------------------------------------------------------------------
// Test sequence
// -----------------------------------------------------------------------
initial begin
	int cyc;

	$display("tb_akiko_txrx_dma starting");
	@(posedge clk);
	do_reset();

	// Per the docs, set INTENA so our IRQ checks reflect intena & intreq.
	// All upper 8 bits enabled.
	bus_write_long(5'b00100, 32'hFF000000);

	// =====================================================================
	// Test A: simple 4-byte TX command at base 0x010000
	// chip RAM at 0x010200..0x010203 holds the command bytes
	// =====================================================================
	$display("--- Test A: 4-byte TX ---");
	set_misc_base(24'h010000);
	// First byte's low nibble selects expected_total_len in the M3 framer.
	// Use opcode 0x0B (unknown) so the framer waits for buffer-full and
	// doesn't gate TX after byte 3 — keeps this M2 test meaningful.
	mem[16'h0200] = 8'hAB;
	mem[16'h0201] = 8'hB2;
	mem[16'h0202] = 8'hC3;
	mem[16'h0203] = 8'hD4;
	// Force command_length back to 0 in case a prior test grew it.
	u_dut.g_cd.cdrom_command_length = 6'd0;
	u_dut.g_cd.cdcomtxinx           = 8'd0;
	// Enable TXD, then write txcmp = 4. Engine waits 3 ticks then DMAs.
	set_config(CFG_TXD);
	write_txcmp(8'd4);
	wait_tx_done(200, cyc);
	check8 ("A.cmd[0]",  8'hAB, u_dut.g_cd.cdrom_command_buffer[0]);
	check8 ("A.cmd[1]",  8'hB2, u_dut.g_cd.cdrom_command_buffer[1]);
	check8 ("A.cmd[2]",  8'hC3, u_dut.g_cd.cdrom_command_buffer[2]);
	check8 ("A.cmd[3]",  8'hD4, u_dut.g_cd.cdrom_command_buffer[3]);
	check8 ("A.txinx",   8'd4,  u_dut.g_cd.cdcomtxinx);
	check8 ("A.cmdlen",  8'd4,  {2'h0, u_dut.g_cd.cdrom_command_length});
	check_bit("A.txdone_intreq", 1'b1, u_dut.g_cd.cdrom_intreq[27]);
	check_bit("A.irq",           1'b1, irq);

	// =====================================================================
	// Test B: simple 4-byte RX response at base 0x020000
	// Bench injects result_buffer + receive_length, sets RXD, sets rxcmp=4.
	// =====================================================================
	$display("--- Test B: 4-byte RX ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	set_misc_base(24'h020000);
	for (int i = 0; i < 32; i++) mem[16'h0000 + i] = 8'h00;
	u_dut.g_cd.cdrom_result_buffer[0] = 8'h11;
	u_dut.g_cd.cdrom_result_buffer[1] = 8'h22;
	u_dut.g_cd.cdrom_result_buffer[2] = 8'h33;
	u_dut.g_cd.cdrom_result_buffer[3] = 8'h44;
	u_dut.g_cd.cdrom_receive_length   = 6'd4;
	u_dut.g_cd.cdrom_receive_offset   = 6'd0;
	// Pre-set DRIVERECV to confirm it gets cleared when the response drains.
	u_dut.g_cd.cdrom_intreq           = CDINT_DRIVERECV;
	set_config(CFG_RXD);
	write_rxcmp(8'd4);
	wait_rx_done(200, cyc);
	check8 ("B.mem[0]", 8'h11, mem[16'h0000]);
	check8 ("B.mem[1]", 8'h22, mem[16'h0001]);
	check8 ("B.mem[2]", 8'h33, mem[16'h0002]);
	check8 ("B.mem[3]", 8'h44, mem[16'h0003]);
	check8 ("B.rxinx",  8'd4,  u_dut.g_cd.cdcomrxinx);
	check_bit("B.rxdone_intreq",   1'b1, u_dut.g_cd.cdrom_intreq[28]);
	check_bit("B.drivexmit_intreq",1'b1, u_dut.g_cd.cdrom_intreq[30]);
	check_bit("B.driverecv_clear", 1'b0, u_dut.g_cd.cdrom_intreq[29]);
	check_bit("B.irq",             1'b1, irq);

	// =====================================================================
	// Test C: Partial RX. receive_length=8 but rxcmp=4 first, then rxcmp=8.
	// Stage 1: drains 4 bytes, sets RXDMADONE, stops (receive_length stays 8).
	// Stage 2: bench bumps rxcmp to 8 → drains remaining 4 → DRIVEXMIT.
	// =====================================================================
	$display("--- Test C: partial RX ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	set_misc_base(24'h030000);
	for (int i = 0; i < 32; i++) mem[16'h0000 + i] = 8'h00;
	for (int i = 0; i < 8;  i++) u_dut.g_cd.cdrom_result_buffer[i] = 8'hA0 + i[7:0];
	u_dut.g_cd.cdrom_receive_length = 6'd8;
	u_dut.g_cd.cdrom_receive_offset = 6'd0;
	set_config(CFG_RXD);
	write_rxcmp(8'd4);
	repeat (40) @(posedge clk);
	check8 ("C1.mem[3]", 8'hA3, mem[16'h0003]);
	check8 ("C1.mem[4]", 8'h00, mem[16'h0004]); // not yet
	check8 ("C1.rxinx",  8'd4,  u_dut.g_cd.cdcomrxinx);
	check_bit("C1.rxdone",    1'b1, u_dut.g_cd.cdrom_intreq[28]);
	check_bit("C1.drivexmit", 1'b0, u_dut.g_cd.cdrom_intreq[30]); // not yet
	check8 ("C1.recvlen", 8'd8, {2'h0, u_dut.g_cd.cdrom_receive_length});
	// Resume
	write_rxcmp(8'd8);
	wait_rx_done(200, cyc);
	check8 ("C2.mem[4]", 8'hA4, mem[16'h0004]);
	check8 ("C2.mem[7]", 8'hA7, mem[16'h0007]);
	check8 ("C2.rxinx",  8'd8,  u_dut.g_cd.cdcomrxinx);
	check_bit("C2.drivexmit", 1'b1, u_dut.g_cd.cdrom_intreq[30]);
	check8 ("C2.recvlen", 8'd0, {2'h0, u_dut.g_cd.cdrom_receive_length});

	// =====================================================================
	// Test D: tx_dma_delay observed. After write_txcmp the engine must wait
	// 3 ticks before the first dma_req. (We set bfm_extra_delay=10 to make
	// the timing easier to read, then count cycles from txcmp write to first
	// dma_req high.)
	// =====================================================================
	$display("--- Test D: tx_dma_delay ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	set_misc_base(24'h040000);
	mem[16'h0200] = 8'hEE;
	u_dut.g_cd.cdrom_command_length = 6'd0;
	u_dut.g_cd.cdcomtxinx           = 8'd0;
	set_config(CFG_TXD);
	bfm_extra_delay = 0;
	write_txcmp(8'd1);
	// Right after the bus_write_byte_lo task returns, we are one clock past
	// the cycle that latched txcmp. The delay reload was =3 in that latch
	// cycle, so on the cycle after, delay=2; cycle+1: delay=1; cycle+2:
	// delay=0; cycle+3: tx_busy goes high → dma_req high. So expect dma_req
	// high no earlier than 3 cycles after the write completes.
	begin
		int cycles_to_req;
		cycles_to_req = 0;
		while (!dma_req && cycles_to_req < 20) begin
			@(posedge clk);
			cycles_to_req = cycles_to_req + 1;
		end
		if (cycles_to_req < 2) begin
			$display("FAIL D.tx_delay: dma_req fired in %0d cycles, expected >=2 (t=%0t)", cycles_to_req, $time);
			errs++;
		end
		checks++;
	end
	wait_tx_done(200, cyc);
	check8 ("D.cmd[0]", 8'hEE, u_dut.g_cd.cdrom_command_buffer[0]);

	// =====================================================================
	// Test E: TX gated by CDFLAG_ENABLE. With ENABLE high, even with TXD set
	// and indices differing, the engine must NOT pull bytes.
	// =====================================================================
	$display("--- Test E: TX gated by ENABLE ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	set_misc_base(24'h050000);
	mem[16'h0200] = 8'h99;
	u_dut.g_cd.cdrom_command_length = 6'd0;
	u_dut.g_cd.cdcomtxinx           = 8'd0;
	// TXD + ENABLE both set → TX must stay idle.
	set_config(CFG_TXD | CFG_ENA);
	write_txcmp(8'd1);
	repeat (60) @(posedge clk);
	check8 ("E.cmdlen_stays_0", 8'd0, {2'h0, u_dut.g_cd.cdrom_command_length});
	check8 ("E.txinx_stays_0",  8'd0, u_dut.g_cd.cdcomtxinx);
	check_bit("E.txdone_clear", 1'b0, u_dut.g_cd.cdrom_intreq[27]);
	// Drop ENABLE, expect engine to run.
	set_config(CFG_TXD);
	wait_tx_done(200, cyc);
	check8 ("E.cmd[0]_after", 8'h99, u_dut.g_cd.cdrom_command_buffer[0]);
	check8 ("E.txinx_after",  8'd1,  u_dut.g_cd.cdcomtxinx);

	// =====================================================================
	// Test F: TX gated by pending RX (receive_length > 0).
	// Inject a 1-byte response without enabling RXD; TX should NOT run.
	// =====================================================================
	$display("--- Test F: TX gated by receive_length ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	set_misc_base(24'h060000);
	mem[16'h0200] = 8'h77;
	u_dut.g_cd.cdrom_command_length = 6'd0;
	u_dut.g_cd.cdcomtxinx           = 8'd0;
	u_dut.g_cd.cdrom_result_buffer[0] = 8'h55;
	u_dut.g_cd.cdrom_receive_length   = 6'd1;
	u_dut.g_cd.cdrom_receive_offset   = 6'd0;
	set_config(CFG_TXD); // RXD off → RX engine cannot run either
	write_txcmp(8'd1);
	repeat (60) @(posedge clk);
	check8 ("F.cmdlen_stays_0", 8'd0, {2'h0, u_dut.g_cd.cdrom_command_length});
	check8 ("F.txinx_stays_0",  8'd0, u_dut.g_cd.cdcomtxinx);
	// Drop receive_length manually (M3 will be the one to clear it via RX
	// drain; here we simulate the post-drain state) and expect TX to run.
	u_dut.g_cd.cdrom_receive_length = 6'd0;
	wait_tx_done(200, cyc);
	check8 ("F.cmd[0]_after", 8'h77, u_dut.g_cd.cdrom_command_buffer[0]);

	// =====================================================================
	// Test G: TX wrap. cdcomtxinx=0xFE, cdcomtxcmp=0x02 → 4 bytes pulled
	// (offsets 0xFE, 0xFF, 0x00, 0x01 within base+0x200).
	// =====================================================================
	$display("--- Test G: TX index wrap ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	set_misc_base(24'h070000);
	// First byte's low nibble must be 0x0B-0x0F (unknown opcode → frame at
	// buffer-full) so the M3 framer doesn't gate TX after the third byte.
	mem[16'h0200 + 16'h00FE] = 8'h0B;
	mem[16'h0200 + 16'h00FF] = 8'h02;
	mem[16'h0200 + 16'h0000] = 8'h03;
	mem[16'h0200 + 16'h0001] = 8'h04;
	u_dut.g_cd.cdrom_command_length = 6'd0;
	u_dut.g_cd.cdcomtxinx           = 8'hFE;
	set_config(CFG_TXD);
	write_txcmp(8'h02);
	wait_tx_done(400, cyc);
	check8 ("G.cmd[0]", 8'h0B, u_dut.g_cd.cdrom_command_buffer[0]);
	check8 ("G.cmd[1]", 8'h02, u_dut.g_cd.cdrom_command_buffer[1]);
	check8 ("G.cmd[2]", 8'h03, u_dut.g_cd.cdrom_command_buffer[2]);
	check8 ("G.cmd[3]", 8'h04, u_dut.g_cd.cdrom_command_buffer[3]);
	check8 ("G.txinx",  8'h02, u_dut.g_cd.cdcomtxinx);
	check_bit("G.txdone", 1'b1, u_dut.g_cd.cdrom_intreq[27]);

	// =====================================================================
	// Test H: BFM with 5-cycle ack latency. Engine must hold dma_req stable
	// across the wait; same byte should land in command_buffer.
	// =====================================================================
	$display("--- Test H: held dma_req across BFM latency ---");
	do_reset();
	bus_write_long(5'b00100, 32'hFF000000);
	set_misc_base(24'h080000);
	mem[16'h0200] = 8'hF0;
	mem[16'h0201] = 8'hF1;
	u_dut.g_cd.cdrom_command_length = 6'd0;
	u_dut.g_cd.cdcomtxinx           = 8'd0;
	bfm_extra_delay = 5;
	set_config(CFG_TXD);
	write_txcmp(8'd2);
	wait_tx_done(400, cyc);
	check8 ("H.cmd[0]", 8'hF0, u_dut.g_cd.cdrom_command_buffer[0]);
	check8 ("H.cmd[1]", 8'hF1, u_dut.g_cd.cdrom_command_buffer[1]);
	check8 ("H.txinx",  8'd2,  u_dut.g_cd.cdcomtxinx);
	check_bit("H.txdone", 1'b1, u_dut.g_cd.cdrom_intreq[27]);
	bfm_extra_delay = 0;

	// =====================================================================
	// Done
	// =====================================================================
	$display("tb_akiko_txrx_dma: %0d checks, %0d errors", checks, errs);
	$finish;
end

endmodule
