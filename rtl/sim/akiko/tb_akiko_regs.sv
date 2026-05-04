// SPDX-License-Identifier: GPL-3.0-or-later
//
// Akiko M1 register/IRQ bench.
//
// Three instances run with the same stimulus:
//   u_ref  : akiko_legacy_ref (frozen current shipping akiko.v)
//   u_dut0 : new akiko with NATIVE_CD32=0 (must be bit-identical to u_ref)
//   u_dut1 : new akiko with NATIVE_CD32=1 (full CD register behavior)
//
// Differential checks for ID/C2P guarantee the new file does not regress
// existing AGA-mode boots. Native-mode-only checks cover INTREQ/INTENA,
// CONFIG, DMA-base masks, read-mirror aliasing, PBX OR-only semantics,
// W1C IRQ clears, and the akiko_irq output.
//
// Hierarchical reads/writes into u_dut1.g_cd.cdrom_intreq are bench-only
// backdoors used to set INTREQ bits (which the CPU cannot write directly)
// so the W1C clear paths can be exercised.

`timescale 1ns / 1ps

module tb_akiko_regs;

// -----------------------------------------------------------------------
// Watchdog
// -----------------------------------------------------------------------
initial begin
	#200000 $fatal(1, "tb_akiko_regs: watchdog timeout");
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

// -----------------------------------------------------------------------
// DUT outputs
// -----------------------------------------------------------------------
wire [15:0] dout_ref;
wire [15:0] dout_dut0;
wire [15:0] dout_dut1;
wire        irq_dut1;

akiko_legacy_ref u_ref (
	.clk(clk),
	.cs(cs), .rd(rd), .wr(wr),
	.addr(addr), .din(din), .dout(dout_ref)
);

akiko #(.NATIVE_CD32(0)) u_dut0 (
	.clk(clk), .reset(reset),
	.cs(cs), .rd(rd), .wr(wr),
	.lds(lds), .uds(uds),
	.addr(addr), .din(din), .dout(dout_dut0),
	.akiko_irq(),
	// M2 DMA port — M1 bench doesn't exercise it; tie off cleanly.
	.dma_req(), .dma_we(), .dma_baddr(), .dma_wbyte(),
	.dma_rbyte(8'h00), .dma_ack(1'b0),
	// M3 HPS bridge — M1 bench doesn't exercise it.
	.hps_cmd_pending(), .hps_cmd_byte(),
	.hps_cmd_pop(1'b0), .hps_cmd_done(1'b0),
	.hps_result_push(1'b0), .hps_result_byte(8'h00), .hps_result_done(1'b0),
	// M4 HPS sector channel — M1 bench doesn't exercise it.
	.hps_sec_req(), .hps_sec_status(),
	.hps_sec_push(1'b0), .hps_sec_byte(8'h00), .hps_sec_done(1'b0),
	// Phase 18: rx_busy status output — bench doesn't observe it.
	.hps_rx_busy(),
	// Phase 32 / 32.5: NVRAM port — bench doesn't exercise it.
	.hps_nvr_addr(10'd0), .hps_nvr_din(8'h00), .hps_nvr_we(1'b0),
	.hps_nvr_dout(), .hps_nvr_clear_dirty(1'b0), .hps_nvr_dirty()
);

akiko #(.NATIVE_CD32(1)) u_dut1 (
	.clk(clk), .reset(reset),
	.cs(cs), .rd(rd), .wr(wr),
	.lds(lds), .uds(uds),
	.addr(addr), .din(din), .dout(dout_dut1),
	.akiko_irq(irq_dut1),
	.dma_req(), .dma_we(), .dma_baddr(), .dma_wbyte(),
	.dma_rbyte(8'h00), .dma_ack(1'b0),
	.hps_cmd_pending(), .hps_cmd_byte(),
	.hps_cmd_pop(1'b0), .hps_cmd_done(1'b0),
	.hps_result_push(1'b0), .hps_result_byte(8'h00), .hps_result_done(1'b0),
	.hps_sec_req(), .hps_sec_status(),
	.hps_sec_push(1'b0), .hps_sec_byte(8'h00), .hps_sec_done(1'b0),
	.hps_rx_busy(),
	.hps_nvr_addr(10'd0), .hps_nvr_din(8'h00), .hps_nvr_we(1'b0),
	.hps_nvr_dout(), .hps_nvr_clear_dirty(1'b0), .hps_nvr_dirty()
);

// -----------------------------------------------------------------------
// Constants (mirror of akiko.v localparams)
// -----------------------------------------------------------------------
localparam [31:0] CDINT_SUBCODE   = 32'h80000000;
localparam [31:0] CDINT_DRIVEXMIT = 32'h40000000;
localparam [31:0] CDINT_DRIVERECV = 32'h20000000;
localparam [31:0] CDINT_RXDMADONE = 32'h10000000;
localparam [31:0] CDINT_TXDMADONE = 32'h08000000;
localparam [31:0] CDINT_PBX       = 32'h04000000;
localparam [31:0] CDINT_OVERFLOW  = 32'h02000000;

localparam [31:0] CFG_PBX    = 32'h08000000; // bit 27
localparam [31:0] CFG_ENABLE = 32'h04000000; // bit 26

// -----------------------------------------------------------------------
// Score keeping
// -----------------------------------------------------------------------
int checks = 0;
int errs   = 0;

task automatic check16(string name, logic [15:0] expected, logic [15:0] actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected 0x%04h got 0x%04h (t=%0t)", name, expected, actual, $time);
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

task automatic check_b(string name, logic expected, logic actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected %0b got %0b (t=%0t)", name, expected, actual, $time);
		errs++;
	end
endtask

// -----------------------------------------------------------------------
// Bus drivers
// -----------------------------------------------------------------------
// Word write (both byte enables active)
task automatic word_write(input [4:0] a, input [15:0] data);
	@(negedge clk);
	addr = a;
	din  = data;
	cs   = 1;
	rd   = 0;
	wr   = 1;
	uds  = 1;
	lds  = 1;
	@(posedge clk);
	@(negedge clk);
	cs = 0; wr = 0; uds = 0; lds = 0;
endtask

// Byte write at byte address ba (5:0). Even ba -> uds, odd ba -> lds.
task automatic byte_write(input [5:0] ba, input [7:0] data);
	@(negedge clk);
	addr = ba[5:1];
	cs   = 1;
	rd   = 0;
	wr   = 1;
	if (ba[0] == 1'b0) begin
		uds = 1; lds = 0; din = {data, 8'h00};
	end else begin
		uds = 0; lds = 1; din = {8'h00, data};
	end
	@(posedge clk);
	@(negedge clk);
	cs = 0; wr = 0; uds = 0; lds = 0;
endtask

// Word read: drive cs/rd then sample dout_* combinationally; deassert.
task automatic word_read(input [4:0] a, output [15:0] r_ref, output [15:0] r_dut0, output [15:0] r_dut1);
	@(negedge clk);
	addr = a;
	cs   = 1;
	rd   = 1;
	wr   = 0;
	uds  = 1;
	lds  = 1;
	#1;
	r_ref  = dout_ref;
	r_dut0 = dout_dut0;
	r_dut1 = dout_dut1;
	@(posedge clk);
	@(negedge clk);
	cs = 0; rd = 0; uds = 0; lds = 0;
endtask

// Convenience: read just dut1 (CD-only addrs not present in legacy)
task automatic read_dut1(input [4:0] a, output [15:0] r);
	logic [15:0] tmp_ref, tmp_dut0;
	word_read(a, tmp_ref, tmp_dut0, r);
endtask

// -----------------------------------------------------------------------
// Stimulus
// -----------------------------------------------------------------------
logic [15:0] vr, v0, v1;

initial begin
	$display("tb_akiko_regs starting");

	// Reset
	reset = 1;
	repeat (4) @(posedge clk);
	@(negedge clk);
	reset = 0;
	repeat (2) @(posedge clk);

	// =================================================================
	// A. ID reads — differential
	// =================================================================
	word_read(5'd0, vr, v0, v1);
	check16("ID@$00 ref==C0CA", 16'hC0CA, vr);
	check16("ID@$00 dut0==ref", vr, v0);
	check16("ID@$00 dut1==ref", vr, v1);

	word_read(5'd1, vr, v0, v1);
	check16("ID@$02 ref==CAFE", 16'hCAFE, vr);
	check16("ID@$02 dut0==ref", vr, v0);
	check16("ID@$02 dut1==ref", vr, v1);

	// =================================================================
	// B. C2P differential — write 8 words then read 16 words; all three
	// modules must produce identical reads cycle-for-cycle.
	// =================================================================
	for (int i = 0; i < 8; i++) begin
		word_write(5'b11100, 16'(i*16'h0101));
	end
	for (int i = 0; i < 16; i++) begin
		word_read(5'b11100, vr, v0, v1);
		check16($sformatf("C2P read[%0d] dut0==ref", i), vr, v0);
		check16($sformatf("C2P read[%0d] dut1==ref", i), vr, v1);
	end

	// =================================================================
	// C. dut0 must be bit-identical to ref across the whole address
	// window for both reads and writes (regression guarantee).
	// Sweep all 32 word addresses with arbitrary writes then reads.
	// =================================================================
	for (int a = 0; a < 32; a++) begin
		word_write(5'(a), 16'(a*16'h1111));
	end
	for (int a = 0; a < 32; a++) begin
		word_read(5'(a), vr, v0, v1);
		check16($sformatf("addr=%0d dut0==ref", a), vr, v0);
	end

	// =================================================================
	// D. INTREQ initial state
	// =================================================================
	read_dut1(5'b00010, v1); // $04
	check16("INTREQ hi initial", 16'h0000, v1);
	read_dut1(5'b00011, v1); // $06
	check16("INTREQ lo initial", 16'h0000, v1);

	// =================================================================
	// E. INTENA write + INTENA_MASK (0xff000000) + mirror at $0C-$0F
	// =================================================================
	// Reset CD state by force-clearing intena via reset
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;

	word_write(5'b00100, 16'hF234); // $08-$09
	word_write(5'b00101, 16'h5678); // $0A-$0B
	read_dut1(5'b00100, v1);
	check16("INTENA hi after wr (only F2 survives)", 16'hF200, v1);
	read_dut1(5'b00101, v1);
	check16("INTENA lo after wr (masked to 0)", 16'h0000, v1);
	// Mirror reads
	read_dut1(5'b00110, v1);
	check16("INTENA mirror $0C", 16'hF200, v1);
	read_dut1(5'b00111, v1);
	check16("INTENA mirror $0E", 16'h0000, v1);

	// =================================================================
	// F. cdrom_addressdata mask 0x00fff000
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	word_write(5'b01000, 16'hABCD); // $10-$11
	word_write(5'b01001, 16'hEF12); // $12-$13
	// Pre-mask 0xABCDEF12 & 0x00FFF000 = 0x00CDE000
	check32("addressdata after long write", 32'h00CDE000, u_dut1.g_cd.cdrom_addressdata);

	// =================================================================
	// G. cdrom_addressmisc mask 0x00fffc00
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	word_write(5'b01010, 16'hABCD); // $14-$15
	word_write(5'b01011, 16'hEF12); // $16-$17
	// Pre-mask 0xABCDEF12 & 0x00FFFC00 = 0x00CDEC00
	check32("addressmisc after long write", 32'h00CDEC00, u_dut1.g_cd.cdrom_addressmisc);

	// =================================================================
	// H. Read-mirror aliasing in $10-$1F (subcodeoffset/txinx/rxinx/0)
	// Backdoor-poke the live status bytes and verify all 4 mirror groups
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	u_dut1.g_cd.cdrom_subcodeoffset = 8'hAA;
	u_dut1.g_cd.cdcomtxinx          = 8'hBB;
	u_dut1.g_cd.cdcomrxinx          = 8'hCC;
	@(negedge clk);
	for (int slot = 0; slot < 4; slot++) begin
		automatic logic [4:0] base = 5'(5'b01000 + slot * 2);
		read_dut1(base,        v1);
		check16($sformatf("mirror[%0d] {sub,tx}", slot), 16'hAABB, v1);
		read_dut1(base + 5'd1, v1);
		check16($sformatf("mirror[%0d] {rx,0}",  slot), 16'hCC00, v1);
	end

	// =================================================================
	// I. CONFIG mask 0xff800000
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	word_write(5'b10010, 16'hFFFF); // $24-$25
	word_write(5'b10011, 16'hFFFF); // $26-$27
	// Pre-mask 0xFFFFFFFF & 0xFF800000 = 0xFF800000
	check32("CONFIG after all-ones write", 32'hFF800000, u_dut1.g_cd.cdrom_flags);

	// =================================================================
	// J. PBX OR-only write semantics + clear-when-CONFIG.PBX-off
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	// Enable PBX in CONFIG so the PBX register can latch
	word_write(5'b10010, 16'h0800); // CONFIG[31:16]=0x0800 -> bit 27 (PBX)
	check32("CONFIG with PBX enable", 32'h08000000, u_dut1.g_cd.cdrom_flags);

	// Set PBX bits 0-3
	word_write(5'b10000, 16'h000F);
	check32("PBX after wr 0x000F", 32'h0000000F, {16'h0, u_dut1.g_cd.cdrom_pbx});
	// OR in bits 4-7; previous bits must remain set (no clear via write-zero)
	word_write(5'b10000, 16'h00F0);
	check32("PBX after wr 0x00F0 OR-only", 32'h000000FF, {16'h0, u_dut1.g_cd.cdrom_pbx});
	// Writing zero must not clear; existing bits stay
	word_write(5'b10000, 16'h0000);
	check32("PBX after wr 0x0000", 32'h000000FF, {16'h0, u_dut1.g_cd.cdrom_pbx});
	// Disabling PBX in CONFIG forces register to 0
	word_write(5'b10010, 16'h0000); // clear bit 27
	// Need to re-trigger pbx-clear path: write to PBX again
	word_write(5'b10000, 16'h0000);
	check32("PBX cleared when CONFIG.PBX off", 32'h0, {16'h0, u_dut1.g_cd.cdrom_pbx});

	// =================================================================
	// K. INTREQ W1C: each clear-on-write address clears its bit
	// =================================================================
	// SUBCODE: write to $18 clears bit 31
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	u_dut1.g_cd.cdrom_intreq = CDINT_SUBCODE | CDINT_PBX | CDINT_OVERFLOW;
	@(negedge clk);
	byte_write(6'h18, 8'h00); // any value
	check32("INTREQ SUBCODE cleared by $18 write",
	        CDINT_PBX | CDINT_OVERFLOW, u_dut1.g_cd.cdrom_intreq);

	// TXDMADONE: write to $1D clears bit 27 AND latches txcmp
	u_dut1.g_cd.cdrom_intreq = CDINT_TXDMADONE | CDINT_RXDMADONE;
	@(negedge clk);
	byte_write(6'h1D, 8'h42);
	check32("INTREQ TXDMADONE cleared by $1D write",
	        CDINT_RXDMADONE, u_dut1.g_cd.cdrom_intreq);
	check32("txcmp latched", 32'h42, {24'h0, u_dut1.g_cd.cdcomtxcmp});

	// RXDMADONE: write to $1F clears bit 28 AND latches rxcmp
	u_dut1.g_cd.cdrom_intreq = CDINT_TXDMADONE | CDINT_RXDMADONE;
	@(negedge clk);
	byte_write(6'h1F, 8'h7E);
	check32("INTREQ RXDMADONE cleared by $1F write",
	        CDINT_TXDMADONE, u_dut1.g_cd.cdrom_intreq);
	check32("rxcmp latched", 32'h7E, {24'h0, u_dut1.g_cd.cdcomrxcmp});

	// PBX clear: write to $20 clears bit 26
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	word_write(5'b10010, 16'h0800);  // CONFIG.PBX on so the write doesn't auto-clear pbx
	u_dut1.g_cd.cdrom_intreq = CDINT_PBX | CDINT_SUBCODE;
	@(negedge clk);
	word_write(5'b10000, 16'h0000);
	check32("INTREQ PBX cleared by $20 write",
	        CDINT_SUBCODE, u_dut1.g_cd.cdrom_intreq);

	// DRIVEXMIT: write to $28 (uds) with CONFIG.TXD off clears bit 30
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	u_dut1.g_cd.cdrom_intreq = CDINT_DRIVEXMIT | CDINT_OVERFLOW;
	@(negedge clk);
	byte_write(6'h28, 8'h99);
	check32("INTREQ DRIVEXMIT cleared by $28 write",
	        CDINT_OVERFLOW, u_dut1.g_cd.cdrom_intreq);
	check32("pio_byte latched", 32'h99, {24'h0, u_dut1.g_cd.pio_byte});

	// =================================================================
	// L. CONFIG.ENABLE 0->1 transition clears OVERFLOW IRQ
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	u_dut1.g_cd.cdrom_intreq = CDINT_OVERFLOW | CDINT_SUBCODE;
	@(negedge clk);
	// Write CONFIG with ENABLE bit (bit 26) set; ENABLE was 0 prior to write.
	word_write(5'b10010, 16'h0400); // bit 26 -> high half bit 10 -> 0x0400
	check32("OVERFLOW cleared on ENABLE 0->1",
	        CDINT_SUBCODE, u_dut1.g_cd.cdrom_intreq);

	// Writing CONFIG again with ENABLE still high should NOT re-clear OVERFLOW
	u_dut1.g_cd.cdrom_intreq = CDINT_OVERFLOW;
	@(negedge clk);
	word_write(5'b10010, 16'h0400); // ENABLE stays high
	check32("OVERFLOW preserved on ENABLE held high",
	        CDINT_OVERFLOW, u_dut1.g_cd.cdrom_intreq);

	// =================================================================
	// M. PIO + NVRAM read-back stubs
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	byte_write(6'h28, 8'hA5);
	read_dut1(5'b10100, v1);
	check16("PIO read-back upper byte", 16'hA500, v1);

	// Phase 13: $30 reads back live bus state (bit 7=SCL, bit 6=SDA), not
	// the master register verbatim. With DIR=0 (released, both lines float
	// high) the bus reads {1,1,6'h0} → 0xC000.
	byte_write(6'h32, 8'h00);                 // DIR = 0 (release both lines)
	byte_write(6'h30, 8'h5A);                 // master writes (no effect — released)
	read_dut1(5'b11000, v1);
	check16("NVRAM I/O released bus high", 16'hC000, v1);

	// With DIR=0xC0 (drive both) and IO=0x40 (SCL=0, SDA=1) the bus reads
	// {0,1,6'h0} → 0x4000. Slave isn't actively driving (no START), so
	// SDA reflects what the master is forcing.
	byte_write(6'h32, 8'hC0);                 // DIR: SCL+SDA driven by master
	byte_write(6'h30, 8'h40);                 // SCL=0, SDA=1
	read_dut1(5'b11000, v1);
	check16("NVRAM I/O master-driven SCL=0 SDA=1", 16'h4000, v1);

	byte_write(6'h32, 8'h3C);
	read_dut1(5'b11001, v1);
	check16("NVRAM DIR read-back", 16'h3C00, v1);

	// =================================================================
	// N. akiko_irq output: |(intreq[31:25] & intena[31:25])
	// =================================================================
	reset = 1; @(posedge clk); @(negedge clk); reset = 0;
	check_b("IRQ low after reset", 1'b0, irq_dut1);

	// Set intena[31:24] = 0xFF, force intreq SUBCODE -> IRQ should rise
	word_write(5'b00100, 16'hFF00); // INTENA[31:24] = 0xFF
	u_dut1.g_cd.cdrom_intreq = CDINT_SUBCODE;
	#1;
	check_b("IRQ high when intreq & intena set", 1'b1, irq_dut1);

	// Clear via $18 write, IRQ should drop
	byte_write(6'h18, 8'h00);
	#1;
	check_b("IRQ low after SUBCODE cleared", 1'b0, irq_dut1);

	// Bit outside the documented [31:25] range must NOT raise IRQ
	u_dut1.g_cd.cdrom_intreq = 32'h01000000; // bit 24 (undefined)
	u_dut1.g_cd.cdrom_intena = 32'hFF000000;
	#1;
	check_b("IRQ low for out-of-range bit", 1'b0, irq_dut1);

	// =================================================================
	// dut0 IRQ must be tied to 0 regardless of address activity
	// =================================================================
	// (u_dut0's akiko_irq is connected to "open" but we can verify the
	// hierarchical signal stays at 0 since g_stub assigns it.)

	// =================================================================
	// Final
	// =================================================================
	@(posedge clk);
	$display("============================================");
	if (errs == 0) begin
		$display("PASS: %0d checks", checks);
		$display("============================================");
		$finish;
	end else begin
		$display("FAIL: %0d checks, %0d errors", checks, errs);
		$display("============================================");
		$fatal(1);
	end
end

endmodule
