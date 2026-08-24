// SPDX-License-Identifier: GPL-3.0-or-later
//
// Akiko save state bench.
//
// Covers the three things the savestate ports have to get right, and nothing
// else -- the register decode itself belongs to tb_akiko_regs.
//
//   A. The idle rule. ss_idle holds the freeze off while a DMA engine is
//      part-way through a transfer, and it must do NOTHING else. A version
//      that also demanded the staging buffers be empty shipped once and broke
//      saving outright: sector_ready is cleared only when the PBX engine ships
//      a sector, so a prefetched sector sits staged for as long as the title
//      likes, and every save of a running CD32 title reported FAIL_QUIESCE.
//      Both directions are checked below -- stuck high is a restore into a
//      wedged drive, stuck low is a feature that cannot be used at all, and
//      the second one is what actually happened.
//
//   B. Capture offsets. Every field is forced to a distinct value and then
//      read back out of the ss_state slice it is supposed to occupy. The
//      offsets are written out again here rather than imported from akiko.v,
//      deliberately: a bench that takes its expectations from the thing under
//      test cannot catch a wrong constant.
//
//   C. Round trip. Capture, reset the chip, restore, compare the whole 1052-bit
//      vector. This is what catches a field that is captured but not restored
//      (or the reverse) -- the failure mode that a one-directional check reads
//      as a pass.
//
// The values are forced hierarchically rather than driven through the CPU bus.
// Several of these registers have no bus write path at all (the TX/RX indices
// are engine-driven, the sector counter is DMA-driven), and the ones that do
// have side effects on the way in -- writing the TX compare reloads the 3-tick
// inhibit, which would then hold ss_idle low. What this bench is testing is
// the plumbing between the registers and the vector, so it puts the values in
// by the shortest route that reaches every one of them.

`timescale 1ns / 1ps

module tb_akiko_savestate;

initial begin
	#200000 $fatal(1, "tb_akiko_savestate: watchdog timeout");
end

// ---------------------------------------------------------------------------
// Field map. See the header: written out again on purpose.
// ---------------------------------------------------------------------------
localparam SS_W          = 1052;
localparam SS_O_INTREQ   =   0;
localparam SS_O_INTENA   =  32;
localparam SS_O_ADDRDATA =  64;
localparam SS_O_ADDRMISC =  96;
localparam SS_O_FLAGS    = 128;
localparam SS_O_PBX      = 160;
localparam SS_O_SUBCOFF  = 176;
localparam SS_O_TXINX    = 184;
localparam SS_O_RXINX    = 192;
localparam SS_O_TXCMP    = 200;
localparam SS_O_RXCMP    = 208;
localparam SS_O_SUBOFF   = 216;
localparam SS_O_SECCNT   = 224;
localparam SS_O_NVRIO    = 232;
localparam SS_O_NVRDIR   = 240;
localparam SS_O_PIO      = 248;
localparam SS_O_SUBIRQ   = 256;
localparam SS_O_SHIPINV  = 257;
localparam SS_O_CMDBUF   = 258;
localparam SS_O_CMDLEN   = 514;
localparam SS_O_RESBUF   = 520;
localparam SS_O_RXLEN    = 776;
localparam SS_O_RXOFF    = 782;
localparam SS_O_C2P      = 788;
localparam SS_O_RPTR     = 1044;
localparam SS_O_WPTR     = 1048;

// The values. Distinct in both halves of every multi-byte field, so a field
// that lost an end shows up as a failure rather than as a plausible number.
localparam [31:0] V_INTREQ   = 32'h1234_5678;
localparam [31:0] V_INTENA   = 32'h9ABC_DEF0;
localparam [31:0] V_ADDRDATA = 32'h00AB_C000;
localparam [31:0] V_ADDRMISC = 32'h00DE_F400;
localparam [31:0] V_FLAGS    = 32'hCC00_0000;
localparam [15:0] V_PBX      = 16'hBEEF;
localparam  [7:0] V_SUBCOFF  = 8'h5A;
localparam  [7:0] V_TXINX    = 8'h11;
localparam  [7:0] V_RXINX    = 8'h22;
localparam  [7:0] V_TXCMP    = 8'h33;
localparam  [7:0] V_RXCMP    = 8'h44;
localparam  [7:0] V_SUBOFF   = 8'h80;
localparam  [7:0] V_SECCNT   = 8'h7E;
localparam  [7:0] V_NVRIO    = 8'hC0;
localparam  [7:0] V_NVRDIR   = 8'h40;
localparam  [7:0] V_PIO      = 8'h99;
localparam        V_SUBIRQ   = 1'b1;
localparam        V_SHIPINV  = 1'b1;
localparam  [3:0] V_RPTR     = 4'h5;
localparam  [3:0] V_WPTR     = 4'hD;
localparam  [5:0] V_CMDLEN   = 6'd7;
localparam  [5:0] V_RXLEN    = 6'd19;
localparam  [5:0] V_RXOFF    = 6'd6;

// ---------------------------------------------------------------------------
// Clock, reset, DUT
// ---------------------------------------------------------------------------
logic clk = 0;
initial forever #5 clk = ~clk;

logic        reset = 1;
logic        sec_push = 0;
logic [15:0] sec_word = 16'h0000;
logic        ss_ld    = 0;
logic [SS_W-1:0] ss_ld_data = {SS_W{1'b0}};

wire [SS_W-1:0] ss_state;
wire            ss_idle;

akiko #(.NATIVE_CD32(1)) u_dut (
	.clk(clk), .reset(reset),
	.cs(1'b0), .rd(1'b0), .wr(1'b0),
	.lds(1'b0), .uds(1'b0),
	.addr(5'd0), .din(16'h0000), .dout(),
	.akiko_irq(),
	.dma_req(), .dma_we(), .dma_baddr(), .dma_wbyte(),
	.dma_rbyte(8'h00), .dma_ack(1'b0), .dma_arm(1'b0),
	.hps_cmd_pending(), .hps_cmd_byte(),
	.hps_cmd_pop(1'b0), .hps_cmd_done(1'b0),
	.hps_result_push(1'b0), .hps_result_byte(8'h00), .hps_result_done(1'b0),
	.hps_sec_req(), .hps_sec_status(),
	.hps_sec_push(sec_push), .hps_sec_word(sec_word), .hps_sec_done(1'b0),
	.hps_rx_busy(),
	.hps_nvr_addr(10'd0),
	.hps_nvr_dout(), .hps_nvr_clear_dirty(1'b0), .hps_nvr_dirty(),
	.nvr_load_addr(10'd0), .nvr_load_din(8'h00), .nvr_load_we(1'b0),
	.hps_subcode_push(1'b0), .hps_subcode_byte(8'h00), .hps_subcode_done(1'b0),
	.ss_state(ss_state), .ss_ld(ss_ld), .ss_ld_data(ss_ld_data),
	.ss_idle(ss_idle)
);

// ---------------------------------------------------------------------------
// Score keeping
// ---------------------------------------------------------------------------
int checks = 0;
int errs   = 0;

task automatic check(input string name, input logic [31:0] expected,
                     input logic [31:0] actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected 0x%08h got 0x%08h (t=%0t)",
		         name, expected, actual, $time);
		errs++;
	end
	else $display("PASS %s", name);
endtask

task automatic check_b(input string name, input logic expected,
                       input logic actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected %0b got %0b (t=%0t)",
		         name, expected, actual, $time);
		errs++;
	end
	else $display("PASS %s", name);
endtask

// ---------------------------------------------------------------------------
// Stimulus
// ---------------------------------------------------------------------------
integer i;
reg [SS_W-1:0] captured;
reg            same;

task automatic pulse_reset;
	begin
		@(negedge clk); reset = 1;
		repeat (3) @(posedge clk);
		@(negedge clk); reset = 0;
		repeat (2) @(posedge clk);
	end
endtask

// Put every field in by the shortest route. See the header for why this is
// hierarchical rather than driven through the bus.
task automatic load_values;
	begin
		@(negedge clk);
		u_dut.g_cd.cdrom_intreq         = V_INTREQ;
		u_dut.g_cd.cdrom_intena         = V_INTENA;
		u_dut.g_cd.cdrom_addressdata    = V_ADDRDATA;
		u_dut.g_cd.cdrom_addressmisc    = V_ADDRMISC;
		u_dut.g_cd.cdrom_flags          = V_FLAGS;
		u_dut.g_cd.cdrom_pbx            = V_PBX;
		u_dut.g_cd.cdrom_subcodeoffset  = V_SUBCOFF;
		u_dut.g_cd.cdcomtxinx           = V_TXINX;
		u_dut.g_cd.cdcomrxinx           = V_RXINX;
		u_dut.g_cd.cdcomtxcmp           = V_TXCMP;
		u_dut.g_cd.cdcomrxcmp           = V_RXCMP;
		u_dut.g_cd.subcode_off          = V_SUBOFF;
		u_dut.g_cd.cdrom_sector_counter = V_SECCNT;
		u_dut.g_cd.nvram_io             = V_NVRIO;
		u_dut.g_cd.nvram_dir            = V_NVRDIR;
		u_dut.g_cd.pio_byte             = V_PIO;
		u_dut.g_cd.subcode_irq          = V_SUBIRQ;
		u_dut.g_cd.pbx_ship_invalid     = V_SHIPINV;
		u_dut.g_cd.cdrom_command_length = V_CMDLEN;
		u_dut.g_cd.cdrom_receive_length = V_RXLEN;
		u_dut.g_cd.cdrom_receive_offset = V_RXOFF;
		for (i = 0; i < 32; i = i + 1) begin
			u_dut.buff[i] = 8'hA0 + i[7:0];
			u_dut.g_cd.cdrom_command_buffer[i] = 8'h40 + i[7:0];
			u_dut.g_cd.cdrom_result_buffer[i]  = 8'h80 + i[7:0];
		end
		u_dut.rptr = V_RPTR;
		u_dut.wptr = V_WPTR;
		@(posedge clk);
		#1;
	end
endtask

initial begin
	$display("== tb_akiko_savestate");

	pulse_reset();

	// -------------------------------------------------------------------
	// A. The idle rule.
	// -------------------------------------------------------------------
	check_b("idle after reset", 1'b1, ss_idle);

	// A DMA engine mid-transfer holds the freeze off. This is the whole of
	// what ss_idle is for.
	@(negedge clk);
	u_dut.g_cd.pbx_busy = 1'b1;
	@(posedge clk); #1;
	check_b("idle drops while the PBX engine ships", 1'b0, ss_idle);
	@(negedge clk);
	u_dut.g_cd.pbx_busy = 1'b0;
	@(posedge clk); #1;
	check_b("idle returns when the engine finishes", 1'b1, ss_idle);

	// And the other direction, which is the one that shipped broken. A
	// staged sector, a staged subcode block, a partly filled buffer and a
	// stalled host result pointer are all states a healthy machine sits in
	// for an unbounded time. None of them may hold the freeze off: dropping
	// a staged sector costs nothing, because cdrom_sector_counter only
	// advances when a sector is SHIPPED, so the next hps_sec_req asks
	// userspace for the very same LBA again.
	@(negedge clk);
	u_dut.g_cd.sector_ready      = 1'b1;
	u_dut.g_cd.subcode_ready     = 1'b1;
	u_dut.g_cd.sec_wr_ptr        = 12'd1000;
	u_dut.g_cd.sub_wr_ptr        = 7'd40;
	u_dut.g_cd.hps_result_wr_ptr = 6'd5;
	u_dut.g_cd.hps_cmd_rd_ptr    = 6'd3;
	u_dut.g_cd.cdrom_receive_length = 6'd12;
	@(posedge clk); #1;
	check_b("staged data does not hold off the freeze", 1'b1, ss_idle);

	// A word pushed and finished is not a transfer in flight, so it must
	// not hold the freeze off.
	@(negedge clk); sec_word = 16'h5A5A; sec_push = 1;
	@(posedge clk);
	@(negedge clk); sec_push = 0;
	#1;
	check_b("a finished slow-path push still allows the freeze",
	        1'b1, ss_idle);

	pulse_reset();
	#1;
	check_b("idle again after reset", 1'b1, ss_idle);

	// -------------------------------------------------------------------
	// B. Capture offsets.
	// -------------------------------------------------------------------
	load_values();

	check("capture INTREQ",   V_INTREQ,   ss_state[SS_O_INTREQ   +: 32]);
	check("capture INTENA",   V_INTENA,   ss_state[SS_O_INTENA   +: 32]);
	check("capture ADDRDATA", V_ADDRDATA, ss_state[SS_O_ADDRDATA +: 32]);
	check("capture ADDRMISC", V_ADDRMISC, ss_state[SS_O_ADDRMISC +: 32]);
	check("capture FLAGS",    V_FLAGS,    ss_state[SS_O_FLAGS    +: 32]);
	check("capture PBX",      V_PBX,      ss_state[SS_O_PBX      +: 16]);
	check("capture SUBCOFF",  V_SUBCOFF,  ss_state[SS_O_SUBCOFF  +:  8]);
	check("capture TXINX",    V_TXINX,    ss_state[SS_O_TXINX    +:  8]);
	check("capture RXINX",    V_RXINX,    ss_state[SS_O_RXINX    +:  8]);
	check("capture TXCMP",    V_TXCMP,    ss_state[SS_O_TXCMP    +:  8]);
	check("capture RXCMP",    V_RXCMP,    ss_state[SS_O_RXCMP    +:  8]);
	check("capture SUBOFF",   V_SUBOFF,   ss_state[SS_O_SUBOFF   +:  8]);
	check("capture SECCNT",   V_SECCNT,   ss_state[SS_O_SECCNT   +:  8]);
	check("capture NVRIO",    V_NVRIO,    ss_state[SS_O_NVRIO    +:  8]);
	check("capture NVRDIR",   V_NVRDIR,   ss_state[SS_O_NVRDIR   +:  8]);
	check("capture PIO",      V_PIO,      ss_state[SS_O_PIO      +:  8]);
	check_b("capture SUBIRQ",  V_SUBIRQ,  ss_state[SS_O_SUBIRQ]);
	check_b("capture SHIPINV", V_SHIPINV, ss_state[SS_O_SHIPINV]);
	check("capture RPTR",     V_RPTR,     ss_state[SS_O_RPTR     +:  4]);
	check("capture WPTR",     V_WPTR,     ss_state[SS_O_WPTR     +:  4]);
	check("capture CMDLEN",   V_CMDLEN,   ss_state[SS_O_CMDLEN   +:  6]);
	check("capture RXLEN",    V_RXLEN,    ss_state[SS_O_RXLEN    +:  6]);
	check("capture RXOFF",    V_RXOFF,    ss_state[SS_O_RXOFF    +:  6]);

	// The C2P buffer, byte by byte: buff[0] at the low byte of the section.
	// A reversed loop is the mistake this is here to catch, and with a
	// monotonic fill pattern it is the only shape that survives the ends.
	same = 1'b1;
	for (i = 0; i < 32; i = i + 1)
		if (ss_state[SS_O_C2P + i*8 +: 8] !== (8'hA0 + i[7:0])) same = 1'b0;
	check_b("capture C2P buffer in order", 1'b1, same);

	same = 1'b1;
	for (i = 0; i < 32; i = i + 1)
		if (ss_state[SS_O_CMDBUF + i*8 +: 8] !== (8'h40 + i[7:0])) same = 1'b0;
	check_b("capture command buffer in order", 1'b1, same);

	same = 1'b1;
	for (i = 0; i < 32; i = i + 1)
		if (ss_state[SS_O_RESBUF + i*8 +: 8] !== (8'h80 + i[7:0])) same = 1'b0;
	check_b("capture result buffer in order", 1'b1, same);

	captured = ss_state;

	// -------------------------------------------------------------------
	// C. Round trip.
	// -------------------------------------------------------------------
	pulse_reset();
	#1;
	check_b("reset cleared the vector", 1'b0, (ss_state === captured));

	@(negedge clk);
	ss_ld_data = captured;
	ss_ld      = 1;
	@(posedge clk);
	@(negedge clk);
	ss_ld = 0;
	@(posedge clk);
	#1;

	check_b("restored vector matches the captured one",
	        1'b1, (ss_state === captured));

	// Field by field as well as whole-vector, so a failure says which end
	// of the map is wrong instead of just that something is.
	check("restore INTREQ",   V_INTREQ,   ss_state[SS_O_INTREQ   +: 32]);
	check("restore ADDRMISC", V_ADDRMISC, ss_state[SS_O_ADDRMISC +: 32]);
	check("restore SECCNT",   V_SECCNT,   ss_state[SS_O_SECCNT   +:  8]);
	check("restore WPTR",     V_WPTR,     ss_state[SS_O_WPTR     +:  4]);
	check_b("restore SHIPINV", V_SHIPINV, ss_state[SS_O_SHIPINV]);
	// The queued response is the field that must survive: a driver that has
	// half-drained one is the case ss_idle deliberately does not wait out.
	check("restore RXLEN",    V_RXLEN,    ss_state[SS_O_RXLEN    +:  6]);
	check("restore RXOFF",    V_RXOFF,    ss_state[SS_O_RXOFF    +:  6]);
	same = 1'b1;
	for (i = 0; i < 32; i = i + 1)
		if (ss_state[SS_O_RESBUF + i*8 +: 8] !== (8'h80 + i[7:0])) same = 1'b0;
	check_b("restore result buffer in order", 1'b1, same);

	// -------------------------------------------------------------------
	// D. A restore lands idle even if the chip was not.
	//
	// The transients are not in the vector; the restore forces them to
	// their idle values instead. On a save taken under the idle rule that
	// writes back what was there anyway -- this checks the case where it
	// does not, which is the one that would otherwise wedge the drive.
	// -------------------------------------------------------------------
	@(negedge clk);
	u_dut.g_cd.sector_ready = 1'b1;
	u_dut.g_cd.pbx_busy     = 1'b1;
	u_dut.g_cd.sec_wr_ptr   = 12'd100;
	@(posedge clk);
	#1;
	check_b("idle low with the engines running", 1'b0, ss_idle);

	@(negedge clk);
	ss_ld_data = captured;
	ss_ld      = 1;
	@(posedge clk);
	@(negedge clk);
	ss_ld = 0;
	@(posedge clk);
	#1;
	check_b("restore leaves the engines idle", 1'b1, ss_idle);

	$display("== %0d checks, %0d failures", checks, errs);
	if (errs == 0) $display("RUN: PASS");
	else           $display("RUN: FAIL");
	$finish;
end

endmodule
