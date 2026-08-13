`timescale 1ns/1ns

// hps_ext.v's save state diagnostic sub-channel (UIO class 0xF600).
//
// This bench exists because reading the source has already produced three wrong
// answers about this area, and the one assumption the new channel rests on --
// which byte_cnt a given userspace word lands on -- is not visible anywhere in
// the source. It is a convention, established by the akiko and cdtv bridges,
// that `spi8(cmd); spi32_w(addr);` consumes byte_cnt 0, 1 and 2, so the first
// payload word of a read is byte_cnt 3. Everything in minimig_ssdiag.cpp is
// built on that, so it is checked here against the real decode rather than
// against the comment that asserts it.
//
// What this can and cannot settle. It drives EXT_BUS exactly as sys/hps_io.sv
// does (io_din on [31:16], strobe on [33], io_uio on [34], and the DUT's
// io_dout coming back on [15:0] with dout_en on [32]), so it settles the whole
// of hps_ext's behaviour: the class decode, the word order, dout_en, and the
// fact that a diagnostic read pulses no other bridge's read strobe. It cannot
// settle whether the HPS-side transaction returns the io_dout written on strobe
// N or on strobe N-1, because that lives in the bridge below hps_io and not in
// this module. That is why word 0 of the window is a signature and why the
// poller searches for it instead of assuming an offset.

module tb_hps_ext_ssdiag;

reg clk_sys = 0;
always #5 clk_sys = ~clk_sys;

// --- the HPS side of EXT_BUS -------------------------------------------------
reg  [15:0] hps_dout   = 16'd0;
reg         hps_strobe = 1'b0;
reg         hps_uio    = 1'b0;
reg         hps_fpga   = 1'b0;

wire [35:0] EXT_BUS;
assign EXT_BUS[31:16] = hps_dout;
assign EXT_BUS[33]    = hps_strobe;
assign EXT_BUS[34]    = hps_uio;
assign EXT_BUS[35]    = hps_fpga;

wire [15:0] fpga_dout_bus = EXT_BUS[15:0];
wire        dout_en       = EXT_BUS[32];

// --- DUT ports ---------------------------------------------------------------
//
// Everything hps_ext has, declared so the `.*` connection below is unambiguous
// about widths. The inputs the bench does not drive are tied off at their
// declaration; only ss_diag carries anything this bench cares about.
wire        io_strobe, io_fpga, io_uio;
wire [15:0] io_din;
wire [15:0] fpga_dout = 16'd0;

wire [15:0] ide_din = 16'hEEEE;
wire [15:0] ide_dout;
wire  [4:0] ide_addr;
wire        ide_rd, ide_wr;
wire  [5:0] ide_req = 6'd0;

wire  [2:0] mouse_buttons;
wire        kbd_mouse_level;
wire  [1:0] kbd_mouse_type;
wire  [7:0] kbd_mouse_data;

wire [11:0] scr_hbl_l = 12'd0, scr_hbl_r = 12'd0, scr_hsize = 12'd0;
wire [11:0] scr_vbl_t = 12'd0, scr_vbl_b = 12'd0, scr_vsize = 12'd0;
wire  [6:0] scr_flg   = 7'd0;
wire  [1:0] scr_res   = 2'd0;
wire [11:0] shbl_l, shbl_r, svbl_t, svbl_b;
wire        sset;

wire        cdda_req = 1'b0;
wire        cdda_wr;
wire [15:0] cdda_dout;

wire [15:0] akiko_din = 16'hAAAA;
wire [15:0] akiko_dout;
wire        akiko_wr, akiko_rd, akiko_cs, akiko_cs_sec, akiko_cs_nvr, akiko_cs_subcode;
wire        akiko_req = 1'b0, akiko_sec_req = 1'b0, akiko_rx_busy = 1'b0, akiko_nvr_dirty = 1'b0;

wire [15:0] cdtv_din = 16'hCCCC;
wire [15:0] cdtv_dout;
wire        cdtv_wr, cdtv_rd, cdtv_cs, cdtv_cs_sec, cdtv_cs_stch;
wire        cdtv_req = 1'b0;

// The diagnostic window, filled with a value per word that could not be
// confused with any other word, with any tied-off bridge input above, or with
// the all-zero pattern a core without this channel returns.
wire [127:0] ss_diag = { 16'h8877, 16'h7766, 16'h6655, 16'h5544,
                         16'h4433, 16'h3322, 16'h2211, 16'h1100 };

hps_ext dut (.*);

// --- bus helpers -------------------------------------------------------------

integer errors = 0;

task check(input [255:0] name, input [31:0] got, input [31:0] want);
begin
	if (got !== want) begin
		$display("FAIL %0s: got %08h, want %08h", name, got, want);
		errors = errors + 1;
	end
	else $display("PASS %0s", name);
end
endtask

// One 16-bit word of a UIO transaction. Exactly one clk_sys cycle of io_strobe,
// which is what hps_ext's `else if(io_strobe)` acts on.
task xfer(input [15:0] d);
begin
	@(negedge clk_sys); hps_dout = d; hps_strobe = 1'b1;
	@(negedge clk_sys); hps_strobe = 1'b0;
end
endtask

// Same, capturing what the DUT left on the bus for that strobe.
task xfer_rd(output [15:0] v);
begin
	xfer(16'd0);
	v = fpga_dout_bus;
end
endtask

// Any read strobe leaking onto another bridge would mean a diagnostic poll
// popped somebody's FIFO. Sticky for the whole run.
reg leaked = 1'b0;
always @(posedge clk_sys)
	if (ide_rd | ide_wr | akiko_rd | akiko_wr | cdtv_rd | cdtv_wr | cdda_wr) leaked <= 1'b1;

reg [15:0] w [0:8];
integer k;

initial begin
	// Watchdog: a bench that hangs must say so rather than time out silently.
	#20000;
	$display("FAIL watchdog: simulation did not finish");
	$fatal(1);
end

initial begin
	@(negedge clk_sys);
	hps_uio = 1'b1;

	// UIO_DMA_READ, then the 32-bit address as two words, exactly as
	// cdtv_sec_space_bytes() and akiko_read_sec_counter() send it.
	xfer(16'h0062);        // byte_cnt 0 -> cmd
	xfer(16'hF600);        // byte_cnt 1 -> class decode
	xfer(16'h0000);        // byte_cnt 2 -> address high half

	// dout_en is what lets hps_ext drive the bus back at all; without it
	// sys/hps_io.sv returns its own io_dout and every word below reads as
	// whatever hps_io last had, which is a far more confusing failure than
	// silence.
	check("dout_en asserted for 'h62", {31'd0, dout_en}, 32'd1);

	for (k = 0; k < 9; k = k + 1) xfer_rd(w[k]);

	// The signature comes first, on byte_cnt 3 -- the first word after the
	// address. This is the alignment the poller assumes.
	check("word 0 is the signature", {16'd0, w[0]}, 32'h000055AA);

	// Then the window, low word first, in ss_diag[15:0] .. ss_diag[127:112]
	// order. Word order is the whole contract with minimig_ssdiag.cpp: a
	// reversed window would decode into a plausible-looking wrong answer
	// rather than into nothing.
	check("word 1", {16'd0, w[1]}, 32'h00001100);
	check("word 2", {16'd0, w[2]}, 32'h00002211);
	check("word 3", {16'd0, w[3]}, 32'h00003322);
	check("word 4", {16'd0, w[4]}, 32'h00004433);
	check("word 5", {16'd0, w[5]}, 32'h00005544);
	check("word 6", {16'd0, w[6]}, 32'h00006655);
	check("word 7", {16'd0, w[7]}, 32'h00007766);
	check("word 8", {16'd0, w[8]}, 32'h00008877);

	// Reading past the window returns zero rather than repeating the last
	// word, so an over-long read is visibly over-long.
	xfer_rd(w[0]);
	check("past the end reads zero", {16'd0, w[0]}, 32'd0);

	// No chip select and no read strobe reached any other bridge. 0xF600 is
	// one bit away from akiko's 0xF400 class in io_din[15:9], so this is the
	// check that the class decode actually discriminates.
	check("akiko not selected", {31'd0, akiko_cs}, 32'd0);
	check("cdtv not selected",  {31'd0, cdtv_cs},  32'd0);
	check("no bridge strobed",  {31'd0, leaked},   32'd0);

	@(negedge clk_sys); hps_uio = 1'b0;
	@(negedge clk_sys);

	// A read on a class this module does not own must not answer with the
	// diagnostic window. 0xF400 is akiko's, and akiko_din is tied to 0xAAAA
	// here, so a decode that fell through to the diagnostic mux would return
	// the signature instead.
	hps_uio = 1'b1;
	xfer(16'h0062);
	xfer(16'hF400);
	xfer(16'h0000);
	xfer_rd(w[0]);
	check("akiko class still reads akiko", {16'd0, w[0]}, 32'h0000AAAA);

	@(negedge clk_sys); hps_uio = 1'b0;
	@(negedge clk_sys);

	if (errors) begin
		$display("%0d FAILURE(S)", errors);
		$fatal(1);
	end
	$display("RUN: PASS");
	$finish;
end

endmodule
