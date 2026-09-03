// agnus_copper: does a WAIT fire on the line it names?
//
// There was no copper bench at all. rtl/sim/ has ten directories and none of
// them covers the one part of Agnus that every Amiga program steers the display
// with, so a WAIT that fires on the wrong line -- or once and then never again --
// passes CI in full.
//
// Written 2026-09-03, running the vAmigaTS VPOS suite on hardware for T7.
// probe1's copper list is sixteen WAITs, one every second line from $E0 to $FE,
// each followed by a MOVE to INTREQ that makes the handler sample VHPOSR.
// Fifteen of the sixteen slots came back empty. probe2's list is a single WAIT
// at $E881 and sixteen back-to-back MOVEs, and all sixteen of those landed. The
// difference between those two lists is what this bench reproduces.
//
// It drives the real agnus_beamcounter into the real agnus_copper, wired as
// agnus.v wires them -- hpos_slot included -- rather than modelling the beam
// grid here, because the grid is the thing most likely to be modelled wrongly
// and a bench that gets it wrong tests nothing.
//
// A note on what the hardware numbers can and cannot say: VHPOSR reports
// vpos[7:0], so the $31 those tests returned is line 49 OR line 305. Nothing
// here depends on which -- this bench asks the copper directly which line it
// fired on, using the full counter.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../agnus_beamcounter.v ../../agnus_copper.v tb_copper_wait.sv && vvp tb

`timescale 1ns/1ps

module tb_copper_wait;

	localparam [8:1] A_COP1LCH = 8'h40;   // 9'h080 >> 1
	localparam [8:1] A_COP1LCL = 8'h41;   // 9'h082 >> 1
	localparam [8:1] A_COPJMP1 = 8'h44;   // 9'h088 >> 1
	localparam [8:1] A_COPINS  = 8'h46;   // 9'h08c >> 1
	localparam [8:1] A_COLOR00 = 8'h90;   // 9'h180 >> 1
	localparam [8:1] A_DIWSTRT = 8'h47;   // 9'h08e >> 1
	localparam [8:1] A_DIWSTOP = 8'h48;   // 9'h090 >> 1
	localparam [8:1] A_DDFSTRT = 8'h49;   // 9'h092 >> 1
	localparam [8:1] A_DDFSTOP = 8'h4A;   // 9'h094 >> 1
	localparam [8:1] A_BPLCON0 = 8'h80;   // 9'h100 >> 1

	reg         clk = 0;
	reg         clk7_en = 0;
	reg         reset = 1;
	reg         cck = 0;

	reg  [15:0] data_in = 0;
	reg  [8:1]  reg_address_in = 8'hFF;   // an address nothing decodes

	wire [15:0] bc_data_out;
	wire [8:0]  hpos;
	wire [10:0] vpos;
	wire        eof;
	wire [8:0]  htotal;

	wire        reqdma;
	wire [8:1]  reg_address_cop;
	wire [20:1] address_cop;

	integer errors = 0;
	integer i;

	always #1 clk = ~clk;

	reg [1:0] phase = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end
	always @(posedge clk) if (clk7_en) cck <= ~cck;

	agnus_beamcounter bc (
		.clk(clk), .clk7_en(clk7_en), .reset(reset), .cck(cck),
		.ntsc(1'b0), .aga(1'b1), .ecs(1'b1), .a1k(1'b0),
		.data_in(data_in), .data_out(bc_data_out), .reg_address_in(reg_address_in),
		.lpen_vpos(11'h7FF), .lpen_hpos(9'd0),
		.hpos(hpos), .vpos(vpos),
		._hsync(), ._vsync(), .field1(), .lace(), ._csync(),
		.hblank(), .vblank(), .vbl(), .vblend(),
		.eol(), .eof(eof), .vbl_int(),
		.htotal_out(htotal), .harddis_out(), .varbeamen_out()
	);

	// agnus.v:483-484 verbatim. The copper's slot grid runs one colour clock
	// ahead of hpos and wraps on htotal, and getting this wrong moves every
	// WAIT by a cycle.
	wire [7:0] hpos_slot_hi = (hpos[8:1] == htotal[8:1]) ? 8'd0 : hpos[8:1] + 8'd1;
	wire [8:0] hpos_slot    = {hpos_slot_hi, hpos[0]};

	// Bitplane DMA, the copper's competition. agnus.v gives it priority over the
	// copper outright -- ena_cop = ~dma_bpl, and the arbiter's if-else chain
	// reaches dma_cop only when dma_bpl has not claimed the cycle. Held off for
	// the first two cases so a WAIT is tested without contention first, then
	// enabled for the third.
	reg  bpl_dmaena = 0;
	wire dma_bpl;

	agnus_bitplanedma bpd (
		.clk(clk), .clk7_en(clk7_en), .reset(reset),
		.harddis(1'b0), .aga(1'b1), .ecs(1'b1), .a1k(1'b0),
		.sof(eof), .dmaena(bpl_dmaena),
		.vpos(vpos), .hpos(hpos), .hpos_slot(hpos_slot),
		.hde(), .dma(dma_bpl),
		.reg_address_in(reg_address_in), .reg_address_out(),
		.data_in(data_in), .address_out()
	);

	// Sprites, disk, audio and the blitter are not modelled: none of the
	// vAmigaTS programs in scope here runs them. Refresh is the one remaining
	// gap in this arbitration -- it takes the first slots of every line, ahead
	// of everything, and is not modelled.
	wire ena_cop = ~dma_bpl;
	wire ack_cop = reqdma & ena_cop;

	agnus_copper cp (
		.clk(clk), .clk7_en(clk7_en), .reset(reset), .ecs(1'b1),
		.reqdma(reqdma), .ackdma(ack_cop), .enadma(ena_cop),
		.sof(eof), .blit_busy(1'b0),
		.vpos(vpos[7:0]), .hpos(hpos), .hpos_slot(hpos_slot),
		.data_in(data_in), .reg_address_in(reg_address_in),
		.reg_address_out(reg_address_cop), .address_out(address_cop)
	);

	initial begin
		bc.vpos        = 11'd0;
		bc.hpos        = 9'd0;
		bc.end_of_line = 1'b0;
		bc.vpos_inc    = 1'b0;
		bc.long_line   = 1'b0;
		bc.long_frame  = 1'b0;
		bc.extra_line  = 1'b0;
		bc.vser        = 1'b0;
		bc.hblank      = 1'b0;
		bc.vblank      = 1'b0;
		bc.vbl_int     = 1'b0;
		bc._hsync      = 1'b1;
		bc._vsync      = 1'b1;

		// agnus_bitplanedma has no reset on the state that feeds its `dma`
		// output, so at time zero dma_bpl is X -- and X on ena_cop stops the
		// copper dead whether or not bitplane DMA is meant to be running. That
		// is a harness artefact, not the DUT: on real hardware these come up
		// from power-on with defined levels. Poked to the same zeros the reset
		// path would leave, exactly as the beamcounter block above does.
		bpd.ddfrun         = 1'b0;
		bpd.ddfseq         = 5'd0;
		bpd.plane          = 5'd0;
		bpd.ddfena         = 1'b0;
		bpd.ddfena_0       = 1'b0;
		bpd.hardena        = 1'b0;
		bpd.softena        = 1'b0;
		bpd.bplcon0        = 6'd0;
		bpd.bplcon0_delayed = 6'd0;
		bpd.dmaena_delayed = 2'b00;
		bpd.ddfstrt        = 7'd0;
		bpd.ddfstop        = 7'd0;
		bpd.hde            = 1'b0;
	end

	// ---- the copper list ---------------------------------------------------
	// Word-addressed, matching address_out[20:1]. Base is arbitrary; only the
	// low bits are decoded here.
	localparam [20:1] LIST_BASE = 20'h01000;
	reg [15:0] cmem [0:255];

	task put(input integer idx, input [15:0] w);
		begin cmem[idx] = w; end
	endtask

	// ---- the fetch path ----------------------------------------------------
	// In the real machine the copper drives address_out and COPINS on its
	// granted slot, and the chip bus writes the fetched word back to COPINS.
	// Reproduced here: on a granted cycle, latch the address, then present the
	// word as a COPINS write on the following clk7_en.
	reg        fetch_pending = 0;
	reg [20:1] fetch_addr = 0;
	reg        feeding = 0;

	always @(posedge clk) if (clk7_en) begin
		feeding <= 1'b0;
		if (fetch_pending) begin
			reg_address_in <= A_COPINS;
			data_in        <= cmem[fetch_addr[8:1]];
			fetch_pending  <= 1'b0;
			feeding        <= 1'b1;
		end
		else if (!driving_host) begin
			// host writes hold the bus themselves; otherwise idle it
			reg_address_in <= 8'hFF;
		end

		if (cp.dma_ack && cp.selins && !fetch_pending) begin
			fetch_addr    <= address_cop;
			fetch_pending <= 1'b1;
		end
	end

	// ---- observing MOVEs ---------------------------------------------------
	// selreg without selins is a MOVE putting a chip register address on the
	// bus. Record the beam position of each one.
	integer  move_count = 0;
	integer  move_line  [0:31];
	integer  move_cck   [0:31];

	always @(posedge clk) if (clk7_en) begin
		if (cp.dma_ack && cp.selreg && !cp.selins && move_count < 32) begin
			move_line[move_count] = vpos;
			move_cck[move_count]  = hpos[8:1];
			move_count            = move_count + 1;
		end
	end

	// ---- host register writes ----------------------------------------------
	reg driving_host = 0;

	task host_wr(input [8:1] a, input [15:0] d);
		begin
			@(posedge clk); while (!clk7_en) @(posedge clk);
			driving_host   = 1'b1;
			reg_address_in = a;
			data_in        = d;
			@(posedge clk); while (!clk7_en) @(posedge clk);
			@(posedge clk); while (!clk7_en) @(posedge clk);
			driving_host   = 1'b0;
			reg_address_in = 8'hFF;
			@(posedge clk);
		end
	endtask

	task start_copper;
		begin
			host_wr(A_COP1LCH, {11'd0, LIST_BASE[20:16]});
			host_wr(A_COP1LCL, {LIST_BASE[15:1], 1'b0});
			host_wr(A_COPJMP1, 16'h0000);
		end
	endtask

	task wait_lines(input integer n);
		integer k;
		begin
			for (k = 0; k < n; k = k + 1) begin
				@(negedge bc.vpos_inc);
			end
		end
	endtask

	initial begin
		for (i = 0; i < 256; i = i + 1) cmem[i] = 16'hFFFE;

		repeat (40) @(posedge clk);
		reset = 0;
		repeat (20) @(posedge clk);

		// ---- 1. probe2's shape: one WAIT, then MOVEs --------------------------
		// $E881 = WAIT, VP $E8 (232), HP $80. On hardware this list delivered
		// all sixteen of its writes.
		put(0, 16'hE881); put(1, 16'hFFFE);          // WAIT 232, $80
		put(2, {8'h00, A_COLOR00[8:1]} << 1);        // MOVE COLOR00
		put(3, 16'h0F00);
		put(4, 16'hFFFF); put(5, 16'hFFFE);          // park until end of frame

		move_count = 0;
		start_copper;
		wait_lines(300);

		if (move_count < 1) begin
			$display("FAIL: single WAIT $E881 never released -- no MOVE executed");
			errors = errors + 1;
		end
		else if (move_line[0] !== 232) begin
			$display("FAIL: WAIT $E881 released on line %0d, want 232", move_line[0]);
			errors = errors + 1;
		end
		else begin
			$display("ok:   WAIT $E881 released on line 232 (cck %0d)", move_cck[0]);
		end

		// ---- 2. probe1's shape: sixteen WAITs, one every second line ---------
		// $E051 $E253 $E455 ... $FE6F, each followed by a MOVE. This is the list
		// that lost fifteen of its sixteen writes on hardware. Every WAIT must
		// release on its own line: a copper that stops after the first, or that
		// runs them all together once the beam is past the last one, fails here
		// and passes every other bench in this repository.
		for (i = 0; i < 256; i = i + 1) cmem[i] = 16'hFFFE;
		for (i = 0; i < 16; i = i + 1) begin
			// VP = $E0 + 2i, HP = $51 + 2i, exactly as probe1.s writes them.
			put(i*4 + 0, {8'hE0 + i[7:0]*8'd2, 8'h51 + i[7:0]*8'd2});
			put(i*4 + 1, 16'hFFFE);
			put(i*4 + 2, {8'h00, A_COLOR00[8:1]} << 1);
			put(i*4 + 3, 16'h0F00);
		end
		put(64, 16'hFFFF); put(65, 16'hFFFE);        // park until end of frame

		move_count = 0;
		start_copper;
		wait_lines(320);

		if (move_count !== 16) begin
			$display("FAIL: probe1 list executed %0d of 16 MOVEs", move_count);
			errors = errors + 1;
		end
		for (i = 0; i < move_count && i < 16; i = i + 1) begin
			if (move_line[i] !== 224 + i*2) begin
				$display("FAIL: probe1 WAIT %0d released on line %0d, want %0d",
				         i, move_line[i], 224 + i*2);
				errors = errors + 1;
			end
		end
		if (move_count == 16 && errors == 0)
			$display("ok:   probe1's sixteen WAITs released on lines 224..254");

		// ---- 3. the same list, with bitplane DMA competing --------------------
		// probe.i sets DDFSTRT $38, DDFSTOP $D0, DIWSTRT $2C81, DIWSTOP $F4C1,
		// BPL1MOD 0, and enables bitplane DMA; probe1's copper then raises
		// BPLCON0 to $2200, which is BPU=2. So every one of these WAITs releases
		// into a line that is fetching two bitplanes, and the copper has to take
		// its slot around them. Case 2 above grants the copper everything and is
		// the control for this one.
		//
		// DIWSTOP $F4C1 puts the display window's last line at 244, so waits at
		// 224..244 land inside the fetch and 246..254 land past it. If only the
		// contended ones misbehave, that split will show in which indices fail.
		host_wr(A_DDFSTRT, 16'h0038);
		host_wr(A_DDFSTOP, 16'h00D0);
		host_wr(A_DIWSTRT, 16'h2C81);
		host_wr(A_DIWSTOP, 16'hF4C1);
		host_wr(A_BPLCON0, 16'h2200);   // BPU = 2, as probe1's copper sets it
		bpl_dmaena = 1'b1;

		move_count = 0;
		start_copper;
		wait_lines(320);

		if (move_count !== 16) begin
			$display("FAIL: with bitplane DMA, probe1 list executed %0d of 16 MOVEs",
			         move_count);
			errors = errors + 1;
		end
		for (i = 0; i < move_count && i < 16; i = i + 1) begin
			if (move_line[i] !== 224 + i*2) begin
				$display("FAIL: with bitplane DMA, WAIT %0d released on line %0d, want %0d",
				         i, move_line[i], 224 + i*2);
				errors = errors + 1;
			end
		end
		if (move_count == 16)
			$display("ok:   under bitplane DMA the sixteen WAITs still release on 224..254");

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#600000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
