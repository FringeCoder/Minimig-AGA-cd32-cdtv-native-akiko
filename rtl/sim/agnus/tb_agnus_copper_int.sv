// Whole-Agnus integration: probe1's copper list, arbitrating against everything.
//
// The benches in rtl/sim/copper and rtl/sim/beamcounter each isolate one module
// and all of them pass, so the fault the vAmigaTS VPOS suite shows on hardware
// is not in any of them individually. This one puts the real agnus.v in, which
// is the first thing here that arbitrates the way the machine does: refresh,
// disk, audio, sprite, bitplane, copper and blitter all competing through the
// priority chain in agnus.v, with the CPU taking the bus whenever no DMA
// channel claims it.
//
// Refresh is the specific gap this closes. It takes the first slots of every
// line ahead of everything else, and no bench modelled it.
//
// The copper list is probe1's, exactly: sixteen WAITs at $E051 $E253 ... $FE6F,
// each followed by MOVE INTREQ,$8004. On hardware fifteen of the sixteen writes
// never arrive. Every one of them must land here, on its own line, and become a
// level 1 request that paula_intcontroller presents to the CPU.
//
// Wiring follows minimig.v and gary.v rather than being invented:
//   custom_data_in = dbr ? ram_data_out : cpu_data_out        (gary.v:141)
// so Agnus reads chip RAM on its own DMA slots and the CPU's data at all other
// times, and Paula sits on reg_address_out with that same bus, as minimig.v
// wires it.
//
// COMPILE ORDER MATTERS. This file must come first. agnus_bitplanedma.v has no
// `timescale of its own and writes every register with <= #1; compiled after a
// file that sets one it inherits the default instead, its non-blocking
// assignments land outside the run, and every register write to it silently
// does nothing while it sits there looking like configured hardware. The
// bitplane and refresh cycle counters below exist so that cannot pass quietly.
//
//   iverilog -g2012 -o tb tb_agnus_copper_int.sv ../../agnus*.v ../../paula_intcontroller.v && vvp tb

`timescale 1ns/1ps

module tb_agnus_copper_int;

	localparam [8:1] A_DIWSTRT = 8'h47;   // 9'h08e >> 1
	localparam [8:1] A_DIWSTOP = 8'h48;   // 9'h090 >> 1
	localparam [8:1] A_DDFSTRT = 8'h49;   // 9'h092 >> 1
	localparam [8:1] A_DDFSTOP = 8'h4A;   // 9'h094 >> 1
	localparam [8:1] A_DMACON  = 8'h4B;   // 9'h096 >> 1
	localparam [8:1] A_INTENA  = 8'h4D;   // 9'h09a >> 1
	localparam [8:1] A_INTREQ  = 8'h4E;   // 9'h09c >> 1
	localparam [8:1] A_COP1LCH = 8'h40;   // 9'h080 >> 1
	localparam [8:1] A_COP1LCL = 8'h41;   // 9'h082 >> 1
	localparam [8:1] A_COPJMP1 = 8'h44;   // 9'h088 >> 1
	localparam [8:1] A_BPLCON0 = 8'h80;   // 9'h100 >> 1

	reg clk = 0, clk7_en = 0, cck = 0, reset = 1;

	// CPU side of the register bus
	reg        aen = 0, hwr = 0, lwr = 0, rd = 0;
	reg [8:1]  address_in = 8'hFF;
	reg [15:0] cpu_data = 0;

	wire [15:0] ag_data_out;
	wire [20:1] address_out;
	wire [8:1]  reg_address_out;
	wire        dbr, dbwe, cpu_custom;
	wire [10:0] vpos;
	wire [8:0]  hpos;
	wire        sof, vbl_int, blit_busy, int3;

	integer errors = 0;
	integer i;

	always #1 clk = ~clk;

	reg [1:0] phase = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end
	always @(posedge clk) if (clk7_en) cck <= ~cck;

	// ---- chip RAM ----------------------------------------------------------
	// Word addressed. Only the copper list matters; everything else the DMA
	// channels fetch reads as $FFFF, which is harmless -- bitplane and sprite
	// data goes to Denise, which is not in this bench.
	reg [15:0] cmem [0:2047];
	wire [15:0] ram_data_out = cmem[address_out[11:1]];

	// gary.v:141
	wire [15:0] custom_data_in = dbr ? ram_data_out : cpu_data;

	agnus ag (
		.clk(clk), .clk7_en(clk7_en), .cck(cck), .reset(reset),
		.aen(aen), .rd(rd), .hwr(hwr), .lwr(lwr),
		.data_in(custom_data_in), .data_out(ag_data_out), .address_in(address_in),
		.ss_replay_we(1'b0), .ss_replay_addr(8'h00),
		.address_out(address_out), .reg_address_out(reg_address_out),
		.cpu_custom(cpu_custom), .dbr(dbr), .dbwe(dbwe),
		._hsync(), ._vsync(), ._csync(), .field1(), .lace(),
		.hblank(), .vblank(), .hde(), .sol(), .sof(sof),
		.vbl_int(vbl_int), .strhor_denise(), .strhor_paula(),
		.htotal(), .harddis(), .varbeamen(),
		.int3(int3), .blit_busy(blit_busy),
		.audio_dmal(4'b0000), .audio_dmas(4'b0000),
		.disk_dmal(1'b0), .disk_dmas(1'b0), .bls(1'b0),
		.ntsc(1'b0), .a1k(1'b0), .ecs(1'b1), .aga(1'b1),
		.floppy_speed(1'b0),
		.lpen_vpos(11'h7FF), .lpen_hpos(9'd0),
		.ss_vpos_out(vpos), .ss_hpos_out(hpos)
	);

	// ---- Paula's interrupt controller --------------------------------------
	// minimig.v gives Paula the same reg_address/custom_data_in bus Agnus
	// drives, so a copper MOVE to INTREQ reaches it exactly as it does here.
	// The emulated handler clears through a mux on Paula's own inputs so
	// servicing never disturbs the machine's bus.
	reg         int_clear = 0;
	wire [8:1]  pic_addr = int_clear ? A_INTREQ : reg_address_out;
	wire [15:0] pic_data = int_clear ? 16'h0004 : custom_data_in;
	wire [2:0]  ipl;

	paula_intcontroller pic (
		.clk(clk), .clk7_en(clk7_en), .reset(reset),
		.reg_address_in(pic_addr), .data_in(pic_data), .data_out(),
		.rxint(1'b0), .txint(1'b0), .vblint(1'b0),
		.int2(1'b0), .int3(1'b0), .int6(1'b0),
		.blckint(1'b0), .syncint(1'b0), .audint(4'b0000),
		.audpen(), .rbfmirror(), ._ipl(ipl),
		.ss_intreq(), .ss_intena()
	);

	localparam [2:0] IPL_LEVEL1 = 3'd6;

	// ---- power-up state ----------------------------------------------------
	// Same pokes the standalone benches need: neither the beam counter nor the
	// bitplane sequencer resets all of its state, and X on those propagates
	// into the arbitration.
	initial begin
		ag.bc1.vpos        = 11'd0;
		ag.bc1.hpos        = 9'd0;
		ag.bc1.end_of_line = 1'b0;
		ag.bc1.vpos_inc    = 1'b0;
		ag.bc1.long_line   = 1'b0;
		ag.bc1.long_frame  = 1'b0;
		ag.bc1.extra_line  = 1'b0;
		ag.bc1.vser        = 1'b0;
		ag.bc1.hblank      = 1'b0;
		ag.bc1.vblank      = 1'b0;
		ag.bc1.vbl_int     = 1'b0;
		ag.bc1._hsync      = 1'b1;
		ag.bc1._vsync      = 1'b1;

		ag.bpd1.ddfrun          = 1'b0;
		ag.bpd1.ddfseq          = 5'd0;
		ag.bpd1.plane           = 5'd0;
		ag.bpd1.ddfena          = 1'b0;
		ag.bpd1.ddfena_0        = 1'b0;
		ag.bpd1.hardena         = 1'b0;
		ag.bpd1.softena         = 1'b0;
		ag.bpd1.bplcon0         = 6'd0;
		ag.bpd1.bplcon0_delayed = 6'd0;
		ag.bpd1.dmaena_delayed  = 2'b00;
	end

	// ---- contention counters ------------------------------------------------
	// A pass means nothing unless the channels this bench exists to include
	// were actually running.
	integer bpl_cycles = 0;
	integer ref_cycles = 0;
	always @(posedge clk) if (clk7_en) begin
		if (ag.dma_bpl) bpl_cycles = bpl_cycles + 1;
		if (ag.dma_ref) ref_cycles = ref_cycles + 1;
	end

	// ---- observing the copper's INTREQ writes -------------------------------
	integer move_count = 0;
	integer move_line [0:31];

	always @(posedge clk) if (clk7_en) begin
		if (dbr && reg_address_out == A_INTREQ && move_count < 32) begin
			move_line[move_count] = vpos;
			move_count            = move_count + 1;
		end
	end

	// ---- the emulated handler ----------------------------------------------
	integer int_raises = 0;
	integer int_line [0:31];
	reg     watch_int = 0;
	reg     ipl_latched = 0;

	always @(posedge clk) if (clk7_en) begin
		if (int_clear)
			int_clear <= 1'b0;
		else if (watch_int && ipl == IPL_LEVEL1 && !ipl_latched && int_raises < 32) begin
			int_line[int_raises] <= vpos;
			int_raises           <= int_raises + 1;
			int_clear            <= 1'b1;
			ipl_latched          <= 1'b1;
		end
		else if (ipl != IPL_LEVEL1)
			ipl_latched <= 1'b0;
	end

	// ---- CPU register writes -------------------------------------------------
	// Agnus only passes the CPU's address to reg_address_out on cycles no DMA
	// channel claims, so a write is held until it is observed on the bus rather
	// than for a fixed number of cycles. COPJMP1 is a strobe: holding it a fixed
	// time would restart the copper repeatedly.
	task cpu_wr(input [8:1] a, input [15:0] d);
		integer guard;
		begin
			@(posedge clk); while (!clk7_en) @(posedge clk);
			address_in = a; cpu_data = d; aen = 1'b1; hwr = 1'b1; lwr = 1'b1;
			guard = 0;
			while (guard < 2000 && !(clk7_en && reg_address_out == a)) begin
				@(posedge clk);
				guard = guard + 1;
			end
			if (guard >= 2000) begin
				$display("FAIL: register write %02x never reached the bus", a);
				errors = errors + 1;
			end
			@(posedge clk); while (!clk7_en) @(posedge clk);
			aen = 1'b0; hwr = 1'b0; lwr = 1'b0;
			address_in = 8'hFF;
			@(posedge clk);
		end
	endtask

	task wait_lines(input integer n);
		integer k;
		begin
			for (k = 0; k < n; k = k + 1) @(negedge ag.bc1.vpos_inc);
		end
	endtask

	localparam [20:1] LIST_BASE = 20'h00200;

	initial begin
		for (i = 0; i < 2048; i = i + 1) cmem[i] = 16'hFFFF;

		repeat (40) @(posedge clk);
		reset = 0;
		repeat (20) @(posedge clk);

		// probe.i's display setup, then probe1's own BPLCON0.
		cpu_wr(A_DDFSTRT, 16'h0038);
		cpu_wr(A_DDFSTOP, 16'h00D0);
		cpu_wr(A_DIWSTRT, 16'h2C81);
		cpu_wr(A_DIWSTOP, 16'hF4C1);
		cpu_wr(A_BPLCON0, 16'h2200);        // BPU = 2
		cpu_wr(A_INTENA,  16'hC004);        // SET, master, SOFT

		// probe.i:66-69 -- copper DMA, bitplane DMA, DMAEN, blitter priority.
		cpu_wr(A_DMACON, 16'h8080);
		cpu_wr(A_DMACON, 16'h8100);
		cpu_wr(A_DMACON, 16'h8200);
		cpu_wr(A_DMACON, 16'h8400);

		// probe1's list.
		for (i = 0; i < 16; i = i + 1) begin
			cmem[LIST_BASE[11:1] + i*4 + 0] = {8'hE0 + i[7:0]*8'd2, 8'h51 + i[7:0]*8'd2};
			cmem[LIST_BASE[11:1] + i*4 + 1] = 16'hFFFE;
			cmem[LIST_BASE[11:1] + i*4 + 2] = {8'h00, A_INTREQ} << 1;
			cmem[LIST_BASE[11:1] + i*4 + 3] = 16'h8004;
		end
		cmem[LIST_BASE[11:1] + 64] = 16'hFFFF;   // park to end of frame
		cmem[LIST_BASE[11:1] + 65] = 16'hFFFE;

		cpu_wr(A_COP1LCH, {11'd0, LIST_BASE[20:16]});
		cpu_wr(A_COP1LCL, {LIST_BASE[15:1], 1'b0});

		watch_int  = 1'b1;
		move_count = 0;
		int_raises = 0;
		bpl_cycles = 0;
		ref_cycles = 0;
		cpu_wr(A_COPJMP1, 16'h0000);

		wait_lines(320);
		watch_int = 1'b0;

		// The channels that make this bench different from the isolated ones.
		if (ref_cycles < 100) begin
			$display("FAIL: refresh took %0d cycles -- not arbitrating", ref_cycles);
			errors = errors + 1;
		end
		else
			$display("ok:   refresh took %0d cycles over the frame", ref_cycles);

		if (bpl_cycles < 1000) begin
			$display("FAIL: bitplane DMA took %0d cycles -- no real contention", bpl_cycles);
			errors = errors + 1;
		end
		else
			$display("ok:   bitplane DMA took %0d cycles over the frame", bpl_cycles);

		if (move_count !== 16) begin
			$display("FAIL: %0d of 16 INTREQ writes reached the bus", move_count);
			errors = errors + 1;
		end
		else
			$display("ok:   all sixteen INTREQ writes reached the bus");

		for (i = 0; i < move_count && i < 16; i = i + 1) begin
			if (move_line[i] < 224 + i*2 || move_line[i] > 225 + i*2) begin
				$display("FAIL: INTREQ write %0d on line %0d, want %0d",
				         i, move_line[i], 224 + i*2);
				errors = errors + 1;
			end
		end

		if (int_raises !== 16) begin
			$display("FAIL: %0d of 16 became a level 1 request", int_raises);
			errors = errors + 1;
		end
		else
			$display("ok:   all sixteen raised level 1");

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#900000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
