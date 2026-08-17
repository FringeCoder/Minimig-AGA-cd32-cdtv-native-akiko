`timescale 1ns/1ps

// The save state register bus, end to end, through the real minimig.v.
//
// minimig.v carries three pieces of save state wiring that nothing has ever
// simulated:
//
//   1. the snoop tap      -- ss_rga_addr/ss_rga_data, taken off the two buses
//                            every chipset module reads
//   2. the replay mux     -- reg_address and custom_data_in switched onto
//                            ss_replay_addr/ss_replay_data while ss_replay_we
//   3. the memory map     -- ss_map out, ss_map_in/ss_map_we back in
//
// The module-level benches next door (rtl/tb/ss_regshadow_tb.v and friends,
// in the Main_MiSTer repo) prove ss_regshadow emits the right sequence. What
// they cannot see is whether that sequence LANDS: they end at the module's
// pins. This bench instantiates the real minimig -- real agnus, real paula,
// real denise, real gary -- alongside a real ss_regshadow, wires them the way
// Minimig.sv does including both amiga_clk instances and the freeze, and then
// reads the chipset's own registers to see whether a replay arrived.
//
// The Amiga-side clock generator matters and is not a detail. Minimig.sv runs
// two amiga_clk instances: a master that always runs, and the Amiga's own,
// whose `ce` is dropped by the save state freeze. ss_regshadow is clocked by
// the master, minimig.v by the Amiga's. Every chipset register write in the
// machine is decoded under `if (clk7_en)`, so which of the two generators is
// running while the replay walks decides whether the replay writes anything at
// all. That relationship is reproduced here exactly.
//
// Peeks into the DUT hierarchy are deliberate: the registers being checked
// (Denise's BPLCON0, Agnus's DMACON, Paula's INTENA) are write-only in
// hardware and this core is faithful about that, so a bus read cannot see
// them. Reading the flip-flop is the only observation available, and it is the
// one that matters -- it is the state a restore is trying to install.

module tb_ss_regbus_mux;

integer errs = 0;

task expect_eq16(input [511:0] what, input [15:0] got, input [15:0] want);
begin
	if (got !== want) begin
		$display("FAIL %0s: got %04h want %04h", what, got, want);
		errs = errs + 1;
	end
	else $display("ok   %0s = %04h", what, got);
end
endtask

task expect_eq1(input [511:0] what, input got, input want);
begin
	if (got !== want) begin
		$display("FAIL %0s: got %b want %b", what, got, want);
		errs = errs + 1;
	end
	else $display("ok   %0s = %b", what, got);
end
endtask

// ------------------------------------------------------------------- clocks

reg clk_r = 1'b0;
always #17.618 clk_r = ~clk_r;      // 28.37516 MHz

reg rst_n = 1'b0;

// The master generator: Minimig.sv's first amiga_clk, ce tied high. It clocks
// ss_ctrl and ss_regshadow and never stops.
wire        m_clk7_en;
wire        m_clk7n_en;
wire        m_c1, m_c3, m_cck;
wire [9:0]  m_eclk;

amiga_clk master_clk
(
	.clk_28   (clk_r      ),
	.clk7_en  (m_clk7_en  ),
	.clk7n_en (m_clk7n_en ),
	.c1       (m_c1       ),
	.c3       (m_c3       ),
	.cck      (m_cck      ),
	.eclk     (m_eclk     ),
	.ce       (1'b1       ),
	.reset_n  (rst_n      )
);

// The freeze and the replay tick, from the same module Minimig.sv instantiates
// rather than a copy of its logic. That is the point of the module: the
// sampling phase decides whether a replay's ticks move the beam, and a bench
// carrying its own copy of the rule would agree with itself while the hardware
// drifted.
reg  ss_freeze = 1'b0;
wire ss_freeze_7m;

ss_freeze_phase freeze_phase
(
	.clk         (clk_r          ),
	.rst_n       (rst_n          ),
	.clk7_en     (m_clk7_en      ),
	.cck         (m_cck          ),
	.freeze      (ss_freeze      ),
	.replay_we   (ss_replay_we   ),
	.freeze_7m   (ss_freeze_7m   ),
	.replay_tick (ss_replay_tick )
);

// The Amiga's generator: held by the freeze. Its outputs are minimig.v's
// clock inputs, which the generated pin file leaves for us to drive.
amiga_clk amiga_clock
(
	.clk_28   (clk_r          ),
	.clk7_en  (clk7_en        ),
	.clk7n_en (clk7n_en       ),
	.c1       (c1             ),
	.c3       (c3             ),
	.cck      (cck            ),
	.eclk     (eclk           ),
	.ce       (~ss_freeze_7m  ),
	.reset_n  (rst_n          )
);

// ---------------------------------------------------------------------- DUT
//
// minimig_pins.vh declares every port, drives the ordinary inputs from `drv_`
// regs, and instantiates `dut`. Clock inputs and the replay bus are left bare
// for the wiring above and below.

assign clk = clk_r;

`include "minimig_pins.vh"

// ------------------------------------------------------------- the shadow
//
// Same instantiation as Minimig.sv: master clk7_en, tap in, replay out.

reg        sh_ld_we    = 1'b0;
reg  [7:0] sh_ld_addr  = 8'd0;
reg [15:0] sh_ld_data  = 16'd0;
reg        sh_start    = 1'b0;
reg [14:0] restored_intreq = 15'd0;

wire        sh_replay_active;
wire        sh_replay_done;
wire [15:0] sh_rd_data;
wire        sh_rd_writable;
wire        sh_rd_setclear;

ss_regshadow shadow
(
	.clk            (clk_r            ),
	.clk7_en        (m_clk7_en        ),
	.rst_n          (rst_n            ),
	.reg_address_in (ss_rga_addr      ),
	.data_in        (ss_rga_data      ),
	.rd_addr        (8'd0             ),
	.rd_data        (sh_rd_data       ),
	.rd_writable    (sh_rd_writable   ),
	.rd_setclear    (sh_rd_setclear   ),
	.ld_we          (sh_ld_we         ),
	.ld_addr        (sh_ld_addr       ),
	.ld_data        (sh_ld_data       ),
	.replay_start   (sh_start         ),
	// The RESTORED INTREQ, as Minimig.sv now wires it -- ss_state_fanout's
	// unpacked field, not Paula's live output. Driven by the test so the two
	// can be told apart.
	.intreq_in      (restored_intreq  ),
	.replay_active  (sh_replay_active ),
	.replay_we      (ss_replay_we     ),
	.replay_addr    (ss_replay_addr   ),
	.replay_data    (ss_replay_data   ),
	.replay_done    (sh_replay_done   )
);

// ------------------------------------------------------- continuous checkers
//
// These run for the whole simulation, so they cover the ordinary running
// machine as well as the replay.

integer tap_errs    = 0;
integer mux_errs    = 0;
integer excl_errs   = 0;
reg     checks_live = 1'b0;

always @(posedge clk_r) if (checks_live) begin
	// The tap is taken after the mux, so it always carries whatever the
	// chipset is actually seeing.
	if (ss_rga_addr !== dut.reg_address || ss_rga_data !== dut.custom_data_in) begin
		if (tap_errs == 0)
			$display("FAIL tap: rga %02h/%04h but bus %02h/%04h at %0t",
			         ss_rga_addr, ss_rga_data, dut.reg_address,
			         dut.custom_data_in, $time);
		tap_errs = tap_errs + 1;
	end

	if (ss_replay_we) begin
		// Replay drives both buses.
		if (dut.reg_address !== ss_replay_addr ||
		    dut.custom_data_in !== ss_replay_data) begin
			if (mux_errs == 0)
				$display("FAIL mux: replay %02h/%04h but bus %02h/%04h at %0t",
				         ss_replay_addr, ss_replay_data, dut.reg_address,
				         dut.custom_data_in, $time);
			mux_errs = mux_errs + 1;
		end
		// Nothing below $020 may ever be driven: that is the read window, and
		// a write there would be decoded as a read cycle's address.
		if ({ss_replay_addr, 1'b0} < 9'h020) begin
			if (excl_errs == 0)
				$display("FAIL excluded: replay drove $%03h at %0t",
				         {ss_replay_addr, 1'b0}, $time);
			excl_errs = excl_errs + 1;
		end
	end
	else begin
		// Idle: straight through from agnus and gary.
		if (dut.reg_address !== dut.reg_address_agnus ||
		    dut.custom_data_in !== dut.custom_data_in_gary) begin
			if (mux_errs == 0)
				$display("FAIL passthrough: bus %02h/%04h but agnus/gary %02h/%04h at %0t",
				         dut.reg_address, dut.custom_data_in,
				         dut.reg_address_agnus, dut.custom_data_in_gary, $time);
			mux_errs = mux_errs + 1;
		end
	end
end

// ------------------------------------------------------------------- helpers

localparam [7:0] IDX_BPLCON0 = 8'h80;   // $100 >> 1, plain, lands in denise
localparam [7:0] IDX_DMACON  = 8'h4B;   // $096 >> 1, set/clear, agnus
localparam [7:0] IDX_INTENA  = 8'h4D;   // $09A >> 1, set/clear, paula
localparam [7:0] IDX_VHPOSW  = 8'h16;   // $02C >> 1, plain, agnus beamcounter

task shadow_load(input [7:0] idx, input [15:0] val);
begin
	@(posedge clk_r);
	sh_ld_we   <= 1'b1;
	sh_ld_addr <= idx;
	sh_ld_data <= val;
	@(posedge clk_r);
	sh_ld_we   <= 1'b0;
end
endtask

// Drive replay_start until replay_done, the way ss_ctrl's S_L_REPLAY does.
task run_replay;
integer guard;
begin
	guard = 0;
	@(posedge clk_r);
	sh_start <= 1'b1;
	while (!sh_replay_done && guard < 200000) begin
		@(posedge clk_r);
		guard = guard + 1;
	end
	sh_start <= 1'b0;
	if (guard >= 200000) begin
		$display("FAIL replay never completed");
		errs = errs + 1;
	end
	@(posedge clk_r);
end
endtask

task freeze_machine;
begin
	ss_freeze = 1'b1;
	// Wait for the sampled copy, then for the generator to have actually
	// stopped -- ss_freeze_7m only moves on a master clk7_en.
	while (!ss_freeze_7m) @(posedge clk_r);
	repeat (8) @(posedge clk_r);
end
endtask

task thaw_machine;
begin
	ss_freeze = 1'b0;
	while (ss_freeze_7m) @(posedge clk_r);
	repeat (8) @(posedge clk_r);
end
endtask

// --------------------------------------------------------------------- test

integer i;

initial begin
	$dumpfile("tb_ss_regbus_mux.vcd");
	$dumpvars(1, tb_ss_regbus_mux);

	pins_reset_inputs();

	// A 68000 that is present but idle: strobes released, bus in read.
	drv__cpu_as      = 1'b1;
	drv__cpu_uds     = 1'b1;
	drv__cpu_lds     = 1'b1;
	drv_cpu_r_w      = 1'b1;
	drv__cpu_reset_in= 1'b1;
	drv_rst_ext      = 1'b1;

	// userio's OSD control bits are written only by an OSD command, so in
	// simulation they start as X and the X on usrrst reaches
	// minimig_syscontrol's mrst, which then holds the whole machine in reset
	// forever. On hardware Quartus powers these up at 0. Do the same.
	dut.USERIO1.usrrst = 1'b0;
	dut.USERIO1.cpurst = 1'b0;
	dut.USERIO1.cpuhlt = 1'b0;

	repeat (40) @(posedge clk_r);
	rst_n = 1'b1;
	repeat (40) @(posedge clk_r);
	drv_rst_ext = 1'b0;
	repeat (20) @(posedge clk_r);   // let mrst settle before the poke below

	// minimig_syscontrol counts four start-of-frame pulses before releasing
	// the system reset, which is four Amiga frames of simulation for a bench
	// that never needs a frame. Preloading the counter is exactly equivalent
	// and costs 80 ms of simulated time less. It has to happen with mrst
	// already low for a clock: mrst high schedules `rst_cnt <= 0`, and a
	// non-blocking assignment scheduled in the same timestep lands after the
	// poke and undoes it.
	dut.CONTROL1.rst_cnt = 3'b100;
	while (dut.sys_reset) @(posedge clk_r);
	repeat (20) @(posedge clk_r);

	checks_live = 1'b1;
	repeat (200) @(posedge clk_r);

	$display("--- machine idle, out of reset ---");
	expect_eq16("denise bplcon0 at reset", dut.DENISE1.bplcon0, 16'h0000);
	expect_eq16("agnus dmacon at reset",   {3'b000, dut.AGNUS1.dmacon}, 16'h0000);
	expect_eq16("paula intena at reset",   {1'b0, dut.PAULA1.pi1.intena}, 16'h0000);

	// ---------------------------------------------------------------------
	// 1. A replay run the way ss_ctrl runs it today: inside the freeze.
	//
	// The values must reach the chipset. The whole point of the shadow is
	// that a restored machine comes back with the chipset it was saved with.
	// ---------------------------------------------------------------------
	$display("--- replay inside the freeze (ss_ctrl's S_L_REPLAY) ---");

	shadow_load(IDX_BPLCON0, 16'hA55A);
	shadow_load(IDX_DMACON,  16'h0060);   // blitter + sprite bits, DMAEN low
	shadow_load(IDX_INTENA,  16'h0028);
	shadow_load(IDX_VHPOSW,  16'h0055);   // vpos[7:0]=$00, hpos[8:1]=$55

	// The value the restore is carrying, and a live register set to something
	// else entirely. Poked rather than driven: Paula's INTREQ has no write path
	// from this bench that does not go through the replay itself.
	restored_intreq = 15'h2841;
	dut.PAULA1.pi1.intreq = 15'h7FFF;

	freeze_machine();
	run_replay();

	// The beam, which the replay both sets and must not then drift. VHPOSW is
	// an ordinary shadowed register, so a replay writes it like any other:
	// hpos[8:1] takes its low byte and vpos[7:0] its high byte. What must NOT
	// happen is any movement after that write, and there are ~400 more replay
	// ticks behind it -- agnus_beamcounter increments hpos under
	// `clk7_en && cck`, so a machine parked with cck high would step the beam
	// once per remaining write. Checking the exact value therefore checks the
	// phase the freeze parks on. hpos[0] is not a counter bit; it is cck
	// itself, wired straight through.
	expect_eq16("frozen replay -> beam hpos, no drift after VHPOSW",
	            {8'd0, dut.AGNUS1.bc1.hpos[8:1]}, {8'd0, 8'h55});
	expect_eq16("frozen replay -> beam vpos, no drift after VHPOSW",
	            {5'd0, dut.AGNUS1.bc1.vpos}, 16'h0000);

	thaw_machine();
	repeat (40) @(posedge clk_r);

	expect_eq16("frozen replay -> denise bplcon0",
	            dut.DENISE1.bplcon0, 16'hA55A);
	expect_eq16("frozen replay -> agnus dmacon",
	            {3'b000, dut.AGNUS1.dmacon}, 16'h0060);
	expect_eq16("frozen replay -> paula intena",
	            {1'b0, dut.PAULA1.pi1.intena}, 16'h0028);

	// INTREQ, the one chipset register carried by VALUE rather than rebuilt
	// from bus writes: Paula raises its bits in hardware as well as by write,
	// so an accumulator drifts within a frame. The replay writes it back with
	// the set/clear dance from ss_regshadow's intreq_in, which Minimig.sv now
	// drives from the RESTORED vector rather than from Paula's live output --
	// wired live, a restore reinstalled the pending interrupts of the machine
	// it was replacing.
	//
	// Not an equality check, because Paula's hardware request lines are ORed
	// into intreq on every clk7_en and some of them are asserted in a machine
	// sitting idle like this one -- the four audio channels here. That is not
	// the replay leaking: those lines re-raise the moment the machine runs
	// again, restore or no restore, and they were asserted at save time too.
	// What must hold is that the saved bits are back and the bits that were
	// in Paula beforehand and nowhere else are gone. Paula was poked to
	// 0x7FFF -- every bit -- so SOFT and COPER below can only be clear if the
	// clearing half of the dance ran against the register.
	expect_eq16("frozen replay -> saved intreq bits are back",
	            {1'b0, dut.PAULA1.pi1.intreq} & 16'h2841, 16'h2841);
	expect_eq16("frozen replay -> INTREQ bit 2 (SOFT) cleared",
	            {15'd0, dut.PAULA1.pi1.intreq[2]}, 16'h0000);
	expect_eq16("frozen replay -> INTREQ bit 4 (COPER) cleared",
	            {15'd0, dut.PAULA1.pi1.intreq[4]}, 16'h0000);

	// ---------------------------------------------------------------------
	// 2. The same replay with the machine running, which isolates the mux
	//    and the decode from the clock question above. Different values, so
	//    a pass here cannot be left over from the run before.
	// ---------------------------------------------------------------------
	$display("--- replay with the machine running ---");

	shadow_load(IDX_BPLCON0, 16'h3C0F);
	shadow_load(IDX_DMACON,  16'h00C0);
	shadow_load(IDX_INTENA,  16'h0014);

	run_replay();
	repeat (40) @(posedge clk_r);

	expect_eq16("running replay -> denise bplcon0",
	            dut.DENISE1.bplcon0, 16'h3C0F);
	expect_eq16("running replay -> agnus dmacon",
	            {3'b000, dut.AGNUS1.dmacon}, 16'h00C0);
	expect_eq16("running replay -> paula intena",
	            {1'b0, dut.PAULA1.pi1.intena}, 16'h0014);

	// ---------------------------------------------------------------------
	// 3. The memory map bits, in both directions and in the same bit order.
	//    ss_map[3] is ovl and ss_map[2] is gary's rom_readonly; the low two
	//    are address decodes and are not written back. Driven under the
	//    freeze because that is where a restore drives it -- these run on
	//    clk_sys and do not need the Amiga's clock enables.
	// ---------------------------------------------------------------------
	$display("--- memory map restore ---");

	freeze_machine();

	drv_ss_map_in <= 4'b1100;             // ovl=1, rom_readonly=1
	@(posedge clk_r);
	drv_ss_map_we <= 1'b1;
	@(posedge clk_r);
	drv_ss_map_we <= 1'b0;
	repeat (4) @(posedge clk_r);

	expect_eq1("ss_map_in[3] -> ovl",              dut.ovl, 1'b1);
	expect_eq1("ss_map_in[2] -> gary rom_readonly", dut.GARY1.rom_readonly, 1'b1);
	expect_eq1("ss_map[3] reads back ovl",          ss_map[3], 1'b1);
	expect_eq1("ss_map[2] reads back rom_readonly", ss_map[2], 1'b1);

	drv_ss_map_in <= 4'b0000;
	@(posedge clk_r);
	drv_ss_map_we <= 1'b1;
	@(posedge clk_r);
	drv_ss_map_we <= 1'b0;
	repeat (4) @(posedge clk_r);

	expect_eq1("ss_map_in[3]=0 -> ovl clears",     dut.ovl, 1'b0);
	expect_eq1("ss_map_in[2]=0 -> rom_readonly clears",
	           dut.GARY1.rom_readonly, 1'b0);

	// ---------------------------------------------------------------------
	// 4. The CIAs, which the register shadow cannot reach: they are on the CPU
	//    bus, and reading them back over it would clear ICR's pending flags and
	//    move TOD's latch. So they are exported straight out of the flip-flops
	//    and written back the same way, on one pulse.
	//
	//    The check is a round trip through the real registers: drive a pattern
	//    in, read the export out, and require the two to match bit for bit. A
	//    field wired to the wrong slice in either direction shows up as a
	//    mismatch here rather than as a machine that misbehaves later.
	// ---------------------------------------------------------------------
	$display("--- CIA capture and restore ---");

	freeze_machine();

	// Distinguishable per field rather than a walking pattern: a swap between
	// two fields of the same width is exactly what a slice mistake looks like,
	// and a walking pattern would hide it.
	drv_ss_cia_a_in <= { 75'h2A_AAAA_AAAA_AAAA_AAAA,   // timer D
	                     39'h33_3333_3333,             // timer B
	                     39'h11_1111_1111,             // timer A
	                     10'h155,                      // int: mask + pending
	                      8'hC3,                       // sdr
	                      8'h0F,                       // ddrportb
	                      8'hF0,                       // ddrporta
	                      4'h9 };                      // regporta
	drv_ss_cia_b_in <= { 75'h55_5555_5555_5555_5555,   // timer D
	                     39'h44_4444_4444,             // timer B
	                     39'h22_2222_2222,             // timer A
	                     10'h2AA,                      // int: mask + pending
	                      8'h3C,                       // sdr
	                      8'hAA,                       // ddrportb
	                      8'h55,                       // regportb
	                      8'hCC,                       // ddrporta
	                      8'h81 };                     // regporta
	@(posedge clk_r);
	drv_ss_cia_we <= 1'b1;
	@(posedge clk_r);
	drv_ss_cia_we <= 1'b0;
	repeat (4) @(posedge clk_r);

	if (ss_cia_a !== drv_ss_cia_a_in) begin
		$display("FAIL CIA A round trip: got %h want %h", ss_cia_a, drv_ss_cia_a_in);
		errs = errs + 1;
	end
	else $display("ok   CIA A restored and exported identically");

	if (ss_cia_b !== drv_ss_cia_b_in) begin
		$display("FAIL CIA B round trip: got %h want %h", ss_cia_b, drv_ss_cia_b_in);
		errs = errs + 1;
	end
	else $display("ok   CIA B restored and exported identically");

	// And the same values seen from inside, at three registers picked because
	// each is reached by a different path: a sub-module counter, a sub-module
	// mask, and one of the CIA's own port registers. A round trip that agreed
	// with itself through a pair of matching slice errors would still fail
	// these.
	// Expected values are written as slices of the word that was driven in,
	// spelling out where each field is meant to sit. That is the claim being
	// tested: ciaa.v and ciab.v document a layout in a comment, and this is
	// what says the wiring agrees with it.
	//
	//   CIA A: timer A at [76:38], and within it {tmr, tmlh, tmll, tmcr},
	//          so the counter is the top sixteen bits of that field.
	expect_eq16("CIA A timer A counter",
	            dut.CIAA1.tmra.tmr, drv_ss_cia_a_in[76:61]);
	//   CIA A: cia_int at [37:28] as {icrmask, icr}.
	expect_eq16("CIA A interrupt mask",
	            {11'd0, dut.CIAA1.cnt.icrmask}, {11'd0, drv_ss_cia_a_in[37:33]});
	//   CIA B: port A direction at [15:8].
	expect_eq16("CIA B port A direction",
	            {8'd0, dut.CIAB1.ddrporta}, {8'd0, drv_ss_cia_b_in[15:8]});

	// The CIAs run on clk7_en, which is stopped: a write that had been left
	// inside that gate would never land at all while frozen, which is the
	// mistake gary's rom_readonly restore avoids in the same way.
	//   CIA A: timer D at [190:116], and within it {tod, alarm, tod_latch, ...},
	//          so the TOD counter is the top twenty-four bits of that field.
	expect_eq16("CIA A TOD counter, low half",
	            dut.CIAA1.tmrd.tod[15:0], drv_ss_cia_a_in[182:167]);

	thaw_machine();
	repeat (100) @(posedge clk_r);

	// ------------------------------------------------------------- verdict
	checks_live = 1'b0;

	if (tap_errs)  begin
		$display("FAIL snoop tap disagreed with the bus on %0d cycles", tap_errs);
		errs = errs + 1;
	end
	else $display("ok   snoop tap followed the bus for the whole run");

	if (mux_errs) begin
		$display("FAIL replay mux wrong on %0d cycles", mux_errs);
		errs = errs + 1;
	end
	else $display("ok   replay mux: passthrough when idle, replay when driven");

	if (excl_errs) begin
		$display("FAIL replay drove the read window on %0d cycles", excl_errs);
		errs = errs + 1;
	end
	else $display("ok   replay never drove an address below $020");

	if (errs == 0) $display("RUN: PASS");
	else           $display("RUN: FAIL (%0d errors)", errs);

	$finish;
end

// Watchdog. A hung handshake should not hang the pipeline. It prints the
// signals a hang here can hang on, because "timeout" on its own says nothing.
initial begin
	#20_000_000;
	$display("RUN: FAIL (timeout)");
	$display("  sys_reset=%b rst_cnt=%b usrrst=%b rst_ext=%b",
	         dut.sys_reset, dut.CONTROL1.rst_cnt, dut.usrrst, rst_ext);
	$display("  ss_freeze=%b ss_freeze_7m=%b am_clk7_en=%b m_clk7_en=%b",
	         ss_freeze, ss_freeze_7m, clk7_en, m_clk7_en);
	$display("  replay_active=%b replay_done=%b replay_addr=%02h",
	         sh_replay_active, sh_replay_done, ss_replay_addr);
	$finish;
end

endmodule
