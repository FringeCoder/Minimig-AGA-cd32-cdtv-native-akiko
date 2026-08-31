// cpu_wrapper: the save state park, the stock-speed throttle, and the
// chip-slot guard.
//
// cpu_wrapper had no simulation coverage at all, and it is not a quiet corner.
// Three things live in it that a future upstream merge can plausibly remove
// without anyone noticing, and all three came close already:
//
//   1. The save state CPU park -- ss_bus_settled & ~ss_cpu_hold. During the
//      74d6ce0 merge, upstream's side of the conflict carried a plainer
//      clkena_p_base and taking theirs would have deleted the park silently.
//   2. The stock-speed throttle's cooldown constant, which upstream 148931d
//      had wrong by a factor of two (9 instead of 4, i.e. half A1200 speed).
//   3. The chip-slot guard from 114ab43: chipreq must stay low while the
//      fastchip or CDTV bridge is serving the access.
//
// The park is the part most worth pinning down. Its own comment explains that
// reading ss_at_boundary live rather than latched releases the CPU while
// ss_ctrl is still writing chip RAM -- ss_resume clears decodeOPC, so the
// boundary goes false the instant the sequencer is re-seeded. A restore that
// lands on half-rewritten memory is indistinguishable from the mid-instruction
// restore the park exists to prevent, which is why test 2 below is the one that
// matters: it drops the boundary while the arm is still held and requires the
// hold to survive.
//
// The two CPU cores are stubbed -- see cpu_core_stubs.v. None of the logic
// under test needs a working 68000, only control over busstate and
// ss_at_boundary.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../cpu_wrapper.v cpu_core_stubs.v tb_cpu_wrapper_park.sv
//   vvp tb

`timescale 1ns/1ps

module tb_cpu_wrapper_park;

	reg         clk = 0;
	reg         reset = 0;          // active low
	reg         ph1 = 0, ph2 = 0;
	reg  [2:0]  cpucfg = 3'b001;    // [1:0] non-zero selects TG68K; [2] is stock speed
	reg  [2:0]  fastramcfg = 3'd0;
	reg  [2:0]  cachecfg = 3'd0;
	reg         bootrom = 1'b0;
	reg         ss_arm = 1'b0;
	reg         ramready = 1'b0;
	reg         fastchip_selack = 1'b0;
	reg         fastchip_ready = 1'b0;
	reg         cdtv_selack = 1'b0;
	reg         cdtv_mode = 1'b0;
	reg         chip_dtack = 1'b1;

	wire        ss_bus_settled;
	wire  [1:0] cpustate;
	wire        ramsel;

	integer errors = 0;

	always #5 clk = ~clk;

	cpu_wrapper dut (
		.reset(reset), .reset_out(),
		.clk(clk), .ph1(ph1), .ph2(ph2),
		.cpucfg(cpucfg), .fastramcfg(fastramcfg), .cachecfg(cachecfg),
		.bootrom(bootrom),

		.ss_arm(ss_arm),
		.ss_reg_index(4'd0), .ss_reg_data(),
		.ss_pc(), .ss_exe_pc(), .ss_at_boundary(),
		.ss_trap_vector(), .ss_trap_active(),
		.ss_bus_settled(ss_bus_settled),
		.ss_sr(), .ss_usp(), .ss_vbr(), .ss_cacr(),
		.ss_wr_index(4'd0), .ss_wr_data(32'd0), .ss_wr_en(1'b0),
		.ss_pc_wr(1'b0), .ss_sr_wr(1'b0), .ss_usp_wr(1'b0),
		.ss_vbr_wr(1'b0), .ss_cacr_wr(1'b0), .ss_resume(1'b0),

		.chip_addr(), .chip_dout(16'd0), .chip_din(),
		.chip_as(), .chip_uds(), .chip_lds(), .chip_rw(),
		.chip_dtack(chip_dtack), .chip_ipl(3'b111),

		.fastchip_dout(16'd0), .fastchip_sel(),
		.fastchip_lds(), .fastchip_uds(), .fastchip_rnw(), .fastchip_lw(),
		.fastchip_selack(fastchip_selack), .fastchip_ready(fastchip_ready),

		.ramsel(ramsel), .ramaddr(), .ramdin(), .ramdout(16'd0),
		.ramready(ramready), .ramlds(), .ramuds(), .ramshared(),

		.toccata_ena(), .toccata_base(), .a2065_ena(), .a2065_base(),

		.cdtv_mode(cdtv_mode), .cdtv_din(16'd0), .cdtv_selack(cdtv_selack),
		.cdtv_base(),

		.cpustate(cpustate), .cacr(), .nmi_addr(),
		.z2ram_ena_out(), .z3ram_base0_out(), .z3ram_ena0_out(),
		.z3ram_base1_out(), .z3ram_ena1_out(), .dcache_sw_en()
	);

	// Shorthands for what is being asserted about.
	wire clkena = dut.clkena_p_throttled;

	task check(input [511:0] what, input integer got, input integer want);
		begin
			if (got !== want) begin
				$display("FAIL: %0s: got %0d, want %0d", what, got, want);
				errors = errors + 1;
			end else begin
				$display("ok:   %0s = %0d", what, got);
			end
		end
	endtask

	// Count clkena ticks over a window, which is how the throttle is measured.
	task automatic count_ticks(input integer cycles, output integer n);
		integer i;
		begin
			n = 0;
			for (i = 0; i < cycles; i = i + 1) begin
				@(posedge clk);
				#1;
				if (clkena) n = n + 1;
			end
		end
	endtask

	// Longest run of consecutive cycles with clkena low, over a window.
	task automatic max_gap(input integer cycles, output integer worst);
		integer i, run;
		begin
			worst = 0;
			run   = 0;
			for (i = 0; i < cycles; i = i + 1) begin
				@(posedge clk);
				#1;
				if (clkena) begin
					if (run > worst) worst = run;
					run = 0;
				end else run = run + 1;
			end
		end
	endtask

	integer n, gap;

	initial begin
		repeat (4) @(posedge clk);
		reset = 1'b1;                     // out of reset
		repeat (4) @(posedge clk);

		// Bus idle: busstate 1 means no memory access, so cpu_req is low and
		// ss_bus_settled is true without anything having to answer.
		dut.cpu_inst_p.r_busstate    = 2'd1;
		dut.cpu_inst_p.r_at_boundary = 1'b0;
		repeat (4) @(posedge clk);

		// ---- 1. free running -------------------------------------------------
		check("bus settled with no request", ss_bus_settled, 1);
		count_ticks(20, n);
		check("clkena ticks every cycle when free", n, 20);

		// ---- 2. the park, and that it is LATCHED ----------------------------
		// Arm while sitting on a boundary: the CPU must stop.
		dut.cpu_inst_p.r_at_boundary = 1'b1;
		ss_arm = 1'b1;
		repeat (2) @(posedge clk);
		count_ticks(20, n);
		check("parked on a boundary", n, 0);

		// Now take the boundary away with the arm still held. This is what
		// ss_resume does when it re-seeds the sequencer: decodeOPC clears and
		// ss_at_boundary goes false. A live-gated hold releases the CPU here,
		// which is the bug -- ss_ctrl is still writing chip RAM at this point.
		dut.cpu_inst_p.r_at_boundary = 1'b0;
		repeat (2) @(posedge clk);
		count_ticks(40, n);
		check("still parked after the boundary goes away", n, 0);

		// Releasing the arm releases the CPU.
		ss_arm = 1'b0;
		repeat (2) @(posedge clk);
		count_ticks(20, n);
		check("released when ss_arm drops", n, 20);

		// The latch must not carry over into the next arm-and-release, so a
		// second park behaves like the first.
		dut.cpu_inst_p.r_at_boundary = 1'b1;
		ss_arm = 1'b1;
		repeat (2) @(posedge clk);
		dut.cpu_inst_p.r_at_boundary = 1'b0;
		count_ticks(20, n);
		check("second park holds too", n, 0);
		ss_arm = 1'b0;
		dut.cpu_inst_p.r_at_boundary = 1'b0;
		repeat (2) @(posedge clk);

		// Arming while NOT on a boundary must not stop the CPU dead: it has to
		// be allowed to reach one. Nothing here ever reaches a boundary, so the
		// CPU keeps running -- which is the behaviour that lets a save request
		// wait rather than deadlock.
		ss_arm = 1'b1;
		count_ticks(20, n);
		check("armed but never at a boundary keeps running", n, 20);
		ss_arm = 1'b0;
		repeat (2) @(posedge clk);

		// ---- 3. an outstanding bus cycle holds clkena low -------------------
		// busstate 2 is a data read, so cpu_req goes high and nothing has
		// answered. The freeze must not be taken with a fill half done.
		dut.cpu_inst_p.r_busstate = 2'd2;
		repeat (2) @(posedge clk);
		check("bus not settled with a request outstanding", ss_bus_settled, 0);
		count_ticks(20, n);
		check("clkena held while the bus is outstanding", n, 0);

		// Any one of the four ready sources settles it.
		ramready = 1'b1;
		repeat (2) @(posedge clk);
		check("ramready settles the bus", ss_bus_settled, 1);
		ramready = 1'b0;
		cdtv_selack = 1'b1;
		repeat (2) @(posedge clk);
		check("cdtv_selack settles the bus", ss_bus_settled, 1);
		cdtv_selack = 1'b0;
		fastchip_ready = 1'b1;
		repeat (2) @(posedge clk);
		check("fastchip_ready settles the bus", ss_bus_settled, 1);
		fastchip_ready = 1'b0;
		dut.cpu_inst_p.r_busstate = 2'd1;
		repeat (2) @(posedge clk);

		// ---- 4. the stock-speed throttle ------------------------------------
		// Four sysclk of cooldown after every tick, so one tick in five and a
		// longest gap of exactly 4. 148931d had this at 9, which is half A1200
		// speed, and the CD32 and A1200 presets both ride on it.
		cpucfg = 3'b101;                 // stock speed on, TG68K still selected
		repeat (8) @(posedge clk);
		count_ticks(50, n);
		check("stock speed: one tick in five over 50 cycles", n, 10);
		max_gap(50, gap);
		check("stock speed: cooldown is 4 cycles", gap, 4);

		cpucfg = 3'b001;                 // stock speed off
		repeat (8) @(posedge clk);
		max_gap(30, gap);
		check("turbo: no cooldown", gap, 0);

		// ---- 5. the chip-slot guard -----------------------------------------
		// chipreq must not assert while another bridge is serving the access.
		// The stub's address is CIA space, which is not RAM -- assert that,
		// because with ramsel high chipreq would be low anyway and every check
		// below would pass without testing anything.
		dut.cpu_inst_p.r_busstate = 2'd2;
		repeat (3) @(posedge clk);
		check("precondition: the test address is not RAM", ramsel, 0);
		check("chipreq asserts for a plain chip access", dut.chipreq, 1);

		cdtv_selack = 1'b1;
		repeat (2) @(posedge clk);
		check("chipreq suppressed by cdtv_selack", dut.chipreq, 0);
		cdtv_selack = 1'b0;

		fastchip_selack = 1'b1;
		repeat (2) @(posedge clk);
		check("chipreq suppressed by fastchip_selack", dut.chipreq, 0);
		fastchip_selack = 1'b0;
		repeat (2) @(posedge clk);
		check("chipreq back after the bridges let go", dut.chipreq, 1);

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#2000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
