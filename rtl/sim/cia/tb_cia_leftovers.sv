// The 2005 CIA leftovers: CIA-A port B, PB6/PB7 timer output, and CIA-B's
// serial data register.
//
// ciaa.v has listed these in its header since 2005:
//
//     // NOT implemented is:
//     // serial data register for CIA B(but keyboard input for CIA A is supported)
//     // port B for CIA A
//     // counter inputs for timer A and B other then 'E' clock
//     // toggling of PB6/PB7 by timer A/B
//
// Three of the four are covered here. The count-source selects are the fourth
// and stay unimplemented on purpose -- upstream b013ce3 added them without
// wiring CNT, which broke Flink, and the revert is guarded by its own step in
// rtl-sim.yml.
//
// The serial register is the interesting one, because it looks like it needs
// CNT and does not. WinUAE cia.cpp shifts inside the timer A underflow path,
// gated on (cr & (CR_SPMODE | CR_RUNMODE)) == CR_SPMODE: in output mode the CIA
// generates CNT rather than receiving it. CIA-B's CNT pin goes to the expansion
// bus and is unconnected on a stock Amiga, so input mode has no source on real
// hardware either.
//
// Runs standalone under Icarus:
//   iverilog -g2012 -o tb ../../ciaa.v ../../ciab.v ../../cia_timera.v \
//     ../../cia_timerb.v ../../cia_int.v ../../cia_timerd.v tb_cia_leftovers.sv
//   vvp tb

`timescale 1ns/1ps

module tb_cia_leftovers;

	// CIA register selects.
	localparam [3:0] R_PRA  = 4'h0;
	localparam [3:0] R_PRB  = 4'h1;
	localparam [3:0] R_DDRB = 4'h3;
	localparam [3:0] R_TALO = 4'h4;
	localparam [3:0] R_TAHI = 4'h5;
	localparam [3:0] R_TBLO = 4'h6;
	localparam [3:0] R_TBHI = 4'h7;
	localparam [3:0] R_SDR  = 4'hC;
	localparam [3:0] R_ICR  = 4'hD;
	localparam [3:0] R_CRA  = 4'hE;
	localparam [3:0] R_CRB  = 4'hF;

	// Control register bits.
	localparam [7:0] CR_START   = 8'h01;
	localparam [7:0] CR_PBON    = 8'h02;
	localparam [7:0] CR_OUTMODE = 8'h04;
	localparam [7:0] CR_SPMODE  = 8'h40;

	reg        clk = 0;
	reg        clk7_en = 0;
	wire       eclk;
	reg        reset = 1;
	reg        aen_a = 0, aen_b = 0;
	reg        rd = 0, wr = 0;
	reg  [3:0] rs = 4'h0;
	reg  [7:0] data_in = 0;
	reg  [7:0] portb_in = 8'h00;

	wire [7:0] dout_a, dout_b;
	wire [7:0] portb_out_b;

	integer errors = 0;

	always #1 clk = ~clk;

	// clk7_en one in four; eclk one clk7_en in ten, which is the shape the
	// timers assume -- they decrement once per eclk under clk7_en.
	reg [1:0] phase = 0;
	reg [3:0] ediv  = 0;
	always @(posedge clk) begin
		phase   <= phase + 2'd1;
		clk7_en <= (phase == 2'd0);
	end
	// eclk must be high on the same cycle clk7_en is: the timers decrement
	// inside `if (clk7_en) ... else if (start && count)`, so an eclk that only
	// went high between clk7_en pulses would never be seen and no timer would
	// ever count.
	assign eclk = clk7_en && (ediv == 4'd9);
	always @(posedge clk)
		if (clk7_en) ediv <= (ediv == 4'd9) ? 4'd0 : ediv + 4'd1;

	ciaa ca (
		.clk(clk), .clk7_en(clk7_en), .clk7n_en(1'b0),
		.aen(aen_a), .rd(rd), .wr(wr), .reset(reset),
		.rs(rs), .data_in(data_in), .data_out(dout_a),
		.tick(1'b0), .eclk(eclk), .irq(),
		.porta_in(6'h3F), .porta_out(),
		.portb_in(portb_in),
		.kms_level(1'b0), .kbd_mouse_type(2'd0), .kbd_mouse_data(8'd0),
		.freeze(), .hrtmon_en(1'b0),
		.ss_state(), .ss_ld(1'b0), .ss_ld_data(191'd0)
	);

	ciab cb (
		.clk(clk), .clk7_en(clk7_en),
		.aen(aen_b), .rd(rd), .wr(wr), .reset(reset),
		.rs(rs), .data_in(data_in), .data_out(dout_b),
		.tick(1'b0), .eclk(eclk), .flag(1'b1), .irq(),
		.porta_in(6'h3F), .porta_out(),
		.ss_state(), .ss_ld(1'b0), .ss_ld_data(203'd0),
		.portb_out(portb_out_b)
	);

	// Bus access. Driven off the falling edge -- the CIAs sample on posedge with
	// clk7_en, and assigning at a posedge races the sampler.
	task wr_a(input [3:0] r, input [7:0] d);
		begin
			@(negedge clk);
			aen_a = 1; wr = 1; rd = 0; rs = r; data_in = d;
			@(posedge clk); while (!clk7_en) @(posedge clk);
			@(negedge clk);
			aen_a = 0; wr = 0; rs = 4'h0; data_in = 8'h00;
		end
	endtask

	task wr_b(input [3:0] r, input [7:0] d);
		begin
			@(negedge clk);
			aen_b = 1; wr = 1; rd = 0; rs = r; data_in = d;
			@(posedge clk); while (!clk7_en) @(posedge clk);
			@(negedge clk);
			aen_b = 0; wr = 0; rs = 4'h0; data_in = 8'h00;
		end
	endtask

	task rd_a(input [3:0] r, output [7:0] d);
		begin
			@(negedge clk);
			aen_a = 1; rd = 1; wr = 0; rs = r;
			@(posedge clk);
			#1;
			d = dout_a;
			@(negedge clk);
			aen_a = 0; rd = 0; rs = 4'h0;
		end
	endtask

	task rd_b(input [3:0] r, output [7:0] d);
		begin
			@(negedge clk);
			aen_b = 1; rd = 1; wr = 0; rs = r;
			@(posedge clk);
			#1;
			d = dout_b;
			@(negedge clk);
			aen_b = 0; rd = 0; rs = 4'h0;
		end
	endtask

	task check(input [511:0] what, input [7:0] got, input [7:0] want);
		begin
			if (got !== want) begin
				$display("FAIL: %0s: got %02x want %02x", what, got, want);
				errors = errors + 1;
			end else begin
				$display("ok:   %0s = %02x", what, got);
			end
		end
	endtask

	// Count transitions of one port B bit over a window, which is how the
	// PB6/PB7 toggle is measured.
	task automatic count_edges(input integer bit_sel, input integer cycles,
	                           output integer edges);
		integer i;
		reg prev, now;
		begin
			edges = 0;
			prev  = ca.portb_out[bit_sel];
			for (i = 0; i < cycles; i = i + 1) begin
				@(posedge clk);
				#1;
				now = ca.portb_out[bit_sel];
				if (now !== prev) edges = edges + 1;
				prev = now;
			end
		end
	endtask

	reg [7:0] v;
	integer edges;

	initial begin
		repeat (20) @(posedge clk);
		reset = 0;
		repeat (20) @(posedge clk);

		// ---- 1. CIA-A port B is a real port ---------------------------------
		// It had no output register at all: a write to PRB was dropped and a
		// read returned the pins whatever DDRB said.
		portb_in = 8'h5A;
		wr_a(R_DDRB, 8'hFF);            // all outputs
		wr_a(R_PRB,  8'hA5);
		rd_a(R_PRB, v);
		check("PRB reads back what was written", v, 8'hA5);

		wr_a(R_DDRB, 8'h00);            // all inputs
		rd_a(R_PRB, v);
		check("PRB reads the pins when DDRB is input", v, 8'h5A);

		wr_a(R_DDRB, 8'h0F);            // low nibble out, high nibble in
		rd_a(R_PRB, v);
		check("PRB mixes register and pins per DDRB", v, 8'h55);

		rd_a(R_DDRB, v);
		check("DDRB reads back", v, 8'h0F);

		// ---- 2. PB6 driven by timer A ---------------------------------------
		wr_a(R_DDRB, 8'hFF);
		wr_a(R_PRB,  8'h00);            // PB6 low from the register
		rd_a(R_PRB, v);
		check("PB6 follows the register with PBON clear", v, 8'h00);

		// Short period so underflows come quickly, then start with PBON and
		// OUTMODE set: PB6 toggles on every underflow.
		wr_a(R_TALO, 8'd2);
		wr_a(R_TAHI, 8'd0);
		wr_a(R_CRA, CR_START | CR_PBON | CR_OUTMODE);
		count_edges(6, 4000, edges);
		if (edges < 2) begin
			$display("FAIL: PB6 did not toggle (edges=%0d)", edges);
			errors = errors + 1;
		end else begin
			$display("ok:   PB6 toggles under timer A (%0d edges)", edges);
		end

		// Clearing PBON hands the pin back to the register.
		wr_a(R_CRA, CR_START | CR_OUTMODE);
		repeat (200) @(posedge clk);
		rd_a(R_PRB, v);
		check("PB6 back under the register with PBON clear", v, 8'h00);
		wr_a(R_CRA, 8'h00);

		// ---- 3. PB7 driven by timer B ---------------------------------------
		wr_a(R_TBLO, 8'd2);
		wr_a(R_TBHI, 8'd0);
		wr_a(R_CRB, CR_START | CR_PBON | CR_OUTMODE);
		count_edges(7, 4000, edges);
		if (edges < 2) begin
			$display("FAIL: PB7 did not toggle (edges=%0d)", edges);
			errors = errors + 1;
		end else begin
			$display("ok:   PB7 toggles under timer B (%0d edges)", edges);
		end
		wr_a(R_CRB, 8'h00);

		// ---- 4. CIA-B serial register ---------------------------------------
		// Output mode, timer A running: eight underflows shift the byte out and
		// the SP interrupt fires. Before this existed the SDR was a latch, ser
		// was tied low, and software waiting on SP waited forever.
		rd_b(R_ICR, v);                 // reading ICR clears it
		wr_b(R_TALO, 8'd2);
		wr_b(R_TAHI, 8'd0);
		wr_b(R_CRA, CR_START | CR_SPMODE);
		wr_b(R_SDR, 8'hC3);

		// Let eight underflows go by, then look for SP (ICR bit 3).
		repeat (8000) @(posedge clk);
		rd_b(R_ICR, v);
		if (!v[3]) begin
			$display("FAIL: CIA-B SP interrupt never fired (ICR=%02x)", v);
			errors = errors + 1;
		end else begin
			$display("ok:   CIA-B SP interrupt fired (ICR=%02x)", v);
		end

		// The shifter is idle again, and a read still returns the written byte.
		rd_b(R_SDR, v);
		check("CIA-B SDR reads back the written byte", v, 8'hC3);

		// In input mode nothing shifts, so no SP interrupt arrives.
		rd_b(R_ICR, v);                 // clear
		wr_b(R_CRA, CR_START);          // SPMODE clear
		wr_b(R_SDR, 8'h3C);
		repeat (8000) @(posedge clk);
		rd_b(R_ICR, v);
		if (v[3]) begin
			$display("FAIL: CIA-B SP fired in input mode (ICR=%02x)", v);
			errors = errors + 1;
		end else begin
			$display("ok:   CIA-B SP stays quiet in input mode");
		end

		if (errors == 0) $display("RUN: PASS");
		else             $display("RUN: FAIL (%0d)", errors);
		$finish;
	end

	initial begin
		#4000000;
		$display("FAIL: timeout");
		$display("RUN: FAIL (timeout)");
		$finish;
	end

endmodule
