// SPDX-License-Identifier: GPL-3.0-or-later
//
// Where each memory region lands in SDRAM.
//
// The savestate reads chip RAM, slow RAM and the Kickstart ROM through
// sdram_ctrl's borrowed CPU port, and it has to name each region by the
// address that port expects. That address is not the CPU address: gary drops
// the low 18 bits into ram_address_out, minimig_bankmapper picks a bank from
// the region selects, and minimig_sram_bridge folds the two into a 22-bit word
// address -- discarding CPU address bit 23 on the way, and remapping chip and
// Kickstart entirely.
//
// The rule that comes out of all that is simple enough to state in one line:
//
//     sdram word address = (cpu byte address & $7FFFFF) >> 1
//
// for every region the bridge passes through, which is slow RAM, the cartridge
// and the 1 MB Kickstart slot. Chip RAM and the $F80000 Kickstart are remapped
// and have to be checked separately.
//
// Stating it is not the same as knowing it. ss_ctrl's kick_base was derived by
// hand once and happened to be right; the same derivation for slow RAM would
// be a guess with a save state riding on it, and a wrong base reads 1.5 MB of
// the wrong memory and restores it over the right memory. So this bench asks
// the two real modules instead of asking a comment.

`timescale 1ns / 1ps

module tb_membank_map;

// ---------------------------------------------------------------------------
// The two modules under test, wired the way minimig.v wires them.
// ---------------------------------------------------------------------------
logic [3:0] sel_chip = 4'b0000;
logic [2:0] sel_slow = 3'b000;
logic       sel_kick = 1'b0;
logic       sel_kick1mb = 1'b0;
logic       sel_kick256kmirror = 1'b0;
logic       sel_cart = 1'b0;
logic [1:0] memory_config = 2'b11;     // 2 MB chip, the AmigaCD default

wire  [7:0] bank;

minimig_bankmapper u_bmap (
	.chip0(sel_chip[0]), .chip1(sel_chip[1]),
	.chip2(sel_chip[2]), .chip3(sel_chip[3]),
	.slow0(sel_slow[0]), .slow1(sel_slow[1]), .slow2(sel_slow[2]),
	.kick(sel_kick), .kick1mb(sel_kick1mb),
	.kick256kmirror(sel_kick256kmirror),
	.cart(sel_cart),
	.memory_config(memory_config),
	.bank(bank)
);

logic [23:1] address_in = 23'd0;
wire  [22:1] address;

minimig_sram_bridge u_bridge (
	.clk(1'b0), .c1(1'b0), .c3(1'b0),
	.bank(bank),
	.address_in(address_in),
	.data_in(16'h0000), .data_out(),
	.rd(1'b1), .hwr(1'b0), .lwr(1'b0),
	._bhe(), ._ble(), ._we(), ._oe(),
	.address(address), .data(), .ramdata_in(16'h0000)
);

// ---------------------------------------------------------------------------
// Score keeping
// ---------------------------------------------------------------------------
int checks = 0;
int errs   = 0;

task automatic check(input string name, input logic [23:0] expected,
                     input logic [23:0] actual);
	checks++;
	if (expected !== actual) begin
		$display("FAIL %s: expected $%06h got $%06h", name, expected, actual);
		errs++;
	end
	else $display("PASS %s = $%06h", name, actual);
endtask

// gary's ram_address_out for a CPU byte address, for the regions this bench
// covers. The remapped Kickstart mirrors are gary's business and are not
// exercised here; every region below takes the plain path.
function automatic logic [23:1] gary_addr(input logic [23:0] cpu_byte);
	gary_addr = cpu_byte[23:1];
endfunction

// Ask the bridge where a CPU byte address lands. Returns the 22-bit word
// address as sdram_ctrl's cpuAddr sees it -- which is the vector read as a
// number, i.e. bit index i carries weight 2**(i-1).
task automatic probe(input logic [23:0] cpu_byte, output logic [23:0] out);
	begin
		address_in = gary_addr(cpu_byte);
		#1;
		out = {2'b00, address};
	end
endtask

logic [23:0] a;

initial begin
	$display("== tb_membank_map");

	// -------------------------------------------------------------------
	// Kickstart at $F80000. This one is already known good: ss_ctrl's
	// SS_KICK_BASE is $3C0000 and the fingerprint it computes has matched
	// the file's on hardware. It is here as the control -- if the bench
	// disagrees with the shipping value, the bench is wrong, not the core.
	// -------------------------------------------------------------------
	sel_kick = 1'b1;
	probe(24'hF80000, a);
	check("kick $F80000", 24'h3C0000, a);
	probe(24'hFFFFFE, a);
	check("kick $FFFFFE", 24'h3FFFFF, a);
	sel_kick = 1'b0;

	// -------------------------------------------------------------------
	// Chip RAM. Remapped by the bank[5] branch, so it does NOT follow the
	// passthrough rule and has to be asked for directly. ss_ctrl uses
	// chip_base = 0 and 2 MB of it.
	// -------------------------------------------------------------------
	sel_chip = 4'b0001;
	probe(24'h000000, a);
	check("chip $000000", 24'h000000, a);
	sel_chip = 4'b1000;
	probe(24'h1FFFFE, a);
	check("chip $1FFFFE", 24'h0FFFFF, a);
	sel_chip = 4'b0000;

	// -------------------------------------------------------------------
	// Slow RAM, $C00000-$D7FFFF. This is the answer the savestate needs.
	// -------------------------------------------------------------------
	sel_slow = 3'b001;
	probe(24'hC00000, a);
	check("slow $C00000", 24'h200000, a);
	sel_slow = 3'b010;
	probe(24'hC80000, a);
	check("slow $C80000", 24'h240000, a);
	sel_slow = 3'b100;
	probe(24'hD00000, a);
	check("slow $D00000", 24'h280000, a);
	probe(24'hD7FFFE, a);
	check("slow $D7FFFE", 24'h2BFFFF, a);
	sel_slow = 3'b000;

	// -------------------------------------------------------------------
	// The two neighbours, so that "slow RAM is 1.5 MB starting at $200000"
	// is bounded on both sides rather than merely starting in the right
	// place. A base that was right and a length that ran long would read
	// the 1 MB Kickstart slot into the save file.
	// -------------------------------------------------------------------
	sel_cart = 1'b1;
	probe(24'hA00000, a);
	check("cart $A00000", 24'h100000, a);
	sel_cart = 1'b0;

	sel_kick1mb = 1'b1;
	probe(24'hE00000, a);
	check("kick1mb $E00000", 24'h300000, a);
	sel_kick1mb = 1'b0;

	$display("== %0d checks, %0d failures", checks, errs);
	if (errs == 0) $display("RUN: PASS");
	else           $display("RUN: FAIL");
	$finish;
end

endmodule
