// SPDX-License-Identifier: GPL-3.0-or-later
//
// memory_router -- shared CPU/DMA address decode + ramaddr remap.
//
// Extracted verbatim from cpu_wrapper.v (lines 98-147 + the comment block
// at 128-140 describing the SDRAM/DDR3 split). Two consumers:
//
//   - cpu_wrapper.v       : pass real cchip / ckick / wr, use all sel_*
//                           outputs to drive ramsel.
//   - chipdma_arb.v       : tie cchip=0, ckick=0, wr=0; consume only
//                           ramaddr + zram_sel to decide ram1 vs ram2.
//
// The address remap (sel_z2ram → ramaddr[28]=1 → DDR3 row) is the
// "authenticity bridge" that makes both controllers look like one
// coherent memory map. See research/docs/dma-fastram-routing-design.md.
//
// THIS MODULE IS A PURE REFACTOR. Phase A acceptance criterion is that
// the synthesized RBF is functionally identical to the pre-refactor
// build. Any change here is a Phase B (or later) concern.

module memory_router
(
	// Amiga byte address. Only [31:1] is meaningfully decoded; bit 0 is
	// the byte selector, handled by the consumer's UDS/LDS lanes.
	input      [31:0] cpu_addr,

	// CPU-side gates. The bridge ties these to 0 — it has no turbo
	// mode and never writes to KS ROM.
	input             cchip,         // 1 = turbo chip enabled (gates sel_chipram)
	input             ckick,         // 1 = turbo kick enabled (gates sel_kickram)
	input             wr,            // 1 = write cycle (sel_kickram is write-only)

	// KS lower-half flag. Affects ramaddr[18] when sel_kicklower fires.
	input             bootrom,

	// AutoConfig'd fast-RAM enable + base state. Owned by cpu_wrapper
	// today (lines 549-619); exported via new ports for the bridge in
	// Phase B.
	input             z2ram_ena,
	input       [4:0] z3ram_base0,
	input             z3ram_ena0,
	input       [3:0] z3ram_base1,
	input             z3ram_ena1,

	// Decode signals. Consumed by cpu_wrapper to drive ramsel + cache
	// gating. The bridge ignores sel_chipram / sel_kickram (it always
	// reaches chip RAM via chipdma_arb's existing chipAddr path) and
	// uses only ramaddr / zram_sel.
	output            sel_chipram,
	output            sel_kickram,
	output            sel_kicklower,
	output            sel_z2ram,
	output            sel_z3ram0,
	output            sel_z3ram1,
	output            sel_zram,
	output            sel_dd,
	output            sel_rtg,

	// Physical RAM index after remap. zram_sel chooses which controller
	// the address belongs to (Minimig.sv:596-598 mux pattern).
	output     [28:1] ramaddr,
	output            zram_sel        // 1 = ram2 (DDR3), 0 = ram1 (SDRAM)
);

// -------------------------------------------------------------------------
// Address decode (verbatim from cpu_wrapper.v:104-115)
// -------------------------------------------------------------------------

assign sel_z3ram0   = (cpu_addr[31:27] == z3ram_base0) && z3ram_ena0;
assign sel_z3ram1   = (cpu_addr[31:28] == z3ram_base1) && z3ram_ena1;
assign sel_z2ram    = !cpu_addr[31:24] && (cpu_addr[23] ^ |cpu_addr[22:21]) && z2ram_ena; // addr[23:21] = 1..4
assign sel_zram     = sel_z3ram0 | sel_z3ram1 | sel_z2ram;
assign sel_dd       = (cpu_addr[31:16] == 16'h00DD) && (cpu_addr[15:13] == 3'b010);
assign sel_rtg      = (cpu_addr[31:24] == 8'h02);

// don't sel_kickram when writing
// CD32 mirrors $a8xxxx (mirror of $f8xxxx) and $b0xxxx (mirror of $e0xxxx)
assign sel_kickram   = !cpu_addr[31:24] && (&cpu_addr[23:19] || (cpu_addr[23:19] == 5'b11100) || (cpu_addr[23:19] == 5'b10101) || (cpu_addr[23:19] == 5'b10110)) && ckick && wr;
assign sel_kicklower = !cpu_addr[31:24] && (cpu_addr[23:18] == 6'b111110);
assign sel_chipram   = !cpu_addr[31:21] && cchip;

// -------------------------------------------------------------------------
// Address remap (verbatim from cpu_wrapper.v:128-147)
//
//       Main  DDx  RTG  8M  128M  256M
//       ----  ---  ---  --  ----  ----
//        SDR  DDR  RTG  Z2  Z3_0  Z3_1
// 28      0    0    0   1    0     1
// 27      0    0    0   1    1     X
// 26      0    1    1   0    X     X
// 25-23   0   111  110  0    X     X
// supported configs: SDR + (Z2, Z3_1, Z3_0+Z3_1)
//
// This is the mapping to the sram
// map 00-1f to 00-1f (chipram), a0-ff to 20-7f. All non-fastram goes into
// the first 8M block (SDRAM). This map should be the same as in
// minimig_sram_bridge.v. All Zorro RAM goes to DDR3.
// -------------------------------------------------------------------------

assign ramaddr[28]    = sel_zram & ~sel_z3ram0;
assign ramaddr[27]    = sel_zram & (~sel_z3ram1 | cpu_addr[27]);
assign ramaddr[26:23] = (sel_z3ram0 | sel_z3ram1) ? cpu_addr[26:23] : (sel_rtg ? 4'b1110 : {4{sel_dd}});
assign ramaddr[22:19] = {4{sel_dd}} | cpu_addr[22:19];
assign ramaddr[18]    =    sel_dd   | (sel_kicklower & bootrom) | cpu_addr[18];
assign ramaddr[17:16] = {2{sel_dd}} | cpu_addr[17:16];
assign ramaddr[15:1]  = cpu_addr[15:1];

// Same selector Minimig.sv:598 uses to mux ram1 vs ram2 outputs.
assign zram_sel = |ramaddr[28:26];

endmodule
