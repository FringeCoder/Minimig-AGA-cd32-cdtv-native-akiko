// Copyright 2026 (CDTV native-mode bridge)
//
// This file is part of Minimig
//
// Minimig is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// Minimig is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
//----------------------------------------------------------------------------------
//
// cdtv_nvram — 16 KB M10K-backed CDTV battery RAM at $DC8000-$DCFFFF.
//
// Per spec research/docs/cdtv-bridge-spec.md section 5:
//   * Only 14 address bits are decoded (CDTV_NVRAM_MASK = 0x3FFF) — the
//     upper 16 KB of the $DC8000-$DCFFFF window mirrors the lower 16 KB
//     even though the chip is documented as 32 KB. WinUAE source is the
//     truth (spec section 5.3 + section 10 contradiction #1).
//   * Byte-wide on the wire; the CIA clock-bank dispatcher decomposes any
//     word/long access into byte transactions, so this module sees one
//     byte access at a time (spec section 5.2). hwr selects even byte
//     (data[15:8]), lwr selects odd byte (data[7:0]).
//   * Persistence: dirty flag set on any write; userspace will drain
//     via a UIO sub-channel in a future session. For now hps_save_dout
//     is wired to 0 — userspace plumbing is out of scope per spec.
//
// Storage: single altsyncram, width=16, depth=8192 (= 16 KB), with
// byte-enable on Port A. Same pattern as akiko_nvram.v but byteena lets
// us collapse two byte-wide BRAMs into one block. Maps to M10K (~13
// blocks on Cyclone V) — verified versus the prior `reg [7:0] mem[]`
// inference which Quartus collapsed to ALMs (87,624 ALMs / 209% of
// the device — fit failure 2026-05-20 with 287% total utilisation).
//
//----------------------------------------------------------------------------------

module cdtv_nvram
(
	input             clk,
	input             reset,

	// CPU port — chip-bus convention. addr is the word address, hwr/lwr
	// disambiguate the byte slot. Spec section 5.2: the CIA dispatcher
	// only ever issues byte accesses, so exactly one of hwr/lwr fires
	// per write; rd fires for either byte read and we return both bytes
	// in dout so the chip-bus mux at minimig.v selects via UDS/LDS.
	input             sel,
	input      [23:1] addr,            // word address from chip bus (bits 13:1 used)
	input      [15:0] din,
	output     [15:0] dout,            // {even_byte, odd_byte}
	input             rd,
	input             hwr,             // upper / even-byte write strobe
	input             lwr,             // lower / odd-byte write strobe

	// HPS load port — driven by hps_io.ioctl_download (canonical pattern,
	// analogous to akiko_nvram). Boot-time only, before BIOS touches NVR,
	// so no contention with the CPU port. Byte-wide so the load addr is
	// the flat 14-bit byte address (bit 0 picks even=high or odd=low).
	input      [13:0] hps_load_addr,
	input       [7:0] hps_load_din,
	input             hps_load_we,

	// HPS save port — userspace will drain via UIO read burst. Wired off
	// for this pass; future session adds the drain logic. Port retained
	// so the minimig.v port list doesn't change.
	input      [13:0] hps_save_addr,   // unused this pass
	output      [7:0] hps_save_dout,   // tied to 0 — see banner

	// Dirty flag — set on any CPU write, cleared on userspace drain
	// completion (clear_dirty pulse from a future UIO bridge).
	output reg        dirty = 1'b0,
	input             clear_dirty
);

// Word address into the 8192-word (16 KB) BRAM. addr[14] drops on the
// floor — that's the spec §5.3 16 KB mirror inside the 32 KB window.
wire [12:0] cpu_waddr = addr[13:1];
wire [12:0] hps_waddr = hps_load_addr[13:1];
wire        hps_byte  = hps_load_addr[0];          // 0 = even (high), 1 = odd (low)

// Write mux. HPS load wins because it only fires before BIOS boots.
// CPU writes use the chip-bus byte enables. byteena[1] = data[15:8] = even
// (hwr), byteena[0] = data[7:0] = odd (lwr) — Quartus altsyncram convention.
wire [12:0] write_addr    = hps_load_we ? hps_waddr : cpu_waddr;
wire [15:0] write_data    = hps_load_we ? {hps_load_din, hps_load_din} : din;
wire [1:0]  write_byteena = hps_load_we ? {~hps_byte, hps_byte}
                                        : {sel & hwr, sel & lwr};
wire        write_we      = hps_load_we | (sel & (hwr | lwr));

// Single dual-port altsyncram. Port A = writes, Port B = CPU reads.
// Forced M10K via ram_block_type defparam — see akiko_nvram.v:179 for
// the pattern Quartus 17.0 requires.
altsyncram nvram_inst (
	.address_a      (write_addr),
	.clock0         (clk),
	.data_a         (write_data),
	.byteena_a      (write_byteena),
	.wren_a         (write_we),
	.address_b      (cpu_waddr),
	.q_b            (dout),
	.aclr0          (1'b0),
	.aclr1          (1'b0),
	.addressstall_a (1'b0),
	.addressstall_b (1'b0),
	.byteena_b      (1'b1),
	.clock1         (1'b1),
	.clocken0       (1'b1),
	.clocken1       (1'b1),
	.clocken2       (1'b1),
	.clocken3       (1'b1),
	.data_b         (16'h0000),
	.eccstatus      (),
	.q_a            (),
	.rden_a         (1'b1),
	.rden_b         (1'b1),
	.wren_b         (1'b0)
);
defparam
	nvram_inst.address_aclr_b                  = "NONE",
	nvram_inst.address_reg_b                   = "CLOCK0",
	nvram_inst.clock_enable_input_a            = "BYPASS",
	nvram_inst.clock_enable_input_b            = "BYPASS",
	nvram_inst.clock_enable_output_b           = "BYPASS",
	nvram_inst.intended_device_family          = "Cyclone V",
	nvram_inst.lpm_type                        = "altsyncram",
	nvram_inst.numwords_a                      = 8192,
	nvram_inst.numwords_b                      = 8192,
	nvram_inst.operation_mode                  = "DUAL_PORT",
	nvram_inst.outdata_aclr_b                  = "NONE",
	nvram_inst.outdata_reg_b                   = "UNREGISTERED",
	nvram_inst.power_up_uninitialized          = "FALSE",
	nvram_inst.ram_block_type                  = "M10K",
	nvram_inst.read_during_write_mode_mixed_ports = "OLD_DATA",
	nvram_inst.widthad_a                       = 13,
	nvram_inst.widthad_b                       = 13,
	nvram_inst.width_a                         = 16,
	nvram_inst.width_b                         = 16,
	nvram_inst.width_byteena_a                 = 2;

// HPS save port deferred to future session — tie off so the port list
// in minimig.v stays valid.
assign hps_save_dout = 8'h00;

// Dirty flag — set on any CPU write (hps_load_we does NOT set dirty,
// per the akiko_nvram convention: loading saved state must not provoke
// an immediate re-save of what we just loaded). Cleared on the drain
// completion pulse from userspace.
always @(posedge clk) begin
	if (sel & (hwr | lwr)) dirty <= 1'b1;
	else if (clear_dirty)  dirty <= 1'b0;
end

endmodule
