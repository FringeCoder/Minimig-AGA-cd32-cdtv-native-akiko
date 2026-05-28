// SPDX-License-Identifier: GPL-3.0-or-later
//
// z2_trace -- 256-entry ring buffer of CPU accesses through memory_router.
// Drained over UIO via a new sub-channel class 7'b1111101 (i.e. io_din[15:9]
// == 0x7D — byte-1 high half of 0xFA00, matching the akiko_cs / cdtv_cs
// allocation pattern).
//
// Purpose: instrument the Z2-enabled hang where mem=0x33 (Z2 8MB) wedges
// the CD32 BIOS at splash while mem=0x03 (no fast) and mem=0x83 (Z3 256MB)
// boot cleanly on the same RBF. The trace captures the CPU's view of fast
// RAM activity from AC enumeration through the hang point.
//
// Trigger sources (capture one entry per event, with a 1-bit event-type
// flag identifying which fired):
//   - Rising edge of z2ram_ena  -> AC-done sentinel (1 entry, ev=01)
//   - Rising edge of ramsel && (sel_z2ram | sel_z3ram0 | sel_z3ram1 |
//     sel_kickram | sel_kicklower)  -> per-access entry (ev=00)
//   - ramsel asserted but !ramready for >256 sysclks  -> stall sentinel
//     (ev=10; one entry per N-cycle window, capped at 1 entry to avoid
//     flooding)
//
// One trace entry = 16 bytes, drained LSB-first:
//   byte 0..3 : timestamp[31:0]  (free-running clk counter, wraps)
//   byte 4..7 : cpu_addr[31:0]
//   byte 8    : flags  = {wr, ramready, cpustate[1:0], cchip, ckick,
//                          uds_in, lds_in}
//   byte 9    : sels   = {sel_z2ram, sel_z3ram0, sel_z3ram1, sel_kickram,
//                          sel_chipram, sel_dd, sel_rtg, z2ram_ena}
//   byte 10..13: {3'b0, ramaddr[28:1], 1'b0}   (28-bit ramaddr packed)
//   byte 14..15: ramdat[15:0]   (read return / write data lane)
//
// The "empty" condition is reported by returning 0x00 for byte 0 of a
// non-entry (the userspace drainer treats a zero-timestamp as ring-empty
// only after the ring is reset; once any activity is logged, drain reads
// every 16 bytes contiguously, treating a wrapped read_ptr == write_ptr
// as empty).
//
// 2026-05-27: ring capacity is 256 entries (= 4 KB, one M10K block) — big
// enough to capture the full AC enumeration window plus the first dozens
// of CPU fast-RAM accesses post-AC, which is where the hang is suspected
// to surface (BIOS AddMemList → first read-back of the just-added Z2
// memory list nodes).

module z2_trace #(parameter CAPTURE_ENABLE = 1)(
	input             clk,
	input             reset,

	// Bus snapshot from cpu_wrapper's memory_router context.
	input             ramsel,           // cpu_wrapper.v:123
	input             ramready,         // ddram/sdram handshake
	input      [31:0] cpu_addr,
	input      [28:1] ramaddr,
	input      [15:0] ramdat,           // sel_rtg-flipped lane (cpu_wrapper.v:176)
	input             wr,
	input             uds_in,
	input             lds_in,
	input       [1:0] cpustate,
	input             cchip,
	input             ckick,

	input             sel_z2ram,
	input             sel_z3ram0,
	input             sel_z3ram1,
	input             sel_kickram,
	input             sel_kicklower,
	input             sel_chipram,
	input             sel_dd,
	input             sel_rtg,

	input             z2ram_ena,

	// UIO read port. uio_rd pulses pop one byte (16-byte entries
	// auto-advance via internal byte_idx).
	input             uio_cs_trace,
	input             uio_rd,
	output reg  [7:0] uio_dout
);

// 256 entries x 128 bits = 4 KB ring buffer.
reg [127:0] ring [0:255];
reg   [7:0] wr_ptr;
reg   [7:0] rd_ptr;
wire        empty = (wr_ptr == rd_ptr);

// Free-running timestamp counter (wraps every 2^32 sysclks ~= 43 s @ 100 MHz —
// good enough for boot-to-hang capture; userspace can detect wrap by
// non-monotonic timestamps).
reg  [31:0] tstamp;
always @(posedge clk) tstamp <= reset ? 32'b0 : tstamp + 1'b1;

// Edge detect on ramsel — capture once per access begin (multi-cycle
// ramsel during slow DDR3 fills must not flood the ring).
reg ramsel_d;
always @(posedge clk) ramsel_d <= ramsel;
wire ramsel_rise = ramsel & ~ramsel_d;

// Edge detect on z2ram_ena rising — AC-done sentinel.
reg z2ena_d;
always @(posedge clk) z2ena_d <= z2ram_ena;
wire z2ena_rise = z2ram_ena & ~z2ena_d;

// Stall sentinel: ramsel asserted for >256 cycles without ramready.
reg [8:0] stall_cnt;
always @(posedge clk) begin
	if (reset || !ramsel || ramready) stall_cnt <= 9'b0;
	else if (!stall_cnt[8])           stall_cnt <= stall_cnt + 1'b1;
end
wire stall_hit = (stall_cnt == 9'h100) && ramsel && !ramready;
// One-shot guard so a single stall doesn't fill the ring with copies.
reg stall_fired;
always @(posedge clk) begin
	if (reset || !ramsel || ramready) stall_fired <= 1'b0;
	else if (stall_hit)               stall_fired <= 1'b1;
end

// Fast-RAM access predicate -- only capture cycles that route to fast RAM
// via ramsel. sel_kicklower never contributes to ramsel itself (cpu_wrapper
// line 123 excludes it) so omit from the trigger.
wire fast_hit = ramsel_rise &
	(sel_z2ram | sel_z3ram0 | sel_z3ram1 | sel_kickram);

// Combined capture enable. Event-type bits in flags are derived from the
// trigger source so userspace can sort entries.
wire cap_ev_access   = fast_hit;
wire cap_ev_acdone   = z2ena_rise;
wire cap_ev_stall    = stall_hit & ~stall_fired;
wire cap_en          = CAPTURE_ENABLE & (cap_ev_access | cap_ev_acdone | cap_ev_stall);

// 2-bit event type for the high two bits of the flags byte (overrides
// uds/lds when set non-zero).
wire [1:0] ev_type =
	cap_ev_acdone ? 2'b01 :
	cap_ev_stall  ? 2'b10 :
	                2'b00;

// Pack one entry. byte 8 keeps the full 8-bit flags so wr/ramready survive;
// ev_type lives in the high two bits of the ramaddr_pad word (byte 13) so
// each event still self-identifies without sacrificing CPU-state bits.
wire [7:0] flags_byte = {wr, ramready, cpustate, cchip, ckick, uds_in, lds_in};
wire [7:0] sels_byte  = {sel_z2ram, sel_z3ram0, sel_z3ram1, sel_kickram,
                         sel_chipram, sel_dd, sel_rtg, z2ram_ena};
// ramaddr is [28:1] = 28 bits. Pack into 32 bits as
//   { 2'b0, ev_type[1:0], ramaddr[28:1] }
// → byte 10..12 = ramaddr LSBs, byte 13 = {2'b0, ev_type, ramaddr[28:25]}.
wire [31:0] ramaddr_pad = {2'b0, ev_type, ramaddr};
wire [127:0] entry = {
	ramdat,        // byte 14..15
	ramaddr_pad,   // byte 10..13 (ev_type in byte 13 bits[5:4])
	sels_byte,     // byte 9
	flags_byte,    // byte 8
	cpu_addr,      // byte 4..7
	tstamp         // byte 0..3
};

always @(posedge clk) begin
	if (reset) begin
		wr_ptr <= 8'b0;
	end
	else if (cap_en) begin
		ring[wr_ptr] <= entry;
		wr_ptr       <= wr_ptr + 1'b1;
	end
end

// Drain side: 16 bytes per entry, byte_idx[3:0] selects byte. Pop entry
// (rd_ptr advances) after byte 15.
reg [3:0] byte_idx;

always @(*) begin
	if (empty) begin
		uio_dout = 8'h00;
	end else begin
		case (byte_idx)
			4'h0: uio_dout = ring[rd_ptr][  7:  0];
			4'h1: uio_dout = ring[rd_ptr][ 15:  8];
			4'h2: uio_dout = ring[rd_ptr][ 23: 16];
			4'h3: uio_dout = ring[rd_ptr][ 31: 24];
			4'h4: uio_dout = ring[rd_ptr][ 39: 32];
			4'h5: uio_dout = ring[rd_ptr][ 47: 40];
			4'h6: uio_dout = ring[rd_ptr][ 55: 48];
			4'h7: uio_dout = ring[rd_ptr][ 63: 56];
			4'h8: uio_dout = ring[rd_ptr][ 71: 64];
			4'h9: uio_dout = ring[rd_ptr][ 79: 72];
			4'hA: uio_dout = ring[rd_ptr][ 87: 80];
			4'hB: uio_dout = ring[rd_ptr][ 95: 88];
			4'hC: uio_dout = ring[rd_ptr][103: 96];
			4'hD: uio_dout = ring[rd_ptr][111:104];
			4'hE: uio_dout = ring[rd_ptr][119:112];
			4'hF: uio_dout = ring[rd_ptr][127:120];
		endcase
	end
end

always @(posedge clk) begin
	if (reset) begin
		byte_idx <= 0;
		rd_ptr   <= 0;
	end
	else if (uio_cs_trace && uio_rd) begin
		if (!empty) begin
			if (byte_idx == 4'hF) begin
				rd_ptr   <= rd_ptr + 1'b1;
				byte_idx <= 0;
			end else begin
				byte_idx <= byte_idx + 1'b1;
			end
		end
	end
	else if (!uio_cs_trace) begin
		// Reset byte index outside a transaction so a partial read doesn't
		// leave us mid-entry.
		byte_idx <= 0;
	end
end

endmodule
