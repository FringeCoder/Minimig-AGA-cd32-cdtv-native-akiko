// Copyright 2026 (CDTV native-mode HPS bridge)
//
// This file is part of Minimig
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
//----------------------------------------------------------------------------
//
// cdtv_hps_bridge: minimal UIO byte-stream adapter for cdtv_bridge.v.
//
// CDTV userspace command/reply stream over UIO class 0xF800
// (hps_ext.v: io_din[15:9] == 7'b1111100).
//
// Direction model (same convention as akiko_hps_bridge):
//   - UIO read  (0x62 + 0xF800): byte = cmd_in_byte; cmd_in_pop pulses.
//   - UIO write (0x61 + 0xF800): cmd_out_data = uio_din; cmd_out_push pulses.
//
// Sub-channels:
//   - UIO read  (0x62 + 0xF840): bit 0 = stch_ack (the BIOS took the STCH
//     interrupt); the read clears it.
//   - UIO write (0x61 + 0xF840): pulses stch_inject for one clk. The bridge
//     OR's stch_pulse into its TPI ilatch[2] (see cdtv_bridge.v:313), so this
//     is the userspace path for firing a status-change interrupt on disc
//     mount. CDTV BIOS is event-driven and won't advance past the initial
//     STATUS poll without seeing STCH after the disc appears.
//   - UIO write (0x61 + 0xF820): each byte pushes sec_byte_data + pulses
//     sec_byte_push. The bridge enqueues the byte into the
//     8 KB sector staging FIFO; the drain FSM then master-writes it to
//     chip RAM at acr via chipdma_arb.
//   - UIO read  (0x62 + 0xF820): byte = sec_space, the FIFO's free space in
//     32-byte units (0 = full, 255 = empty). This is the CDTV analogue of
//     akiko's 1-byte sec_status read on 0xF500, and it exists for the same
//     reason: the write direction has no handshake, so userspace needs a
//     credit to read before it streams a sector. Without it a stalled drain
//     -- e.g. a save state holding chipdma_arb off for the whole chip RAM
//     dump -- makes cdtv_bridge's overflow guard drop CD bytes silently.
//     A read here has no side effects: hps_ext keeps cdtv_cs and
//     cdtv_cs_stch LOW for this sub-channel, so neither cmd_in_pop nor
//     stch_ack_clr can fire, and sec_byte_push is qualified by uio_wr.
//
// `req` exposes cmd_in_pending so the host-side poll loop can detect when
// a command has been framed and ready to drain.
//
//----------------------------------------------------------------------------

module cdtv_hps_bridge
(
	input             clk,
	input             reset,

	// UIO byte stream from hps_ext
	input             uio_cs,
	input             uio_cs_stch,    // STCH-inject sub-channel
	input             uio_cs_sec,     // sector-push sub-channel
	input             uio_wr,
	input             uio_rd,
	input       [7:0] uio_din,
	output     [15:0] uio_dout,

	// cdtv_bridge cmd/reply port
	input             cmd_in_pending,
	input       [7:0] cmd_in_byte,
	output            cmd_in_pop,
	output            cmd_out_push,
	output      [7:0] cmd_out_data,

	// Sector byte push to cdtv_bridge.sec_byte_push/data (1-clk per UIO byte).
	output            sec_byte_push,
	output      [7:0] sec_byte_data,

	// Sector FIFO free-space credit from cdtv_bridge, presented on every
	// read strobe of the sector sub-channel. Combinational status, exactly
	// like akiko_hps_bridge's sec_status.
	input       [7:0] sec_space,

	// STCH-inject pulse to cdtv_bridge.stch_pulse (1-clk on UIO write).
	output            stch_inject,

	// STCH ack flag from cdtv_bridge, readable on the STCH sub-channel.
	input             stch_ack,
	output            stch_ack_clr,

	// Status bit for hps_ext 0x63 word
	output            req
);

assign cmd_in_pop    = uio_rd & uio_cs;
assign cmd_out_push  = uio_wr & uio_cs;
assign cmd_out_data  = uio_din;
// Read mux precedence mirrors hps_ext's mutually-exclusive cs decode:
// at most one of cs / cs_stch / cs_sec is asserted per transaction.
assign uio_dout      = uio_cs_stch ? {15'h0000, stch_ack} :
                       uio_cs_sec  ? {8'h00,    sec_space} :
                                     {8'h00,    cmd_in_byte};
assign sec_byte_push = uio_wr & uio_cs_sec;
assign sec_byte_data = uio_din;
assign stch_inject   = uio_wr & uio_cs_stch;
assign stch_ack_clr  = uio_rd & uio_cs_stch;
assign req           = cmd_in_pending;

endmodule
