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
// cdtv_trace — 1024-deep ring buffer of CDTV bridge accesses.
//
// Style mirrors rtl/akiko_bus_trace.v but wider (64 bits per entry) and
// deeper (1024 vs 128) because the CDTV bridge fires on every DMAC/TPI
// access, not just the 256-byte Akiko window. At 1024 entries × 64 bits
// the ring fits in two Cyclone V M10K blocks.
//
// Entry layout (from cdtv_bridge.v trace_data):
//   bits [63:32] : timestamp — free-running 32-bit counter local to this
//                  module (incremented every clk).
//   bits [31:24] : access tag — see cdtv_bridge.v "Trace output" comment.
//                  Bit 7 set = write, clear = read.
//   bits [23:16] : CPU data byte (din[7:0])
//   bits [15: 0] : byte offset within $E9xxxx
//
// Drain protocol: 1 entry = 8 bytes LSB-first over the UIO sub-channel.
// The 9th byte returned in each entry is the "valid" marker — 0xFF means
// a real entry, 0x00 means the ring drained empty (userspace polls until
// it sees 0x00 and stops).
//
// NOTE: This file declares the UIO read port but no drain logic — the
// hps_ext sub-channel allocation is a separate work item. The ring fills
// from cdtv_bridge regardless, so traces accumulate even before the
// userspace drain is wired.
//
//----------------------------------------------------------------------------------

module cdtv_trace
(
	input             clk,
	input             reset,

	// Capture port — from cdtv_bridge.v.
	input             trace_we,
	input      [63:0] trace_data,

	// Free-running timestamp counter to stamp on every entry. Local; not
	// fed externally. Wraps at 2^32 which is ~150s at 28.6 MHz — adequate
	// for boot-time traces.

	// UIO drain port — out of scope to wire up this session, but the
	// port shape mirrors akiko_bus_trace.v so a future hps_ext sub-channel
	// allocation drops in cleanly. uio_dout returns 0 when ring is empty.
	input             uio_cs,
	input             uio_rd,
	output reg  [7:0] uio_dout
);

// Two-block ring. 1024 × 64.
reg [63:0] ring [0:1023];
reg  [9:0] wr_ptr;
reg  [9:0] rd_ptr;
wire       empty = (wr_ptr == rd_ptr);

// Free-running timestamp.
reg [31:0] ts;

// Stage the capture inputs one cycle to break the path from
// cdtv_bridge's combinational decode into the ring write port (same
// pattern as akiko_bus_trace.v).
reg        trace_we_d;
reg [63:0] trace_data_d;

always @(posedge clk) begin
	ts <= ts + 32'd1;
	trace_we_d   <= trace_we;
	trace_data_d <= {ts, trace_data[31:0]};   // overwrite timestamp slot
	if (trace_we_d) begin
		ring[wr_ptr] <= trace_data_d;
		wr_ptr       <= wr_ptr + 10'd1;
	end
	if (reset) begin
		wr_ptr <= 10'd0;
		ts     <= 32'd0;
	end
end

// Drain side — 8 bytes per entry. Byte 8 is the valid marker (0xFF when
// the ring has data, 0x00 when empty). Userspace reads 9 bytes per entry
// and stops on a 0x00 valid byte.
reg [3:0] byte_idx;

always @* begin
	uio_dout = 8'h00;
	if (!empty) begin
		case (byte_idx)
			4'd0: uio_dout = ring[rd_ptr][7:0];
			4'd1: uio_dout = ring[rd_ptr][15:8];
			4'd2: uio_dout = ring[rd_ptr][23:16];
			4'd3: uio_dout = ring[rd_ptr][31:24];
			4'd4: uio_dout = ring[rd_ptr][39:32];
			4'd5: uio_dout = ring[rd_ptr][47:40];
			4'd6: uio_dout = ring[rd_ptr][55:48];
			4'd7: uio_dout = ring[rd_ptr][63:56];
			default: uio_dout = 8'hFF;   // byte 8 = valid marker
		endcase
	end
end

always @(posedge clk) begin
	if (reset) begin
		rd_ptr   <= 10'd0;
		byte_idx <= 4'd0;
	end else if (uio_cs && uio_rd) begin
		if (!empty) begin
			if (byte_idx == 4'd8) begin
				rd_ptr   <= rd_ptr + 10'd1;
				byte_idx <= 4'd0;
			end else begin
				byte_idx <= byte_idx + 4'd1;
			end
		end
	end else if (!uio_cs) begin
		byte_idx <= 4'd0;
	end
end

endmodule
