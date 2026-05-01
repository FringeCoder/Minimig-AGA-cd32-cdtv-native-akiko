// Copyright 2026 (CD32 native-mode HPS bridge)
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
// akiko_hps_bridge: adapts the hps_ext UIO byte-stream to the akiko HPS port.
//
// Direction model:
//   - Main read  (UIO cmd 0x62, akiko_cs=1)  -> drains command bytes from akiko.
//     hps_ext pulses uio_rd per strobe; bridge asserts cmd_pop the same cycle so
//     akiko advances its read pointer by NBA on the next clock; the byte already
//     latched on uio_dout this strobe is the byte at the *current* rd_ptr.
//
//   - Main write (UIO cmd 0x61, akiko_cs=1)  -> pushes response bytes to akiko.
//     hps_ext pulses uio_wr per strobe; bridge asserts result_push the same cycle
//     and forwards uio_din[7:0] as result_byte so akiko stores it at the current
//     wr_ptr by NBA on the next clock.
//
// The transaction end is hps_ext clearing uio_active (io_uio drop / DisableIO).
// Bridge tracks whether the transaction was a read or a write and pulses
// cmd_done or result_done on the falling edge of (uio_active & uio_cs).
//
// Address class decoded by hps_ext via io_din[15:9] == 7'b1111_010 (0xF400).
//
//----------------------------------------------------------------------------------

module akiko_hps_bridge
(
	input             clk,
	input             reset,

	// UIO byte stream from hps_ext (akiko_cs latched on byte_cnt==1, cleared
	// when io_uio falls — so cs alone is the activity signal for this class).
	input             uio_cs,        // akiko_cs from hps_ext
	input             uio_wr,        // pulse: Main wrote one data byte (byte_cnt >= 3)
	input             uio_rd,        // pulse: Main read one data byte
	input       [7:0] uio_din,       // low byte of io_din from Main
	output      [7:0] uio_dout,      // low byte for hps_ext to put on io_dout

	// Akiko-side bridge port (matches akiko.v hps_* signature)
	input             cmd_pending,
	input       [7:0] cmd_byte,
	output            cmd_pop,
	output            cmd_done,
	output            result_push,
	output      [7:0] result_byte,
	output            result_done,

	// Status bit for the hps_ext 0x63 status word
	output            req
);

reg cs_d;
reg saw_read;
reg saw_write;

always @(posedge clk) begin
	if (reset) begin
		cs_d      <= 1'b0;
		saw_read  <= 1'b0;
		saw_write <= 1'b0;
	end else begin
		cs_d <= uio_cs;
		if (!uio_cs) begin
			saw_read  <= 1'b0;
			saw_write <= 1'b0;
		end else begin
			if (uio_rd) saw_read  <= 1'b1;
			if (uio_wr) saw_write <= 1'b1;
		end
	end
end

wire xfer_end = cs_d & ~uio_cs;

assign cmd_pop     = uio_rd;
assign result_push = uio_wr;
assign result_byte = uio_din;
assign uio_dout    = cmd_byte;
assign cmd_done    = xfer_end & saw_read;
assign result_done = xfer_end & saw_write;
assign req         = cmd_pending;

endmodule
