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
// Two sub-channels share UIO address class 0xF400, distinguished by io_din[8]
// captured on byte_cnt==1 in hps_ext:
//   - cs only           (io_din[8] == 0): M3 command-out / response-in stream.
//   - cs && cs_sec      (io_din[8] == 1): M4 sector-data-in / status-out stream.
//
// Direction model (same for both sub-channels):
//   - Main read  (UIO cmd 0x62) -> hps_ext pulses uio_rd per strobe; bridge
//     presents uio_dout the same cycle. For the cmd channel, that's the
//     current byte at hps_cmd_rd_ptr; for the sec channel, it's the 1-byte
//     sec_status (cdrom_sector_counter). Pop is forwarded to akiko on the cmd
//     channel only.
//
//   - Main write (UIO cmd 0x61) -> hps_ext pulses uio_wr per strobe; bridge
//     forwards uio_din[7:0] as result_byte (cmd channel) or sec_byte (sec
//     channel) and pulses the matching push.
//
// Transaction end (io_uio drop): at the cycle io_uio drops, both uio_cs and
// uio_cs_sec drop together (hps_ext clears them). cs_d/cs_sec_d hold the
// prior values, so we know which sub-channel the just-ended transaction used.
// Pulses one of cmd_done / result_done / sec_done accordingly.
//
//----------------------------------------------------------------------------------

module akiko_hps_bridge
(
	input             clk,
	input             reset,

	// UIO byte stream from hps_ext (cs/cs_sec held while io_uio is asserted,
	// dropped together on io_uio fall).
	input             uio_cs,        // akiko_cs (any 0xF400 transaction)
	input             uio_cs_sec,    // sec sub-channel (io_din[8] from byte_cnt==1)
	input             uio_wr,        // pulse: Main wrote one data byte
	input             uio_rd,        // pulse: Main read one data byte
	input       [7:0] uio_din,
	output      [7:0] uio_dout,

	// Akiko cmd/response port (M3)
	input             cmd_pending,
	input       [7:0] cmd_byte,
	output            cmd_pop,
	output            cmd_done,
	output            result_push,
	output      [7:0] result_byte,
	output            result_done,

	// Akiko sector port (M4)
	input             sec_req,
	input       [7:0] sec_status,
	output            sec_push,
	output      [7:0] sec_byte,
	output            sec_done,

	// Status bits for hps_ext 0x63 status word
	output            req,           // bit [11]: framed command waiting
	output            sec_req_out    // bit [10]: sector needed
);

reg cs_d;
reg cs_sec_d;
reg saw_read;
reg saw_write;

always @(posedge clk) begin
	if (reset) begin
		cs_d      <= 1'b0;
		cs_sec_d  <= 1'b0;
		saw_read  <= 1'b0;
		saw_write <= 1'b0;
	end else begin
		cs_d     <= uio_cs;
		cs_sec_d <= uio_cs_sec;
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

// Sub-channel routing.
//   cmd channel: uio_cs && !uio_cs_sec
//   sec channel: uio_cs &&  uio_cs_sec
// (uio_rd / uio_wr only pulse while uio_cs is asserted, so the cs gate is
// formally redundant but kept explicit.)
assign cmd_pop     = uio_rd & uio_cs & ~uio_cs_sec;
assign result_push = uio_wr & uio_cs & ~uio_cs_sec;
assign result_byte = uio_din;

assign sec_push    = uio_wr & uio_cs &  uio_cs_sec;
assign sec_byte    = uio_din;

// Read-mux: sec_status on the sec channel, cmd_byte on the cmd channel.
assign uio_dout    = uio_cs_sec ? sec_status : cmd_byte;

// done pulses fire on transaction end, routed by the sub-channel the
// transaction ran on (captured in cs_sec_d the cycle before the drop).
assign cmd_done    = xfer_end & saw_read  & ~cs_sec_d;
assign result_done = xfer_end & saw_write & ~cs_sec_d;
assign sec_done    = xfer_end &              cs_sec_d;

assign req         = cmd_pending;
assign sec_req_out = sec_req;

endmodule
