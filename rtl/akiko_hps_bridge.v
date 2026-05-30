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
// Three sub-channels share UIO address class 0xF400, distinguished by extra
// bits captured on byte_cnt==1 in hps_ext:
//   - cs only             (no extra bits)        : M3 command-out / response-in.
//   - cs && cs_sec        (io_din[8] == 1, 0xF500): M4 sector-data / status.
//   - cs && cs_nvr        (io_din[6] == 1, 0xF440): NVRAM save-dump (read 1024
//                                                   bytes; auto-clears the
//                                                   dirty flag on transaction
//                                                   end).
//
// cs_sec, cs_nvr (and future cs_aud) are mutually exclusive — hps_ext picks
// at most one based on the highest set bit. The plain cs is asserted alongside
// any sub-channel selector so legacy demux logic stays unchanged.
//
// Direction model:
//   - Main read  (UIO cmd 0x62) -> hps_ext pulses uio_rd per strobe; bridge
//     presents uio_dout the same cycle. For the cmd channel, that's the
//     current byte at hps_cmd_rd_ptr; for the sec channel, it's the 1-byte
//     sec_status (cdrom_sector_counter); for the nvr channel, it's the
//     1024-byte BRAM image, auto-incrementing through nvr_addr.
//
//   - Main write (UIO cmd 0x61) -> hps_ext pulses uio_wr per strobe; bridge
//     forwards uio_din[7:0] as result_byte (cmd channel) or sec_byte (sec
//     channel). The bridge does NOT handle NVRAM writes — those come in via
//     hps_io.ioctl_download, plumbed as nvr_load_we/addr/din from Minimig.sv
//     directly into akiko_nvram (bypassing this bridge entirely).
//
// Transaction end (io_uio drop): at the cycle io_uio drops, all cs_* signals
// drop together (hps_ext clears them). cs_d/cs_sec_d/cs_nvr_d hold the prior
// values so we know which sub-channel the just-ended transaction used. Pulses
// one of cmd_done / result_done / sec_done / nvr_done accordingly.
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
	input             uio_cs_nvr,    // NVRAM save-dump sub-channel (io_din[6])
	input             uio_cs_subcode,// subcode push sub-channel (io_din[4], 0xF410)
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

	// Akiko subcode push port (W-only; 96 bytes/frame during CDDA play)
	output            subcode_push,
	output      [7:0] subcode_byte,
	output            subcode_done,

	// NVRAM save-dump port. Bridge drives nvr_addr (auto-incrementing read
	// counter) into akiko_nvram, reads back nvr_dout (1-cycle BRAM latency),
	// and pulses nvr_clear_dirty on end of a read burst (= save complete).
	// The LOAD path is no longer here — it goes through hps_io.ioctl_download
	// directly to akiko_nvram's load port (see Minimig.sv NVR_LOAD_INDEX).
	output      [9:0] nvr_addr,
	input       [7:0] nvr_dout,
	output            nvr_clear_dirty,
	output            nvr_done,
	input             nvr_dirty,     // from akiko_nvram

	// Phase 18: rx_busy = receive engine has a queued or in-flight response.
	input             rx_busy,

	// Status bits for hps_ext 0x63 status word
	output            req,           // bit [11]: framed command waiting
	output            sec_req_out,   // bit [10]: sector needed
	output            rx_busy_out,   // bit [9]:  RX engine busy (Phase 18)
	output            nvr_dirty_out  // bit [7]:  NVRAM dirty
);

reg cs_d;
reg cs_sec_d;
reg cs_nvr_d;
reg cs_subcode_d;
reg saw_read;
reg saw_write;

// NVRAM read-burst address counter. Reset to 0 on cs_nvr rising edge,
// auto-incremented on each uio_rd while cs_nvr is asserted. BRAM read
// has 1-cycle latency; cs_nvr is asserted many clk cycles before the
// first strobe lands (Main has to send the 4 address bytes first), so
// nvr_dout is valid by the time it's needed.
reg [9:0] nvr_addr_cnt;

// cmd-channel sub-selector: only true when no extra cs is set.
wire cs_cmd = uio_cs & ~uio_cs_sec & ~uio_cs_nvr & ~uio_cs_subcode;

always @(posedge clk) begin
	if (reset) begin
		cs_d         <= 1'b0;
		cs_sec_d     <= 1'b0;
		cs_nvr_d     <= 1'b0;
		cs_subcode_d <= 1'b0;
		saw_read     <= 1'b0;
		saw_write    <= 1'b0;
		nvr_addr_cnt <= 10'd0;
	end else begin
		cs_d         <= uio_cs;
		cs_sec_d     <= uio_cs_sec;
		cs_nvr_d     <= uio_cs_nvr;
		cs_subcode_d <= uio_cs_subcode;
		if (!uio_cs) begin
			saw_read  <= 1'b0;
			saw_write <= 1'b0;
		end else begin
			if (uio_rd) saw_read  <= 1'b1;
			if (uio_wr) saw_write <= 1'b1;
		end
		// NVRAM addr counter: rearm at cs_nvr rising edge, advance per
		// uio_rd (read = save dump). Writes do not happen on this
		// channel anymore.
		if (uio_cs_nvr & ~cs_nvr_d) begin
			nvr_addr_cnt <= 10'd0;
		end else if (uio_rd & uio_cs_nvr) begin
			nvr_addr_cnt <= nvr_addr_cnt + 10'd1;
		end
	end
end

wire xfer_end = cs_d & ~uio_cs;

// Sub-channel routing.
//   cmd channel: uio_cs && !cs_sec && !cs_nvr
//   sec channel: uio_cs &&  cs_sec
//   nvr channel: uio_cs &&  cs_nvr   (read-only here — load goes via ioctl_download)
assign cmd_pop         = uio_rd & cs_cmd;
assign result_push     = uio_wr & cs_cmd;
assign result_byte     = uio_din;

assign sec_push        = uio_wr & uio_cs &  uio_cs_sec;
assign sec_byte        = uio_din;

assign subcode_push    = uio_wr & uio_cs &  uio_cs_subcode;
assign subcode_byte    = uio_din;
assign subcode_done    = xfer_end &              cs_subcode_d;

assign nvr_addr        = nvr_addr_cnt;
assign nvr_clear_dirty = xfer_end & saw_read & cs_nvr_d;

// Read-mux precedence (mirrors hps_ext mutex):
//   nvr_dout > sec_status > cmd_byte
assign uio_dout        = uio_cs_nvr ? nvr_dout    :
                         uio_cs_sec ? sec_status  :
                                      cmd_byte;

// done pulses fire on transaction end, routed by the sub-channel the
// transaction ran on (captured in cs_*_d the cycle before the drop).
assign cmd_done        = xfer_end & saw_read  & ~cs_sec_d & ~cs_nvr_d & ~cs_subcode_d;
assign result_done     = xfer_end & saw_write & ~cs_sec_d & ~cs_nvr_d & ~cs_subcode_d;
assign sec_done        = xfer_end &              cs_sec_d;
assign nvr_done        = xfer_end &              cs_nvr_d;

assign req             = cmd_pending;
assign sec_req_out     = sec_req;
assign rx_busy_out     = rx_busy;
assign nvr_dirty_out   = nvr_dirty;

endmodule
