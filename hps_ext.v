//
// hps_ext for Minimig
//
// Copyright (c) 2020 Alexey Melnikov
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////

module hps_ext
(
	input             clk_sys,
	inout      [35:0] EXT_BUS,

	output            io_strobe,
	output            io_fpga,
	output            io_uio,
	output     [15:0] io_din,
	input      [15:0] fpga_dout,

	input      [15:0] ide_din,
	output reg [15:0] ide_dout,
	output reg  [4:0] ide_addr,
	output reg        ide_rd,
	output reg        ide_wr,
	input       [5:0] ide_req,

	output reg  [2:0] mouse_buttons,
	output reg        kbd_mouse_level,
	output reg  [1:0] kbd_mouse_type,
	output reg  [7:0] kbd_mouse_data,

	input      [11:0] scr_hbl_l,
	input      [11:0] scr_hbl_r,
	input      [11:0] scr_hsize,
	input      [11:0] scr_vbl_t,
	input      [11:0] scr_vbl_b,
	input      [11:0] scr_vsize,
	input       [6:0] scr_flg,
	input       [1:0] scr_res,

	output reg [11:0] shbl_l,
	output reg [11:0] shbl_r,
	output reg [11:0] svbl_t,
	output reg [11:0] svbl_b,
	output reg        sset,

	input             cdda_req,
	output reg        cdda_wr,
	output reg [15:0] cdda_dout,

	// Akiko bridge (CD32 native mode). Address class 0xF400 = io_din[15:9]==7'b1111_010.
	// Sub-channels share the class, picked from extra bits captured on
	// byte_cnt==1 (mutually exclusive — at most one extra bit set per txn):
	//   no extra bits  -> cmd/result stream (M3)             0xF400
	//   io_din[8]=1    -> sector data stream (M4)            0xF500
	//   io_din[6]=1    -> NVRAM save-dump (read-only)        0xF440
	// Sec and Nvr both ride alongside akiko_cs so the bridge can mux on the
	// sub-channel cs flag.
	//
	// NVRAM LOAD (disk → BRAM) does NOT come through this UIO path. It uses
	// the canonical hps_io.ioctl_download mechanism wired directly from
	// Minimig.sv into akiko_nvram's load port (NVR_LOAD_INDEX). This bridge
	// only handles the save-dump (BRAM → disk) read direction.
	input      [15:0] akiko_din,
	output reg [15:0] akiko_dout,
	output reg        akiko_wr,
	output reg        akiko_rd,
	output reg        akiko_cs,
	output reg        akiko_cs_sec,
	output reg        akiko_cs_nvr,    // NVRAM save-dump sub-channel
	output reg        akiko_cs_subcode,// subcode push sub-channel (io_din[4], 0xF410)
	input             akiko_req,
	input             akiko_sec_req,
	input             akiko_rx_busy,
	input             akiko_nvr_dirty, // status word bit 7

	// CDTV bridge — UIO class 0xF800 = io_din[15:9] == 7'b1111100.
	// Sub-channels share the class:
	//   no extra bits  -> cmd byte stream (R/W cmd_in_fifo / cmd_out_fifo)  0xF800
	//   io_din[5]=1    -> sector byte push (W-only; M2 phase-1b)            0xF820
	//   io_din[6]=1    -> STCH inject (W-only; any write pulses stch)       0xF840
	// The STCH sub-channel lets userspace fire the CDTV
	// status-change interrupt on disc mount — the BIOS is event-driven and
	// without this it never advances past the initial 0x81 STATUS poll.
	// The sector-push sub-channel: userspace pushes raw CHD
	// sector bytes via 0xF820 and the bridge's drain FSM writes them to
	// chip RAM at acr via chipdma_arb's cdtv master port.
	input      [15:0] cdtv_din,
	output reg [15:0] cdtv_dout,
	output reg        cdtv_wr,
	output reg        cdtv_rd,
	output reg        cdtv_cs,
	output reg        cdtv_cs_sec,     // sector-push sub-channel
	output reg        cdtv_cs_stch,    // STCH-inject sub-channel
	output reg        cdtv_cs_nvr,     // battery-RAM sub-channel
	output reg        cdtv_cs_card,    // memory-card sub-channel
	input             cdtv_req,        // bit 6 of status word
	input             cdtv_nvr_dirty,  // bit 12 of status word
	input             cdtv_card_dirty, // bit 13 of status word

	// Save state diagnostics -- UIO class 0xF600 = io_din[15:9] == 7'b1111011.
	//
	// A read-only status window, eight 16-bit words wide, assembled in
	// Minimig.sv from ss_ctrl's diagnostic ports and its own outcome
	// synchronisers. support/minimig/minimig_ssdiag.cpp polls it and logs
	// every change to /tmp/ss_dbg.log.
	//
	// It deliberately does NOT ride the info_req / UIO_INFO_GET path the OSD
	// toast uses, because that path is one of the things being diagnosed. A
	// window that shared it could not tell "the core never raised the
	// request" from "the core raised it and no toast appeared", which is the
	// first question it has to answer.
	//
	// 7'b1111011 is the one free class in the range: ide is 1111000, cdda
	// 1111001, akiko 1111010 and cdtv 1111100.
	input     [127:0] ss_diag,

	// Live memory peek. Same UIO class as the diagnostics (0xF600) with
	// io_din[5] as the sub-channel bit, so it costs no new class and cannot be
	// confused with the read-only status window.
	//
	// The request rides in the 32-bit UIO address the host already sends: the
	// class word carries the high eight address bits in its spare bits, the
	// second word carries the low sixteen. One transaction asks; a later one
	// reads the answer back, so the host never has to wait on the SDRAM
	// inside a transaction.
	output reg [24:1] ss_peek_addr,   // [24] unused; the arm bit took its place
	output reg        ss_peek_req,
	input     [127:0] ss_peek_data,
	input             ss_peek_valid,

	// Restore post-mortem, published in the same peek window past the data:
	// four PC samples taken after the freeze dropped, and the two fingerprints
	// the last restore compared.
	input     [127:0] ss_pc_snapshot,
	input      [63:0] ss_kick_pair,
	// Live chipset diagnostics, read back at byte_cnt 25-27.
	input      [14:0] ss_intena_live,
	input      [14:0] ss_intreq_live,
	input       [7:0] ss_frame_count,
	// Live VBR and SR. The state vector records VBR=0 for a game that demonstrably
	// contains movec-to-VBR code, and its level-3 vector at physical $6C points
	// into the middle of a ROM routine -- which the running machine could not
	// survive if it were really using it. Either VBR is nonzero live and the
	// capture is wrong, or it is zero and the running machine never takes
	// level 3. One reading decides which.

	// Beam diagnostics. See the latches in Minimig.sv for what each answers.
	input      [10:0] ss_vpos,
	input      [10:0] ss_vpos_max,
	input       [8:0] ss_hpos_max,
	input       [7:0] ss_vbl_int_count,
	input       [8:0] ss_htotal,
	input             ss_varbeamen,
	input             ss_harddis
);

assign EXT_BUS[15:0] = io_fpga ? fpga_dout : io_dout;
assign io_din = EXT_BUS[31:16];
assign EXT_BUS[32] = dout_en | io_fpga;
assign io_strobe = EXT_BUS[33];
assign io_uio = EXT_BUS[34];
assign io_fpga = EXT_BUS[35];

localparam EXT_CMD_MIN  = UIO_GET_VMODE;
localparam EXT_CMD_MAX  = UIO_SET_VPOS;
localparam EXT_CMD_MIN2 = 'h61;
localparam EXT_CMD_MAX2 = 'h63;

localparam UIO_MOUSE     = 'h04;
localparam UIO_KEYBOARD  = 'h05;
localparam UIO_KBD_OSD   = 'h06;
localparam UIO_GET_VMODE = 'h2C;
localparam UIO_SET_VPOS  = 'h2D;

reg [15:0] io_dout;
reg        dout_en;
// Six bits, not five: the savestate diagnostic readback now runs past word 31,
// and at five bits byte_cnt saturated there (~&byte_cnt) so every later word
// read back as the same one. Narrower comparisons elsewhere zero-extend and
// are unaffected.
reg  [5:0] byte_cnt;
// Save state diagnostics chip select. Module level rather than local to
// main_proc below only so it needs no declaration initialiser: the ~io_uio
// branch clears it at the end of every transaction, which is before any read
// can reach it.
reg        ss_diag_cs;
reg        ss_peek_cs;
reg        ss_peek_arm;

always@(posedge clk_sys) begin : main_proc
	reg [15:0] cmd;
	reg ide_cs = 0;
	reg cdda_cs = 0;

	sset <= 0;

	{ide_rd, ide_wr} <= 0;
	cdda_wr <= 0;
	{akiko_rd, akiko_wr} <= 0;
	{cdtv_rd, cdtv_wr} <= 0;
	if((ide_rd | ide_wr) & ~&ide_addr[3:0]) ide_addr <= ide_addr + 1'd1;

	if(~io_uio) begin
		dout_en <= 0;
		io_dout <= 0;
		byte_cnt <= 0;
		ide_cs <= 0;
		cdda_cs <= 0;
		akiko_cs <= 0;
		akiko_cs_sec <= 0;
		akiko_cs_nvr <= 0;
		akiko_cs_subcode <= 0;
		cdtv_cs <= 0;
		cdtv_cs_sec <= 0;
		cdtv_cs_stch <= 0;
		cdtv_cs_nvr  <= 0;
		cdtv_cs_card <= 0;
		ss_diag_cs <= 0;
		ss_peek_cs <= 0;
		ss_peek_arm <= 0;
		ss_peek_req <= 0;
		if(cmd == 'h2D) sset <= 1;
	end
	else if(io_strobe) begin

		io_dout <= 0;
		if(~&byte_cnt) byte_cnt <= byte_cnt + 1'd1;

		ide_dout <= io_din;
		cdda_dout <= io_din;
		akiko_dout <= io_din;
		cdtv_dout <= io_din;
		if(byte_cnt == 1) begin
			ide_addr     <= {io_din[8],io_din[3:0]};
			ide_cs       <= (io_din[15:9] == 7'b1111000);
			cdda_cs      <= (io_din[15:9] == 7'b1111001);
			// Trace sub-channel (io_din[7]=1) and peek sub-channel (io_din[5]=1)
			// are exclusive: keep cs/cs_sec LOW so the M3/M4 bridge isn't fed
			// during a debug drain.
			akiko_cs        <= (io_din[15:9] == 7'b1111010) && !io_din[7] && !io_din[5];
			akiko_cs_sec    <= (io_din[15:9] == 7'b1111010) && !io_din[7] && !io_din[5] && io_din[8];
			akiko_cs_nvr    <= (io_din[15:9] == 7'b1111010) && !io_din[7] && !io_din[5] && io_din[6];
			akiko_cs_subcode<= (io_din[15:9] == 7'b1111010) && !io_din[7] && !io_din[5] && io_din[4];
			// CDTV bridge cmd byte stream — io_din[7]=0, io_din[6]=0, io_din[5]=0.
			cdtv_cs          <= (io_din[15:9] == 7'b1111100) && !io_din[7] && !io_din[6] && !io_din[5];
			// CDTV sector-push sub-channel — io_din[7]=0, io_din[6]=0, io_din[5]=1.
			cdtv_cs_sec      <= (io_din[15:9] == 7'b1111100) && !io_din[7] && !io_din[6] &&  io_din[5];
			// CDTV STCH inject sub-channel — io_din[7]=0, io_din[6]=1.
			cdtv_cs_stch     <= (io_din[15:9] == 7'b1111100) && !io_din[7] &&  io_din[6];
			cdtv_cs_nvr      <= (io_din[15:9] == 7'b1111100) &&  io_din[7] && !io_din[6];
			cdtv_cs_card     <= (io_din[15:9] == 7'b1111100) &&  io_din[7] &&  io_din[6];
			// Save state diagnostics, and the peek sub-channel beside it.
			// io_din[5] picks between them, so the status window is unchanged
			// for any host that never asks for a peek.
			ss_diag_cs       <= (io_din[15:9] == 7'b1111011) && !io_din[5];
			ss_peek_cs       <= (io_din[15:9] == 7'b1111011) &&  io_din[5];
			// io_din[4] ARMS a new read; without it the transaction only reads
			// back what the last one fetched. They have to be separate, because
			// a readback that re-armed would clear the valid flag it is about to
			// report and race its own data -- which is exactly what the first
			// version of this did on hardware.
			ss_peek_arm      <= (io_din[15:9] == 7'b1111011) &&  io_din[5] && io_din[4];
			// Address bits [23:17] out of the remaining spare bits. Seven bits
			// reach 16 MB of word address, well past this design's SDRAM.
			if (io_din[15:9] == 7'b1111011 && io_din[5] && io_din[4])
				ss_peek_addr[23:17] <= {io_din[8:6], io_din[3:0]};
		end

		// One cycle wide: ss_ctrl edge-detects it after a domain crossing.
		ss_peek_req <= 1'b0;

		// Low sixteen address bits, then go. The address is complete here --
		// the high bits landed with the class word on the previous byte.
		if(byte_cnt == 2 && ss_peek_arm) begin
			ss_peek_addr[16:1] <= io_din;
			ss_peek_req        <= 1'b1;
		end

		if(byte_cnt == 0) begin
			cmd <= io_din;
			dout_en <= (io_din >= EXT_CMD_MIN && io_din <= EXT_CMD_MAX) || (io_din >= EXT_CMD_MIN2 && io_din <= EXT_CMD_MAX2);
			if(io_din == 'h63) begin
				// bit [11] = akiko_req (M3: command framed, ready to drain)
				// bit [10] = akiko_sec_req (M4: PBX wants a sector pushed)
				// Bit  [9] = akiko_rx_busy
				// bit  [8] = cdda_req (legacy stock-Minimig CDDA — dormant in NATIVE_CD32)
				// bit [13] = cdtv_card_dirty (memory card written since last clear)
				// bit [12] = cdtv_nvr_dirty (CDTV battery RAM written since last clear)
				// bit  [7] = akiko_nvr_dirty (NVRAM written since last clear)
				// bit  [6] = cdtv_req (CDTV cmd_in_fifo has data)
				// bits [5:0] = ide_req
				io_dout <= {2'b00, cdtv_card_dirty, cdtv_nvr_dirty, akiko_req, akiko_sec_req, akiko_rx_busy, cdda_req, akiko_nvr_dirty, cdtv_req, ide_req};
			end
			// A core that answers 1 here is telling userspace it carries the
			// Akiko/CDTV hardware, which is what gates the CD polls.
			if(io_din == UIO_GET_VMODE) io_dout <= 1;
		end else begin
			case(cmd)

				UIO_MOUSE:
					case(byte_cnt)
						1: begin
								kbd_mouse_data <= io_din[7:0];
								kbd_mouse_type <= 0;
								kbd_mouse_level <= ~kbd_mouse_level;
							end
						2: begin
								// second byte contains movement data
								kbd_mouse_data <= io_din[7:0];
								kbd_mouse_type <= 1;
								kbd_mouse_level <= ~kbd_mouse_level;
							end
						3: begin
								// third byte contains the buttons
								mouse_buttons <= io_din[2:0];
							end
						4: begin
								// wheel
								kbd_mouse_data <= io_din[7:0];
								kbd_mouse_level <= ~kbd_mouse_level;
							end
					endcase

				UIO_KEYBOARD:
					if(byte_cnt == 1) begin
						kbd_mouse_data <= io_din[7:0];
						kbd_mouse_type <= 2;
						kbd_mouse_level <= ~kbd_mouse_level;
					end

				UIO_KBD_OSD:
					if(byte_cnt == 1) begin
						kbd_mouse_data <= io_din[7:0];
						kbd_mouse_type <= 3;
						kbd_mouse_level <= ~kbd_mouse_level;
					end

				UIO_GET_VMODE:
					case(byte_cnt)
						1: io_dout <= {1'b1, scr_flg, 6'd0, scr_res};
						2: io_dout <= scr_hsize;
						3: io_dout <= scr_vsize;
						4: io_dout <= scr_hbl_l;
						5: io_dout <= scr_hbl_r;
						6: io_dout <= scr_vbl_t;
						7: io_dout <= scr_vbl_b;
					endcase

				UIO_SET_VPOS:
					case(byte_cnt)
						1: shbl_l <= io_din[11:0];
						2: shbl_r <= io_din[11:0];
						3: svbl_t <= io_din[11:0];
						4: svbl_b <= io_din[11:0];
					endcase

				'h61: begin
					if(byte_cnt >= 3) begin
						cdda_wr  <= cdda_cs;
						ide_wr   <= ide_cs;
						akiko_wr <= akiko_cs;
						// cdtv_wr feeds the cmd-byte channel plus the STCH,
						// sector, battery-RAM and memory-card sub-channels.
						// cdtv_bridge gates each on its own cs, so OR'ing here
						// just routes the strobe to whichever is selected.
						cdtv_wr  <= cdtv_cs | cdtv_cs_stch | cdtv_cs_sec | cdtv_cs_nvr | cdtv_cs_card;
					end
				end

				'h62: begin
					if(byte_cnt >= 3 && ide_cs) begin
						io_dout <= ide_din;
						ide_rd  <= 1;
					end
					if(byte_cnt >= 3 && akiko_cs) begin
						io_dout  <= akiko_din;
						akiko_rd <= 1;
					end
					// cdtv_cs_sec is write-only, so it stays out of the read
					// path. The cs_* are mutually exclusive, so a read here can
					// neither pop the cmd FIFO nor disturb the sector stream.
					if(byte_cnt >= 3 && (cdtv_cs | cdtv_cs_stch | cdtv_cs_nvr | cdtv_cs_card)) begin
						io_dout <= cdtv_din;
						cdtv_rd <= 1;
					end
					// Save state diagnostics. No strobe and no state of its own:
					// the words are muxed straight off byte_cnt the way
					// UIO_GET_VMODE is, so a read cannot disturb what it is
					// observing. That matters more here than it would elsewhere --
					// this channel exists to watch a save state, and a readback
					// that perturbed ss_ctrl would be measuring itself.
					//
					// Word 0 is a fixed signature rather than data. Against a core
					// built before this channel existed, cmd 'h62 still raises
					// dout_en and io_dout simply reads back zero -- so a poller with
					// nothing to check would log a plausible all-zero state forever
					// instead of saying the channel is absent. It is also how
					// userspace proves its own read alignment rather than assuming
					// it: see minimig_ssdiag.cpp's signature search.
					// Peek readback. Word 0 is its own signature so a host can
					// tell a core with the window from one without, exactly as
					// the status window does; word 9 carries the valid flag, so
					// a host that reads too early sees stale data marked stale
					// rather than fresh data it cannot trust.
					if(byte_cnt >= 3 && ss_peek_cs) begin
						case(byte_cnt)
							5'd3:  io_dout <= 16'h5A5A;
							5'd4:  io_dout <= ss_peek_data[15:0];
							5'd5:  io_dout <= ss_peek_data[31:16];
							5'd6:  io_dout <= ss_peek_data[47:32];
							5'd7:  io_dout <= ss_peek_data[63:48];
							5'd8:  io_dout <= ss_peek_data[79:64];
							5'd9:  io_dout <= ss_peek_data[95:80];
							5'd10: io_dout <= ss_peek_data[111:96];
							5'd11: io_dout <= ss_peek_data[127:112];
							5'd12: io_dout <= {15'd0, ss_peek_valid};
							// The post-mortem rides behind the peek data rather
							// than in a class of its own: it is read at the same
							// moments, by the same poller.
							5'd13: io_dout <= ss_pc_snapshot[15:0];
							5'd14: io_dout <= ss_pc_snapshot[31:16];
							5'd15: io_dout <= ss_pc_snapshot[47:32];
							5'd16: io_dout <= ss_pc_snapshot[63:48];
							5'd17: io_dout <= ss_pc_snapshot[79:64];
							5'd18: io_dout <= ss_pc_snapshot[95:80];
							5'd19: io_dout <= ss_pc_snapshot[111:96];
							5'd20: io_dout <= ss_pc_snapshot[127:112];
							5'd21: io_dout <= ss_kick_pair[15:0];
							5'd22: io_dout <= ss_kick_pair[31:16];
							5'd23: io_dout <= ss_kick_pair[47:32];
							5'd24: io_dout <= ss_kick_pair[63:48];
							5'd25: io_dout <= {1'b0, ss_intena_live};
							5'd26: io_dout <= {1'b0, ss_intreq_live};
							5'd27: io_dout <= {8'd0, ss_frame_count};
							5'd28: io_dout <= 16'd0;   // retired diagnostic
							5'd29: io_dout <= 16'd0;   // retired diagnostic
							5'd30: io_dout <= 16'd0;   // retired diagnostic
							6'd31: io_dout <= 16'd0;   // retired diagnostic
							6'd32: io_dout <= 16'd0;   // retired diagnostic
							6'd33: io_dout <= 16'd0;   // retired diagnostic
							6'd34: io_dout <= 16'd0;   // retired diagnostic
							6'd35: io_dout <= 16'd0;   // retired diagnostic
							6'd36: io_dout <= 16'd0;   // retired diagnostic
							6'd37: io_dout <= 16'd0;   // retired diagnostic
							6'd38: io_dout <= 16'd0;   // retired diagnostic
							6'd39: io_dout <= 16'd0;   // retired diagnostic
							6'd40: io_dout <= 16'd0;   // retired diagnostic
							6'd41: io_dout <= 16'd0;   // retired diagnostic
							6'd42: io_dout <= 16'd0;   // retired diagnostic
							6'd43: io_dout <= 16'd0;   // retired diagnostic
							6'd44: io_dout <= 16'd0;   // retired diagnostic
							6'd45: io_dout <= 16'd0;   // retired diagnostic
							6'd46: io_dout <= 16'd0;   // retired diagnostic
							6'd47: io_dout <= 16'd0;   // retired diagnostic
							6'd48: io_dout <= 16'd0;   // retired diagnostic
							6'd49: io_dout <= 16'd0;   // retired diagnostic
							6'd50: io_dout <= {5'd0, ss_vpos};
							6'd51: io_dout <= {5'd0, ss_vpos_max};
							6'd52: io_dout <= {7'd0, ss_hpos_max};
							6'd53: io_dout <= {8'd0, ss_vbl_int_count};
							6'd54: io_dout <= {5'd0, ss_harddis, ss_varbeamen, ss_htotal};
							default: io_dout <= 16'd0;
						endcase
					end
					if(byte_cnt >= 3 && ss_diag_cs) begin
						case(byte_cnt)
							5'd3:  io_dout <= 16'h55AA;
							5'd4:  io_dout <= ss_diag[15:0];
							5'd5:  io_dout <= ss_diag[31:16];
							5'd6:  io_dout <= ss_diag[47:32];
							5'd7:  io_dout <= ss_diag[63:48];
							5'd8:  io_dout <= ss_diag[79:64];
							5'd9:  io_dout <= ss_diag[95:80];
							5'd10: io_dout <= ss_diag[111:96];
							5'd11: io_dout <= ss_diag[127:112];
							default: io_dout <= 16'd0;
						endcase
					end
				end
			endcase
		end
	end
end

endmodule
