// Copyright 2021 Alexey Melnikov
// Copyright 2026 (CD32 native-mode register/IRQ extensions)
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
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http:// www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------
//
// Akiko CPU-bus slave at $B80000-$B8003F.
//
// Always present (any mode):
//   - $00-$03 ID  ($C0CACAFE — Kickstart only checks $CAFE at $B80002.W)
//   - $38-$3B C2P chunky-to-planar register
//
// Gated by parameter NATIVE_CD32 (default 0 — A1200 behavior unchanged):
//   - INTREQ/INTENA, CONFIG, DMA base registers, TX/RX/PBX index regs,
//     PIO byte, NVRAM I2C byte stubs, and akiko_irq output.
//
// Register semantics, masks, and read-mirror behavior are derived from
// WinUAE's akiko.cpp (akiko_bget2/akiko_bput2). Citations in comments
// below reference WinUAE line numbers at SHA 2c7f8581.
//
// Two places where the WinUAE comments and the WinUAE code disagree:
//   * The cmd/status DMA sub-buffer offsets are documented inconsistently
//     in WinUAE: the top-of-file comment claims base+0x000 is the command
//     buffer, but the executable code (akiko.cpp:1939-1941) uses
//       cdrx_address  = base | 0x000  // drive -> memory (response/status)
//       cdtx_address  = base | 0x200  // memory -> drive (command)
//       subcode_addr  = base | 0x100
//     The code is the truth, and these are the offsets the TX/RX DMA
//     below uses.
//   * The "data DMA base must be 64K aligned" comment disagrees with the
//     code mask 0x00fff000 (4K alignment). Code wins; we use 4K.
//
//----------------------------------------------------------------------------------

`include "rtl/ss_state.vh"

module akiko #(parameter NATIVE_CD32 = 0)
(
	input             clk,
	input             reset,
	input             cs,
	input             rd,
	input             wr,        // ~rnw & (lds|uds), pre-OR'd at fastchip
	input             lds,       // active high, lower byte (odd byte address)
	input             uds,       // active high, upper byte (even byte address)
	input       [5:1] addr,
	input      [15:0] din,
	output reg [15:0] dout,
	output            akiko_irq,

	// ---------------------------------------------------------------------
	// Chip-RAM master port (M2: TX/RX command DMA).
	// Byte-granular, single-byte-per-transaction. Held high until dma_ack.
	// Inactive (all zero) when NATIVE_CD32 = 0.
	// M3+ wires this through fastchip into a chip-RAM master/arbiter.
	// ---------------------------------------------------------------------
	output            dma_req,
	output            dma_we,    // 1 = akiko writing chip RAM (RX), 0 = reading (TX)
	output     [23:0] dma_baddr, // byte address
	output      [7:0] dma_wbyte,
	input       [7:0] dma_rbyte, // valid in the cycle dma_ack pulses
	input             dma_ack,
	// chipdma_arb pulses this for ONE clk_sys cycle
	// when it latches an akiko byte onto the chip bus (arm_now & ~cdtv). We
	// freeze which sub-engine (RX/PBX/TX) owns the transaction at that instant
	// so the later dma_ack is credited to the engine the arb actually serviced
	// — not to whatever engine the combinational priority mux now favours. This
	// closes the cross-engine ack-ownership race: a higher engine asserting
	// busy mid-flight stole a lower engine's ack, dropping one CD byte and
	// leaving the title with a corrupted pointer or scattered graphics.
	input             dma_arm,

	// ---------------------------------------------------------------------
	// HPS bridge port (M3: framed-command stream to Main_MiSTer, response
	// stream back). All bridge inputs are single-cycle pulses synchronous
	// to clk; all bridge outputs are combinational status. Inactive (zero)
	// when NATIVE_CD32 = 0.
	// ---------------------------------------------------------------------
	output            hps_cmd_pending, // a complete framed command is in the buffer
	output      [7:0] hps_cmd_byte,    // command_buffer[hps_cmd_rd_ptr]
	input             hps_cmd_pop,     // pulse: advance hps_cmd_rd_ptr by 1
	input             hps_cmd_done,    // pulse: clear command buffer; bridge has the command
	input             hps_result_push, // pulse: store hps_result_byte at hps_result_wr_ptr++
	input       [7:0] hps_result_byte,
	input             hps_result_done, // pulse: commit hps_result_wr_ptr -> receive_length

	// ---------------------------------------------------------------------
	// HPS sector channel (M4: one raw sector pushed from Main as 1176
	// 16-bit words, plus a 1-byte status read for the current
	// cdrom_sector_counter). Inactive (zero) when NATIVE_CD32 = 0.
	// ---------------------------------------------------------------------
	output            hps_sec_req,     // status: I have a free PBX slot and an empty buffer
	output      [7:0] hps_sec_status,  // 1-byte read mux (currently == cdrom_sector_counter)
	input             hps_sec_push,    // pulse: store hps_sec_word at sec_wr_ptr++
	input      [15:0] hps_sec_word,
	input             hps_sec_done,    // pulse: commit; if sec_wr_ptr == 11'd1176, sector_ready<=1

	// rx_busy = receive engine has a queued or in-flight response.
	// Userspace gates unsolicited pushes (TOC drip, post-INFO media-status)
	// on this — matches WinUAE's cdrom_can_return_data() semantics.
	output            hps_rx_busy,

	// NVRAM save-dump port. Bridge drives hps_nvr_addr (auto-incrementing
	// read counter) and pulses hps_nvr_clear_dirty on read-burst end;
	// nvram returns hps_nvr_dout one clk later. hps_nvr_dirty flags any
	// successful BIOS write to the EEPROM since the last clear-dirty pulse.
	input       [9:0] hps_nvr_addr,
	output      [7:0] hps_nvr_dout,
	input             hps_nvr_clear_dirty,
	output            hps_nvr_dirty,

	// NVRAM load-from-disk port. Driven by hps_io.ioctl_download via a
	// gated signal at Minimig.sv level (NVR_LOAD_INDEX). Lives in HPS
	// reset domain — fires before BIOS sees the I²C bus, so there's no
	// contention between the load and BIOS-initiated I²C transactions.
	// nvr_load_we does NOT set the dirty flag (loading saved state must
	// not trigger an immediate re-save).
	input       [9:0] nvr_load_addr,
	input       [7:0] nvr_load_din,
	input             nvr_load_we,

	// Subcode streaming push channel (akiko_cd32.cpp), mirrors hps_sec_*
	// slow path. Main pushes 96 INTERLEAVED subchannel bytes per CD frame
	// during CDDA play; the DMA FSM ships them to subcode_address.
	input             hps_subcode_push,
	input       [7:0] hps_subcode_byte,
	input             hps_subcode_done,

	// ---------------------------------------------------------------------
	// Save state.
	//
	// ss_state is the whole of what a restore puts back, and ss_ld writes all
	// of it on one clock -- the same shape ciaa.v and ciab.v use, for the same
	// reason: there is nothing here to sequence and nothing that can land
	// half-written.
	//
	// What the vector does NOT carry is the transient machinery: the 2352-byte
	// sector staging buffer, the 96-byte subcode block, the command and result
	// buffers, and the DMA engines' in-flight bookkeeping. ss_idle covers
	// those instead. The snapshot is only taken in a cycle where every engine
	// is idle and nothing is staged, so there is no in-flight byte to carry --
	// and, just as important, none to silently lose.
	//
	// WinUAE draws the same line (save_akiko writes the registers and the C2P
	// buffer, and re-reads sectors from the image on restore). It can afford
	// to re-read because it owns the drive; we do not own it, so we wait for
	// the gap instead of trying to recreate one.
	//
	// The layout is written out at SS_ prefixed localparams below. It is the
	// on-disk field order, so appending is free and reordering is not.
	output [`SS_AKIKO_W-1:0] ss_state,
	input                    ss_ld,
	input  [`SS_AKIKO_W-1:0] ss_ld_data,
	output                   ss_idle
);

// -----------------------------------------------------------------------
// WinUAE-derived constants (akiko.cpp at SHA 2c7f8581)
// -----------------------------------------------------------------------
localparam [31:0] INTENA_MASK     = 32'hff000000; // bput line 1924
localparam [31:0] CONFIG_MASK     = 32'hff800000; // bput line 1986
localparam [31:0] ADDRDATA_MASK   = 32'h00fff000; // bput line 1931 (4K-align)
localparam [31:0] ADDRMISC_MASK   = 32'h00fffc00; // bput line 1938 (1K-align)

localparam [31:0] CDINT_SUBCODE   = 32'h80000000; // bit 31
localparam [31:0] CDINT_DRIVEXMIT = 32'h40000000; // bit 30 (PIO)
localparam [31:0] CDINT_DRIVERECV = 32'h20000000; // bit 29 (PIO)
localparam [31:0] CDINT_RXDMADONE = 32'h10000000; // bit 28
localparam [31:0] CDINT_TXDMADONE = 32'h08000000; // bit 27
localparam [31:0] CDINT_PBX       = 32'h04000000; // bit 26
localparam [31:0] CDINT_OVERFLOW  = 32'h02000000; // bit 25

localparam        CDFLAG_TXD_BIT    = 30; // CONFIG bit 30 (TX command DMA enable)
localparam        CDFLAG_RXD_BIT    = 29; // CONFIG bit 29 (RX status DMA enable)
localparam        CDFLAG_PBX_BIT    = 27; // CONFIG bit 27 (data DMA enable)
localparam        CDFLAG_ENABLE_BIT = 26; // CONFIG bit 26 (CD interface enable)

// -----------------------------------------------------------------------
// Save state field map.
//
// Offsets are LSB bit positions in ss_state and ss_ld_data. Both directions
// index from these names, so capture and restore cannot disagree about where
// a field lives; and because this IS the on-disk field order, appending is
// free while reordering breaks every existing save.
// -----------------------------------------------------------------------
localparam SS_O_INTREQ   =   0; // 32  cdrom_intreq
localparam SS_O_INTENA   =  32; // 32  cdrom_intena
localparam SS_O_ADDRDATA =  64; // 32  cdrom_addressdata
localparam SS_O_ADDRMISC =  96; // 32  cdrom_addressmisc
localparam SS_O_FLAGS    = 128; // 32  cdrom_flags (CONFIG)
localparam SS_O_PBX      = 160; // 16  cdrom_pbx
localparam SS_O_SUBCOFF  = 176; //  8  cdrom_subcodeoffset (the register)
localparam SS_O_TXINX    = 184; //  8  cdcomtxinx
localparam SS_O_RXINX    = 192; //  8  cdcomrxinx
localparam SS_O_TXCMP    = 200; //  8  cdcomtxcmp
localparam SS_O_RXCMP    = 208; //  8  cdcomrxcmp
localparam SS_O_SUBOFF   = 216; //  8  subcode_off (the DMA write base, 0/128)
localparam SS_O_SECCNT   = 224; //  8  cdrom_sector_counter
localparam SS_O_NVRIO    = 232; //  8  nvram_io
localparam SS_O_NVRDIR   = 240; //  8  nvram_dir
localparam SS_O_PIO      = 248; //  8  pio_byte
localparam SS_O_SUBIRQ   = 256; //  1  subcode_irq
localparam SS_O_SHIPINV  = 257; //  1  pbx_ship_invalid
localparam SS_O_CMDBUF   = 258; // 256 cdrom_command_buffer[0..31]
localparam SS_O_CMDLEN   = 514; //   6 cdrom_command_length
localparam SS_O_RESBUF   = 520; // 256 cdrom_result_buffer[0..31]
localparam SS_O_RXLEN    = 776; //   6 cdrom_receive_length
localparam SS_O_RXOFF    = 782; //   6 cdrom_receive_offset
localparam SS_O_C2P      = 788; // 256 buff[0..31], buff[0] at the low byte
localparam SS_O_RPTR     = 1044;//   4 rptr
localparam SS_O_WPTR     = 1048;//   4 wptr
                                //     total 1052 == `SS_AKIKO_W

// -----------------------------------------------------------------------
// Existing C2P logic (preserved bit-equivalent to legacy akiko.v)
// -----------------------------------------------------------------------
wire c2p_sel = (addr[5:2] == 'b1110);

reg [7:0] buff[32];
reg [3:0] rptr = 0, wptr = 0;

integer ss_ci;
always @(posedge clk) begin
	// ss_ld first: a restore overrides whatever the bus is doing, and the
	// CPU is parked at an instruction boundary while it runs, so the two
	// cannot legitimately collide anyway.
	if (ss_ld) begin
		for (ss_ci = 0; ss_ci < 32; ss_ci = ss_ci + 1)
			buff[ss_ci] <= ss_ld_data[SS_O_C2P + ss_ci*8 +: 8];
		rptr <= ss_ld_data[SS_O_RPTR +: 4];
		wptr <= ss_ld_data[SS_O_WPTR +: 4];
	end
	else if((wr|rd) & cs & c2p_sel) begin
		if (wr) begin
			rptr <= 0;
			wptr <= wptr + 1'd1;
			{buff[{wptr,1'b0}],buff[{wptr,1'b1}]} <= din;
		end
		else begin
			wptr <= 0;
			rptr <= rptr + 1'd1;
		end
	end
end

// The C2P buffer, out to the state vector. A generate loop rather than a
// 32-term concatenation: the array is indexed everywhere else and writing
// the list out by hand is a transposition waiting to happen.
wire [255:0] c2p_ss_buf;
genvar ss_gi;
generate
	for (ss_gi = 0; ss_gi < 32; ss_gi = ss_gi + 1) begin : g_c2p_ss
		assign c2p_ss_buf[ss_gi*8 +: 8] = buff[ss_gi];
	end
endgenerate

reg [15:0] c2p_dout;
always @(*) begin : c2p_read
	reg [4:0] i;
	c2p_dout = 16'h0;
	for (i=0; i<16; i=i+1'd1)
		c2p_dout[i] = buff[{rptr[0],~i[3:0]}][rptr[3:1]];
end

// -----------------------------------------------------------------------
// CD register block — only synthesized when NATIVE_CD32 = 1
// -----------------------------------------------------------------------
wire [15:0] cd_dout;
wire        cd_irq;
wire        cd_dma_req;
wire        cd_dma_we;
wire [23:0] cd_dma_baddr;
wire  [7:0] cd_dma_wbyte;
wire        cd_hps_cmd_pending;
wire  [7:0] cd_hps_cmd_byte;
wire        cd_hps_sec_req;
wire  [7:0] cd_hps_sec_status;
wire        cd_hps_rx_busy;
wire  [7:0] cd_hps_nvr_dout;
wire        cd_hps_nvr_dirty;
wire [787:0] cd_ss_native;   // the CD register block's slice of ss_state
wire         cd_ss_idle;

generate
if (NATIVE_CD32) begin : g_cd

	reg [31:0] cdrom_intreq;
	reg [31:0] cdrom_intena;
	reg [31:0] cdrom_addressdata;
	reg [31:0] cdrom_addressmisc;
	reg [31:0] cdrom_flags;          // CONFIG
	reg [15:0] cdrom_pbx;
	reg  [7:0] cdrom_subcodeoffset;
	reg  [7:0] cdcomtxinx;           // current TX index (live status)
	reg  [7:0] cdcomrxinx;           // current RX index
	reg  [7:0] cdcomtxcmp;           // TX end (compare)
	reg  [7:0] cdcomrxcmp;           // RX end
	reg  [7:0] pio_byte;             // last PIO write — M1 stub
	reg  [7:0] nvram_io;             // $30 master-driven SCL/SDA pair (bit 7=SCL, 6=SDA)
	reg  [7:0] nvram_dir;            // $32 direction (1=master output, 0=floating-high input)

	// Real I2C slave EEPROM (1 KiB, 24LC08-equivalent) replaces
	// the M1 stub. See akiko_nvram.v for protocol; bus model below for
	// open-drain wiring.
	//
	// BIOS bit-bangs the bus through the $B80030 / $B80032 registers
	// (nvram_io / nvram_dir). The slave only ever pulls SDA low for
	// ACK / read-data; SCL is master-only. NVRAM LOAD from disk does NOT
	// touch this bus — it goes through akiko_nvram's load_we port directly
	// (driven by hps_io.ioctl_download from Minimig.sv).
	wire       nvram_scl_master_drive = nvram_dir[7];
	wire       nvram_sda_master_drive = nvram_dir[6];
	wire       nvram_scl_bus = nvram_scl_master_drive ? nvram_io[7] : 1'b1;
	wire       nvram_sda_master_value =
	               nvram_sda_master_drive ? nvram_io[6] : 1'b1;
	wire       nvram_slave_sda_drive;
	wire       nvram_sda_bus = nvram_sda_master_value & ~nvram_slave_sda_drive;

	// M2 TX/RX command DMA state.
	// command_buffer / command_length accumulate bytes pulled by TX DMA;
	// result_buffer / receive_length / receive_offset feed RX DMA.
	// In M2 the bench injects results directly via hierarchical access
	// to result_buffer + receive_length. M3 will set them from the
	// HPS-bridge command-response path.
	reg  [7:0] cdrom_command_buffer [32];
	reg  [5:0] cdrom_command_length;        // 0..32
	reg  [7:0] cdrom_result_buffer  [32];
	reg  [5:0] cdrom_receive_length;        // 0..32 (0 = no result pending)
	reg  [5:0] cdrom_receive_offset;        // bytes already DMA'd to chip RAM
	reg  [1:0] tx_dma_delay;                // 3-tick post-write inhibit
	reg  [1:0] rx_dma_delay;
	reg        tx_busy;                     // engine waiting for dma_ack (TX read)
	reg        rx_busy;                     // engine waiting for dma_ack (RX write)
	reg        rx_inflight;                 // BFM has accepted our request (post-quiet-cycle)

	// The DMA-bus owner latched at chipdma_arb's
	// arm_now (dma_arm). dma_owned is high for the whole arm->ack transaction;
	// dma_owner records WHICH engine the arb serviced, sampled from the exact
	// same combinational priority (rx>pbx>tx>sub) that produced dma_baddr on
	// that edge — so address-written and ack-credited always agree even if a
	// higher-priority engine raises its busy flag mid-flight.
	localparam [1:0] OWN_RX = 2'd0, OWN_PBX = 2'd1, OWN_TX = 2'd2, OWN_SUB = 2'd3;
	reg        dma_owned;
	reg  [1:0] dma_owner;
	wire       own_rx  = dma_owned & (dma_owner == OWN_RX);
	wire       own_pbx = dma_owned & (dma_owner == OWN_PBX);
	wire       own_tx  = dma_owned & (dma_owner == OWN_TX);
	wire       own_sub = dma_owned & (dma_owner == OWN_SUB);

	// M3 HPS bridge state.
	// hps_cmd_rd_ptr indexes into cdrom_command_buffer for the bridge's
	// command-stream read; hps_result_wr_ptr accumulates response bytes
	// from the bridge before commit. Both reset to 0 on transaction
	// boundaries (hps_cmd_done / hps_result_done).
	reg  [5:0] hps_cmd_rd_ptr;
	reg  [5:0] hps_result_wr_ptr;

	// Restore-side loop index for the two command buffers.
	integer    ss_bj;

	// M4 PBX sector DMA state.
	//
	// sector_buffer holds one raw 2352-byte sector pushed by Main via the
	// HPS sector channel as 1176 words. sector_ready latches when a full
	// sector arrives (sec_wr_ptr == 1176 at hps_sec_done). cdrom_sector_counter resets to
	// 0 on CDFLAG_ENABLE 0->1 (akiko.cpp:1973-1976); increments after each
	// successful slot write.
	//
	// PBX engine FSM:
	//   PBX_IDLE  : await (CDFLAG_ENABLE & CDFLAG_PBX & cdrom_pbx & sector_ready);
	//               latch seccnt = highest_set_bit(cdrom_pbx); -> PBX_DATA.
	//   PBX_DATA  : drive write of byte_idx 0..2351 to slot+byte_idx.
	//               byte source is computed below (zeros / counter / buffer).
	//   PBX_ZERO  : drive write of zero to slot+0xc00 + zero_idx for 0..145.
	//   PBX_FIN   : clear pbx[seccnt], set CDINT_PBX, increment counter,
	//               drop sector_ready, -> PBX_IDLE.
	// One sector is 1176 16-bit words, not 2352 bytes. The host pushes the
	// whole sector as words over the ext bus, which halves the number of SPI
	// elements per sector and leaves a single write port here.
	reg [15:0] sector_buffer [1176];
	reg [10:0] sec_wr_ptr;
	reg        sector_ready;

	wire        sec_w_we   = hps_sec_push && sec_wr_ptr != 11'd1176;
	wire [10:0] sec_w_addr = sec_wr_ptr;
	wire [15:0] sec_w_din  = hps_sec_word;
	reg  [7:0] cdrom_sector_counter;
	reg        pbx_busy;
	reg  [1:0] pbx_state;
	localparam PBX_IDLE = 2'd0;
	localparam PBX_DATA = 2'd1;
	localparam PBX_ZERO = 2'd2;
	localparam PBX_FIN  = 2'd3;
	reg  [3:0] pbx_seccnt;       // selected slot (0..15)
	reg [11:0] pbx_byte_idx;     // 0..2351 in DATA, 0..145 in ZERO

	// STICKY ship-invalidation. A PBX ship in flight
	// (or starting this cycle) when a new READ DATA fires (CDFLAG_ENABLE 0->1)
	// belongs to the PREVIOUS read; its PBX_FIN must NOT bump cdrom_sector_counter
	// (the new read already reset it to 0 — bumping = off-by-one recurrence).
	// pbx_ship_invalid is SET on any enable_rising while the ship is live and
	// CLEARED only when a fresh ship starts without a concurrent enable_rising.
	// A sticky set bit is idempotent under N rapid back-to-back READ DATA pulses,
	// unlike a toggle-gen, which aliases back to "valid" after an even number
	// of rises.
	reg        pbx_ship_invalid;

	// Subcode streaming (WinUAE akiko.cpp:1486-1509). subbuf holds one 96-byte
	// INTERLEAVED P-W subchannel block pushed by Main during CDDA play; the DMA
	// FSM writes it to (addressmisc|0x100)+offset, appends a 0xffff0000 marker,
	// ping-pongs cdrom_subcodeoffset 0/128 (+=100), and raises CDINT_SUBCODE.
	// One write port (UIO push) so the array can map to M10K. subcode_irq is a
	// dedicated IRQ latch so the streaming interrupt never races the many
	// cdrom_intreq writers.
	reg  [7:0] subbuf [96];
	reg  [6:0] sub_wr_ptr;       // 0..96 fill pointer
	reg        subcode_ready;    // a full 96-byte block is staged
	reg        subcode_busy;     // DMA in progress
	reg        subcode_irq;      // dedicated SUBCODE IRQ latch
	reg  [1:0] subcode_state;
	localparam SUB_IDLE = 2'd0;
	localparam SUB_DATA = 2'd1;
	localparam SUB_FIN  = 2'd2;
	localparam CDFLAG_SUBCODE_BIT = 31; // CONFIG bit 31 (subcode stream enable)
	reg  [7:0] subcode_off;      // current write base (0 or 128)
	reg  [7:0] sub_idx;          // 0..99 byte walk (96 data + 4 marker)

	// Expected total command length (incl trailing checksum byte) for the
	// command currently being framed. Mirrors WinUAE akiko.cpp:1136
	// command_lengths[]. Negative entries (opcodes 0x0b-0x0f) are unknown
	// commands — frame on buffer-full so Main can reject with CH_ERR_BADCMD.
	function [5:0] expected_total_len;
		input [3:0] op;
		case (op)
			4'h0: expected_total_len = 6'd2;   // 1 + chk
			4'h1: expected_total_len = 6'd3;   // 2 + chk (STOP)
			4'h2: expected_total_len = 6'd2;   // 1 + chk (PAUSE)
			4'h3: expected_total_len = 6'd2;   // 1 + chk (UNPAUSE)
			4'h4: expected_total_len = 6'd13;  // 12 + chk (PLAY/READ)
			4'h5: expected_total_len = 6'd3;   // 2 + chk (LED)
			4'h6: expected_total_len = 6'd2;   // 1 + chk (SUBQ)
			4'h7: expected_total_len = 6'd2;   // 1 + chk (INFO/STATUS)
			4'h8: expected_total_len = 6'd5;   // 4 + chk
			4'h9: expected_total_len = 6'd2;   // 1 + chk
			4'ha: expected_total_len = 6'd3;   // 2 + chk
			default: expected_total_len = 6'd32; // unknown — frame on buffer full
		endcase
	endfunction

	wire [3:0] cmd_op       = cdrom_command_buffer[0][3:0];
	wire [5:0] cmd_total    = expected_total_len(cmd_op);
	wire       cmd_pending  = (cdrom_command_length != 6'd0)
	                       && ((cdrom_command_length >= cmd_total)
	                          || (cdrom_command_length == 6'd32));

	// WinUAE addressmisc layout (akiko.cpp:1937-1942)
	wire [23:0] cdrx_address = cdrom_addressmisc[23:0];                 // base | 0x000
	wire [23:0] cdtx_address = cdrom_addressmisc[23:0] | 24'h000200;    // base | 0x200
	wire [23:0] subcode_address = cdrom_addressmisc[23:0] | 24'h000100;  // base | 0x100
	wire [23:0] subcode_dma_addr = subcode_address
	                             + {16'h0, subcode_off} + {16'h0, sub_idx};
	wire  [7:0] subcode_dma_byte = (sub_idx < 8'd96) ? subbuf[sub_idx[6:0]] :
	                              (sub_idx < 8'd98) ? 8'hff : 8'h00;

	wire tx_can_start =  cdrom_flags[CDFLAG_TXD_BIT]
	                  && !cdrom_flags[CDFLAG_ENABLE_BIT]
	                  && (cdcomtxinx != cdcomtxcmp)
	                  && (tx_dma_delay == 2'd0)
	                  && (cdrom_receive_length == 6'd0)
	                  && (cdrom_command_length != 6'd32)
	                  && !cmd_pending;       // hold while bridge has work to do

	wire rx_can_start =  cdrom_flags[CDFLAG_RXD_BIT]
	                  && (cdrom_receive_length != 6'd0)
	                  && (cdcomrxinx != cdcomrxcmp)
	                  && (rx_dma_delay == 2'd0);

	// Subcode mutual-exclusion helpers. chipdma_arb latches the live akiko DMA
	// address at arm_now and acks ~5 cycles later, so a higher-priority engine
	// asserting inside that window would steal subcode's ack (and corrupt its own
	// progress). Subcode is therefore run strictly non-overlapping: it claims the
	// bus only when rx/pbx/tx are idle AND none is about to start, and those three
	// are blocked from starting while subcode_busy (below). All no-ops for
	// non-subcode games (subcode_busy gated on CDFLAG_SUBCODE -> always 0 there).
	wire pbx_can_start =  (pbx_state == PBX_IDLE)
	                   && cdrom_flags[CDFLAG_ENABLE_BIT]
	                   && cdrom_flags[CDFLAG_PBX_BIT]
	                   && (cdrom_pbx != 16'h0)
	                   && sector_ready;
	wire others_busy     = rx_busy | pbx_busy | tx_busy;
	wire others_starting = rx_can_start | tx_can_start | pbx_can_start;

	// M4 PBX engine derived signals.
	// pbx_slot_base = addressdata + seccnt*4096; current pbx_addr depends on
	// the engine phase (DATA at slot+byte_idx, ZERO at slot+0xc00+byte_idx).
	// sector_byte_at_idx implements the WinUAE per-byte source rules:
	//   bytes 0..2: zero
	//   byte 3:     sector_counter & 31
	//   bytes 4..2351: sector_buffer[idx]
	wire [23:0] pbx_slot_base = cdrom_addressdata[23:0] + {8'h0, pbx_seccnt, 12'h0};
	wire [23:0] pbx_addr_c    = pbx_slot_base
	                          + ((pbx_state == PBX_DATA)
	                              ? {12'h0, pbx_byte_idx}
	                              : (24'h000c00 + {12'h0, pbx_byte_idx}));
	// Register pbx_addr so the SDRAM-bound critical
	// path no longer carries two cascaded 24-bit adders + chipdma_arb mux
	// chain in a single combinational arc. The downstream chipdma_arb only
	// samples this address on c_7m_rise (≥3 clk_sys cycles after the byte
	// transition that updates pbx_byte_idx), so a 1-cycle latency here is
	// invisible to the SDRAM master.
	reg  [23:0] pbx_addr;
	always @(posedge clk) begin
		if (reset) pbx_addr <= 24'h0;
		else       pbx_addr <= pbx_addr_c;
	end
	// sector_buffer must NOT be read combinationally here
	// (sector_buffer[pbx_byte_idx]): that forces Quartus to map the array
	// into ALM registers behind a wide async read mux and an equally wide
	// write decode — a timing-marginal structure that intermittently
	// mis-fills and mis-reads under back-to-back block writes. Hardware A/B
	// on the old byte-wide array: fast push garbled every time, slow
	// per-byte push a quarter of the time, and both a D-Cache and a PBX-write
	// fix were falsified, so the corruption was born in the FILL of this
	// array. Registering the read makes the array infer M10K block RAM
	// (synchronous read) and incidentally aligns the read
	// latency with the already-registered pbx_addr. The 1-cycle latency is
	// invisible: pbx_byte_idx is stable between dma_acks and chipdma_arb samples
	// pbx_addr/pbx_wbyte on c_7m_rise ≥3 clk_sys cycles after the byte
	// transition (the same argument that makes pbx_addr's registration safe).
	reg [15:0] sector_rd_w;
	reg        sector_rd_hi;
	always @(posedge clk) begin
		sector_rd_w  <= sector_buffer[pbx_byte_idx[11:1]];
		sector_rd_hi <= pbx_byte_idx[0];
	end
	// Words arrive in host byte order, so the odd byte of each pair is the
	// high half.
	wire [7:0] sector_rd_q = sector_rd_hi ? sector_rd_w[15:8] : sector_rd_w[7:0];
	wire [7:0]  sector_byte_at_idx = (pbx_byte_idx <  12'd3   ) ? 8'h00 :
	                                 (pbx_byte_idx == 12'd3   ) ? (cdrom_sector_counter & 8'h1f) :
	                                 (pbx_byte_idx <  12'd2352) ? sector_rd_q :
	                                                              8'h00;
	wire [7:0]  pbx_wbyte = (pbx_state == PBX_DATA) ? sector_byte_at_idx : 8'h00;

	// Highest set bit in cdrom_pbx (4-bit slot index 0..15). WinUAE iterates
	// 15 down to 0 (akiko.cpp:1314-1318); equivalent here is "loop 0..15
	// and overwrite — last set bit wins".
	function [3:0] highest_bit;
		input [15:0] m;
		integer i;
		begin
			highest_bit = 4'd0;
			for (i = 0; i < 16; i = i + 1)
				if (m[i]) highest_bit = i[3:0];
		end
	endfunction

	// sec_req: high when PBX wants a sector AND the single staging buffer is
	// truly writable. Drops as soon as Main commits a sector (sector_ready->1)
	// and stays low while the PBX engine is consuming the buffer (pbx_busy).
	//
	// The !pbx_busy term serializes the single-buffer
	// producer (HPS fast-fill) against the consumer (PBX ship). The off-by-one
	// ENABLE-clear can leave sector_ready=0 while pbx_busy=1; without this term
	// the bridge would push a new sector into the buffer PBX is still reading
	// (tear), or push into a buffer whose sector_ready re-asserts from a racing
	// byte-2351 commit (total drop -> fill=0 -> stale-sector ship -> wild seek).
	// In normal operation sector_ready is already high throughout a ship, so
	// this only closes the sector_ready=0 && pbx_busy=1 window the ENABLE-clear
	// opened; no throughput change for CF / single-read games.
	wire sec_req_w =  cdrom_flags[CDFLAG_ENABLE_BIT]
	               && cdrom_flags[CDFLAG_PBX_BIT]
	               && (cdrom_pbx != 16'h0)
	               && !sector_ready
	               && !pbx_busy;

	wire write = wr & cs;

	// CDFLAG_ENABLE 0->1 this cycle = a new READ DATA "generation". CONFIG-high
	// reg ($24-$27, addr 5'b10010); ENABLE is bit 26, in the upper byte, so it
	// arrives on uds via din[10]. Combinational so it can give the restart
	// priority over same-cycle PBX_FIN counter-bump and byte-2351 sector_ready
	// set inside the clocked block below (later NBAs win, so the restart guards
	// those two lower-priority writes with !enable_rising).
	wire enable_rising = write && (addr == 5'b10010) && uds && din[10]
	                  && !cdrom_flags[CDFLAG_ENABLE_BIT];

	// A PBX ship is STARTING this cycle (PBX_IDLE picks up a staged sector) —
	// mirror of the PBX_IDLE->PBX_DATA guard. Used by the sticky-invalidation
	// logic so a ship begun the SAME cycle a new READ DATA fires is marked stale.
	wire pbx_starting = (pbx_state == PBX_IDLE)
	                 && cdrom_flags[CDFLAG_ENABLE_BIT]
	                 && cdrom_flags[CDFLAG_PBX_BIT]
	                 && (cdrom_pbx != 16'h0)
	                 && sector_ready && !subcode_busy;

	always @(posedge clk) begin
		if (reset) begin
			// WinUAE akiko.cpp:2135 inits cdrom_intreq = CDINTERRUPT_SUBCODE
			// at reset/restore. SUBCODE bit is the "drive heartbeat" the
			// BIOS samples at boot to confirm Akiko presence; without it
			// BIOS may treat drive as absent and never advance to MULTI.
			cdrom_intreq        <= CDINT_SUBCODE;
			cdrom_intena        <= 32'h0;
			cdrom_addressdata   <= 32'h0;
			cdrom_addressmisc   <= 32'h0;
			cdrom_flags         <= 32'h0;
			cdrom_pbx           <= 16'h0;
			cdrom_subcodeoffset <= 8'h0;
			cdcomtxinx          <= 8'h0;
			cdcomrxinx          <= 8'h0;
			cdcomtxcmp          <= 8'h0;
			cdcomrxcmp          <= 8'h0;
			pio_byte            <= 8'h0;
			nvram_io            <= 8'h0;
			nvram_dir           <= 8'h0;
			cdrom_command_length <= 6'h0;
			cdrom_receive_length <= 6'h0;
			cdrom_receive_offset <= 6'h0;
			tx_dma_delay         <= 2'h0;
			rx_dma_delay         <= 2'h0;
			tx_busy              <= 1'b0;
			rx_busy              <= 1'b0;
			rx_inflight          <= 1'b0;
			dma_owned            <= 1'b0;
			dma_owner            <= OWN_RX;
			hps_cmd_rd_ptr       <= 6'h0;
			hps_result_wr_ptr    <= 6'h0;
			sec_wr_ptr           <= 11'h0;
			sector_ready         <= 1'b0;
			sub_wr_ptr           <= 7'h0;
			subcode_ready        <= 1'b0;
			subcode_busy         <= 1'b0;
			subcode_irq          <= 1'b0;
			subcode_state        <= SUB_IDLE;
			subcode_off          <= 8'h0;
			sub_idx              <= 8'h0;
			cdrom_sector_counter <= 8'h0;
			pbx_busy             <= 1'b0;
			pbx_state            <= PBX_IDLE;
			pbx_seccnt           <= 4'h0;
			pbx_byte_idx         <= 12'h0;
			pbx_ship_invalid     <= 1'b0;
		end else begin
			// 3-tick post-write delay decay (akiko.cpp:1949,1954)
			if (tx_dma_delay != 2'd0) tx_dma_delay <= tx_dma_delay - 2'd1;
			if (rx_dma_delay != 2'd0) rx_dma_delay <= rx_dma_delay - 2'd1;

			// Latch the serviced engine at arm_now and
			// hold it until the transaction's ack. Sampled from the SAME
			// combinational priority that drives dma_baddr (RX>PBX>TX>SUB), so
			// the byte the arb wrote and the engine that consumes the ack always
			// match. arm and ack are >=4 clk_sys cycles apart (S_DRIVE), so they
			// never coincide; arm takes priority defensively.
			if (dma_arm) begin
				// Only claim ownership when a real engine is live this edge. The arb arms off the REGISTERED
				// akiko_dma_req_q while addr/owner are live-combinational, so it
				// can arm a "stale" slot where req was high last cycle but every
				// *_busy already cleared (e.g. just after PBX_FIN). Latching a
				// real owner there would default to OWN_SUB and let a later
				// subcode txn consume that stale ack for a byte never written.
				// dma_owned=0 on a stale arm => own_* all 0 => nobody consumes.
				dma_owned <= rx_busy | pbx_busy | tx_busy | subcode_busy;
				dma_owner <= rx_busy  ? OWN_RX  :
				             pbx_busy ? OWN_PBX :
				             tx_busy  ? OWN_TX  : OWN_SUB;
			end else if (dma_ack) begin
				dma_owned <= 1'b0;
			end

			if (write) begin
			case (addr)
				// $08-$09 = INTENA bytes 0-1 (only byte 0 survives mask)
				5'b00100: begin : intena_hi
					reg [31:0] tmp;
					tmp = cdrom_intena;
					if (uds) tmp[31:24] = din[15:8];
					if (lds) tmp[23:16] = din[7:0];
					cdrom_intena <= tmp & INTENA_MASK;
				end
				// $0A-$0B = INTENA bytes 2-3 (always masked to zero)
				5'b00101: begin : intena_lo
					reg [31:0] tmp;
					tmp = cdrom_intena;
					if (uds) tmp[15:8] = din[15:8];
					if (lds) tmp[7:0]  = din[7:0];
					cdrom_intena <= tmp & INTENA_MASK;
				end
				// $10-$11 / $12-$13 = data DMA base (mask 0x00fff000)
				5'b01000: begin : addrdata_hi
					reg [31:0] tmp;
					tmp = cdrom_addressdata;
					if (uds) tmp[31:24] = din[15:8];
					if (lds) tmp[23:16] = din[7:0];
					cdrom_addressdata <= tmp & ADDRDATA_MASK;
				end
				5'b01001: begin : addrdata_lo
					reg [31:0] tmp;
					tmp = cdrom_addressdata;
					if (uds) tmp[15:8] = din[15:8];
					if (lds) tmp[7:0]  = din[7:0];
					cdrom_addressdata <= tmp & ADDRDATA_MASK;
				end
				// $14-$15 / $16-$17 = misc DMA base (mask 0x00fffc00)
				5'b01010: begin : addrmisc_hi
					reg [31:0] tmp;
					tmp = cdrom_addressmisc;
					if (uds) tmp[31:24] = din[15:8];
					if (lds) tmp[23:16] = din[7:0];
					cdrom_addressmisc <= tmp & ADDRMISC_MASK;
				end
				5'b01011: begin : addrmisc_lo
					reg [31:0] tmp;
					tmp = cdrom_addressmisc;
					if (uds) tmp[15:8] = din[15:8];
					if (lds) tmp[7:0]  = din[7:0];
					cdrom_addressmisc <= tmp & ADDRMISC_MASK;
				end
				// $18 byte write = clear SUBCODE IRQ (akiko.cpp:1943-1945)
				5'b01100: begin
					if (uds) begin
						cdrom_intreq <= cdrom_intreq & ~CDINT_SUBCODE;
						subcode_irq  <= 1'b0;
					end
				end
				// $1D byte write = TX compare; clears TXDMADONE IRQ; reloads
				// 3-tick TX inhibit (akiko.cpp:1946-1950)
				5'b01110: begin
					if (lds) begin
						cdcomtxcmp   <= din[7:0];
						cdrom_intreq <= cdrom_intreq & ~CDINT_TXDMADONE;
						tx_dma_delay <= 2'd3;
					end
				end
				// $1F byte write = RX compare; clears RXDMADONE IRQ; reloads
				// 3-tick RX inhibit (akiko.cpp:1951-1955)
				5'b01111: begin
					if (lds) begin
						cdcomrxcmp   <= din[7:0];
						cdrom_intreq <= cdrom_intreq & ~CDINT_RXDMADONE;
						rx_dma_delay <= 2'd3;
					end
				end
				// $20-$21 PBX, set-only OR semantics; clears PBX IRQ (akiko.cpp:1956-1966)
				// PBX register is forced to zero if CONFIG.PBX disabled.
				5'b10000: begin : pbx_w
					reg [15:0] tmp;
					tmp = cdrom_pbx;
					if (uds) tmp[15:8] = tmp[15:8] | din[15:8];
					if (lds) tmp[7:0]  = tmp[7:0]  | din[7:0];
					if (!cdrom_flags[CDFLAG_PBX_BIT]) tmp = 16'h0;
					cdrom_pbx    <= tmp;
					cdrom_intreq <= cdrom_intreq & ~CDINT_PBX;
				end
				// $24-$27 CONFIG (mask 0xff800000), with ENABLE 0->1 clears OVERFLOW IRQ,
				// PBX off clears pbx register (akiko.cpp:1967-1987)
				5'b10010: begin : cfg_high
					reg [31:0] new_flags;
					new_flags = cdrom_flags;
					if (uds) new_flags[31:24] = din[15:8];
					if (lds) new_flags[23:16] = din[7:0];
					new_flags = new_flags & CONFIG_MASK;
					cdrom_flags <= new_flags;
					// CDFLAG_ENABLE 0->1: reset sector_counter and clear OVERFLOW
					// (akiko.cpp:1973-1976).
					//
					// ALSO drop any stale
					// staged sector (sector_ready) and reset the slow-path write
					// pointer. Each READ DATA toggles ENABLE 0->1, resetting the
					// counter to 0. If a sector staged by the PREVIOUS read is
					// still pending (sector_ready=1) when the new read begins, the
					// PBX engine ships THAT stale sector tagged counter&0x1f=0 and
					// PBX_FIN advances the counter to 1 — so the new read's first
					// real fetch is base+1, not base (HW-confirmed: read#2 start=21
					// delivered lba=22). The BIOS then parses a sector-shifted
					// ISO9660 directory and walks a garbage extent. WinUAE starts
					// each read fresh; clearing sector_ready here matches that so
					// the counter=0 fetch (base+0) ships first, tagged 0 — aligning
					// both the data AND the BIOS's per-slot ordering tag. CF (one
					// long streaming read) is unaffected: a single ENABLE 0->1 at
					// boot with an already-empty buffer.
					if (new_flags[CDFLAG_ENABLE_BIT] && !cdrom_flags[CDFLAG_ENABLE_BIT]) begin
						cdrom_intreq         <= cdrom_intreq & ~CDINT_OVERFLOW;
						cdrom_sector_counter <= 8'h0;
						sector_ready         <= 1'b0;
						sec_wr_ptr           <= 11'h0;
						// A new READ DATA invalidates any
						// PBX ship still in flight from the PREVIOUS read so its
						// PBX_FIN cannot bump the counter we just reset to 0 (off-
						// by-one recurrence). The actual SET of pbx_ship_invalid is
						// done in one place after the PBX case (so it also covers a
						// ship STARTING this same cycle and wins same-cycle conflicts
						// against the fresh-ship clear). We do NOT abort the ship —
						// letting it finish its DMA avoids dropping dma_req mid-
						// handshake (cd_dma_req held until dma_ack, akiko.v:1052); it
						// delivers the old read's data to the old buffer, only the
						// counter (which belongs to the NEW read) is protected. The
						// bridge meanwhile waits on sec_req's !pbx_busy term, so it
						// can't push into the buffer the stale ship is still reading.
						// When PBX is already IDLE (validated off-by-one case) this
						// is a pure no-op.
					end
					if (!new_flags[CDFLAG_PBX_BIT]) cdrom_pbx <= 16'h0;
				end
				5'b10011: begin : cfg_low
					reg [31:0] new_flags;
					new_flags = cdrom_flags;
					if (uds) new_flags[15:8] = din[15:8];
					if (lds) new_flags[7:0]  = din[7:0];
					new_flags = new_flags & CONFIG_MASK;
					cdrom_flags <= new_flags;
					// ENABLE/PBX bits are above 23, so the side effects here
					// can never trigger via this address pair, but model the
					// mask anyway so flags stays clean.
				end
				// $28 PIO byte write — uds (M1 stub: just latch + clear IRQ)
				5'b10100: begin
					if (uds) begin
						pio_byte     <= din[15:8];
						cdrom_intreq <= cdrom_intreq & ~CDINT_DRIVEXMIT;
					end
				end
				// $30 / $32 NVRAM I2C — M1 stub
				5'b11000: begin
					if (uds) nvram_io  <= din[15:8];
					if (lds) ;
				end
				5'b11001: begin
					if (uds) nvram_dir <= din[15:8];
				end
				default: ;
			endcase
			end // if (write)

			// -----------------------------------------------------------------
			// TX command DMA engine (akiko.cpp:1196-1235 / cdrom_run_command)
			// Reads one byte from chip RAM at cdtx_address+cdcomtxinx into the
			// command buffer per dma_ack. Arbitration: RX > PBX > TX. We only
			// accept ack in the TX path when both RX and PBX are idle so that
			// dma_ack pulses for higher-priority engines don't false-advance
			// TX. (CDFLAG_ENABLE also gates tx_can_start, so in steady state
			// PBX never coexists with NEW TX, but an in-flight tx_busy could
			// still be present when ENABLE rises mid-burst.)
			// -----------------------------------------------------------------
			if (tx_busy) begin
				// owner-freeze: consume only the ack the arb credited to TX
				if (dma_ack && own_tx) begin
					if (cdrom_command_length != 6'd32)
						cdrom_command_buffer[cdrom_command_length] <= dma_rbyte;
					cdrom_command_length <= cdrom_command_length + 6'd1;
					cdcomtxinx           <= cdcomtxinx + 8'd1;
					if ((cdcomtxinx + 8'd1) == cdcomtxcmp)
						cdrom_intreq <= cdrom_intreq | CDINT_TXDMADONE;
					tx_busy <= 1'b0;
				end
			end else if (!rx_busy && !pbx_busy && tx_can_start && !subcode_busy) begin
				tx_busy <= 1'b1;
			end

			// -----------------------------------------------------------------
			// RX status DMA engine (akiko.cpp:865-903 / cdrom_return_data)
			// Writes result_buffer[receive_offset] to chip RAM at
			// cdrx_address+cdcomrxinx per dma_ack. When the result is fully
			// drained, clear DRIVERECV and set DRIVEXMIT (the "drive ready"
			// signal CD32 Kickstart waits on). RX wins the arbiter.
			//
			// rx_inflight handshake: when rx_busy goes 0->1 mid-PBX-burst, the
			// BFM may already be completing a PBX cycle whose dma_ack arrives
			// the very next cycle. Without arbitration, RX would consume that
			// (unrelated) ack and skip its own write. Solution: only count
			// dma_ack after we've seen at least one cycle of !dma_ack while
			// rx_busy=1 (i.e. the BFM has gone quiet, our request is what it
			// will pick up next). Adds 1 cycle of latency per RX byte; PBX
			// safely re-runs the displaced byte (idempotent write of same data).
			// -----------------------------------------------------------------
			if (rx_busy) begin
				// owner-freeze: ground-truth ownership replaces the rx_inflight
				// "wait for a quiet cycle" heuristic, which only covered the
				// same-cycle case and still dropped RX bytes when RX asserted
				// two or more cycles before the ack. rx_inflight is
				// left wired for the dormant BFM path but no longer gates here.
				if (dma_ack && own_rx) begin
					cdcomrxinx           <= cdcomrxinx + 8'd1;
					cdrom_receive_offset <= cdrom_receive_offset + 6'd1;
					if ((cdrom_receive_offset + 6'd1) == cdrom_receive_length) begin
						cdrom_receive_length <= 6'd0;
						cdrom_receive_offset <= 6'd0;
						// Combine: clear DRIVERECV, set DRIVEXMIT, plus
						// optionally set RXDMADONE if compare also matched.
						cdrom_intreq <= ((cdrom_intreq & ~CDINT_DRIVERECV) | CDINT_DRIVEXMIT)
						              | (((cdcomrxinx + 8'd1) == cdcomrxcmp) ? CDINT_RXDMADONE : 32'h0);
					end else if ((cdcomrxinx + 8'd1) == cdcomrxcmp) begin
						// Rxcmp match mid-delivery sets RXDMADONE but
						// MUST preserve receive_length/offset. WinUAE
						// cdrom_return_data (akiko.cpp:883-895) only `break`s
						// the per-call loop here; the queued response stays
						// pending and the next BIOS bump of rxcmp resumes
						// delivery from the current offset until either
						// offset==length (full delivery, length cleared above)
						// or another rxcmp match (another partial drain).
						//
						// Do NOT clear length/offset on the first rxcmp match.
						// That destroys bytes 1..N-1 of any response BIOS has
						// not pre-sized rxcmp for: the post-INFO 3-byte media-
						// status push collapses to a single byte and BIOS waits
						// forever for a frame it never receives, never advancing
						// to MULTI/TOC.
						cdrom_intreq <= cdrom_intreq | CDINT_RXDMADONE;
					end
					rx_busy     <= 1'b0;
					rx_inflight <= 1'b0;
				end else if (!rx_inflight && !dma_ack) begin
					rx_inflight <= 1'b1;
				end
			end else if (rx_can_start && !subcode_busy) begin
				rx_busy     <= 1'b1;
				rx_inflight <= 1'b0;
			end

			// -----------------------------------------------------------------
			// PBX sector DMA engine (akiko.cpp:1296-1376 / cdrom_run_read).
			// One pass per slot: walk slot+0..2351 writing sector_buffer (with
			// the per-byte rules in sector_byte_at_idx), then walk +0xc00..+0xc91
			// writing zeros, then bump sector_counter and clear the slot bit.
			// Bus mux gives PBX priority over TX but yields to RX.
			// -----------------------------------------------------------------
			case (pbx_state)
				PBX_IDLE: begin
					if (cdrom_flags[CDFLAG_ENABLE_BIT]
					    && cdrom_flags[CDFLAG_PBX_BIT]
					    && (cdrom_pbx != 16'h0)
					    && sector_ready && !subcode_busy) begin
						pbx_seccnt   <= highest_bit(cdrom_pbx);
						pbx_byte_idx <= 12'h0;
						pbx_busy     <= 1'b1;
						pbx_state    <= PBX_DATA;
					end
				end
				PBX_DATA: begin
					if (dma_ack && own_pbx) begin
						if (pbx_byte_idx == 12'd2351) begin
							pbx_byte_idx <= 12'h0;
							pbx_state    <= PBX_ZERO;
						end else begin
							pbx_byte_idx <= pbx_byte_idx + 12'd1;
						end
					end
				end
				PBX_ZERO: begin
					if (dma_ack && own_pbx) begin
						if (pbx_byte_idx == 12'd145) begin
							pbx_byte_idx <= 12'h0;
							pbx_state    <= PBX_FIN;
						end else begin
							pbx_byte_idx <= pbx_byte_idx + 12'd1;
						end
					end
				end
				PBX_FIN: begin
					cdrom_pbx[pbx_seccnt] <= 1'b0;
					cdrom_intreq          <= cdrom_intreq | CDINT_PBX;
					// Advance the counter ONLY for a ship not invalidated by an
					// intervening READ DATA. Two cases are suppressed:
					//   pbx_ship_invalid: a prior-cycle enable_rising marked this
					//     ship stale (it began under the previous read; ENABLE 0->1
					//     already reset the counter to 0) — bumping now would make
					//     the new read's first fetch base+1 (off-by-one recurrence).
					//   enable_rising: a new read starts THIS same cycle; its
					//     cfg_high counter<=0 (textually earlier) would otherwise
					//     lose the NBA race to this +1. !enable_rising lets it stand.
					if (!pbx_ship_invalid && !enable_rising)
						cdrom_sector_counter <= cdrom_sector_counter + 8'd1;
					sector_ready          <= 1'b0;
					pbx_busy              <= 1'b0;
					pbx_state             <= PBX_IDLE;
				end
			endcase

			// Centralized sticky ship-invalidation.
			// Placed AFTER the PBX case so it wins same-cycle NBA conflicts
			// against the fresh-ship clear and also covers a ship STARTING this
			// cycle (pbx_starting). A ship that is live (in flight OR starting)
			// when a new READ DATA fires belongs to the PREVIOUS read -> mark it
			// invalid; the SET is sticky and idempotent under repeated rapid
			// rises, where a toggle-gen would alias back to "valid" after an
			// even number of rises. A fresh ship starting
			// with no concurrent enable_rising belongs to the current read ->
			// clear (valid). Otherwise hold the sticky value.
			if (enable_rising && (pbx_busy || pbx_starting))
				pbx_ship_invalid <= 1'b1;
			else if (pbx_starting)
				pbx_ship_invalid <= 1'b0;

			// -----------------------------------------------------------------
			// HPS bridge: sector-data in. One write port, one source: the
			// host pushes 1176 words over the ext bus. Keeping it to a single
			// `sector_buffer[addr] <= din` statement is what lets Quartus
			// infer M10K instead of building the array out of ALMs.
			// -----------------------------------------------------------------
			if (sec_w_we && !sector_ready && !enable_rising) begin
				sector_buffer[sec_w_addr] <= sec_w_din;
			end

			// Guard the sector_ready SET with
			// !enable_rising. A fast-fill byte-2351 that lands in the same cycle
			// as a new READ DATA (ENABLE 0->1) must NOT re-assert sector_ready
			// after the restart cleared it — that race is exactly what let the
			// bridge push a full sector into an already-"full" buffer (every
			// byte gated off, fill=0) and ship a stale sector. The fill belongs
			// to the OLD read and is discarded; the new read re-fetches base+0.
			// !enable_rising: a new READ DATA resets sec_wr_ptr<=0 in the
			// cfg_high block (textually earlier); without this guard a same-
			// cycle increment would win the NBA and leave the ptr at 1 after
			// a restart.
			if (hps_sec_push && !sector_ready && sec_wr_ptr != 11'd1176
			    && !enable_rising) begin
				sec_wr_ptr <= sec_wr_ptr + 11'd1;
			end
			if (hps_sec_done) begin
				if (sec_wr_ptr == 11'd1176 && !enable_rising) sector_ready <= 1'b1;
				sec_wr_ptr <= 11'h0;
			end

			// -----------------------------------------------------------------
			// HPS bridge: subcode-block in + DMA out (CDDA position heartbeat).
			// Main pushes a 96-byte INTERLEAVED subchannel block per CD frame
			// during play. On done, if CDFLAG_SUBCODE is set the FSM DMAs 96
			// bytes + a 0xffff0000 marker to subcode_address + (ping-pong 0/128)
			// and raises CDINT_SUBCODE — the heartbeat the in-game music manager
			// waits on (WinUAE akiko.cpp:1486-1509). Flag clear -> block dropped,
			// so non-subcode games are unaffected.
			// -----------------------------------------------------------------
			if (hps_subcode_push && !subcode_ready && !subcode_busy
			    && sub_wr_ptr != 7'd96) begin
				subbuf[sub_wr_ptr[6:0]] <= hps_subcode_byte;
				sub_wr_ptr <= sub_wr_ptr + 7'd1;
			end
			if (hps_subcode_done) begin
				if (sub_wr_ptr == 7'd96 && !subcode_busy) subcode_ready <= 1'b1;
				sub_wr_ptr <= 7'h0;
			end

			case (subcode_state)
				SUB_IDLE: begin
					if (subcode_ready) begin
						if (!cdrom_flags[CDFLAG_SUBCODE_BIT]) begin
							subcode_ready <= 1'b0; // flag off: drop, no DMA/IRQ
						end else if (!others_busy && !others_starting) begin
							// Claim only when rx/pbx/tx are idle and none is about to
							// start -> subcode never overlaps a higher engine, so the
							// chipdma_arb ack is unambiguously ours and we cannot
							// corrupt another engine's in-flight transfer.
							subcode_off   <= (cdrom_subcodeoffset >= 8'd128) ? 8'd0 : 8'd128;
							sub_idx       <= 8'h0;
							subcode_busy  <= 1'b1;
							subcode_state <= SUB_DATA;
						end
						// else: bus contended -> hold the block, retry next cycle
					end
				end
				SUB_DATA: begin
					// Walk is ATOMIC: once claimed the bus is exclusively ours, so we
					// DMA all 100 bytes then IRQ. A mid-walk CDFLAG_SUBCODE clear is
					// deliberately NOT aborted here: dropping subcode_busy with a byte
					// already armed in chipdma_arb would let rx/pbx/tx start and consume
					// the stale subcode ack (the same ownership race).
					// Delivering one final heartbeat block is benign (game ignores it).
					// Gate on the frozen owner,
					// not the live subcode_grant, so a stale no-live-engine arm
					// (which sets dma_owned=0) can never advance subcode.
					if (dma_ack && own_sub) begin
						if (sub_idx == 8'd99) subcode_state <= SUB_FIN;
						else                  sub_idx <= sub_idx + 8'd1;
					end
				end
				SUB_FIN: begin
					cdrom_subcodeoffset <= subcode_off + 8'd100;
					subcode_irq         <= 1'b1;
					subcode_ready       <= 1'b0;
					subcode_busy        <= 1'b0;
					subcode_state       <= SUB_IDLE;
				end
			endcase

			// -----------------------------------------------------------------
			// HPS bridge: command-stream out (Main reads framed command bytes).
			// hps_cmd_byte is combinational at hps_cmd_rd_ptr; pop advances the
			// pointer; done releases the framer for the next packet.
			// `cmd_pending` gates TX so the buffer can't grow under us between
			// pop and done.
			// -----------------------------------------------------------------
			if (hps_cmd_pop && (hps_cmd_rd_ptr != 6'd32))
				hps_cmd_rd_ptr <= hps_cmd_rd_ptr + 6'd1;
			if (hps_cmd_done) begin
				cdrom_command_length <= 6'd0;
				hps_cmd_rd_ptr       <= 6'd0;
			end

			// -----------------------------------------------------------------
			// HPS bridge: result-stream in (Main writes response bytes, then
			// pulses done to commit). `done` only takes effect when the RX
			// engine is idle (receive_length == 0); the previous DMA cleared
			// receive_offset to 0 already, so commit is a clean kick.
			// -----------------------------------------------------------------
			if (hps_result_push && (hps_result_wr_ptr != 6'd32)) begin
				cdrom_result_buffer[hps_result_wr_ptr[4:0]] <= hps_result_byte;
				hps_result_wr_ptr <= hps_result_wr_ptr + 6'd1;
			end
			if (hps_result_done && (cdrom_receive_length == 6'd0)) begin
				cdrom_receive_length <= hps_result_wr_ptr;
				hps_result_wr_ptr    <= 6'd0;
				// DRIVERECV only, never SUBCODE. DRIVERECV alone signals
				// "result ready", which is what the BIOS path needs; BIOS
				// reads SUBCODE as "drive playing, subcode coming" and stalls
				// waiting for subcode data that is not on its way.
				cdrom_intreq         <= cdrom_intreq | CDINT_DRIVERECV;
			end

			// ---------------------------------------------------------
			// Restore. Last in the block so it wins over everything
			// above it, the same placement ciaa.v uses. The CPU is
			// parked at an instruction boundary while this runs and
			// Akiko was quiesced before the capture, so there is no
			// legitimate bus or DMA activity to lose to it.
			//
			// The transient registers are not written from the vector
			// because they are not in it -- they are forced to their
			// idle values instead. That is not a shortcut: the freeze
			// only happened because they were already idle, so this
			// writes back exactly what was there. Doing it explicitly
			// means a restore into a machine whose Akiko is mid-transfer
			// (a slot that was saved before the idle rule existed, say)
			// lands in a consistent state rather than a wedged one.
			// ---------------------------------------------------------
			if (ss_ld) begin
				cdrom_intreq         <= ss_ld_data[SS_O_INTREQ   +: 32];
				cdrom_intena         <= ss_ld_data[SS_O_INTENA   +: 32];
				cdrom_addressdata    <= ss_ld_data[SS_O_ADDRDATA +: 32];
				cdrom_addressmisc    <= ss_ld_data[SS_O_ADDRMISC +: 32];
				cdrom_flags          <= ss_ld_data[SS_O_FLAGS    +: 32];
				cdrom_pbx            <= ss_ld_data[SS_O_PBX      +: 16];
				cdrom_subcodeoffset  <= ss_ld_data[SS_O_SUBCOFF  +:  8];
				cdcomtxinx           <= ss_ld_data[SS_O_TXINX    +:  8];
				cdcomrxinx           <= ss_ld_data[SS_O_RXINX    +:  8];
				cdcomtxcmp           <= ss_ld_data[SS_O_TXCMP    +:  8];
				cdcomrxcmp           <= ss_ld_data[SS_O_RXCMP    +:  8];
				subcode_off          <= ss_ld_data[SS_O_SUBOFF   +:  8];
				cdrom_sector_counter <= ss_ld_data[SS_O_SECCNT   +:  8];
				nvram_io             <= ss_ld_data[SS_O_NVRIO    +:  8];
				nvram_dir            <= ss_ld_data[SS_O_NVRDIR   +:  8];
				pio_byte             <= ss_ld_data[SS_O_PIO      +:  8];
				subcode_irq          <= ss_ld_data[SS_O_SUBIRQ];
				pbx_ship_invalid     <= ss_ld_data[SS_O_SHIPINV];

				// The command path travels by value: a queued response
				// the driver has not finished reading is normal state, not
				// a transient, so it goes back exactly as it was. See the
				// ss_idle comment for why it cannot be waited out instead.
				cdrom_command_length <= ss_ld_data[SS_O_CMDLEN +: 6];
				cdrom_receive_length <= ss_ld_data[SS_O_RXLEN  +: 6];
				cdrom_receive_offset <= ss_ld_data[SS_O_RXOFF  +: 6];
				for (ss_bj = 0; ss_bj < 32; ss_bj = ss_bj + 1) begin
					cdrom_command_buffer[ss_bj] <=
						ss_ld_data[SS_O_CMDBUF + ss_bj*8 +: 8];
					cdrom_result_buffer[ss_bj]  <=
						ss_ld_data[SS_O_RESBUF + ss_bj*8 +: 8];
				end

				// Transients, forced idle. See above.
				tx_dma_delay         <= 2'h0;
				rx_dma_delay         <= 2'h0;
				tx_busy              <= 1'b0;
				rx_busy              <= 1'b0;
				rx_inflight          <= 1'b0;
				dma_owned            <= 1'b0;
				hps_cmd_rd_ptr       <= 6'h0;
				hps_result_wr_ptr    <= 6'h0;
				sec_wr_ptr           <= 11'h0;
				sector_ready         <= 1'b0;
				sub_wr_ptr           <= 7'h0;
				subcode_ready        <= 1'b0;
				subcode_busy         <= 1'b0;
				subcode_state        <= SUB_IDLE;
				sub_idx              <= 8'h0;
				pbx_busy             <= 1'b0;
				pbx_state            <= PBX_IDLE;
				pbx_seccnt           <= 4'h0;
				pbx_byte_idx         <= 12'h0;
			end
		end // else !reset
	end

	// Effective INTREQ: fold in the dedicated subcode IRQ latch so the SUBCODE
	// streaming interrupt never races the many cdrom_intreq writers.
	wire [31:0] cdrom_intreq_eff = cdrom_intreq | (subcode_irq ? CDINT_SUBCODE : 32'h0);

	// Read mux
	reg [15:0] cd_dout_r;
	always @(*) begin
		cd_dout_r = 16'h0;
		case (addr)
			// $04-$05 INTREQ high half
			5'b00010: cd_dout_r = cdrom_intreq_eff[31:16];
			// $06-$07 INTREQ low half
			5'b00011: cd_dout_r = cdrom_intreq_eff[15:0];
			// $08-$09 INTENA high half
			5'b00100: cd_dout_r = cdrom_intena[31:16];
			// $0A-$0B INTENA low half
			5'b00101: cd_dout_r = cdrom_intena[15:0];
			// $0C-$0F INTENA mirror (akiko.cpp:1747-1753)
			5'b00110: cd_dout_r = cdrom_intena[31:16];
			5'b00111: cd_dout_r = cdrom_intena[15:0];
			// $10-$1F: read-mirror block — addr[1] selects the pair
			//   addr[1]=0 ($10/$14/$18/$1C bytes 0,1) -> {subcodeoffset, txinx}
			//   addr[1]=1 ($12/$16/$1A/$1E bytes 2,3) -> {rxinx, 0}
			// (akiko.cpp:1755-1783)
			5'b01000, 5'b01010, 5'b01100, 5'b01110:
				cd_dout_r = {cdrom_subcodeoffset, cdcomtxinx};
			5'b01001, 5'b01011, 5'b01101, 5'b01111:
				cd_dout_r = {cdcomrxinx, 8'h0};
			// $20-$21 PBX (akiko.cpp:1785-1788)
			5'b10000: cd_dout_r = cdrom_pbx;
			// $24-$27 CONFIG (akiko.cpp:1789-1794)
			5'b10010: cd_dout_r = cdrom_flags[31:16];
			5'b10011: cd_dout_r = cdrom_flags[15:0];
			// $28 PIO byte read — stub, returns last write in upper byte
			5'b10100: cd_dout_r = {pio_byte, 8'h0};
			// $30 NVRAM I/O byte — reflects the live I2C bus state
			// (master's drives ANDed with the slave's open-drain
			// pull-down via akiko_nvram). bit 7 = SCL, bit 6 = SDA;
			// remaining bits are 0. WinUAE akiko.cpp:285-296 reads back
			// the same shape from eeprom_i2c_set() returns.
			5'b11000: cd_dout_r = {nvram_scl_bus, nvram_sda_bus, 6'h0, 8'h0};
			// $32 direction — read-back of last write (master's choice
			// of which lines are inputs vs outputs).
			5'b11001: cd_dout_r = {nvram_dir, 8'h0};
			default:  cd_dout_r = 16'h0;
		endcase
	end

	assign cd_dout = cs ? cd_dout_r : 16'h0;
	assign cd_irq  = |(cdrom_intreq_eff[31:25] & cdrom_intena[31:25]);

	// Master DMA port — arbitration RX > PBX > TX. While idle the bus is
	// held LOW. dma_we is don't-care during TX (read), 1 for RX/PBX writes.
	wire subcode_grant  = subcode_busy & ~rx_busy & ~pbx_busy & ~tx_busy;
	assign cd_dma_req   = tx_busy | rx_busy | pbx_busy | subcode_busy;
	assign cd_dma_we    = rx_busy | pbx_busy | subcode_grant;
	assign cd_dma_baddr = rx_busy  ? (cdrx_address + {16'h0, cdcomrxinx}) :
	                      pbx_busy ? pbx_addr :
	                      tx_busy  ? (cdtx_address + {16'h0, cdcomtxinx}) :
	                                 subcode_dma_addr;
	assign cd_dma_wbyte = rx_busy  ? cdrom_result_buffer[cdrom_receive_offset] :
	                      pbx_busy ? pbx_wbyte :
	                                 subcode_dma_byte;

	// HPS bridge outputs (status + current command-stream byte).
	assign cd_hps_cmd_pending = cmd_pending;
	assign cd_hps_cmd_byte    = cdrom_command_buffer[hps_cmd_rd_ptr[4:0]];

	// HPS sector-channel outputs.
	assign cd_hps_sec_req     = sec_req_w;
	assign cd_hps_sec_status  = cdrom_sector_counter;

	// rx_busy out — receive engine has a queued or in-flight response.
	assign cd_hps_rx_busy     = (cdrom_receive_length != 6'd0);

	// -------------------------------------------------------------------
	// Save state: capture, restore, and the idle condition that makes the
	// pair honest.
	//
	// ss_idle covers exactly one thing: no DMA engine may be part-way
	// through a transfer when the machine freezes. Nothing else.
	//
	// It was written the other way round first -- every staging buffer had
	// to be empty as well -- on the reasoning that a staged sector would be
	// lost, because the HPS had already handed it over and moved on. Two of
	// those terms are not transient at all, and on hardware the freeze then
	// never happened: every save of a running CD32 title reported
	// FAIL_QUIESCE, and so did every diagnostic peek, which shares the
	// condition.
	//
	//   sector_ready is set when a sector arrives and cleared only when the
	//   PBX engine ships it, which needs the title to free a slot. Between
	//   loads a prefetched sector sits staged for as long as the game likes.
	//
	//   hps_result_wr_ptr is never cleared if hps_result_done arrives while
	//   a response is still queued -- the commit is skipped and the pointer
	//   keeps its value with nothing left to clear it.
	//
	// The premise was wrong as well as the terms. A staged sector is NOT
	// lost by dropping it, because cdrom_sector_counter advances at PBX_FIN
	// -- when a sector is SHIPPED, not when it arrives -- and userspace
	// fetches cd_data_lba_base + counter. So a sector that arrived but was
	// never shipped is precisely the sector at base + counter: clear
	// sector_ready and the very next hps_sec_req asks for the same LBA
	// again. The restore forces those registers idle already, so this is
	// self-healing rather than merely tolerable. The same holds for a
	// partly filled sector and for a subcode block, which is one 96-byte
	// subchannel frame out of seventy-five a second during CDDA.
	//
	// tx_dma_delay and rx_dma_delay are the 3-tick post-write inhibit:
	// nonzero means a transfer has been asked for and has not started yet,
	// which is no more restorable than one already running.
	//
	// The two command buffers travel in the state vector, so nothing here
	// needs to wait on them. That was the right call for a different reason
	// -- cdrom_receive_length is non-zero for as long as the driver leaves a
	// response half-drained, which is a resting state, not a transient one.
	assign cd_ss_idle =
	         ~pbx_busy & ~subcode_busy & ~tx_busy & ~rx_busy
	       & ~rx_inflight & ~dma_owned
	       & (pbx_state == PBX_IDLE) & (subcode_state == SUB_IDLE)
	       & (tx_dma_delay == 2'd0) & (rx_dma_delay == 2'd0);

	// The two 32-byte command buffers, out to the vector. Generate loops for
	// the same reason the C2P buffer uses one: these are arrays everywhere
	// else, and a hand-written 32-term concatenation is a transposition
	// waiting to happen.
	wire [255:0] cmdbuf_ss;
	wire [255:0] resbuf_ss;
	genvar ss_bi;
	for (ss_bi = 0; ss_bi < 32; ss_bi = ss_bi + 1) begin : g_cmdbuf_ss
		assign cmdbuf_ss[ss_bi*8 +: 8] = cdrom_command_buffer[ss_bi];
		assign resbuf_ss[ss_bi*8 +: 8] = cdrom_result_buffer[ss_bi];
	end

	assign cd_ss_native = {
		cdrom_receive_offset,   // [787:782]
		cdrom_receive_length,   // [781:776]
		resbuf_ss,              // [775:520]
		cdrom_command_length,   // [519:514]
		cmdbuf_ss,              // [513:258]
		pbx_ship_invalid,       // [257]
		subcode_irq,            // [256]
		pio_byte,               // [255:248]
		nvram_dir,              // [247:240]
		nvram_io,               // [239:232]
		cdrom_sector_counter,   // [231:224]
		subcode_off,            // [223:216]
		cdcomrxcmp,             // [215:208]
		cdcomtxcmp,             // [207:200]
		cdcomrxinx,             // [199:192]
		cdcomtxinx,             // [191:184]
		cdrom_subcodeoffset,    // [183:176]
		cdrom_pbx,              // [175:160]
		cdrom_flags,            // [159:128]
		cdrom_addressmisc,      // [127:96]
		cdrom_addressdata,      // [95:64]
		cdrom_intena,           // [63:32]
		cdrom_intreq            // [31:0]
	};

	// I2C slave EEPROM (1 KiB, 24LC08-equivalent). Pulls SDA
	// low for ACK and read-data; never drives SCL. Volatile BRAM —
	// persistence is a separate feature.
	akiko_nvram nvram_inst (
		.clk              (clk),
		// Slave I²C state machine uses initial-value powerup + bus-protocol
		// STOP/START recovery; no async reset on the slave (decoupled from
		// the chip-wide `reset` = ~cpu_rst | ~cpu_nrst_out so HPS-side
		// load via load_we works regardless of CD32 CPU reset state).
		.reset            (1'b0),
		.scl_in           (nvram_scl_bus),
		.sda_in           (nvram_sda_bus),
		.sda_drive        (nvram_slave_sda_drive),

		// Save-dump read port (driven by akiko_hps_bridge's read counter).
		.host_addr        (hps_nvr_addr),
		.host_dout        (cd_hps_nvr_dout),
		.host_clear_dirty (hps_nvr_clear_dirty),
		.nvram_dirty      (cd_hps_nvr_dirty),

		// Load-from-disk write port (driven by hps_io.ioctl_download).
		.load_addr        (nvr_load_addr),
		.load_din         (nvr_load_din),
		.load_we          (nvr_load_we)
	);

end else begin : g_stub
	assign cd_dout            = 16'h0;
	assign cd_irq             = 1'b0;
	assign cd_dma_req         = 1'b0;
	assign cd_dma_we          = 1'b0;
	assign cd_dma_baddr       = 24'h0;
	assign cd_dma_wbyte       = 8'h0;
	assign cd_hps_cmd_pending = 1'b0;
	assign cd_hps_cmd_byte    = 8'h0;
	assign cd_hps_sec_req     = 1'b0;
	assign cd_hps_sec_status  = 8'h0;
	assign cd_hps_rx_busy     = 1'b0;
	assign cd_hps_nvr_dout    = 8'h0;
	assign cd_hps_nvr_dirty   = 1'b0;
	// No CD block to be busy, so never a reason to hold up a freeze. The C2P
	// half of the vector is still real in this build and is captured above.
	assign cd_ss_native       = 788'h0;
	assign cd_ss_idle         = 1'b1;
end
endgenerate

// -----------------------------------------------------------------------
// Combined dout (ID + C2P + CD regs). Each contributor is zero outside
// its address range, so a simple OR is safe (matches WinUAE bget2 default
// of returning 0 for unknown addresses).
// -----------------------------------------------------------------------
always @(*) begin
	dout = 16'h0;
	if (cs) begin
		if (addr == 5'd0) dout = 16'hC0CA;
		if (addr == 5'd1) dout = 16'hCAFE;
		if (c2p_sel)      dout = c2p_dout;
	end
	dout = dout | cd_dout;
end

assign akiko_irq = cd_irq;

assign dma_req   = cd_dma_req;
assign dma_we    = cd_dma_we;
assign dma_baddr = cd_dma_baddr;
assign dma_wbyte = cd_dma_wbyte;

assign hps_cmd_pending = cd_hps_cmd_pending;
assign hps_cmd_byte    = cd_hps_cmd_byte;

assign hps_sec_req     = cd_hps_sec_req;
assign hps_sec_status  = cd_hps_sec_status;

assign hps_rx_busy     = cd_hps_rx_busy;

// NVRAM save-dump port out to fastchip / bridge.
assign hps_nvr_dout    = cd_hps_nvr_dout;
assign hps_nvr_dirty   = cd_hps_nvr_dirty;

// Save state out. The CD register block occupies the low 258 bits; the C2P
// buffer and its two pointers, which exist in every build, sit above it.
assign ss_state = { wptr, rptr, c2p_ss_buf, cd_ss_native };
assign ss_idle  = cd_ss_idle;

endmodule
