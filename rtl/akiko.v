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
// research/repos/WinUAE/akiko.cpp (akiko_bget2/akiko_bput2). Citations
// in comments below reference WinUAE line numbers at SHA 2c7f8581.
//
// Notes worth remembering for later milestones (M2+):
//   * The cmd/status DMA sub-buffer offsets are documented inconsistently
//     in WinUAE: the top-of-file comment claims base+0x000 is the command
//     buffer, but the executable code (akiko.cpp:1939-1941) uses
//       cdrx_address  = base | 0x000  // drive -> memory (response/status)
//       cdtx_address  = base | 0x200  // memory -> drive (command)
//       subcode_addr  = base | 0x100
//     The code is the truth. M1 only stores the masked base; M2 must use
//     these offsets when implementing TX/RX DMA.
//   * The "data DMA base must be 64K aligned" comment disagrees with the
//     code mask 0x00fff000 (4K alignment). Code wins; we use 4K.
//
//----------------------------------------------------------------------------------

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
	// HPS sector channel (M4: 2352-byte raw-sector pushes from Main, plus a
	// 1-byte status read for the current cdrom_sector_counter). Inactive
	// (zero) when NATIVE_CD32 = 0.
	// ---------------------------------------------------------------------
	output            hps_sec_req,     // status: I have a free PBX slot and an empty buffer
	output      [7:0] hps_sec_status,  // 1-byte read mux (currently == cdrom_sector_counter)
	input             hps_sec_push,    // pulse: store hps_sec_byte at sec_wr_ptr++
	input       [7:0] hps_sec_byte,
	input             hps_sec_done,    // pulse: commit; if sec_wr_ptr == 12'd2352, sector_ready<=1

	// Phase 18: rx_busy = receive engine has a queued or in-flight response.
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

	// M5+ fast sector path via hps_io's UIO_SECTOR_RD pipeline.
	// Coexists with hps_sec_push/byte/done (the slow per-byte SSPI_ACK
	// path); userspace picks one per push. When userspace sends
	// `spi_w(UIO_SECTOR_RD | (AKIKO_SEC_SLOT<<8))` followed by a 2352-byte
	// fast block write, hps_io drives sd_ack[AKIKO_SEC_SLOT] high for the
	// whole transfer and pulses sd_buff_wr per byte with sd_buff_addr
	// auto-incrementing 0..2351. The pipeline absorbs back-to-back bytes
	// at SPI clock without dropping (which the per-cs/sec_push path can't,
	// see research/docs/known-issues-deferred.md "Per-sector SPI throughput
	// vs WinUAE"). All four signals tied 0 leaves only the legacy path active.
	input             hps_sec_dma_active,  // = sd_ack[AKIKO_SEC_SLOT]
	input       [7:0] hps_sec_dma_byte,    // = sd_buff_dout
	input      [13:0] hps_sec_dma_addr,    // = sd_buff_addr
	input             hps_sec_dma_we       // = sd_buff_wr
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
// Existing C2P logic (preserved bit-equivalent to legacy akiko.v)
// -----------------------------------------------------------------------
wire c2p_sel = (addr[5:2] == 'b1110);

reg [7:0] buff[32];
reg [3:0] rptr = 0, wptr = 0;

always @(posedge clk) begin
	if((wr|rd) & cs & c2p_sel) begin
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

	// Phase 13: real I2C slave EEPROM (1 KiB, 24LC08-equivalent) replaces
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

	// M3 HPS bridge state.
	// hps_cmd_rd_ptr indexes into cdrom_command_buffer for the bridge's
	// command-stream read; hps_result_wr_ptr accumulates response bytes
	// from the bridge before commit. Both reset to 0 on transaction
	// boundaries (hps_cmd_done / hps_result_done).
	reg  [5:0] hps_cmd_rd_ptr;
	reg  [5:0] hps_result_wr_ptr;

	// M4 PBX sector DMA state.
	//
	// sector_buffer holds one raw 2352-byte sector pushed by Main via the
	// HPS sector channel. sector_ready latches when a full sector arrives
	// (sec_wr_ptr == 2352 at hps_sec_done). cdrom_sector_counter resets to
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
	reg  [7:0] sector_buffer [2352];
	reg [11:0] sec_wr_ptr;
	reg        sector_ready;
	reg  [7:0] cdrom_sector_counter;
	reg        pbx_busy;
	reg  [1:0] pbx_state;
	localparam PBX_IDLE = 2'd0;
	localparam PBX_DATA = 2'd1;
	localparam PBX_ZERO = 2'd2;
	localparam PBX_FIN  = 2'd3;
	reg  [3:0] pbx_seccnt;       // selected slot (0..15)
	reg [11:0] pbx_byte_idx;     // 0..2351 in DATA, 0..145 in ZERO

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
	// Phase 32 timing fix: register pbx_addr so the SDRAM-bound critical
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
	wire [7:0]  sector_byte_at_idx = (pbx_byte_idx <  12'd3   ) ? 8'h00 :
	                                 (pbx_byte_idx == 12'd3   ) ? (cdrom_sector_counter & 8'h1f) :
	                                 (pbx_byte_idx <  12'd2352) ? sector_buffer[pbx_byte_idx] :
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

	// sec_req: high when PBX wants a sector but the staging buffer is empty.
	// Drops as soon as Main commits a sector (sector_ready -> 1). Rises again
	// after PBX_FIN clears sector_ready, if more pbx slots remain.
	wire sec_req_w =  cdrom_flags[CDFLAG_ENABLE_BIT]
	               && cdrom_flags[CDFLAG_PBX_BIT]
	               && (cdrom_pbx != 16'h0)
	               && !sector_ready;

	wire write = wr & cs;

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
			hps_cmd_rd_ptr       <= 6'h0;
			hps_result_wr_ptr    <= 6'h0;
			sec_wr_ptr           <= 12'h0;
			sector_ready         <= 1'b0;
			cdrom_sector_counter <= 8'h0;
			pbx_busy             <= 1'b0;
			pbx_state            <= PBX_IDLE;
			pbx_seccnt           <= 4'h0;
			pbx_byte_idx         <= 12'h0;
		end else begin
			// 3-tick post-write delay decay (akiko.cpp:1949,1954)
			if (tx_dma_delay != 2'd0) tx_dma_delay <= tx_dma_delay - 2'd1;
			if (rx_dma_delay != 2'd0) rx_dma_delay <= rx_dma_delay - 2'd1;

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
					if (uds) cdrom_intreq <= cdrom_intreq & ~CDINT_SUBCODE;
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
					if (new_flags[CDFLAG_ENABLE_BIT] && !cdrom_flags[CDFLAG_ENABLE_BIT]) begin
						cdrom_intreq         <= cdrom_intreq & ~CDINT_OVERFLOW;
						cdrom_sector_counter <= 8'h0;
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
				if (dma_ack && !rx_busy && !pbx_busy) begin
					if (cdrom_command_length != 6'd32)
						cdrom_command_buffer[cdrom_command_length] <= dma_rbyte;
					cdrom_command_length <= cdrom_command_length + 6'd1;
					cdcomtxinx           <= cdcomtxinx + 8'd1;
					if ((cdcomtxinx + 8'd1) == cdcomtxcmp)
						cdrom_intreq <= cdrom_intreq | CDINT_TXDMADONE;
					tx_busy <= 1'b0;
				end
			end else if (!rx_busy && !pbx_busy && tx_can_start) begin
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
				if (rx_inflight && dma_ack) begin
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
						// Phase 12: rxcmp match mid-delivery sets RXDMADONE but
						// MUST preserve receive_length/offset. WinUAE
						// cdrom_return_data (akiko.cpp:883-895) only `break`s
						// the per-call loop here; the queued response stays
						// pending and the next BIOS bump of rxcmp resumes
						// delivery from the current offset until either
						// offset==length (full delivery, length cleared above)
						// or another rxcmp match (another partial drain).
						//
						// The previous "M5 truncation hack" cleared length/offset
						// on the first rxcmp match, destroying bytes 1..N-1 of
						// any response BIOS hadn't pre-sized rxcmp for. That
						// turned the post-INFO 3-byte media-status push into a
						// single byte, leaving BIOS waiting for a frame it
						// never received and never advancing to MULTI/TOC.
						cdrom_intreq <= cdrom_intreq | CDINT_RXDMADONE;
					end
					rx_busy     <= 1'b0;
					rx_inflight <= 1'b0;
				end else if (!rx_inflight && !dma_ack) begin
					rx_inflight <= 1'b1;
				end
			end else if (rx_can_start) begin
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
					    && sector_ready) begin
						pbx_seccnt   <= highest_bit(cdrom_pbx);
						pbx_byte_idx <= 12'h0;
						pbx_busy     <= 1'b1;
						pbx_state    <= PBX_DATA;
					end
				end
				PBX_DATA: begin
					if (dma_ack && !rx_busy) begin
						if (pbx_byte_idx == 12'd2351) begin
							pbx_byte_idx <= 12'h0;
							pbx_state    <= PBX_ZERO;
						end else begin
							pbx_byte_idx <= pbx_byte_idx + 12'd1;
						end
					end
				end
				PBX_ZERO: begin
					if (dma_ack && !rx_busy) begin
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
					cdrom_sector_counter  <= cdrom_sector_counter + 8'd1;
					sector_ready          <= 1'b0;
					pbx_busy              <= 1'b0;
					pbx_state             <= PBX_IDLE;
				end
			endcase

			// -----------------------------------------------------------------
			// HPS bridge: sector-data in (Main pushes 2352 raw bytes per
			// sector). Bridge guarantees push and done don't overlap (done
			// fires one cycle after deselect), so the simple pointer-vs-2352
			// check below is race-free.
			// -----------------------------------------------------------------
			if (hps_sec_push && !sector_ready && sec_wr_ptr != 12'd2352) begin
				sector_buffer[sec_wr_ptr] <= hps_sec_byte;
				sec_wr_ptr <= sec_wr_ptr + 12'd1;
			end
			if (hps_sec_done) begin
				if (sec_wr_ptr == 12'd2352) sector_ready <= 1'b1;
				sec_wr_ptr <= 12'h0;
			end

			// M5+ fast sector path via UIO_SECTOR_RD pipeline. Bytes stream
			// in directly addressed by sd_buff_addr (which hps_io resets to
			// 0 at byte_cnt==0 and auto-increments via the b_wr<<1 cascade).
			// Last byte (addr=2351) latches sector_ready; PBX state machine
			// clears it on consume. Independent of the legacy path above —
			// only one is active per transfer because they use different
			// hps_io commands (0x17 vs 0x61), and the legacy path's gating
			// signals (hps_sec_push) stay 0 during a SECTOR_RD transfer.
			if (hps_sec_dma_active && hps_sec_dma_we && !sector_ready
			    && hps_sec_dma_addr < 14'd2352) begin
				sector_buffer[hps_sec_dma_addr[11:0]] <= hps_sec_dma_byte;
				if (hps_sec_dma_addr == 14'd2351) sector_ready <= 1'b1;
			end

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
				// v20: drop the M5 SUBCODE-on-push hack; BIOS may interpret
				// SUBCODE as "drive playing, subcode coming" and stall waiting
				// for actual subcode data. DRIVERECV alone signals "result
				// ready" which is what the BIOS path actually needs.
				cdrom_intreq         <= cdrom_intreq | CDINT_DRIVERECV;
			end
		end // else !reset
	end

	// Read mux
	reg [15:0] cd_dout_r;
	always @(*) begin
		cd_dout_r = 16'h0;
		case (addr)
			// $04-$05 INTREQ high half
			5'b00010: cd_dout_r = cdrom_intreq[31:16];
			// $06-$07 INTREQ low half
			5'b00011: cd_dout_r = cdrom_intreq[15:0];
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
			// $28 PIO byte read — M1 stub returns last write in upper byte
			5'b10100: cd_dout_r = {pio_byte, 8'h0};
			// $30 NVRAM I/O byte — Phase 13: now reflects the live I2C bus
			// state (master's drives ANDed with the slave's open-drain
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
	assign cd_irq  = |(cdrom_intreq[31:25] & cdrom_intena[31:25]);

	// Master DMA port — arbitration RX > PBX > TX. While idle the bus is
	// held LOW. dma_we is don't-care during TX (read), 1 for RX/PBX writes.
	assign cd_dma_req   = tx_busy | rx_busy | pbx_busy;
	assign cd_dma_we    = rx_busy | pbx_busy;
	assign cd_dma_baddr = rx_busy  ? (cdrx_address + {16'h0, cdcomrxinx}) :
	                      pbx_busy ? pbx_addr :
	                                 (cdtx_address + {16'h0, cdcomtxinx});
	assign cd_dma_wbyte = rx_busy  ? cdrom_result_buffer[cdrom_receive_offset]
	                               : pbx_wbyte;

	// HPS bridge outputs (status + current command-stream byte).
	assign cd_hps_cmd_pending = cmd_pending;
	assign cd_hps_cmd_byte    = cdrom_command_buffer[hps_cmd_rd_ptr[4:0]];

	// HPS sector-channel outputs.
	assign cd_hps_sec_req     = sec_req_w;
	assign cd_hps_sec_status  = cdrom_sector_counter;

	// Phase 18: rx_busy out — receive engine has a queued or in-flight response.
	assign cd_hps_rx_busy     = (cdrom_receive_length != 6'd0);

	// Phase 13: I2C slave EEPROM (1 KiB, 24LC08-equivalent). Pulls SDA
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

// Phase 32: NVRAM save-dump port out to fastchip / bridge.
assign hps_nvr_dout    = cd_hps_nvr_dout;
assign hps_nvr_dirty   = cd_hps_nvr_dirty;

endmodule
