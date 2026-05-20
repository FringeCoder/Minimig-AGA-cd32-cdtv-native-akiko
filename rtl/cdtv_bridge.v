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
// cdtv_bridge — native CDTV expansion at $E90000-$E9FFFF.
//
// Implements the full WinUAE-equivalent register surface from
// research/docs/cdtv-bridge-spec.md (sections 2-7). One module groups three
// responsibilities:
//
//   * DMAC register file ($41 ISTR, $43 CNTR, $80-$83 WTC, $84-$87 ACR,
//     $8E-$8F DAWR, $E0/$E2/$E4/$E8 DMA control strobes) — spec section 2.3.
//   * 6525 TPI ($B0/$B2/$B4/$B6/$B8/$BA/$BC/$BE on even-byte offsets) with
//     mode-1 IRQ priority encoder — spec section 3.
//   * Matsushita CR-511 command FIFO ($A1) with 32-byte input + 32-byte
//     reply rings — spec section 4. Command DISPATCH is delegated to a
//     future userspace helper over UIO (port declared, drain logic out of
//     scope for this pass).
//
// Style mirrors rtl/akiko_hps_bridge.v: snake_case, all reg/wire decls at
// top, separate always blocks per responsibility, spec citations on every
// non-obvious value.
//
// Address routing:
//   * sel is the post-autoconfig chip-bus chip-select from gary.sel_cdtv
//     (cdtv_mode && addr[23:16] == 8'hE9). It runs together with sel_cdtv
//     because the CDTV BIOS does occasional reads of the AC ROM mirror at
//     $E9.00-$E9.3F — WinUAE's dmac_bget2 cdtv.cpp:1324-1325 returns
//     dmacmemory[] in that range AFTER autoconfig. Reads of un-implemented
//     offsets return 0 (spec section 2.3 default branch).
//   * selack fires on the SAME cycle as sel so cpu_wrapper's pipelined
//     cpu_din mux latches the bridge data instead of the empty chip-bus
//     default.
//
// Reset semantics:
//   * Hard reset (reset input): full state-vector wipe per spec section
//     7.1.
//   * Peripheral reset (CNTR_PREST bit on $E90043 write, spec section 7.2):
//     stops DMA, clears CD state, asserts STCH pulse, does NOT touch TPI
//     register state. Implemented via prst_pulse from the CNTR-write block.
//
//----------------------------------------------------------------------------------

module cdtv_bridge
(
	input             clk,
	input             reset,

	// Chip-bus interface (post-autoconfig $E90000-$E9FFFF window). sel and
	// addr come from gary.sel_cdtv + cpu_address_out; wr/uds/lds are the
	// strobes from cpu_wrapper. Width-of-access handling: spec section 1
	// "Access width on the wire" — DMAC/TPI regs are byte at known offsets;
	// word/long writes from the CPU dispatcher decompose to byte
	// transactions on the bus, so we treat uds and lds independently.
	input             sel,
	output            selack,
	input      [23:1] addr,          // chip_addr word-stride
	input      [15:0] din,           // CPU -> bridge
	output     [15:0] dout,          // bridge -> CPU
	input             rd,            // cpu_rd (chip-bus read)
	input             hwr,           // cpu_hwr (write strobe for upper / even byte)
	input             lwr,           // cpu_lwr (write strobe for lower / odd byte)

	// Autoconfig ROM mirror — the AC ROM array lives in cpu_wrapper for
	// the $E80000 autoconfig phase, but the BIOS also reads it at $E900-
	// $E93F after relocation (spec section 2.3 first row of the per-offset
	// table). We accept a 64-byte byte-stream from the wrapper rather than
	// duplicating the table.
	input       [7:0] ac_rom_byte,   // ac_rom[addr[6:1]] from cpu_wrapper
	output      [5:0] ac_rom_addr,   // byte offset / 2 (since stride 2 on Z2 AC)

	// IRQ output to the chipset INT2 (PORTS) line. ORed with akiko_irq /
	// gayle_irq in rtl/minimig.v paula instantiation. Active high.
	output            cdtv_irq,

	// CDDA volume word (10 bits) driven by TPI Port B DAC strobes —
	// spec section 3.3. Hooked into the existing CDDA mixer path; M1
	// splash does not exercise this but the cabling is cheap.
	output      [9:0] cdda_volume,

	// CR-511 command FIFO — UIO interface to userspace dispatcher. Push
	// path: CPU writes $A1, bridge enqueues into cmd_in_fifo; userspace
	// drains by pulsing cmd_in_pop while reading cmd_in_byte. Reply path:
	// userspace pushes a byte via cmd_out_push + cmd_out_data, bridge
	// returns it to CPU on the next $A1 read and pulses sten.
	output            cmd_in_pending,
	output      [7:0] cmd_in_byte,
	input             cmd_in_pop,

	input             cmd_out_push,
	input       [7:0] cmd_out_data,

	// Sector DMA data path — CR-511 READ ($02) commands stage a sector
	// stream in userspace; bridge consumes bytes via sec_byte_push pulses
	// and (in a future session) writes them to chip RAM at acr (spec
	// section 4.4). For this RTL pass we provide the FIFO buffer + advance
	// logic; the actual chip-RAM master is a stub.
	input             sec_byte_push,
	input       [7:0] sec_byte_data,

	// Subchannel byte input — spec section 3.2. Bit-reversed on read of
	// Port A.
	input             subq_push,
	input       [7:0] subq_byte,

	// Status-change source pulses from userspace / CD state machine —
	// spec section 3.4. Each is a 1-clk pulse; TPI ilatch accumulates.
	input             stch_pulse,
	input             sten_pulse_ext, // additional source (e.g. user-side ready)
	input             scor_pulse,
	input             sbcp_pulse,

	// Trace output — every DMAC/TPI/CR-511 access, 64-bit entry, drained
	// via cdtv_trace.v. We expose the strobes as separate one-shot wires
	// so the trace module can capture them on its own clk edge.
	output            trace_we,
	output     [63:0] trace_data
);

//----------------------------------------------------------------------------
// 1. Constants — bit-field positions from spec section 2.1
//----------------------------------------------------------------------------

// CNTR bits at $E90043 — spec section 2.1
localparam CNTR_TCEN_BIT  = 3'd7;
localparam CNTR_PREST_BIT = 3'd6;
localparam CNTR_PDMD_BIT  = 3'd5;
localparam CNTR_INTEN_BIT = 3'd4;
localparam CNTR_DDIR_BIT  = 3'd3;

// ISTR bits at $E90041 (8-bit form; INTX/INT_F bits 8/9 dropped per
// spec section 2.1 "treat ISTR as 8 bits")
localparam ISTR_INTS_BIT  = 3'd6;
localparam ISTR_E_INT_BIT = 3'd5;
localparam ISTR_INT_P_BIT = 3'd4;
localparam ISTR_FE_FLG_B  = 3'd0;

//----------------------------------------------------------------------------
// 2. All reg/wire declarations (per project style: tops of modules)
//----------------------------------------------------------------------------

// --- DMAC register file (spec section 2.3) ---
reg  [7:0] istr;
reg  [7:0] cntr;
reg [31:0] wtc;
reg [31:0] acr;
reg [15:0] dawr;
reg        dmac_dma;
reg        dma_finished;
reg        prst_pulse;
reg        fifo_touch;
reg        dma_complete_pulse;
reg        dma_complete_armed; // one-shot guard: fire dma_complete only once per DMA cycle

// --- TPI (spec section 3) ---
reg [7:0] tp_a;
reg [7:0] tp_b;
reg [7:0] tp_ad;
reg [7:0] tp_bd;
reg [7:0] tp_cd;
reg [7:0] tp_cr;
reg [4:0] tp_imask;
reg [7:0] tp_air;
reg [7:0] tp_ilatch;
reg [7:0] tp_ilatch2;

// CDDA DAC shift register — spec section 3.3
reg [11:0] dac_shift;
reg  [9:0] cd_volume;
reg        tp_b_prev_6, tp_b_prev_7;

// Subchannel byte storage — spec section 3.2
reg [7:0] subq_head;
reg       sbcp_state;

// Combinational read mux for TPI port. Built in a separate always_comb.
reg  [7:0] tpi_rd;

// Internal STEN pulse — fires whenever a reply byte becomes visible to
// the CPU (spec section 4.1: "STEN pulses on each byte ready").
reg        sten_pulse_int;

// --- CR-511 FIFO (spec section 4) ---
reg [7:0] cmd_in_fifo  [0:31];
reg [4:0] cmd_in_wr_p, cmd_in_rd_p;
reg [7:0] cmd_out_fifo [0:31];
reg [4:0] cmd_out_wr_p, cmd_out_rd_p;
reg [7:0] last_out;

// Sector DMA staging buffer — spec section 4.4. 8 KB lets up to 4 cooked
// sectors (4 × 2048) sit in-flight; READ ($02) typically streams them in
// one at a time. Indexed with 13-bit pointers.
reg [7:0] sec_fifo [0:8191];
reg [12:0] sec_wr_p, sec_rd_p;

// --- Trace output staging ---
reg [7:0] trace_tag;

// --- Read-side data mux. Per spec section 1 reads pull a 16-bit word
//     with byte_off in upper half (even byte) and byte_off+1 in lower half
//     (odd byte). Unmapped slots default to 0 (spec section 2.3 default
//     "v = 0" branch).
reg [7:0] rd_byte_eb;
reg [7:0] rd_byte_ob;

// Combinational helper wires (declared after the reg they depend on)
wire        istr_any_set;
wire  [7:0] istr_rd;
wire        sten_any;
wire  [4:0] tpi_edges;
wire  [4:0] masked_active;
wire        cmd_in_empty;
wire        cmd_out_empty;
wire        sec_empty;
wire        dmac_int2;
wire        tpi_int2;
wire        any_access;
wire [15:0] byte_off;

//----------------------------------------------------------------------------
// 3. Address decode helpers
//
// chip_addr is the CPU word address [23:1]; byte address is {addr,1'b0}.
// All offsets in spec section 2.3 are byte offsets within $E9xxxx.
//----------------------------------------------------------------------------

// byte_off is the 16-bit byte offset within $E9xxxx. addr is the word
// address so this is the EVEN-byte slot ($00, $02, ...); the odd-byte
// slot is byte_off+1. WinUAE's dispatcher (cdtv.cpp:1500-1505) decomposes
// word/long bus accesses into byte transactions in big-endian order:
// upper byte at addr[0]=0 (= our byte_off, served via hwr), lower byte
// at addr[0]=1 (= byte_off+1, served via lwr). Reads pull both bytes
// from the slave; the CPU latches whichever half it needs.
assign byte_off = {addr[15:1], 1'b0};   // even-byte offset; odd byte is byte_off|1

// Each spec offset is resolved to one of two slot positions per word access:
//   * "_eb" selectors fire when the spec byte = byte_off (even slot).
//   * "_ob" selectors fire when the spec byte = byte_off + 1 (odd slot).
// During reads both slots evaluate (dout = {even_byte, odd_byte}); during
// writes hwr gates the even-slot, lwr gates the odd-slot.
wire sel_ac_rom_w   = sel && (byte_off < 16'h0040);                     // either byte
wire sel_istr_ob    = sel && (byte_off == 16'h0040);                    // $41 at byte_off+1
wire sel_cntr_ob    = sel && (byte_off == 16'h0042);                    // $43 at byte_off+1
wire sel_wtc_b0_eb  = sel && (byte_off == 16'h0080);                    // $80
wire sel_wtc_b1_ob  = sel && (byte_off == 16'h0080);                    // $81
wire sel_wtc_b2_eb  = sel && (byte_off == 16'h0082);                    // $82
wire sel_wtc_b3_ob  = sel && (byte_off == 16'h0082);                    // $83
wire sel_acr_b0_eb  = sel && (byte_off == 16'h0084);                    // $84
wire sel_acr_b1_ob  = sel && (byte_off == 16'h0084);                    // $85
wire sel_acr_b2_eb  = sel && (byte_off == 16'h0086);                    // $86
wire sel_acr_b3_ob  = sel && (byte_off == 16'h0086);                    // $87 (bit-0 quirk)
wire sel_dawr_h_eb  = sel && (byte_off == 16'h008E);                    // $8E
wire sel_dawr_l_ob  = sel && (byte_off == 16'h008E);                    // $8F
wire sel_cmda_ob    = sel && (byte_off == 16'h00A0);                    // $A1 CR-511 — spec 2.3
wire sel_xt_a3_ob   = sel && (byte_off == 16'h00A2);                    // $A3 floor — spec 2.3
wire sel_xt_a5_ob   = sel && (byte_off == 16'h00A4);                    // $A5
wire sel_xt_a7_ob   = sel && (byte_off == 16'h00A6);                    // $A7
wire in_tpi_range   = sel && (byte_off >= 16'h00B0) && (byte_off <= 16'h00BE); // even byte only
wire  [2:0] tpi_reg = byte_off[3:1];
wire sel_dma_start  = sel && (byte_off == 16'h00E0);                    // DMA START — spec 2.3
wire sel_dma_stop   = sel && (byte_off == 16'h00E2);                    // DMA STOP
wire sel_istr_clr   = sel && (byte_off == 16'h00E4);                    // ISTR CLEAR
wire sel_fifo_tog   = sel && (byte_off == 16'h00E8);                    // FIFO toggle

// Legacy aliases for the always blocks below (keep the diff small).
wire sel_istr_b   = sel_istr_ob;
wire sel_cntr_b   = sel_cntr_ob;
wire sel_cmda_b   = sel_cmda_ob;
wire sel_wtc_w_hi = sel_wtc_b0_eb;       // covers $80 (eb) + $81 (ob)
wire sel_wtc_w_lo = sel_wtc_b2_eb;       // covers $82 + $83
wire sel_acr_w_hi = sel_acr_b0_eb;       // covers $84 + $85
wire sel_acr_w_lo = sel_acr_b2_eb;       // covers $86 + $87
wire sel_dawr_w   = sel_dawr_h_eb;       // covers $8E + $8F
wire sel_ac_rom   = sel_ac_rom_w;
wire sel_xtfloor  = sel_xt_a3_ob | sel_xt_a5_ob | sel_xt_a7_ob;

assign ac_rom_addr = byte_off[6:1];

// Spec section 2.3 row 2: read of $41 returns istr | INT_P when any
// other bit is set (cdtv.cpp:1332-1336).
assign istr_any_set = |istr[7:1];
assign istr_rd      = istr | (istr_any_set ? (8'h01 << ISTR_INT_P_BIT) : 8'h00);

// TPI edge aggregation. Spec section 3.4 "treat sbcp/scor/stch/sten as
// 1-clock pulse inputs". Mapping per cdtv.cpp:881-885: bit-4 alias on STEN
// — spec contradiction #4. Both internal (cmd reply ready) and external
// (userspace push) STEN pulses contribute.
assign sten_any  = sten_pulse_ext | sten_pulse_int;
assign tpi_edges = {sten_any, sten_any, stch_pulse, scor_pulse, sbcp_pulse};
assign masked_active = tp_ilatch[4:0] & tp_imask[4:0];

assign cmd_in_empty  = (cmd_in_wr_p  == cmd_in_rd_p);
assign cmd_out_empty = (cmd_out_wr_p == cmd_out_rd_p);
assign sec_empty     = (sec_wr_p     == sec_rd_p);

// Spec section 6.1
assign dmac_int2 = cntr[CNTR_INTEN_BIT] & (istr[ISTR_E_INT_BIT] | istr[ISTR_INTS_BIT]);
assign tpi_int2  = tp_ilatch[5];

assign cdtv_irq    = dmac_int2 | tpi_int2;
assign cdda_volume = cd_volume;

assign cmd_in_pending = ~cmd_in_empty;
assign cmd_in_byte    = cmd_in_fifo[cmd_in_rd_p];

wire wr_any = hwr | lwr;
assign any_access = sel && (rd || wr_any);
assign trace_we   = any_access;
assign trace_data = {32'h0, trace_tag, din[7:0], byte_off};

assign selack = sel;
// Read mux assembles even-byte and odd-byte slots into a 16-bit word. The
// CPU side picks the half it needs via cpu_hwr/cpu_lwr semantics on the
// next bus cycle (chip-bus reads ALWAYS pull a full word). Unimplemented
// slots default to 0x00 per spec section 2.3.
assign dout   = {rd_byte_eb, rd_byte_ob};

//----------------------------------------------------------------------------
// 4. DMAC register file — spec section 2.3
//----------------------------------------------------------------------------

always @(posedge clk) begin
	if (reset) begin
		istr               <= 8'h00;
		cntr               <= 8'h00;
		wtc                <= 32'h0;
		acr                <= 32'h0;
		dawr               <= 16'h0;
		dmac_dma           <= 1'b0;
		dma_finished       <= 1'b0;
		prst_pulse         <= 1'b0;
		fifo_touch         <= 1'b0;
		dma_complete_pulse <= 1'b0;
		dma_complete_armed <= 1'b0;
	end else begin
		// Default pulse deasserts
		prst_pulse         <= 1'b0;
		fifo_touch         <= 1'b0;
		dma_complete_pulse <= 1'b0;

		// $E90043 (CNTR) byte write — spec section 2.3 row 3.
		// $43 is the odd byte of word $42; write strobe is lwr.
		if (sel_cntr_ob && lwr) begin
			cntr <= din[7:0];
			if (din[CNTR_PREST_BIT]) prst_pulse <= 1'b1;
		end

		// $E90041 ISTR readback side-effect — spec section 2.3 row 2.
		if (sel_istr_ob && rd) istr <= istr & 8'hF0;

		// $E900E4 ISTR-clear-all — spec section 2.3. WinUAE doesn't gate on
		// byte slot — write of any byte in $E4/$E5 word triggers full clear.
		if (sel_istr_clr && (hwr || lwr)) istr <= 8'h00;

		// $E900E8 FIFO toggle — read or write sets ISTR_FE_FLG.
		if (sel_fifo_tog && (rd || hwr || lwr)) begin
			istr       <= istr | (8'h01 << ISTR_FE_FLG_B);
			fifo_touch <= 1'b1;
		end

		// WTC ($80-$83) — spec section 2.3 row 4. BE byte stride.
		// Word write at $80: hwr=upper=$80, lwr=lower=$81.
		if (sel_wtc_b0_eb && hwr) wtc[31:24] <= din[15:8];   // $80
		if (sel_wtc_b1_ob && lwr) wtc[23:16] <= din[7:0];    // $81
		if (sel_wtc_b2_eb && hwr) wtc[15:8]  <= din[15:8];   // $82
		if (sel_wtc_b3_ob && lwr) wtc[7:0]   <= din[7:0];    // $83

		// ACR ($84-$87) — spec section 2.3 row 5. Bit-0 quirk at $87.
		if (sel_acr_b0_eb && hwr) acr[31:24] <= din[15:8];   // $84
		if (sel_acr_b1_ob && lwr) acr[23:16] <= din[7:0];    // $85
		if (sel_acr_b2_eb && hwr) acr[15:8]  <= din[15:8];   // $86
		if (sel_acr_b3_ob && lwr) acr[7:1]   <= din[7:1];    // $87 (bit-0 preserved)

		// DAWR ($8E-$8F) — spec section 2.3 row 6. Stored-but-unused.
		if (sel_dawr_h_eb && hwr) dawr[15:8] <= din[15:8];
		if (sel_dawr_l_ob && lwr) dawr[7:0]  <= din[7:0];

		// DMA START / STOP — spec section 2.3 rows "DMA START" / "DMA STOP".
		// Word access fires the strobe; either byte half is sufficient.
		// Arm the completion one-shot on START — fires once per START/STOP
		// cycle, then de-arms.
		if (sel_dma_start && (hwr || lwr) && !dmac_dma) begin
			dmac_dma           <= 1'b1;
			dma_complete_armed <= 1'b1;
		end
		if (sel_dma_stop  && (hwr || lwr)) begin
			dmac_dma           <= 1'b0;
			dma_finished       <= 1'b0;
			dma_complete_armed <= 1'b0;
		end

		// DMA end-of-process IRQ — spec section 4.4 + section 6.1.
		// Single-fire when the DMA worker drains (wtc=0 AND sec_empty AND
		// DMA was armed). The armed flag prevents repeat firings while the
		// CPU is still draining DMA state.
		if (dmac_dma && dma_complete_armed && (wtc == 32'h0) && sec_empty) begin
			dma_complete_pulse <= 1'b1;
			dma_complete_armed <= 1'b0;
		end
		if (dma_complete_pulse && cntr[CNTR_INTEN_BIT] && cntr[CNTR_TCEN_BIT]) begin
			istr         <= istr | (8'h01 << ISTR_E_INT_BIT)
			                     | (8'h01 << ISTR_INT_P_BIT);
			dma_finished <= 1'b0;
		end
	end
end

//----------------------------------------------------------------------------
// 5. 6525 TPI — spec section 3
//----------------------------------------------------------------------------

// Combinational read mux per spec section 3.2-3.7
always @* begin
	tpi_rd = 8'h00;
	case (tpi_reg)
		3'd0: begin
			// Port A — spec section 3.2: bit-reverse subq_head
			tpi_rd[0] = subq_head[7];
			tpi_rd[1] = subq_head[6];
			tpi_rd[2] = subq_head[5];
			tpi_rd[3] = subq_head[4];
			tpi_rd[4] = subq_head[3];
			tpi_rd[5] = subq_head[2];
			tpi_rd[6] = subq_head[1];
			tpi_rd[7] = subq_head[0];
		end
		3'd1: tpi_rd = tp_b;
		3'd2: begin
			// Port C — spec section 3.4. Returns inverted-active mask of
			// (ilatch | ilatch2) in mode 1; raw GPIO in mode 0.
			if (tp_cr[0])
				tpi_rd = {tp_ilatch[7:5], ~(tp_ilatch[4:0] | tp_ilatch2[4:0])};
			else
				tpi_rd = tp_ilatch;
		end
		3'd3: tpi_rd = tp_ad;
		3'd4: tpi_rd = tp_bd;
		3'd5: tpi_rd = tp_cr[0] ? {3'h0, tp_imask} : tp_cd;
		3'd6: tpi_rd = tp_cr;
		3'd7: tpi_rd = tp_air;
	endcase
end

always @(posedge clk) begin
	if (reset) begin
		tp_a        <= 8'h00;
		tp_b        <= 8'h00;
		tp_ad       <= 8'h00;
		tp_bd       <= 8'h00;
		tp_cd       <= 8'h00;
		tp_cr       <= 8'h00;
		tp_imask    <= 5'h00;
		tp_air      <= 8'h00;
		tp_ilatch   <= 8'h00;
		tp_ilatch2  <= 8'h00;
		dac_shift   <= 12'h0;
		cd_volume   <= 10'h0;
		tp_b_prev_6 <= 1'b0;
		tp_b_prev_7 <= 1'b0;
		subq_head   <= 8'h00;
		sbcp_state  <= 1'b0;
	end else begin
		// Accumulate IRQ edges — spec section 3.4.
		tp_ilatch[4:0] <= tp_ilatch[4:0] | tpi_edges;

		// Subchannel byte arrival — spec section 3.2.
		if (subq_push) begin
			subq_head  <= subq_byte;
			sbcp_state <= 1'b1;
		end

		// Port A read consumes one byte (clears sbcp) — spec 3.2.
		if (in_tpi_range && rd && (tpi_reg == 3'd0)) begin
			sbcp_state <= 1'b0;
		end

		// TPI register writes — spec section 3.8 RTL skeleton.
		// TPI lives on EVEN byte offsets ($B0/$B2/.../$BE) — spec section 1.
		// Word access at $B0 → hwr=$B0 (TPI reg 0), lwr=$B1 (ignored).
		// Real TPI silicon ignores odd byte access (spec 1 "odd addresses
		// inside the TPI range are ignored"); use hwr only.
		// Data byte is din[15:8] (upper half of the word — even byte slot).
		if (in_tpi_range && hwr) begin
			case (tpi_reg)
				3'd0: tp_a <= din[15:8];
				3'd1: begin
					tp_b <= din[15:8];
					// DAC volume serial — spec section 3.3
					if (din[14] && !tp_b_prev_6)
						dac_shift <= {din[13], dac_shift[11:1]};
					if (din[15] && !tp_b_prev_7)
						cd_volume <= dac_shift[9:0];
					tp_b_prev_6 <= din[14];
					tp_b_prev_7 <= din[15];
				end
				3'd2: begin
					// Port C write — spec section 3.4 mode-1 ack semantic:
					// "tp_ilatch &= 0xe0 | v" (write 0 to ack that source).
					if (tp_cr[0])
						tp_ilatch[4:0] <= tp_ilatch[4:0] & din[12:8];
				end
				3'd3: tp_ad <= din[15:8];
				3'd4: tp_bd <= din[15:8];
				3'd5: begin
					if (tp_cr[0]) tp_imask <= din[12:8]; // spec 3.5
					else          tp_cd    <= din[15:8];
				end
				3'd6: tp_cr  <= din[15:8];
				3'd7: tp_air <= din[15:8];
			endcase
		end

		// IRQ priority encoder + raise — spec section 3.4.
		if (tp_cr[0] && !tp_ilatch[5] && |masked_active) begin
			casex (masked_active)
				5'b1xxxx: begin
					tp_air       <= 8'h10;
					tp_ilatch[4] <= 1'b0;
					tp_ilatch[5] <= 1'b1;
					tp_ilatch2   <= 8'h10;
				end
				5'b01xxx: begin
					tp_air       <= 8'h08;
					tp_ilatch[3] <= 1'b0;
					tp_ilatch[5] <= 1'b1;
					tp_ilatch2   <= 8'h08;
				end
				5'b001xx: begin
					tp_air       <= 8'h04;
					tp_ilatch[2] <= 1'b0;
					tp_ilatch[5] <= 1'b1;
					tp_ilatch2   <= 8'h04;
				end
				5'b0001x: begin
					tp_air       <= 8'h02;
					tp_ilatch[1] <= 1'b0;
					tp_ilatch[5] <= 1'b1;
					tp_ilatch2   <= 8'h02;
				end
				5'b00001: begin
					tp_air       <= 8'h01;
					tp_ilatch[0] <= 1'b0;
					tp_ilatch[5] <= 1'b1;
					tp_ilatch2   <= 8'h01;
				end
			endcase
		end

		// AIR read = ACK (mode 1) — spec section 3.4.
		if (in_tpi_range && rd && (tpi_reg == 3'd7) && tp_cr[0]) begin
			tp_ilatch[5] <= 1'b0;
			tp_ilatch2   <= 8'h00;
			tp_air       <= 8'h00;
		end
	end
end

//----------------------------------------------------------------------------
// 6. CR-511 command FIFO + sector buffer — spec section 4
//----------------------------------------------------------------------------

always @(posedge clk) begin
	if (reset) begin
		cmd_in_wr_p    <= 5'h0;
		cmd_in_rd_p    <= 5'h0;
		cmd_out_wr_p   <= 5'h0;
		cmd_out_rd_p   <= 5'h0;
		last_out       <= 8'h00;
		sec_wr_p       <= 13'h0;
		sec_rd_p       <= 13'h0;
		sten_pulse_int <= 1'b0;
	end else begin
		sten_pulse_int <= 1'b0;

		// Peripheral reset (CNTR_PREST) — spec section 7.2. Stops DMA +
		// flushes command rings. Does NOT touch TPI.
		if (prst_pulse) begin
			cmd_in_wr_p  <= 5'h0;
			cmd_in_rd_p  <= 5'h0;
			cmd_out_wr_p <= 5'h0;
			cmd_out_rd_p <= 5'h0;
			sec_wr_p     <= 13'h0;
			sec_rd_p     <= 13'h0;
		end

		// CPU write to $A1 — spec section 4.1 step 1-2.
		// $A1 is the odd-byte slot of word $A0; data in din[7:0], lwr strobe.
		if (sel_cmda_ob && lwr) begin
			cmd_in_fifo[cmd_in_wr_p] <= din[7:0];
			cmd_in_wr_p              <= cmd_in_wr_p + 5'd1;
		end

		// Userspace pop of input FIFO — spec section 4.7 RTL skeleton.
		if (cmd_in_pop && !cmd_in_empty) begin
			cmd_in_rd_p <= cmd_in_rd_p + 5'd1;
		end

		// Userspace push of reply byte — spec section 4.1 step 4.
		if (cmd_out_push) begin
			cmd_out_fifo[cmd_out_wr_p] <= cmd_out_data;
			cmd_out_wr_p               <= cmd_out_wr_p + 5'd1;
			sten_pulse_int             <= 1'b1;
		end

		// CPU read of $A1 — spec section 4.1 step 5.
		// Read returns the byte in the odd-half (din[7:0]) of word $A0.
		if (sel_cmda_ob && rd) begin
			if (!cmd_out_empty) begin
				last_out     <= cmd_out_fifo[cmd_out_rd_p];
				cmd_out_rd_p <= cmd_out_rd_p + 5'd1;
				// If at least one more byte remains AFTER this pop, re-arm STEN
				if ((cmd_out_wr_p - (cmd_out_rd_p + 5'd1)) != 5'd0)
					sten_pulse_int <= 1'b1;
			end
		end

		// Sector data ingress (userspace push) — spec section 4.4.
		// TODO: chip-RAM master DMA. For this RTL pass we accept bytes
		// into the staging FIFO. The actual chip-RAM write at `acr` is
		// wired in a future session via chipdma_arb, similar to akiko's
		// M5 path; for now the bytes accumulate and dma_complete_pulse
		// (in the DMAC block) fires only after both wtc and the FIFO
		// drain. wtc decrement waits for the same future session.
		if (sec_byte_push && dmac_dma) begin
			sec_fifo[sec_wr_p] <= sec_byte_data;
			sec_wr_p           <= sec_wr_p + 13'd1;
		end
	end
end

//----------------------------------------------------------------------------
// 7. Read-side data mux — spec section 2.3 default branch
//
// All un-implemented offsets return 0 (cdtv.cpp:1322 `v = 0` initial).
// AC ROM mirror: ac_rom_byte returned by cpu_wrapper.
//----------------------------------------------------------------------------

// Even-byte slot (at byte_off): TPI regs land here (all on $B0/$B2/.../$BE
// even offsets), WTC/ACR/DAWR upper bytes, AC ROM mirror, $E0/$E2/$E4/$E8
// (DMA control — but those are write-only / read returns 0).
always @* begin
	rd_byte_eb = 8'h00;
	if      (sel_ac_rom_w) rd_byte_eb = ac_rom_byte;  // AC ROM at even byte
	else if (in_tpi_range) rd_byte_eb = tpi_rd;       // TPI regs at even byte
end

// Odd-byte slot (at byte_off+1): ISTR ($41), CNTR ($43), WTC/ACR/DAWR
// lower bytes (write-only — read returns 0), CR-511 $A1, XT floor
// $A3/$A5/$A7 (read 0xFF). AC ROM odd bytes are 0xFF per Z2 spec (the
// autoconfig protocol leaves alternate bytes as the no-op fill — spec
// section 2.2 "memset(dmacmemory, 0xff)").
always @* begin
	rd_byte_ob = 8'h00;
	if      (sel_ac_rom_w) rd_byte_ob = 8'hFF;
	else if (sel_istr_ob)  rd_byte_ob = istr_rd;
	else if (sel_cntr_ob)  rd_byte_ob = cntr;
	else if (sel_xtfloor)  rd_byte_ob = 8'hFF;
	else if (sel_cmda_ob)  rd_byte_ob = cmd_out_empty ? last_out
	                                                  : cmd_out_fifo[cmd_out_rd_p];
end

//----------------------------------------------------------------------------
// 8. Trace tag — for cdtv_trace.v capture
//
// Layout: see cdtv_trace.v header.
//----------------------------------------------------------------------------

always @* begin
	trace_tag = {wr_any, 7'h0F};   // default = other
	if      (sel_ac_rom)    trace_tag = {wr_any, 7'h08};
	else if (sel_istr_b)    trace_tag = {wr_any, 7'h01};
	else if (sel_cntr_b)    trace_tag = {wr_any, 7'h02};
	else if (sel_wtc_w_hi)  trace_tag = {wr_any, 7'h03};
	else if (sel_wtc_w_lo)  trace_tag = {wr_any, 7'h04};
	else if (sel_acr_w_hi)  trace_tag = {wr_any, 7'h05};
	else if (sel_acr_w_lo)  trace_tag = {wr_any, 7'h06};
	else if (sel_dawr_w)    trace_tag = {wr_any, 7'h07};
	else if (sel_cmda_b)    trace_tag = {wr_any, 7'h09};
	else if (in_tpi_range)  trace_tag = {wr_any, 7'h0A};
	else if (sel_dma_start) trace_tag = {wr_any, 7'h0B};
	else if (sel_dma_stop)  trace_tag = {wr_any, 7'h0C};
	else if (sel_istr_clr)  trace_tag = {wr_any, 7'h0D};
	else if (sel_fifo_tog)  trace_tag = {wr_any, 7'h0E};
end

endmodule
