// SPDX-License-Identifier: GPL-3.0-or-later
//
// chipdma_arb -- M5 chip-RAM master arbiter.
//
// Sits between minimig.v's chipset DMA signals (Agnus / blitter / copper)
// and sdram_ctrl's chipDMA port. Default forwards minimig's signals
// through unchanged. When the chipset is not using a slot, the arbiter
// can claim it for akiko's single-byte master, run one access, sample
// chipRD when the read returns, and pulse akiko_dma_ack with the byte.
//
// Clocking: this module runs on clk_sys (the 28.6MHz Minimig system clock,
// same domain as akiko). sdram_ctrl runs on clk_114 (4x faster) -- minimig's
// existing chipset signals into sdram_ctrl already cross that boundary
// without explicit synchronization, and we follow the same pattern. chipRD
// from sdram_ctrl is sampled at clk_sys late in the slot when it has
// definitely settled.
//
// Slot synchronization: c_7m (= clk_sys / 4) is the chipset slot clock.
// At every c_7m rising edge, we sample whether minimig is idle (chip_in_dma
// AND chip_in_rw both HIGH). If idle AND akiko_dma_req is asserted, we
// claim the slot for akiko -- drive chip_out_* with akiko's request, hold
// for the slot (~4 clk_sys cycles = 16 clk_114 cycles), sample chipRD at
// the end, then pulse dma_ack. If minimig is busy, skip and wait for the
// next c_7m edge.
//
// We never preempt the chipset: minimig's signals are passed through
// during minimig-active slots, and the arbiter only drives during slots
// it confirmed minimig is not using.
//
// Akiko addressing: dma_baddr is a 24-bit BYTE address. Word index is
// dma_baddr[24:1]; the byte selector is dma_baddr[0]. Amiga is
// big-endian: dma_baddr[0]=0 picks the upper byte (chipU=0, chipL=1);
// dma_baddr[0]=1 picks the lower byte (chipU=1, chipL=0).

module chipdma_arb
(
	input             clk,        // sysclk (114MHz on hardware, 100MHz in bench)
	input             reset,
	input             c_7m,       // chipset slot clock, used as slot boundary

	// From minimig (chipset DMA)
	input      [24:1] chip_in_addr,
	input             chip_in_l,
	input             chip_in_u,
	input             chip_in_rw,
	input             chip_in_dma,
	input      [15:0] chip_in_wr,

	// From akiko (single-byte master). req held high until ack pulses.
	input             akiko_dma_req,
	input             akiko_dma_we,
	input      [23:0] akiko_dma_baddr,
	input       [7:0] akiko_dma_wbyte,
	output      [7:0] akiko_dma_rbyte,
	output            akiko_dma_ack,

	// To sdram_ctrl chipDMA port
	output     [24:1] chip_out_addr,
	output            chip_out_l,
	output            chip_out_u,
	output            chip_out_rw,
	output            chip_out_dma,
	output     [15:0] chip_out_wr,
	input      [15:0] chip_in_rd
);

// --- c_7m rising-edge detector (slot boundary) ---
reg c_7m_d;
always @(posedge clk) c_7m_d <= c_7m;
wire c_7m_rise = c_7m & ~c_7m_d;

// --- Slot timer: counts clk_sys cycles within the active akiko slot. ---
// 4 clk_sys cycles per c_7m period (= 16 clk_114 cycles in sdram_ctrl).
// Sample chipRD at the last cycle of the slot, by which time sdram_ctrl
// has progressed well past its state 9 read latch.
reg [2:0] slot_cnt;

// --- Akiko-side latched request fields. ---
reg [24:1] ak_addr;
reg        ak_l;
reg        ak_u;
reg        ak_rw;       // 1 = read, 0 = write (matches sdram_ctrl chipRW)
reg [15:0] ak_wr_data;
reg        ak_we;       // remembers whether this slot is a write (no chipRD sample)
reg        ak_baddr0;   // byte selector for read demux

// --- State ---
localparam [1:0]
	S_IDLE     = 2'd0,
	S_DRIVE    = 2'd1,
	S_ACK      = 2'd2,
	S_COOLDOWN = 2'd3;

reg [1:0] state;

// --- Output regs back to akiko ---
reg [7:0] ak_rbyte_r;
reg       ak_ack_r;

assign akiko_dma_rbyte = ak_rbyte_r;
assign akiko_dma_ack   = ak_ack_r;

// --- Output mux to sdram_ctrl: pass minimig through unless we're driving. ---
wire arb_drive = (state == S_DRIVE);
assign chip_out_addr = arb_drive ? ak_addr    : chip_in_addr;
assign chip_out_l    = arb_drive ? ak_l       : chip_in_l;
assign chip_out_u    = arb_drive ? ak_u       : chip_in_u;
assign chip_out_rw   = arb_drive ? ak_rw      : chip_in_rw;
// chipDMA is active-low. For a read, hold it LOW; for a write,
// chipRW going low is enough -- chipDMA can stay HIGH (sdram_ctrl arms
// on `~chipDMA | ~chipRW`). We hold chipDMA LOW for both to keep the
// slot consistent.
assign chip_out_dma  = arb_drive ? 1'b0       : chip_in_dma;
assign chip_out_wr   = arb_drive ? ak_wr_data : chip_in_wr;

// "minimig idle this slot" -- sampled at c_7m rising edge.
wire minimig_idle = chip_in_dma & chip_in_rw;

always @(posedge clk) begin
	if (reset) begin
		state      <= S_IDLE;
		slot_cnt   <= 5'd0;
		ak_ack_r   <= 1'b0;
		ak_rbyte_r <= 8'h00;
	end else begin
		ak_ack_r <= 1'b0;  // ack defaults low; pulse in S_ACK

		case (state)
		S_IDLE: begin
			// Latch a new akiko request only at a c_7m rising edge AND
			// when minimig is idle for this slot.
			if (akiko_dma_req && c_7m_rise && minimig_idle) begin
				// akiko provides a 24-bit BYTE address; chipAddr is
				// 24-bit WORD address. Word index = baddr[23:1] (23 bits)
				// padded with one zero MSB.
				ak_addr    <= {1'b0, akiko_dma_baddr[23:1]};
				// dma_baddr[0]=0 -> upper byte: chipU=0 (enabled), chipL=1.
				ak_u       <= akiko_dma_baddr[0];   // 0 -> chipU=0
				ak_l       <= ~akiko_dma_baddr[0];  // 0 -> chipL=0 when odd
				ak_rw      <= ~akiko_dma_we;        // chipRW: 1=read, 0=write
				ak_wr_data <= {akiko_dma_wbyte, akiko_dma_wbyte};
				ak_we      <= akiko_dma_we;
				ak_baddr0  <= akiko_dma_baddr[0];
				slot_cnt   <= 3'd0;
				state      <= S_DRIVE;
			end
		end

		S_DRIVE: begin
			slot_cnt <= slot_cnt + 3'd1;
			// At slot_cnt 3 (last clk_sys cycle of the slot), sample
			// chipRD. By now sdram_ctrl has had 3 clk_sys cycles =
			// 12 clk_114 cycles since slot start, well past its state-9
			// read latch.
			if (slot_cnt == 3'd3) begin
				if (!ak_we) begin
					ak_rbyte_r <= ak_baddr0 ? chip_in_rd[7:0]
					                        : chip_in_rd[15:8];
				end
				state <= S_ACK;
			end
		end

		S_ACK: begin
			ak_ack_r <= 1'b1;
			state    <= S_COOLDOWN;
		end

		S_COOLDOWN: begin
			// Guarantee at least one cycle of !dma_ack between two
			// successive akiko services so akiko.v's rx_inflight
			// handshake (lines 522-528) sees the gap.
			state <= S_IDLE;
		end
		endcase
	end
end

endmodule
