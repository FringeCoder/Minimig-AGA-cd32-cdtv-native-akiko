// SPDX-License-Identifier: GPL-3.0-or-later
//
// chipdma_arb -- M5 chip-RAM master arbiter (v2: c_7m-aligned drive).
//
// Sits between minimig.v's chipset DMA signals (Agnus / blitter / copper)
// and sdram_ctrl's chipDMA port. Default forwards minimig's signals
// through unchanged. When the chipset is not using a slot, the arbiter
// claims it for akiko's single-byte master, runs one access, samples
// chipRD when the read returns, and pulses akiko_dma_ack with the byte.
//
// Clocking: this module runs on clk_sys (the 28.6 MHz Minimig system
// clock, same domain as akiko). sdram_ctrl runs on clk_114 (4× faster).
// c_7m (= clk_sys / 4) is the chipset slot clock and gates the slot
// boundary in both modules.
//
// The c_7m boundary race (and why chip_out_* MUST be combinational):
//
//   In hardware sdram_ctrl samples chipDMA at clk_114 cycle K+1, where
//   K is the clk_114 edge that detected c_7m rising (~8.7 ns after the
//   c_7m_rise instant). Minimig's existing chipDMA inputs come from
//   agnus's REGISTERED outputs through gary's COMBINATIONAL gating, so
//   they are valid at sdram_ctrl's sample point with comfortable margin.
//
//   If we drive chip_out_dma from a register that flips on c_7m_rise,
//   the FF clock-to-Q on the same clk_sys edge happens at the SAME
//   instant sdram_ctrl is sampling -- too late by ~8 ns. sdram_ctrl
//   sees chipDMA still HIGH and skips the slot. Akiko's read fails.
//
//   The fix is to drive chip_out_* from purely combinational logic
//   that responds to c_7m_rise within the same clk_sys edge, mimicking
//   how agnus's combinational outputs reach sdram_ctrl with no extra
//   register stage.
//
// Priority: combinational gating gives minimig priority on every cycle.
// We never override chip_in_dma when minimig has it asserted -- our
// arb_drive is masked by ~minimig_busy. This prevents a stale view of
// chip_in_dma from corrupting a slot minimig is about to use (the v8
// hardware regression that caused screen flicker).
//
// Akiko addressing: dma_baddr is a 24-bit BYTE address. Word index is
// dma_baddr[23:1]; the byte selector is dma_baddr[0]. Amiga is
// big-endian: dma_baddr[0]=0 picks the upper byte (chipU=0, chipL=1);
// dma_baddr[0]=1 picks the lower byte (chipU=1, chipL=0).

module chipdma_arb
(
	input             clk,        // clk_sys
	input             reset,
	input             c_7m,       // chipset slot clock; rising edge marks slot start

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
// At slot_cnt==3 sdram_ctrl's chipRD register has been valid for >3
// clk_114 cycles (chipRD updates at sdram_state==9 ≈ clk_114 cycle K+10).
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

// --- Minimig idleness on this slot. Combinational from chip_in_dma /
//     chip_in_rw (driven through gary from agnus's registered DMA
//     scheduler). At sdram_ctrl's sample point this value reflects
//     the current slot.
wire minimig_idle = chip_in_dma & chip_in_rw;
wire minimig_busy = ~minimig_idle;

// --- arm_now: combinational claim at the c_7m_rise edge. Drives
//     chip_out_dma=0 within the same clk_sys edge, in time for
//     sdram_ctrl to see it ~8.7 ns later (the existing minimig path
//     fits the same budget the same way). Gated by minimig_idle so we
//     never preempt the chipset.
wire arm_now = (state == S_IDLE) & c_7m_rise & minimig_idle & akiko_dma_req;

// --- arb_request: we want to be on the bus this cycle. Either we
//     just armed combinationally, or we are mid-slot (S_DRIVE).
wire arb_request = arm_now | (state == S_DRIVE);

// --- arb_drive: the actual override. Masked by minimig_busy on EVERY
//     cycle so a slot minimig grabs (e.g. across a c_7m boundary into
//     the next slot) wins immediately. arb_request is registered for
//     subsequent cycles, so we keep driving as long as minimig stays
//     idle; if minimig becomes busy we yield without disturbing the
//     in-progress sdram_ctrl access (it snapshotted at slot start).
wire arb_drive = arb_request & minimig_idle;

// --- When arming, the latched ak_* registers are stale (they hold the
//     PREVIOUS request). Use the live akiko_dma_* inputs combinationally
//     for the arming cycle so chip_out_addr/etc. reach sdram_ctrl with
//     the correct address on the very first cycle of the slot. After
//     the arming edge, state==S_DRIVE and arm_now==0, so the registered
//     ak_* values take over -- they were latched by the always block
//     using the same NBA at the arming edge.
wire [24:1] ak_addr_w    = arm_now ? {1'b0, akiko_dma_baddr[23:1]}      : ak_addr;
wire        ak_l_w       = arm_now ? ~akiko_dma_baddr[0]                 : ak_l;
wire        ak_u_w       = arm_now ?  akiko_dma_baddr[0]                 : ak_u;
wire        ak_rw_w      = arm_now ? ~akiko_dma_we                       : ak_rw;
wire [15:0] ak_wr_data_w = arm_now ? {akiko_dma_wbyte, akiko_dma_wbyte}  : ak_wr_data;

assign chip_out_addr = arb_drive ? ak_addr_w    : chip_in_addr;
assign chip_out_l    = arb_drive ? ak_l_w       : chip_in_l;
assign chip_out_u    = arb_drive ? ak_u_w       : chip_in_u;
assign chip_out_rw   = arb_drive ? ak_rw_w      : chip_in_rw;
assign chip_out_dma  = arb_drive ? 1'b0         : chip_in_dma;
assign chip_out_wr   = arb_drive ? ak_wr_data_w : chip_in_wr;

always @(posedge clk) begin
	if (reset) begin
		state      <= S_IDLE;
		slot_cnt   <= 3'd0;
		ak_ack_r   <= 1'b0;
		ak_rbyte_r <= 8'h00;
	end else begin
		ak_ack_r <= 1'b0;  // ack defaults low; pulse in S_ACK

		case (state)
		S_IDLE: begin
			if (arm_now) begin
				ak_addr    <= {1'b0, akiko_dma_baddr[23:1]};
				ak_u       <= akiko_dma_baddr[0];
				ak_l       <= ~akiko_dma_baddr[0];
				ak_rw      <= ~akiko_dma_we;
				ak_wr_data <= {akiko_dma_wbyte, akiko_dma_wbyte};
				ak_we      <= akiko_dma_we;
				ak_baddr0  <= akiko_dma_baddr[0];
				slot_cnt   <= 3'd0;
				state      <= S_DRIVE;
			end
		end

		S_DRIVE: begin
			slot_cnt <= slot_cnt + 3'd1;
			// Sample chipRD at slot_cnt==3 (4 clk_sys cycles after the
			// arming edge = 16 clk_114 cycles, well past sdram_ctrl's
			// state-9 chipRD update).
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
