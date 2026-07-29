// SPDX-License-Identifier: GPL-3.0-or-later
//
// chipdma_arb -- M5 chip-RAM master arbiter (v2: c_7m-aligned drive).
//
// Sits between minimig.v's chipset DMA signals (Agnus / blitter / copper)
// and sdram_ctrl's chipDMA port. Default forwards minimig's signals
// through unchanged. When the chipset is not using a slot, the arbiter
// claims it for one of two single-byte masters:
//   - akiko (CD32 native-Akiko PBX/DMA path) — same as v2
//   - cdtv  (CDTV bridge sector DMA, M2 phase-1b)
// Static priority: akiko > cdtv. CD32 and CDTV cores never coexist (the
// chipset gate that enables akiko also disables cdtv_mode in cpu_wrapper),
// so the static choice never actually arbitrates — it just keeps the
// fallthrough deterministic.
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
	// One-clk_sys pulse on the arming edge when this
	// slot services akiko. akiko latches which sub-engine it is servicing here
	// so the matching dma_ack is credited to the right engine (closes the
	// cross-engine ack-ownership race; see akiko.v dma_arm).
	output            akiko_arm,

	// From cdtv bridge (single-byte master, M2 phase-1b sector DMA).
	// Same protocol as akiko: req held until ack pulses. CDTV does writes
	// only (sector bytes into chip RAM at `acr`), so rbyte is unused, but
	// the port is symmetrical to keep the diff minimal.
	input             cdtv_dma_req,
	input             cdtv_dma_we,
	input      [23:0] cdtv_dma_baddr,
	input       [7:0] cdtv_dma_wbyte,
	output      [7:0] cdtv_dma_rbyte,
	output            cdtv_dma_ack,

	// To sdram_ctrl chipDMA port (chip RAM / slow RAM / KS — anything
	// living in the on-board SDRAM).
	output     [24:1] chip_out_addr,
	output            chip_out_l,
	output            chip_out_u,
	output            chip_out_rw,
	output            chip_out_dma,
	output     [15:0] chip_out_wr,
	input      [15:0] chip_in_rd,

	// AC-config state + DDR3 (ram2) DMA write port. When the
	// active master's address falls in a Zorro fast-RAM window the slot
	// routes to ddr_out_* instead of chip_out_*. memory_router does the
	// decode using the same logic cpu_wrapper applies for CPU access.
	input             z2ram_ena,
	input       [4:0] z3ram_base0,
	input             z3ram_ena0,
	input       [3:0] z3ram_base1,
	input             z3ram_ena1,

	output     [28:1] ddr_out_addr,
	output            ddr_out_l,
	output            ddr_out_u,
	output            ddr_out_we,
	output            ddr_out_cs,
	output     [15:0] ddr_out_wr,
	input             ddr_in_ack,
	// Read return path. ddr_in_rd is the 16-bit word captured by ddram_ctrl
	// at the same time it raises ddr_in_ack on a read. Data is stable for
	// many sysclks before the level-ack arrives, so the 2-FF ack sync is
	// sufficient — no separate CDC needed on the data lines (SDC false_paths
	// them like the write data lines).
	input      [15:0] ddr_in_rd
);

// --- c_7m rising-edge detector (slot boundary) ---
reg c_7m_d;
always @(posedge clk) c_7m_d <= c_7m;
wire c_7m_rise = c_7m & ~c_7m_d;

// --- Register akiko_dma_req to cut the long
// combinational arc from akiko's rx_busy/tx_busy state-machine internals
// through arm_now → arb_request → arb_drive into the sd_addr mux selector.
// That arc was the worst-case setup path on the emu PLL (-0.981 ns slack).
// Latency cost: up to 1 clk_sys cycle on the first byte of an akiko burst;
// since req → next c_7m_rise is typically 0..3 clk_sys cycles anyway, this
// is invisible at burst level. Only the SELECTOR is registered — the data
// fields (akiko_dma_baddr, _we, _wbyte) remain combinational into ak_*_w
// so chip_out_addr/etc. still arrive at sdram_ctrl on the same edge.
reg akiko_dma_req_q;
reg cdtv_dma_req_q;
always @(posedge clk) begin
	if (reset) begin
		akiko_dma_req_q <= 1'b0;
		cdtv_dma_req_q  <= 1'b0;
	end else begin
		akiko_dma_req_q <= akiko_dma_req;
		cdtv_dma_req_q  <= cdtv_dma_req;
	end
end

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

// --- Latched DDR3 (ram2) routing fields. memory_router decides
//     ram1 vs ram2 from the master's 24-bit byte address + AC state.
//     Latched at arm_now alongside ak_addr; held through the slot.
reg        ak_is_ddr;
reg [28:1] ak_ddr_addr;

// --- Registered DDR3 bus driven from clk_sys to ddram_ctrl
//     (clk_114). All six lines latched at arm_now and held until the
//     synchronized ack returns. This makes the data lines stable for many
//     clk_114 cycles before ddram_ctrl's sync_CS-rise edge — no need to
//     close cross-domain timing on them; SDC false_path's them. dmaCS is
//     the only signal that must be synchronized (2-FF chain inside
//     ddram_ctrl).
reg        dma_ddr_cs_r;
reg [28:1] dma_ddr_addr_r;
reg        dma_ddr_l_r;
reg        dma_ddr_u_r;
reg [15:0] dma_ddr_wr_r;
// Latched WE, rather than a hardwired `ddr_out_we = 1'b1`, so the bridge can
// do BOTH reads (Akiko TX command fetch, dma_we=0 from akiko.v:819 when only
// tx_busy is set) AND writes (PBX sector data, dma_we=1). Hardwired high,
// Z2-allocated CMD blocks are unreachable and the CD32 BIOS hangs at
// $9FFC00 waiting for a TX read that the bridge silently drops.
reg        dma_ddr_we_r;

// --- 2-FF synchronizer on the LEVEL ddr_in_ack from
//     ddram_ctrl (clk_114). ddr_in_ack goes high when DDR3 commits the
//     write and stays high until we drop dma_ddr_cs_r — long enough that
//     a 2-FF chain always catches the transition cleanly.
reg ddr_in_ack_sync1;
reg ddr_in_ack_sync2;
always @(posedge clk) begin
    if (reset) begin
        ddr_in_ack_sync1 <= 1'b0;
        ddr_in_ack_sync2 <= 1'b0;
    end else begin
        ddr_in_ack_sync1 <= ddr_in_ack;
        ddr_in_ack_sync2 <= ddr_in_ack_sync1;
    end
end
wire ddr_ack_safe = ddr_in_ack_sync2;

// --- State ---
localparam [1:0]
	S_IDLE     = 2'd0,
	S_DRIVE    = 2'd1,
	S_ACK      = 2'd2,
	S_COOLDOWN = 2'd3;

reg [1:0] state;

// --- Output regs back to masters ---
reg [7:0] ak_rbyte_r;
reg       ak_ack_r;
reg       cdtv_ack_r;

// --- Which master is being serviced this slot? Latched at arm_now and
//     held through S_DRIVE/S_ACK/S_COOLDOWN. Static priority akiko > cdtv:
//     in practice the two never both request (different cores), so this
//     just guarantees a deterministic pick if both were ever asserted
//     simultaneously.
reg active_is_cdtv;

assign akiko_dma_rbyte = ak_rbyte_r;
assign akiko_dma_ack   = ak_ack_r;
// CDTV is write-only; rbyte tie-off keeps the port symmetrical.
assign cdtv_dma_rbyte  = 8'h00;
assign cdtv_dma_ack    = cdtv_ack_r;

// --- Minimig idleness on this slot. Combinational from chip_in_dma /
//     chip_in_rw (driven through gary from agnus's registered DMA
//     scheduler). At sdram_ctrl's sample point this value reflects
//     the current slot.
wire minimig_idle = chip_in_dma & chip_in_rw;
wire minimig_busy = ~minimig_idle;

// --- pending_req: either master wants the bus. Selector picks akiko if
//     both raise simultaneously (static priority).
wire any_req       = akiko_dma_req_q | cdtv_dma_req_q;
wire arming_is_cdtv = ~akiko_dma_req_q & cdtv_dma_req_q;

// Live master-side inputs at the arming edge (combinational pick).
wire        live_we     = arming_is_cdtv ? cdtv_dma_we    : akiko_dma_we;
wire [23:0] live_baddr  = arming_is_cdtv ? cdtv_dma_baddr : akiko_dma_baddr;
wire  [7:0] live_wbyte  = arming_is_cdtv ? cdtv_dma_wbyte : akiko_dma_wbyte;

// --- arm_now: combinational claim at the c_7m_rise edge. Drives
//     chip_out_dma=0 within the same clk_sys edge, in time for
//     sdram_ctrl to see it ~8.7 ns later (the existing minimig path
//     fits the same budget the same way). Gated by minimig_idle so we
//     never preempt the chipset.
wire arm_now = (state == S_IDLE) & c_7m_rise & minimig_idle & any_req;

// --- arb_request: we want to be on the bus this cycle. Either we
//     just armed combinationally, or we are mid-slot (S_DRIVE).
wire arb_request = arm_now | (state == S_DRIVE);

// --- owner-freeze: tell akiko the exact cycle we latch its byte so it can
//     freeze the serving engine (arm_now picks akiko when ~arming_is_cdtv).
assign akiko_arm = arm_now & ~arming_is_cdtv;

// --- arb_drive: the actual override. Masked by minimig_busy on EVERY
//     cycle so a slot minimig grabs (e.g. across a c_7m boundary into
//     the next slot) wins immediately. arb_request is registered for
//     subsequent cycles, so we keep driving as long as minimig stays
//     idle; if minimig becomes busy we yield without disturbing the
//     in-progress sdram_ctrl access (it snapshotted at slot start).
wire arb_drive = arb_request & minimig_idle;

// --- When arming, the latched ak_* registers are stale (they hold the
//     PREVIOUS request). Use the live (akiko or cdtv) inputs combinationally
//     for the arming cycle so chip_out_addr/etc. reach sdram_ctrl with
//     the correct address on the very first cycle of the slot. After
//     the arming edge, state==S_DRIVE and arm_now==0, so the registered
//     ak_* values take over -- they were latched by the always block
//     using the same NBA at the arming edge.
wire [24:1] ak_addr_w    = arm_now ? {1'b0, live_baddr[23:1]}            : ak_addr;
wire        ak_l_w       = arm_now ? ~live_baddr[0]                       : ak_l;
wire        ak_u_w       = arm_now ?  live_baddr[0]                       : ak_u;
wire        ak_rw_w      = arm_now ? ~live_we                             : ak_rw;
wire [15:0] ak_wr_data_w = arm_now ? {live_wbyte, live_wbyte}             : ak_wr_data;

// --- ram1-vs-ram2 routing decision via shared memory_router.
//     cchip / ckick / wr tied to 0 — the bridge always reaches chip RAM
//     via chip_out_* regardless of CPU turbo gates, and never writes to
//     KS ROM. Only ramaddr + zram_sel are consumed.
wire [28:1] router_ramaddr;
wire        router_zram_sel;

memory_router u_router
(
	.cpu_addr      ({8'h00, live_baddr}),  // 24-bit byte addr → 32 bits, top byte 0
	.cchip         (1'b0),
	.ckick         (1'b0),
	.wr            (1'b0),
	.bootrom       (1'b0),
	.z2ram_ena     (z2ram_ena),
	.z3ram_base0   (z3ram_base0),
	.z3ram_ena0    (z3ram_ena0),
	.z3ram_base1   (z3ram_base1),
	.z3ram_ena1    (z3ram_ena1),
	.sel_chipram   (),
	.sel_kickram   (),
	.sel_kicklower (),
	.sel_z2ram     (),
	.sel_z3ram0    (),
	.sel_z3ram1    (),
	.sel_zram      (),
	.sel_dd        (),
	.sel_rtg       (),
	.ramaddr       (router_ramaddr),
	.zram_sel      (router_zram_sel)
);

// During arm_now use the live router output; after that, use registered.
// is_ddr_now still needs the live path so the chip-vs-DDR mux below
// switches in time for the SDRAM same-edge sample. arb_drive_ddr is
// supplied by the registered dma_ddr_cs_r below (no clk_sys → SDRAM
// timing dependency, so combinational is unnecessary).
wire        is_ddr_now   = arm_now ? router_zram_sel : ak_is_ddr;

// SDRAM (ram1) override only fires when the slot routes to ram1. This
// path keeps the original combinational shape because sdram_ctrl samples
// on the same clk_sys edge as arm_now (8.7 ns budget).
wire arb_drive_chip = arb_drive & ~is_ddr_now;

assign chip_out_addr = arb_drive_chip ? ak_addr_w    : chip_in_addr;
assign chip_out_l    = arb_drive_chip ? ak_l_w       : chip_in_l;
assign chip_out_u    = arb_drive_chip ? ak_u_w       : chip_in_u;
assign chip_out_rw   = arb_drive_chip ? ak_rw_w      : chip_in_rw;
assign chip_out_dma  = arb_drive_chip ? 1'b0         : chip_in_dma;
assign chip_out_wr   = arb_drive_chip ? ak_wr_data_w : chip_in_wr;

// DDR DMA bus is REGISTERED in chipdma_arb. Data lines stay
// stable from arm_now until the synchronized ack returns and we drop CS,
// so ddram_ctrl can sample them safely after its 2-FF dmaCS sync edge.
// ddr_out_we is hard-wired 1 today (bridge writes only into Z2/Z3).
assign ddr_out_cs   = dma_ddr_cs_r;
assign ddr_out_addr = dma_ddr_addr_r;
assign ddr_out_l    = dma_ddr_l_r;
assign ddr_out_u    = dma_ddr_u_r;
assign ddr_out_we   = dma_ddr_we_r;
assign ddr_out_wr   = dma_ddr_wr_r;

always @(posedge clk) begin
	if (reset) begin
		state          <= S_IDLE;
		slot_cnt       <= 3'd0;
		ak_ack_r       <= 1'b0;
		cdtv_ack_r     <= 1'b0;
		ak_rbyte_r     <= 8'h00;
		active_is_cdtv <= 1'b0;
		ak_is_ddr      <= 1'b0;
		dma_ddr_cs_r   <= 1'b0;
		dma_ddr_we_r   <= 1'b1;  // safe default — bridge was write-only before this fix
	end else begin
		ak_ack_r   <= 1'b0;  // ack defaults low; pulse in S_ACK on owner only
		cdtv_ack_r <= 1'b0;

		case (state)
		S_IDLE: begin
			if (arm_now) begin
				ak_addr        <= {1'b0, live_baddr[23:1]};
				ak_u           <= live_baddr[0];
				ak_l           <= ~live_baddr[0];
				ak_rw          <= ~live_we;
				ak_wr_data     <= {live_wbyte, live_wbyte};
				ak_we          <= live_we;
				ak_baddr0      <= live_baddr[0];
				ak_is_ddr      <= router_zram_sel;
				ak_ddr_addr    <= router_ramaddr;
				slot_cnt       <= 3'd0;
				active_is_cdtv <= arming_is_cdtv;
				// When this slot routes to DDR, latch the
				// full DDR bus on the same arm_now edge. Data is held
				// stable from here until S_ACK clears dma_ddr_cs_r.
				if (router_zram_sel) begin
					dma_ddr_cs_r   <= 1'b1;
					dma_ddr_addr_r <= router_ramaddr;
					dma_ddr_l_r    <= ~live_baddr[0];
					dma_ddr_u_r    <=  live_baddr[0];
					dma_ddr_wr_r   <= {live_wbyte, live_wbyte};
					// Latch the real WE so ddram_ctrl
					// can route this transaction as either a write (PBX) or
					// a read (TX command fetch).
					dma_ddr_we_r   <= live_we;
				end
				state          <= S_DRIVE;
			end
		end

		S_DRIVE: begin
			if (ak_is_ddr) begin
				// Routing to ram2 (DDR3). dma_ddr_cs_r is
				// REGISTERED high; ddram_ctrl synchronizes it through a
				// 2-FF chain in the clk_114 domain, edge-detects the
				// rise, and latches our (already-stable) data into its
				// own write buffer. ddr_in_ack comes back as a level
				// signal which we sample through ddr_in_ack_sync2.
				if (ddr_ack_safe) begin
					// On a TX (read) the
					// captured 16-bit word is in ddr_in_rd, latched
					// by ddram_ctrl's read FSM at the same instant
					// ddr_in_ack rises. Demux to the byte the master
					// asked for, same convention as the chip-RAM
					// path below (ak_baddr0=0 → upper byte).
					if (!ak_we) begin
						ak_rbyte_r <= ak_baddr0 ? ddr_in_rd[7:0]
						                        : ddr_in_rd[15:8];
					end
					state <= S_ACK;
				end
			end else begin
				slot_cnt <= slot_cnt + 3'd1;
				// Sample chipRD at slot_cnt==3 (4 clk_sys cycles after
				// the arming edge = 16 clk_114 cycles, well past
				// sdram_ctrl's state-9 chipRD update).
				if (slot_cnt == 3'd3) begin
					if (!ak_we) begin
						ak_rbyte_r <= ak_baddr0 ? chip_in_rd[7:0]
						                        : chip_in_rd[15:8];
					end
					state <= S_ACK;
				end
			end
		end

		S_ACK: begin
			// Pulse ack only to the owner master.
			if (active_is_cdtv) cdtv_ack_r <= 1'b1;
			else                ak_ack_r   <= 1'b1;
			// Drop the registered CS — ddram_ctrl's dmaCS_sync chain
			// will see the falling edge a few clk_114 cycles later and
			// release dmaACK so the next request can latch.
			dma_ddr_cs_r <= 1'b0;
			state <= S_COOLDOWN;
		end

		S_COOLDOWN: begin
			// Guarantee at least one cycle of !dma_ack between two
			// successive master services so the master's handshake
			// (e.g. akiko.v:522-528 rx_inflight) sees the gap. Also
			// gives the dmaACK sync chain (in chipdma_arb) time to
			// settle low before the next arm_now.
			//
			// For DDR (ram2) slots, also
			// hold here until ddr_ack_safe has fully DEASSERTED before
			// returning to S_IDLE. ddr_in_ack is a LEVEL from ddram_ctrl
			// that stays high until it sees our dmaCS drop (through its own
			// 2-FF sync); ddr_ack_safe is a further 2-FF sync of that. With
			// only a 1-cycle cooldown, a tightly-spaced next byte re-arms
			// dma_ddr_cs_r and re-enters S_DRIVE while ddr_ack_safe is still
			// HIGH from the previous byte, so we (a) trust a stale ack and
			// jump to S_ACK without the new byte being committed, and (b)
			// ddram_ctrl's dmaACK_r is still high when our new dmaCS rise
			// arrives, so its latch gate (dmaCS_rise & ~dmaACK_r) blocks and
			// the write is dropped. Waiting for ddr_ack_safe low closes both
			// halves: it guarantees ddram_ctrl's dmaACK_r is low (>=2 clk_sys
			// old) before we raise the next CS. Chip (ram1) slots are
			// unaffected (~ak_is_ddr keeps the original 1-cycle path).
			if (~ak_is_ddr | ~ddr_ack_safe)
			state <= S_IDLE;
		end
		endcase
	end
end

endmodule
