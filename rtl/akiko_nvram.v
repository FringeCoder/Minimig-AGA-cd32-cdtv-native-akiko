// CD32 NVRAM I2C slave EEPROM (24LC08-equivalent, 1 KiB)
//
// Phase 13: replaces the M1 stub at akiko.v:711 that returned 0xFF (NACK)
// for every NVRAM access. CD32 BIOS spent ~half its boot trace bit-banging
// I2C looking for the NVRAM chip; on NACK it retries forever and never
// finishes cd.device init, so MULTI/TOC never fires. With a real I2C slave
// answering, BIOS reads zeros (blank NVRAM, "no saved config") and moves on.
//
// Reference implementation: WinUAE flashrom.cpp:369-520 (eeprom_i2c_set,
// bitbang_i2c_state) — this is a faithful Verilog port of that state
// machine. See also akiko.cpp:247-300 for the host-side bit/direction
// register layout (bit 7 = SCL, bit 6 = SDA on $B80030).
//
// Storage is volatile BRAM. Persistence (save to SD via HPS) is a
// separate step — not needed for boot or for Cannon Fodder MVP.
//
// Bus model: open-drain wired-AND. Master writes drive scl_in/sda_in
// when its direction bits select output; otherwise the line floats
// high (1) at the akiko.v wrapper. The slave only ever PULLS DOWN
// (sda_drive=1 → bus reads 0); when sda_drive=0 the master sees its
// own value or pulled-high-by-resistor.

module akiko_nvram
#(
	// Path to a 1024-byte hex image (one byte per line) used to pre-load
	// the M10K. Default works when synthesis is run from the project root
	// (Quartus); standalone benches override with the sim-relative path.
	parameter INIT_FILE = "rtl/init/nvram_init.hex"
)
(
	input  wire        clk,
	input  wire        reset,

	// I2C bus state, post-master-output mux done in akiko.v wrapper.
	input  wire        scl_in,    // current SCL value on the bus
	input  wire        sda_in,    // current SDA value on the bus

	// Slave open-drain pull-down. 1 = slave is asserting SDA low.
	output wire        sda_drive,

	// Phase 32: HPS host port for save-to-disk dump.
	// Phase 32.5: extended with write port for load-from-disk on core init.
	// Inferred as a true dual-port M10K alongside the I2C path. host_addr
	// is registered into BRAM by the always block; host_dout lags by one
	// clk edge. host_we writes host_din at host_addr (does NOT set dirty —
	// loading saved state should NOT trigger an immediate re-save).
	// host_clear_dirty pulses to clear nvram_dirty after a save dump
	// completes (driven by bridge xfer_end on a read transaction).
	input  wire [9:0]  host_addr,
	input  wire [7:0]  host_din,
	input  wire        host_we,
	output reg  [7:0]  host_dout,
	input  wire        host_clear_dirty,
	output reg         nvram_dirty
);

// 24LC08: 1 KiB = 1024 bytes, 16-byte page, addr-bits-in-device-byte = 2 (A8..A9)
//
// Memory is inferred as a single M10K block: one synchronous read port (mem_dout
// always reflects memory[eeprom_addr] one cycle later) and one synchronous write
// port (mem_we gated). Without this structure Quartus 17 spreads 8192 flops
// across the device (~3900 ALMs) which pushes the borderline pbx_seccnt -> sdram
// path into setup violation.
localparam ADDR_W = 10;
// Phase 33-A iter3 (2026-05-04): explicit dual-memory pattern. Iter2's
// single-array+single-write-port template was supposed to infer one M10K
// or fan a single write to two duplicates. Quartus 17 actually inferred
// TWO M10Ks (memory_rtl_0 for I2C reads, memory_rtl_1 for HPS reads) but
// only fanned the I2C-side mem_we write port to BOTH; host_we writes
// landed nowhere visible to host_dout. Empirically: host_dout returned
// all zeros even after host_we write loops, then "magically" returned
// FlashFile magic 5s after boot — i.e., once BIOS wrote the same magic
// via I2C. The .mif also didn't initialize memory_rtl_1 at FPGA config.
//
// Fix: declare TWO memory arrays explicitly. Both get $readmemh init,
// both receive both writes (host_we and mem_we) on the same cycle. This
// removes all inference ambiguity — Quartus must give us 2 M10Ks each
// with their own write port driven by both signals (ORed via mem_we_mux).
(* ramstyle = "M10K, no_rw_check" *) reg [7:0] memory_a [0:1023];
(* ramstyle = "M10K, no_rw_check" *) reg [7:0] memory_b [0:1023];
reg [7:0]        mem_dout;
reg              mem_we;
reg [ADDR_W-1:0] mem_waddr;     // captured at write-issue time so eeprom_addr can advance NBA in the same cycle

// Phase 14: pre-load M10K with a real cd32.nvr so BIOS sees the FlashFile
// magic (0x00 0x56 0xA9 ...) at boot. With all-zeros NVRAM, BIOS fails its
// FlashFile root-block check and loops searching forever instead of issuing
// MULTI. $readmemh is honored by both ModelSim and Quartus 17 for inferred
// M10K, so the same image is used in sim and on hardware.
initial $readmemh(INIT_FILE, memory_a);
initial $readmemh(INIT_FILE, memory_b);

// I2C state machine. Mirrors WinUAE bitbang_i2c_state but compacted:
// the WinUAE per-bit states (SENDING_BIT7..0, RECEIVING_BIT7..0) are
// replaced by a 3-bit bit_count inside ST_RX_DATA / ST_TX_DATA.
localparam ST_IDLE    = 3'd0;  // waiting for START
localparam ST_RX_DATA = 3'd1;  // master is shifting a byte INTO us
localparam ST_RX_ACK  = 3'd2;  // we drive ACK low for the 9th clock pulse
localparam ST_TX_DATA = 3'd3;  // we are shifting a byte OUT to master
localparam ST_TX_ACK  = 3'd4;  // master drives ACK on the 9th clock pulse

// Byte-interpretation phase (matches WinUAE's `estate`).
localparam BYTE_DEVADDR  = 2'd0;
localparam BYTE_WORDADDR = 2'd1;
localparam BYTE_DATA     = 2'd2;

reg [2:0] state;
reg [1:0] byte_phase;       // BYTE_DEVADDR/WORDADDR/DATA
reg [3:0] bit_count;        // 0..7 within a byte, plus a 1-bit "ack second-fall" flag
reg [7:0] shift_reg;
reg [ADDR_W-1:0] eeprom_addr;
reg       is_read;          // direction from device-address byte (1=read, 0=write)
reg       dev_match;        // device address byte was 1010xxxx
reg       sda_oe;           // 1 = pull SDA low

// Edge detection on SCL/SDA. Sampled at clk_sys, way faster than I2C
// software bit-bang (BIOS toggles in microseconds vs our nanosecond clock).
reg prev_scl, prev_sda;
wire scl_rise = ~prev_scl &  scl_in;
wire scl_fall =  prev_scl & ~scl_in;
wire sda_rise = ~prev_sda &  sda_in;
wire sda_fall =  prev_sda & ~sda_in;

// I2C START / STOP are SDA transitions while SCL is high.
wire start_cond = scl_in & sda_fall;
wire stop_cond  = scl_in & sda_rise;

// Synchronous BRAM ports. mem_dout / host_dout each lag their respective
// addr by 1 cycle. Since the I2C addr is stable for many clk cycles
// between bus edges and the host_addr increments deliberately per UIO
// strobe, the readouts are always valid by the time downstream consumes
// them.
//
// Phase 32  — added a second read port for the HPS save-dump path.
// Phase 32.5 — added a write side (host_we / host_din) so userspace can
//              push a saved cd32-<hash>.nvr back into BRAM at core init.
// Phase 33-A iter1 (2026-05-04) — TDP attempt with two separate always
//              blocks: Quartus 17 refused inference ("RAM logic ... is
//              uninferred due to too many ports") AND the fabric-flop
//              fallback also dropped host_we writes (verified on
//              hardware: NVR LOAD VERIFY FAILED 4/1024). Reverted.
// Phase 33-A iter2 (2026-05-04) — canonical SIMPLE DUAL PORT template.
//              Both writes feed a single muxed write port; host_we wins
//              on (impossible) collision. Two read ports remain (one
//              for I2C, one for HPS dump). This is the textbook SDP-2R
//              pattern Quartus 17 reliably infers as one M10K with no
//              inference ambiguity.
//
// Write-write collision protocol: host_we only fires during the
// load-from-disk burst, which happens at akiko_cd32_init() before BIOS
// touches I2C. mem_we only fires during runtime I2C transactions, after
// BIOS owns the bus. Temporally disjoint, so the priority mux is purely
// a safety belt; runtime behaviour is unaffected.

wire [ADDR_W-1:0] mem_waddr_mux = host_we ? host_addr : mem_waddr;
wire        [7:0] mem_din_mux   = host_we ? host_din  : shift_reg;
wire              mem_we_mux    = host_we | mem_we;

// memory_a feeds the I2C read port (mem_dout, addressed by eeprom_addr).
// memory_b feeds the HPS read port (host_dout, addressed by host_addr).
// Both arrays receive the same writes on every clock — they are kept
// byte-for-byte identical. This is the canonical "1W+2R = 2 SDP M10Ks
// with shared write" pattern, written explicitly so Quartus can't lose
// the host_we fan-out.
always @(posedge clk) begin
	if (mem_we_mux) begin
		memory_a[mem_waddr_mux] <= mem_din_mux;
		memory_b[mem_waddr_mux] <= mem_din_mux;
	end
	mem_dout  <= memory_a[eeprom_addr];
	host_dout <= memory_b[host_addr];
end

// Dirty flag: latched by any successful sequential write from the I2C
// path (mem_we). Userspace reads the flag via the bridge status word,
// dumps NVRAM via the read sub-channel — which auto-clears dirty on
// transaction end (host_clear_dirty pulse from the bridge). Reset
// clears it so a fresh boot doesn't look dirty just because the .hex
// baseline equals the eventual SD-saved file.
//
// Phase 32.5 invariants:
//   - host_we does NOT set dirty (loading from disk should not trigger
//     an immediate re-save of what we just loaded).
//   - mem_we wins over host_clear_dirty on the same cycle: if BIOS
//     writes during the dump, dirty stays set so the next poll re-saves.
always @(posedge clk) begin
	if (reset)                  nvram_dirty <= 1'b0;
	else if (mem_we)            nvram_dirty <= 1'b1;
	else if (host_clear_dirty)  nvram_dirty <= 1'b0;
end

always @(posedge clk) begin
	prev_scl <= scl_in;
	prev_sda <= sda_in;
	mem_we   <= 1'b0;            // single-cycle pulse; default off

	if (reset) begin
		state       <= ST_IDLE;
		byte_phase  <= BYTE_DEVADDR;
		bit_count   <= 4'd0;
		shift_reg   <= 8'h0;
		eeprom_addr <= {ADDR_W{1'b0}};
		is_read     <= 1'b0;
		dev_match   <= 1'b0;
		sda_oe      <= 1'b0;
	end
	// STOP condition unconditionally returns to idle and releases SDA.
	else if (stop_cond) begin
		state      <= ST_IDLE;
		byte_phase <= BYTE_DEVADDR;
		bit_count  <= 4'd0;
		sda_oe     <= 1'b0;
	end
	// START condition (or repeated START) resets the byte phase, releases
	// SDA, and arms us to receive the device-address byte.
	else if (start_cond) begin
		state      <= ST_RX_DATA;
		byte_phase <= BYTE_DEVADDR;
		bit_count  <= 4'd0;
		shift_reg  <= 8'h0;
		sda_oe     <= 1'b0;
	end
	else begin
		case (state)

		// ----------------------------------------------------------
		// IDLE: wait for START. Nothing to do here — the start_cond
		// branch above will move us to ST_RX_DATA.
		// ----------------------------------------------------------
		ST_IDLE: ;

		// ----------------------------------------------------------
		// ST_RX_DATA: master is shifting bits in (MSB first). Sample
		// SDA on each SCL rising edge until we have 8 bits, then move
		// to ST_RX_ACK to drive our ACK on the 9th clock pulse.
		// ----------------------------------------------------------
		ST_RX_DATA: begin
			if (scl_rise) begin
				shift_reg <= {shift_reg[6:0], sda_in};
				if (bit_count == 4'd7) begin
					state     <= ST_RX_ACK;
					bit_count <= 4'd0;
				end else begin
					bit_count <= bit_count + 4'd1;
				end
			end
		end

		// ----------------------------------------------------------
		// ST_RX_ACK: 9th clock pulse — we are responsible for ACKing
		// the byte we just received. Two SCL fallings happen during
		// this state:
		//   - first fall (after 8th rising): commit byte side-effects
		//     (write to memory / capture word addr / validate dev addr)
		//     and drive SDA low to ACK (or leave it released to NACK).
		//   - second fall (after 9th rising = master sampled our ACK):
		//     release SDA and advance to next byte / next state.
		// bit_count[0] tracks which fall we're on.
		// ----------------------------------------------------------
		ST_RX_ACK: begin
			if (scl_fall && bit_count == 4'd0) begin
				bit_count <= 4'd1;
				case (byte_phase)
				BYTE_DEVADDR: begin
					// 24Cxx device address = 1010 a2 a1 R/W
					// where a2..a1 are the high bits of the word addr.
					// Our 1 KiB part uses 2 of those (A8, A9).
					if (shift_reg[7:4] == 4'b1010) begin
						dev_match            <= 1'b1;
						is_read              <= shift_reg[0];
						eeprom_addr[ADDR_W-1:8] <= shift_reg[2:1];
						sda_oe               <= 1'b1;        // ACK
					end else begin
						dev_match <= 1'b0;
						sda_oe    <= 1'b0;                  // NACK
					end
				end
				BYTE_WORDADDR: begin
					eeprom_addr[7:0] <= shift_reg;
					sda_oe           <= 1'b1;               // ACK
				end
				BYTE_DATA: begin
					// Sequential byte write. Real 24LC08 wraps within a
					// 16-byte page on auto-increment; we wrap at the full
					// 1 KiB instead, which is simpler and what BIOS sees
					// as long as it never relies on wrap-within-page.
					mem_we              <= 1'b1;
					mem_waddr           <= eeprom_addr;     // capture pre-increment
					eeprom_addr         <= eeprom_addr + 10'd1;
					sda_oe              <= 1'b1;            // ACK
				end
				endcase
			end
			else if (scl_fall && bit_count == 4'd1) begin
				// 9th SCL fall — release SDA and decide next state.
				sda_oe    <= 1'b0;
				bit_count <= 4'd0;
				if (!dev_match) begin
					state <= ST_IDLE;
				end
				else case (byte_phase)
				BYTE_DEVADDR: begin
					if (is_read) begin
						// Master will now read bytes from current addr.
						// Drive bit 7 on entry — master samples it on the
						// FIRST scl_rise after this transition, before
						// ST_TX_DATA's first scl_fall handler runs. Load
						// bits 6..0 into shift_reg's top, then ST_TX_DATA
						// shifts on each scl_fall to drive bits 6..0.
						// mem_dout already holds memory[eeprom_addr] (the
						// addr has been stable for many clk cycles since
						// it was assembled in the first-fall handler).
						state       <= ST_TX_DATA;
						sda_oe      <= ~mem_dout[7];
						shift_reg   <= {mem_dout[6:0], 1'b0};
						eeprom_addr <= eeprom_addr + 10'd1;
						bit_count   <= 4'd1;
						byte_phase  <= BYTE_DATA;
					end else begin
						// Master will send word-address byte next.
						state      <= ST_RX_DATA;
						byte_phase <= BYTE_WORDADDR;
					end
				end
				BYTE_WORDADDR: begin
					// Next byte is data (write).
					state      <= ST_RX_DATA;
					byte_phase <= BYTE_DATA;
				end
				BYTE_DATA: begin
					// Another data byte coming (sequential write).
					state <= ST_RX_DATA;
				end
				endcase
			end
		end

		// ----------------------------------------------------------
		// ST_TX_DATA: we drive a byte out to the master, MSB first.
		// SDA is updated on each SCL falling edge so it's stable when
		// the master samples on the rising. After bit 0 is sent
		// (8 falls), we release SDA on the next fall and move to
		// ST_TX_ACK to sample the master's ACK/NACK.
		// ----------------------------------------------------------
		ST_TX_DATA: begin
			// Bit 7 is driven on STATE ENTRY (in ST_RX_ACK or ST_TX_ACK
			// fall handler), with bit_count preset to 1. We drive bits
			// 6..0 here, one per scl_fall (so they're stable on the
			// upcoming master sample). After bit 0 is driven (bit_count
			// reaches 8), the next scl_fall releases SDA and hands off
			// to ST_TX_ACK for the master's ACK.
			if (scl_fall) begin
				if (bit_count <= 4'd7) begin
					sda_oe    <= ~shift_reg[7];
					shift_reg <= {shift_reg[6:0], 1'b0};
					bit_count <= bit_count + 4'd1;
				end
				else begin
					sda_oe    <= 1'b0;
					bit_count <= 4'd0;
					state     <= ST_TX_ACK;
				end
			end
		end

		// ----------------------------------------------------------
		// ST_TX_ACK: master is responsible for ACK/NACK on the 9th
		// SCL rising. We sample SDA there: 0 = ACK (read more), 1 =
		// NACK (stop / wait for STOP/START). Then on the 9th SCL
		// fall we either pre-fetch the next byte or sit until STOP.
		// ----------------------------------------------------------
		ST_TX_ACK: begin
			if (scl_rise) begin
				// dev_match flag doubles as "master ACKed last byte"
				// to gate the upcoming fall-edge transition.
				dev_match <= ~sda_in;   // 1 if ACK (sda_in=0), 0 if NACK
			end
			else if (scl_fall) begin
				if (dev_match) begin
					// Master ACKed → fetch next byte and continue read.
					// Drive bit 7 immediately (matches the entry from
					// ST_RX_ACK pattern) — master samples it on the
					// upcoming first scl_rise. mem_dout reflects
					// memory[eeprom_addr] from the cycle before this
					// scl_fall (eeprom_addr was incremented in the prior
					// ST_TX_DATA exit, then held stable through the ACK
					// rise/fall by ST_TX_ACK).
					state       <= ST_TX_DATA;
					sda_oe      <= ~mem_dout[7];
					shift_reg   <= {mem_dout[6:0], 1'b0};
					eeprom_addr <= eeprom_addr + 10'd1;
					bit_count   <= 4'd1;
				end else begin
					// Master NACKed → wait for STOP (or new START).
					state <= ST_IDLE;
				end
			end
		end

		default: state <= ST_IDLE;
		endcase
	end
end

assign sda_drive = sda_oe;

endmodule
