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
	// Path to a Quartus .mif image (1024 bytes) that pre-loads the
	// altsyncram M10K instances. Default works when synthesis is run
	// from the project root (Quartus); standalone benches override
	// with a sim-relative path or the literal string "UNUSED" to skip
	// preload (RAM powers up all-zero with power_up_uninitialized=FALSE).
	parameter INIT_FILE = "rtl/init/nvram_init.mif"
)
(
	input  wire        clk,
	input  wire        reset,

	// I2C bus state, post-master-output mux done in akiko.v wrapper.
	input  wire        scl_in,    // current SCL value on the bus
	input  wire        sda_in,    // current SDA value on the bus

	// Slave open-drain pull-down. 1 = slave is asserting SDA low.
	output wire        sda_drive,

	// HPS save-dump read port. host_addr is registered into BRAM by the
	// altsyncram Port B address register; host_dout lags by one clk edge.
	// host_clear_dirty pulses to clear nvram_dirty after a save dump
	// completes (driven by bridge xfer_end on a read transaction).
	input  wire [9:0]  host_addr,
	output wire [7:0]  host_dout,
	input  wire        host_clear_dirty,
	output reg         nvram_dirty = 1'b0,

	// HPS load-from-disk write port. Driven by hps_io.ioctl_download
	// gated on NVR_LOAD_INDEX in Minimig.sv. Lives in HPS reset domain —
	// fires before BIOS touches I²C, so no contention with the slave
	// state machine. load_we does NOT set nvram_dirty (loading saved
	// state must not provoke an immediate re-save).
	input  wire [9:0]  load_addr,
	input  wire [7:0]  load_din,
	input  wire        load_we
);

// 24LC08: 1 KiB = 1024 bytes, 16-byte page, addr-bits-in-device-byte = 2 (A8..A9)
localparam ADDR_W = 10;

// Storage signals; mem_dout/host_dout are driven by altsyncram Port B
// outputs declared further down. mem_we / mem_waddr are produced by the
// I2C state machine and routed via mem_*_mux to Port A of both rams.
// Initial values (no-reset). The slave's I²C state machine is decoupled
// from the chip-wide `reset` signal (= ~cpu_rst | ~cpu_nrst_out) so the
// load_we-driven HPS write path can fire regardless of CD32 CPU reset
// state — load happens before BIOS touches I²C.
wire [7:0]        mem_dout;
reg               mem_we    = 1'b0;
reg  [ADDR_W-1:0] mem_waddr = {ADDR_W{1'b0}}; // captured at write-issue time so eeprom_addr can advance NBA in the same cycle

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

// Phase 33-J: initial values for all I²C state-machine regs (no-reset
// powerup). See banner comment above.
reg [2:0] state                   = ST_IDLE;
reg [1:0] byte_phase              = BYTE_DEVADDR;
reg [3:0] bit_count               = 4'd0;
reg [7:0] shift_reg               = 8'h00;
reg [ADDR_W-1:0] eeprom_addr      = {ADDR_W{1'b0}};
reg       is_read                 = 1'b0;          // direction from device-address byte (1=read, 0=write)
reg       dev_match                = 1'b0;          // device address byte was 1010xxxx
reg       sda_oe                  = 1'b0;          // 1 = pull SDA low

// Edge detection on SCL/SDA. Sampled at clk_sys, way faster than I2C
// software bit-bang (BIOS toggles in microseconds vs our nanosecond clock).
reg prev_scl = 1'b1, prev_sda = 1'b1;
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
// Two altsyncram megafunctions explicitly instantiated (rather than
// inferred) so Quartus emits a per-instance .mif binding for the M10K
// init_file — one shared write port, two independent read ports
// (memory_a feeds the I²C slave, memory_b feeds the HPS save-dump).
//
// Write priority: load_we (HPS-driven from ioctl_download) wins over
// mem_we (I²C slave). They are temporally disjoint in normal use —
// load fires at core init before BIOS touches I²C — so the priority
// is a safety belt, not a contention resolver.

wire [ADDR_W-1:0] mem_waddr_mux = load_we ? load_addr : mem_waddr;
wire        [7:0] mem_din_mux   = load_we ? load_din  : shift_reg;
wire              mem_we_mux    = load_we | mem_we;

// memory_a_inst feeds the I2C read port (mem_dout, addressed by eeprom_addr).
// memory_b_inst feeds the HPS read port (host_dout, addressed by host_addr).
// Both instances receive identical writes on Port A every clock and use the
// same INIT_FILE. Output latency = 1 clk (address registered into Port B,
// output wire combinational from BRAM array — matches the prior always-block
// behaviour exactly).
altsyncram memory_a_inst (
	.address_a (mem_waddr_mux),
	.clock0    (clk),
	.data_a    (mem_din_mux),
	.wren_a    (mem_we_mux),
	.address_b (eeprom_addr),
	.q_b       (mem_dout),
	.aclr0     (1'b0),
	.aclr1     (1'b0),
	.addressstall_a (1'b0),
	.addressstall_b (1'b0),
	.byteena_a (1'b1),
	.byteena_b (1'b1),
	.clock1    (1'b1),
	.clocken0  (1'b1),
	.clocken1  (1'b1),
	.clocken2  (1'b1),
	.clocken3  (1'b1),
	.data_b    (8'h00),
	.eccstatus (),
	.q_a       (),
	.rden_a    (1'b1),
	.rden_b    (1'b1),
	.wren_b    (1'b0)
);
defparam
	memory_a_inst.address_aclr_b = "NONE",
	memory_a_inst.address_reg_b = "CLOCK0",
	memory_a_inst.clock_enable_input_a = "BYPASS",
	memory_a_inst.clock_enable_input_b = "BYPASS",
	memory_a_inst.clock_enable_output_b = "BYPASS",
	memory_a_inst.init_file = INIT_FILE,
	memory_a_inst.intended_device_family = "Cyclone V",
	memory_a_inst.lpm_type = "altsyncram",
	memory_a_inst.numwords_a = 1024,
	memory_a_inst.numwords_b = 1024,
	memory_a_inst.operation_mode = "DUAL_PORT",
	memory_a_inst.outdata_aclr_b = "NONE",
	memory_a_inst.outdata_reg_b = "UNREGISTERED",
	memory_a_inst.power_up_uninitialized = "FALSE",
	memory_a_inst.ram_block_type = "M10K",
	memory_a_inst.read_during_write_mode_mixed_ports = "OLD_DATA",
	memory_a_inst.widthad_a = 10,
	memory_a_inst.widthad_b = 10,
	memory_a_inst.width_a = 8,
	memory_a_inst.width_b = 8,
	memory_a_inst.width_byteena_a = 1;

altsyncram memory_b_inst (
	.address_a (mem_waddr_mux),
	.clock0    (clk),
	.data_a    (mem_din_mux),
	.wren_a    (mem_we_mux),
	.address_b (host_addr),
	.q_b       (host_dout),
	.aclr0     (1'b0),
	.aclr1     (1'b0),
	.addressstall_a (1'b0),
	.addressstall_b (1'b0),
	.byteena_a (1'b1),
	.byteena_b (1'b1),
	.clock1    (1'b1),
	.clocken0  (1'b1),
	.clocken1  (1'b1),
	.clocken2  (1'b1),
	.clocken3  (1'b1),
	.data_b    (8'h00),
	.eccstatus (),
	.q_a       (),
	.rden_a    (1'b1),
	.rden_b    (1'b1),
	.wren_b    (1'b0)
);
defparam
	memory_b_inst.address_aclr_b = "NONE",
	memory_b_inst.address_reg_b = "CLOCK0",
	memory_b_inst.clock_enable_input_a = "BYPASS",
	memory_b_inst.clock_enable_input_b = "BYPASS",
	memory_b_inst.clock_enable_output_b = "BYPASS",
	memory_b_inst.init_file = INIT_FILE,
	memory_b_inst.intended_device_family = "Cyclone V",
	memory_b_inst.lpm_type = "altsyncram",
	memory_b_inst.numwords_a = 1024,
	memory_b_inst.numwords_b = 1024,
	memory_b_inst.operation_mode = "DUAL_PORT",
	memory_b_inst.outdata_aclr_b = "NONE",
	memory_b_inst.outdata_reg_b = "UNREGISTERED",
	memory_b_inst.power_up_uninitialized = "FALSE",
	memory_b_inst.ram_block_type = "M10K",
	memory_b_inst.read_during_write_mode_mixed_ports = "OLD_DATA",
	memory_b_inst.widthad_a = 10,
	memory_b_inst.widthad_b = 10,
	memory_b_inst.width_a = 8,
	memory_b_inst.width_b = 8,
	memory_b_inst.width_byteena_a = 1;

// Dirty flag: latched by any successful sequential write from the I2C
// path (mem_we). Userspace reads the flag via the bridge status word,
// dumps NVRAM via the read sub-channel — which auto-clears dirty on
// transaction end (host_clear_dirty pulse from the bridge).
//
// Invariants:
//   - load_we does NOT set dirty (loading from disk must not trigger
//     an immediate re-save of what we just loaded).
//   - mem_we wins over host_clear_dirty on the same cycle: if BIOS
//     writes during the dump, dirty stays set so the next poll re-saves.
// `reset` removed from these always blocks: initial values cover FPGA
// configuration; STOP/START bus-protocol recovery covers stuck-state
// escape. Decoupling from `reset` (= ~cpu_rst | ~cpu_nrst_out) lets
// load_we work regardless of CD32 CPU reset state.
always @(posedge clk) begin
	if (mem_we)                 nvram_dirty <= 1'b1;
	else if (host_clear_dirty)  nvram_dirty <= 1'b0;
end

always @(posedge clk) begin
	prev_scl <= scl_in;
	prev_sda <= sda_in;
	mem_we   <= 1'b0;            // single-cycle pulse; default off

	// STOP condition unconditionally returns to idle and releases SDA.
	if (stop_cond) begin
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
