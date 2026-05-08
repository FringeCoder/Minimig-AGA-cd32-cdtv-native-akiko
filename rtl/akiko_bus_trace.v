// Akiko bus trace — 128-deep ring buffer of CPU accesses to the Akiko window
// at $B80000-$B800FF. Drained by Main_MiSTer via a new SPI sub-channel
// (akiko_cs && io_din[7], i.e. UIO class 0xF400 with bit 7 set on byte 1).
//
// v27: writes-only — reads were drowning the ring at MHz rates.
// v28 (CR2/HQ2/Microcosm retry-storm diagnosis): re-introduce filtered reads.
//   - Writes always captured (every cycle).
//   - Reads captured only when ALL of:
//       rd_filter_en        (build-time / runtime gate, expected to be 1)
//       rd_arm              (one-shot pulse-extended window from akiko.v —
//                            armed for ~22 ms after each cdcomtxcmp write,
//                            so we capture the BIOS read traffic that
//                            *follows* a CMD submission and ignore the
//                            quiet-state INTREQ polling)
//       rd_addr_interesting (control/status registers only — INTREQ, INTENA,
//                            TX/RX index pair, PBX, CDFLAG)
//   The else-if mux into the ring's single write port preserves Quartus
//   M10K inference (one writer per cycle).
//
// One trace entry = 32 bits, drained as 4 bytes LSB-first:
//   byte 0: {wr/rd, addr[6:0]}    (bit 7: 1 = write, 0 = read)
//   byte 1: data[7:0]
//   byte 2: data[15:8]
//   byte 3: 0xFF = entry valid, 0x00 = ring empty
//
// The valid bit lets userspace poll-read until it sees a zero and stop.

module akiko_bus_trace
(
	input             clk,
	input             reset,

	// Bus snapshot from fastchip (one-cycle sel_akiko pulse per access).
	input             sel,           // sel_akiko
	input             rd,            // rnw
	input             wr,            // ~rnw & (lds|uds)
	input       [6:0] addr,          // addr[7:1]
	input      [15:0] din,           // CPU -> Akiko (writes)
	input      [15:0] dout,          // Akiko -> CPU (reads)

	// v28 read-trace gates. rd_arm is pulsed by akiko.v on cdcomtxcmp writes.
	input             rd_filter_en,  // 1 = capture filtered reads (else writes only)
	input             rd_arm,        // 1 = inside post-CMD arm window

	// UIO read port. uio_rd pulses pop one byte (4-byte entries auto-advance).
	input             uio_cs_trace,  // akiko_cs && io_din[7] (new sub-channel)
	input             uio_rd,
	output reg  [7:0] uio_dout
);

// 128-entry ring (7-bit pointers). Writes-only (see header).
reg [31:0] ring [0:127];
reg  [6:0] wr_ptr;
reg  [6:0] rd_ptr;
wire       empty = (wr_ptr == rd_ptr);

// Stage the bus inputs one cycle. `dout_d` in particular breaks the long
// combinational path from akiko's register-file read mux into the ring's
// write port, which was costing ~0.4 ns of setup slack on clk_sys.
//
// BUGFIX: dout_d MUST be sampled when sel is asserted (same cycle as the
// real read), not unconditionally. cd_dout reverts to 16'h0 the cycle
// after `cs` drops, and addr/dout_r evaluate to whatever the next bus
// access wants. Sampling unconditionally captured stale combinational
// values from the NEXT bus cycle, producing nonsense in the ring (e.g.
// $B80000 ID reads showed 0x0000 instead of 0xC0CA, and $B8001A reads
// showed advancing rxinx values that did not match register state).
// din is fine to stage unconditionally because the CPU drives it stably
// during the write cycle and write data does not change shape.
reg sel_d, rd_d, wr_d;
reg [6:0] addr_d;
reg [15:0] din_d, dout_d;

// v28 read-capture filter — narrow to control/status registers only so
// the cycles armed by rd_arm don't bury the writes we still want.
//   $04-$07 INTREQ        addr[6:1] = 6'b000010..6'b000011 (addr[6:1]==000010)
//   $08-$0B INTENA        addr[6:1] = 6'b000100..6'b000101 (addr[6:1]==000100)
//   $10-$1F TX/RX inx/cmp addr[6:3] = 4'b0010
//   $20-$23 PBX           addr[6:1] = 6'b010000
//   $24-$27 CDFLAG        addr[6:1] = 6'b010010
wire rd_addr_interesting =
       (addr_d[6:2] == 5'b00001)    // $04-$07 INTREQ
    || (addr_d[6:2] == 5'b00010)    // $08-$0B INTENA
    || (addr_d[6:3] == 4'b0010 )    // $10-$1F TX/RX inx/cmp
    || (addr_d[6:1] == 6'b010000)   // $20-$21 PBX (high)
    || (addr_d[6:1] == 6'b010001)   // $22-$23 PBX (low/unused)
    || (addr_d[6:1] == 6'b010010)   // $24-$25 CDFLAG (high)
    || (addr_d[6:1] == 6'b010011);  // $26-$27 CDFLAG (low)

always @(posedge clk) begin
	sel_d  <= sel;
	rd_d   <= rd;
	wr_d   <= wr;
	addr_d <= addr;
	din_d  <= din;
	if (sel) dout_d <= dout;  // sample-and-hold during the actual read

	// Capture writes always; capture filtered reads only when armed.
	// Mutual-exclusive `else if` keeps a single BRAM write port -> M10K stays.
	// Byte order on drain (LSB first):
	//   byte 0 = {wrnrd, addr[6:0]}     (bit 7: 1=write, 0=read)
	//   byte 1 = data[7:0]
	//   byte 2 = data[15:8]
	//   byte 3 = 0xFF (valid) / 0x00 (ring empty)
	if (sel_d && wr_d) begin
		ring[wr_ptr] <= {8'hFF, din_d, 1'b1, addr_d};
		wr_ptr       <= wr_ptr + 1'b1;
	end else if (sel_d && rd_d && rd_filter_en && rd_arm
	             && rd_addr_interesting) begin
		ring[wr_ptr] <= {8'hFF, dout_d, 1'b0, addr_d};
		wr_ptr       <= wr_ptr + 1'b1;
	end

	if (reset) begin
		wr_ptr <= 0;
	end
end

// Drain side: count bytes within an entry, advance rd_ptr after byte 3.
reg [1:0] byte_idx;

always @(*) begin
	if (empty) begin
		uio_dout = 8'h00;  // valid bit clear -> userspace stops
	end else begin
		case (byte_idx)
			2'd0: uio_dout = ring[rd_ptr][7:0];    // addr | rdwr
			2'd1: uio_dout = ring[rd_ptr][15:8];   // data low
			2'd2: uio_dout = ring[rd_ptr][23:16];  // data high
			2'd3: uio_dout = ring[rd_ptr][31:24];  // valid marker (0xFF)
		endcase
	end
end

always @(posedge clk) begin
	if (reset) begin
		byte_idx <= 0;
		rd_ptr   <= 0;
	end
	else if (uio_cs_trace && uio_rd) begin
		if (!empty) begin
			if (byte_idx == 2'd3) begin
				rd_ptr   <= rd_ptr + 1'b1;
				byte_idx <= 0;
			end else begin
				byte_idx <= byte_idx + 1'b1;
			end
		end
	end
	else if (!uio_cs_trace) begin
		// Reset byte index outside a transaction so a partial read doesn't
		// leave us mid-entry.
		byte_idx <= 0;
	end
end

endmodule
