// Akiko bus trace — 32-deep ring buffer of CPU accesses to the Akiko window
// at $B80000-$B8003F. Drained by Main_MiSTer via a new SPI sub-channel
// (akiko_cs && io_din[7], i.e. UIO class 0xF400 with bit 7 set on byte 1).
//
// One trace entry = 32 bits, drained as 4 bytes LSB-first:
//   byte 0: addr[6:0] | rd_or_wr (bit 7)
//   byte 1: data[7:0]
//   byte 2: data[15:8]
//   byte 3: bit 0 = entry valid (0 = ring empty, ignore the rest)
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

	// UIO read port. uio_rd pulses pop one byte (4-byte entries auto-advance).
	input             uio_cs_trace,  // akiko_cs && io_din[7] (new sub-channel)
	input             uio_rd,
	output reg  [7:0] uio_dout
);

// 32-entry ring (5-bit pointers).
reg [31:0] ring [0:31];
reg  [4:0] wr_ptr;
reg  [4:0] rd_ptr;
wire       empty = (wr_ptr == rd_ptr);

// Stage the bus inputs one cycle. `dout_d` in particular breaks the long
// combinational path from akiko's register-file read mux into the ring's
// write port, which was costing ~0.4 ns of setup slack on clk_sys.
reg sel_d, rd_d, wr_d;
reg [6:0] addr_d;
reg [15:0] din_d, dout_d;

always @(posedge clk) begin
	sel_d  <= sel;
	rd_d   <= rd;
	wr_d   <= wr;
	addr_d <= addr;
	din_d  <= din;
	dout_d <= dout;

	// Capture one entry per sel_akiko cycle (any access within the window).
	// Byte order on drain (LSB first):
	//   byte 0 = {wr_bit, addr[6:0]}    (rdwr in bit 7, word addr in bits 6:0)
	//   byte 1 = data[7:0]
	//   byte 2 = data[15:8]
	//   byte 3 = 0xFF (valid) / 0x00 (ring empty)
	if (sel_d && (rd_d || wr_d)) begin
		ring[wr_ptr] <= {8'hFF, wr_d ? din_d : dout_d, wr_d ? 1'b1 : 1'b0, addr_d};
		wr_ptr <= wr_ptr + 1'b1;
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
