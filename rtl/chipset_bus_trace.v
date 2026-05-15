// Chipset bus trace — 1024-deep ring buffer of Agnus register writes.
//
// Compile-time gated by CHIPSET_TRACE param in agnus.v. When the gate is 0,
// the instantiation lives inside a `generate if` block that Quartus
// dead-code-eliminates, producing a bit-identical RBF to the no-trace build.
//
// One entry = 64 bits, drained as 8 bytes LSB-first over a dedicated UIO
// sub-channel. Layout (bit numbering MSB-first as packed into the entry):
//
//   [63:48] data[15:0]
//   [47:40] reg_addr[7:0]   (i.e., reg_address_out[8:1])
//   [39:37] src[2:0]        (000=CPU, 001=cop, 010=blt, 011=spr,
//                             100=bpl, 101=dsk, 110=aud, 111=refresh)
//   [36:26] vpos[10:0]
//   [25:17] hpos[8:0]
//   [16]    dbwe (true write to chip RAM, only valid for CPU/blt sources)
//   [15:0]  reserved (0); future seq/parity
//
// Drain protocol (matches akiko_bus_trace pattern):
//   byte 0: data[7:0]
//   byte 1: data[15:8]
//   byte 2: reg_addr (8 bits, includes [7:0] of reg_address)
//   byte 3: {dbwe, src[2:0], vpos[10:8]} - high context byte
//   byte 4: vpos[7:0]
//   byte 5: hpos[7:0]
//   byte 6: {7'b0, hpos[8]}
//   byte 7: 0xFF (valid) or 0x00 (empty -> userspace stops)
//
// Capture trigger (`write_strobe`): the user wires this from a filtered
// combinational block (see chipset-trace-plan.md). The module itself does
// no filtering; it just commits to the ring when write_strobe is high.

module chipset_bus_trace
(
	input             clk,
	input             reset,

	// Snoop inputs
	input             write_strobe,   // 1-cycle pulse: commit one entry
	input       [7:0] reg_addr,       // chipset register address (word index)
	input      [15:0] data,           // value being written
	input       [2:0] src,            // bus source id (see header)
	input      [10:0] vpos,           // vertical beam position
	input       [8:0] hpos,           // horizontal beam position
	input             dbwe,           // chip-RAM write enable (Agnus.dbwe)

	// UIO drain port — pulses uio_rd to advance one byte per access.
	input             uio_cs_trace,
	input             uio_rd,
	output reg  [7:0] uio_dout
);

// 1024-entry ring, 10-bit pointers.
reg [63:0] ring [0:1023];
reg  [9:0] wr_ptr;
reg  [9:0] rd_ptr;
wire       empty = (wr_ptr == rd_ptr);

// Capture: commit one entry per pulse of write_strobe.
always @(posedge clk) begin
	if (write_strobe) begin
		ring[wr_ptr] <= {data,                  // [63:48]
		                 reg_addr,              // [47:40]
		                 src,                   // [39:37]
		                 vpos,                  // [36:26]
		                 hpos,                  // [25:17]
		                 dbwe,                  // [16]
		                 16'h0000};             // [15:0] reserved
		wr_ptr <= wr_ptr + 1'b1;
	end

	if (reset) begin
		wr_ptr <= 0;
	end
end

// Drain: byte_idx walks 0..7 across the entry; rd_ptr advances after byte 7.
reg [2:0] byte_idx;

always @(*) begin
	if (empty) begin
		uio_dout = 8'h00;
	end else begin
		case (byte_idx)
			3'd0: uio_dout = ring[rd_ptr][55:48];                    // data lo
			3'd1: uio_dout = ring[rd_ptr][63:56];                    // data hi
			3'd2: uio_dout = ring[rd_ptr][47:40];                    // reg_addr
			3'd3: uio_dout = {ring[rd_ptr][16],                      // dbwe
			                  ring[rd_ptr][39:37],                   // src[2:0]
			                  1'b0,                                  // pad
			                  ring[rd_ptr][36:34]};                  // vpos[10:8]
			3'd4: uio_dout = ring[rd_ptr][33:26];                    // vpos[7:0]
			3'd5: uio_dout = ring[rd_ptr][24:17];                    // hpos[7:0]
			3'd6: uio_dout = {7'b0, ring[rd_ptr][25]};               // hpos[8]
			3'd7: uio_dout = 8'hFF;                                  // valid sentinel
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
			if (byte_idx == 3'd7) begin
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
