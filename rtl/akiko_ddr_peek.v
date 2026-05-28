// Akiko DDR peek — snoops bridge writes to DDR3 (ram2) and captures
// (ramaddr, data, byte-enables) into a 64-entry ring. Drained by Main_MiSTer
// via UIO sub-channel 0xF420 (akiko_cs && io_din[5]).
//
// Purpose (2026-05-28): verify what the RX engine + chipdma_arb actually
// write to Z2 during the CD32 BIOS hang. Userspace already knows what bytes
// it pushed via UIO_DMA_WRITE; this ring reveals what bytes the bridge then
// commits to DDR3. Mismatch = byte-lane / address / endian bug. Match =
// CPU-side cache coherency bug (snoop pulse missing the line).
//
// Trigger: rising edge of (dma_cs & dma_we) — captures one entry per
// completed bridge write. Edge-triggering avoids ring flood from the level
// signals being asserted for many sysclk cycles per write.
//
// Entry format (8 bytes, LSB-first on drain):
//   byte 0 : {addr[7:1], 1'b0}                    — byte-addr low
//   byte 1 : addr[15:8]
//   byte 2 : addr[23:16]
//   byte 3 : {U, L, 1'b0, addr[28:24]}
//   byte 4 : data[7:0]
//   byte 5 : data[15:8]
//   byte 6 : 0xA5  (sentinel for sanity)
//   byte 7 : 0xFF if entry valid, 0x00 if ring empty (signals userspace to stop)

module akiko_ddr_peek #(parameter CAPTURE_ENABLE = 1)(
	input             clk,
	input             reset,

	// Snoop signals from chipdma_arb / Minimig.sv top.
	input             dma_cs,
	input             dma_we,
	input      [28:1] dma_addr,
	input             dma_l,
	input             dma_u,
	input      [15:0] dma_wr,

	// UIO drain port (one byte per uio_rd while cs_peek is asserted).
	input             uio_cs_peek,
	input             uio_rd,
	output reg  [7:0] uio_dout
);

// 64-entry ring (6-bit pointers). 46 bits payload per entry, padded to 48.
//   ring[15: 0] = data
//   ring[43:16] = addr[28:1]
//   ring[44]    = L
//   ring[45]    = U
//   ring[47:46] = 2'b0 (reserved)
reg [47:0] ring [0:63];
reg  [5:0] wr_ptr;
reg  [5:0] rd_ptr;
wire       empty = (wr_ptr == rd_ptr);

// Rising-edge detect on the bridge write event.
reg  dma_event_d;
wire dma_event = dma_cs & dma_we;

always @(posedge clk) begin
	dma_event_d <= dma_event;

	if (CAPTURE_ENABLE && (dma_event & ~dma_event_d)) begin
		ring[wr_ptr] <= {2'b00, dma_u, dma_l, dma_addr, dma_wr};
		wr_ptr       <= wr_ptr + 1'b1;
	end

	if (reset) begin
		wr_ptr      <= 6'd0;
		dma_event_d <= 1'b0;
	end
end

// Drain side.
reg [2:0] byte_idx;
wire [47:0] cur = ring[rd_ptr];

always @(*) begin
	if (empty) begin
		uio_dout = 8'h00;
	end else begin
		case (byte_idx)
			3'd0: uio_dout = {cur[22:16], 1'b0};               // addr[7:1] padded
			3'd1: uio_dout = cur[30:23];                       // addr[15:8]
			3'd2: uio_dout = cur[38:31];                       // addr[23:16]
			3'd3: uio_dout = {cur[45], cur[44], 1'b0, cur[43:39]}; // U,L,_,addr[28:24]
			3'd4: uio_dout = cur[ 7: 0];                       // data[7:0]
			3'd5: uio_dout = cur[15: 8];                       // data[15:8]
			3'd6: uio_dout = 8'hA5;                            // sentinel
			3'd7: uio_dout = 8'hFF;                            // valid marker
		endcase
	end
end

always @(posedge clk) begin
	if (reset) begin
		byte_idx <= 3'd0;
		rd_ptr   <= 6'd0;
	end
	else if (uio_cs_peek && uio_rd) begin
		if (!empty) begin
			if (byte_idx == 3'd7) begin
				rd_ptr   <= rd_ptr + 1'b1;
				byte_idx <= 3'd0;
			end else begin
				byte_idx <= byte_idx + 1'b1;
			end
		end
	end
	else if (!uio_cs_peek) begin
		byte_idx <= 3'd0;
	end
end

endmodule
