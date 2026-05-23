//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
// DDR3 memory interface                                                    // 
// Copyright (c)2019 Alexey Melnikov                                        //
// Based on SDRAM controller by Tobias Gubener                              //
//                                                                          //
// This source file is free software: you can redistribute it and/or modify //
// it under the terms of the GNU General Public License as published        //
// by the Free Software Foundation, either version 3 of the License, or     //
// (at your option) any later version.                                      //
//                                                                          //
// This source file is distributed in the hope that it will be useful,      //
// but WITHOUT ANY WARRANTY; without even the implied warranty of           //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            //
// GNU General Public License for more details.                             //
//                                                                          //
// You should have received a copy of the GNU General Public License        //
// along with this program.  If not, see <http://www.gnu.org/licenses/>.    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////


module ddram_ctrl
(
	// system
	input             sysclk,
	input             reset_n,
	input             cache_rst,
	input             cache_inhibit,
	input       [3:0] cpu_cache_ctrl,

	// DDR3    
	output            DDRAM_CLK,
	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output reg [28:0] DDRAM_ADDR,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY,
	output reg        DDRAM_RD,
	output reg [63:0] DDRAM_DIN,
	output reg  [7:0] DDRAM_BE,
	output reg        DDRAM_WE,

	// cpu
	input      [28:1] cpuAddr,
	input             cpuCS,
	input       [1:0] cpustate,
	input             cpuL,
	input             cpuU,
	input      [15:0] cpuWR,
	output     [15:0] cpuRD,
	input             ramshared,
	output            ramready,

	// Phase B: bridge (Akiko / CDTV) DMA write port. Single-byte writes
	// arrive as 16-bit word + UDS/LDS so the same address mapping the CPU
	// uses (in cpu_wrapper.v) routes them to the right DDR3 row. dmaCS
	// is held high until dmaACK pulses; one write per CS edge.
	// dmaWE is hardwired 1 today (bridge is write-only into Z2/Z3).
	input      [28:1] dmaAddr,
	input             dmaCS,
	input             dmaWE,
	input             dmaL,
	input             dmaU,
	input      [15:0] dmaWR,
	output            dmaACK
);

wire ramsel = cpuCS & (~&cpustate | ~cpuU | ~cpuL);

wire cache_hit;
wire cache_req;
reg  cache_fill;
wire cache_ack;

// Phase B: snoop pulse + latched address/data for the cache.
// Fires for one sysclk when the bridge DMA write is latched into
// dmaWriteAddr/Dat/BE below. cpu_cache_new uses "write-through" snoop —
// updates the cached copy of the line if present, otherwise no-op.
reg        dma_snoop_act;
reg [28:1] dma_snoop_adr;
reg [15:0] dma_snoop_dat;
reg  [1:0] dma_snoop_bs;

cpu_cache_new cpu_cache
(
	.clk              (sysclk),                 // clock
	.rst              (~reset_n | ~cache_rst),  // cache reset
	.cpu_cache_ctrl   (cpu_cache_ctrl),         // CPU cache control
	.cache_inhibit    (cache_inhibit | ramshared), // cache inhibit
	.cpu_cs           (ramsel),                 // cpu activity
	.cpu_adr          (cpuAddr),                // cpu address
	.cpu_bs           (~{cpuU, cpuL}),          // cpu byte selects
	.cpu_we           (cpustate == 3),          // cpu write
	.cpu_ir           (cpustate == 0),          // cpu instruction read
	.cpu_dr           (cpustate == 2),          // cpu data read
	.cpu_dat_w        (cpuWR),                  // cpu write data
	.cpu_dat_r        (cpuRD),                  // cpu read data
	.cpu_ack          (cache_hit),              // cpu acknowledge
	.wb_en            (cache_ack),              // write enable
	.sdr_dat_r        (ddr_swap ? {ddr_data[7:0], ddr_data[15:8]} : ddr_data), // sdram read data
	.sdr_read_req     (cache_req),              // sdram read request from cache
	.sdr_read_ack     (cache_fill),             // sdram read acknowledge to cache
	.snoop_act        (dma_snoop_act),          // Phase B: bridge DMA write pulse
	.snoop_adr        (dma_snoop_adr),
	.snoop_dat_w      (dma_snoop_dat),
	.snoop_bs         (dma_snoop_bs)
);

// write buffer, enables CPU to continue while a write is in progress
reg        write_ena;
reg        write_req;
reg        write_ack;
reg  [1:0] writeBE;
reg [28:1] writeAddr;
reg [15:0] writeDat;

always @ (posedge sysclk) begin
	reg  [1:0] write_state;

	if(~reset_n) begin
		write_req   <= 0;
		write_ena   <= 0;
		write_state <= 0;
	end else begin
		case(write_state)
			default:
				if(ramsel && cpustate == 3) begin
					writeAddr <= cpuAddr;
					writeDat  <= ramshared ? {cpuWR[7:0],cpuWR[15:8]} : cpuWR;
					writeBE   <= ramshared ? ~{cpuL, cpuU} : ~{cpuU, cpuL};
					write_req <= 1;
					if(cache_ack) begin
						write_ena   <= 1;
						write_state <= 1;
					end
				end

			1: if(write_ack) begin
					// The SDRAM controller has picked up the request
					write_req   <= 0;
					write_state <= 2;
				end

			2: if(!write_ack) write_state <= 0;
		endcase
		if(~ramsel) write_ena <= 0;
	end
end

assign ramready = cache_hit || write_ena;

// -------------------------------------------------------------------------
// Phase B: bridge DMA write buffer + ack handshake.
//
// chipdma_arb asserts dmaCS for the duration of a write request and holds
// dmaAddr/dmaWR/dmaL/dmaU stable. We latch into our own buffer, raise
// dma_write_req for the DDR FSM, and pulse dmaACK when the DDR commit
// happens. The cache snoop fires for one cycle on latch — cpu_cache_new
// will update any cached line covering this address with the new data.
// -------------------------------------------------------------------------
reg        dma_write_req;
reg        dma_write_ack;
reg [28:1] dmaWriteAddr;
reg [15:0] dmaWriteDat;
reg  [1:0] dmaWriteBE;
reg        dmaACK_r;

assign dmaACK = dmaACK_r;

always @ (posedge sysclk) begin
	dma_snoop_act <= 0;

	if (~reset_n) begin
		dma_write_req <= 0;
		dmaACK_r      <= 0;
	end else begin
		// Latch a new request when bridge raises CS and we are idle.
		if (dmaCS & dmaWE & ~dma_write_req & ~dmaACK_r) begin
			dmaWriteAddr  <= dmaAddr;
			dmaWriteDat   <= dmaWR;
			dmaWriteBE    <= ~{dmaU, dmaL};
			dma_write_req <= 1'b1;
			// Snoop pulse — same data going to DDR. cpu_cache_new will
			// update any cached line.
			dma_snoop_act <= 1'b1;
			dma_snoop_adr <= dmaAddr;
			dma_snoop_dat <= dmaWR;
			dma_snoop_bs  <= ~{dmaU, dmaL};
		end

		// DDR FSM has committed the write — clear req, pulse ack to bridge.
		if (dma_write_ack) begin
			dma_write_req <= 1'b0;
			dmaACK_r      <= 1'b1;
		end

		// Bridge dropped CS — release ack so the next request can latch.
		if (~dmaCS) dmaACK_r <= 1'b0;
	end
end

assign DDRAM_CLK = sysclk;
assign DDRAM_BURSTCNT = 1;

reg        ddr_swap;
reg [15:0] ddr_data;

always @ (posedge sysclk) begin
	reg  [2:0] state = 0;
	reg  [1:0] ba;
	reg [63:0] dout;

	cache_fill <= 0;
	ddr_data <= dout[{ba, 4'b0000} +:16];

	if(~DDRAM_BUSY) begin
		DDRAM_WE  <= 0;
		DDRAM_RD  <= 0;
	end

	if(~reset_n) begin
		state         <= 0;
		write_ack     <= 0;
		dma_write_ack <= 0;
	end
	else begin
		case(state)
			0: if(~DDRAM_BUSY) begin
					// Phase B: bridge DMA write has priority over CPU.
					// Bandwidth is tiny (one byte per ~150 KB/s sector),
					// so this never starves the CPU in practice, but
					// guarantees no head-of-line wait behind a slow CPU
					// cache fill.
					if(~dma_write_ack & dma_write_req) begin
						DDRAM_ADDR    <= {3'b001, dmaWriteAddr[28:3]};
						DDRAM_BE      <= {6'b000000,dmaWriteBE}<<{dmaWriteAddr[2:1],1'b0};
						DDRAM_DIN     <= {dmaWriteDat,dmaWriteDat,dmaWriteDat,dmaWriteDat};
						DDRAM_WE      <= 1;
						dma_write_ack <= 1;
					end
					else if(~write_ack & write_req) begin
						DDRAM_ADDR <= {3'b001, writeAddr[28:3]};
						DDRAM_BE   <= {6'b000000,writeBE}<<{writeAddr[2:1],1'b0};
						DDRAM_DIN  <= {writeDat,writeDat,writeDat,writeDat};
						DDRAM_WE   <= 1;
						write_ack  <= 1;
					end
					else if(cache_req) begin
						DDRAM_ADDR <= {3'b001, cpuAddr[28:3]};
						DDRAM_BE   <= 8'hFF;
						DDRAM_RD   <= 1;
						ba         <= cpuAddr[2:1];
						state      <= 1;
						ddr_swap   <= ramshared;
					end
				end
			1: if(~DDRAM_BUSY & DDRAM_DOUT_READY) begin
					ddr_data      <= DDRAM_DOUT[{ba, 4'b0000} +:16];
					dout          <= DDRAM_DOUT;
					cache_fill    <= 1;
					ba            <= ba + 1'd1;
					state         <= state + 1'd1;
				end
			2,3: begin
					cache_fill    <= 1;
					ba            <= ba + 1'd1;
					state         <= state + 1'd1;
				end
			4: begin
					cache_fill    <= 1;
					state         <= 0;
				end
		endcase

		if(~write_req) write_ack <= 0;
		if(~dma_write_req) dma_write_ack <= 0;
	end
end

endmodule
