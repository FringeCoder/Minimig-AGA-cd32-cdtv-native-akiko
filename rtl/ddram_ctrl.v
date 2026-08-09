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
	// D-cache software toggle: gates dtag matches independently
	// of cpu_cache_ctrl[0]. See cpu_cache_new.cc_den.
	input             dcache_sw_en,

	// DDR3    
	output            DDRAM_CLK,
	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output     [28:0] DDRAM_ADDR,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY,
	output            DDRAM_RD,
	output     [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	output            DDRAM_WE,

	// Second memory port, shared onto the same DDR3 interface. Used by the
	// A2065 Ethernet card, which touches DDR3 rarely; the CPU's fast RAM has
	// priority over it and is never made to wait.
	input      [28:0] mem2_address,
	input       [7:0] mem2_burstcount,
	input             mem2_read,
	output     [63:0] mem2_readdata,
	output            mem2_readdatavalid,
	input      [63:0] mem2_writedata,
	input       [7:0] mem2_byteenable,
	input             mem2_write,
	output            mem2_waitrequest,

	// Save state writer. Takes over master 0 of the DDR3 arbiter while
	// ss_freeze is asserted. Master 0 is the fast-RAM path, which cannot
	// issue anything then because the CPU is parked -- the same argument
	// that lets ss_dma borrow the SDRAM CPU port. The arbiter itself is not
	// touched.
	input             ss_freeze,
	input      [28:0] ss_address,
	input      [63:0] ss_writedata,
	input       [7:0] ss_byteenable,
	input             ss_write,
	output            ss_waitrequest,
	// High when no master-0 read is outstanding and nothing is queued, i.e.
	// when it is safe to take the port away. See ss_rd_outstanding below.
	output            ss_ram_idle,

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

	// Bridge (Akiko / CDTV) DMA port. Single-byte transfers arrive
	// as 16-bit word + UDS/LDS so the same address mapping the CPU uses
	// (in cpu_wrapper.v) routes them to the right DDR3 row. dmaCS is held
	// high until dmaACK pulses; one transfer per CS edge.
	// z2-read-fix: dmaWE is no longer hardwired 1 at chipdma_arb.
	// dmaWE=1 → write (PBX sector data into Z2/Z3); dmaWE=0 → read
	// (Akiko TX command fetch from Z2/Z3, returned via dmaRD). Without the
	// read path the CD32 BIOS hangs when it allocates the Akiko CMD block
	// in Z2 (largest free pool) — TX reads silently dropped.
	input      [28:1] dmaAddr,
	input             dmaCS,
	input             dmaWE,
	input             dmaL,
	input             dmaU,
	input      [15:0] dmaWR,
	output reg [15:0] dmaRD,
	output            dmaACK
);

wire ramsel = cpuCS & (~&cpustate | ~cpuU | ~cpuL);

wire cache_hit;
wire cache_req;
reg  cache_fill;
wire cache_ack;

// Snoop pulse + latched address/data for the cache.
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
	.dcache_sw_en     (dcache_sw_en),           // D-cache software toggle
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
	.snoop_act        (dma_snoop_act),          // bridge DMA write pulse
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
// Bridge DMA write buffer + ack handshake with proper CDC.
//
// chipdma_arb runs on clk_sys (28.4 MHz); we run on sysclk = clk_114
// (113.5 MHz). dmaCS / dmaAddr / dmaWR / dmaL / dmaU all come from
// REGISTERED outputs in chipdma_arb that stay stable for the entire
// request (arm_now → S_ACK). We synchronize dmaCS through a 2-FF chain,
// detect the rising edge in our domain, and on that edge latch the data
// (which has been valid for many sysclk cycles by then). SDC false_paths
// the data lines so Quartus is free to place them wherever it wants.
//
// After committing the DDR write, dmaACK_r goes high and STAYS high
// until chipdma_arb drops dmaCS — chipdma_arb's own 2-FF sync on dmaACK
// gives it time to latch the level safely. We release dmaACK when our
// synchronized view of dmaCS goes low.
// -------------------------------------------------------------------------
reg dmaCS_sync1;
reg dmaCS_sync2;
reg dmaCS_sync3;
always @ (posedge sysclk) begin
	if (~reset_n) begin
		dmaCS_sync1 <= 0;
		dmaCS_sync2 <= 0;
		dmaCS_sync3 <= 0;
	end else begin
		dmaCS_sync1 <= dmaCS;
		dmaCS_sync2 <= dmaCS_sync1;
		dmaCS_sync3 <= dmaCS_sync2;
	end
end
wire dmaCS_rise = dmaCS_sync2 & ~dmaCS_sync3;

reg        dma_write_req;
reg        dma_write_ack;
reg [28:1] dmaWriteAddr;
reg [15:0] dmaWriteDat;
reg  [1:0] dmaWriteBE;
reg        dmaACK_r;

// z2-read-fix: bridge READ request bookkeeping. Mirrors
// dma_write_req/_ack but with no snoop pulse (reads don't change cache
// contents). The captured 16-bit word at dma_read_done time is registered
// in dmaRD and returned to chipdma_arb.
reg        dma_read_req;
reg        dma_read_ack;
reg [28:1] dmaReadAddr;
reg  [1:0] dmaReadBA;       // word selector inside the 64-bit DDR beat
reg        dma_read_in_flight; // 1 = DDR read issued, waiting for DOUT_READY

assign dmaACK = dmaACK_r;

always @ (posedge sysclk) begin
	dma_snoop_act <= 0;

	if (~reset_n) begin
		dma_write_req <= 0;
		dma_read_req  <= 0;
		dmaACK_r      <= 0;
	end else begin
		// Latch a new request on the synchronized CS rising edge.
		// Data has been stable in chipdma_arb's registers since arm_now,
		// many sysclk cycles ago, so sampling here is safe regardless of
		// where Quartus placed the data lines.
		// z2-read-fix: split write vs read paths on the same
		// latching edge. dmaWE is the latched (and stable) WE bit.
		if (dmaCS_rise & ~dma_write_req & ~dma_read_req & ~dmaACK_r) begin
			if (dmaWE) begin
			dmaWriteAddr  <= dmaAddr;
			dmaWriteDat   <= dmaWR;
			dmaWriteBE    <= ~{dmaU, dmaL};
			dma_write_req <= 1'b1;
				// Snoop pulse — same data going to DDR. cpu_cache_new
				// will update any cached line covering this address
				// with the new value (write-through coherency).
			dma_snoop_act <= 1'b1;
			dma_snoop_adr <= dmaAddr;
			dma_snoop_dat <= dmaWR;
			dma_snoop_bs  <= ~{dmaU, dmaL};
			end else begin
				dmaReadAddr  <= dmaAddr;
				dma_read_req <= 1'b1;
			end
		end

		// DDR FSM has committed the write — clear req, raise ack to bridge.
		if (dma_write_ack) begin
			dma_write_req <= 1'b0;
			dmaACK_r      <= 1'b1;
		end

		// DDR FSM has captured the read word into dmaRD — clear req,
		// raise ack to bridge so chipdma_arb's S_DRIVE samples dmaRD.
		if (dma_read_ack) begin
			dma_read_req <= 1'b0;
			dmaACK_r     <= 1'b1;
		end

		// Bridge dropped its CS (seen via the sync chain) — release ack
		// so the next request can latch on the next dmaCS_rise.
		if (~dmaCS_sync2) dmaACK_r <= 1'b0;
	end
end

assign DDRAM_CLK = sysclk;

// Fast RAM's own view of DDR3. It goes through the arbiter below rather than
// straight to the pins, so that the second port can share the interface.
reg  [28:0] ram_addr;
reg  [63:0] ram_din;
reg   [7:0] ram_be;
reg         ram_rd, ram_we;
wire        ram_busy;
wire [63:0] ram_dout       = DDRAM_DOUT;
wire        ram_dout_ready;
wire        m0_waitrequest_int;

// Holding ram_busy high for the whole freeze is what protects a request
// that was asserted but not yet accepted when the port was taken away:
// the state machine below only clears ram_rd/ram_we under ~ram_busy, so a
// pending request is held and re-issued once the freeze lifts rather than
// being dropped on the floor. It also stalls the fast-RAM path outright if
// a future change makes it ask for something mid-save.
assign ram_busy       = ss_freeze ? 1'b1               : m0_waitrequest_int;
assign ss_waitrequest = ss_freeze ? m0_waitrequest_int : 1'b1;

// A read that has already been accepted by the arbiter is a different
// problem: its readdatavalid comes back out of band, state 1 below waits
// for it under ~ram_busy, and ram_busy is pinned high while frozen -- so a
// pulse arriving mid-freeze would be lost and the reader (a CPU cache fill,
// or an Akiko/CDTV bridge read, which the quiesce's blitter/disk/audio
// conditions say nothing about) would wait for it forever. Rather than
// change the state machine, the freeze simply waits for the port to be
// quiet: ss_ram_idle joins cpu_boundary in Minimig.sv, the same way
// blit_busy does.
reg ss_rd_outstanding;
always @(posedge sysclk) begin
	if (~reset_n)                            ss_rd_outstanding <= 1'b0;
	else if (ram_rd & ~m0_waitrequest_int)   ss_rd_outstanding <= 1'b1;
	else if (ram_dout_ready)                 ss_rd_outstanding <= 1'b0;
end
assign ss_ram_idle = ~ss_rd_outstanding & ~ram_rd & ~ram_we;

a2065_ddram_arbiter arbiter
(
	.clk             (sysclk),
	.rst             (~reset_n),

	.m0_address      (ss_freeze ? ss_address    : ram_addr),
	.m0_burstcount   (8'd1),
	.m0_read         (ss_freeze ? 1'b0          : ram_rd),
	.m0_readdata     (),
	.m0_readdatavalid(ram_dout_ready),
	.m0_writedata    (ss_freeze ? ss_writedata  : ram_din),
	.m0_byteenable   (ss_freeze ? ss_byteenable : ram_be),
	.m0_write        (ss_freeze ? ss_write      : ram_we),
	.m0_waitrequest  (m0_waitrequest_int),

	.m1_address      (mem2_address),
	.m1_burstcount   (mem2_burstcount),
	.m1_read         (mem2_read),
	.m1_readdata     (),
	.m1_readdatavalid(mem2_readdatavalid),
	.m1_writedata    (mem2_writedata),
	.m1_byteenable   (mem2_byteenable),
	.m1_write        (mem2_write),
	.m1_waitrequest  (mem2_waitrequest),

	.s_address       (DDRAM_ADDR),
	.s_burstcount    (DDRAM_BURSTCNT),
	.s_read          (DDRAM_RD),
	.s_readdata      (DDRAM_DOUT),
	.s_readdatavalid (DDRAM_DOUT_READY),
	.s_writedata     (DDRAM_DIN),
	.s_byteenable    (DDRAM_BE),
	.s_write         (DDRAM_WE),
	.s_waitrequest   (DDRAM_BUSY)
);

assign mem2_readdata = DDRAM_DOUT;

reg        ddr_swap;
reg [15:0] ddr_data;

always @ (posedge sysclk) begin
	reg  [2:0] state = 0;
	reg  [1:0] ba;
	reg [63:0] dout;

	cache_fill <= 0;
	ddr_data <= dout[{ba, 4'b0000} +:16];

	if(~ram_busy) begin
		ram_we  <= 0;
		ram_rd  <= 0;
	end

	if(~reset_n) begin
		state         <= 0;
		write_ack     <= 0;
		dma_write_ack <= 0;
		dma_read_ack       <= 0;
		dma_read_in_flight <= 0;
	end
	else begin
		case(state)
			0: if(~ram_busy) begin
					// Bridge DMA write has priority over CPU. Bandwidth is
					// tiny (one byte per ~150 KB/s sector), so this never
					// starves the CPU in practice, but it does guarantee no
					// head-of-line wait behind a slow CPU cache fill.
					if(~dma_write_ack & dma_write_req) begin
						ram_addr      <= {3'b001, dmaWriteAddr[28:3]};
						ram_be        <= {6'b000000,dmaWriteBE}<<{dmaWriteAddr[2:1],1'b0};
						ram_din       <= {dmaWriteDat,dmaWriteDat,dmaWriteDat,dmaWriteDat};
						ram_we        <= 1;
						dma_write_ack <= 1;
					end
					// Bridge DMA read shares the state-1 read-return shape
					// used by CPU cache fills. Priority below dma_write so a
					// write already in flight completes first; above CPU
					// write/cache to keep bridge latency low, since the BIOS
					// waits synchronously for this.
					else if(~dma_read_ack & dma_read_req & ~dma_read_in_flight) begin
						ram_addr           <= {3'b001, dmaReadAddr[28:3]};
						ram_be             <= 8'hFF;
						ram_rd             <= 1;
						dmaReadBA          <= dmaReadAddr[2:1];
						dma_read_in_flight <= 1;
						state              <= 1;
					end
					else if(~write_ack & write_req) begin
						ram_addr <= {3'b001, writeAddr[28:3]};
						ram_be   <= {6'b000000,writeBE}<<{writeAddr[2:1],1'b0};
						ram_din  <= {writeDat,writeDat,writeDat,writeDat};
						ram_we   <= 1;
						write_ack  <= 1;
					end
					else if(cache_req) begin
						ram_addr <= {3'b001, cpuAddr[28:3]};
						ram_be   <= 8'hFF;
						ram_rd   <= 1;
						ba         <= cpuAddr[2:1];
						state      <= 1;
						ddr_swap   <= ramshared;
					end
				end
			1: if(~ram_busy & ram_dout_ready) begin
					// Distinguish a bridge-DMA read (single 16-bit word, no
					// cache_fill) from a CPU cache fill (4-beat burst into
					// cpu_cache_new). dma_read_in_flight was set at state-0
					// issue time for DMA reads.
					if (dma_read_in_flight) begin
						dmaRD              <= ram_dout[{dmaReadBA, 4'b0000} +:16];
						dma_read_ack       <= 1;
						dma_read_in_flight <= 0;
						state              <= 0;
					end else begin
					ddr_data      <= ram_dout[{ba, 4'b0000} +:16];
					dout          <= ram_dout;
					cache_fill    <= 1;
					ba            <= ba + 1'd1;
					state         <= state + 1'd1;
				end
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
		if(~dma_read_req)  dma_read_ack  <= 0;
	end
end

endmodule
