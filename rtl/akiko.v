// Copyright 2021 Alexey Melnikov
// Copyright 2026 (CD32 native-mode register/IRQ extensions)
//
// This file is part of Minimig
//
// Minimig is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// Minimig is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http:// www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------------
//
// Akiko CPU-bus slave at $B80000-$B8003F.
//
// Always present (any mode):
//   - $00-$03 ID  ($C0CACAFE — Kickstart only checks $CAFE at $B80002.W)
//   - $38-$3B C2P chunky-to-planar register
//
// Gated by parameter NATIVE_CD32 (default 0 — A1200 behavior unchanged):
//   - INTREQ/INTENA, CONFIG, DMA base registers, TX/RX/PBX index regs,
//     PIO byte, NVRAM I2C byte stubs, and akiko_irq output.
//
// Register semantics, masks, and read-mirror behavior are derived from
// research/repos/WinUAE/akiko.cpp (akiko_bget2/akiko_bput2). Citations
// in comments below reference WinUAE line numbers at SHA 2c7f8581.
//
// Notes worth remembering for later milestones (M2+):
//   * The cmd/status DMA sub-buffer offsets are documented inconsistently
//     in WinUAE: the top-of-file comment claims base+0x000 is the command
//     buffer, but the executable code (akiko.cpp:1939-1941) uses
//       cdrx_address  = base | 0x000  // drive -> memory (response/status)
//       cdtx_address  = base | 0x200  // memory -> drive (command)
//       subcode_addr  = base | 0x100
//     The code is the truth. M1 only stores the masked base; M2 must use
//     these offsets when implementing TX/RX DMA.
//   * The "data DMA base must be 64K aligned" comment disagrees with the
//     code mask 0x00fff000 (4K alignment). Code wins; we use 4K.
//
//----------------------------------------------------------------------------------

module akiko #(parameter NATIVE_CD32 = 0)
(
	input             clk,
	input             reset,
	input             cs,
	input             rd,
	input             wr,        // ~rnw & (lds|uds), pre-OR'd at fastchip
	input             lds,       // active high, lower byte (odd byte address)
	input             uds,       // active high, upper byte (even byte address)
	input       [5:1] addr,
	input      [15:0] din,
	output reg [15:0] dout,
	output            akiko_irq
);

// -----------------------------------------------------------------------
// WinUAE-derived constants (akiko.cpp at SHA 2c7f8581)
// -----------------------------------------------------------------------
localparam [31:0] INTENA_MASK     = 32'hff000000; // bput line 1924
localparam [31:0] CONFIG_MASK     = 32'hff800000; // bput line 1986
localparam [31:0] ADDRDATA_MASK   = 32'h00fff000; // bput line 1931 (4K-align)
localparam [31:0] ADDRMISC_MASK   = 32'h00fffc00; // bput line 1938 (1K-align)

localparam [31:0] CDINT_SUBCODE   = 32'h80000000; // bit 31
localparam [31:0] CDINT_DRIVEXMIT = 32'h40000000; // bit 30 (PIO)
localparam [31:0] CDINT_DRIVERECV = 32'h20000000; // bit 29 (PIO)
localparam [31:0] CDINT_RXDMADONE = 32'h10000000; // bit 28
localparam [31:0] CDINT_TXDMADONE = 32'h08000000; // bit 27
localparam [31:0] CDINT_PBX       = 32'h04000000; // bit 26
localparam [31:0] CDINT_OVERFLOW  = 32'h02000000; // bit 25

localparam        CDFLAG_PBX_BIT    = 27; // CONFIG bit 27 (data DMA enable)
localparam        CDFLAG_ENABLE_BIT = 26; // CONFIG bit 26 (CD interface enable)

// -----------------------------------------------------------------------
// Existing C2P logic (preserved bit-equivalent to legacy akiko.v)
// -----------------------------------------------------------------------
wire c2p_sel = (addr[5:2] == 'b1110);

reg [7:0] buff[32];
reg [3:0] rptr = 0, wptr = 0;

always @(posedge clk) begin
	if((wr|rd) & cs & c2p_sel) begin
		if (wr) begin
			rptr <= 0;
			wptr <= wptr + 1'd1;
			{buff[{wptr,1'b0}],buff[{wptr,1'b1}]} <= din;
		end
		else begin
			wptr <= 0;
			rptr <= rptr + 1'd1;
		end
	end
end

reg [15:0] c2p_dout;
always @(*) begin : c2p_read
	reg [4:0] i;
	c2p_dout = 16'h0;
	for (i=0; i<16; i=i+1'd1)
		c2p_dout[i] = buff[{rptr[0],~i[3:0]}][rptr[3:1]];
end

// -----------------------------------------------------------------------
// CD register block — only synthesized when NATIVE_CD32 = 1
// -----------------------------------------------------------------------
wire [15:0] cd_dout;
wire        cd_irq;

generate
if (NATIVE_CD32) begin : g_cd

	reg [31:0] cdrom_intreq;
	reg [31:0] cdrom_intena;
	reg [31:0] cdrom_addressdata;
	reg [31:0] cdrom_addressmisc;
	reg [31:0] cdrom_flags;          // CONFIG
	reg [15:0] cdrom_pbx;
	reg  [7:0] cdrom_subcodeoffset;
	reg  [7:0] cdcomtxinx;           // current TX index (live status)
	reg  [7:0] cdcomrxinx;           // current RX index
	reg  [7:0] cdcomtxcmp;           // TX end (compare)
	reg  [7:0] cdcomrxcmp;           // RX end
	reg  [7:0] pio_byte;             // last PIO write — M1 stub
	reg  [7:0] nvram_io;             // $30 — M1 stub
	reg  [7:0] nvram_dir;            // $32 — M1 stub

	wire write = wr & cs;

	always @(posedge clk) begin
		if (reset) begin
			cdrom_intreq        <= 32'h0;
			cdrom_intena        <= 32'h0;
			cdrom_addressdata   <= 32'h0;
			cdrom_addressmisc   <= 32'h0;
			cdrom_flags         <= 32'h0;
			cdrom_pbx           <= 16'h0;
			cdrom_subcodeoffset <= 8'h0;
			cdcomtxinx          <= 8'h0;
			cdcomrxinx          <= 8'h0;
			cdcomtxcmp          <= 8'h0;
			cdcomrxcmp          <= 8'h0;
			pio_byte            <= 8'h0;
			nvram_io            <= 8'h0;
			nvram_dir           <= 8'h0;
		end else if (write) begin
			case (addr)
				// $08-$09 = INTENA bytes 0-1 (only byte 0 survives mask)
				5'b00100: begin : intena_hi
					reg [31:0] tmp;
					tmp = cdrom_intena;
					if (uds) tmp[31:24] = din[15:8];
					if (lds) tmp[23:16] = din[7:0];
					cdrom_intena <= tmp & INTENA_MASK;
				end
				// $0A-$0B = INTENA bytes 2-3 (always masked to zero)
				5'b00101: begin : intena_lo
					reg [31:0] tmp;
					tmp = cdrom_intena;
					if (uds) tmp[15:8] = din[15:8];
					if (lds) tmp[7:0]  = din[7:0];
					cdrom_intena <= tmp & INTENA_MASK;
				end
				// $10-$11 / $12-$13 = data DMA base (mask 0x00fff000)
				5'b01000: begin : addrdata_hi
					reg [31:0] tmp;
					tmp = cdrom_addressdata;
					if (uds) tmp[31:24] = din[15:8];
					if (lds) tmp[23:16] = din[7:0];
					cdrom_addressdata <= tmp & ADDRDATA_MASK;
				end
				5'b01001: begin : addrdata_lo
					reg [31:0] tmp;
					tmp = cdrom_addressdata;
					if (uds) tmp[15:8] = din[15:8];
					if (lds) tmp[7:0]  = din[7:0];
					cdrom_addressdata <= tmp & ADDRDATA_MASK;
				end
				// $14-$15 / $16-$17 = misc DMA base (mask 0x00fffc00)
				5'b01010: begin : addrmisc_hi
					reg [31:0] tmp;
					tmp = cdrom_addressmisc;
					if (uds) tmp[31:24] = din[15:8];
					if (lds) tmp[23:16] = din[7:0];
					cdrom_addressmisc <= tmp & ADDRMISC_MASK;
				end
				5'b01011: begin : addrmisc_lo
					reg [31:0] tmp;
					tmp = cdrom_addressmisc;
					if (uds) tmp[15:8] = din[15:8];
					if (lds) tmp[7:0]  = din[7:0];
					cdrom_addressmisc <= tmp & ADDRMISC_MASK;
				end
				// $18 byte write = clear SUBCODE IRQ (akiko.cpp:1943-1945)
				5'b01100: begin
					if (uds) cdrom_intreq <= cdrom_intreq & ~CDINT_SUBCODE;
				end
				// $1D byte write = TX compare; clears TXDMADONE IRQ (akiko.cpp:1946-1950)
				5'b01110: begin
					if (lds) begin
						cdcomtxcmp   <= din[7:0];
						cdrom_intreq <= cdrom_intreq & ~CDINT_TXDMADONE;
					end
				end
				// $1F byte write = RX compare; clears RXDMADONE IRQ (akiko.cpp:1951-1955)
				5'b01111: begin
					if (lds) begin
						cdcomrxcmp   <= din[7:0];
						cdrom_intreq <= cdrom_intreq & ~CDINT_RXDMADONE;
					end
				end
				// $20-$21 PBX, set-only OR semantics; clears PBX IRQ (akiko.cpp:1956-1966)
				// PBX register is forced to zero if CONFIG.PBX disabled.
				5'b10000: begin : pbx_w
					reg [15:0] tmp;
					tmp = cdrom_pbx;
					if (uds) tmp[15:8] = tmp[15:8] | din[15:8];
					if (lds) tmp[7:0]  = tmp[7:0]  | din[7:0];
					if (!cdrom_flags[CDFLAG_PBX_BIT]) tmp = 16'h0;
					cdrom_pbx    <= tmp;
					cdrom_intreq <= cdrom_intreq & ~CDINT_PBX;
				end
				// $24-$27 CONFIG (mask 0xff800000), with ENABLE 0->1 clears OVERFLOW IRQ,
				// PBX off clears pbx register (akiko.cpp:1967-1987)
				5'b10010: begin : cfg_high
					reg [31:0] new_flags;
					new_flags = cdrom_flags;
					if (uds) new_flags[31:24] = din[15:8];
					if (lds) new_flags[23:16] = din[7:0];
					new_flags = new_flags & CONFIG_MASK;
					cdrom_flags <= new_flags;
					if (new_flags[CDFLAG_ENABLE_BIT] && !cdrom_flags[CDFLAG_ENABLE_BIT])
						cdrom_intreq <= cdrom_intreq & ~CDINT_OVERFLOW;
					if (!new_flags[CDFLAG_PBX_BIT]) cdrom_pbx <= 16'h0;
				end
				5'b10011: begin : cfg_low
					reg [31:0] new_flags;
					new_flags = cdrom_flags;
					if (uds) new_flags[15:8] = din[15:8];
					if (lds) new_flags[7:0]  = din[7:0];
					new_flags = new_flags & CONFIG_MASK;
					cdrom_flags <= new_flags;
					// ENABLE/PBX bits are above 23, so the side effects here
					// can never trigger via this address pair, but model the
					// mask anyway so flags stays clean.
				end
				// $28 PIO byte write — uds (M1 stub: just latch + clear IRQ)
				5'b10100: begin
					if (uds) begin
						pio_byte     <= din[15:8];
						cdrom_intreq <= cdrom_intreq & ~CDINT_DRIVEXMIT;
					end
				end
				// $30 / $32 NVRAM I2C — M1 stub
				5'b11000: begin
					if (uds) nvram_io  <= din[15:8];
					if (lds) ;
				end
				5'b11001: begin
					if (uds) nvram_dir <= din[15:8];
				end
				default: ;
			endcase
		end
	end

	// Read mux
	reg [15:0] cd_dout_r;
	always @(*) begin
		cd_dout_r = 16'h0;
		case (addr)
			// $04-$05 INTREQ high half
			5'b00010: cd_dout_r = cdrom_intreq[31:16];
			// $06-$07 INTREQ low half
			5'b00011: cd_dout_r = cdrom_intreq[15:0];
			// $08-$09 INTENA high half
			5'b00100: cd_dout_r = cdrom_intena[31:16];
			// $0A-$0B INTENA low half
			5'b00101: cd_dout_r = cdrom_intena[15:0];
			// $0C-$0F INTENA mirror (akiko.cpp:1747-1753)
			5'b00110: cd_dout_r = cdrom_intena[31:16];
			5'b00111: cd_dout_r = cdrom_intena[15:0];
			// $10-$1F: read-mirror block — addr[1] selects the pair
			//   addr[1]=0 ($10/$14/$18/$1C bytes 0,1) -> {subcodeoffset, txinx}
			//   addr[1]=1 ($12/$16/$1A/$1E bytes 2,3) -> {rxinx, 0}
			// (akiko.cpp:1755-1783)
			5'b01000, 5'b01010, 5'b01100, 5'b01110:
				cd_dout_r = {cdrom_subcodeoffset, cdcomtxinx};
			5'b01001, 5'b01011, 5'b01101, 5'b01111:
				cd_dout_r = {cdcomrxinx, 8'h0};
			// $20-$21 PBX (akiko.cpp:1785-1788)
			5'b10000: cd_dout_r = cdrom_pbx;
			// $24-$27 CONFIG (akiko.cpp:1789-1794)
			5'b10010: cd_dout_r = cdrom_flags[31:16];
			5'b10011: cd_dout_r = cdrom_flags[15:0];
			// $28 PIO byte read — M1 stub returns last write in upper byte
			5'b10100: cd_dout_r = {pio_byte, 8'h0};
			// $30/$32 NVRAM I2C — M1 stub
			5'b11000: cd_dout_r = {nvram_io,  8'h0};
			5'b11001: cd_dout_r = {nvram_dir, 8'h0};
			default:  cd_dout_r = 16'h0;
		endcase
	end

	assign cd_dout = cs ? cd_dout_r : 16'h0;
	assign cd_irq  = |(cdrom_intreq[31:25] & cdrom_intena[31:25]);

end else begin : g_stub
	assign cd_dout = 16'h0;
	assign cd_irq  = 1'b0;
end
endgenerate

// -----------------------------------------------------------------------
// Combined dout (ID + C2P + CD regs). Each contributor is zero outside
// its address range, so a simple OR is safe (matches WinUAE bget2 default
// of returning 0 for unknown addresses).
// -----------------------------------------------------------------------
always @(*) begin
	dout = 16'h0;
	if (cs) begin
		if (addr == 5'd0) dout = 16'hC0CA;
		if (addr == 5'd1) dout = 16'hCAFE;
		if (c2p_sel)      dout = c2p_dout;
	end
	dout = dout | cd_dout;
end

assign akiko_irq = cd_irq;

endmodule
