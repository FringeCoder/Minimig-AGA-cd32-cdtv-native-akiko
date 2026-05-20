//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
//                                                                          //
// Copyright (c) 2009-2011 Tobias Gubener                                   //
// Copyright (c) 2017-2019 Alexey Melnikov                                  //
// Subdesign fAMpIGA by TobiFlex                                            //
//                                                                          //
// This is the cpu wrapper to generate 68K Bus signals                      //
// and configure Zorro cards                                                //
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
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//

module cpu_wrapper
(
	input             reset,
	output reg        reset_out,

	input             clk,
	input             ph1,
	input             ph2,

	input       [1:0] cpucfg,
	input       [2:0] fastramcfg,
	input       [3:0] cachecfg,
	input             bootrom,

	output reg [23:1] chip_addr,
	input      [15:0] chip_dout,
	output reg [15:0] chip_din,
	output reg        chip_as,
	output reg        chip_uds,
	output reg        chip_lds,
	output reg        chip_rw,
	input             chip_dtack,
	input       [2:0] chip_ipl,
	
	input      [15:0] fastchip_dout,
	output reg        fastchip_sel,
	output            fastchip_lds,
	output            fastchip_uds,
	output            fastchip_rnw,
	output reg        fastchip_lw,
	input             fastchip_selack,
	input             fastchip_ready,

	output            ramsel,
	output     [28:1] ramaddr,
	output     [15:0] ramdin,
	input      [15:0] ramdout,
	input             ramready,
	output            ramlds,
	output            ramuds,
	output            ramshared,

	output            toccata_ena,
	output reg  [7:0] toccata_base,

	// CDTV mode (M1: autoconfig responder only). When 1, the CDTV DMAC
	// card joins the autoconfig chain ahead of Toccata/Z2 fastram with
	// the WinUAE-canonical ROM bytes from cdtv.cpp cdtv_init.
	input             cdtv_mode,

	// CDTV bridge data path. After autoconfig at $E80000, the BIOS talks
	// to the bridge at $E90000-$E9FFFF (DMAC/TPI/CR-511) and $DC8000-
	// $DCFFFF (NVRAM). Both windows are decoded in gary.v (sel_cdtv +
	// sel_cdtv_nvram) and the bridge / NVRAM module drive cdtv_din +
	// cdtv_selack to short-circuit cpu_din the same cycle (no DTACK wait,
	// same pattern as fastchip_selack / fastchip_dout). Spec section 1.
	input      [15:0] cdtv_din,
	input             cdtv_selack,

	// AC ROM mirror feed — the BIOS reads back the autoconfig identity
	// bytes at $E900-$E93F after relocating the board (spec section 2.3
	// row 1). cdtv_bridge.v provides the byte offset; cpu_wrapper looks
	// up the 8-bit ROM value from its existing ac_rom table.
	input       [5:0] cdtv_ac_rom_addr,
	output      [7:0] cdtv_ac_rom_byte,

	output reg  [1:0] cpustate,
	output reg  [3:0] cacr,
	output reg [31:0] nmi_addr
);

assign ramsel       = cpu_req & ~sel_nmi_vector & (sel_zram | sel_chipram | sel_kickram | sel_dd | sel_rtg);
assign ramshared    = sel_dd;

// NMI
always @(posedge clk) nmi_addr <= vbr + 32'h7c;

wire sel_z3ram0 = (cpu_addr[31:27] == z3ram_base0) && z3ram_ena0;
wire sel_z3ram1 = (cpu_addr[31:28] == z3ram_base1) && z3ram_ena1;
wire sel_z2ram  = !cpu_addr[31:24] && (cpu_addr[23] ^ |cpu_addr[22:21]) && z2ram_ena; // addr[23:21] = 1..4
wire sel_zram   = sel_z3ram0 | sel_z3ram1 | sel_z2ram;
wire sel_dd     = (cpu_addr[31:16] == 16'h00DD) && (cpu_addr[15:13] == 'b010);
wire sel_rtg    = (cpu_addr[31:24] == 8'h02);

// don't sel_kickram when writing
// CD32 mirrors $a8xxxx (mirror of $f8xxxx) and $b0xxxx (mirror of $e0xxxx)
wire sel_kickram   = !cpu_addr[31:24] && (&cpu_addr[23:19] || (cpu_addr[23:19] == 5'b11100) || (cpu_addr[23:19] == 5'b10101) || (cpu_addr[23:19] == 5'b10110)) && ckick && wr;	// $f8xxxx, $e0xxxx, $a8/$b0 CD32 mirrors
wire sel_kicklower = !cpu_addr[31:24] && (cpu_addr[23:18] == 6'b111110);
wire sel_chipram   = !cpu_addr[31:21] && cchip; 		             //$000000 - $1FFFFF

// we route everything hrtmon related through cart.v (needs a couple of signals to
// decide what to do, would not be good style to replicate that here). 
wire sel_nmi_vector = (cpu_addr[31:2] == nmi_addr[31:2]) && (cpustate == 2);

wire [15:0] ramdat;

assign ramlds = sel_rtg ? uds_in : lds_in;
assign ramuds = sel_rtg ? lds_in : uds_in;
assign ramdin = sel_rtg ? {cpu_dout[7:0],cpu_dout[15:8]} : cpu_dout;
assign ramdat = sel_rtg ? {ramdout[7:0], ramdout[15:8]}  : ramdout;

//       Main  DDx  RTG  8M  128M  256M
//       ----  ---  ---  --  ----  ----
//        SDR  DDR  RTG  Z2  Z3_0  Z3_1
// 28      0    0    0   1    0     1
// 27      0    0    0   1    1     X
// 26      0    1    1   0    X     X
// 25-23   0   111  110  0    X     X
// supported configs: SDR + (Z2, Z3_1, Z3_0+Z3_1)

// This is the mapping to the sram
// map 00-1f to 00-1f (chipram), a0-ff to 20-7f. All non-fastram goes into the first
// 8M block(SDRAM). This map should be the same as in minimig_sram_bridge.v 
// All Zorro RAM goes to DDR3
assign ramaddr[28]    = sel_zram & ~sel_z3ram0;
assign ramaddr[27]    = sel_zram & (~sel_z3ram1 | cpu_addr[27]);
assign ramaddr[26:23] = (sel_z3ram0 | sel_z3ram1) ? cpu_addr[26:23]: (sel_rtg ? 4'b1110 : {4{sel_dd}});
assign ramaddr[22:19] = {4{sel_dd}} | cpu_addr[22:19];
assign ramaddr[18]    =    sel_dd   | (sel_kicklower & bootrom) | cpu_addr[18];
assign ramaddr[17:16] = {2{sel_dd}} | cpu_addr[17:16];
assign ramaddr[15:1]  = cpu_addr[15:1];

assign fastchip_lds = lds_in;
assign fastchip_uds = uds_in;
assign fastchip_rnw = wr;

reg  [31:0] cpu_addr;
reg  [15:0] cpu_dout;
// CDTV bridge slots in alongside fastchip in the cpu_din mux. Spec section 1:
// $E90000-$E9FFFF (DMAC/TPI/CR-511) + $DC8000-$DCFFFF (NVRAM) live on the
// chip bus, but the bridge fires cdtv_selack the same cycle as sel so the
// CPU does not wait on chip-bus DTACK — same short-circuit as fastchip uses.
wire [15:0] cpu_din = ramsel ? ramdat :
                      fastchip_selack ? fastchip_dout :
                      cdtv_selack ? cdtv_din :
                      {sel_autoconfig ? autocfg_data : chip_data[15:12], chip_data[11:0]};
reg         wr;
reg         uds_in;
reg         lds_in;
reg  [15:0] chip_data;
reg  [31:0] vbr;

always @* begin
	if(cpucfg[1:0]) begin
		cpu_dout     = cpu_dout_p;
		cpu_addr     = cpu_addr_p;
		cpustate     = cpustate_p;
		cacr         = cacr_p;
		vbr          = vbr_p;
		wr           = wr_p;
		uds_in       = uds_p;
		lds_in       = lds_p;
		reset_out    = reset_out_p;
		chip_as      = c_as;
		chip_rw      = c_rw;
		chip_uds     = c_uds;
		chip_lds     = c_lds;
		chip_addr    = cpu_addr_p[23:1];
		chip_din     = cpu_dout_p;
		chip_data    = chipdout_i;
		fastchip_sel = cpu_req & !cpu_addr_p[31:24];
		fastchip_lw  = longword;
	end
	else begin
		cpu_dout     = cpu_dout_o;
		cpu_addr     = {cpu_addr_o,1'b0};
		cpustate     = as_o ? 2'b01 : ~{wr_o,wr_o};
		cacr         = 1;
		vbr          = 0;
		wr           = wr_o;
		uds_in       = uds_o;
		lds_in       = lds_o;
		reset_out    = reset_out_o;
		chip_as      = ramsel | as_o;
		chip_rw      = wr_o;
		chip_uds     = uds_o;
		chip_lds     = lds_o;
		chip_addr    = cpu_addr_o[23:1];
		chip_din     = cpu_dout_o;
		chip_data    = chip_dout;
		fastchip_sel = 0;
		fastchip_lw  = 0;
	end
end

wire [15:0] cpu_dout_p;
wire [31:0] cpu_addr_p;
wire  [1:0] cpustate_p;
wire  [3:0] cacr_p;
wire [31:0] vbr_p;
wire        wr_p;
wire        uds_p;
wire        lds_p;
wire        reset_out_p;
wire        longword;

TG68KdotC_Kernel
#(
	.sr_read(2),        // 0=>user,   1=>privileged,    2=>switchable with CPU(0)
	.vbr_stackframe(2), // 0=>no,     1=>yes/extended,  2=>switchable with CPU(0)
	.extaddr_mode(2),   // 0=>no,     1=>yes,           2=>switchable with CPU(1)
	.mul_mode(2),       // 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no MUL,
	.div_mode(2),       // 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no DIV,
	.bitfield(2)        // 0=>no,     1=>yes,           2=>switchable with CPU(1)
)
cpu_inst_p
(
  .clk(clk),
  .nreset(reset),
  .clkena_in(clkena_p_throttled),
  .data_in(cpu_din),
  .ipl(cpu_ipl),
  .ipl_autovector(1),
  .regin_out(),
  .addr_out(cpu_addr_p),
  .data_write(cpu_dout_p),
  .nwr(wr_p),
  .nuds(uds_p),
  .nlds(lds_p),
  .nresetout(reset_out_p),
  .longword(longword),
  
  .cpu(cpucfg),
  .busstate(cpustate_p),		// 0: fetch code, 1: no memaccess, 2: read data, 3: write data
  .cacr_out(cacr_p),
  .vbr_out(vbr_p)
);

wire [15:0] cpu_dout_o;
wire [23:1] cpu_addr_o;
wire  [2:0] fc_o;
wire        wr_o;
wire        as_o;
wire        uds_o;
wire        lds_o;
wire        reset_out_o;

fx68k cpu_inst_o
(
	.clk(clk),
	.enPhi1(ph1),
	.enPhi2(ph2),

	.extReset(~reset),
	.pwrUp(~reset),
	.oRESETn(reset_out_o),
	.HALTn(1),

	.eRWn(wr_o),
	.ASn(as_o),
	.LDSn(lds_o),
	.UDSn(uds_o),
	.DTACKn(ramsel ? ~ramready : chip_dtack),

	.FC0(fc_o[0]),
	.FC1(fc_o[1]),
	.FC2(fc_o[2]), 

	.VPAn(~&fc_o),
	.BERRn(1),
	.BRn(1),
	.BGACKn(1),
	.IPL0n(chip_ipl[0]),
	.IPL1n(chip_ipl[1]),
	.IPL2n(chip_ipl[2]),
	.iEdb(cpu_din),
	.oEdb(cpu_dout_o),
	.eab(cpu_addr_o)
);

wire cpu_req = (cpustate != 1);

wire cchip = turbochip_d & (!cpustate | dcache_d);
wire ckick = turbokick_d & (!cpustate | dcache_d);

reg turbochip_d;
reg turbokick_d;
reg dcache_d;
always @(posedge clk) begin
	if (~reset | ~reset_out) begin
		turbochip_d <= 0;
		turbokick_d <= 0;
		dcache_d    <= 0;
	end
	else if (~cpu_req) begin	// No mem access, so safe to switch chipram access mode
		turbochip_d <= cachecfg[0] & cpucfg[1];
		turbokick_d <= cachecfg[1] & cpucfg[1];
		dcache_d    <= cachecfg[2];
	end
end

// Stock-speed 14 MHz throttle. cachecfg[3]=1 forces a 9-sysclk cooldown
// after every clkena tick. During cooldown, the bus controllers sit in
// IDLE (no in-flight transaction, since CPU hasn't moved on yet), so no
// memory handshake is at risk. Net pipeline rate drops ~10x to ~1.37 MIPS,
// landing near a real A1200 020 at 14 MHz.
wire stock_speed   = cachecfg[3];
// clkena gates the CPU pipeline forward-tick. The bridge fires cdtv_selack
// combinationally with sel, so the data is available on the same cycle —
// equivalent to fastchip_ready being asserted "immediately" alongside
// fastchip_selack. We treat cdtv_selack as the bridge's "ready" signal.
wire clkena_p_base = ~cpu_req | chipready | ramready | fastchip_ready | cdtv_selack;

reg [3:0] cooldown;
always @(posedge clk) begin
	if (~reset)                                cooldown <= 4'd0;
	else if (cooldown != 4'd0)                 cooldown <= cooldown - 4'd1;
	else if (stock_speed & clkena_p_base)      cooldown <= 4'd9;
end
wire clkena_p_throttled = clkena_p_base & (cooldown == 4'd0);

reg       chipreq;
reg [2:0] cpu_ipl;
always @(posedge clk) begin
	// chipreq drives the chip-bus DTACK handshake. Gate off the CDTV bridge
	// the same way fastchip is gated — when cdtv_selack fires, the bridge
	// is serving the access and the chip bus must stay idle.
	chipreq <= cpu_req & ~ramsel & ~fastchip_selack & ~cdtv_selack;
	cpu_ipl <= ipl_i;
end

reg ph1n, ph2n;
always @(posedge clk) begin
	ph1n <= ph1;
	ph2n <= ph2;
end

reg        chipready;
reg [15:0] chipdout_i;
reg  [2:0] ipl_i;
reg        c_as,c_rw,c_uds,c_lds;
always @(negedge clk, negedge reset) begin
	reg [1:0] stage;
	reg waitm;
	reg ready;

	if(~reset) begin
		stage <= 0;
		c_as <= 1;
		c_rw <= 1;
		c_uds <= 1;
		c_lds <= 1;
		ready <= 0;
	end
	else begin
		if (ph2n) begin
			waitm <= chip_dtack;
			if(~stage[0]) ipl_i <= chip_ipl;
		end

		chipready <= 0;
		if (ph1n) begin
			chipready <= ready;
			ready <= 0;
			case (stage)
				0: if (chipreq) begin
						c_as <= 0;
						c_rw <= wr;
						c_uds <= uds_in;
						c_lds <= lds_in;
						stage <= 1;
					end
				1: stage <= 2;
				2: begin
						chipdout_i <= chip_dout;
						if (~waitm) begin
							c_as <= 1;
							c_rw <= 1;
							c_uds <= 1;
							c_lds <= 1;
							ready <= 1;
							stage <= 3;
						end
					end
				3: stage <= 0;
			endcase
		end
	end
end

///////////////////// AUTOCONFIG ////////////////////////////

reg       ac_toccata;
reg       ac_cdtv;
reg [2:0] ac_memcard;
reg [3:0] autocfg_data;
reg [7:0] cdtv_base;

always @(*) begin
	autocfg_data = 4'b1111;

	// CDTV DMAC — first in the chain when cdtv_mode is on. ROM image is
	// the WinUAE cdtv.cpp dmacmemory[] (cdtv_init: ew(0x00,0xC1),
	// ew(0x04,0x03), ew(0x08,0x40), ew(0x10,0x02), ew(0x14,0x02), serial
	// 0). Nibbles below are what the CPU reads as the high half of each
	// byte at offsets 0,2,...,3E inside the $E80000 window. chip_addr[6:1]
	// is the byte offset / no, wait — chip_addr[6:1] is the upper 6 bits
	// of the byte address inside the autoconfig page (matches the existing
	// fastram case statement convention).
	if (ac_cdtv) begin
		case (chip_addr[6:1])
			6'h00: autocfg_data = 4'b1100; // byte 0x00: 0xC1 high nibble (NOT inv) -> 0xC
			6'h01: autocfg_data = 4'b0001; // byte 0x02: 0xC1 low nibble (NOT inv)  -> 0x1
			// All other registers below are inverted-nibble. Defaults to 4'b1111
			// (= NOT 0x0) for serial/reserved bytes, set explicitly only where the
			// underlying nibble is non-zero.
			6'h03: autocfg_data = 4'b1100; // byte 0x06: NOT(0x03 lo nibble) = NOT 0x3 = 0xC (product number)
			6'h04: autocfg_data = 4'b1011; // byte 0x08: NOT(0x40 hi nibble) = NOT 0x4 = 0xB (er_flags: CANT_SHUTUP)
			6'h09: autocfg_data = 4'b1101; // byte 0x12: NOT(0x02 lo nibble) = NOT 0x2 = 0xD (manuf hi)
			6'h0B: autocfg_data = 4'b1101; // byte 0x16: NOT(0x02 lo nibble) = NOT 0x2 = 0xD (manuf lo)
			default: autocfg_data = 4'b1111;
		endcase
	end
	// Zorro II RAM (Up to 8 meg at 0x200000). It has a fixed base, so it must be first in the chain.
	else if (~ac_memcard[2] && ac_memcard[1:0]) begin
		case (chip_addr[6:1])
			6'b000000: autocfg_data = 4'b1110;	// Zorro-II card, add mem, no ROM
			6'b000001:
				case (ac_memcard[1:0])
							1: autocfg_data = 4'b0110; // 2MB
							2: autocfg_data = 4'b0111; // 4MB
					default: autocfg_data = 4'b0000; // 8MB
				endcase
			6'b001000: autocfg_data = 4'b1110;	// Manufacturer ID: 0x139c
			6'b001001: autocfg_data = 4'b1100;
			6'b001010: autocfg_data = 4'b0110;
			6'b001011: autocfg_data = 4'b0011;
			6'b010011: autocfg_data = 4'b1110; //serial=1
			  default:;
		endcase
	end
	// Zorro II other cards
	else if(ac_toccata) begin
		case (chip_addr[6:1])
			6'h0: autocfg_data = 4'b1100; // Zorro-II card, no link, no ROM
			6'h1: autocfg_data = 4'b0001; // Next board not related, size 'h64k
			// Inverted from here on
			6'h3: autocfg_data = 4'b0011; // Lower byte product number
			//6'h5: autocfg_data = 4'b1101; // logical size 64k -- commented out -> logical size == physical size. Issue with KS1.3?
			6'h8: autocfg_data = 4'b1011; // Manufacturer ID: 0x4754
			6'h9: autocfg_data = 4'b1000;
			6'ha: autocfg_data = 4'b1010;
			6'hb: autocfg_data = 4'b1011;
			default: ;
		endcase
	end
	// Zorro III RAM 128MB/256MB/384MB
	else if(ac_memcard[2]) begin
		case (chip_addr[6:1])
			6'b000000: autocfg_data = 4'b1010;	// Zorro-III card, add mem, no ROM
			6'b000001: autocfg_data = ac_memcard[1] ? 4'b0011 : 4'b0100; // 128MB or 256MB, extended
			6'b000010: autocfg_data = 4'b1110;	// ProductID=0x10 (only setting upper nibble)
			6'b000100: autocfg_data = 4'b0000;	// Memory card, not silenceable, Extended size, reserved.
			6'b000101: autocfg_data = 4'b1111;	// 0000 - logical size matches physical size TODO change this to 0001, so it is autosized by the OS, WHEN it will be 24MB.
			6'b001000: autocfg_data = 4'b1110;	// Manufacturer ID: 0x139c
			6'b001001: autocfg_data = 4'b1100;
			6'b001010: autocfg_data = 4'b0110;
			6'b001011: autocfg_data = 4'b0011;
			6'b010011: autocfg_data = {2'b11, ~ac_memcard[1], ac_memcard[1]};	// serial=1/2
			  default:;
		endcase
	end
end

wire sel_autoconfig = (chip_addr[23:16] == 8'b11101000) && (ac_memcard || ac_toccata || ac_cdtv); //$E80000 - $E8FFFF

// CDTV AC ROM byte mirror — spec section 2.2 + section 2.3 row 1.
// The BIOS reads $E900-$E93F (AC ROM at the post-relocation base) to
// re-identify the board. Returned bytes are the Z2-encoded NIBBLE form
// stored in WinUAE's dmacmemory[] array — the BIOS does NOT see the
// raw logical value, it sees the encoded nibbles split across two
// adjacent offsets.
//
// ew() helper at cdtv.cpp:1610-1619 splits each logical byte:
//   * Offsets $00/$02/$40/$42 use the NOT-inverted form:
//       dmacmemory[addr  ] = value & 0xF0
//       dmacmemory[addr+2] = (value & 0x0F) << 4
//   * Other offsets use the INVERTED form (Z2 complement):
//       dmacmemory[addr  ] = ~(value & 0xF0) & 0xF0
//       dmacmemory[addr+2] = ~((value & 0x0F) << 4) & 0xF0
// Untouched slots stay at 0xFF (per memset(dmacmemory, 0xff) at line 1753).
//
// cdtv_ac_rom_addr [5:0] is the byte offset / 2 — so addr=0 hits the
// AC ROM at byte $00, addr=1 hits byte $02, etc.
//
// ew() calls in cdtv_init (cdtv.cpp:1755-1769):
//   ew(0x00, 0xC1)  type=Z2+linked+ROM (NOT inv)
//   ew(0x04, 0x03)  product number = 3
//   ew(0x08, 0x40)  size flag = 64 KB
//   ew(0x10, 0x02)  manuf hi = 2
//   ew(0x14, 0x02)  manuf lo = 2
//   ew(0x18..0x24, 0)  serial = 0
//
// Resulting bytes at the AC ROM offsets used by the BIOS:
reg [7:0] cdtv_ac_rom_byte_r;
always @* begin
	cdtv_ac_rom_byte_r = 8'hFF;
	case (cdtv_ac_rom_addr)
		6'h00: cdtv_ac_rom_byte_r = 8'hC0;   // byte $00 = 0xC1 hi nibble (NOT inv)
		6'h01: cdtv_ac_rom_byte_r = 8'h10;   // byte $02 = 0xC1 lo nibble << 4 (NOT inv)
		6'h02: cdtv_ac_rom_byte_r = 8'hF0;   // byte $04 = ~(0x03 hi nibble) = 0xF0 (inv)
		6'h03: cdtv_ac_rom_byte_r = 8'hC0;   // byte $06 = ~(0x03 lo nibble << 4) = 0xC0 (inv)
		6'h04: cdtv_ac_rom_byte_r = 8'hB0;   // byte $08 = ~(0x40 hi nibble) = 0xB0 (inv)
		6'h05: cdtv_ac_rom_byte_r = 8'hF0;   // byte $0A = ~(0x40 lo nibble << 4) = 0xF0
		6'h08: cdtv_ac_rom_byte_r = 8'hF0;   // byte $10 = ~(0x02 hi) = 0xF0
		6'h09: cdtv_ac_rom_byte_r = 8'hD0;   // byte $12 = ~(0x02 lo << 4) = 0xD0
		6'h0A: cdtv_ac_rom_byte_r = 8'hF0;   // byte $14 = ~(0x02 hi) = 0xF0
		6'h0B: cdtv_ac_rom_byte_r = 8'hD0;   // byte $16 = ~(0x02 lo << 4) = 0xD0
		// Serial bytes $18 / $1C / $20 / $24 = ew(_, 0): both nibbles 0 inv -> 0xF0
		6'h0C: cdtv_ac_rom_byte_r = 8'hF0;   // byte $18
		6'h0D: cdtv_ac_rom_byte_r = 8'hF0;   // byte $1A
		6'h0E: cdtv_ac_rom_byte_r = 8'hF0;   // byte $1C
		6'h0F: cdtv_ac_rom_byte_r = 8'hF0;   // byte $1E
		6'h10: cdtv_ac_rom_byte_r = 8'hF0;   // byte $20
		6'h11: cdtv_ac_rom_byte_r = 8'hF0;   // byte $22
		6'h12: cdtv_ac_rom_byte_r = 8'hF0;   // byte $24
		6'h13: cdtv_ac_rom_byte_r = 8'hF0;   // byte $26
		default: cdtv_ac_rom_byte_r = 8'hFF; // memset(0xff) default
	endcase
end
assign cdtv_ac_rom_byte = cdtv_ac_rom_byte_r;

reg       z2ram_ena;
reg [4:0] z3ram_base0;
reg [3:0] z3ram_base1;
reg       z3ram_ena0;
reg       z3ram_ena1;
always @(posedge clk) begin
	reg old_uds;
	old_uds <= chip_uds;

	if (~reset | ~reset_out) begin
		ac_memcard  <= cpucfg[1] ? fastramcfg : fastramcfg[2] ? 3'd3 : {1'b0, fastramcfg[1:0]};
		// In CDTV mode, suppress Toccata (real CDTV had no soundcard) so
		// the DMAC card is alone in the chain — matches the WinUAE log
		// "Card 01: CDTV DMAC" / end.
		ac_toccata  <= cdtv_mode ? 1'b0 : 1'b1;
		// BISECTION 2026-05-20: ac_cdtv normally <= cdtv_mode. Force to 0
		// to test whether the CDTV AC responder is what's blanking B_ksonly
		// (CDTV mode ON, KS 1.3 only). If KS hand reappears, the AC ROM
		// table is the bug; if still blank, the issue is elsewhere ($F00000
		// mirror, bridge IRQ pulses, etc.).
		ac_cdtv     <= 1'b0;
		cdtv_base   <= 8'hE9;       // WinUAE default fallback ($E90000)
		z2ram_ena   <= 0;
		z3ram_ena0  <= 0;
		z3ram_ena1  <= 0;
		z3ram_base0 <= 1;
		z3ram_base1 <= 1;
	end
	else if (sel_autoconfig && ~chip_rw && ~chip_uds && old_uds) begin
		if(ac_cdtv) begin
			if (chip_addr[6:1] == 6'b100100) begin // Register 0x48 — KS writes the base address byte
				cdtv_base <= cpu_dout[15:8];
				ac_cdtv   <= 0;
			end
			else if (chip_addr[6:1] == 6'b100110) begin // Register 0x4C — "shut up" / decline
				ac_cdtv   <= 0;
			end
		end
		else if(~ac_memcard[2] && ac_memcard[1:0]) begin
			if (chip_addr[6:1] == 6'b100100) begin // Register 0x48 - config, ZII RAM
				z2ram_ena <= 1;
				ac_memcard <= 0;
			end
		end
		else if(ac_toccata) begin
			if (chip_addr[6:1] == 6'b100100) begin // Register 0x48 - config, Toccata card in ZII io space ($E90000)
				toccata_base <= cpu_dout[7:0];
				ac_toccata<=0;
			end
		end
		else if(ac_memcard[2]) begin
			if(chip_addr[6:1] == 6'b100010) begin // Register 0x44, assign base address to ZIII RAM.
				if(~ac_memcard[1]) begin
					z3ram_base1 <= cpu_dout[15:12]; //256MB chunk
					z3ram_ena1 <= 1;
					ac_memcard <= {ac_memcard[0], ac_memcard[0], 1'b0};
				end
				else begin
					z3ram_base0 <= cpu_dout[15:11]; //128MB chunk
					z3ram_ena0 <= 1;
					ac_memcard <= 0;
				end
			end
		end
	end
end

// In CDTV mode ac_toccata resets to 0 to suppress Toccata from autoconfig,
// but toccata_base is uninitialized — without the cdtv_mode mask Toccata
// would claim addr $00xxxx (CPU reset vector) at FPGA boot. Mask out.
assign toccata_ena = ~ac_toccata & ~cdtv_mode;

endmodule
