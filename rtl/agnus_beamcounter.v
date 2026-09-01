////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// Copyright 2006, 2007 Dennis van Weeren                                     //
//                                                                            //
// This file is part of Minimig                                               //
//                                                                            //
// Minimig is free software; you can redistribute it and/or modify            //
// it under the terms of the GNU General Public License as published by       //
// the Free Software Foundation; either version 3 of the License, or          //
// (at your option) any later version.                                        //
//                                                                            //
// Minimig is distributed in the hope that it will be useful,                 //
// but WITHOUT ANY WARRANTY; without even the implied warranty of             //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              //
// GNU General Public License for more details.                               //
//                                                                            //
// You should have received a copy of the GNU General Public License          //
// along with this program.  If not, see <http://www.gnu.org/licenses/>.      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// Agnus beamcounter                                                          //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


module agnus_beamcounter
(
	input	            clk,            // bus clock
	input             clk7_en,
	input	            reset,          // reset
	input	            cck,            // CCK clock
	input	            ntsc,           // NTSC mode switch
	input             aga,            
	input	            ecs,            // ECS enable switch
	input	            a1k,            // enable A1000 VBL interrupt timing
	input	     [15:0] data_in,        // bus data in
	output reg [15:0] data_out,       // bus data out
	input       [8:1] reg_address_in, // register address inputs
	input      [10:0] lpen_vpos,      // light-pen vertical position, latched by userspace (userio.v)
	input       [8:0] lpen_hpos,      // light-pen horizontal position, latched by userspace (userio.v)
	output reg  [8:0] hpos,           // horizontal beam counter (140ns)
	output reg [10:0] vpos,           // vertical beam counter
	output reg        _hsync,         // horizontal sync
	output reg        _vsync,         // vertical sync
	output            field1,         // 
	output reg        lace,
	output            _csync,         // composite sync
	output reg        hblank,         // video blanking
	output reg        vblank,         // video blanking
	output            vbl,            // vertical blanking
	output            vblend,         // last line of vertival blanking
	output            eol,            // end of video line
	output            eof,            // end of video frame
	output reg        vbl_int,        // vertical interrupt request (for Paula)
	output      [8:0] htotal_out,     // video line length
	output            harddis_out,
	output            varbeamen_out
);

//register names and adresses		
parameter VPOSR    = 9'h004;
parameter VPOSW    = 9'h02A;
parameter VHPOSR   = 9'h006;
parameter VHPOSW   = 9'h02C;
parameter BPLCON0  = 9'h100;
parameter HTOTAL   = 9'h1C0;
parameter HSSTOP   = 9'h1C2;
parameter HBSTRT   = 9'h1C4;
parameter HBSTOP   = 9'h1C6;
parameter VTOTAL   = 9'h1C8;
parameter VSSTOP   = 9'h1CA;
parameter VBSTRT   = 9'h1CC;
parameter VBSTOP   = 9'h1CE;
parameter HSSTRT   = 9'h1DE;
parameter BEAMCON0 = 9'h1DC;
parameter HHPOSR   = 9'h1DA;
parameter VSSTRT   = 9'h1E0;
parameter HCENTER  = 9'h1E2;

parameter HBSTRT_VAL      = 17+4+4;          // horizontal blanking start
parameter HSSTRT_VAL      = 29+4+4;          // front porch = 1.6us (29)
parameter HSSTOP_VAL      = 63-1+4+4;        // hsync pulse duration = 4.7us (63)
parameter HBSTOP_VAL      = 103-5+4;         // back porch = 4.7us (103) shorter blanking for overscan visibility
parameter HCENTER_VAL     = 256+4+4;         // position of vsync pulse during the long field of interlaced screen
parameter VSSTRT_VAL      = 2;               // vertical sync start
parameter VSSTOP_VAL      = 5;               // PAL vsync width: 2.5 lines (NTSC: 3 lines - not implemented)
parameter VBSTRT_VAL      = 0;               // vertical blanking start
parameter HTOTAL_VAL      = 8'd227 - 8'd1;   // line length of 227 CCKs; NTSC alternates 227/228 via long_line
parameter VTOTAL_PAL_VAL  = 11'd312 - 11'd1; // total number of lines (PAL: 312 lines, NTSC: 262)
parameter VTOTAL_NTSC_VAL = 11'd262 - 11'd1; // total number of lines (PAL: 312 lines, NTSC: 262)
parameter VBSTOP_PAL_VAL  = 9'd25;           // vertical blanking end (PAL 26 lines, NTSC vblank 21 lines)
parameter VBSTOP_NTSC_VAL = 9'd20;           // vertical blanking end (PAL 26 lines, NTSC vblank 21 lines)

//wire	[8:0] vbstop;		// vertical blanking stop

//beam position output signals
//assign	htotal = 8'd227 - 8'd1;                           // line length of 227 CCKs in PAL mode (NTSC line length of 227.5 CCKs is not supported)
//assign	vtotal = pal ? VTOTAL_PAL_VAL : VTOTAL_NTSC_VAL;  // total number of lines (PAL: 312 lines, NTSC: 262)
//assign	vbstop = pal ? VBSTOP_PAL_VAL : VBSTOP_NTSC_VAL;  // vertical blanking end (PAL 26 lines, NTSC vblank 21 lines)

//first visible line $1A (PAL) or $15 (NTSC)
//sprites are fetched on line $19 (PAL) or $14 (NTSC) - vblend signal used to tell Agnus to fetch sprites during the last vertical blanking line

//--------------------------------------------------------------------------------------

//beamcounter read registers VPOSR and VHPOSR
//
// The light pen FREEZES these two registers, it does not replace them. That
// distinction is the whole design. A plain mux -- which is what this was --
// holds VPOSR at whatever userspace last latched for as long as BPLCON0 bit 3
// stays set, so the counters never advance again and every beam-wait loop on
// the machine spins forever. With no gun connected they froze at zero. The
// freeze here is a latch with a defined start and a defined end, so the
// counters always come back; see the LIGHT PEN LATCH block below.
//
// lpen_frozen is 0 on any machine that never touches the light pen -- lpen_trig
// can only be set while lpen_en is -- so both terms below collapse to exactly
// the original expression, vpos[10:8] and {vpos[7:0],hpos[8:1]}. That is the
// read every game polling the beam position depends on; do not disturb the
// false-condition path.
reg        lpen_trig;   // latch armed: VPOSR/VHPOSR are frozen
reg [10:0] vpos_lpen;   // the frozen vertical position
reg  [8:0] hpos_lpen;   // the frozen horizontal position

wire lpen_frozen = lpen_trig & ~lpendis;

always @(*) begin
	if (reg_address_in[8:1]==VPOSR[8:1] || reg_address_in[8:1]==VPOSW[8:1])
		data_out[15:0] = {long_frame,1'b0,ecs,ntsc,2'b00,{2{aga}},long_line,4'b0000,
		                  lpen_frozen ? vpos_lpen[10:8] : vpos[10:8]};
	else if (reg_address_in[8:1]==VHPOSR[8:1] || reg_address_in[8:1]==VHPOSW[8:1])
		// The live half is 06f30af verbatim: the internal hpos runs one colour
		// clock ahead of what real Agnus reports, so the readback decrements it,
		// and a zero means the htotal wrap unless an ERSY genlock freeze is
		// holding it there. Measured upstream against the vAmigaTS VPOS suite.
		//
		// The frozen half deliberately does NOT get that correction. hpos_lpen
		// does not hold a sample of the internal counter -- it holds the value a
		// program is meant to READ, computed by userspace from WinUAE's raster
		// geometry, which is already reported-space. Decrementing it again would
		// shift the pen one colour clock left of where userspace aimed it.
		//
		// The trigger comparison in the latch below is the loose end: it matches
		// the internal hpos against that reported-space target, so it arms one
		// colour clock early. Left alone rather than guessed at -- the position
		// path has never been verified against a gun that locks, and the
		// userspace constants would need re-measuring with it. See the note on
		// LPEN_HPOS_MIN in support/lightpen/amiga_lightpen.cpp.
		data_out[15:0] = lpen_frozen
		    ? {vpos_lpen[7:0], hpos_lpen[8:1]}
		    : {vpos[7:0], |hpos[8:1] ? hpos[8:1] - 8'd1 : ersy ? 8'd0 : htotal_cck};
	// HHPOSR ($1DA, ECS, read only) reports the same horizontal counter VHPOSR
	// does, in the low byte and on its own. WinUAE custom.cpp: HHPOSR() returns
	// the light pen latch when one is armed and hhpos otherwise, masked to
	// 0xff; hhpos is assigned agnus_hpos every colour clock except in BEAMCON0
	// DUAL mode, where it free-runs and HHPOSW ($1D8) can reseed it.
	//
	// This core does not implement DUAL mode, so outside it the two readbacks
	// agree by construction and this mirrors the horizontal half of VHPOSR
	// exactly -- decrement, ERSY case, light pen freeze and all. HHPOSW is not
	// decoded for the same reason: with hhpos not free-running there is nothing
	// for a write to hold.
	else if (ecs && reg_address_in[8:1]==HHPOSR[8:1])
		data_out[15:0] = {8'h00, lpen_frozen
		    ? hpos_lpen[8:1]
		    : (|hpos[8:1] ? hpos[8:1] - 8'd1 : ersy ? 8'd0 : htotal_cck)};
	else
		data_out[15:0] = 0;
end

// BEAMCON0 register
reg [15:0] beamcon0_reg;
always @ (posedge clk) begin
	if (clk7_en) begin
		if (reset)
			beamcon0_reg <= {10'b0, ~ntsc, 5'b0};
		else if ((reg_address_in[8:1] == BEAMCON0[8:1]) && ecs)
			beamcon0_reg <= data_in[15:0];
	end
end

wire harddis      = beamcon0_reg[14];
wire lpendis      = beamcon0_reg[13];
wire varvben      = beamcon0_reg[12];
wire loldis       = beamcon0_reg[11];
//wire cscben       = beamcon0_reg[10];
wire varvsyen     = beamcon0_reg[ 9];
wire varhsyen     = beamcon0_reg[ 8];
wire varbeamen    = beamcon0_reg[ 7];
//wire displaydual  = beamcon0_reg[ 6];
//wire displaypal   = beamcon0_reg[ 5];
//wire varcsyen     = beamcon0_reg[ 4];
//wire blanken      = beamcon0_reg[ 3];
//wire csynctrue    = beamcon0_reg[ 2];
//wire vsynctrue    = beamcon0_reg[ 1];
//wire hsynctrue    = beamcon0_reg[ 0];


// write ERSY bit of bplcon0 register (External ReSYnchronization - genlock)
reg ersy;
always @(posedge clk) begin
	if (clk7_en) begin
		if (reset)
			ersy <= 1'b0;
		else if (reg_address_in[8:1] == BPLCON0[8:1])
			ersy <= data_in[1];
	end
end

//BPLCON0 register
// lpen_en (bit 3, LPEN) rides in the same always block as lace (bit 2): both
// are loaded only on a BPLCON0 write, so this reuses the one comparator
// instead of adding a second decode for the same address.
reg lpen_en;
always @(posedge clk) begin
	if (clk7_en) begin
		if (reset) begin
			lace    <= 1'b0;
			lpen_en <= 1'b0;
		end
		else if (reg_address_in[8:1]==BPLCON0[8:1]) begin
			lace    <= data_in[2];
			lpen_en <= data_in[3];
		end
	end
end

//BEAMCON0 register
reg pal;	// pal mode switch
always @(posedge clk) begin
	if (clk7_en) begin
		if (reset)
			pal <= ~ntsc;
		else if (reg_address_in[8:1]==BEAMCON0[8:1] && ecs)
			pal <= data_in[5];
	end
end

// programmable display mode regs
reg [ 8:0] htotal_reg;
reg [ 8:0] hsstrt_reg;
reg [ 8:0] hsstop_reg;
reg [ 8:0] hcenter_reg;
// HBSTRT and HBSTOP do carry sub-colour-clock position -- WinUAE masks them to
// 0x7ff where the other four horizontal registers get 0xff -- but those extra
// bits belong to a mechanism this core does not have, and they must NOT reach
// the comparison below.
//
// Two different consumers, and only one of them is us. drawing.cpp's
// update_hblank() builds denise_phbstrt_lores from bit 10, and does so ONLY
// inside `if (exthblankon_aga)`; its else branch sets every programmed position
// to -1. That is Denise's extended-HBLANK path, AGA only, absent here.
//
// What this register drives is the Agnus-side programmed blanking, and for that
// WinUAE uses colour clocks and nothing finer -- custom.cpp keeps the raw write
// but compares against hbstrt_cck:
//
//     hbstrt = value & 0x7ff;
//     hbstrt_cck = hbstrt & 0xff;
//     ...
//     if (hhp == hbstrt_cck) { agnus_phblank = true; ... }
//
// so the stored comparison value is the colour clock shifted up, with a zero in
// the half-colour-clock position. Feeding bit 10 in here instead shifts every
// programmed blanking edge by half a lores pixel, which on hardware reads as a
// blurred picture and a doubled OSD -- measured 2026-09-01, PAL, and the reason
// this comment is longer than the code.
reg [ 8:0] hbstrt_reg;
reg [ 8:0] hbstop_reg;
reg [10:0] vtotal_reg;
reg [10:0] vsstrt_reg;
reg [10:0] vsstop_reg;
//reg [10:0] vbstrt_reg;
reg [10:0] vbstop_reg;

always @ (posedge clk) begin
	if (clk7_en) begin
		if (reset) begin
			htotal_reg  <= HTOTAL_VAL << 1;
			hsstrt_reg  <= HSSTRT_VAL[8:0];
			hsstop_reg  <= HSSTOP_VAL[8:0];
			hcenter_reg <= HCENTER_VAL[8:0];
			hbstrt_reg  <= HBSTRT_VAL[8:0];
			hbstop_reg  <= HBSTOP_VAL[8:0];
			vtotal_reg  <= pal ? VTOTAL_PAL_VAL : VTOTAL_NTSC_VAL;
			vsstrt_reg  <= VSSTRT_VAL[10:0];
			vsstop_reg  <= VSSTOP_VAL[10:0];
			//vbstrt_reg  <= VBSTRT_VAL[10:0];
			vbstop_reg  <= pal ? VBSTOP_PAL_VAL : VBSTOP_NTSC_VAL;
		end else begin
			case (reg_address_in[8:1])
				HTOTAL [8:1] : htotal_reg  <= {data_in[ 7:0], 1'b0};
				HSSTRT [8:1] : hsstrt_reg  <= {data_in[ 7:0], 1'b0};
				HSSTOP [8:1] : hsstop_reg  <= {data_in[ 7:0], 1'b0};
				HCENTER[8:1] : hcenter_reg <= {data_in[ 7:0], 1'b0};
				HBSTRT [8:1] : hbstrt_reg  <= {data_in[ 7:0], 1'b0};
				HBSTOP [8:1] : hbstop_reg  <= {data_in[ 7:0], 1'b0};
				VTOTAL [8:1] : vtotal_reg  <= {data_in[10:0]};
				VSSTRT [8:1] : vsstrt_reg  <= {data_in[10:0]};
				VSSTOP [8:1] : vsstop_reg  <= {data_in[10:0]};
				//VBSTRT [8:1] : vbstrt_reg  <= {data_in[10:0]};
				VBSTOP [8:1] : vbstop_reg  <= {data_in[10:0]};
			endcase
		end
	end
end

// NTSC lines are 227.5 colour clocks, which the chipset produces by alternating
// 227 and 228. long_line is that alternation, declared here because the line
// length below depends on it.
//
// WinUAE custom.cpp:
//
//     if (!(new_beamcon0 & BEAMCON0_PAL) && !(new_beamcon0 & BEAMCON0_LOLDIS)) {
//         lol = lol ? false : true;
//         linetoggle = true;
//     } else {
//         lol = false;
//         linetoggle = false;
//     }
//     ...
//     maxhpos = maxhpos_short + lol;
//
// The toggle was already here and already correct -- what was missing is the
// second half, the line actually being a colour clock longer. That is
// htotal_cck below, and it is what end_of_line, htotal_out and the VHPOSR wrap
// value all use, so nothing has to know about long_line separately.
reg long_line;

// programmable display mode values
wire [ 8:0] htotal  =             varbeamen ? htotal_reg  : HTOTAL_VAL << 1; // line length of 227 CCKs; NTSC alternates 227/228 via long_line

// The last colour clock of THIS line. htotal is the short-line length, as
// WinUAE's maxhpos_short is, and a long line runs one colour clock past it.
wire [ 7:0] htotal_cck = htotal[8:1] + {7'd0, long_line};
wire [ 8:0] hsstrt  = varhsyen && varbeamen ? hsstrt_reg  : HSSTRT_VAL[8:0];
wire [ 8:0] hsstop  = varhsyen && varbeamen ? hsstop_reg  : HSSTOP_VAL[8:0];
wire [ 8:0] hcenter = varhsyen && varbeamen ? hcenter_reg : HCENTER_VAL[8:0];
wire [ 8:0] hbstrt  =             varbeamen ? hbstrt_reg  : HBSTRT_VAL[8:0];
wire [ 8:0] hbstop  =             varbeamen ? hbstop_reg  : HBSTOP_VAL[8:0];
wire [10:0] vtotal  =             varbeamen ? vtotal_reg  : pal ? VTOTAL_PAL_VAL : VTOTAL_NTSC_VAL;
wire [10:0] vsstrt  = varvsyen && varbeamen ? vsstrt_reg  : VSSTRT_VAL[10:0];
wire [10:0] vsstop  = varvsyen && varbeamen ? vsstop_reg  : VSSTOP_VAL[10:0];
//wire [10:0] vbstrt  = varvben  && varbeamen ? vbstrt_reg  : VBSTRT_VAL[10:0];
wire [10:0] vbstop  = varvben  && varbeamen ? vbstop_reg  : pal ? VBSTOP_PAL_VAL : VBSTOP_NTSC_VAL;

// The effective length, not the short-line one. agnus.v:483 wraps its DMA slot
// lookahead at htotal[8:1], so exporting the short length would make the slot
// grid wrap one colour clock early on every long line.
assign htotal_out    = {htotal_cck, htotal[0]};
assign harddis_out   = harddis || varbeamen || varvben;
assign varbeamen_out = varbeamen;


//--------------------------------------------------------------------------------------//
//                                                                                      //
//   HORIZONTAL BEAM COUNTER                                                            //
//                                                                                      //
//--------------------------------------------------------------------------------------//

//generate start of line signal
reg end_of_line;
always @(posedge clk) begin
	if (clk7_en) begin
		if (hpos[8:0]=={htotal_cck,1'b0})
			end_of_line <= 1'b1;
		else
			end_of_line <= 1'b0;
	end
end

// horizontal beamcounter
always @(posedge clk) begin
	if (clk7_en) begin
		if (reg_address_in[8:1]==VHPOSW[8:1])
			hpos[8:1] <= data_in[7:0]; 
		else if (end_of_line)
			hpos[8:1] <= 0;
		else if (cck && (~ersy || |hpos[8:1]))
			hpos[8:1] <= hpos[8:1] + 1'b1;
	end
end

always @(cck) hpos[0] = cck;

// The long-line alternation itself. Declared above, next to the line length it
// feeds.
//
// VPOSW RESETS it. It does not write it from a data bit, which is what this
// first shipped as and what put a wrong line length on a PAL screen. WinUAE's
// actual handler, custom.cpp VPOSW():
//
//     // LOL is always reset when VPOSW is written to.
//     // Implemented in all NTSC Agnus versions and ECS/AGA Agnus in NTSC mode.
//     if (lol) {
//         lol = false;
//         setmaxhpos();
//     }
//
// The earlier version came from custom.cpp:7712, "lol = (i & 0x0080) != 0",
// which is NOT the register handler -- it is inside restore_custom(), reading a
// savestate blob word by word, where the comments merely label the sequence.
// Reading a savestate reader as if it were hardware semantics is how this got
// written, and the same mistake in the same session also produced the HBSTRT
// bit 10 change. If a line looks like it defines a register's behaviour, check
// which function it is in.
//
// Why it mattered so much: this branch takes priority over the end-of-line
// clear below, so on a PAL machine -- where long_line must always be 0 -- a
// VPOSW write carrying bit 7 left it SET until the next end of line, making
// that line 228 colour clocks instead of 227.
always @(posedge clk) begin
	if (clk7_en) begin
		if (reg_address_in[8:1]==VPOSW[8:1])
			long_line <= 1'b0;
		else if (end_of_line)
			if (pal || (loldis && varbeamen))
				long_line <= 1'b0;
			else if (!(loldis && varbeamen))
				long_line <= ~long_line;
	end
end

//--------------------------------------------------------------------------------------//
//                                                                                      //
//   VERTICAL BEAM COUNTER                                                              //
//                                                                                      //
//--------------------------------------------------------------------------------------//

//vertical counter increase
reg vpos_inc; // increase vertical position counter
always @(posedge clk) begin
	if (clk7_en) begin
		if (hpos==2) //actual chipset works in this way
			vpos_inc <= 1'b1;
		else
			vpos_inc <= 1'b0;
	end
end

//external signals assigment
assign eol = vpos_inc;

//vertical position counter
//vpos changes after hpos equals 3
always @(posedge clk) begin
	if (clk7_en) begin
		if (reg_address_in[8:1]==VPOSW[8:1])
			vpos[10:8] <= data_in[2:0];
		else if (reg_address_in[8:1]==VHPOSW[8:1])
			vpos[7:0] <= data_in[15:8];
		else if (vpos_inc)
			if (last_line)
				vpos <= 0;
			else
				vpos <= vpos + 1'b1;
	end
end

// long_frame - long frame signal used in interlaced mode
reg long_frame; // 1 : long frame (313 lines); 0 : normal frame (312 lines)
always @(posedge clk) begin
	if (clk7_en) begin
		if (reset)
			long_frame <= 1'b1;
		else if (reg_address_in[8:1]==VPOSW[8:1])
			long_frame <= data_in[15];
		else if (end_of_frame && lace) // interlace
			long_frame <= ~long_frame;
	end
end

//maximum position of vertical beam position
wire vpos_equ_vtotal = (vpos==vtotal); // vertical beam counter is equal to its maximum count (in interlaced mode it counts one line more)

//extra line in interlaced mode	
reg extra_line; // extra line (used in interlaced mode)
always @(posedge clk) begin
	if (clk7_en) begin
		if (vpos_inc)
			if (long_frame && vpos_equ_vtotal)
				extra_line <= 1'b1;
			else
				extra_line <= 1'b0;
	end
end

//in non-interlaced display the last line is equal to vtotal or vtotal+1 (depends on long_frame)
//in interlaced mode every second frame is vtotal+1 long
wire last_line = long_frame ? extra_line : vpos_equ_vtotal;

assign field1 = (~long_frame) & lace;

//generate end of frame signal
wire end_of_frame = vpos_inc & last_line;

//external signal assigment
assign eof = end_of_frame;

always @(posedge clk) if (clk7_en) vbl_int <= hpos==8 && vpos==(a1k ? 1 : 0); // OCS AGNUS CHIPS 8361/8367 assert vbl int in line #1

//--------------------------------------------------------------------------------------//
//                                                                                      //
//  VIDEO SYNC GENERATOR                                                                //
//                                                                                      //
//--------------------------------------------------------------------------------------//

//horizontal sync
always @(posedge clk) begin
	if (clk7_en) begin
		if (hpos==hsstrt)//start of sync pulse (front porch = 1.69us)
			_hsync <= 1'b0;
		else if (hpos==hsstop)//end of sync pulse (sync pulse = 4.65us)
			_hsync <= 1'b1;
	end
end

//vertical sync and vertical blanking
// PAL: Long field Vsync line 3 - 5.5, Short field: line 2.5 - 5
always @(posedge clk) begin
	if (clk7_en) begin
		if ((vpos==vsstrt+1 && hpos==hsstrt && long_frame) || (vpos==vsstrt && hpos==hcenter && !long_frame))
			_vsync <= 1'b0;
		else if ((vpos==vsstop && hpos==hcenter && long_frame) || (vpos==vsstop && hpos==hsstrt && !long_frame))
			_vsync <= 1'b1;		
	end
end

//apparently generating csync from vsync alligned with leading edge of hsync results in malfunction of the AD724 CVBS/S-Video encoder (no colour in interlaced mode)
//to overcome this limitation semi (only present before horizontal sync pulses) vertical sync serration pulses are inserted into csync
reg vser; // vertical sync serration pulses for composite sync
always @(posedge clk) begin //sync
	if (clk7_en) begin
		if (hpos==hsstrt-(hsstop-hsstrt))//start of sync pulse (front porch = 1.69us)
			vser <= 1'b1;
		else if (hpos==hsstrt)//end of sync pulse	(sync pulse = 4.65us)
			vser <= 1'b0;
	end
end

//composite sync
assign _csync = _hsync & _vsync | vser; //composite sync with serration pulses

//--------------------------------------------------------------------------------------//
//                                                                                      //
//  VIDEO BLANKING GENERATOR                                                            //
//                                                                                      //
//--------------------------------------------------------------------------------------//

/*
//vertical blanking
reg vbl_reg;
always @ (posedge clk) begin
  if (reset)
    vbl_reg <= 1'b0;
  else if (vpos == vbstrt)
    vbl_reg <= 1'b1;
  else if (vpos == vbstop)
    vbl_reg <= 1'b0;
end

assign vbl = vbl_reg; // TODO
*/

assign vbl = (vpos <= vbstop);

//vertical blanking end (last line)
assign vblend = vpos==vbstop;

//--------------------------------------------------------------------------------------//
//                                                                                      //
//   LIGHT PEN LATCH                                                                    //
//                                                                                      //
//--------------------------------------------------------------------------------------//
//
// Modelled on WinUAE (custom.cpp: hsync_handler_pre, BPLCON0, islightpentriggered),
// because what software depends on is the timing of the freeze, not the position
// reported. Four events, in priority order:
//
//   - reset, or BPLCON0 bit 3 clear: never frozen. Clearing the bit unfreezes
//     immediately, which is how a program releases the registers.
//   - end of the last vblank line: unfreeze. This is the one that matters. It
//     is what stops a set bit 3 from wedging VPOSR forever, and it runs every
//     frame whether or not a pen is connected or ever triggers.
//   - end of the first vblank line, with nothing else having fired: freeze
//     anyway, at that line and hpos 1. WinUAE's fallback. It gives a program
//     probing with no pen a defined answer that refreshes every frame, instead
//     of a dead one.
//   - the raster reaching the position userspace latched: freeze there. The
//     real trigger, gated on the pen being on screen and the beam being outside
//     vblank, since a pen cannot see a beam that is not being drawn.
//
// vpos_lpen/hpos_lpen snapshot rather than track: once frozen the reported
// position must not move even if the gun does, until the next unfreeze.
//
// hpos_lpen is compared and stored at full 140ns resolution but read back as
// [8:1], the same 280ns CCK units VHPOSR reports for hpos. Userspace scales for
// that; see LPEN_HPOS_MIN in support/lightpen/amiga_lightpen.cpp.
//
// lpen_vpos == 11'h7FF is userspace saying there is no position to report -- no
// gun, or the pointer is off screen.
//
// The power-up value needs no such marker and userio.v gives it none, which is
// deliberate: the latch comes up 0, and line 0 is inside vblank, so the ~vbl
// gate below refuses it on its own. Initialising the port would have meant a
// declaration assignment in a file Quartus reads as plain Verilog -- exactly
// the kind of thing that simulates and then fails the fit.
wire lpen_valid = (lpen_vpos != 11'h7FF);

always @(posedge clk) begin
	if (clk7_en) begin
		if (reset || ~lpen_en)
			lpen_trig <= 1'b0;
		else if (vblend && eol)
			lpen_trig <= 1'b0;
		else if (~lpen_trig) begin
			if (vpos == 11'd0 && eol) begin
				vpos_lpen <= vpos;
				hpos_lpen <= 9'd2;
				lpen_trig <= 1'b1;
			end
			else if (lpen_valid && ~vbl && vpos == lpen_vpos && hpos[8:1] == lpen_hpos[8:1]) begin
				vpos_lpen <= vpos;
				hpos_lpen <= lpen_hpos;
				lpen_trig <= 1'b1;
			end
		end
	end
end

//composite display blanking
always @(posedge clk) begin
	if (clk7_en) begin
		if (hpos==hbstrt)//start of blanking (active line=51.88us)
			hblank <= 1;
		else if (hpos==hbstop) begin //end of blanking (back porch=5.78us)
			vblank <= vbl;
			hblank <= 0;
		end
	end
end

endmodule
