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
// This is the bitplane parallel to serial converter & scroller               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


module denise_bitplane_shifter
(
  input  wire           clk,      // 35ns pixel clock
  input  wire           clk7_en,  // 7MHz clock enable
  input  wire           c1,       // clock phase signals
  input  wire           c3,       // clock phase signals
  input  wire           load,     // load shift register signal
  input  wire           hires,    // high resolution select
  input  wire           shres,    // super high resolution select (takes priority over hires)
  input  wire [  2-1:0] fmode,    // AGA fetch mode
  input  wire           aga,
  input  wire [ 64-1:0] data_in,  // parallel load data input
  input  wire [  8-1:0] scroll,   // scrolling value
  output                out       // shift register output
);


// local signals
reg  [ 6-1:0] fmode_mask;         // fetchmode mask
reg  [64-1:0] shifter;            // main shifter
reg  [64-1:0] scroller;           // scroller shifter
reg           shift;              // shifter enable
reg  [ 6-1:0] select;             // shifter pixel select
wire          scroller_out;       // scroller output
reg  [ 8-1:0] sh_scroller;        // superhires scroller
reg  [ 3-1:0] sh_select;          // superhires scroller pixel select


// fetchmode mask
always @ (*) begin
  case(fmode[1:0])
    2'b00 : fmode_mask = 6'b00_1111;
    2'b01,
    2'b10 : fmode_mask = 6'b01_1111;
    2'b11 : fmode_mask = 6'b11_1111;
  endcase
end

// main shifter and scroller control
always @ (*) begin
  if (shres) begin
    // super hires mode
    shift = 1'b1; // shifter always enabled
    select[5:0] = scroll[5:0] & fmode_mask;
  end else if (hires) begin
    // hires mode
    shift = ~c1 ^ c3; // shifter enabled every other clock cycle
    select[5:0] = scroll[6:1] & fmode_mask;
  end else begin
    // lowres mode
    shift = ~c1 & ~c3; // shifter enabled once every 4 clock cycles
    select[5:0] = scroll[7:2] & fmode_mask;
  end
end


// main shifter
always @ (posedge clk) begin
  if (load && !c1 && !c3) begin
    // load new data into shifter
    shifter[63:0] <= data_in[63:0];
  end else if (shift) begin
    // shift already loaded data
    shifter[63:0] <= {shifter[62:0],1'b0};
  end
end


// main scroller
always @ (posedge clk) begin
  if (shift) begin
    // shift scroller data
    scroller[63:0] <= {scroller[62:0],shifter[63]};
  end
end


// main scroller output
assign scroller_out = scroller[select];


// superhires scroller control
//
// TODO test if this is correct -- and note, checked 2026-08-31, that WinUAE
// cannot settle it. This delay line is Minimig's own compensation for a Denise
// pipeline that is not cycle exact; WinUAE has no corresponding structure to
// compare a tap against, because it computes pixel positions directly rather
// than delaying a stream to line them up. So the right tap is a property of
// THIS implementation's latency, and the only instrument that can read it is the
// display.
//
// Worth writing down what looks inconsistent, for whoever gets to a screen: the
// lores case selects {aga, scroll[1:0]}, an AGA base of 4 plus the scroll, while
// the hires case selects {aga, scroll[0], 1'b1}, an AGA base of 5 or 7. The
// bases differ. Whether that is the bug or the compensation is exactly what
// needs looking at, and guessing at it is how the light pen trigger cost two
// rounds before someone measured it.
always @ (*) begin
  if (shres) begin
    sh_select = aga ? 3'b110 : 3'b011;
  end else if (hires) begin
    sh_select = {aga, scroll[0], 1'b1}; // MSB bit should probably be 0, this is a hack for kickstart screen ...
  end else begin
    sh_select = {aga, scroll[1:0]};
  end
end


// superhires scroller
always @ (posedge clk) begin
  sh_scroller[7:0] <= {sh_scroller[6:0], scroller_out};
end


// superhires scroller output
assign out = sh_scroller[sh_select];


endmodule

