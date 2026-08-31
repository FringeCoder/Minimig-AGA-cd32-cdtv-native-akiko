// CIA B (Complex Interface Adapter B)
// MOS 8520 CIA implementation for Amiga computers
//
// CIA B handles:
// - Serial port control (RS-232)
// - Disk drive selection and motor control
// - Disk step and direction signals
// - FLAG input for disk index detection
// - Two 16-bit timers
// - Time-of-day clock with alarm
// - Interrupt generation (generates INT6)
//
// Memory mapped at $BFD000-$BFDF00 (even addresses)

module ciab
(
  input   clk,              // System clock
  input clk7_en,            // 7MHz clock enable
  input   aen,              // Address enable (chip select)
  input  rd,                // Read enable
  input  wr,                // Write enable
  input   reset,            // System reset
  input   [3:0] rs,         // Register select (address bits)
  input   [7:0] data_in,    // CPU data bus input
  output   [7:0] data_out,  // CPU data bus output
  input   tick,             // TOD tick input (50/60 Hz)
  input   eclk,             // E clock (system clock / 10)
  input   flag,             // FLAG input (disk index pulse)
  output   irq,             // Interrupt request (INT6)

  // Port A connections (serial and disk control)
  input  [5:0] porta_in,    // Port A inputs
  output   [7:6] porta_out, // Port A outputs
  // Bit 0: /RTS - RS-232 Request To Send (input)
  // Bit 1: /CD  - RS-232 Carrier Detect (input)
  // Bit 2: /CTS - RS-232 Clear To Send (input)
  // Bit 3: /DSR - RS-232 Data Set Ready (input)
  // Bit 4: SEL  - Centronics Select (input)
  // Bit 5: POUT - Centronics Paper Out (input)
  // Bit 6: /DTR - RS-232 Data Terminal Ready (output)
  // Bit 7: /RE  - RS-232 Ring Indicator (output)

  // Save state. Same reasoning as ciaa.v: the CIAs are on the CPU bus, and
  // reading ICR or TOD to capture them would change the machine. Layout,
  // low bits first:
  //   [7:0]     regporta      port A output register
  //   [15:8]    ddrporta      port A direction
  //   [23:16]   regportb      port B output register
  //   [31:24]   ddrportb      port B direction
  //   [39:32]   sdr_latch     serial data register
  //   [49:40]   cia_int       {icrmask, icr}
  //   [88:50]   cia_timera    {tmr, tmlh, tmll, tmcr}
  //   [127:89]  cia_timerb    same shape as timer A
  //   [202:128] cia_timerd    {tod, alarm, tod_latch, crb7, count_ena, latch_ena}
  output [202:0] ss_state,
  input          ss_ld,
  input  [202:0] ss_ld_data,

  // Port B - Disk drive control signals
  output  [7:0] portb_out   // Port B outputs
  // Bit 0: /STEP - Disk head step pulse
  // Bit 1: DIR   - Disk head direction (0=out, 1=in)
  // Bit 2: /SIDE - Disk side select (0=upper, 1=lower)
  // Bit 3: /SEL0 - Select drive 0 (internal)
  // Bit 4: /SEL1 - Select drive 1 (external)
  // Bit 5: /SEL2 - Select drive 2 (external)
  // Bit 6: /SEL3 - Select drive 3 (external)
  // Bit 7: /MTR  - Disk motor on/off
);

// Internal signal declarations
  wire   [7:0] icr_out;     // Interrupt control register output
  wire  [7:0] tmra_out;     // Timer A data output
  wire  [7:0] tmrb_out;     // Timer B data output
  wire  [7:0] tmrd_out;     // Timer D (TOD) data output
  reg    [7:0] pa_out;      // Port A data output
  reg    [7:0] pb_out;      // Port B data output
  wire  alrm;               // TOD alarm interrupt
  wire  ta;                 // Timer A interrupt
  wire  tb;                 // Timer B interrupt
  wire  tmra_ovf;           // Timer A underflow signal

  reg    [7:0] sdr_latch;   // Serial data register
  wire  [7:0] sdr_out;      // SDR output
  wire  spmode;             // Timer A serial port mode (0=input, 1=output)
  wire  ta_pb_on, ta_pb_val;  // Timer A driving PB6
  wire  tb_pb_on, tb_pb_val;  // Timer B driving PB7

  reg    tick_del;          // Delayed tick for edge detection

//----------------------------------------------------------------------------------
// Address decoder for CIA registers
//----------------------------------------------------------------------------------
  wire  pra,prb,ddra,ddrb,cra,talo,tahi,crb,tblo,tbhi,tdlo,tdme,tdhi,sdr,icrs;
  wire  enable;

assign enable = aen & (rd | wr);

// CIA B Register Map (same offsets as CIA A):
// $BFD000 - PRA   - Port A data (serial port control)
// $BFD100 - PRB   - Port B data (disk drive control)
// $BFD200 - DDRA  - Port A direction
// $BFD300 - DDRB  - Port B direction
// $BFD400 - TALO  - Timer A low byte
// $BFD500 - TAHI  - Timer A high byte
// $BFD600 - TBLO  - Timer B low byte
// $BFD700 - TBHI  - Timer B high byte
// $BFD800 - TDLO  - TOD low byte (1/10 seconds)
// $BFD900 - TDME  - TOD middle byte (seconds)
// $BFDA00 - TDHI  - TOD high byte (minutes)
// $BFDC00 - SDR   - Serial data register (unused)
// $BFDD00 - ICR   - Interrupt control register
// $BFDE00 - CRA   - Control register A
// $BFDF00 - CRB   - Control register B

// Generate register select signals
assign  pra  = (enable && rs==4'h0) ? 1'b1 : 1'b0;
assign  prb  = (enable && rs==4'h1) ? 1'b1 : 1'b0;
assign  ddra = (enable && rs==4'h2) ? 1'b1 : 1'b0;
assign  ddrb = (enable && rs==4'h3) ? 1'b1 : 1'b0;
assign  talo = (enable && rs==4'h4) ? 1'b1 : 1'b0;
assign  tahi = (enable && rs==4'h5) ? 1'b1 : 1'b0;
assign  tblo = (enable && rs==4'h6) ? 1'b1 : 1'b0;
assign  tbhi = (enable && rs==4'h7) ? 1'b1 : 1'b0;
assign  tdlo = (enable && rs==4'h8) ? 1'b1 : 1'b0;
assign  tdme = (enable && rs==4'h9) ? 1'b1 : 1'b0;
assign  tdhi = (enable && rs==4'hA) ? 1'b1 : 1'b0;
assign  sdr  = (enable && rs==4'hC) ? 1'b1 : 1'b0;
assign  icrs = (enable && rs==4'hD) ? 1'b1 : 1'b0;
assign  cra  = (enable && rs==4'hE) ? 1'b1 : 1'b0;
assign  crb  = (enable && rs==4'hF) ? 1'b1 : 1'b0;

// Data output multiplexer - OR together all module outputs
assign data_out = icr_out | tmra_out | tmrb_out | tmrd_out | sdr_out | pb_out | pa_out;

// Serial port data register.
//
// The shift register is clocked by timer A underflow, not by the CNT pin.
// WinUAE cia.cpp does the shift inside the timer A underflow path, gated on
// (cr & (CR_SPMODE | CR_RUNMODE)) == CR_SPMODE -- in output mode the CIA
// GENERATES CNT rather than receiving it, so nothing external is needed. That
// is what makes this implementable here: CIA-B's CNT pin goes to the expansion
// bus and is unconnected on a stock Amiga, so input mode has no source on real
// hardware either and an input-mode read returns whatever was last shifted in,
// which is nothing.
//
// A write while a shift is running is held and starts when the current byte
// finishes, again following WinUAE's sdr_load.
//
// sdr_buf, sdr_cnt and sdr_load are deliberately NOT in ss_state -- see the
// note in rtl/ss_state.vh. A save taken mid-transmission restores with the
// shifter idle and that byte's SP interrupt lost. That is strictly better than
// what this did before, which was to never transmit and never interrupt at all.
always @(posedge clk)
  if (ss_ld)
    sdr_latch[7:0] <= ss_ld_data[39:32];
  else if (clk7_en) begin
    if (reset)
      sdr_latch[7:0] <= 8'h00;
    else if (wr & sdr)
      sdr_latch[7:0] <= data_in[7:0];
  end

reg [7:0] sdr_buf;    // the byte being shifted out
reg [3:0] sdr_cnt;    // bits left to shift, 0 = idle
reg       sdr_load;   // a byte is waiting for the current one to finish
reg       ser_int;    // SP interrupt: a byte has finished

always @(posedge clk)
  if (clk7_en) begin
    ser_int <= 1'b0;
    if (reset) begin
      sdr_buf  <= 8'h00;
      sdr_cnt  <= 4'd0;
      sdr_load <= 1'b0;
    end
    else if (!spmode) begin
      // Input mode. Nothing drives CNT on this machine, so the shifter simply
      // does not run; drop any transmission in progress rather than leaving it
      // half done across a mode change.
      sdr_cnt  <= 4'd0;
      sdr_load <= 1'b0;
    end
    else begin
      if (wr & sdr) begin
        if (sdr_cnt == 4'd0) begin
          sdr_buf <= data_in[7:0];
          sdr_cnt <= 4'd8;
        end
        else sdr_load <= 1'b1;
      end

      if (ta && sdr_cnt != 4'd0) begin
        sdr_buf <= {sdr_buf[6:0], 1'b0};
        sdr_cnt <= sdr_cnt - 4'd1;
        if (sdr_cnt == 4'd1) begin
          ser_int <= 1'b1;
          if (sdr_load) begin
            sdr_buf  <= sdr_latch[7:0];
            sdr_cnt  <= 4'd8;
            sdr_load <= 1'b0;
          end
        end
      end
    end
  end

// SDR read returns last written value
assign sdr_out = (!wr && sdr) ? sdr_latch[7:0] : 8'h00;

//----------------------------------------------------------------------------------
// Port A - Serial port control signals
//----------------------------------------------------------------------------------
reg [5:0] porta_in2;        // Synchronized input data
reg [7:0] regporta;         // Port A output register
reg [7:0] ddrporta;         // Port A direction register

// Synchronize external inputs
always @(posedge clk)
  if (clk7_en) begin
    porta_in2 <= porta_in;
  end

// Port A output register
always @(posedge clk)
  if (ss_ld)
    regporta[7:0] <= ss_ld_data[7:0];
  else if (clk7_en) begin
    if (reset)
      regporta[7:0] <= 8'd0;
    else if (wr && pra)
      regporta[7:0] <= data_in[7:0];
  end

// Port A direction register
always @(posedge clk)
  if (ss_ld)
    ddrporta[7:0] <= ss_ld_data[15:8];
  else if (clk7_en) begin
    if (reset)
      ddrporta[7:0] <= 8'd0;
    else if (wr && ddra)
       ddrporta[7:0] <= data_in[7:0];
  end

// Port A read multiplexer
always @(*)
begin
  if (!wr && pra)
    pa_out[7:0] = {porta_out[7:6],porta_in2[5:0]}; // Mix outputs and inputs
  else if (!wr && ddra)
    pa_out[7:0] = ddrporta[7:0];                   // Read direction register
  else
    pa_out[7:0] = 8'h00;
end

// Port A outputs (only bits 7:6 are outputs on CIA B)
// Pull-ups ensure undriven pins read as 1
assign porta_out[7:6] = (~ddrporta[7:6]) | regporta[7:6];

//----------------------------------------------------------------------------------
// Port B - Disk drive control (all outputs)
//----------------------------------------------------------------------------------
reg [7:0] regportb;         // Port B output register
reg [7:0] ddrportb;         // Port B direction register

// Port B output register
always @(posedge clk)
  if (ss_ld)
    regportb[7:0] <= ss_ld_data[23:16];
  else if (clk7_en) begin
    if (reset)
      regportb[7:0] <= 8'd0;
    else if (wr && prb)
      regportb[7:0] <= data_in[7:0];
  end

// Port B direction register
always @(posedge clk)
  if (ss_ld)
    ddrportb[7:0] <= ss_ld_data[31:24];
  else if (clk7_en) begin
    if (reset)
      ddrportb[7:0] <= 8'd0;
    else if (wr && ddrb)
       ddrportb[7:0] <= data_in[7:0];
  end

// Port B read multiplexer
always @(*)
begin
  if (!wr && prb)
    pb_out[7:0] = portb_out[7:0];  // Read output state
  else if (!wr && ddrb)
    pb_out[7:0] = ddrportb[7:0];   // Read direction register
  else
    pb_out[7:0] = 8'h00;
end

// Port B outputs with pull-up simulation
// All bits are typically configured as outputs for disk control.
//
// PB6 and PB7 can be driven by the timers instead of by the port register when
// PBON is set, exactly as on CIA-A. Note what those two pins are here: /SEL3 and
// /MTR. A program that sets PBON on CIA-B is driving the drive-select and motor
// lines from a timer, which on a real Amiga is just as true and just as
// destructive. This is faithful, not safe -- and no Amiga software does it,
// which is why it has gone unnoticed since 2005.
wire [7:0] portb_pins = (~ddrportb[7:0]) | regportb[7:0];

assign portb_out[7:0] = {tb_pb_on ? tb_pb_val : portb_pins[7],
                         ta_pb_on ? ta_pb_val : portb_pins[6],
                         portb_pins[5:0]};

// Delayed tick signal for edge detection
always @(posedge clk)
  if (clk7_en) begin
    tick_del <= tick;
  end

//----------------------------------------------------------------------------------
// Instantiate sub-modules
//----------------------------------------------------------------------------------

// Interrupt controller
cia_int cnt
(
  .clk(clk),
  .clk7_en(clk7_en),
  .wr(wr),
  .reset(reset),
  .icrs(icrs),
  .ta(ta),
  .tb(tb),
  .alrm(alrm),
  .flag(flag),              // Disk index pulse interrupt
  .ser(ser_int),            // SP: a serial byte has finished shifting
  .data_in(data_in),
  .data_out(icr_out),
  .irq(irq),
  .ss_state(ss_cnt),
  .ss_ld(ss_ld),
  .ss_ld_data(ss_ld_data[49:40])
);

// Timer A - General purpose timer
cia_timera tmra
(
  .clk(clk),
  .clk7_en(clk7_en),
  .wr(wr),
  .reset(reset),
  .tlo(talo),
  .thi(tahi),
  .tcr(cra),
  .data_in(data_in),
  .data_out(tmra_out),
  .eclk(eclk),
  .tmra_ovf(tmra_ovf),
  .spmode(spmode),
  .irq(ta),
  .pb_on(ta_pb_on),
  .pb_val(ta_pb_val),
  .ss_state(ss_tmra),
  .ss_ld(ss_ld),
  .ss_ld_data(ss_ld_data[88:50])
);

// Timer B - General purpose timer, can cascade with Timer A
cia_timerb tmrb
(
  .clk(clk),
  .clk7_en(clk7_en),
  .wr(wr),
  .reset(reset),
  .tlo(tblo),
  .thi(tbhi),
  .tcr(crb),
  .data_in(data_in),
  .data_out(tmrb_out),
  .eclk(eclk),
  .tmra_ovf(tmra_ovf),
  .irq(tb),
  .pb_on(tb_pb_on),
  .pb_val(tb_pb_val),
  .ss_state(ss_tmrb),
  .ss_ld(ss_ld),
  .ss_ld_data(ss_ld_data[127:89])
);

// Timer D - Time of Day clock with alarm
cia_timerd tmrd
(
  .clk(clk),
  .clk7_en(clk7_en),
  .wr(wr),
  .reset(reset),
  .tlo(tdlo),
  .tme(tdme),
  .thi(tdhi),
  .tcr(crb),
  .data_in(data_in),
  .data_out(tmrd_out),
  .count(tick & ~tick_del),  // Count on rising edge of tick
  .irq(alrm),
  .ss_state(ss_tmrd),
  .ss_ld(ss_ld),
  .ss_ld_data(ss_ld_data[202:128])
);


// Save state: the sub-modules' state, gathered.
wire  [9:0] ss_cnt;
wire [38:0] ss_tmra;
wire [38:0] ss_tmrb;
wire [74:0] ss_tmrd;

assign ss_state = {ss_tmrd, ss_tmrb, ss_tmra, ss_cnt, sdr_latch[7:0],
                   ddrportb[7:0], regportb[7:0], ddrporta[7:0], regporta[7:0]};

endmodule