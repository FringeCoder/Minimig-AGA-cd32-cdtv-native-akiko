// CIA Timer A Module
// 16-bit down counter with multiple operating modes
// Can be used for timing, event counting, and serial port baud rate generation
module cia_timera
(
  input   clk,            // System clock
  input clk7_en,          // 7MHz clock enable for synchronization
  input  wr,              // Write enable signal
  input   reset,          // Synchronous reset

  // Register select signals from address decoder
  input   tlo,            // Timer low byte select ($x4)
  input  thi,             // Timer high byte select ($x5)
  input  tcr,             // Timer control register select ($xE)

  input   [7:0] data_in,  // CPU data bus input
  output   [7:0] data_out, // CPU data bus output
  input  eclk,            // External clock input (usually E clock = system clock/10)
  output  tmra_ovf,       // Timer A overflow signal (used by Timer B cascade mode)
  output  spmode,         // Serial port mode: 0=input, 1=output
  output  irq,            // Timer underflow interrupt request

  // PB6 output, CRA bits 1 and 2. pb_on is PBON: while it is set the port B
  // pin is driven by the timer instead of by the port register. pb_val is
  // OUTMODE: 0 pulses high for the one count the timer underflows on, 1
  // toggles on every underflow.
  output  pb_on,
  output  pb_val,

  // Save state. {tmr, tmlh, tmll, tmcr} -- the counter, both latch
  // bytes and the control register. Exported rather than read over the
  // bus: a timer's value cannot be recovered from the writes that set
  // it, because it counts. ss_ld is taken outside the clk7_en gate --
  // the restore runs with the Amiga's timebase stopped.
  output [38:0] ss_state,
  input         ss_ld,
  input  [38:0] ss_ld_data
);

// Internal registers
reg    [15:0] tmr;        // 16-bit timer counter (counts down)
reg    [7:0] tmlh;        // Timer latch high byte (reload value)
reg    [7:0] tmll;        // Timer latch low byte (reload value)
reg    [6:0] tmcr;        // Timer control register (bit 4 excluded - it's strobe only)
reg    forceload;         // Force load strobe signal
wire  oneshot;            // One-shot mode flag
wire  start;              // Timer start/stop control
reg    thi_load;          // Load timer when high byte written
wire  reload;             // Reload timer from latch
wire  zero;               // Timer reached zero
wire  underflow;          // Timer underflow condition
wire  count;              // Count enable signal

// Count source is always eclk for Timer A
assign count = eclk;

// Timer Control Register (CRA) bit definitions:
// Bit 0: START - Start/stop timer (1=start, 0=stop)
// Bit 1: PBON - drive PB6 from the timer instead of the port register
// Bit 2: OUTMODE - 0=pulse for one count on underflow, 1=toggle on underflow
// Bit 3: RUNMODE - 0=continuous, 1=one-shot
// Bit 4: LOAD - Force load timer from latch (strobe, write-only)
// Bit 5: INMODE - Count source: 0=system clock, 1=CNT pin (simplified to eclk only)
// Bit 6: SPMODE - Serial port mode: 0=input, 1=output (for baud rate generation)
// Bit 7: Unused (TOD clock select on CRB register only)

// Write to control register
always @(posedge clk)
  if (ss_ld)
    tmcr[6:0] <= ss_ld_data[6:0];
  else if (clk7_en) begin
    if (reset)
      tmcr[6:0] <= 7'd0;
    else if (tcr && wr)
      // Load control register, bit 4 (LOAD strobe) is not stored
      tmcr[6:0] <= {data_in[6:5],1'b0,data_in[3:0]};
    else if (thi_load && oneshot)
      // In one-shot mode, writing to timer high byte starts the timer
      tmcr[0] <= 1'd1;
    else if (underflow && oneshot)
      // In one-shot mode, timer stops automatically on underflow
      tmcr[0] <= 1'd0;
  end

// Capture force load strobe (bit 4 of control register write)
always @(posedge clk)
  if (clk7_en) begin
    forceload <= tcr & wr & data_in[4];
  end

// Control register bit aliases for readability
assign oneshot = tmcr[3];    // One-shot mode enable
assign start = tmcr[0];      // Timer start/stop
assign spmode = tmcr[6];     // Serial port mode output

// Timer latch registers - hold reload value
// Low byte latch
always @(posedge clk)
  if (ss_ld)
    tmll[7:0] <= ss_ld_data[14:7];
  else if (clk7_en) begin
    if (reset)
      tmll[7:0] <= 8'b1111_1111;  // Default to $FF
    else if (tlo && wr)
      tmll[7:0] <= data_in[7:0];
  end

// High byte latch
always @(posedge clk)
  if (ss_ld)
    tmlh[7:0] <= ss_ld_data[22:15];
  else if (clk7_en) begin
    if (reset)
      tmlh[7:0] <= 8'b1111_1111;  // Default to $FF
    else if (thi && wr)
      tmlh[7:0] <= data_in[7:0];
  end

reg thi_load_latched;
wire thi_load_eclk= thi_load_latched & eclk;

// Detect write to timer high byte
// In stopped state or one-shot mode, this triggers timer reload
always @(posedge clk)
  if (clk7_en) begin
  	if(eclk)
		  thi_load_latched<=1'b0;
	  if (thi & wr & (~start | oneshot))
		  thi_load_latched <= 1'b1;
    thi_load <= thi & wr & (~start | oneshot);
  end

// Timer reload conditions:
// 1. Force load strobe (bit 4 of control register)
// 2. Write to high byte when stopped or in one-shot mode
// 3. Timer underflow (automatic reload)
assign reload = thi_load_eclk | forceload | underflow;

// 16-bit down counter
always @(posedge clk)
  if (ss_ld)
    tmr[15:0] <= ss_ld_data[38:23];
  else if (clk7_en) begin
    if (reset)
      tmr[15:0] <= 16'hFF_FF;  // Reset to maximum value
    else if (reload)
      tmr[15:0] <= {tmlh[7:0],tmll[7:0]};  // Load from latches
    else if (start && count)
      tmr[15:0] <= tmr[15:0] - 16'd1;     // Count down
  end

// Timer state detection
assign zero = ~|tmr;                    // Timer equals zero
assign underflow = zero & start & count; // Underflow when counting through zero

// Output signals
assign tmra_ovf = underflow;  // Timer A overflow for Timer B cascade mode
assign irq = underflow;       // Interrupt request on underflow

// PB6. The toggle flop is deliberately not in ss_state: see the note in
// rtl/ss_state.vh. It is half a cycle of a timer output on a port pin, and
// capturing it would push the state section past the padding it currently
// fits in and invalidate every save file on disk.
reg pb_tgl;
always @(posedge clk)
  if (clk7_en) begin
    if (reset)          pb_tgl <= 1'b0;
    else if (underflow) pb_tgl <= ~pb_tgl;
  end

assign pb_on  = tmcr[1];
assign pb_val = tmcr[2] ? pb_tgl : underflow;

// CPU read data multiplexer
// Reading timer returns current count
// Reading control register returns current settings (bit 4 always reads as 0)
assign data_out[7:0] = ({8{~wr&tlo}} & tmr[7:0])        // Timer low byte
          | ({8{~wr&thi}} & tmr[15:8])                  // Timer high byte
          | ({8{~wr&tcr}} & {1'b0,tmcr[6:0]});         // Control register


assign ss_state = {tmr[15:0], tmlh[7:0], tmll[7:0], tmcr[6:0]};

endmodule