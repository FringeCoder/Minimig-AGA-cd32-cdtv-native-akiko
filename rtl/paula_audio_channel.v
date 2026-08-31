//This module handles a single amiga audio channel.
// 2020-09-07: pwm controlled volume gated sample output implemented by OKK
// 2026-08-31: ADKCON attach (modulation) modes implemented
//
// Attach mode, ADKCON bits 0..7. Bit n attaches channel n's VOLUME to channel
// n+1, bit 4+n attaches its PERIOD. The modulating channel stops being heard and
// its fetched data words become the next channel's volume or period instead --
// two channels spent for one richer voice, modulated at the audio DMA rate
// rather than at whatever rate the CPU manages.
//
// From WinUAE audio.cpp. loaddat():
//
//     int audav = adkcon & (0x01 << nr);
//     int audap = adkcon & (0x10 << nr);
//     if (audav || (modper && audap)) {
//         if (nr >= 3) return;
//         if (modper && audap)  cdp[1].per = ...cdp->dat...;
//         else if (audav)       update_volume(nr + 1, cdp->dat);
//     }
//
// and the two call sites that give the timing: loaddat(nr, true) on the 2->3
// state transition (period), plain loaddat(nr) on 3->2 (volume). WinUAE's states
// 2 and 3 are the high and low sample of the fetched word, which are our
// AUDIO_STATE_3 and AUDIO_STATE_4, so the period strobe is 3->4 and the volume
// strobe is 4->3.
//
// Silencing is separate, and it is not conditional on there being a channel to
// modulate -- audio_update_adkmasks() masks channel n's output whenever either
// of its attach bits is set, channel 3 included, where the modulation itself
// does nothing.

module paula_audio_channel
(
  input   clk,          //bus clock
  input clk7_en,
  input  cck,          //colour clock enable
  input   reset,            //reset
  input  aen,          //address enable
  input  dmaena,          //dma enable
  input  [3:1] reg_address_in,    //register address input
  input   [15:0] data,       //bus data input
  input   att_vol,        //ADKCON: this channel's volume is attached to the next
  input   att_per,        //ADKCON: this channel's period is attached to the next
  input   mod_vol_we,      //previous channel is writing our volume
  input   mod_per_we,      //previous channel is writing our period
  input   [15:0] mod_data,    //previous channel's data word
  output  reg mod_vol_stb,    //we are writing the next channel's volume
  output  reg mod_per_stb,    //we are writing the next channel's period
  output  [15:0] mod_dout,    //our data word, for the next channel
  output  [6:0] volume,      //channel volume output
  output  [7:0] sample,      //channel sample output
  output  [7:0] sample_okk,  //channel sample output (PWM gated by volumectr like suggested in the HW manual)
  output  intreq,          //interrupt request
  input  intpen,          //interrupt pending input
  output  reg dmareq,        //dma request
  output  reg dmas,        //dma special (restart)
  input  strhor          //horizontal strobe
);

//register names and addresses
parameter  AUDLEN = 4'h4;
parameter  AUDPER = 4'h6;
parameter  AUDVOL = 4'h8;
parameter  AUDDAT = 4'ha;

//local signals
reg    [15:0] audlen;      //audio length register
reg    [15:0] audper;      //audio period register
reg    [6:0] audvol;      //audio volume register
reg    [15:0] auddat;      //audio data register

reg    [15:0] datbuf;      //audio data buffer
reg    [2:0] audio_state;    //audio current state
reg    [2:0] audio_next;     //audio next state

wire   datwrite;        //data register is written

reg    [5:0] volcnt;	   // pwm volume counter
wire   volcount;	      // increase pwm volume counter
reg    volcntrld;       // not used (BS! Oh yes it's used!!!)
wire   volgate;		   // pwm pulse for gating sample output

reg    pbufld1;        //load output sample from sample buffer

reg    [15:0] percnt;      //audio period counter
reg    percount;        //decrease period counter
reg    percntrld;        //reload period counter
wire  perfin;          //period counter expired

reg    [15:0] lencnt;      //audio length counter
reg    lencount;        //decrease length counter
reg    lencntrld;        //reload length counter
wire  lenfin;          //length counter expired

reg   AUDxDAT;        //audio data buffer was written
wire  AUDxON;          //audio DMA channel is enabled
reg    AUDxDR;          //audio DMA request
reg    AUDxIR;          //audio interrupt request
wire  AUDxIP;          //audio interrupt is pending

reg    intreq2_set;
reg    intreq2_clr;
reg    intreq2;        //buffered interrupt request

reg    dmasen;          //pointer register reloading request
reg    penhi;          //enable high byte of sample buffer

reg silence;  // AMR: disable audio if repeat length is 1
reg silence_d;  // AMR: disable audio if repeat length is 1
reg dmaena_d;


//length register bus write
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset)
      audlen[15:0] <= 16'h00_00;
    else if (aen && (reg_address_in[3:1]==AUDLEN[3:1]))
      audlen[15:0] <= data[15:0];
  end
end

//period register bus write
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset)
      audper[15:0] <= 16'h00_00;
    else if (aen && (reg_address_in[3:1]==AUDPER[3:1]))
      audper[15:0] <= data[15:0];
    else if (mod_per_we)  //period modulated by the previous channel
      audper[15:0] <= mod_data[15:0];
  end
end

//volume register bus write
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset)
      audvol[6:0] <= 7'b000_0000;
    else if (aen && (reg_address_in[3:1]==AUDVOL[3:1]))
      audvol[6:0] <= data[6:0];
    else if (mod_vol_we)  //volume modulated by the previous channel
      audvol[6:0] <= mod_data[6:0];
  end
end

//data register strobe
assign datwrite = (aen && (reg_address_in[3:1]==AUDDAT[3:1])) ? 1'b1 : 1'b0;

//data register bus write
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset)
      auddat[15:0] <= 16'h00_00;
    else if (datwrite)
      auddat[15:0] <= data[15:0];
  end
end

always @(posedge clk) begin
  if (clk7_en) begin
    if (datwrite)
      AUDxDAT <= 1'b1;
    else if (cck)
      AUDxDAT <= 1'b0;
  end
end

assign  AUDxON = dmaena;  //dma enable

assign  AUDxIP = intpen;  //audio interrupt pending

assign intreq = AUDxIR;    //audio interrupt request


//volume counter
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset && cck) volcnt <= 6'h00;

    else if (volcntrld && cck)    // load volume counter from audio volume register
      volcnt[5:0] <= audvol[5:0];
    else if (volcount && cck)     // volume counter count up
      volcnt[5:0] <= volcnt[5:0] - 6'd1;
  end
end

assign volcount = 1'd1;
assign volgate = (volcnt < audvol);

//period counter
always @(posedge clk) begin
  if (clk7_en) begin
    if (percntrld && cck)//load period counter from audio period register
      percnt[15:0] <= audper[15:0];
    else if (percount && cck)//period counter count down
      percnt[15:0] <= percnt[15:0] - 16'd1;
  end
end

assign perfin = (percnt[15:0]==1 && cck) ? 1'b1 : 1'b0;

//length counter
always @(posedge clk) begin
  if (clk7_en) begin
    if (lencntrld && cck) begin //load length counter from audio length register
      lencnt[15:0] <= (audlen[15:0]);
      silence<=1'b0;
      if(audlen==1 || audlen==0)
         silence<=1'b1;
    end else if (lencount && cck)//length counter count down
      lencnt[15:0] <= (lencnt[15:0] - 1'd1);
    // Silence fix
    dmaena_d<=dmaena;
    if(dmaena_d==1'b1 && dmaena==1'b0) begin
      silence_d<=1'b1; // Prevent next write from unsilencing the channel.
      silence<=1'b1;
    end
    if(AUDxDAT && cck)  // Unsilence the channel if the CPU writes to AUDxDAT
      if(silence_d)
        silence_d<=1'b0;
      else
        silence<=1'b0;
  end
end

assign lenfin = (lencnt[15:0]==1 && cck) ? 1'b1 : 1'b0;


//audio buffer
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset)
      datbuf[15:0] <= 16'h00_00;
    else if (pbufld1 && cck)
      datbuf[15:0] <= auddat[15:0];
  end
end

// A channel with either attach bit set is not heard: its data is the next
// channel's volume or period, not a sample. WinUAE audio_update_adkmasks():
//
//     unsigned long t = adkcon | (adkcon >> 4);
//     audio_channel[0].data.adk_mask = (((t >> 0) & 1) - 1);
//
// which is a full mask when neither bit is set and zero when either is.
wire attached = att_vol | att_per;

//assign sample[7:0] = penhi ? datbuf[15:8] : datbuf[7:0];
assign sample[7:0] = (silence | attached) ? 8'b0 : (penhi ? datbuf[15:8] : datbuf[7:0]);

// By: OKK (The correct way Paula really works!)
assign sample_okk[7:0] = (silence | attached | !volgate ? 8'b0 : (penhi ? datbuf[15:8] : datbuf[7:0])); // pwm volume gated sample output

// Modulation strobes. The next channel's registers are loaded from the word
// this channel just fetched, on the same two state transitions WinUAE hangs
// loaddat() off. Registered rather than decoded inside the transition function
// so the FSM's output list is left alone -- every branch there assigns every
// output, and two more would be two more chances to leave a latch behind.
assign mod_dout[15:0] = auddat[15:0];

always @(posedge clk) begin
  if (clk7_en) begin
    mod_per_stb <= 1'b0;
    mod_vol_stb <= 1'b0;
    if (reset) begin
      mod_per_stb <= 1'b0;
      mod_vol_stb <= 1'b0;
    end else if (cck) begin
      if (audio_state == AUDIO_STATE_3 && audio_next == AUDIO_STATE_4)
        mod_per_stb <= att_per;
      if (audio_state == AUDIO_STATE_4 && audio_next == AUDIO_STATE_3)
        mod_vol_stb <= att_vol;
    end
  end
end

//volume output
assign volume[6:0] = audvol[6:0];

//dma request logic
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset)
    begin
      dmareq <= 1'b0;
      dmas <= 1'b0;
    end
    else if (AUDxDR && cck)
    begin
      dmareq <= 1'b1;
      dmas <= dmasen | lenfin;
    end
    else if (strhor) //dma request are cleared when transfered to Agnus
    begin
      dmareq <= 1'b0;
      dmas <= 1'b0;
    end
  end
end

//buffered interrupt request
always @(posedge clk) begin
  if (clk7_en) begin
    if (cck)
      if (intreq2_set)
        intreq2 <= 1'b1;
      else if (intreq2_clr)
        intreq2 <= 1'b0;
  end
end

//audio states
parameter AUDIO_STATE_0 = 3'b000;
parameter AUDIO_STATE_1 = 3'b001;
parameter AUDIO_STATE_2 = 3'b011;
parameter AUDIO_STATE_3 = 3'b010;
parameter AUDIO_STATE_4 = 3'b110;

//audio channel state machine
always @(posedge clk) begin
  if (clk7_en) begin
    if (reset)
      audio_state <= AUDIO_STATE_0;
    else if (cck)
      audio_state <= audio_next;
  end
end

//transition function
always @(*) begin
  case (audio_state)

    AUDIO_STATE_0: //audio FSM idle state
    begin
      intreq2_clr = 1'b1;
      intreq2_set = 1'b0;
      lencount = 1'b0;
      penhi = 1'b0;
      percount = 1'b0;
      percntrld = 1'b1;

      if (AUDxON) //start of DMA driven audio playback
      begin
        audio_next = AUDIO_STATE_1;
        AUDxDR = 1'b1;
        AUDxIR = 1'b0;
        dmasen = 1'b1;
        lencntrld = 1'b1;
        pbufld1 = 1'b0;
        volcntrld = 1'b0;
      end
      else if (AUDxDAT && !AUDxON && !AUDxIP)  //CPU driven audio playback
      begin
        audio_next = AUDIO_STATE_3;
        AUDxDR = 1'b0;
        AUDxIR = 1'b1;
        dmasen = 1'b0;
        lencntrld = 1'b0;
        pbufld1 = 1'b1;
        volcntrld = 1'b1;
      end
      else
      begin
        audio_next = AUDIO_STATE_0;
        AUDxDR = 1'b0;
        AUDxIR = 1'b0;
        dmasen = 1'b0;
        lencntrld = 1'b0;
        pbufld1 = 1'b0;
        volcntrld = 1'b0;
      end
    end

    AUDIO_STATE_1: //audio DMA has been enabled
    begin
      dmasen = 1'b0;
      intreq2_clr = 1'b1;
      intreq2_set = 1'b0;
      lencntrld = 1'b0;
      penhi = 1'b0;
      percount = 1'b0;

      if (AUDxON && AUDxDAT) //requested data has arrived
      begin
        audio_next = AUDIO_STATE_2;
        AUDxDR = 1'b1;
        AUDxIR = 1'b1;
        lencount = ~lenfin;
        pbufld1 = 1'b0;  //first data received, discard it since first data access is used to reload pointer
        percntrld = 1'b0;
        volcntrld = 1'b0;
      end
      else if (!AUDxON) //audio DMA has been switched off so go to IDLE state
      begin
        audio_next = AUDIO_STATE_0;
        AUDxDR = 1'b0;
        AUDxIR = 1'b0;
        lencount = 1'b0;
        pbufld1 = 1'b0;
        percntrld = 1'b0;
        volcntrld = 1'b0;
      end
      else
      begin
        audio_next = AUDIO_STATE_1;
        AUDxDR = 1'b0;
        AUDxIR = 1'b0;
        lencount = 1'b0;
        pbufld1 = 1'b0;
        percntrld = 1'b0;
        volcntrld = 1'b0;
      end
    end

    AUDIO_STATE_2: //audio DMA has been enabled
    begin
      dmasen = 1'b0;
      intreq2_clr = 1'b1;
      intreq2_set = 1'b0;
      lencntrld = 1'b0;
      penhi = 1'b0;
      percount = 1'b0;

      if (AUDxON && AUDxDAT) //requested data has arrived
      begin
        audio_next = AUDIO_STATE_3;
        AUDxDR = 1'b1;
        AUDxIR = 1'b0;
        lencount = ~lenfin;
        pbufld1 = 1'b1;  //new data has been just received so put it in the output buffer
        percntrld = 1'b1;
        volcntrld = 1'b1;
      end
      else if (!AUDxON) //audio DMA has been switched off so go to IDLE state
      begin
        audio_next = AUDIO_STATE_0;
        AUDxDR = 1'b0;
        AUDxIR = 1'b0;
        lencount = 1'b0;
        pbufld1 = 1'b0;
        percntrld = 1'b0;
        volcntrld = 1'b0;
      end
      else
      begin
        audio_next = AUDIO_STATE_2;
        AUDxDR = 1'b0;
        AUDxIR = 1'b0;
        lencount = 1'b0;
        pbufld1 = 1'b0;
        percntrld = 1'b0;
        volcntrld = 1'b0;
      end
    end

    AUDIO_STATE_3: //first sample is being output
    begin
      AUDxDR = 1'b0;
      AUDxIR = 1'b0;
      dmasen = 1'b0;
      intreq2_clr = 1'b0;
      intreq2_set = lenfin & AUDxON & AUDxDAT;
      lencount = ~lenfin & AUDxON & AUDxDAT;
      lencntrld = lenfin & AUDxON & AUDxDAT;
      pbufld1 = 1'b0;
      penhi = 1'b1;
      volcntrld = 1'b0;

      if (perfin) //if period counter expired output other sample from buffer
      begin
        audio_next = AUDIO_STATE_4;
        percount = 1'b0;
        percntrld = 1'b1;
      end
      else
      begin
        audio_next = AUDIO_STATE_3;
        percount = 1'b1;
        percntrld = 1'b0;
      end
    end

    AUDIO_STATE_4: //second sample is being output
    begin
      dmasen = 1'b0;
      intreq2_set = lenfin & AUDxON & AUDxDAT;
      lencount = ~lenfin & AUDxON & AUDxDAT;
      lencntrld = lenfin & AUDxON & AUDxDAT;
      penhi = 1'b0;
      volcntrld = 1'b0;

      if (perfin && (AUDxON || !AUDxIP)) //period counter expired and audio DMA active
      begin
        audio_next = AUDIO_STATE_3;
        AUDxDR = AUDxON;
        AUDxIR = (intreq2 & AUDxON) | ~AUDxON;
        intreq2_clr = intreq2;
        pbufld1 = 1'b1;
        percount = 1'b0;
        percntrld = 1'b1;
      end
      else if (perfin && !AUDxON && AUDxIP) //period counter expired and audio DMA inactive
      begin
        audio_next = AUDIO_STATE_0;
        AUDxDR = 1'b0;
        AUDxIR = 1'b0;
        intreq2_clr = 1'b0;
        pbufld1 = 1'b0;
        percount = 1'b0;
        percntrld = 1'b0;
      end
      else
      begin
        audio_next = AUDIO_STATE_4;
        AUDxDR = 1'b0;
        AUDxIR = 1'b0;
        intreq2_clr = 1'b0;
        pbufld1 = 1'b0;
        percount = 1'b1;
        percntrld = 1'b0;
      end
    end

    default:
    begin
      audio_next = AUDIO_STATE_0;
      AUDxDR = 1'b0;
      AUDxIR = 1'b0;
      dmasen = 1'b0;
      intreq2_clr = 1'b0;
      intreq2_set = 1'b0;
      lencntrld = 1'b0;
      lencount = 1'b0;
      pbufld1 = 1'b0;
      penhi = 1'b0;
      percount = 1'b0;
      percntrld = 1'b0;
      volcntrld = 1'b0;
    end

  endcase
end


endmodule

