// tb_sdram_fwd -- posted-write -> chip-read store-to-load forwarding in sdram_ctrl.
//
// Strategy: hold chipDMA=0 so the CHIP slot ALWAYS wins state-0 arbitration; the
// posted CPU write then never gets a CPU_WRITECACHE slot and stays pending
// (write_req=1) for the whole test, so the forwarding path is continuously
// exercised and the DUT never drives sd_data (no committed write) -- the bench
// owns sd_data and feeds a deterministic per-state read pattern.
//
// For each of the 4 burst positions p (0..3) and a set of byte-enable masks we
// post a CPU write to word (block|p), issue a CHIP read burst of `block`, and
// check the captured {chipRD,chip48_1,chip48_2,chip48_3}: position p must show
// writeDat merged per byte-enable; the other 3 must equal the raw SDRAM pattern.

`timescale 1ns/1ps
module tb_sdram_fwd;

  integer errs = 0;

  reg sysclk = 0;
  always #5 sysclk = ~sysclk;            // 100 MHz-ish; ratios are what matter

  // c_7m: rising edge every 16 sysclk -> sdram_state runs 0..15 then resets
  reg [3:0] clkdiv = 0;
  reg c_7m = 0;
  always @(posedge sysclk) begin
    clkdiv <= clkdiv + 1'b1;
    c_7m   <= clkdiv[3];                 // toggles every 8 cycles -> period 16
  end

  reg reset_n = 0;

  // chip port
  reg  [24:1] chipAddr = 0;
  reg         chipL = 1, chipU = 1;      // active-low byte selects (1 = not selected)
  reg         chipRW = 1;                // 1 = read
  reg         chipDMA = 0;               // 0 -> CHIP slot always selected
  reg  [15:0] chipWR = 0;
  wire [15:0] chipRD;
  wire [47:0] chip48;

  // cpu port
  reg  [24:1] cpuAddr = 0;
  reg         cpuCS = 0;
  reg  [1:0]  cpustate = 2'b01;          // 1 = idle-ish (not a write)
  reg         cpuL = 1, cpuU = 1;
  reg  [15:0] cpuWR = 0;
  wire [15:0] cpuRD;
  wire        ramready;

  // sdram bus
  wire [12:0] sd_addr;
  wire [1:0]  sd_ba;
  wire        sd_cs, sd_we, sd_ras, sd_cas, sd_clk, sd_cke;
  wire [1:0]  sd_dqm;
  wire [15:0] sd_data;

  // bench drives a deterministic read pattern onto sd_data whenever the DUT is
  // not driving it (DUT only drives strong on a committed write, which we never
  // allow). Pattern is a function of sdram_state so each burst word is unique.
  wire [3:0] st = dut.sdram_state;
  reg drive = 1;
  assign sd_data = drive ? (16'hBB00 + {12'd0, st}) : 16'hzzzz;

  sdram_ctrl dut (
    .sysclk(sysclk), .c_7m(c_7m), .reset_n(reset_n),
    .cache_rst(1'b1), .cache_inhibit(1'b0),
    .cpu_cache_ctrl(4'b1111), .dcache_sw_en(1'b1),
    .sd_addr(sd_addr), .sd_ba(sd_ba), .sd_cs(sd_cs), .sd_we(sd_we),
    .sd_ras(sd_ras), .sd_cas(sd_cas), .sd_dqm(sd_dqm), .sd_data(sd_data),
    .sd_clk(sd_clk), .sd_cke(sd_cke),
    .chipAddr(chipAddr), .chipL(chipL), .chipU(chipU), .chipRW(chipRW),
    .chipDMA(chipDMA), .chipWR(chipWR), .chipRD(chipRD), .chip48(chip48),
    .cpuAddr(cpuAddr), .cpuCS(cpuCS), .cpustate(cpustate),
    .cpuL(cpuL), .cpuU(cpuU), .cpuWR(cpuWR), .cpuRD(cpuRD), .ramready(ramready)
  );

  // expected raw burst words (no forwarding), from sd_data sampled at states
  // 7,9,11,13 -> chipRD, chip48_1, chip48_2, chip48_3.
  function [15:0] raw_word(input [1:0] idx);
    raw_word = 16'hBB00 + (7 + 2*idx);
  endfunction

  task post_cpu_write(input [24:1] a, input [15:0] d, input bU, input bL);
    begin
      @(posedge sysclk);
      cpuAddr  <= a;
      cpuWR    <= d;
      cpuU     <= bU;          // active low (0 = byte selected)
      cpuL     <= bL;
      cpustate <= 2'b11;       // write
      cpuCS    <= 1'b1;
      repeat (4) @(posedge sysclk);
      // drop the access but keep it pending (write_req stays high; chipDMA=0
      // starves CPU_WRITECACHE so it never commits)
      cpuCS    <= 1'b0;
      cpustate <= 2'b01;
      cpuU     <= 1'b1;
      cpuL     <= 1'b1;
    end
  endtask

  // sync to a FRESH frame (so its state-0 latches current chipAddr/write_req),
  // let the whole burst capture, then sample the 4 words.
  task capture_burst(output [15:0] w0, output [15:0] w1, output [15:0] w2, output [15:0] w3);
    begin
      @(posedge sysclk);
      wait (dut.sdram_state == 4'd0);        // start of a fresh frame
      @(posedge sysclk);
      wait (dut.sdram_state == 4'd15);       // burst captured this frame
      @(posedge sysclk);
      @(posedge sysclk);
      // capture-register index == fwd_pos index. chip48 = {chip48_1,chip48_2,chip48_3}
      // so idx1=chip48[47:32], idx2=chip48[31:16], idx3=chip48[15:0].
      w0 = chipRD; w1 = chip48[47:32]; w2 = chip48[31:16]; w3 = chip48[15:0];
    end
  endtask

  reg probed = 0;
  always @(posedge sysclk)
    if (!probed && dut.init_done && dut.sdram_state==4'd1 && dut.write_req) begin
      $display("PROBE @frame: write_req=%b writeAddr=%h chipAddr=%h fwd_en=%b fwd_pos=%0d",
               dut.write_req, dut.writeAddr, chipAddr, dut.fwd_en, dut.fwd_pos);
      probed <= 1;
    end

  task check(input [255:0] name, input [15:0] got, input [15:0] exp);
    begin
      if (got !== exp) begin
        $display("FAIL %0s: got %04h exp %04h", name, got, exp);
        errs = errs + 1;
      end
    end
  endtask

  reg [15:0] g0,g1,g2,g3;
  reg [15:0] bt;   // Icarus cannot bit-select base(N) inline
  reg [15:0] ctl0,ctl1,ctl2,ctl3;   // measured raw (no-forward) baseline
  integer p;
  reg [24:1] blk;
  reg [15:0] wd;

  // expected raw word at capture-register index idx, from the control baseline
  function [15:0] base(input [1:0] idx);
    base = (idx==0)?ctl0 : (idx==1)?ctl1 : (idx==2)?ctl2 : ctl3;
  endfunction

  // full reset + wait for SDRAM init to complete (single-entry write buffer is
  // sticky once a pending write is starved, so we re-init before each sub-test).
  task reinit;
    begin
      reset_n = 0;
      cpuCS = 0; cpustate = 2'b01; cpuU = 1; cpuL = 1;
      repeat (20) @(posedge sysclk);
      reset_n = 1;
      wait (dut.init_done == 1'b1);
      repeat (32) @(posedge sysclk);
    end
  endtask

  initial begin
    blk = 24'h001000;     // 4-word block base; chipAddr[2:1] = 0 -> burst idx 0 == word @ blk

    // ---- Control: NO pending write -> capture the raw baseline burst ----
    reinit;
    chipAddr <= blk;
    capture_burst(ctl0,ctl1,ctl2,ctl3);
    $display("CTRL baseline: w0=%04h w1=%04h w2=%04h w3=%04h", ctl0,ctl1,ctl2,ctl3);

    // ---- For each position p, post a full-word write to blk|p, expect merge ----
    // fwd_pos = writeAddr[2:1]-chipAddr[2:1]; chipAddr[2:1]=0 so fwd_pos==p, and
    // the patch routes the merge into capture register p. So g[p]==wd, rest==base.
    for (p = 0; p < 4; p = p + 1) begin
      reinit;
      wd = 16'hDEAD + p[15:0];
      post_cpu_write({blk[24:3], p[1:0]}, wd, 1'b0, 1'b0);  // both bytes
      chipAddr <= blk;
      capture_burst(g0,g1,g2,g3);
      check("fullw.match", (p==0?g0:p==1?g1:p==2?g2:g3), wd);
      if (p!=0) check("fullw.w0raw", g0, base(0));
      if (p!=1) check("fullw.w1raw", g1, base(1));
      if (p!=2) check("fullw.w2raw", g2, base(2));
      if (p!=3) check("fullw.w3raw", g3, base(3));
    end

    // ---- byte-enable: write only LOWER byte to position 1 ----
    reinit;
    wd = 16'h1234;
    post_cpu_write({blk[24:3], 2'd1}, wd, 1'b1, 1'b0); // bU=1(masked), bL=0(write low)
    chipAddr <= blk;
    capture_burst(g0,g1,g2,g3);
    bt = base(1); check("lob.merge", g1, {bt[15:8], wd[7:0]});  // hi from SDRAM, lo from write
    check("lob.w0raw", g0, base(0));

    // ---- byte-enable: write only UPPER byte to position 3 ----
    reinit;
    wd = 16'hABCD;
    post_cpu_write({blk[24:3], 2'd3}, wd, 1'b0, 1'b1); // bU=0(write hi), bL=1(masked)
    chipAddr <= blk;
    capture_burst(g0,g1,g2,g3);
    bt = base(3); check("hib.merge", g3, {wd[15:8], bt[7:0]});

    // ---- non-matching block: pending write to a DIFFERENT block, no merge ----
    reinit;
    post_cpu_write({(blk[24:3]+4'd1), 2'd0}, 16'hFFFF, 1'b0, 1'b0);
    chipAddr <= blk;   // read the original block
    capture_burst(g0,g1,g2,g3);
    check("nomatch.w0", g0, base(0));
    check("nomatch.w1", g1, base(1));
    check("nomatch.w2", g2, base(2));
    check("nomatch.w3", g3, base(3));

    if (errs == 0) $display("RUN: PASS (forwarding correct)");
    else           $display("RUN: FAIL (%0d errors)", errs);
    $finish;
  end

  // safety timeout
  initial begin
    #2000000;
    $display("RUN: FAIL (timeout)");
    errs = errs + 1;
    $finish;
  end

endmodule
