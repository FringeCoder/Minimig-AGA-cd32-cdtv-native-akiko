// tb_cache_snoop.sv
// Bench for cpu_cache_new fill-vs-snoop coherency race (D-Cache-ON image garble).
//
// Stage 1: basic fill + read-back (sanity that the mixed model works).
// Stage 2: the RACE sweep. A chip write (snoop pulse + SDRAM commit) lands at a
//          swept offset `d` cycles into a CPU read-miss fill of the SAME line.
//          After the fill, the CPU re-reads the word; a coherent cache must
//          return the NEW (chip-written) value. On the unmodified RTL some
//          offsets return the OLD value -> the cached line is permanently stale
//          (this is the Universe image garble). On the fixed RTL (fill/snoop
//          interlock) every offset must return NEW.
//
// The DUT's only memory interface is sdr_read_req/sdr_read_ack/sdr_dat_r plus
// the snoop port. The real sdram_ctrl knows the fill address because it also
// sees cpuAddr; we mirror that here (responder observes cpu_adr on req).

`timescale 1ns/1ps

module tb_cache_snoop;

  reg         clk = 0;
  reg         rst = 1;

  // cpu side
  reg  [3:0]  cpu_cache_ctrl = 4'b0001; // bit0 = cache enable
  reg         dcache_sw_en   = 1'b1;    // D-cache ON (the failing config)
  reg         cache_inhibit  = 1'b0;
  reg         cpu_cs = 0;
  reg  [28:1] cpu_adr = 0;
  reg  [1:0]  cpu_bs = 2'b11;
  reg         cpu_we = 0;
  reg         cpu_ir = 0;
  reg         cpu_dr = 0;
  reg  [15:0] cpu_dat_w = 0;
  wire [15:0] cpu_dat_r;
  wire        cpu_ack;
  wire        wb_en;

  // sdram side
  wire [15:0] sdr_dat_r;
  wire        sdr_read_req;
  reg         sdr_read_ack = 0;
  reg  [15:0] sdr_dat_r_r  = 0;
  assign sdr_dat_r = sdr_dat_r_r;

  // snoop side
  reg         snoop_act = 0;
  reg  [28:1] snoop_adr = 0;
  reg  [15:0] snoop_dat_w = 0;
  reg  [1:0]  snoop_bs = 2'b11;

  integer errs       = 0;   // basic-correctness failures (must be 0 always)
  integer race_fails = 0;   // race offsets returning stale data

  localparam [15:0] OLD = 16'h1111;
  localparam [15:0] NEW = 16'h2222;

  // ---- clock: 100 MHz ----
  always #5 clk = ~clk;

  // ---- DUT ----
  cpu_cache_new dut (
    .clk            (clk),
    .rst            (rst),
    .cpu_cache_ctrl (cpu_cache_ctrl),
    .dcache_sw_en   (dcache_sw_en),
    .cache_inhibit  (cache_inhibit),
    .cpu_cs         (cpu_cs),
    .cpu_adr        (cpu_adr),
    .cpu_bs         (cpu_bs),
    .cpu_we         (cpu_we),
    .cpu_ir         (cpu_ir),
    .cpu_dr         (cpu_dr),
    .cpu_dat_w      (cpu_dat_w),
    .cpu_dat_r      (cpu_dat_r),
    .cpu_ack        (cpu_ack),
    .wb_en          (wb_en),
    .sdr_dat_r      (sdr_dat_r),
    .sdr_read_req   (sdr_read_req),
    .sdr_read_ack   (sdr_read_ack),
    .snoop_act      (snoop_act),
    .snoop_adr      (snoop_adr),
    .snoop_dat_w    (snoop_dat_w),
    .snoop_bs       (snoop_bs)
  );

  // ---- behavioral SDRAM: word-addressed backing store ----
  // Indexed by word address bits [16:1] (64K words = 128KB, enough for test).
  reg [15:0] backing [0:65535];

  // 4-beat burst responder. On sdr_read_req rising (and idle), capture the
  // requested word address from cpu_adr and SNAPSHOT the 4 line words as they
  // are in SDRAM at fill-start, then emit them over 4 consecutive ack cycles
  // (matching sdram_ctrl's CL=2 BURST=4). The snapshot is deliberate and
  // faithful: an SDRAM burst returns the row as of activation, so a chip write
  // that commits *after* the fill began is NOT visible to the fill -- the snoop
  // port is the only mechanism that can deliver that write into the cache. This
  // is precisely the coherency contract the race stresses.
  reg        bursting = 0;
  reg [2:0]  beat = 0;
  reg [15:0] snap [0:3];     // 4 line words snapshotted at fill start

  always @(posedge clk) begin
    sdr_read_ack <= 1'b0;
    if (rst) begin
      bursting <= 0;
      beat     <= 0;
    end else begin
      if (sdr_read_req && !bursting) begin
        bursting  <= 1'b1;
        beat      <= 3'd0;
        // snapshot the 4 words of the requested line (block-aligned)
        snap[0]   <= backing[{cpu_adr[16:3], 2'd0}];
        snap[1]   <= backing[{cpu_adr[16:3], 2'd1}];
        snap[2]   <= backing[{cpu_adr[16:3], 2'd2}];
        snap[3]   <= backing[{cpu_adr[16:3], 2'd3}];
      end
      if (bursting) begin
        // first beat presents the requested block, wrapping within the line
        sdr_dat_r_r  <= snap[cpu_adr[2:1] + beat[1:0]];
        sdr_read_ack <= 1'b1;
        beat         <= beat + 3'd1;
        if (beat == 3'd3) bursting <= 1'b0;
      end
    end
  end

  // word address [28:1] = {tag=0, idx[7:0], blk[1:0]}
  function [28:1] mkadr(input [7:0] idx, input [1:0] blk);
    mkadr = {18'd0, idx, blk};
  endfunction

  // ---- CPU access helpers ----
  task automatic cpu_data_read(input [28:1] adr, output [15:0] data);
    begin
      @(posedge clk);
      cpu_adr <= adr;
      cpu_bs  <= 2'b11;
      cpu_we  <= 1'b0;
      cpu_ir  <= 1'b0;
      cpu_dr  <= 1'b1;
      cpu_cs  <= 1'b1;
      while (!cpu_ack) @(posedge clk);
      data = cpu_dat_r;
      @(posedge clk);
      cpu_cs <= 1'b0;
      cpu_dr <= 1'b0;
      @(posedge clk);
    end
  endtask

  task automatic do_snoop(input [28:1] adr, input [15:0] data);
    begin
      @(posedge clk);
      snoop_adr <= adr; snoop_dat_w <= data; snoop_bs <= 2'b11; snoop_act <= 1'b1;
      @(posedge clk);
      snoop_act <= 1'b0;
      repeat (4) @(posedge clk);
    end
  endtask

  // Byte-granular snoop: bs=2'b01 lower byte, 2'b10 upper byte, 2'b11 word.
  // The Akiko CD-sector DMA is a single-BYTE master, so real image writes
  // arrive as a stream of byte snoops -- each updates only one dpram lane.
  // `hold` = extra clk cycles snoop_act/adr/data are held HIGH/stable after
  // the first cycle (models the chip slot holding the bus). `gap` = idle
  // cycles after deassert before returning.
  task automatic do_byte_snoop(input [28:1] adr, input [15:0] data,
                               input [1:0] bs, input integer hold,
                               input integer gap);
    integer h;
    begin
      @(posedge clk);
      snoop_adr <= adr; snoop_dat_w <= data; snoop_bs <= bs; snoop_act <= 1'b1;
      for (h = 0; h < hold; h = h + 1) @(posedge clk);
      @(posedge clk);
      snoop_act <= 1'b0;
      for (h = 0; h < gap; h = h + 1) @(posedge clk);
    end
  endtask

  task automatic check(input [15:0] got, input [15:0] exp, input [255:0] msg);
    begin
      if (got !== exp) begin
        errs = errs + 1;
        $display("[FAIL] %0s: got %04x exp %04x @%0t", msg, got, exp, $time);
      end else begin
        $display("[ ok ] %0s: %04x", msg, got);
      end
    end
  endtask

  // One race iteration. CPU read-misses block 0 of line `idx` (triggers the
  // fill, which loads blocks 0,1,2,3 in order across FILL1..FILL4). At +d cycles
  // the chip write of block `sblk` lands: NEW committed to SDRAM + a one-cycle
  // snoop pulse. After the fill, the CPU re-reads block `sblk` (a cache hit) and
  // must observe the coherent value NEW. Returns 1 in `failed` if it reads stale.
  task automatic race_iter(input integer d, input [1:0] sblk, input [7:0] idx,
                           output integer failed);
    reg [15:0] rd;
    integer k;
    begin
      failed = 0;
      backing[{idx,2'd0}] = 16'h00B0;
      backing[{idx,2'd1}] = 16'h00B1;
      backing[{idx,2'd2}] = 16'h00B2;
      backing[{idx,2'd3}] = 16'h00B3;
      backing[{idx,sblk}] = OLD;   // the word the chip will rewrite to NEW

      // launch CPU read of block 0 (miss -> fill of the whole line)
      @(posedge clk);
      cpu_adr <= mkadr(idx, 2'd0);
      cpu_bs  <= 2'b11; cpu_we <= 0; cpu_ir <= 0; cpu_dr <= 1; cpu_cs <= 1;

      // wait d cycles into the fill, then the chip write lands
      for (k = 0; k < d; k = k + 1) @(posedge clk);
      backing[{idx,sblk}] = NEW;
      snoop_adr   <= mkadr(idx, sblk);
      snoop_dat_w <= NEW;
      snoop_bs    <= 2'b11;
      snoop_act   <= 1'b1;
      @(posedge clk);
      snoop_act   <= 1'b0;
      repeat (4) @(posedge clk);   // hold thru snoop FSM 3-cycle latency

      k = 0;
      while (!cpu_ack && k < 40) begin @(posedge clk); k = k + 1; end
      @(posedge clk);
      cpu_cs <= 1'b0; cpu_dr <= 1'b0;
      repeat (8) @(posedge clk);

      // re-read snooped block -> cache HIT, must be coherent (NEW)
      cpu_data_read(mkadr(idx, sblk), rd);
      if (rd !== NEW) begin
        failed = 1;
        $display("  [RACE d=%0d blk=%0d idx=%02x] STALE: re-read got %04x exp %04x",
                 d, sblk, idx, rd, NEW);
      end
    end
  endtask

  reg [15:0] rd;
  integer i;
  integer sb;
  integer f;
  integer idx_ctr;

  initial begin
    // init backing store
    for (i = 0; i < 65536; i = i + 1) backing[i] = 16'hDEAD;
    backing[16'h0100] = 16'h1111;
    backing[16'h0101] = 16'h2222;
    backing[16'h0102] = 16'h3333;
    backing[16'h0103] = 16'h4444;

    // reset
    rst = 1;
    repeat (8) @(posedge clk);
    rst = 0;
    // wait out cache init (256+ cycles clearing tags)
    repeat (400) @(posedge clk);

    // --- Stage 1: basic fill + read-back ---
    cpu_data_read(28'h0100, rd);
    check(rd, 16'h1111, "read word 0x100 (miss->fill)");
    cpu_data_read(28'h0101, rd);
    check(rd, 16'h2222, "read word 0x101 (same line)");
    cpu_data_read(28'h0100, rd);
    check(rd, 16'h1111, "read word 0x100 again (hit)");

    // --- Stage 1b: normal write-through snoop of a VALID line still works ---
    // (no fill active -> interlock must not interfere; data must update)
    backing[{8'hA0,2'd0}] = 16'h3333;
    cpu_data_read(mkadr(8'hA0,2'd0), rd);
    check(rd, 16'h3333, "1b: fill line A0");
    do_snoop(mkadr(8'hA0,2'd0), 16'h4444);
    cpu_data_read(mkadr(8'hA0,2'd0), rd);
    check(rd, 16'h4444, "1b: snoop-updated valid line (write-through intact)");

    // --- Stage 1c: a snoop to a DIFFERENT line during a fill must NOT evict
    // the line being filled (index/tag match gate works; no spurious inval) ---
    backing[{8'hB0,2'd0}] = 16'h5555;
    backing[{8'hB0,2'd1}] = 16'h00C1;
    backing[{8'hB0,2'd2}] = 16'h00C2;
    backing[{8'hB0,2'd3}] = 16'h00C3;
    @(posedge clk);
    cpu_adr <= mkadr(8'hB0, 2'd0);
    cpu_bs <= 2'b11; cpu_we<=0; cpu_ir<=0; cpu_dr<=1; cpu_cs<=1;
    for (i = 0; i < 4; i = i + 1) @(posedge clk);   // worst offset d=4
    snoop_adr <= mkadr(8'hB1, 2'd0);                 // different line
    snoop_dat_w <= 16'h6666; snoop_bs<=2'b11; snoop_act<=1'b1;
    @(posedge clk); snoop_act<=1'b0;
    repeat (4) @(posedge clk);
    i = 0; while (!cpu_ack && i < 40) begin @(posedge clk); i = i + 1; end
    @(posedge clk); cpu_cs<=0; cpu_dr<=0;
    repeat (8) @(posedge clk);
    backing[{8'hB0,2'd0}] = 16'h7777;   // sentinel: only observed if re-filled
    cpu_data_read(mkadr(8'hB0,2'd0), rd);
    check(rd, 16'h5555, "1c: fill line NOT evicted by snoop to other line (hit)");

    // --- Stage 2: fill-vs-snoop race sweep (4 blocks x 19 offsets) ---
    $display("---- race sweep (D-Cache ON): chip write of NEW lands +d into fill ----");
    idx_ctr = 8'h50;
    for (sb = 0; sb < 4; sb = sb + 1) begin
      for (i = 0; i <= 18; i = i + 1) begin
        race_iter(i, sb[1:0], idx_ctr[7:0], f);
        race_fails = race_fails + f;
        idx_ctr = idx_ctr + 1;
      end
    end
    $display("RACE: %0d of 76 (block,offset) cases returned STALE data (cache incoherent)", race_fails);

    // --- Stage 3: BYTE-write snoop coherency on a VALID cached word ---
    // The CD DMA writes image data one byte per chip slot. A byte snoop must
    // update only its lane and leave the other byte intact; a full-word CPU
    // read must then see the correct combined value.
    $display("---- Stage 3: byte-write snoop on valid cached line ----");
    backing[{8'hC0,2'd0}] = 16'hAABB;
    cpu_data_read(mkadr(8'hC0,2'd0), rd);
    check(rd, 16'hAABB, "3: fill word C0:0 = AABB");
    // write lower byte -> CC ; upper must stay AA
    do_byte_snoop(mkadr(8'hC0,2'd0), 16'h00CC, 2'b01, 0, 6);
    cpu_data_read(mkadr(8'hC0,2'd0), rd);
    check(rd, 16'hAACC, "3: lower-byte snoop -> AACC (upper preserved)");
    // write upper byte -> DD ; lower must stay CC
    do_byte_snoop(mkadr(8'hC0,2'd0), 16'hDD00, 2'b10, 0, 6);
    cpu_data_read(mkadr(8'hC0,2'd0), rd);
    check(rd, 16'hDDCC, "3: upper-byte snoop -> DDCC (lower preserved)");

    // --- Stage 4: stream of byte snoops to consecutive words of a cached line
    // (the realistic DMA fill of an already-cached image buffer). Every byte
    // must land. Tests both lanes across all 4 words at realistic spacing. ---
    $display("---- Stage 4: byte-stream snoop over a cached line ----");
    for (i = 0; i < 4; i = i + 1) backing[{8'hC4,i[1:0]}] = 16'h0000;
    for (i = 0; i < 4; i = i + 1) cpu_data_read(mkadr(8'hC4,i[1:0]), rd); // cache the line
    // DMA writes bytes: word w gets lo=0x10+w then hi=0x20+w, separate slots
    for (i = 0; i < 4; i = i + 1) begin
      do_byte_snoop(mkadr(8'hC4,i[1:0]), {8'h00, 8'h10 + i[7:0]}, 2'b01, 0, 5);
      do_byte_snoop(mkadr(8'hC4,i[1:0]), {8'h20 + i[7:0], 8'h00}, 2'b10, 0, 5);
    end
    for (i = 0; i < 4; i = i + 1) begin
      cpu_data_read(mkadr(8'hC4,i[1:0]), rd);
      check(rd, {8'h20 + i[7:0], 8'h10 + i[7:0]},
            "4: byte-stream word coherent");
    end

    // --- Stage 5: snoop-then-read window. Snoop a valid line, then CPU reads
    // the SAME line N cycles later. A transient stale read is acceptable, but
    // the line must NOT be left PERMANENTLY stale. We read at +1/+2/+3 then
    // settle and re-read; the settled read must be NEW. ---
    $display("---- Stage 5: snoop-then-read window (permanent-staleness probe) ----");
    for (sb = 1; sb <= 3; sb = sb + 1) begin
      backing[{8'hD0,2'd0}] = 16'h5500;
      cpu_data_read(mkadr(8'hD0,2'd0), rd);     // cache it (5500)
      // fire a word snoop, then immediately race a CPU read sb cycles in
      @(posedge clk);
      snoop_adr <= mkadr(8'hD0,2'd0); snoop_dat_w <= 16'h66AA;
      snoop_bs <= 2'b11; snoop_act <= 1'b1;
      @(posedge clk); snoop_act <= 1'b0;
      for (i = 0; i < sb; i = i + 1) @(posedge clk);
      cpu_data_read(mkadr(8'hD0,2'd0), rd);     // racing read (may be transient stale)
      repeat (12) @(posedge clk);               // settle
      cpu_data_read(mkadr(8'hD0,2'd0), rd);     // settled read MUST be NEW
      check(rd, 16'h66AA, "5: line coherent after snoop-read race (settled)");
    end

    // --- Stage 6: BYTE write during a fill (byte-granular race). Image DMA
    // writes a byte of the line being filled. The interlock must invalidate so
    // the re-read re-fills committed data. Sweep offset over the fill. ---
    $display("---- Stage 6: byte-write-during-fill race ----");
    idx_ctr = 8'hE0;
    for (i = 0; i <= 10; i = i + 1) begin
      // set up line; block 2 is the byte target
      backing[{idx_ctr[7:0],2'd0}] = 16'h00B0;
      backing[{idx_ctr[7:0],2'd1}] = 16'h00B1;
      backing[{idx_ctr[7:0],2'd2}] = 16'h11B2;  // OLD upper byte 0x11
      backing[{idx_ctr[7:0],2'd3}] = 16'h00B3;
      @(posedge clk);
      cpu_adr <= mkadr(idx_ctr[7:0], 2'd0);
      cpu_bs <= 2'b11; cpu_we<=0; cpu_ir<=0; cpu_dr<=1; cpu_cs<=1;
      for (sb = 0; sb < i; sb = sb + 1) @(posedge clk);
      // chip writes upper byte of block 2 -> 0x99, commits to backing
      backing[{idx_ctr[7:0],2'd2}] = 16'h99B2;
      snoop_adr <= mkadr(idx_ctr[7:0],2'd2); snoop_dat_w <= 16'h9900;
      snoop_bs <= 2'b10; snoop_act <= 1'b1;
      @(posedge clk); snoop_act <= 1'b0;
      repeat (4) @(posedge clk);
      f = 0; while (!cpu_ack && f < 40) begin @(posedge clk); f = f + 1; end
      @(posedge clk); cpu_cs<=0; cpu_dr<=0;
      repeat (8) @(posedge clk);
      cpu_data_read(mkadr(idx_ctr[7:0],2'd2), rd);
      if (rd !== 16'h99B2) begin
        race_fails = race_fails + 1;
        $display("  [BYTE-RACE off=%0d idx=%02x] STALE: got %04x exp 99B2", i, idx_ctr[7:0], rd);
      end
      idx_ctr = idx_ctr + 1;
    end
    $display("Stage 6 done (byte-write-during-fill).");

    if (errs == 0) $display("RUN: PASS (basic checks)");
    else           $display("RUN: FAIL (%0d basic errors)", errs);
    $finish;
  end

  // safety timeout
  initial begin
    #500000;
    $display("RUN: FAIL (timeout)");
    errs = errs + 1;
    $finish;
  end

endmodule
