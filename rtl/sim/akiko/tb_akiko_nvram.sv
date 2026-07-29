// SPDX-License-Identifier: GPL-3.0-or-later
//
// Akiko Phase 13 NVRAM I2C slave bench.
//
// DUT  : akiko_nvram (1 KiB 24LC08-equivalent EEPROM, BRAM-backed)
// BFM  : Soft I2C master implemented as tasks. Drives scl_in/sda_in to the
//        DUT and samples the wired-AND of master + slave for SDA reads.
//
// What we verify:
//   1. Single-byte random write (devaddr=0xA0, wordaddr=0x42, data=0xAB)
//      followed by random read returns 0xAB.
//   2. 4-byte sequential write at 0x100..0x103 followed by sequential read
//      with ACKs returns the same 4 bytes in order.
//   3. NACK on bad device address (e.g. 0xB0).
//   4. High-page addressing: write to addr 0x300 via devaddr=0xA6 (A9..A8=11)
//      reads back from devaddr=0xA6 + wordaddr=0x00.

`timescale 1ns / 1ps

module tb_akiko_nvram;

initial begin
	#500000 $fatal(1, "tb_akiko_nvram: watchdog timeout");
end

logic clk = 0;
initial forever #5 clk = ~clk;   // 100 MHz nominal

logic reset = 1;
initial begin
	@(posedge clk); @(posedge clk);
	reset = 0;
end

// I2C bus model. Master drives scl_master / sda_master; slave pulls SDA low
// via sda_drive. Effective bus = master AND ~slave (open-drain).
logic scl_master = 1;       // released high
logic sda_master = 1;       // released high
wire  sda_drive;            // from DUT
wire  bus_scl = scl_master;
wire  bus_sda = sda_master & ~sda_drive;

// host_* ports are the read-side (save dump). load_* ports are the write-side
// (ioctl_download path). Test 6 covers the read side; Test 7 covers the
// write side and verifies load writes do NOT set the dirty flag.
logic [9:0] host_addr        = 10'd0;
logic       host_clear_dirty = 1'b0;
wire  [7:0] host_dout;
wire        nvram_dirty;

logic [9:0] load_addr        = 10'd0;
logic [7:0] load_din         = 8'h00;
logic       load_we          = 1'b0;

// Sim runs from rtl/sim/akiko/, override INIT_FILE accordingly so altsyncram's
// init_file finds the same cd32.nvr image Quartus loads from project root.
akiko_nvram #(.INIT_FILE("../../init/nvram_init.mif")) dut (
	.clk              (clk),
	.reset            (reset),
	.scl_in           (bus_scl),
	.sda_in           (bus_sda),
	.sda_drive        (sda_drive),
	.host_addr        (host_addr),
	.host_dout        (host_dout),
	.host_clear_dirty (host_clear_dirty),
	.nvram_dirty      (nvram_dirty),
	.load_addr        (load_addr),
	.load_din         (load_din),
	.load_we          (load_we)
);

int errs = 0;
int tests = 0;

task automatic i2c_quarter;
	// Quarter-bit-period delay. Real I2C has 4 phases per bit; we use one
	// "delay" between each level change so the slave's edge detector sees
	// distinct rising/falling edges.
	repeat (8) @(posedge clk);
endtask

task automatic i2c_start;
	// SDA falls while SCL is high.
	scl_master = 1;
	sda_master = 1;
	i2c_quarter();
	sda_master = 0;
	i2c_quarter();
	scl_master = 0;
	i2c_quarter();
endtask

task automatic i2c_stop;
	// SDA rises while SCL is high.
	scl_master = 0;
	sda_master = 0;
	i2c_quarter();
	scl_master = 1;
	i2c_quarter();
	sda_master = 1;
	i2c_quarter();
endtask

// Master sends one bit. SCL=0 → drive SDA → SCL=1 (slave samples) → SCL=0.
task automatic i2c_master_bit (input bit b);
	scl_master = 0;
	sda_master = b;
	i2c_quarter();
	scl_master = 1;
	i2c_quarter();
	scl_master = 0;
	i2c_quarter();
endtask

// Master receives one bit. SCL=0 (release SDA) → SCL=1 (sample) → SCL=0.
task automatic i2c_master_recv_bit (output bit b);
	scl_master = 0;
	sda_master = 1;             // release for slave to drive
	i2c_quarter();
	scl_master = 1;
	i2c_quarter();
	b = bus_sda;                // sample at SCL high
	scl_master = 0;
	i2c_quarter();
endtask

// Send a byte (MSB first), then sample slave's ACK. Returns 1 if ACKed.
task automatic i2c_write (input [7:0] v, output bit ack);
	bit b;
	for (int i = 7; i >= 0; i--) begin
		i2c_master_bit(v[i]);
	end
	// 9th clock: master releases SDA, slave drives ACK.
	i2c_master_recv_bit(b);
	ack = ~b;                   // ACK = SDA low
endtask

// Receive a byte (MSB first), then send ACK or NACK to slave.
task automatic i2c_read (output [7:0] v, input bit ack);
	bit b;
	v = 0;
	for (int i = 7; i >= 0; i--) begin
		i2c_master_recv_bit(b);
		v[i] = b;
	end
	// 9th clock: master drives ACK (low) or NACK (high).
	i2c_master_bit(ack ? 1'b0 : 1'b1);
endtask

// ----------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------

task automatic check (input string name, input [7:0] got, input [7:0] want);
	tests++;
	if (got !== want) begin
		$display("FAIL [%s]: got 0x%02h, want 0x%02h", name, got, want);
		errs++;
	end else begin
		$display("PASS [%s]: 0x%02h", name, got);
	end
endtask

task automatic check_bit (input string name, input bit got, input bit want);
	tests++;
	if (got !== want) begin
		$display("FAIL [%s]: got %0d, want %0d", name, got, want);
		errs++;
	end else begin
		$display("PASS [%s]: %0d", name, got);
	end
endtask

initial begin
	bit ack;
	bit [7:0] data;

	// Wait for reset to deassert and a few clocks for the slave to settle.
	@(negedge reset);
	repeat (8) @(posedge clk);

	// --------------------------------------------------------------
	// Test 1: random byte write at 0x042, then random read back.
	// --------------------------------------------------------------
	$display("=== Test 1: single byte round trip @ 0x042 ===");
	i2c_start();
	i2c_write(8'hA0, ack);              // devaddr 1010_000_W (write, A9:A8=00)
	check_bit("dev_w_ack",   ack, 1);
	i2c_write(8'h42, ack);              // wordaddr A7..A0 = 0x42
	check_bit("waddr_ack",   ack, 1);
	i2c_write(8'hAB, ack);              // data
	check_bit("data_w_ack",  ack, 1);
	i2c_stop();

	repeat (4) @(posedge clk);

	// Random read: devaddr+W, wordaddr, repeated START, devaddr+R, read.
	i2c_start();
	i2c_write(8'hA0, ack);              // devaddr W to set internal addr
	check_bit("dev_w_ack2",  ack, 1);
	i2c_write(8'h42, ack);              // wordaddr 0x42
	check_bit("waddr_ack2",  ack, 1);
	i2c_start();                        // repeated START
	i2c_write(8'hA1, ack);              // devaddr R
	check_bit("dev_r_ack",   ack, 1);
	i2c_read(data, 0);                  // read with NACK (last byte)
	check ("readback",       data, 8'hAB);
	i2c_stop();

	repeat (4) @(posedge clk);

	// --------------------------------------------------------------
	// Test 2: 4-byte sequential write at 0x100..0x103, sequential read.
	// --------------------------------------------------------------
	$display("=== Test 2: 4-byte sequential @ 0x100 ===");
	i2c_start();
	i2c_write(8'hA2, ack);              // devaddr W with A8=1 (0x100)
	check_bit("seq_dev_ack", ack, 1);
	i2c_write(8'h00, ack);              // wordaddr 0x00 -> full addr 0x100
	check_bit("seq_waddr_ack", ack, 1);
	i2c_write(8'h11, ack);
	i2c_write(8'h22, ack);
	i2c_write(8'h33, ack);
	i2c_write(8'h44, ack);
	i2c_stop();

	repeat (4) @(posedge clk);

	i2c_start();
	i2c_write(8'hA2, ack);
	i2c_write(8'h00, ack);
	i2c_start();
	i2c_write(8'hA3, ack);              // devaddr R, A8=1
	i2c_read(data, 1); check("seq[0]", data, 8'h11);
	i2c_read(data, 1); check("seq[1]", data, 8'h22);
	i2c_read(data, 1); check("seq[2]", data, 8'h33);
	i2c_read(data, 0); check("seq[3]", data, 8'h44);
	i2c_stop();

	repeat (4) @(posedge clk);

	// --------------------------------------------------------------
	// Test 3: NACK on bad device address.
	// --------------------------------------------------------------
	$display("=== Test 3: NACK on bad devaddr 0xB0 ===");
	i2c_start();
	i2c_write(8'hB0, ack);              // not 1010xxxx -> should NACK
	check_bit("bad_dev_nack", ack, 0);
	i2c_stop();

	repeat (4) @(posedge clk);

	// --------------------------------------------------------------
	// Test 4: high page (0x300) via devaddr 0xA6 (A9:A8 = 11).
	// --------------------------------------------------------------
	$display("=== Test 4: high page 0x300 via devaddr 0xA6 ===");
	i2c_start();
	i2c_write(8'hA6, ack);
	i2c_write(8'h00, ack);              // wordaddr 0x00 -> full addr 0x300
	i2c_write(8'h5A, ack);
	i2c_stop();

	repeat (4) @(posedge clk);

	i2c_start();
	i2c_write(8'hA6, ack);
	i2c_write(8'h00, ack);
	i2c_start();
	i2c_write(8'hA7, ack);
	i2c_read(data, 0); check("hi_page",  data, 8'h5A);
	i2c_stop();

	repeat (8) @(posedge clk);

	// --------------------------------------------------------------
	// Test 5: Phase 14 — read pre-initialized FlashFile magic from
	// addr 0..15 without any prior write. Confirms $readmemh actually
	// loaded cd32.nvr and the I2C path returns those bytes verbatim.
	// Expected: 00 56 A9 00 00 00 00 00 02 00 00 00 00 00 00 00
	// --------------------------------------------------------------
	begin
		bit [7:0] expected[16];
		expected = '{8'h00, 8'h56, 8'hA9, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00,
		             8'h02, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00};
		$display("=== Test 5: pre-initialized FlashFile magic @ 0x000 ===");
		i2c_start();
		i2c_write(8'hA0, ack);
		i2c_write(8'h00, ack);             // wordaddr 0
		i2c_start();
		i2c_write(8'hA1, ack);             // devaddr R
		for (int i = 0; i < 16; i++) begin
			i2c_read(data, i < 15 ? 1 : 0);  // ACK on all but last
			check($sformatf("init[%0d]", i), data, expected[i]);
		end
		i2c_stop();
	end

	repeat (8) @(posedge clk);

	// --------------------------------------------------------------
	// Test 6: Phase 32 — HPS host read port + dirty flag.
	//   a) After Tests 1-4 wrote bytes via I2C, nvram_dirty must be 1.
	//   b) Pulse host_clear_dirty -> nvram_dirty falls to 0.
	//   c) Drive host_addr through 0..1023 and verify host_dout matches:
	//      - 0x042 (Test 1)             = 0xAB
	//      - 0x100..0x103 (Test 2)      = 0x11 0x22 0x33 0x44
	//      - 0x300 (Test 4)             = 0x5A
	//      - 0x000 (init image)         = 0x00 (FlashFile magic byte 0)
	//      - 0x001 (init image)         = 0x56
	//   d) Re-trigger an I2C write; verify nvram_dirty re-asserts.
	// --------------------------------------------------------------
	begin
		bit [7:0] hd;
		$display("=== Test 6: HPS host port + dirty flag ===");

		// (a) Dirty should be set from prior I2C writes (Tests 1, 2, 4).
		check_bit("dirty_set_after_i2c", nvram_dirty, 1);

		// (b) Pulse host_clear_dirty for one cycle -> dirty falls.
		host_clear_dirty = 1'b1;
		@(posedge clk);
		host_clear_dirty = 1'b0;
		@(posedge clk);  // one extra cycle for the FF to settle
		check_bit("dirty_cleared", nvram_dirty, 0);

		// (c) Spot-check known byte values via the host port.
		// host_dout lags host_addr by one clk edge (sync BRAM read).
		host_addr = 10'h042; @(posedge clk); @(posedge clk);
		check("host_rd_0x042 (Test 1)", host_dout, 8'hAB);

		host_addr = 10'h100; @(posedge clk); @(posedge clk);
		check("host_rd_0x100 (Test 2)", host_dout, 8'h11);
		host_addr = 10'h101; @(posedge clk); @(posedge clk);
		check("host_rd_0x101 (Test 2)", host_dout, 8'h22);
		host_addr = 10'h102; @(posedge clk); @(posedge clk);
		check("host_rd_0x102 (Test 2)", host_dout, 8'h33);
		host_addr = 10'h103; @(posedge clk); @(posedge clk);
		check("host_rd_0x103 (Test 2)", host_dout, 8'h44);

		host_addr = 10'h300; @(posedge clk); @(posedge clk);
		check("host_rd_0x300 (Test 4)", host_dout, 8'h5A);

		host_addr = 10'h000; @(posedge clk); @(posedge clk);
		check("host_rd_0x000 (init)",   host_dout, 8'h00);
		host_addr = 10'h001; @(posedge clk); @(posedge clk);
		check("host_rd_0x001 (init)",   host_dout, 8'h56);

		// (d) Re-trigger a write through I2C; nvram_dirty must re-assert.
		i2c_start();
		i2c_write(8'hA0, ack);
		i2c_write(8'h10, ack);
		i2c_write(8'hCD, ack);
		i2c_stop();
		repeat (4) @(posedge clk);
		check_bit("dirty_relatched", nvram_dirty, 1);

		// And the host port sees the new byte.
		host_addr = 10'h010; @(posedge clk); @(posedge clk);
		check("host_rd_0x010 (relatch)", host_dout, 8'hCD);
	end

	repeat (8) @(posedge clk);

	// --------------------------------------------------------------
	// Test 7: load write port (ioctl_download path).
	//   a) Pulse host_clear_dirty so we start clean.
	//   b) Drive load_we for 4 cycles writing 0xDE 0xAD 0xBE 0xEF
	//      at addresses 0x200..0x203.
	//   c) Read back via host port — bytes match.
	//   d) Verify nvram_dirty is STILL 0 (load writes must NOT set dirty).
	//   e) Drive a single I2C write — dirty must re-assert (the I2C path
	//      is the only thing that sets dirty).
	//   f) Verify the loaded bytes survive: read back via I2C and compare.
	// --------------------------------------------------------------
	begin
		bit [7:0] hd;
		$display("=== Test 7: load write port (ioctl_download path) ===");

		// (a) Start from a known-clean state.
		host_clear_dirty = 1'b1; @(posedge clk);
		host_clear_dirty = 1'b0; @(posedge clk);
		check_bit("dirty_clean_pre_load", nvram_dirty, 0);

		// (b) Burst-write 4 bytes via the load port. Each cycle: set
		//     load_addr / load_din, pulse load_we high.
		load_addr = 10'h200; load_din = 8'hDE; load_we = 1'b1; @(posedge clk);
		load_addr = 10'h201; load_din = 8'hAD;                 @(posedge clk);
		load_addr = 10'h202; load_din = 8'hBE;                 @(posedge clk);
		load_addr = 10'h203; load_din = 8'hEF;                 @(posedge clk);
		load_we = 1'b0; @(posedge clk);

		// (c) Read back via host port. host_dout lags host_addr by 1 clk.
		host_addr = 10'h200; @(posedge clk); @(posedge clk);
		check("load_rd_0x200", host_dout, 8'hDE);
		host_addr = 10'h201; @(posedge clk); @(posedge clk);
		check("load_rd_0x201", host_dout, 8'hAD);
		host_addr = 10'h202; @(posedge clk); @(posedge clk);
		check("load_rd_0x202", host_dout, 8'hBE);
		host_addr = 10'h203; @(posedge clk); @(posedge clk);
		check("load_rd_0x203", host_dout, 8'hEF);

		// (d) Critical invariant: load writes do NOT set dirty.
		check_bit("dirty_unset_after_load", nvram_dirty, 0);

		// (e) An I2C write still sets dirty.
		i2c_start();
		i2c_write(8'hA0, ack);                  // devaddr W
		i2c_write(8'h50, ack);                  // wordaddr 0x050
		i2c_write(8'h99, ack);
		i2c_stop();
		repeat (4) @(posedge clk);
		check_bit("dirty_set_after_i2c_post_load", nvram_dirty, 1);

		// (f) Loaded bytes survive: read back 0x200..0x203 via I2C.
		i2c_start();
		i2c_write(8'hA0, ack);                  // devaddr W (set addr)
		i2c_write(8'h00, ack);                  // wordaddr 0x000 (high page = 2 -> devaddr A4)
		i2c_stop();
		i2c_start();
		i2c_write(8'hA4, ack);                  // devaddr W, page 2 (A8=0,A9=1) -> selects 0x200..0x2FF
		i2c_write(8'h00, ack);                  // wordaddr 0x00 within page
		i2c_start();
		i2c_write(8'hA5, ack);                  // devaddr R, page 2
		i2c_read(hd, 1); check("i2c_rd_0x200", hd, 8'hDE);
		i2c_read(hd, 1); check("i2c_rd_0x201", hd, 8'hAD);
		i2c_read(hd, 1); check("i2c_rd_0x202", hd, 8'hBE);
		i2c_read(hd, 0); check("i2c_rd_0x203", hd, 8'hEF);
		i2c_stop();
	end

	repeat (8) @(posedge clk);

	// --------------------------------------------------------------
	// Test 8: load port works while reset is HIGH.
	// In the integrated design akiko.v ties akiko_nvram .reset(1'b0),
	// so the I2C state machine never sees the CD32 cpu_rst. Bench-side,
	// we model the worst-case "downstream sees reset" scenario by
	// asserting `reset` HIGH for the full duration of a 4-byte load
	// burst, then checking the bytes landed and survived reset release.
	// If the BRAM write port were gated by reset (it isn't, per the
	// load_addr/load_din/load_we mux in akiko_nvram.v), this test
	// would fail and we'd catch any regression that re-couples the
	// load path to a reset domain.
	// --------------------------------------------------------------
	begin
		bit [7:0] hd;
		$display("=== Test 8: load while reset asserted (domain-decoupling) ===");

		// Hold reset HIGH for the entire load burst.
		reset = 1'b1;
		@(posedge clk);
		load_addr = 10'h280; load_din = 8'hCA; load_we = 1'b1; @(posedge clk);
		load_addr = 10'h281; load_din = 8'hFE;                 @(posedge clk);
		load_addr = 10'h282; load_din = 8'hBA;                 @(posedge clk);
		load_addr = 10'h283; load_din = 8'hBE;                 @(posedge clk);
		load_we = 1'b0; @(posedge clk);

		// Release reset; let the I2C path settle.
		reset = 1'b0;
		repeat (8) @(posedge clk);

		// Read back via host port.
		host_addr = 10'h280; @(posedge clk); @(posedge clk);
		check("rst_load_rd_0x280", host_dout, 8'hCA);
		host_addr = 10'h281; @(posedge clk); @(posedge clk);
		check("rst_load_rd_0x281", host_dout, 8'hFE);
		host_addr = 10'h282; @(posedge clk); @(posedge clk);
		check("rst_load_rd_0x282", host_dout, 8'hBA);
		host_addr = 10'h283; @(posedge clk); @(posedge clk);
		check("rst_load_rd_0x283", host_dout, 8'hBE);

		// And via I2C — reset deassertion must leave the I2C path
		// functional and seeing the loaded bytes.
		i2c_start();
		i2c_write(8'hA4, ack);                  // devaddr W, page 2
		i2c_write(8'h80, ack);                  // wordaddr 0x80 -> full 0x280
		i2c_start();
		i2c_write(8'hA5, ack);                  // devaddr R, page 2
		i2c_read(hd, 1); check("rst_i2c_rd_0x280", hd, 8'hCA);
		i2c_read(hd, 1); check("rst_i2c_rd_0x281", hd, 8'hFE);
		i2c_read(hd, 1); check("rst_i2c_rd_0x282", hd, 8'hBA);
		i2c_read(hd, 0); check("rst_i2c_rd_0x283", hd, 8'hBE);
		i2c_stop();
	end

	repeat (8) @(posedge clk);

	// --------------------------------------------------------------
	// Test 9: 1024-byte sequential load burst.
	// Mirrors the production hps_io.ioctl_download path: 1024 back-to-back
	// load_we pulses with load_din varying every cycle. Then read every
	// byte back via host_addr to confirm each address landed with the
	// correct value (and not the stale prior-cycle value, which is what
	// hardware was doing on 2026-05-06 — bytes 0x200..0x3FF stuck at the
	// 0x1FF value while ioctl_addr advanced correctly).
	// --------------------------------------------------------------
	begin
		bit [7:0] hd;
		bit [7:0] expected;
		int    miscount = 0;
		$display("=== Test 9: 1024-byte sequential load burst ===");

		// Drive 1024 distinct bytes: pattern lets a single mismatch at
		// offset N show up as got != want (no aliasing collisions across
		// the 1024 byte address space, unlike the (i & 0x3F) variant we
		// hit on hardware).
		for (int i = 0; i < 1024; i++) begin
			load_addr = i[9:0];
			load_din  = 8'(i ^ (i >> 3));   // varying per address
			load_we   = 1'b1;
			@(posedge clk);
		end
		load_we = 1'b0;
		@(posedge clk); @(posedge clk);

		// Read back every byte and compare. Don't check_*  per-byte (that
		// would log 1024 lines); accumulate and report the count at end.
		for (int i = 0; i < 1024; i++) begin
			host_addr = i[9:0];
			@(posedge clk); @(posedge clk);
			expected = 8'(i ^ (i >> 3));
			if (host_dout !== expected) begin
				if (miscount < 8)
					$display("FAIL [burst @0x%03x]: got 0x%02h, want 0x%02h",
					         i, host_dout, expected);
				miscount++;
			end
		end
		tests++;
		if (miscount) begin
			$display("FAIL [burst_1024]: %0d/1024 mismatched", miscount);
			errs++;
		end else begin
			$display("PASS [burst_1024]: 1024 bytes round-trip");
		end
	end

	repeat (8) @(posedge clk);

	// --------------------------------------------------------------
	// Summary
	// --------------------------------------------------------------
	$display("=================================================");
	$display("tb_akiko_nvram: %0d tests, %0d errors", tests, errs);
	$display("=================================================");
	$finish;
end

endmodule
