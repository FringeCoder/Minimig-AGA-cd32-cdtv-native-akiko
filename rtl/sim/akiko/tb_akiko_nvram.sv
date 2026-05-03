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

// Sim runs from rtl/sim/akiko/, override INIT_FILE accordingly so $readmemh
// finds the same cd32.nvr image Quartus loads from project root.
akiko_nvram #(.INIT_FILE("../../init/nvram_init.hex")) dut (
	.clk       (clk),
	.reset     (reset),
	.scl_in    (bus_scl),
	.sda_in    (bus_sda),
	.sda_drive (sda_drive)
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
	// Summary
	// --------------------------------------------------------------
	$display("=================================================");
	$display("tb_akiko_nvram: %0d tests, %0d errors", tests, errs);
	$display("=================================================");
	$finish;
end

endmodule
