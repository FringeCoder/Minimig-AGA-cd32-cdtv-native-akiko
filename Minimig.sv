/********************************************/
/* minimig.sv                               */
/* MiSTer glue logic                        */
/* 2017-2020 Alexey Melnikov                */
/********************************************/

module emu
(

	`include "sys/emu_ports.vh"
);

assign ADC_BUS  = 'Z;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign BUTTONS = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

// MT32-pi, MiSTer Floppy and PSX SNAC share the SNAC user port, so USER_OUT
// is muxed between them. Declared as wires (Rob uses reg); all are driven
// by module outputs, and a wire says that unambiguously.
wire  [1:0] user_port_mode;              // 0 = MT32-pi, 1 = MiSTer Floppy, 2 = PSX SNAC
wire  [5:0] snac_mode;                   // {port2[2:0], port1[2:0]}
wire  [2:0] mister_floppy_status;        // {cable, drive type, detected}
wire  [6:0] IndirectUserOutmt32;
wire  [6:0] IndirectUserOutFlop;
wire  [6:0] IndirectUserOutSnac;

// One tenant at a time -- they collide on every user-port pin.
assign USER_OUT = (user_port_mode == 2'd2) ? IndirectUserOutSnac :
                  (user_port_mode == 2'd1) ? IndirectUserOutFlop :
                                             IndirectUserOutmt32;

`include "build_id.v" 
`include "rtl/ss_state.vh"
localparam CONF_STR = {
	"AmigaCD;UART115200:230400,MIDI,SS3E000000:400000;",
	"J,Red(Fire),Blue,Yellow,Green,RT,LT,Pause;",
	"jn,A,B,X,Y,R,L,Start;",
	"jp,B,A,X,Y,R,L,Start;",
	"-;",
	"I,",
	"MT32-pi: SoundFont #0,",
	"MT32-pi: SoundFont #1,",
	"MT32-pi: SoundFont #2,",
	"MT32-pi: SoundFont #3,",
	"MT32-pi: SoundFont #4,",
	"MT32-pi: SoundFont #5,",
	"MT32-pi: SoundFont #6,",
	"MT32-pi: SoundFont #7,",
	"MT32-pi: MT-32 v1,",
	"MT32-pi: MT-32 v2,",
	"MT32-pi: CM-32L,",
	"MT32-pi: Unknown mode,",
	// Save state outcomes. These are indices 13..22 of this list, which is what
	// SS_INFO_BASE names; user_io.cpp's show_core_info() counts substrings from
	// 1, so inserting anything above here shifts them and must shift
	// SS_INFO_BASE with it. Order after "restored" is ss_ctrl's load_fail_code
	// 1..6, in order, then the unknown-code fallback.
	"Save state: saved,",
	"Save state: FAILED,",
	"State restored,",
	"Restore: not a save state,",
	"Restore: wrong version,",
	"Restore: not for this core,",
	"Restore: corrupt file,",
	"Restore: wrong Kickstart,",
	// No comma inside a message: substrcpy() (user_io.cpp:512) splits this line
	// on commas, so one here would truncate the toast at "Restore: busy".
	"Restore: busy - try again,",
	"Restore: FAILED;",
	"V,v",`BUILD_DATE
};

wire [15:0] JOY0;
wire [15:0] JOY1;
wire [15:0] JOY2;
wire [15:0] JOY3;
wire [15:0] JOYA0;
wire [15:0] JOYA1;

wire [15:0] snac_pad0, snac_pad1;
wire [31:0] snac_axes0, snac_axes1;
wire  [7:0] snac_id0, snac_id1;
wire [10:0] snac_joy0, snac_joy1;
wire  [6:0] snac_user_out_raw;

// Packed for hps_io's 'hFE read case, so the OSD/menu-side reader (and the
// userspace analog-stick mouse) can see the raw pad. 112 bits = 32+16+32+16+8+8.
// Matches core-menu's concatenation and the byte_cnt/word order userspace
// reads in snac_psx_poll(): status, pad[0], axes0_lo, axes0_hi, pad[1],
// axes1_lo, axes1_hi.
wire [111:0] snac_state = { snac_axes1, snac_pad1, snac_axes0, snac_pad0,
                            snac_id1, snac_id0 };

snac_psx #(.CLK_KHZ(28688), .BAUD_KHZ(250)) snac
(
	.clk(clk_sys),
	.reset(reset),
	.enable(user_port_mode == 2'd2),
	.user_in(USER_IN),
	.user_out(snac_user_out_raw),
	.pad0(snac_pad0), .pad1(snac_pad1),
	.axes0(snac_axes0), .axes1(snac_axes1),
	.id0(snac_id0), .id1(snac_id1)
);

snac_cd32 snac_map0 (.psx(snac_pad0), .joy(snac_joy0));
snac_cd32 snac_map1 (.psx(snac_pad1), .joy(snac_joy1));

// A GunCon derives its timing from the displayed raster, so it needs composite
// sync fed back. hs/vs are active low, so a wired-AND is the classic composite.
// Only driven when a port is actually in light-gun mode; otherwise the reader's
// own idle level goes out.
wire snac_lightgun = (snac_mode[2:0] == 3'd4) || (snac_mode[5:3] == 3'd4);
assign IndirectUserOutSnac = { snac_lightgun ? (hs & vs) : snac_user_out_raw[6],
                               snac_user_out_raw[5:0] };

// OR rather than replace: a USB pad and a SNAC pad can both be connected, and
// locking one out would be a surprise. Bits are active high here; they are
// inverted at the minimig instantiation below.
wire [15:0] JOY0_MUX = JOY0 | {5'd0, snac_joy0};
wire [15:0] JOY1_MUX = JOY1 | {5'd0, snac_joy1};

wire  [7:0] kbd_mouse_data;
wire        kbd_mouse_level;
wire  [1:0] kbd_mouse_type;
wire  [2:0] mouse_buttons;
wire [64:0] RTC;

wire        ce_pix;
wire  [1:0] buttons;
wire [63:0] status;
wire        forced_scandoubler;

wire        io_strobe;
wire        io_wait;
wire        io_fpga;
wire        io_uio;
wire [15:0] io_din;
wire [15:0] fpga_dout;

wire [21:0] gamma_bus;

wire  [7:0] uart_mode;

// VDNUM=2: slot 0 = NVRAM .nvr file (load via canonical SD-block path).
//          slot 1 = akiko sector DMA target. Userspace sends UIO_SECTOR_RD |
//                   (1<<8) followed by 2352 raw bytes; hps_io drives sd_ack[1]
//                   high for the transfer and pulses sd_buff_wr per byte with
//                   sd_buff_addr auto-incrementing 0..2351. akiko.v captures
//                   into sector_buffer at full SPI rate (replaces the slow
//                   per-byte SSPI_ACK path on UIO_DMA_WRITE).
// BLKSZ=3: 1024 bytes per LBA. NVR fits in 1 block; akiko sector path
//          ignores LBA (userspace addresses bytes via sd_buff_addr directly,
//          but the protocol still requires BLKSZ — 1024 is fine because the
//          sector path doesn't gate on LBA boundaries).
hps_io #(.CONF_STR(CONF_STR), .CONF_STR_BRAM(0), .VDNUM(2), .BLKSZ(3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS({HPS_BUS[45:42],ce_pix,HPS_BUS[40:0]}),

	.status(status),
	.status_menumask({mister_floppy_status,mt32_cfg,mt32_available}),
	.snac_state(snac_state),
	// Two producers, one channel. hps_io latches `info` on the rising edge of
	// `info_req`, so the mux has to present the right code on the same cycle
	// the request rises -- hence selecting on ss_info_req rather than latching
	// a winner. A simultaneous MT32-pi mode change would lose its message;
	// that costs a soundfont name once, against a save state outcome that has
	// no other way to be seen at all.
	.info_req(mt32_info_req | ss_info_req),
	.info(ss_info_req ? ss_info_code : {4'd0, mt32_info_disp}),

	.joystick_0(JOY0),
	.joystick_1(JOY1),
	.joystick_2(JOY2),
	.joystick_3(JOY3),
	.joystick_l_analog_0(JOYA0),
	.joystick_l_analog_1(JOYA1),

	.ioctl_wait(io_wait),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),

	.sd_lba(sd_lba),
	.sd_blk_cnt(sd_blk_cnt),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),

	.uart_mode(uart_mode),

	.RTC(RTC),
	.gamma_bus(gamma_bus),

	.EXT_BUS(EXT_BUS)
);

wire [15:0] ide_din;
wire [15:0] ide_dout;
wire  [4:0] ide_addr;
wire        ide_rd;
wire        ide_wr;
wire  [5:0] ide_req;

// Akiko HPS bridge wires (M3). These bind hps_ext's akiko_* ports by name
// via its wildcard instantiation; fastchip drives akiko_din/akiko_req and
// receives akiko_dout/akiko_wr/akiko_rd/akiko_cs from hps_ext. M4 adds
// akiko_cs_sec (sub-channel discriminator) and akiko_sec_req (status bit).
wire [15:0] akiko_din;     // FROM fastchip TO hps_ext (data to Main)
wire [15:0] akiko_dout;    // FROM hps_ext TO fastchip (data from Main)
wire        akiko_wr;
wire        akiko_rd;
wire        akiko_cs;
wire        akiko_cs_sec; // M4 sub-channel selector
wire        akiko_cs_nvr; // NVRAM save-dump sub-channel (io_din[6])
wire        akiko_cs_subcode; // subcode push sub-channel (io_din[4], 0xF410)
wire        akiko_req;     // FROM fastchip TO hps_ext (cmd status bit)
wire        akiko_sec_req; // FROM fastchip TO hps_ext (M4 sector status bit)
wire        akiko_rx_busy; // FROM fastchip TO hps_ext (RX engine busy)
wire        akiko_nvr_dirty; // FROM fastchip TO hps_ext (NVRAM dirty bit)

// CDTV HPS bridge wires (M2 phase-1a). Bound to hps_ext's cdtv_* ports
// via wildcard instantiation. cmd byte-stream sub-channel only —
// sector / status pulses come later phases.
wire [15:0] cdtv_din;          // FROM cdtv_hps_bridge TO hps_ext
wire [15:0] cdtv_dout;         // FROM hps_ext TO cdtv_hps_bridge
wire        cdtv_wr;
wire        cdtv_rd;
wire        cdtv_cs;
wire        cdtv_cs_sec;       // phase-1b sector-push sub-channel
wire        cdtv_cs_stch;      // phase-1e STCH-inject sub-channel
wire        cdtv_stch_inject;  // 1-clk pulse from cdtv_hps_bridge -> minimig
wire        cdtv_stch_ack;     // BIOS took the STCH interrupt
wire        cdtv_stch_ack_clr;
wire        cdtv_sec_byte_push_w; // 1-clk pulse per UIO sec byte
wire  [7:0] cdtv_sec_byte_data_w;
wire  [7:0] cdtv_sec_space_w;  // sector FIFO free space, 32-byte units
wire        cdtv_sec_empty_w;  // sector FIFO exactly empty
wire        cdtv_req;          // bit 6 of 0x63 status word

// NVRAM load-from-disk via canonical SD-block path (the pattern SNES,
// Saturn, GBA, NeoGeo, CDi all use). Userspace calls user_io_file_mount
// with the .nvr path; hps_io fires img_mounted[0], we kick off a single
// 1024-byte read by raising sd_rd_nvr with sd_lba_nvr=0, hps_io streams
// each byte on sd_buff_dout / sd_buff_addr / sd_buff_wr, and we forward
// directly to akiko_nvram's load_we BRAM port. Sidesteps the gp_out CDC
// race that broke the ioctl_download path on hardware (data freezes at
// the first byte-value transition; bench-clean, hw-broken).
//
// VDNUM=1 (slot 0 = NVR), BLKSZ=3 (1024 B/block — whole NVR fits in one
// LBA), WIDE=0 (byte-wide sd_buff_dout — direct match for our 8-bit
// BRAM port, no unpack needed). The akiko_nvram BRAM write port lives
// outside the CD32 CPU reset domain (akiko.v nvram_inst has .reset(1'b0)),
// so the load works whether or not cpu_rst is asserted at mount time.
wire        img_mounted;
wire        img_readonly;
wire [63:0] img_size;
wire [31:0] sd_lba   [1:0];
wire  [5:0] sd_blk_cnt[1:0];
wire  [1:0] sd_ack;
wire [13:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din[1:0];
wire        sd_buff_wr;

reg         sd_rd_nvr;
reg         sd_wr_nvr;        // unused for now (save still goes via UIO dump)
reg         img_mounted_d;

// Slot 0 = NVR. Slot 1 = akiko sector DMA target — sd_rd/sd_wr stay 0
// (userspace pushes UIO_SECTOR_RD on demand without going through the
// menu-driven sd_rd/sd_wr handshake; hps_io still asserts sd_ack[1] on
// receipt of the opcode, which is all the akiko path needs).
wire  [1:0] sd_rd = {1'b0, sd_rd_nvr};
wire  [1:0] sd_wr = {1'b0, sd_wr_nvr};
assign sd_lba[0]      = 32'd0;
assign sd_lba[1]      = 32'd0;        // unused: akiko addresses by sd_buff_addr
assign sd_blk_cnt[0]  = 6'd0;         // single 1024-byte block per NVR load
assign sd_blk_cnt[1]  = 6'd0;         // unused; tied off
assign sd_buff_din[0] = 8'h00;        // NVR save not wired through this path
assign sd_buff_din[1] = 8'h00;        // akiko slot is read-only from the host

always @(posedge clk_sys) begin
	img_mounted_d <= img_mounted;
	// Rising edge of img_mounted with the right size kicks off the
	// load by raising sd_rd_nvr. hps_io picks this up, asserts sd_ack
	// (HIGH for the *entire* transfer — see sys/hps_io.sv 'h17/0X18
	// case) and streams bytes on sd_buff_dout/_addr/_wr. We drop
	// sd_rd_nvr the moment sd_ack rises so the request isn't re-issued.
	if (img_mounted && !img_mounted_d &&
	    (img_size == 64'd1024) && !img_readonly) begin
		sd_rd_nvr <= 1'b1;
	end else if (sd_ack[0]) begin
		sd_rd_nvr <= 1'b0;
	end
	sd_wr_nvr <= 1'b0;
end

// Gate by sd_ack[0] (HIGH for the whole NVR transfer), NOT sd_rd_nvr (which
// drops one cycle after the transfer starts and would mask every byte).
wire [9:0]  nvr_load_addr  = sd_buff_addr[9:0];
wire [7:0]  nvr_load_din   = sd_buff_dout;
wire        nvr_load_we    = sd_buff_wr & sd_ack[0];

// Akiko sector DMA path (slot 1). sd_ack[1] gates capture in akiko.v;
// sd_buff_dout/_addr/_wr are shared with the NVR slot but the gate keeps
// transfers exclusive (only one slot's sd_ack is HIGH at a time per the
// hps_io.sv `'h0X17: sd_ack <= disk[VD:0]` assignment).
wire        akiko_sec_dma_active = sd_ack[1];
wire  [7:0] akiko_sec_dma_byte   = sd_buff_dout;
wire [13:0] akiko_sec_dma_addr   = sd_buff_addr;
wire        akiko_sec_dma_we     = sd_buff_wr;

// Akiko chip-RAM master wires (M5). akiko's DMA engines emit single-byte
// requests; chipdma_arb (instantiated below sdram_ctrl) muxes them onto
// the chipDMA port whenever minimig isn't using the slot.
wire        akiko_dma_req_w;
wire        akiko_dma_we_w;
wire [23:0] akiko_dma_baddr_w;
wire  [7:0] akiko_dma_wbyte_w;
wire  [7:0] akiko_dma_rbyte_w;
wire        akiko_dma_ack_w;
wire        akiko_dma_arm_w;   // owner-freeze pulse: chipdma_arb -> fastchip/akiko

// Arbiter outputs that drive sdram_ctrl's chipDMA port (replacing the
// direct minimig wiring). Default: pass minimig through. When akiko
// claims an idle slot, drive akiko's address/data instead.
wire [24:1] arb_chip_addr;
wire        arb_chip_l;
wire        arb_chip_u;
wire        arb_chip_rw;
wire        arb_chip_dma;
wire [15:0] arb_chip_wr;

wire [35:0] EXT_BUS;

// Save state diagnostics, published on hps_ext's 0xF600 UIO read
// sub-channel. Assembled at the bottom of the save state section below;
// declared here because the `.*` connection needs the net to exist, and
// connected by name rather than left to `.*` so the instance says out loud
// that this port is wired.
wire [127:0] ss_diag;

// Live memory peek: hps_ext (clk_sys) asks, ss_ctrl (clk_114) answers.
//
// The request is a single clk_sys pulse and clk_114 is four times faster, so
// it cannot be missed -- but it would be sampled on several clk_114 edges, and
// ss_ctrl must act once. Carried as a toggle and edge-detected on the far side.
wire [24:1]  ss_peek_addr;
wire         ss_peek_req_sys;
wire [127:0] ss_peek_data;
wire         ss_peek_valid;
wire         ss_peek_busy;
wire         ss_peek_scan;
wire [127:0] ss_pc_snapshot;
wire  [63:0] ss_kick_pair;

reg  ss_peek_tgl = 1'b0;
always @(posedge clk_sys) if (ss_peek_req_sys) ss_peek_tgl <= ~ss_peek_tgl;

reg  ss_peek_tgl_meta, ss_peek_tgl_sync, ss_peek_tgl_d;
always @(posedge clk_114) begin
	ss_peek_tgl_meta <= ss_peek_tgl;
	ss_peek_tgl_sync <= ss_peek_tgl_meta;
	ss_peek_tgl_d    <= ss_peek_tgl_sync;
end
wire ss_peek_req_114 = ss_peek_tgl_sync ^ ss_peek_tgl_d;

hps_ext hps_ext(.*, .ide_req(ide_fast ? ide_f_req : ide_c_req),  .ide_din(ide_fast ? ide_f_readdata : ide_c_readdata), .ss_diag(ss_diag),
	.ss_peek_addr(ss_peek_addr), .ss_peek_req(ss_peek_req_sys),
	.ss_peek_data(ss_peek_data), .ss_peek_valid(ss_peek_valid),
	.ss_pc_snapshot(ss_pc_snapshot), .ss_kick_pair(ss_kick_pair),
	.ss_intena_live(ss_intena), .ss_intreq_live(ss_intreq),
	.ss_frame_count(ss_frame_count), .ss_reset_src(ss_reset_src),
	.ss_reset_pc(ss_reset_pc), .ss_fault_vec(ss_fault_vec),
	.ss_fault_pc(ss_fault_pc), .ss_int_count(ss_int_count));

assign LED_POWER[1] = 1;
assign LED_DISK     = {1'b0, ide_fast ? ide_f_led : ide_c_led};

assign VGA_SCALER   = FB_EN;

wire clk_114;
wire clk_sys;
wire locked;

pll pll
(
	.refclk(CLK_50M),
	.outclk_0(clk_114),
	.outclk_1(clk_sys),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll),
	.locked(locked)
);

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;
wire        cfg_waitrequest;
reg         cfg_write;
reg   [5:0] cfg_address;
reg  [31:0] cfg_data;

pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	.mgmt_write(cfg_write),
	.mgmt_address(cfg_address),
	.mgmt_writedata(cfg_data),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

always @(posedge CLK_50M) begin
	reg ntscd = 0, ntscd2 = 0;
	reg [2:0] state = 0;
	reg ntsc_r;

	ntscd <= ntsc;
	ntscd2 <= ntscd;

	cfg_write <= 0;
	if(ntscd2 == ntscd && ntscd2 != ntsc_r) begin
		state <= 1;
		ntsc_r <= ntscd2;
	end

	if(!cfg_waitrequest) begin
		if(state) state<=state+1'd1;
		case(state)
			1: begin
					cfg_address <= 0;
					cfg_data <= 0;
					cfg_write <= 1;
				end
			3: begin
					cfg_address <= 7;
					cfg_data <= ntsc_r ? 702807747 : 343817200;
					cfg_write <= 1;
				end
			5: begin
					cfg_address <= 2;
					cfg_data <= 0;
					cfg_write <= 1;
				end
		endcase
	end
end

wire reset = ~locked | buttons[1] | RESET;

reg reset_d;
always @(posedge clk_sys, posedge reset) begin
	reg [7:0] reset_s;
	reg rs;
	
	if(reset) reset_s <= '1;
	else begin
		reset_s <= reset_s << 1;
		rs <= reset_s[7];
		reset_d <= rs;
	end
end

//// amiga clocks ////
//
// Both amiga_clk instances are reset from ~reset_d, never from ~reset.
// `reset` is ~locked | buttons[1] | RESET -- asynchronous to clk_sys -- and
// amiga_clk recovers it with `always @(posedge clk_28, negedge reset_n)`.
// With two instances that is two INDEPENDENT recoveries of the same
// asynchronous release edge: one metastable capture and the two phase
// generators come out of reset a clk_28 cycle apart and stay that way for
// the whole session. chipdma_arb (see its header) requires minimig's chip
// DMA inputs to be aligned to the c_7m it samples, and those now come from
// different generators, so a permanent one-cycle skew is a permanent
// chip-RAM slot corruption -- nondeterministic, boot-time, and invisible
// until something tears.
//
// reset_d is the clk_sys-synchronised reset already used by sdram_ctrl,
// ddram_ctrl, chipdma_arb, minimig, ss_ctrl and ss_freeze_7m. It is only
// ever written on posedge clk_sys (the async `posedge reset` branch of its
// always block writes reset_s, not reset_d), so both its assertion and its
// release are synchronous and both instances leave reset on the same edge.
wire       clk7_en;
wire       clk7n_en;
wire       c1;
wire       c3;
wire       cck;
wire [9:0] eclk;

amiga_clk amiga_clk
(
	.clk_28   ( clk_sys    ), // input  clock c1 ( 28.687500MHz)
	.clk7_en  ( clk7_en    ), // output clock 7 enable (on 28MHz clock domain)
	.clk7n_en ( clk7n_en   ), // 7MHz negedge output clock enable (on 28MHz clock domain)
	.c1       ( c1         ), // clk28m clock domain signal synchronous with clk signal
	.c3       ( c3         ), // clk28m clock domain signal synchronous with clk signal delayed by 90 degrees
	.cck      ( cck        ), // colour clock output (3.54 MHz)
	.eclk     ( eclk       ), // 0.709379 MHz clock enable output (clk domain pulse)
	.ce       ( 1'b1       ), // free-running: this copy drives sdram_ctrl and chipdma_arb
	.reset_n  ( ~reset_d   )  // synchronous release -- see note above
);


//////////////////////////  SAVE STATES (phase 1A)  /////////////////////////
//
// Declarations only; the sequencer itself is below the minimig instance,
// where every signal it observes already exists.
//
// The freeze does not gate clk7_en/clk7n_en the way the plan described. c1,
// c3, cck and eclk are Gray-coded phase LEVELS shared by agnus, denise, the
// CIAs and the sram bridge, and a one-in-four phase decode off them becomes
// true *every* cycle if the levels are held, so ANDing them with ~freeze
// speeds the chipset up rather than stopping it. Instead the whole 7 MHz
// timebase feeding minimig comes from a second amiga_clk whose ce can be
// dropped. The original instance is untouched and still drives sdram_ctrl
// and chipdma_arb: SDRAM refresh must not stop for the ~150 ms a 2 MB chip
// RAM dump takes, and the dump itself reads chip RAM through sdram_ctrl's
// CPU port.
wire        ss_freeze;
wire        ss_save_busy;

wire        am_clk7_en;
wire        am_clk7n_en;
wire        am_c1;
wire        am_c3;
wire        am_cck;
wire  [9:0] am_eclk;

// The freeze's two clk_sys-domain signals: the enable that stops the Amiga's
// timebase, and the register-decode tick a replay needs while it is stopped.
// Both live in rtl/ss_freeze_phase.v rather than here, because the core repo's
// rtl/sim/ssmux bench instantiates the same module around a real minimig --
// written out twice, the sampling phase in particular would drift between the
// hardware and the bench that is supposed to check it. The reasoning for the
// phase, and why its polarity reads backwards, is in that file.
wire ss_freeze_7m;
wire ss_replay_tick;

ss_freeze_phase ss_freeze_phase_inst
(
	.clk         ( clk_sys       ),
	.rst_n       ( ~reset_d      ),
	// The MASTER generator's outputs, not am_*: the Amiga's stop when the
	// freeze takes hold.
	.clk7_en     ( clk7_en       ),
	.cck         ( cck           ),
	.freeze      ( ss_freeze     ),
	.replay_we   ( ss_replay_we  ),
	.freeze_7m   ( ss_freeze_7m  ),
	.replay_tick ( ss_replay_tick)
);

amiga_clk amiga_clk_am
(
	.clk_28   ( clk_sys       ),
	.clk7_en  ( am_clk7_en    ),
	.clk7n_en ( am_clk7n_en   ),
	.c1       ( am_c1         ),
	.c3       ( am_c3         ),
	.cck      ( am_cck        ),
	.eclk     ( am_eclk       ),
	.ce       ( ~ss_freeze_7m ),
	.reset_n  ( ~reset_d      )  // same synchronous release as the master copy
);

// TG68K register file sweep. One read port, so the sixteen registers are
// sampled one per clk_sys cycle while the CPU is parked.
reg  [3:0]  ss_reg_index;
wire [31:0] ss_reg_data;
// The saved PC is the kernel's ARCHITECTURAL PC (exe_pc), not TG68_PC.
// TG68_PC is a fetch pointer: mid-instruction it has already run on into the
// operand words, so a state saved with it resumes the CPU decoding data as
// code. That is what the restores that ended in a reset were doing.
wire [31:0] ss_pc;
wire        ss_cpu_at_boundary;
wire        ss_cpu_bus_settled;
wire [15:0] ss_sr;
wire [31:0] ss_usp;
wire [31:0] ss_vbr;
wire  [3:0] ss_cacr;
reg  [31:0] ss_cpu_d0, ss_cpu_d1, ss_cpu_d2, ss_cpu_d3;
reg  [31:0] ss_cpu_d4, ss_cpu_d5, ss_cpu_d6, ss_cpu_d7;
reg  [31:0] ss_cpu_a0, ss_cpu_a1, ss_cpu_a2, ss_cpu_a3;
reg  [31:0] ss_cpu_a4, ss_cpu_a5, ss_cpu_a6, ss_cpu_a7;
reg         ss_regs_valid;

// From minimig.
wire  [3:0] ss_map;
wire        ss_blit_busy;
wire        ss_disk_busy;
wire        ss_audio_busy;

// INTREQ by value, straight out of paula_intcontroller. The register shadow
// rebuilds the other three set/clear registers from bus writes; this one is
// raised by hardware as well as by the CPU, so it has to be read, not inferred.
wire [14:0] ss_intreq;
// Diagnostics only. ss_intena is Paula's live enable mask and ss_frame_count
// counts vertical blanks, so a single readback answers the two questions a
// restore that resumes into a frame-wait loop cannot otherwise distinguish:
// is the chipset still running at all, and did the INTENA replay land.
wire [14:0] ss_intena;
reg  [7:0]  ss_frame_count;   // incremented below, where ss_frame_tick exists
// Which source reset the Amiga, latched since the current restore began. The
// latch is cleared when a restore starts, so whatever it holds afterwards is
// what rebooted the machine as a consequence of that restore.
wire [2:0]  ss_reset_src;
reg         ss_load_busy_d;
always @(posedge clk_sys) ss_load_busy_d <= ss_load_busy;
wire        ss_reset_src_clr = ss_load_busy & ~ss_load_busy_d;

// The address that executed the RESET instruction. nResetOut is asserted by
// exactly one thing (TG68KdotC_Kernel.vhd:518, exec(opcRESET)), so latching
// the architectural PC on its falling edge names the instruction that rebooted
// the Amiga. That single number says whether the game deliberately reset
// itself or the CPU wandered into ROM and Kickstart did -- which the reset
// source alone cannot distinguish.
// The FIRST fault taken after a restore, and how many interrupts ran before
// it. The RESET the machine ends on is Kickstart's own, executed at $00F800D0
// as part of booting -- an effect, not a cause. What matters is the exception
// that put the CPU in ROM to begin with: $08 bus error, $0C address error,
// $10 illegal instruction, $20 privilege violation. Vectors $60 and up are
// interrupt autovectors and are normal, so they are counted rather than
// latched, which also proves interrupts are being taken at all.
reg [15:0] ss_fault_vec;
reg [31:0] ss_fault_pc;
reg  [7:0] ss_int_count;
reg        ss_trap_active_d;
wire [9:0] ss_trap_vector;
wire       ss_trap_active;
always @(posedge clk_sys) begin
	ss_trap_active_d <= ss_trap_active;
	if (ss_reset_src_clr) begin
		ss_fault_vec <= 16'd0;
		ss_fault_pc  <= 32'd0;
		ss_int_count <= 8'd0;
	end
	else if (ss_trap_active & ~ss_trap_active_d) begin
		if (ss_trap_vector >= 10'h060) begin
			if (ss_int_count != 8'hFF) ss_int_count <= ss_int_count + 8'd1;
		end
		else if (ss_fault_vec == 16'd0) begin
			ss_fault_vec <= {6'd0, ss_trap_vector};
			ss_fault_pc  <= ss_pc;
		end
	end
end

reg [31:0] ss_reset_pc;
reg        ss_nrst_out_d;
always @(posedge clk_sys) begin
	ss_nrst_out_d <= cpu_nrst_out;
	if (ss_reset_src_clr)                      ss_reset_pc <= 32'd0;
	// First, not last: Kickstart executes its own RESET while booting, which
	// would otherwise overwrite the interesting one.
	else if (ss_nrst_out_d & ~cpu_nrst_out && ss_reset_pc == 32'd0)
		ss_reset_pc <= ss_pc;
end

// Custom chipset register shadow. Most Amiga custom registers are write-only in
// hardware and this core is faithful about that, so they cannot be exported the
// way the CPU's register file was -- but every write to them passes through the
// two wires minimig.v routes to agnus, paula, denise and both CIAs. The shadow
// snoops those and replays them back through the same buses.
//
// Instantiated here rather than inside minimig.v so all the save state logic
// stays next to ss_ctrl, which owns the freeze and the ordering.
wire  [8:1] ss_rga_addr;
wire [15:0] ss_rga_data;
wire        ss_replay_we;
wire  [8:1] ss_replay_addr;
wire [15:0] ss_replay_data;
wire        ss_replay_active;
wire        ss_replay_done;
wire [15:0] ss_shadow_data;
wire        ss_shadow_writable;
wire        ss_shadow_setclear;

// Driven by ss_ctrl: it reads the shadow out into the payload on a save, loads
// it back on a restore, and starts the replay once chip RAM is in place.
wire        ss_replay_start;
wire  [7:0] ss_shadow_rd;
wire        ss_shadow_ld_we;
wire  [7:0] ss_shadow_ld_addr;
wire [15:0] ss_shadow_ld_data;

ss_regshadow ss_regshadow_inst
(
	.clk            (clk_sys           ),
	.clk7_en        (clk7_en           ),
	.rst_n          (~reset_d          ),
	.reg_address_in (ss_rga_addr       ),
	.data_in        (ss_rga_data       ),
	.rd_addr        (ss_shadow_rd      ),
	.ld_we          (ss_shadow_ld_we   ),
	.ld_addr        (ss_shadow_ld_addr ),
	.ld_data        (ss_shadow_ld_data ),
	.rd_data        (ss_shadow_data    ),
	.rd_writable    (ss_shadow_writable),
	.rd_setclear    (ss_shadow_setclear),
	.replay_start   (ss_replay_start   ),
	// The RESTORED INTREQ, not Paula's live one. The shadow cannot
	// accumulate this register from bus writes -- Paula raises its bits in
	// hardware too -- so it is carried by value in the state vector and
	// comes back out of ss_state_fanout. Wired to ss_intreq, a restore
	// reinstalled the pending interrupts of the machine it was replacing.
	// ss_intreq itself is still the CAPTURE side: it feeds the vector.
	.intreq_in      (ss_restored_intreq),
	.replay_active  (ss_replay_active  ),
	.replay_we      (ss_replay_we      ),
	.replay_addr    (ss_replay_addr    ),
	.replay_data    (ss_replay_data    ),
	.replay_done    (ss_replay_done    )
);

// From chipdma_arb: a bridge (Akiko / CDTV) chip-RAM slot is armed or in
// flight. Joins cpu_boundary so the freeze is never taken with a bridge
// write half-committed; the ss_freeze_7m hold on the arbiter keeps new ones
// from starting once it is.
wire        ss_dma_busy;

// Borrowed sdram_ctrl CPU port.
wire [24:1] ss_sd_addr;
wire        ss_sd_cs;
wire  [1:0] ss_sd_state;
wire        ss_sd_uds_n;
wire        ss_sd_lds_n;
wire        ss_sd_cache_inhibit;
wire [15:0] ss_sd_wr;

// ss_ctrl raises this whenever it is walking the Kickstart ROM for the
// fingerprint. On a SAVE that pass runs inside the freeze and ss_freeze alone
// would have covered it. On a RESTORE it runs before the freeze, deliberately:
// the fingerprint is the last gate, and a gate that stopped the Amiga in order
// to refuse a file would be worse than the file it was guarding against. So
// there is a window where ss_ctrl needs this port with ss_freeze low, and
// ss_rom_scan is what announces it.
wire        ss_rom_scan;

// Invalidate the 68020's cache at the end of a restore, while the machine is
// still frozen. ORed into cpu_cache_ctrl[3] below -- the CACR clear bit, which
// cpu_cache_new edge-detects -- so this reuses the machine's own invalidate
// instead of adding a second mechanism.
//
// It is needed because the restore writes chip RAM through the borrowed SDRAM
// CPU port, while the cache's snoop port is tied to chipWE, the chip DMA write
// path (sdram_ctrl.v:135). DMA writes update the cache; the restore's 2 MB do
// not go near it, so without this the CPU resumes against a cache still
// describing the memory that was there before the restore.
wire        ss_cache_flush;

// Who owns sdram_ctrl's CPU port this cycle. One expression, used for all
// seven of the port's signals at the ram1 instance below.
//
// The 68k is off the port for both windows, by cpu_wrapper's ss_arm rather
// than by anything here: ss_arm is tied to (save_busy | load_busy | fan-out
// busy) at the cpu_wrapper instance, and it parks the CPU at its next
// instruction boundary. That does NOT drop ram_cs on its own -- an
// instruction boundary never has an idle bus -- which is why the freeze also
// waits for ss_cpu_bus_settled, so the outstanding cycle is finished and
// acked before the port changes hands. Both busies are high for
// milliseconds before ss_rom_scan can rise (a restore has an entire payload
// CRC pass to get through first, a save is already frozen), so the CPU is long
// since parked. Even in the impossible case where it were not, the failure is
// benign: the CPU's chip select stops being routed, its ramready never
// arrives, and it resumes the same cycle the scan gives the port back. That is
// the bus stall this design accepts -- ~20 ms once per restore, with the
// chipset still running -- not a freeze.
// ss_peek_scan is the debug window's own port claim. Kept separate from
// ss_rom_scan inside ss_ctrl -- driving that one from the peek states as
// well widened its fan-in enough to fail setup -- and merged here, where
// it costs a single OR gate.
wire        ss_port_own = ss_freeze | ss_rom_scan | ss_peek_scan;

// Where the Kickstart ROM physically lives in SDRAM, as the CPU port's own
// word address -- DERIVED, not assumed. Amiga $F80000 goes through
// memory_router.v (the CPU-side map, which minimig_sram_bridge.v:70-74 mirrors
// for the DMA side):
//
//   ramaddr[26:23] = 4'b0000       -- not z3/rtg/dd, so bit 23 is DROPPED
//                                     ("map a0-ff to 20-7f", memory_router:93)
//   ramaddr[22:19] = cpu_addr[22:19] = 4'b1111
//   ramaddr[18]    = cpu_addr[18]  = 0   at $F80000
//   ramaddr[17:1]  = 0
//
// so the byte address is $F80000 with bit 23 cleared = $780000, and ram1's
// cpuAddr is {2'b00, ram_addr[22:1]}, a WORD address: $780000 >> 1 =
// $3C0000. ss_sd_addr[24:1] is that same vector, so kick_base is 24'h3C0000
// and the 512 KB (0x40000-word) default scan covers $F80000-$FFFFFF.
//
// This is right for every ROM size the uploader supports, which is what makes
// the fingerprint reproducible across a power cycle:
//   512 KB and 1 MB images  -- minimig_config.cpp:286,315 send $F80000, so
//                              the whole 512 KB region is written.
//   256 KB images           -- minimig_config.cpp:336-338 send the image TWICE,
//                              to $F80000 and again to $FC0000, so both halves
//                              of the region hold it. The fingerprint is then a
//                              CRC of the image twice over: still deterministic
//                              and still distinguishing, which is all it has to
//                              be.
// The one gap is an 8 KB A1000 boot ROM (minimig_config.cpp:301,308), where
// only the first 8 KB is ever written and the rest of the region is whatever
// SDRAM powered up holding. A state saved in that configuration will not
// restore across a power cycle -- it refuses with "wrong Kickstart", which is
// a safe direction to be wrong in, and that configuration boots its Kickstart
// off a floppy anyway.
// 24 bits, matching ss_ctrl's kick_base[24:1] port by width.
localparam [23:0] SS_KICK_BASE = 24'h3C0000;

// Borrowed DDR3 arbiter master 0. Both directions: the save path writes, the
// restore path reads the window back out and checks it before touching
// anything.
wire [28:0] ss_ddr_address;
wire [63:0] ss_ddr_writedata;
wire  [7:0] ss_ddr_byteenable;
wire        ss_ddr_write;
wire        ss_ddr_read;
wire [63:0] ss_ddr_readdata;
wire        ss_ddr_readdatavalid;
wire        ss_ddr_waitrequest;
wire        ss_ram_idle;

// Restore path.
wire        ss_load_busy;
wire [`SS_STATE_W-1:0] ss_state_out;
wire        ss_state_we;

// Outcomes, for the OSD toast. Levels held by ss_ctrl until the next attempt.
wire        ss_save_ok;
wire        ss_save_fail;
wire        ss_load_ok;
wire        ss_load_fail;
wire  [3:0] ss_load_fail_code;

// ss_ctrl's diagnostic ports. Observation only; see the ss_diag block at
// the end of this section.
wire  [5:0] ss_dbg_state;
wire [23:0] ss_dbg_idx;
wire        ss_dbg_kick_warn;
wire        ss_dbg_kick_unstable;

// ss_state_fanout's outputs. It runs on clk_sys, which is both the CPU's
// clock and minimig.v's, so nothing it drives needs a domain crossing.
wire  [3:0] ss_fanout_wr_index;
wire [31:0] ss_fanout_wr_data;
wire        ss_fanout_wr_en;
wire        ss_fanout_pc_wr;
wire        ss_fanout_sr_wr;
wire        ss_fanout_usp_wr;
wire        ss_fanout_vbr_wr;
wire        ss_fanout_cacr_wr;
wire        ss_fanout_resume;
wire  [3:0] ss_fanout_map_in;
wire        ss_fanout_map_we;
// The INTREQ that came out of the file, as opposed to the one Paula is
// holding right now. ss_regshadow's replay writes this into Paula.
wire [14:0] ss_restored_intreq;

// The CIAs. ss_cia_a/ss_cia_b are the capture side, straight out of minimig;
// the _restored pair and the pulse are the restore side. Both CIAs take their
// whole word on one clk_sys edge, so there is no sequence here to get wrong.
//
// The capture side is REGISTERED into clk_114 before it joins the state vector.
// The CIA flops live in minimig's clk_sys domain and ss_serdes shifts on
// clk_114, so putting them in the vector raw hands the fitter ~400 timed
// crossings into one shift register -- which failed hold by 0.567 ns across a
// few dozen of them, CIAB1's timer counters into serdes|shifter. One flop on
// this side breaks every one of those paths.
//
// Safe because the data is static exactly when it is read: ss_ctrl serialises
// the vector inside the freeze, and a frozen machine has no clk7_en, so no CIA
// register can change while the capture runs. This is a retiming stage, not a
// synchroniser, and the two clocks come from the same PLL at 4:1.
wire [190:0] ss_cia_a_raw;
wire [202:0] ss_cia_b_raw;

reg  [190:0] ss_cia_a;
reg  [202:0] ss_cia_b;
always @(posedge clk_114) begin
	ss_cia_a <= ss_cia_a_raw;
	ss_cia_b <= ss_cia_b_raw;
end
wire [190:0] ss_restored_cia_a;
wire [202:0] ss_restored_cia_b;
wire         ss_restored_cia_we;

// Registered into clk_sys before they reach the CIAs, for the same reason the
// capture side is registered into clk_114: ss_state_fanout unpacks these
// combinationally from ss_ctrl's clk_114 state_out, so raw they are ~400 timed
// crossings landing directly in CIA flip-flops. One of them failed hold by
// 0.318 ns -- state_out[324] into CIAA1's TOD read latch.
//
// Safe for the same reason too. ss_state_out is loaded once and held, and the
// fan-out does not pulse cia_we until step 22 of its sequence, so these have
// been stable for twenty-odd cycles by the time anything samples them.
reg  [190:0] ss_restored_cia_a_q;
reg  [202:0] ss_restored_cia_b_q;
always @(posedge clk_sys) begin
	ss_restored_cia_a_q <= ss_restored_cia_a;
	ss_restored_cia_b_q <= ss_restored_cia_b;
end
wire        ss_fanout_busy;
wire        ss_fanout_ack;
/////////////////////////////////////////////////////////////////////////////


wire cpu_type = cpucfg[1];
reg  cpu_ph1;
reg  cpu_ph2;
reg  ram_cs;
reg  cyc;

always @(posedge clk_114) begin
	reg [3:0] div;
	reg       c1d;

	div <= div + 1'd1;
	 
	c1d <= c1;
	if (~c1d & c1) div <= 3;
	
	if (~cpu_rst) begin
		cyc <= 0;
		cpu_ph1 <= 0;
		cpu_ph2 <= 0;
	end
	else begin
		cyc <= !div[1:0];
		if (div[1] & ~div[0]) begin
			cpu_ph1 <= 0;
			cpu_ph2 <= 0;
			case (div[3:2])
				0: cpu_ph2 <= 1;
				2: cpu_ph1 <= 1;
			endcase
		end
	end

	ram_cs <= ~(ram_ready & cyc & cpu_type) & ram_sel;
end

wire  [1:0] cpu_state;
wire        cpu_nrst_out;
wire  [3:0] cpu_cacr;
wire [31:0] cpu_nmi_addr;
wire        cpu_rst;

wire  [2:0] chip_ipl;
wire        chip_dtack;
wire        chip_as;
wire        chip_uds;
wire        chip_lds;
wire        chip_rw;
wire [15:0] chip_dout;
wire [15:0] chip_din;
wire [23:1] chip_addr;

wire [28:1] ram_addr;
wire        ram_sel;
wire        ram_lds;
wire        ram_uds;
wire [15:0] ram_din;
wire [15:0] ram_dout  = zram_sel ? ram_dout2  : ram_dout1;
wire        ram_ready = zram_sel ? ram_ready2 : ram_ready1;
wire        zram_sel  = |ram_addr[28:26];
wire        ramshared;

wire [7:0] toccata_base;
wire toccata_ena;

wire a2065_ena;
wire [7:0] a2065_base;


wire cdtv_mode;

// CDTV bridge ↔ cpu_wrapper short-circuit.
// Bridge fires cdtv_selack on every $E90000-$E9FFFF or $DC8000-$DCFFFF
// access; cpu_wrapper's cpu_din mux uses it the same way it uses
// fastchip_selack.
wire [15:0] cdtv_din_w;
wire        cdtv_selack_w;
wire  [5:0] cdtv_ac_rom_addr_w;
wire  [7:0] cdtv_ac_rom_byte_w;

// UIO / HPS-side ports.
// cmd_in_pending / cmd_in_byte come up from cdtv_bridge → cpu_wrapper.
// The cdtv_hps_bridge (instantiated below) drives the *_to_bridge_w
// signals that feed back down through cpu_wrapper into cdtv_bridge.
wire        cdtv_cmd_in_pending_w;
wire  [7:0] cdtv_cmd_in_byte_w;
wire        cdtv_cmd_in_pop_w;
wire        cdtv_cmd_out_push_w;
wire  [7:0] cdtv_cmd_out_data_w;
wire  [9:0] cdtv_cdda_volume_w;
wire        cdtv_nvr_dirty_w;
wire  [7:0] cdtv_nvr_save_dout_w;

// CDTV chip-RAM master DMA wires. cdtv_bridge → chipdma_arb.
wire        cdtv_dma_req_w;
wire        cdtv_dma_we_w;
wire [23:0] cdtv_dma_baddr_w;
wire  [7:0] cdtv_dma_wbyte_w;
wire        cdtv_dma_ack_w;

// CDTV HPS bridge — UIO byte-stream adapter for cdtv_bridge cmd channel.
cdtv_hps_bridge cdtv_hps_bridge_inst
(
	.clk            (clk_sys                ),
	.reset          (reset                  ),
	.uio_cs         (cdtv_cs                ),
	.uio_cs_sec     (cdtv_cs_sec            ),
	.uio_cs_stch    (cdtv_cs_stch           ),
	.uio_wr         (cdtv_wr                ),
	.uio_rd         (cdtv_rd                ),
	.uio_din        (cdtv_dout[7:0]         ),
	.uio_dout       (cdtv_din               ),
	.cmd_in_pending (cdtv_cmd_in_pending_w  ),
	.cmd_in_byte    (cdtv_cmd_in_byte_w     ),
	.cmd_in_pop     (cdtv_cmd_in_pop_w      ),
	.cmd_out_push   (cdtv_cmd_out_push_w    ),
	.cmd_out_data   (cdtv_cmd_out_data_w    ),
	.sec_byte_push  (cdtv_sec_byte_push_w   ),
	.sec_byte_data  (cdtv_sec_byte_data_w   ),
	.sec_space      (cdtv_sec_space_w       ),
	.stch_inject    (cdtv_stch_inject       ),
	.stch_ack       (cdtv_stch_ack          ),
	.stch_ack_clr   (cdtv_stch_ack_clr      ),
	.req            (cdtv_req               )
);

cpu_wrapper cpu_wrapper
(
	.reset        (cpu_rst         ),
	.reset_out    (cpu_nrst_out    ),

	.clk          (clk_sys         ),
	.ph1          (cpu_ph1         ),
	.ph2          (cpu_ph2         ),

	.chip_addr    (chip_addr       ),
	.chip_dout    (chip_dout       ),
	.chip_din     (chip_din        ),
	.chip_as      (chip_as         ),
	.chip_uds     (chip_uds        ),
	.chip_lds     (chip_lds        ),
	.chip_rw      (chip_rw         ),
	.chip_dtack   (chip_dtack      ),
	.chip_ipl     (chip_ipl        ),

	.fastchip_dout   (fastchip_dout   ),
	.fastchip_sel    (fastchip_sel    ),
	.fastchip_lds    (fastchip_lds    ),
	.fastchip_uds    (fastchip_uds    ),
	.fastchip_rnw    (fastchip_rnw    ),
	.fastchip_selack (fastchip_selack ),
	.fastchip_ready  (fastchip_ready  ),
	.fastchip_lw     (fastchip_lw     ),

	.cpucfg       (cpucfg          ),
	.cachecfg     (cachecfg        ),

	// Save state export, and the park request that makes it meaningful.
	//
	// A restore parks the CPU too, and for longer: ss_load_busy rises the
	// moment the request is taken and stays up through validation (a CRC scan
	// of the whole payload) and the replay. Parking that early is not just
	// tidiness -- the DDR3 read master the validation needs is master 0, the
	// same port the CPU's fast RAM uses, and ddram_ctrl will not grant it
	// until that port is idle.
	//
	// ss_fanout_busy extends the park past ss_load_busy's fall, over the
	// handful of cycles in which the register file is actually being written.
	// Releasing the CPU into a half-written register file is the one ordering
	// error in this path that would look like a game bug rather than a save
	// state bug.
	// ss_peek_busy joins them: the SDRAM CPU port will not answer a
	// borrowed read while the 68k is still driving it, which is why the
	// first live peek timed out.
	.ss_arm       (ss_save_busy | ss_load_busy | ss_fanout_busy | ss_peek_busy),
	.ss_reg_index (ss_reg_index    ),
	.ss_reg_data  (ss_reg_data     ),
	.ss_exe_pc    (ss_pc           ),
	.ss_at_boundary(ss_cpu_at_boundary),
	.ss_trap_vector(ss_trap_vector),
	.ss_trap_active(ss_trap_active),
	.ss_bus_settled(ss_cpu_bus_settled),
	.ss_sr        (ss_sr           ),
	.ss_usp       (ss_usp          ),
	.ss_vbr       (ss_vbr          ),
	.ss_cacr      (ss_cacr         ),

	// Restore write port into the TG68K register file, driven by
	// ss_state_fanout at the bottom of this file. One shared data bus and one
	// enable per destination, so the fan-out presents them one per cycle --
	// the register file has a single write port and the sixteen registers go
	// in one at a time, mirroring the Phase 1A read sweep above.
	.ss_wr_index  (ss_fanout_wr_index),
	.ss_wr_data   (ss_fanout_wr_data ),
	.ss_wr_en     (ss_fanout_wr_en   ),
	.ss_pc_wr     (ss_fanout_pc_wr   ),
	.ss_sr_wr     (ss_fanout_sr_wr   ),
	.ss_usp_wr    (ss_fanout_usp_wr  ),
	.ss_vbr_wr    (ss_fanout_vbr_wr  ),
	.ss_cacr_wr   (ss_fanout_cacr_wr ),
	.ss_resume    (ss_fanout_resume  ),
	.fastramcfg   (memcfg[6:4]     ),
	.bootrom      (bootrom         ),

	.toccata_ena  (toccata_ena     ),
	.a2065_ena    (a2065_ena       ),
	.a2065_base   (a2065_base      ),
	.toccata_base (toccata_base    ),
	.cdtv_mode    (cdtv_mode       ),

	// CDTV bridge data path — spec section 1. Short-circuits cpu_din the
	// same cycle cdtv_selack fires, same shape as the fastchip path above.
	.cdtv_din           (cdtv_din_w           ),
	.cdtv_selack        (cdtv_selack_w        ),
	.cdtv_ac_rom_addr   (cdtv_ac_rom_addr_w   ),
	.cdtv_ac_rom_byte   (cdtv_ac_rom_byte_w   ),

	.ramsel       (ram_sel         ),
	.ramaddr      (ram_addr        ),
	.ramlds       (ram_lds         ),
	.ramuds       (ram_uds         ),
	.ramdout      (ram_dout        ),
	.ramdin       (ram_din         ),
	.ramready     (ram_ready       ),
	.ramshared    (ramshared       ),

	//custom CPU signals
	.cpustate     (cpu_state       ),
	.cacr         (cpu_cacr        ),
	.nmi_addr     (cpu_nmi_addr    ),

	// AC-config state exported for chipdma_arb's memory_router.
	.z2ram_ena_out   (z2ram_ena_w     ),
	.z3ram_base0_out (z3ram_base0_w   ),
	.z3ram_ena0_out  (z3ram_ena0_w    ),
	.z3ram_base1_out (z3ram_base1_w   ),
	.z3ram_ena1_out  (z3ram_ena1_w    ),
	// D-cache software toggle from TG68K CACR bit 8.
	.dcache_sw_en    (dcache_sw_en_w  )
);

wire dcache_sw_en_w;


// AC-state exported from cpu_wrapper, fanout to chipdma_arb's
// memory_router so bridge DMA picks the same ram1-vs-ram2 routing the CPU
// would for the same byte address.
wire       z2ram_ena_w;
wire [4:0] z3ram_base0_w;
wire       z3ram_ena0_w;
wire [3:0] z3ram_base1_w;
wire       z3ram_ena1_w;

// ram2 (DDR3) bridge-DMA bus. Driven by chipdma_arb when the
// active master's address falls in a Zorro fast window; ack from ram2's
// new dmaACK port closes the handshake.
wire [28:1] dma_ddr_addr_w;
wire        dma_ddr_l_w;
wire        dma_ddr_u_w;
wire        dma_ddr_we_w;
wire        dma_ddr_cs_w;
wire [15:0] dma_ddr_wr_w;
wire        dma_ddr_ack_w;
// Bridge DMA read return path. ddram_ctrl latches
// the 16-bit word into dma_ddr_rd_w at the same instant it raises
// dma_ddr_ack_w on a read; chipdma_arb's S_DRIVE samples it under
// ddr_ack_safe (2-FF sync ensures data has stabilized).
wire [15:0] dma_ddr_rd_w;

wire [15:0] ram_dout1;
wire        ram_ready1;

sdram_ctrl ram1
(
	.sysclk       (clk_114         ),
	.reset_n      (~reset_d        ),
	.c_7m         (c1              ),

	.cache_rst    (cpu_rst         ),
	.cpu_cache_ctrl(cpu_cacr | {ss_cache_flush, 3'b000}),
	.dcache_sw_en (dcache_sw_en_w  ),

	.sd_data      (SDRAM_DQ        ),
	.sd_addr      (SDRAM_A         ),
	.sd_dqm       ({SDRAM_DQMH, SDRAM_DQML}),
	.sd_cs        (SDRAM_nCS       ),
	.sd_ba        (SDRAM_BA        ),
	.sd_we        (SDRAM_nWE       ),
	.sd_ras       (SDRAM_nRAS      ),
	.sd_cas       (SDRAM_nCAS      ),
	.sd_cke       (SDRAM_CKE       ),
	.sd_clk       (SDRAM_CLK       ),

	// While frozen the CPU port belongs to ss_dma. The CPU itself is parked
	// at an instruction boundary with its last bus cycle settled (see
	// ss_cpu_settled_q), so nothing is in flight to be displaced.
	// cache_inhibit was previously unconnected (and so tied low); ss_dma
	// asserts it for the whole dump because chip DMA writes do not pass
	// through this cache and a cached read could return a stale word.
	//
	// cpuWR has to be muxed as well now that the restore direction exists:
	// ss_dma drives cpustate 3 ("write data") and puts the word on ss_sd_wr.
	// Leaving this tied to ram_din would write whatever the parked CPU
	// happened to leave on its data bus into every chip RAM address the
	// restore touched -- silently, since the addresses and the handshake
	// would all still look correct.
	//
	// ss_port_own, not ss_freeze. See its declaration above: the Kickstart
	// fingerprint pass on the RESTORE side needs this port while ss_freeze is
	// still low, by design, and every one of the seven signals has to move
	// together -- a mux that switched the address but not the chip select, or
	// the state but not the address, would issue the scan's reads against the
	// CPU's address or the CPU's cycle against the scan's, and both of those
	// corrupt a running machine rather than merely failing a restore.
	.cpuWR        (ss_port_own ? ss_sd_wr     : ram_din),
	.cpuAddr      (ss_port_own ? ss_sd_addr   : {2'b00, ram_addr[22:1]}),
	.cpuU         (ss_port_own ? ss_sd_uds_n  : ram_uds),
	.cpuL         (ss_port_own ? ss_sd_lds_n  : ram_lds),
	.cpustate     (ss_port_own ? ss_sd_state  : cpu_state),
	.cpuCS        (ss_port_own ? ss_sd_cs     : (~zram_sel & ram_cs)),
	.cache_inhibit(ss_port_own & ss_sd_cache_inhibit),
	.cpuRD        (ram_dout1       ),
	.ramready     (ram_ready1      ),

	.chipWR       (arb_chip_wr     ),
	.chipAddr     (arb_chip_addr   ),
	.chipU        (arb_chip_u      ),
	.chipL        (arb_chip_l      ),
	.chipRW       (arb_chip_rw     ),
	.chipDMA      (arb_chip_dma    ),
	.chipRD       (ramdata_in      ),
	.chip48       (chip48          )
);

// M5: chip-RAM master arbiter. Sits between minimig's chipset DMA signals
// and sdram_ctrl's chipDMA port. Default forwards minimig untouched. On
// c_7m rising edges where minimig is idle, claims the slot for akiko's
// single-byte master and pulses akiko_dma_ack with the read byte.
chipdma_arb chipdma_arb
(
	.clk             (clk_sys              ),
	.reset           (reset_d              ),
	.c_7m            (c1                   ),

	// From minimig (existing chipset DMA wires). chipAddr is 24-bit word
	// addr; minimig provides only 23 bits (ram_address[23:1]) so pad MSB.
	.chip_in_addr    ({1'b0, ram_address}  ),
	.chip_in_l       (_ram_ble             ),
	.chip_in_u       (_ram_bhe             ),
	.chip_in_rw      (_ram_we              ),
	.chip_in_dma     (_ram_oe              ),
	.chip_in_wr      (ram_data             ),

	// From / to akiko (via fastchip).
	.akiko_dma_req   (akiko_dma_req_w      ),
	.akiko_dma_we    (akiko_dma_we_w       ),
	.akiko_dma_baddr (akiko_dma_baddr_w    ),
	.akiko_dma_wbyte (akiko_dma_wbyte_w    ),
	.akiko_dma_rbyte (akiko_dma_rbyte_w    ),
	.akiko_dma_ack   (akiko_dma_ack_w      ),
	.akiko_arm       (akiko_dma_arm_w      ),

	// From / to cdtv bridge (M2 phase-1b sector DMA — chip-RAM writes).
	.cdtv_dma_req    (cdtv_dma_req_w       ),
	.cdtv_dma_we     (cdtv_dma_we_w        ),
	.cdtv_dma_baddr  (cdtv_dma_baddr_w     ),
	.cdtv_dma_wbyte  (cdtv_dma_wbyte_w     ),
	.cdtv_dma_rbyte  (                     ),  // CDTV is write-only
	.cdtv_dma_ack    (cdtv_dma_ack_w       ),

	// To sdram_ctrl chipDMA port (drives the actual SDRAM access).
	.chip_out_addr   (arb_chip_addr        ),
	.chip_out_l      (arb_chip_l           ),
	.chip_out_u      (arb_chip_u           ),
	.chip_out_rw     (arb_chip_rw          ),
	.chip_out_dma    (arb_chip_dma         ),
	.chip_out_wr     (arb_chip_wr          ),
	.chip_in_rd      (ramdata_in           ),

	// AC-state inputs (memory_router decode) + DDR (ram2) write
	// bus when the bridge address falls in a Zorro fast-RAM window.
	.z2ram_ena       (z2ram_ena_w          ),
	.z3ram_base0     (z3ram_base0_w        ),
	.z3ram_ena0      (z3ram_ena0_w         ),
	.z3ram_base1     (z3ram_base1_w        ),
	.z3ram_ena1      (z3ram_ena1_w         ),

	.ddr_out_addr    (dma_ddr_addr_w       ),
	.ddr_out_l       (dma_ddr_l_w          ),
	.ddr_out_u       (dma_ddr_u_w          ),
	.ddr_out_we      (dma_ddr_we_w         ),
	.ddr_out_cs      (dma_ddr_cs_w         ),
	.ddr_out_wr      (dma_ddr_wr_w         ),
	.ddr_in_ack      (dma_ddr_ack_w        ),
	.ddr_in_rd       (dma_ddr_rd_w         ),

	// Save state: stop granting bridge slots for the duration of the chip
	// RAM dump, and tell the quiescer when a bridge transfer is in flight.
	// ss_freeze_7m rather than ss_freeze because chipdma_arb is a clk_sys
	// module and ss_freeze_7m is already the clk_sys-domain, clk7_en-aligned
	// copy that stops the chipset -- using it makes the bridge stop on the
	// same edge the chipset does. Both are 0 when no save is in progress, so
	// arm_now is unchanged on an idle machine.
	.dma_hold        (ss_freeze_7m         ),
	.dma_busy        (ss_dma_busy          )
);

wire [15:0] ram_dout2;
wire        ram_ready2;

ddram_ctrl ram2
(
	.sysclk       (clk_114         ),
	.reset_n      (~reset_d        ),

	.cache_rst    (cpu_rst         ),
	.cpu_cache_ctrl(cpu_cacr | {ss_cache_flush, 3'b000}),
	.dcache_sw_en (dcache_sw_en_w  ),

	.DDRAM_CLK    (DDRAM_CLK       ),
	.DDRAM_BUSY   (DDRAM_BUSY      ),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT ),
	.DDRAM_ADDR   (DDRAM_ADDR      ),
	.DDRAM_DOUT   (DDRAM_DOUT      ),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD     (DDRAM_RD        ),
	.DDRAM_DIN    (DDRAM_DIN       ),
	.DDRAM_BE     (DDRAM_BE        ),
	.DDRAM_WE     (DDRAM_WE        ),

	.mem2_address      (a2065_mem_address),
	.mem2_burstcount   (a2065_mem_burstcount),
	.mem2_read         (a2065_mem_read),
	.mem2_readdata     (a2065_mem_readdata),
	.mem2_readdatavalid(a2065_mem_readdatavalid),
	.mem2_writedata    (a2065_mem_writedata),
	.mem2_byteenable   (a2065_mem_byteenable),
	.mem2_write        (a2065_mem_write),
	.mem2_waitrequest  (a2065_mem_waitrequest),

	.cpuWR        (ram_din         ),
	.cpuAddr      (ram_addr        ),
	.cpuU         (ram_uds         ),
	.cpuL         (ram_lds         ),
	.cpustate     (cpu_state       ),
	.cpuCS        (zram_sel&ram_cs ),
	.cpuRD        (ram_dout2       ),
	.ramshared    (ramshared       ),
	.ramready     (ram_ready2      ),

	// Save state port, muxed onto DDR3 arbiter master 0. The write side is
	// taken while frozen; the read side is taken for the whole of a restore,
	// which starts well before the freeze because the window is validated
	// against the running machine. ss_load_busy is what asks for it.
	.ss_freeze    (ss_freeze          ),
	.ss_load_busy (ss_load_busy       ),
	.ss_address   (ss_ddr_address     ),
	.ss_writedata (ss_ddr_writedata   ),
	.ss_byteenable(ss_ddr_byteenable  ),
	.ss_write     (ss_ddr_write       ),
	.ss_read      (ss_ddr_read        ),
	.ss_readdata  (ss_ddr_readdata    ),
	.ss_readdatavalid(ss_ddr_readdatavalid),
	.ss_waitrequest(ss_ddr_waitrequest),
	.ss_ram_idle  (ss_ram_idle        ),

	// Bridge (Akiko/CDTV) DMA port — see chipdma_arb.
	// dmaRD carries the read-return word so the
	// bridge can satisfy Akiko's TX command fetches from Z2/Z3.
	.dmaAddr      (dma_ddr_addr_w  ),
	.dmaCS        (dma_ddr_cs_w    ),
	.dmaWE        (dma_ddr_we_w    ),
	.dmaL         (dma_ddr_l_w     ),
	.dmaU         (dma_ddr_u_w     ),
	.dmaWR        (dma_ddr_wr_w    ),
	.dmaRD        (dma_ddr_rd_w    ),
	.dmaACK       (dma_ddr_ack_w   )
);

////////////////////////////  A2065 ETHERNET  ///////////////////////////////
//
// The card is self-contained inside minimig; all that surfaces here is its
// memory port, which shares the core's DDR3 interface with the fast-RAM
// controller above. Fast RAM has priority: it carries every 68k access to
// Zorro RAM, while the card touches DDR3 rarely and can wait.

wire [28:0] a2065_mem_address;
wire [7:0]  a2065_mem_burstcount, a2065_mem_byteenable;
wire        a2065_mem_read, a2065_mem_write;
wire [63:0] a2065_mem_writedata, a2065_mem_readdata;
wire        a2065_mem_readdatavalid, a2065_mem_waitrequest;

wire [15:0] fastchip_dout;
wire        fastchip_sel;
wire        fastchip_lds;
wire        fastchip_uds;
wire        fastchip_rnw;
wire        fastchip_selack;
wire        fastchip_ready;
wire        fastchip_lw;

wire        ide_fast;
wire        ide_f_led;
wire        ide_f_irq;
wire        akiko_f_irq;
wire  [5:0] ide_f_req;
wire [15:0] ide_f_readdata;

// fastchip is working on CPU clock.
// Only high performance 68020 devices are inside
fastchip fastchip
(
	.clk          (clk_114           ),
	.cyc          (cyc               ),
	.clk_sys      (clk_sys           ),

	.reset        (~cpu_rst | ~cpu_nrst_out ),
	.sel          (fastchip_sel      ),
	.sel_ack      (fastchip_selack   ),
	.ready        (fastchip_ready    ),

	.addr         ({chip_addr,1'b0}  ),
	.din          (chip_din          ),
	.dout         (fastchip_dout     ),
	.lds          (~fastchip_lds     ),
	.uds          (~fastchip_uds     ),
	.rnw          (fastchip_rnw      ),
	.longword     (fastchip_lw       ),

	//RTG framebuffer control
	.rtg_ena      (FB_EN             ),
	.rtg_hsize    (FB_WIDTH          ),
	.rtg_vsize    (FB_HEIGHT         ),
	.rtg_format   (FB_FORMAT         ),
	.rtg_base     (FB_BASE           ),
	.rtg_stride   (FB_STRIDE         ),
	.rtg_pal_clk  (FB_PAL_CLK        ),
	.rtg_pal_dw   (FB_PAL_DOUT       ),
	.rtg_pal_dr   (FB_PAL_DIN        ),
	.rtg_pal_a    (FB_PAL_ADDR       ),
	.rtg_pal_wr   (FB_PAL_WR         ),

	.ide_ena      (ide_ena & ide_fast),
	.ide_irq      (ide_f_irq         ),
	.ide_req      (ide_f_req         ),
	.ide_address  (ide_addr          ),
	.ide_write    (ide_wr            ),
	.ide_writedata(ide_dout          ),
	.ide_read     (ide_rd            ),
	.ide_readdata (ide_f_readdata    ),
	.ide_led      (ide_f_led         ),

	.akiko_irq    (akiko_f_irq       ),

	// M5: full chip-RAM master via chipdma_arb (instantiated next to
	// sdram_ctrl). akiko's TX engine reads command bytes from chip RAM,
	// RX engine writes response bytes back; both addressed by
	// dma_baddr[23:0] (byte address). dma_ack pulses one clk_sys cycle
	// per byte; rx_inflight handshake at akiko.v:522-528 expects a
	// >=1-cycle gap between successive acks (chipdma_arb's S_COOLDOWN).
	.akiko_dma_req   (akiko_dma_req_w   ),
	.akiko_dma_we    (akiko_dma_we_w    ),
	.akiko_dma_baddr (akiko_dma_baddr_w ),
	.akiko_dma_wbyte (akiko_dma_wbyte_w ),
	.akiko_dma_rbyte (akiko_dma_rbyte_w ),
	.akiko_dma_ack   (akiko_dma_ack_w   ),
	.akiko_dma_arm   (akiko_dma_arm_w   ),

	// M3: Akiko HPS bridge to hps_ext (akiko_uio_* on fastchip side, akiko_*
	// on hps_ext side — names flip because the two modules describe the
	// same wires from opposite directions). M4 adds akiko_uio_cs_sec
	// (sub-channel discriminator) and akiko_uio_sec_req (PBX-needs-sector
	// status bit).
	.akiko_uio_cs        (akiko_cs        ),
	.akiko_uio_cs_sec    (akiko_cs_sec    ),
	.akiko_uio_cs_nvr    (akiko_cs_nvr    ),
	.akiko_uio_cs_subcode(akiko_cs_subcode),
	.akiko_uio_wr        (akiko_wr        ),
	.akiko_uio_rd        (akiko_rd        ),
	.akiko_uio_din       (akiko_dout      ),
	.akiko_uio_dout      (akiko_din       ),
	.akiko_uio_req       (akiko_req       ),
	.akiko_uio_sec_req   (akiko_sec_req   ),
	.akiko_uio_rx_busy   (akiko_rx_busy   ),
	.akiko_uio_nvr_dirty (akiko_nvr_dirty ),

	// NVRAM load-from-disk (canonical hps_io.ioctl_download path).
	.nvr_load_addr (nvr_load_addr),
	.nvr_load_din  (nvr_load_din ),
	.nvr_load_we   (nvr_load_we  ),

	// M5+ fast sector DMA (canonical UIO_SECTOR_RD pipeline, slot 1).
	.hps_sec_dma_active (akiko_sec_dma_active),
	.hps_sec_dma_byte   (akiko_sec_dma_byte  ),
	.hps_sec_dma_addr   (akiko_sec_dma_addr  ),
	.hps_sec_dma_we     (akiko_sec_dma_we    )
);


////////////////////////////  UART  //////////////////////////////////// 

wire uart_cts, uart_dsr, uart_rts, uart_dtr;
wire uart_tx, uart_rx;

wire hps_mpu = (uart_mode >= 3);

assign UART_RTS = ~hps_mpu & uart_rts;
assign UART_DTR = ~hps_mpu & uart_dtr;
assign uart_cts = ~hps_mpu & UART_CTS;
assign uart_dsr = ~hps_mpu & UART_DSR;
assign uart_rx  = uart_mode ? UART_RXD : midi_rx;
assign UART_TXD = (hps_mpu & mt32_use) | uart_tx;

///////////////////////////////////////////////////////////////////////

//// minimig top ////
wire  [1:0] cpucfg;
wire  [3:0] cachecfg;
wire  [6:0] memcfg;
wire        bootrom;   
wire [15:0] ram_data;      // sram data bus
wire [15:0] ramdata_in;    // sram data bus in
wire [47:0] chip48;        // big chip read
wire [23:1] ram_address;   // sram address bus
wire        _ram_bhe;      // sram upper byte select
wire        _ram_ble;      // sram lower byte select
wire        _ram_we;       // sram write enable
wire        _ram_oe;       // sram output enable
wire [14:0] ldata;         // left DAC data
wire [14:0] rdata;         // right DAC data
wire [9:0]  ldata_okk;     // left DAC data  (PWM vol version)
wire [9:0]  rdata_okk;     // right DAC data (PWM vol version)
wire        vs;
wire        hs;
wire  [1:0] ar;
wire        ntsc;

wire  [5:0] ide_c_req;
wire [15:0] ide_c_readdata;
wire        ide_c_led;
wire        ide_ena;

wire [15:0] toccata_aud_left;
wire [15:0] toccata_aud_right;

minimig minimig
(
	//m68k pins
	.cpu_address  (chip_addr        ), // M68K address bus
	.cpu_data     (chip_dout        ), // M68K data bus
	.cpudata_in   (chip_din         ), // M68K data in
	._cpu_ipl     (chip_ipl         ), // M68K interrupt request
	._cpu_as      (chip_as          ), // M68K address strobe
	._cpu_uds     (chip_uds         ), // M68K upper data strobe
	._cpu_lds     (chip_lds         ), // M68K lower data strobe
	.cpu_r_w      (chip_rw          ), // M68K read / write
	._cpu_dtack   (chip_dtack       ), // M68K data acknowledge
	._cpu_reset   (cpu_rst          ), // M68K reset
	._cpu_reset_in(cpu_nrst_out     ), // M68K reset out
	.nmi_addr     (cpu_nmi_addr     ), // M68K NMI address

	//sram pins
	.ram_data     (ram_data         ), // SRAM data bus
	.ramdata_in   (ramdata_in       ), // SRAM data bus in
	.ram_address  (ram_address      ), // SRAM address bus
	._ram_bhe     (_ram_bhe         ), // SRAM upper byte select
	._ram_ble     (_ram_ble         ), // SRAM lower byte select
	._ram_we      (_ram_we          ), // SRAM write enable
	._ram_oe      (_ram_oe          ), // SRAM output enable
	.chip48       (chip48           ), // big chipram read

	//system  pins
	.rst_ext      (reset_d          ), // reset from ctrl block
	.rst_out      (                 ), // minimig reset status
	.clk          (clk_sys          ), // output clock c1 ( 28.687500MHz)
	// Freezable copy of the Amiga timebase -- see amiga_clk_am above. The
	// RTC block inside minimig runs off raw clk and is deliberately not
	// frozen, so a state resumed tomorrow sees tomorrow's time.
	.clk7_en      (am_clk7_en       ), // 7MHz clock enable
	.clk7n_en     (am_clk7n_en      ), // 7MHz negedge clock enable
	.c1           (am_c1            ), // clk28m clock domain signal synchronous with clk signal
	.c3           (am_c3            ), // clk28m clock domain signal synchronous with clk signal delayed by 90 degrees
	.cck          (am_cck           ), // colour clock output (3.54 MHz)
	.eclk         (am_eclk          ), // 0.709379 MHz clock enable output (clk domain pulse)

	//rs232 pins
	.rxd          (uart_rx          ), // RS232 receive
	.txd          (uart_tx          ), // RS232 send
	.cts          (uart_cts         ), // RS232 clear to send
	.rts          (uart_rts         ), // RS232 request to send
	.dtr          (uart_dtr         ), // RS232 Data Terminal Ready
	.dsr          (uart_dsr         ), // RS232 Data Set Ready
	.cd           (uart_dsr         ), // RS232 Carrier Detect
	.ri           (1                ), // RS232 Ring Indicator

	//I/O
	._joy1        (~JOY0_MUX        ), // joystick 1 [fire4,fire3,fire2,fire,up,down,left,right] (default mouse port)
	._joy2        (~JOY1_MUX        ), // joystick 2 [fire4,fire3,fire2,fire,up,down,left,right] (default joystick port)
	._joy3        (~JOY2            ), // joystick 1 [fire4,fire3,fire2,fire,up,down,left,right]
	._joy4        (~JOY3            ), // joystick 2 [fire4,fire3,fire2,fire,up,down,left,right]
	.joya1        (JOYA0            ),
	.joya2        (JOYA1            ),
	.mouse_btn    (mouse_buttons    ), // mouse buttons
	.kbd_mouse_data (kbd_mouse_data ), // mouse direction data, keycodes
	.kbd_mouse_type (kbd_mouse_type ), // type of data
	.kms_level    (kbd_mouse_level  ),
	.pwr_led      (pwr_led          ), // power led
	.fdd_led      (LED_USER         ),
	.hdd_led      (ide_c_led        ),
	.rtc          (RTC              ),

	//host controller interface (SPI)
	.IO_UIO       (io_uio           ),
	.IO_FPGA      (io_fpga          ),
	.IO_STROBE    (io_strobe        ),
	.IO_WAIT      (io_wait          ),
	.IO_DIN       (io_din           ),
	.IO_DOUT      (fpga_dout        ),

	//video
	._hsync       (hs               ), // horizontal sync
	._vsync       (vs               ), // vertical sync
	.field1       (field1           ),
	.lace         (lace             ),
	.red          (r                ), // red
	.green        (g                ), // green
	.blue         (b                ), // blue
	.hblank       (hblank           ),
	.vblank       (vbl              ),
	.ar           (ar               ),
	.scanline     (fx               ),
	//.ce_pix     (ce_pix           ),
	.res          (res              ),
	.ntsc         (ntsc             ),

	//audio
	.ldata        (ldata            ), // left DAC data
	.rdata        (rdata            ), // right DAC data
	.ldata_okk    (ldata_okk        ), // 9bit
	.rdata_okk    (rdata_okk        ), // 9bit

	.aud_mix      (AUDIO_MIX        ),

	//toccata soundcard
	.toccata_ena  (toccata_ena),
	.toccata_base (toccata_base),
	.a2065_ena  (a2065_ena),
	.a2065_base (a2065_base),
	.toccata_aud_left (toccata_aud_left),
	.toccata_aud_right(toccata_aud_right),

	.cdtv_mode    (cdtv_mode        ),

	// CDTV bridge — short-circuit data path back up to cpu_wrapper.
	.cdtv_din            (cdtv_din_w           ),
	.cdtv_selack         (cdtv_selack_w        ),
	.cdtv_ac_rom_addr    (cdtv_ac_rom_addr_w   ),
	.cdtv_ac_rom_byte    (cdtv_ac_rom_byte_w   ),

	// CDTV bridge — UIO / HPS-side ports. cmd byte-stream wired via
	// cdtv_hps_bridge_inst above (M2 phase-1a). Sector-push channel added
	// in phase-1b alongside the chip-RAM master DMA path. Subq/status
	// optional channels still tied off — those come in later phases.
	.cdtv_cmd_in_pop     (cdtv_cmd_in_pop_w    ),
	.cdtv_cmd_in_pending (cdtv_cmd_in_pending_w),
	.cdtv_cmd_in_byte    (cdtv_cmd_in_byte_w   ),
	.cdtv_cmd_out_push   (cdtv_cmd_out_push_w  ),
	.cdtv_cmd_out_data   (cdtv_cmd_out_data_w  ),
	.cdtv_sec_byte_push  (cdtv_sec_byte_push_w ),
	.cdtv_sec_byte_data  (cdtv_sec_byte_data_w ),
	.cdtv_sec_space      (cdtv_sec_space_w     ),
	.cdtv_sec_fifo_empty (cdtv_sec_empty_w     ),
	.cdtv_subq_push      (1'b0                 ),
	.cdtv_subq_byte      (8'h00                ),
	.cdtv_stch_pulse     (cdtv_stch_inject     ),
	.cdtv_stch_ack       (cdtv_stch_ack        ),
	.cdtv_stch_ack_clr   (cdtv_stch_ack_clr    ),
	.cdtv_sten_pulse     (1'b0                 ),
	.cdtv_scor_pulse     (1'b0                 ),
	.cdtv_sbcp_pulse     (1'b0                 ),

	// CDTV chip-RAM master DMA — bridge requests, chipdma_arb acks.
	.cdtv_dma_req        (cdtv_dma_req_w       ),
	.cdtv_dma_we         (cdtv_dma_we_w        ),
	.cdtv_dma_baddr      (cdtv_dma_baddr_w     ),
	.cdtv_dma_wbyte      (cdtv_dma_wbyte_w     ),
	.cdtv_dma_ack        (cdtv_dma_ack_w       ),

	.cdtv_nvr_load_addr  (14'h0                ),
	.cdtv_nvr_load_din   (8'h0                 ),
	.cdtv_nvr_load_we    (1'b0                 ),
	.cdtv_nvr_save_addr  (14'h0                ),
	.cdtv_nvr_save_dout  (cdtv_nvr_save_dout_w ),
	.cdtv_nvr_dirty      (cdtv_nvr_dirty_w     ),
	.cdtv_nvr_clear_dirty(1'b0                 ),

	.cdtv_cdda_volume    (cdtv_cdda_volume_w   ),

	//user i/o
	.cpucfg       (cpucfg           ), // CPU config
	.cachecfg     (cachecfg         ), // Cache config
	.memcfg       (memcfg           ), // memory config
	.bootrom      (bootrom          ), // bootrom mode. Needed here to tell tg68k to also mirror the 256k Kickstart 

	.ide_fast     (ide_fast         ),
	.ide_ext_irq  (ide_f_irq        ),
	.akiko_irq    (akiko_f_irq      ),
	.ide_ena      (ide_ena          ),
	.ide_req      (ide_c_req        ),
	.ide_address  (ide_addr         ),
	.ide_write    (ide_wr           ),
	.ide_writedata(ide_dout         ),
	.ide_read     (ide_rd           ),
	.ide_readdata (ide_c_readdata   ),

	.a2065_clk_ddr(DDRAM_CLK),
	.a2065_mem_address(a2065_mem_address),
	.a2065_mem_burstcount(a2065_mem_burstcount),
	.a2065_mem_read(a2065_mem_read),
	.a2065_mem_readdata(a2065_mem_readdata),
	.a2065_mem_readdatavalid(a2065_mem_readdatavalid),
	.a2065_mem_writedata(a2065_mem_writedata),
	.a2065_mem_byteenable(a2065_mem_byteenable),
	.a2065_mem_write(a2065_mem_write),
	.a2065_mem_waitrequest(a2065_mem_waitrequest),

	.USER_IN              (USER_IN              ),
	.USER_OUT             (IndirectUserOutFlop  ),
	.user_port_mode       (user_port_mode       ),
	.snac_mode            (snac_mode            ),
	.mister_floppy_status (mister_floppy_status ),

	.ss_blit_busy         (ss_blit_busy         ),
	.ss_disk_busy         (ss_disk_busy         ),
	.ss_audio_busy        (ss_audio_busy        ),
	.ss_intreq            (ss_intreq            ),
	.ss_intena            (ss_intena            ),
	.ss_reset_src         (ss_reset_src         ),
	.ss_reset_src_clr     (ss_reset_src_clr     ),
	.ss_rga_addr          (ss_rga_addr          ),
	.ss_rga_data          (ss_rga_data          ),
	.ss_replay_we         (ss_replay_we         ),
	.ss_replay_addr       (ss_replay_addr       ),
	.ss_replay_data       (ss_replay_data       ),
	.ss_replay_tick       (ss_replay_tick       ),
	.ss_map               (ss_map               ),
	// Restore side of the same four bits, in the same bit order. minimig.v
	// applies [3] to ovl and [2] to gary's rom_readonly; [1:0] are
	// combinational address decodes with no target -- see rtl/ss_state.vh.
	.ss_map_in            (ss_fanout_map_in     ),
	.ss_map_we            (ss_fanout_map_we     ),
	.ss_cia_a             (ss_cia_a_raw         ),
	.ss_cia_b             (ss_cia_b_raw         ),
	.ss_cia_a_in          (ss_restored_cia_a_q  ),
	.ss_cia_b_in          (ss_restored_cia_b_q  ),
	.ss_cia_we            (ss_restored_cia_we   )
);

//////////////////////////  SAVE STATES (phase 1A)  /////////////////////////

// Request source. menu.cpp's System page drives these: status[51] is the save
// request, status[53:52] picks one of the four DDR3 slot windows the host maps
// (user_io.cpp process_ss). The slot is written immediately before the request
// and never while a save is running, so slot_base below is stable for the whole
// dump.
wire       ss_save_req_osd = status[51];
wire [1:0] ss_slot         = status[53:52];
// The restore request. status[54] is set by the "Restore state" row on
// menu.cpp's AmigaCD Settings page (MENU_AMIGACD_SETTINGS1/2, menusub 6). The
// two have to agree on the bit number or the row does nothing at all and says
// nothing about it.
wire       ss_load_req_osd = status[54];

// *** fx68k lockout ***
// cpu_wrapper's ss_* export is taken from cpu_inst_p (TG68K) unconditionally,
// bypassing the cpucfg mux at cpu_wrapper.v:211. When cpucfg == 0 the running
// CPU is fx68k, which has no equivalent export at all -- not the register
// file, not PC, SR or USP -- so a state captured in that mode would describe a
// CPU that is not executing. fx68k is deferred to a later phase, and until it
// lands save states are refused rather than silently wrong. Gating the request
// is enough: save_busy (and therefore freeze, the CPU park and every port
// takeover below) can only ever rise out of ss_ctrl's S_IDLE on save_req.
wire ss_supported   = |cpucfg;
// *** the request is an EDGE, and it is made one HERE, not in userspace ***
// ss_ctrl re-arms out of S_IDLE on any clock where save_req is high, and it
// drops back to S_IDLE the moment a save finishes. Feeding it status[51] as a
// LEVEL therefore starts the next save on the cycle after the previous one
// ends, forever: the machine spends ~0.2 s frozen out of every ~0.2 s and the
// Amiga never runs again. Userspace does clear the bit after setting it, but
// that clear must not be what stands between the user and a wedged machine --
// one dropped SPI write, one stalled menu task, or one .cfg that happens to
// carry bit 51 set would be enough. So the one-shot is enforced in RTL.
//
// ss_save_pending is SET by the 0->1 edge of status[51] and CLEARED as soon as
// ss_ctrl has taken the request (save_busy). It cannot re-trigger:
//  - Holding status[51] high yields exactly one edge and therefore exactly one
//    save; ss_save_req_d is 1 on every subsequent cycle, so ss_save_edge is 0.
//  - The clear branches are ahead of the set branch, so an edge arriving while
//    a save is already running is dropped, not queued.
//  - save_busy stays high for the whole ~0.2 s dump and pending is cleared one
//    clock after it rises, so by the time ss_ctrl is back in S_IDLE the request
//    has been low for millions of cycles. S_IDLE only ever sees a request it
//    just got a fresh edge for.
//  - ss_save_req_d is clocked unconditionally, INCLUDING while reset_d is
//    asserted, so a status bit that is already high when reset releases reads
//    as a level, not as an edge, and does not fire a save.
//
// The latch is also what makes the CDTV sector-FIFO hold-off below work: that
// state machine needs a request that stays asserted across several video
// frames, which a bare one-cycle pulse could not provide.
//
// !ss_supported clears the latch rather than merely masking it downstream, so
// a request made in an unsupported configuration is discarded outright instead
// of lying in wait for the CPU to be switched back to TG68K.
//
// status[] is an hps_io register in clk_sys and this samples it in clk_114.
// The two come off the same PLL at 4:1, so this is a timed path, not a CDC --
// the same argument ss_dma_busy and cdtv_sec_empty_w already rely on. One SPI
// update of the bit therefore produces exactly one clk_114 edge, at worst one
// clk_114 cycle late.
reg  ss_save_req_d;
reg  ss_save_pending;
wire ss_save_edge = ss_save_req_osd & ~ss_save_req_d;

always @(posedge clk_114) begin
	ss_save_req_d <= ss_save_req_osd;
	if (reset_d || !ss_supported) ss_save_pending <= 1'b0;
	else if (ss_save_busy)        ss_save_pending <= 1'b0;
	else if (ss_save_edge)        ss_save_pending <= 1'b1;
end

// The raw request. ss_save_req itself is derived further down, after the
// CDTV sector-FIFO hold-off (it needs ss_frame_tick, declared below).
wire ss_save_req_raw = ss_save_pending;

// Restore request, latched by exactly the same rules and for exactly the same
// reasons -- ss_ctrl re-arms out of S_IDLE on any clock where load_req is
// high, so a level would restore over and over and the Amiga would never run
// again. Cleared on load_busy, which ss_ctrl raises the cycle it takes the
// request. The ss_supported lockout applies unchanged: there is no way to
// write fx68k's registers, so a restore in that mode would put chip RAM back
// underneath a CPU whose own state was never restored -- worse than refusing.
reg  ss_load_req_d;
reg  ss_load_pending;
wire ss_load_edge = ss_load_req_osd & ~ss_load_req_d;

always @(posedge clk_114) begin
	ss_load_req_d <= ss_load_req_osd;
	if (reset_d || !ss_supported) ss_load_pending <= 1'b0;
	else if (ss_load_busy)        ss_load_pending <= 1'b0;
	else if (ss_load_edge)        ss_load_pending <= 1'b1;
end

wire ss_load_req_raw = ss_load_pending;

// Sweep the TG68K register file read port while the CPU is parked. Sixteen
// cycles at 28 MHz is 560 ns, which is nothing against the dump itself.
//
// This runs *before* the freeze, not during it: ss_serdes latches the entire
// state vector on the one cycle ss_ctrl asserts save_start, which is a single
// clock after quiesced, so a sweep that only started at freeze time would
// serialise sixteen uninitialised registers. ss_save_busy parks the CPU at
// its next instruction boundary (see cpu_wrapper's ss_arm), the sweep runs
// there, and ss_regs_valid is what finally lets cpu_boundary go true.
//
// ss_load_busy is in here as well as ss_save_busy, and it has to be: it is
// what makes cpu_boundary reachable at all on a restore. ss_quiesce will not
// declare the machine quiesced without it, so a restore whose park condition
// only looked at ss_save_busy would validate the file, request the freeze and
// then time out with FAIL_QUIESCE every single time. The register sweep the
// block below performs during a restore is harmless -- it is a read port, and
// the values it lands in ss_cpu_d0..a7 are overwritten by the next save.
//
// The park point is the CPU's instruction boundary, not cpu_state==1. A
// no-memaccess cycle is an internal step in the MIDDLE of an instruction: the
// state captured there is half-executed, and restoring it derails the machine
// even though the PC written back is exactly the one that was saved. That is
// the whole of the reset-on-restore bug.
//
// ss_cpu_bus_settled is latched rather than used directly: the ready it
// reports can be a single cycle, and the parked CPU never consumes it, so
// sampling it live would deadlock the sixteen-cycle register sweep below.
// The latch clears whenever the CPU is not on a boundary, so it can only be
// set by a ready seen during THIS park.
reg ss_cpu_settled_q;
always @(posedge clk_sys) begin
	if (!ss_cpu_at_boundary)     ss_cpu_settled_q <= 1'b0;
	else if (ss_cpu_bus_settled) ss_cpu_settled_q <= 1'b1;
end

// ss_peek_busy belongs here for the same reason ss_load_busy does. The live
// peek freezes the machine to borrow the SDRAM CPU port, and ss_quiesce will
// not declare it quiesced without cpu_boundary, so a peek that was not in this
// term reached S_PEEK_FREEZE, waited, and fell back to idle with freeze still
// low -- every peek returning "not valid yet" and never a byte of memory.
wire ss_cpu_parked = (ss_save_busy | ss_load_busy | ss_peek_busy)
                     & ss_cpu_at_boundary & ss_cpu_settled_q;

always @(posedge clk_sys) begin
	if (!ss_cpu_parked) begin
		ss_reg_index  <= 4'd0;
		ss_regs_valid <= 1'b0;
	end
	else if (!ss_regs_valid) begin
		case (ss_reg_index)
		4'd0:  ss_cpu_d0 <= ss_reg_data;
		4'd1:  ss_cpu_d1 <= ss_reg_data;
		4'd2:  ss_cpu_d2 <= ss_reg_data;
		4'd3:  ss_cpu_d3 <= ss_reg_data;
		4'd4:  ss_cpu_d4 <= ss_reg_data;
		4'd5:  ss_cpu_d5 <= ss_reg_data;
		4'd6:  ss_cpu_d6 <= ss_reg_data;
		4'd7:  ss_cpu_d7 <= ss_reg_data;
		4'd8:  ss_cpu_a0 <= ss_reg_data;
		4'd9:  ss_cpu_a1 <= ss_reg_data;
		4'd10: ss_cpu_a2 <= ss_reg_data;
		4'd11: ss_cpu_a3 <= ss_reg_data;
		4'd12: ss_cpu_a4 <= ss_reg_data;
		4'd13: ss_cpu_a5 <= ss_reg_data;
		4'd14: ss_cpu_a6 <= ss_reg_data;
		4'd15: ss_cpu_a7 <= ss_reg_data;
		endcase
		if (ss_reg_index == 4'd15) ss_regs_valid <= 1'b1;
		ss_reg_index <= ss_reg_index + 4'd1;
	end
end

// Gary's memory map state, unpacked into the names ss_state.vh uses.
wire ss_ovl                = ss_map[3];
wire ss_rom_readonly       = ss_map[2];
wire ss_sel_kick1mb        = ss_map[1];
wire ss_sel_kick256kmirror = ss_map[0];

wire [`SS_STATE_W-1:0] ss_state_in = `SS_STATE_LIST;

// Quiesce timeout reference. vbl is minimig's raw vertical blank, so it stops
// once the chipset freezes -- which is fine, the timeout only matters before
// the freeze.
reg ss_vbl_d;
always @(posedge clk_114) ss_vbl_d <= vbl;
wire ss_frame_tick = vbl & ~ss_vbl_d;

// Diagnostic frame counter: free-running proof that the chipset is still
// generating vertical blanks. A restore that leaves the 68k running but the
// chipset stopped looks identical from the CPU side -- it sits in the game's
// frame-wait loop either way -- and this is what tells the two apart.
always @(posedge clk_sys) if (ss_frame_tick) ss_frame_count <= ss_frame_count + 8'd1;

// --- CDTV sector-FIFO hold-off on the save request ---------------------------
//
// The freeze holds chipdma_arb off for the whole ~0.2 s chip RAM dump
// (dma_hold), so cdtv_bridge's 8 KB sector FIFO cannot drain a single byte
// while a save is running. Userspace throttles against the FIFO's free-space
// credit (cdtv_bridge.sec_space, read back over the 0xF820 sub-channel), which
// is what actually prevents byte loss. This hold-off is the cheap second half:
// starting the freeze with the FIFO already drained hands userspace the full
// 8 KB of headroom before it has to block, which at 1x (~150 KB/s) covers the
// first ~53 ms of the dump for free and keeps the CD stream from visibly
// stalling on a short save.
//
// It is deliberately a PREFERENCE, not a precondition. cdtv_bridge carries
// leftover bytes in sec_fifo between DMA chunks by design (see the "No
// dmac_dma gate" comment there), so "FIFO empty" is a state a healthy CDTV can
// sit out of indefinitely -- gating the request on it outright would make
// saves fail forever on some titles. The hold expires after SS_SEC_HOLD_FRAMES
// video frames and the save proceeds regardless; correctness at that point
// rests entirely on the userspace credit throttle, which is where it belongs.
//
// Inert on every non-CDTV configuration: cdtv_bridge's sec_wr_p / sec_rd_p
// both reset to 0 and only move on a UIO sector push, so cdtv_sec_empty_w is
// constantly 1 when nothing is streaming, ss_sec_go is set on the first clock
// after the request and the request passes through with no delay at all.
// cdtv_sec_empty_w is a clk_sys signal sampled here in clk_114; the two clocks
// come off the same PLL at 4:1, so this is a timed path, not a CDC -- the same
// argument ss_dma_busy already relies on below.
//
// ss_sec_go LATCHES the moment the hold-off is satisfied and stays latched
// until the request itself drops. It must not be a live comparison: once the
// freeze is up the FIFO stops draining and userspace immediately refills it, so
// a live term would go false mid-dump, drop save_req and abort the save it was
// meant to protect.
//
// A RESTORE freezes the machine for the same reasons and for a comparable
// length of time, so it is held off by the same latch. The two requests share
// it because they cannot be pending together: ss_ctrl serves one at a time and
// each pending latch is cleared the cycle its own busy rises.
localparam [3:0] SS_SEC_HOLD_FRAMES = 4'd4;

wire ss_ss_req_raw = ss_save_req_raw | ss_load_req_raw;

reg [3:0] ss_sec_wait;
reg       ss_sec_go;
always @(posedge clk_114) begin
	if (reset_d || !ss_ss_req_raw) begin
		ss_sec_wait <= 4'd0;
		ss_sec_go   <= 1'b0;
	end
	else begin
		if (ss_frame_tick && (ss_sec_wait != SS_SEC_HOLD_FRAMES))
			ss_sec_wait <= ss_sec_wait + 4'd1;
		if (cdtv_sec_empty_w || (ss_sec_wait == SS_SEC_HOLD_FRAMES))
			ss_sec_go   <= 1'b1;
	end
end

wire ss_save_req = ss_save_req_raw & ss_sec_go;
wire ss_load_req = ss_load_req_raw & ss_sec_go;

// Save state window: 0x3E000000, four 4 MB slots. DDRAM_ADDR is a 64-bit word
// address, so the byte base is shifted right by three.
//
// The byte address must be written as a 32-bit literal, not the 29-bit one
// the plan used: 0x3E000000 has bit 29 set, so 29'h3E000000 is truncated to
// 0x1E000000 before the shift and the window lands at byte 0xF000000 --
// below DDR3's valid base, and squarely inside fast RAM. The shifted result
// does fit in 29 bits, which is why the destination width is still 29.
localparam [28:0] SS_SLOT_BASE   = 29'h07C00000;   // byte 0x3E000000 >> 3
localparam [28:0] SS_SLOT_STRIDE = 29'h00080000;   // byte 0x00400000 >> 3

ss_ctrl #(.STATE_W(`SS_STATE_W), .CHIP_WORDS(24'h100000)) savestate
(
	.clk          (clk_114),
	.rst_n        (~reset_d),
	.slot_base    (SS_SLOT_BASE + SS_SLOT_STRIDE * ss_slot),

	.save_req     (ss_save_req),
	.save_busy    (ss_save_busy),
	// Outcomes. All five are levels, held until the next attempt starts, and
	// all five reach the user as an OSD toast -- see the ss_info_* block after
	// this instance. An unconnected outcome makes a refusal silent, which is
	// the exact failure mode the fail codes exist to remove.
	.save_ok      (ss_save_ok),
	.save_fail    (ss_save_fail),

	// The restore path is live end to end: the OSD's "Restore state" row sets
	// status[54], the edge latch above turns it into one request, the DDR3
	// read master below is real, and the state vector reaches the machine
	// through ss_state_fanout at the bottom of this file.
	.load_req     (ss_load_req),
	.load_busy    (ss_load_busy),
	.load_ok      (ss_load_ok),
	.load_fail    (ss_load_fail),
	.load_fail_code(ss_load_fail_code),

	.state_in     (ss_state_in),
	.state_out    (ss_state_out),
	.state_we     (ss_state_we),

	// cpu_boundary carries three extra conditions beyond "the CPU is parked".
	// ss_regs_valid, because the register sweep must be complete before the
	// vector is latched (see above). ss_ram_idle, because a DDR3 read already
	// accepted for master 0 returns its data out of band, and taking master 0
	// away in that window would lose the readdatavalid pulse -- see
	// ddram_ctrl.v. ~ss_dma_busy, because chipdma_arb is a chip RAM WRITE
	// master for the Akiko and CDTV bridges and neither is stopped by the CPU
	// park or the chipset freeze; freezing with one of their slots
	// half-committed would put a write into chip RAM at an unknown point
	// relative to the dump. None of the three is a property of the CPU, but
	// cpu_boundary is the only input ss_quiesce has left for "not yet".
	//
	// ~ss_dma_busy closes the freeze INSTANT only. The rest of the ~0.2 s
	// dump is closed by chipdma_arb's dma_hold input (tied to ss_freeze_7m
	// above), which stops the arbiter granting any further bridge slot. The
	// two together are what make the snapshot self-consistent; either alone
	// only moves the tear.
	//
	// ss_dma_busy is a clk_sys signal sampled here in clk_114. So are
	// ss_blit_busy / ss_disk_busy / ss_audio_busy (minimig.v runs on
	// clk_sys): the two clocks come from the same PLL at 4:1, so these are
	// timed paths, not CDCs.
	.blit_busy    (ss_blit_busy),
	.disk_busy    (ss_disk_busy),
	.audio_busy   (ss_audio_busy),
	.cpu_boundary (ss_cpu_parked & ss_regs_valid & ss_ram_idle & ~ss_dma_busy),
	.frame_tick   (ss_frame_tick),
	.freeze       (ss_freeze),

	.chip_base    (24'h000000),

	// Kickstart fingerprint. See SS_KICK_BASE for the derivation and
	// ss_port_own for the mux this output drives. KICK_WORDS keeps its 512 KB
	// default, which is exactly the $F80000-$FFFFFF region SS_KICK_BASE points
	// at.
	.kick_base    (SS_KICK_BASE),
	.rom_scan     (ss_rom_scan),
	.cache_flush  (ss_cache_flush),

	// Chipset register shadow. See ss_regshadow above.
	.shadow_rd_addr (ss_shadow_rd      ),
	.shadow_rd_data (ss_shadow_data    ),
	.shadow_ld_we   (ss_shadow_ld_we   ),
	.shadow_ld_addr (ss_shadow_ld_addr ),
	.shadow_ld_data (ss_shadow_ld_data ),
	.replay_start   (ss_replay_start   ),
	.replay_done    (ss_replay_done    ),

	.sd_addr      (ss_sd_addr),
	.sd_cs        (ss_sd_cs),
	.sd_state     (ss_sd_state),
	.sd_uds_n     (ss_sd_uds_n),
	.sd_lds_n     (ss_sd_lds_n),
	.sd_cache_inhibit(ss_sd_cache_inhibit),
	.sd_wr        (ss_sd_wr),
	.sd_rd        (ram_dout1),
	.sd_ready     (ram_ready1),

	.ddr_address  (ss_ddr_address),
	.ddr_writedata(ss_ddr_writedata),
	.ddr_byteenable(ss_ddr_byteenable),
	.ddr_write    (ss_ddr_write),
	// The read master is real: ddram_ctrl grants master 0 to the savestate
	// port for the whole of a restore (see its ss_port_own), and returns the
	// beat on ss_readdatavalid. All three go together with load_req -- the
	// read step in ss_ctrl has no watchdog, so a half-wiring parks the
	// machine rather than failing it.
	.ddr_read     (ss_ddr_read),
	.ddr_readdata (ss_ddr_readdata),
	.ddr_readdatavalid(ss_ddr_readdatavalid),
	.ddr_waitrequest(ss_ddr_waitrequest),

	.dbg_state    (ss_dbg_state),
	.dbg_idx      (ss_dbg_idx),
	// Advisory: a restore ran with a Kickstart fingerprint that did not
	// match the file's. The gate is off while the scan is untrustworthy;
	// this is how the host still hears about it.
	.dbg_kick_warn(ss_dbg_kick_warn),
	// Live peek. The address is stable for as long as the request stands,
	// so it crosses without synchronising; only the pulse needs care.
	.peek_req(ss_peek_req_114),
	.peek_addr(ss_peek_addr),
	.peek_data(ss_peek_data),
	.peek_valid(ss_peek_valid),
	.peek_busy(ss_peek_busy),
	.peek_scan(ss_peek_scan),
	.cpu_pc(ss_pc),
	.pc_snapshot(ss_pc_snapshot),
	.kick_pair(ss_kick_pair),
	.dbg_kick_unstable(ss_dbg_kick_unstable)
);

// --- outcome toast -----------------------------------------------------------
//
// Every save and every restore ends by naming what happened, in words, on the
// OSD. Without this a refusal is a no-op the user cannot tell from a row that
// did nothing: the fail codes exist precisely so that "it didn't work" can be
// "this state was made under a different Kickstart", and a code that reaches no
// display is a code that was never computed.
//
// The transport is the framework's own: hps_io latches `info` on a rising edge
// of `info_req` and holds it until user_io.cpp's once-a-second UIO_INFO_GET
// poll reads and clears it (hps_io.sv:296,337); show_core_info() then indexes
// the CONF_STR "I" line by that number and calls Info(). So the strings live in
// CONF_STR at the top of this file and no host code changes at all -- which
// matters, because there is no local ARM toolchain to compile host changes
// against.
//
// This runs in clk_sys, hps_io's domain, sampling ss_ctrl's clk_114 levels.
// Same-PLL 4:1, so these are timed paths and not CDCs -- the same argument
// ss_dma_busy and cdtv_sec_empty_w already rely on. Levels, not pulses, is what
// makes sampling at the slower rate sound: each one is held until the next
// request starts, which is millions of cycles.
localparam [7:0] SS_INFO_BASE = 8'd13;   // "Save state saved" -- see CONF_STR

reg  [7:0] ss_info_code;
reg        ss_info_req;
// ss_ctrl runs on clk_114; this block runs on clk_sys at 28.6 MHz, so its four
// outcome signals are a clock-domain crossing. They used to be sampled straight
// into the edge-detect flops with no synchroniser at all.
//
// Two things had to be true for a toast to appear and neither was. The outcome
// had to be a LEVEL -- a single 8.8 ns clk_114 pulse is invisible to a 35 ns
// sampler, and ss_ctrl's save_ok was exactly that until it was changed to hold
// until the next save. And the crossing had to be synchronised. Measured on
// hardware: the core's toast never fired for either a save or a restore, so a
// restore that validated and refused looked identical to one that never ran.
//
// The fail code comes from its own synchronised copy. It is set in the same
// clk_114 cycle as load_fail and held just as long, so by the time the
// synchronised flag edge arrives it has been stable for two clk_sys cycles.
reg  [3:0] ss_out_meta, ss_out_sync, ss_out_d;
reg  [3:0] ss_code_meta, ss_code_sync;
// Sticky advisory bit from ss_ctrl (clk_114), two-flopped into clk_sys like
// the outcomes beside it. It only ever goes 0 -> 1, so there is nothing to
// miss between samples.
reg        ss_kick_warn_meta, ss_kick_warn_sync;

always @(posedge clk_sys) begin
	ss_info_req  <= 1'b0;

	ss_out_meta  <= {ss_load_fail, ss_load_ok, ss_save_fail, ss_save_ok};
	ss_out_sync  <= ss_out_meta;
	ss_out_d     <= ss_out_sync;

	ss_code_meta <= ss_load_fail_code;
	ss_code_sync <= ss_code_meta;

	ss_kick_warn_meta <= ss_dbg_kick_warn;
	ss_kick_warn_sync <= ss_kick_warn_meta;

	// At most one of the four can rise on a given cycle: ss_ctrl serves one
	// request at a time and clears the previous attempt's outcomes when it
	// takes the next, so the priority below never actually arbitrates.
	if (ss_out_sync[0] & ~ss_out_d[0]) begin
		ss_info_code <= SS_INFO_BASE;
		ss_info_req  <= 1'b1;
	end
	else if (ss_out_sync[1] & ~ss_out_d[1]) begin
		ss_info_code <= SS_INFO_BASE + 8'd1;
		ss_info_req  <= 1'b1;
	end
	else if (ss_out_sync[2] & ~ss_out_d[2]) begin
		ss_info_code <= SS_INFO_BASE + 8'd2;
		ss_info_req  <= 1'b1;
	end
	else if (ss_out_sync[3] & ~ss_out_d[3]) begin
		// Codes 1..6 map straight onto the six strings after "restored"; a code
		// this build does not know about still says something rather than
		// indexing off the end of the list into silence.
		ss_info_code <= (ss_code_sync >= 4'd1 && ss_code_sync <= 4'd6)
		                ? (SS_INFO_BASE + 8'd2 + {4'd0, ss_code_sync})
		                : (SS_INFO_BASE + 8'd9);
		ss_info_req  <= 1'b1;
	end
end

// --- diagnostic readback -----------------------------------------------------
//
// Everything above this line is invisible from userspace. A save that never
// started, a restore refused at its first gate, a restore that ran the whole
// sequence and then crashed the Amiga, and a toast that was raised and never
// displayed all present identically: nothing on screen and nothing in any log.
// This block publishes enough of ss_ctrl's internals -- and of this file's own
// toast request -- to tell those apart, on hps_ext's 0xF600 UIO read
// sub-channel. support/minimig/minimig_ssdiag.cpp polls it and logs changes to
// /tmp/ss_dbg.log.
//
// WHY THE AGGREGATION IS IN RTL. ss_ctrl runs at 113.5 MHz and most of the
// states it passes through last a handful of clocks. A poller sampling
// dbg_state alone would see S_IDLE essentially always, and would report
// "nothing happened" for a restore that ran to the last gate and was refused
// there. So the last non-idle state and a state-change counter are latched
// here, at the rate the events actually happen. The counter is what separates a
// stalled controller (state stuck, counter stuck) from a busy one (state stuck,
// counter climbing) from one that was never asked (both at their reset values).
//
// It is deliberately independent of info_req. That path is one of the things
// being diagnosed, and a diagnostic carried on the channel it is meant to
// diagnose cannot tell its own silence from the fault.
//
// Cheap enough to leave in: two counters, two latches, one 64-bit
// synchroniser, no logic in any path ss_ctrl depends on, and a read that
// hps_ext answers from a byte_cnt mux with no side effects at all.

// --- clk_114 side ---
reg  [5:0]  ss_dbg_last;      // last state that was not S_IDLE
reg  [5:0]  ss_dbg_state_d;
reg  [15:0] ss_dbg_seq;       // one increment per ss_ctrl state change
reg  [7:0]  ss_dbg_ffall;     // falling edges of freeze
reg         ss_dbg_freeze_d;

always @(posedge clk_114) begin
	if (reset_d) begin
		ss_dbg_last     <= 6'd0;
		ss_dbg_state_d  <= 6'd0;
		ss_dbg_seq      <= 16'd0;
		ss_dbg_ffall    <= 8'd0;
		ss_dbg_freeze_d <= 1'b0;
	end
	else begin
		ss_dbg_state_d  <= ss_dbg_state;
		ss_dbg_freeze_d <= ss_freeze;

		if (ss_dbg_state != ss_dbg_state_d) ss_dbg_seq <= ss_dbg_seq + 16'd1;

		// S_IDLE is 0, and ss_ctrl returns to it whether it finished, refused
		// or failed -- so the live state says nothing at all once an attempt is
		// over. This is the register that makes a refusal legible after the
		// fact.
		if (ss_dbg_state != 6'd0) ss_dbg_last <= ss_dbg_state;

		// The restore's own invariant, counted rather than assumed: the freeze
		// must fall exactly once per restore (see ss_ctrl.v's S_L_RELEASE). More
		// than once means the Amiga ran for a few instructions against a machine
		// that was half old and half new, which is a live candidate explanation
		// for a restore that completes and then crashes.
		if (!ss_freeze && ss_dbg_freeze_d) ss_dbg_ffall <= ss_dbg_ffall + 8'd1;
	end
end

// --- clk_sys side ---
//
// The toast path, watched from inside hps_io's own clock domain. ss_info_cnt
// counts every request this file has raised since reset and ss_info_last holds
// the code the most recent one carried: that is the "did the core ever ask?"
// half of the toast question. The other half -- did the host see it -- is
// answered by minimig_ssdiag.cpp logging what UIO_INFO_GET returned, into the
// same file on the same timeline.
//
// ss_info_code is assigned on the same clk_sys edge that raises ss_info_req, so
// by the time this block sees the request high, the code beside it is the one
// that request carries.
reg [7:0] ss_info_cnt   = 8'd0;
reg [7:0] ss_info_last  = 8'd0;

// ss_state_fanout completions. A restore whose register writeback never
// finished leaves the CPU running with someone else's PC, which the state
// vector's own progress cannot show. ack is a LEVEL held until req drops (see
// ss_state_fanout below), so this counts its rising edge, not its cycles.
reg [7:0] ss_fanout_cnt = 8'd0;
reg       ss_fanout_ack_d = 1'b0;

always @(posedge clk_sys) begin
	ss_fanout_ack_d <= ss_fanout_ack;
	if (ss_info_req) begin
		ss_info_cnt  <= ss_info_cnt + 8'd1;
		ss_info_last <= ss_info_code;
	end
	if (ss_fanout_ack && !ss_fanout_ack_d) ss_fanout_cnt <= ss_fanout_cnt + 8'd1;
end

// --- crossing ---
//
// clk_114 to clk_sys, the same 4:1 same-PLL relationship the outcome sampler
// above documents. Double-registered anyway, and as ONE vector rather than
// field by field, so a sample taken across a change is at worst one stale
// snapshot of a consistent set rather than a mixture of two moments. These are
// diagnostics: a stale sample costs one log line, and the poller only logs on
// change, so a torn value could not be mistaken for a sequence.
wire [63:0] ss_dbg114 = { ss_rom_scan, ss_freeze, ss_load_busy, ss_save_busy,
                          ss_dbg_ffall, ss_dbg_idx, ss_dbg_seq,
                          ss_dbg_last, ss_dbg_state };

reg [63:0] ss_dbg_meta = 64'd0;
reg [63:0] ss_dbg_sync = 64'd0;
always @(posedge clk_sys) begin
	ss_dbg_meta <= ss_dbg114;
	ss_dbg_sync <= ss_dbg_meta;
end

wire [5:0]  ss_dg_state = ss_dbg_sync[5:0];
wire [5:0]  ss_dg_last  = ss_dbg_sync[11:6];
wire [15:0] ss_dg_seq   = ss_dbg_sync[27:12];
wire [23:0] ss_dg_idx   = ss_dbg_sync[51:28];
wire [7:0]  ss_dg_ffall = ss_dbg_sync[59:52];
wire [3:0]  ss_dg_flags = ss_dbg_sync[63:60];  // {rom_scan, freeze, load_busy, save_busy}

// The window, low word first. hps_ext presents ss_diag[15:0] as the first word
// after the signature, so the last term of this concatenation is word 0.
// minimig_ssdiag.cpp decodes exactly this layout; SS_DIAG_VERSION is bumped if
// it ever changes, so a poller and a core that disagree say so instead of
// printing a confident wrong answer.
localparam [15:0] SS_DIAG_VERSION = 16'h0001;

assign ss_diag = {
	SS_DIAG_VERSION,                                    // w7: layout version
	ss_info_cnt, ss_info_last,                          // w6: toast requests / last code
	ss_dg_ffall, ss_fanout_cnt,                         // w5: freeze falls / fanout completions
	{8'd0, ss_dg_idx[23:16]},                           // w4: progress index, high
	ss_dg_idx[15:0],                                    // w3: progress index, low
	ss_dg_seq,                                          // w2: state-change counter
	{3'd0, ss_kick_warn_sync, ss_dg_flags,
	       ss_code_sync, ss_out_sync},                  // w1: flags / fail code / outcomes
	{2'd0, ss_dg_last, 2'd0, ss_dg_state}               // w0: last state / live state
};

// --- restore fan-out ---------------------------------------------------------
//
// ss_ctrl runs on clk_114; the CPU and minimig.v run on clk_sys, a quarter of
// it. state_we is a single clk_114 pulse, which a clk_sys edge would miss
// three times out of four, so it is turned into a level here and handed over
// with a request/ack pair. The vector itself needs no latch: ss_serdes holds
// state_out in a register until the next load.
reg ss_fanout_req;
always @(posedge clk_114) begin
	if (reset_d)             ss_fanout_req <= 1'b0;
	else if (ss_fanout_ack)  ss_fanout_req <= 1'b0;
	else if (ss_state_we)    ss_fanout_req <= 1'b1;
end

// ORDERING NOTE. ss_ctrl does not wait for this fan-out; it goes straight from
// the state vector to the chip RAM writeback. That is safe by a wide margin
// rather than by construction: the fan-out is 23 clk_sys cycles (~0.8 us) and
// the chip RAM pass that follows it is a million SDRAM word writes (~ms), so
// the registers are always in place long before the freeze is released. The
// property that actually matters -- the CPU must not execute against a
// half-written register file -- is enforced independently, by ss_fanout_busy
// holding cpu_wrapper's ss_arm past ss_ctrl's release.
ss_state_fanout #(.STATE_W(`SS_STATE_W)) ss_fanout
(
	.clk          (clk_sys),
	.rst_n        (~reset_d),
	.req          (ss_fanout_req),
	.ack          (ss_fanout_ack),
	.state        (ss_state_out),

	.cpu_wr_index (ss_fanout_wr_index),
	.cpu_wr_data  (ss_fanout_wr_data),
	.cpu_wr_en    (ss_fanout_wr_en),
	.cpu_pc_wr    (ss_fanout_pc_wr),
	.cpu_sr_wr    (ss_fanout_sr_wr),
	.cpu_usp_wr   (ss_fanout_usp_wr),
	.cpu_vbr_wr   (ss_fanout_vbr_wr),
	.cpu_cacr_wr  (ss_fanout_cacr_wr),
	.cpu_resume   (ss_fanout_resume),

	.map_in       (ss_fanout_map_in),
	.map_we       (ss_fanout_map_we),
	.intreq_out   (ss_restored_intreq),
	.cia_a_out    (ss_restored_cia_a),
	.cia_b_out    (ss_restored_cia_b),
	.cia_we       (ss_restored_cia_we),

	.busy         (ss_fanout_busy)
);
/////////////////////////////////////////////////////////////////////////////


// power led control
wire pwr_led;
reg [5:0] led_cnt;
reg led_dim;

always @ (posedge clk_sys) begin
  led_cnt <= led_cnt + 1'd1;
  led_dim <= |led_cnt[5:2];
end

assign LED_POWER[0] = pwr_led | ~led_dim;

assign FB_FORCE_BLANK = 0;

reg ce_out = 0;
always @(posedge CLK_VIDEO) begin
	reg [3:0] div;
	reg [3:0] add;
	reg [1:0] fs_res;
	reg old_vs;
	
	div <= div + add;
	if(~hblank & ~vblank) fs_res <= fs_res | res;

	old_vs <= vs;
	if(old_vs & ~vs) begin
		fs_res <= 0;
		div <= 0;
		add <= 1; // 7MHz
		if(fs_res[0]) add <= 2; // 14MHz
		if(fs_res[1] | (~status[42] & ~scandoubler)) add <= 4; // 28MHz
	end

	ce_out <= div[3] & !div[2:0];
end

assign ce_pix = ce_out;

wire [2:0] fx;
wire       scandoubler = (fx || forced_scandoubler) & ~lace;
wire [7:0] R,G,B;

video_mixer #(.LINE_LENGTH(2000), .HALF_DEPTH(0), .GAMMA(1)) video_mixer
(
	.*,
	.hq2x(fx==1),
	.ce_pix(ce_out),
	.freeze_sync(),

	.R(r),
	.G(g),
	.B(b),
	.HSync(~hs),
	.VSync(~vs),
	.HBlank(~hde),
	.VBlank(~vde),

	.VGA_R(R),
	.VGA_G(G),
	.VGA_B(B)
);

assign CLK_VIDEO = clk_114;
assign VGA_F1    = field1;
assign VGA_R     = mt32_lcd ? {{2{mt32_lcd_pix}},R[7:2]} : R;
assign VGA_G     = mt32_lcd ? {{2{mt32_lcd_pix}},G[7:2]} : G;
assign VGA_B     = mt32_lcd ? {{2{mt32_lcd_pix}},B[7:2]} : B;

wire [12:0] arx,ary;
video_freak video_freak
(
	.*,
	.VGA_DE_IN(VGA_DE),
	.VGA_DE(),
	.ARX((!ar) ? 12'd4 : (ar - 1'd1)),
	.ARY((!ar) ? 12'd3 : 12'd0),
	.VIDEO_ARX(arx),
	.VIDEO_ARY(ary),
	.CROP_SIZE(0),
	.CROP_OFF(0),
	.SCALE(status[45:43])
);

reg [11:0] fb_arx, fb_ary;
always @(posedge CLK_VIDEO) begin
	reg [11:0] x, y, x1, y1;
	reg [1:0] cnt;
	
	cnt <= cnt + 1'd1;
	case(cnt)
		0: begin
				x1 <= FB_WIDTH;
				y1 <= FB_HEIGHT;
				x  <= FB_WIDTH;
				y  <= FB_HEIGHT;
			end

		1: if(x && ((x+x1) <= HDMI_WIDTH) && y && ((y+y1) <= HDMI_HEIGHT)) begin
				x <= x+x1;
				y <= y+y1;
				cnt <= 1;
			end

		2: begin
				fb_arx <= x;
				fb_ary <= y;
			end
	endcase
end

assign VIDEO_ARX = FB_EN ? {status[46], fb_arx} : arx;
assign VIDEO_ARY = FB_EN ? {status[46], fb_ary} : ary;

wire [2:0] sl = fx ? fx - 1'd1 : 3'd0;
assign VGA_SL = sl[1:0];

reg  hde;
wire vde = ~(fvbl | svbl);

wire [7:0] red, green, blue, r,g,b;
wire lace, field1;
wire hblank, vbl;
wire vblank = vbl | ~vs;
reg  fhbl, fvbl, shbl, svbl;
wire hbl = fhbl | shbl | ~hs;

wire  [1:0] res;

wire sset;
wire [11:0] shbl_l, shbl_r;
wire [11:0] svbl_t, svbl_b;

reg  [11:0] hbl_l=0, hbl_r=0;
reg  [11:0] hsta, hend, hmax, hcnt;
reg  [11:0] hsize;
always @(posedge clk_sys) begin
	reg old_hs;
	reg old_hblank;

	old_hs <= hs;
	old_hblank <= hblank;

	hcnt <= hcnt + 1'd1;
	if(~hs) hcnt <= 0;

	if(old_hblank & ~hblank) hend <= hcnt;
	if(~old_hblank & hblank) hsta <= hcnt;
	if(old_hs & ~hs)         hmax <= hcnt;

	if(hcnt == hend+hbl_l-2'd2) shbl <= 0;
	if(hcnt == hsta+hbl_r-2'd2) shbl <= 1;

	//force hblank
	if(hcnt == 8)         fhbl <= 0;
	if(hcnt == hmax-4'd8) fhbl <= 1;
	
	if(~old_hblank & hblank & ~field1 & (vcnt == 1'd1)) hsize <= hcnt - hend;
end

reg [11:0] vbl_t=0, vbl_b=0;
reg [11:0] vend, vmax, f1_vend, f1_vsize, vcnt, vs_end;
reg [11:0] vsize;
always @(posedge clk_sys) begin
	reg old_vs;
	reg old_vblank, old_hs, old_hbl;

	old_vs <= vs;
	old_hs <= hs;
	old_vblank <= vblank;
	
	if(old_hs & ~hs) vcnt <= vcnt + 1'd1;
	if(~old_vblank & vblank) vcnt <= 0;

	if(~lace | ~field1) begin
		if(old_vblank & ~vblank) vend <= vcnt;
		if(~old_vs & vs)         vs_end <= vcnt;
		
		if(~old_vblank & vblank) begin
			vmax <= vcnt;
			vsize <= vcnt - vend + f1_vsize;
			f1_vsize <= 0;
		end
	end
	else begin
		if(old_vblank & ~vblank) f1_vend <= vcnt;
		if(~old_vblank & vblank) begin
			f1_vsize <= vcnt - f1_vend;
		end
	end

	old_hbl <= hbl;
	if((old_hbl & ~hbl) | !vcnt) begin
		if(vcnt == vend+vbl_t) svbl <= 0;
		if(vcnt == (vbl_b[11] ? vmax+vbl_b : vbl_b) ) svbl <= 1;

		//force vblank
		if(vcnt == vmax-1)    fvbl <= 1;
		if(vcnt == vs_end+2)  fvbl <= 0;
	end
	
	hde <= ~hbl;
end

always @(posedge clk_sys) begin
	reg old_level;
	reg alt = 0;

	old_level <= kbd_mouse_level;
	if((old_level ^ kbd_mouse_level) && (kbd_mouse_type==3)) begin
		if(kbd_mouse_data == 'h41) begin //backspace
			vbl_t <= 0; vbl_b <= 0;
			hbl_l <= 0; hbl_r <= 0;
		end
		else if(kbd_mouse_data == 'h4c) begin //up
			if(alt) vbl_b <= vbl_b + 1'd1;
			else    vbl_t <= vbl_t + 1'd1;
		end
		else if(kbd_mouse_data == 'h4d) begin //down
			if(alt) vbl_b <= vbl_b - 1'd1;
			else    vbl_t <= vbl_t - 1'd1;
		end
		else if(kbd_mouse_data == 'h4f) begin //left
			if(alt) hbl_r <= hbl_r + 3'd4;
			else    hbl_l <= hbl_l + 3'd4;
		end
		else if(kbd_mouse_data == 'h4e) begin //right
			if(alt) hbl_r <= hbl_r - 3'd4;
			else    hbl_l <= hbl_l - 3'd4;
		end
		else if(kbd_mouse_data == 'h64 || kbd_mouse_data == 'h65) begin //alt press
			alt <= 1;
		end
		else if(kbd_mouse_data == 'hE4 || kbd_mouse_data == 'hE5) begin //alt release
			alt <= 0;
		end
	end
	
	if(sset) begin
		vbl_t <= svbl_t; vbl_b <= svbl_b;
		hbl_l <= shbl_l; hbl_r <= shbl_r;
	end
end


reg [11:0] scr_hbl_l, scr_hbl_r;
reg [11:0] scr_vbl_t, scr_vbl_b;
reg [11:0] scr_hsize, scr_vsize;
reg  [1:0] scr_res;
reg  [6:0] scr_flg;

always @(posedge clk_sys) begin
	reg old_vblank;

	old_vblank <= vblank;
	if(old_vblank & ~vblank) begin
		scr_hbl_l <= hbl_l;
		scr_hbl_r <= hbl_r;
		scr_vbl_t <= vbl_t;
		scr_vbl_b <= vbl_b;
		scr_hsize <= hsize;
		scr_vsize <= vsize;
		scr_res   <= res;

		if(scr_res != res || scr_vsize != vsize || scr_hsize != hsize) scr_flg <= scr_flg + 1'd1;
	end
end

////////////////////////////  MT32pi  ////////////////////////////////// 

// Reset MT32-pi when the user port changes hands, so it does not keep
// driving state onto a bus it no longer owns.
reg         userport_change_reset;
wire        mt32_reset    = status[32] | reset | userport_change_reset;
wire        mt32_disable  = status[33];
wire        mt32_mode_req = status[34];
wire  [1:0] mt32_rom_req  = status[36:35];
wire  [7:0] mt32_sf_req   = status[39:37];
wire  [1:0] mt32_info     = status[41:40];
wire        midi_tx       = uart_tx;

wire [15:0] mt32_i2s_r, mt32_i2s_l;
wire  [7:0] mt32_mode, mt32_rom, mt32_sf;
wire        mt32_lcd_en, mt32_lcd_pix, mt32_lcd_update;
wire        midi_rx;

wire mt32_newmode;
wire mt32_available;
wire mt32_use  = mt32_available & ~mt32_disable;
wire mt32_mute = mt32_available &  mt32_disable;

mt32pi mt32pi
(
	.*,
	.USER_OUT(IndirectUserOutmt32),
	.CE_PIXEL(ce_pix_mt32),
	.reset(mt32_reset),
	.midi_tx(midi_tx | mt32_mute)
);

always @(posedge clk_sys) begin
	reg [1:0] last_userport_mode;
	userport_change_reset <= 0;
	last_userport_mode <= user_port_mode;
	if (last_userport_mode != user_port_mode) userport_change_reset <= 1;
end

wire  [4:0] mt32_cfg = (mt32_mode == 'hA2) ? {mt32_sf[2:0],  2'b10} :
                       (mt32_mode == 'hA1) ? {mt32_rom[1:0], 2'b01} : 5'd0;

reg mt32_info_req;
reg [3:0] mt32_info_disp;
always @(posedge clk_sys) begin
	reg old_mode;

	old_mode <= mt32_newmode;
	mt32_info_req <= (old_mode ^ mt32_newmode) && (mt32_info == 1);
	
	mt32_info_disp <= (mt32_mode == 'hA2) ? (4'd1 + mt32_sf[2:0]) :
                     (mt32_mode == 'hA1 && mt32_rom == 0) ?  4'd9 :
                     (mt32_mode == 'hA1 && mt32_rom == 1) ?  4'd10 :
                     (mt32_mode == 'hA1 && mt32_rom == 2) ?  4'd11 : 4'd12;
end

reg mt32_lcd_on;
always @(posedge CLK_VIDEO) begin
	int to;
	reg old_update;

	old_update <= mt32_lcd_update;
	if(to) to <= to - 1;

	if(mt32_info == 2) mt32_lcd_on <= 1;
	else if(mt32_info != 3) mt32_lcd_on <= 0;
	else begin
		if(!to) mt32_lcd_on <= 0;
		if(old_update ^ mt32_lcd_update) begin
			mt32_lcd_on <= 1;
			to <= 114000000 * 2;
		end
	end
end

wire mt32_lcd = mt32_lcd_on & mt32_lcd_en;

reg ce_pix_mt32;
always @(posedge CLK_VIDEO) begin
	reg [3:0] div;
	
	div <= div + 1'd1;
	ce_pix_mt32 <= !div;
end

/* ------------------------------------------------------------------------------ */

wire flt_en    = ~status[48] ? pwr_led : status[47];
wire aud_1200  = status[49];
wire paula_pwm = status[50];

wire [15:0] paula_smp_l = (paula_pwm ? {ldata_okk[8:0], 7'b0} : {ldata[14:0], 1'b0});
wire [15:0] paula_smp_r = (paula_pwm ? {rdata_okk[8:0], 7'b0} : {rdata[14:0], 1'b0});

// LPF 4400Hz, 1st order, 6db/oct
wire [15:0] lpf4400_l, lpf4400_r;
IIR_filter #(0) lpf4400
(
	.clk(clk_sys),
	.reset(reset),

	.ce(clk7_en | clk7n_en),
	.sample_ce(1),

	.cx (40'd4304835800),
	.cx0(1),
	.cy0(-2088941),
	
	.input_l(paula_smp_l),
	.input_r(paula_smp_r),
	.output_l(lpf4400_l),
	.output_r(lpf4400_r)
);

wire [15:0] audm_l = aud_1200 ? paula_smp_l : lpf4400_l;
wire [15:0] audm_r = aud_1200 ? paula_smp_r : lpf4400_r;

// LPF 3000Hz 1st + 3400Hz 1st
wire [15:0] lpf3275_l, lpf3275_r;
IIR_filter #(0) lpf3275
(
	.clk(clk_sys),
	.reset(reset),

	.ce(clk7_en | clk7n_en),
	.sample_ce(1),

	.cx (40'd8536629),
	.cx0(2),
	.cx1(1),
	.cy0(-4182432),
	.cy1(2085297),

	.input_l(audm_l),
	.input_r(audm_r),
	.output_l(lpf3275_l),
	.output_r(lpf3275_r)
);

reg [15:0] aud_l, aud_r;
always @(posedge CLK_AUDIO) begin
	reg [15:0] old_l0, old_l1, old_r0, old_r1;

	old_l0 <= flt_en ? lpf3275_l : audm_l;
	old_l1 <= old_l0;
	if(old_l0 == old_l1) aud_l <= old_l1;

	old_r0 <= flt_en ? lpf3275_r : audm_r;
	old_r1 <= old_r0;
	if(old_r0 == old_r1) aud_r <= old_r1;
end

wire  [15:0] cdda_l;
wire  [15:0] cdda_r;
wire  [15:0] cdda_dout;
wire         cdda_req;
wire         cdda_wr;

cdda #(28375160) cdda
(
	.CLK(clk_sys),
	.nRESET(~reset),
	.WRITE_REQ(cdda_req),
	.WRITE(cdda_wr),
	.DIN(cdda_dout),
	.AUDIO_L(cdda_l),
	.AUDIO_R(cdda_r)
);

reg [15:0] out_l, out_r;
always @(posedge CLK_AUDIO) begin
	reg [16:0] tmp_l, tmp_r;

	tmp_l <= {aud_l[15],aud_l} + {toccata_aud_left[15],toccata_aud_left} + (mt32_mute ? 17'd0 : {mt32_i2s_l[15],mt32_i2s_l}) + {cdda_l[15], cdda_l};
	tmp_r <= {aud_r[15],aud_r} + {toccata_aud_right[15],toccata_aud_right} + (mt32_mute ? 17'd0 : {mt32_i2s_r[15],mt32_i2s_r}) + {cdda_r[15], cdda_r};

	// clamp the output
	out_l <= (^tmp_l[16:15]) ? {tmp_l[16], {15{tmp_l[15]}}} : tmp_l[15:0];
	out_r <= (^tmp_r[16:15]) ? {tmp_r[16], {15{tmp_r[15]}}} : tmp_r[15:0];
end

assign AUDIO_S = 1;
assign AUDIO_L = out_l;
assign AUDIO_R = out_r;

endmodule


// The restore fan-out -- ss_state_fanout, which walks the state vector back
// into the CPU register file and Gary's map -- is instantiated above and
// defined in rtl/ss_state_fanout.v. It was written here and lived here, which
// meant nothing could simulate it without the whole top level.

