// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: NEW FILE.
//
// VHDL-friendly wrapper around rtl/minimig.v for the MEGA65 port.
//
// Why this file exists:
// minimig.v uses port names with a leading underscore (_cpu_as, _hsync,
// _joy1, ...) which are not legal VHDL identifiers, so the M2M framework's
// CORE/vhdl/main.vhd cannot instantiate minimig directly. This wrapper
// 1. renames all underscore-prefixed ports to the M2M convention
//    (active-low signals get a _n suffix instead of the _ prefix),
// 2. ties off every subsystem that the Amiga 500 configuration never uses
//    (Toccata, IDE/Gayle externals, RS232 modem lines, RTC, joystick ports
//    3/4, analog joysticks, AGA chip48 bus), so that CORE/vhdl/main.vhd
//    stays free of clutter and the unused logic constant-folds in synthesis.
//
// The wrapper adds NO logic - it is pure renaming and constant tie-offs.
// See .research/PORTING-PLAN.md and the port table in
// .research/phase-a/sweep-minimig.md for the underlying contract.

module minimig_m65
(
	// see port comments below; declaration order matches minimig.v groups

	// m68k CPU interface (cpu_wrapper.v)
	input  [23:1] cpu_address,    // m68k address bus
	output [15:0] cpu_data,       // m68k data bus (read data to CPU)
	input  [15:0] cpudata_in,     // m68k data in (write data from CPU)
	output  [2:0] cpu_ipl_n,      // m68k interrupt request, active low
	input         cpu_as_n,       // m68k address strobe, active low
	input         cpu_uds_n,      // m68k upper data strobe, active low
	input         cpu_lds_n,      // m68k lower data strobe, active low
	input         cpu_r_w,        // m68k read / write (1=read)
	output        cpu_dtack_n,    // m68k data acknowledge, active low
	output        cpu_reset_n,    // m68k reset (to CPU), active low
	input         cpu_reset_in_n, // m68k reset feedback (RESET instruction), active low
	input  [31:0] nmi_addr,       // m68k NMI vector address (from cpu_wrapper)

	// SRAM-style memory interface (served by BRAM in mega65.vhd)
	output [15:0] ram_data,       // write data
	input  [15:0] ramdata_in,     // read data
	output [22:1] ram_address,    // BANKED word address (see minimig_sram_bridge.v;
	                              // minimig's bit 23 is constant 0 after the AExp
	                              // sweep change and is dropped here)
	output        ram_bhe_n,      // upper byte enable (bits 15:8), active low
	output        ram_ble_n,      // lower byte enable (bits 7:0), active low
	output        ram_we_n,       // write enable, active low
	output        ram_oe_n,       // output/read enable, active low

	// system
	input         rst_ext,        // external reset request, active high
	output        rst_out,        // minimig reset status
	input         clk,            // 28.375 MHz master clock
	input         clk7_en,        // 7.09 MHz posedge clock enable (amiga_clk.v)
	input         clk7n_en,       // 7.09 MHz negedge clock enable
	input         c1,             // quadrature phase 1
	input         c3,             // quadrature phase 3
	input         cck,            // colour clock enable (3.55 MHz)
	input   [9:0] eclk,           // E-clock one-hot ring (709 kHz)

	// input devices
	input  [15:0] joy1_n,         // mouse port,    active low {...,fire2,fire,up,down,left,right}
	input  [15:0] joy2_n,         // joystick port, active low
	input   [2:0] mouse_btn,      // mouse buttons {M,R,L}, active high
	input         kms_level,      // keyboard/mouse event toggle strobe
	input   [1:0] kbd_mouse_type, // 2 = raw Amiga keyboard scancode
	input   [7:0] kbd_mouse_data, // scancode (bit 7 = release)

	// LEDs
	output        pwr_led,
	output        fdd_led,
	output        hdd_led,

	// MEGA65 battery-backed RTC (issue #13): MiSTer-format 65-bit conduit,
	// [63:0] = MSM6242B BCD nibbles, [64] = "new value" toggle. Driven by the
	// M2M framework from the board RTC; decoded by minimig.v at $DC0000.
	input  [64:0] rtc,

	// host controller interface, shared IO_STROBE/IO_DIN bus with two frame
	// enables: io_uio selects userio.v (config FSM amiga_config.vhd), io_fpga
	// selects paula_floppy.v (ADF track engine adf_track_engine.vhd)
	input         io_uio,         // command channel frame (userio.v IO_ENA)
	input         io_fpga,        // floppy channel frame (paula_floppy.v IO_ENA)
	input         io_strobe,      // word strobe, 1 clk wide
	output        io_wait,
	input  [15:0] io_din,
	output [15:0] io_dout,        // response data (paula_floppy only; userio has none)

	// video (28.375 MHz domain)
	output        hsync_n,        // active low
	output        vsync_n,        // active low
	output        hblank,         // active high (Agnus hbl with blver=0)
	output        vblank,         // active high
	output  [7:0] red,
	output  [7:0] green,
	output  [7:0] blue,
	output        ce_pix,         // minimig's own pixel CE (7.09/14.19 MHz; info)
	output  [1:0] res,            // {shres, hires} resolution flags for the frame-locked CE
	output        lace,           // interlace mode flag
	output        field1,         // field flag (interlace)

	// audio (Paula, 15-bit signed)
	output [14:0] ldata,
	output [14:0] rdata
);

// minimig's ram_address[23] is driven constant 0 (AExp sweep change); consume it here
wire ram_address23_unused;

minimig minimig_inst
(
	//m68k pins
	.cpu_address   (cpu_address  ),
	.cpu_data      (cpu_data     ),
	.cpudata_in    (cpudata_in   ),
	._cpu_ipl      (cpu_ipl_n    ),
	._cpu_as       (cpu_as_n     ),
	._cpu_uds      (cpu_uds_n    ),
	._cpu_lds      (cpu_lds_n    ),
	.cpu_r_w       (cpu_r_w      ),
	._cpu_dtack    (cpu_dtack_n  ),
	._cpu_reset    (cpu_reset_n  ),
	._cpu_reset_in (cpu_reset_in_n),
	.nmi_addr      (nmi_addr     ),
	.ovr           (             ), // unconnected, as in MiSTer's Minimig.sv

	//sram pins
	.ram_data      (ram_data     ),
	.ramdata_in    (ramdata_in   ),
	.ram_address   ({ram_address23_unused, ram_address}),
	._ram_bhe      (ram_bhe_n    ),
	._ram_ble      (ram_ble_n    ),
	._ram_we       (ram_we_n     ),
	._ram_oe       (ram_oe_n     ),
	.chip48        (48'h0        ), // AGA 64-bit fetch: OCS keeps fmode=0, logic prunes

	//system pins
	.rst_ext       (rst_ext      ),
	.rst_out       (rst_out      ),
	.clk           (clk          ),
	.clk7_en       (clk7_en      ),
	.clk7n_en      (clk7n_en     ),
	.c1            (c1           ),
	.c3            (c3           ),
	.cck           (cck          ),
	.eclk          (eclk         ),

	//rs232 pins (no serial port wired in milestone 1; inactive levels as MiSTer)
	.rxd           (1'b1         ),
	.txd           (             ),
	.cts           (1'b1         ),
	.rts           (             ),
	.dtr           (             ),
	.dsr           (1'b1         ),
	.cd            (1'b1         ),
	.ri            (1'b1         ),

	//I/O
	._joy1         (joy1_n       ),
	._joy2         (joy2_n       ),
	._joy3         (16'hFFFF     ), // not connected (active low, idle)
	._joy4         (16'hFFFF     ),
	.joya1         (16'h0000     ), // analog joysticks: unused (cmd 0xF9 = 0)
	.joya2         (16'h0000     ),
	.mouse_btn     (mouse_btn    ),
	.kms_level     (kms_level    ),
	.kbd_mouse_type(kbd_mouse_type),
	.kbd_mouse_data(kbd_mouse_data),
	.pwr_led       (pwr_led      ),
	.fdd_led       (fdd_led      ),
	.hdd_led       (hdd_led      ),
	// MiSTer2MEGA65 (AExp Amiga 500 port), July 2026: MEGA65 battery RTC wired
	// through to Minimig's MSM6242B clock at $DC0000 (issue #13).
	.rtc           (rtc          ),

	//host controller interface
	// MiSTer2MEGA65 (AExp Amiga 500 port), July 2026: IO_FPGA was tied 1'b0
	// and IO_DOUT left open in milestone 1 (no floppy). Both are now real
	// ports for the ADF floppy milestone (track engine on the paula_floppy
	// host channel).
	.IO_UIO        (io_uio       ),
	.IO_FPGA       (io_fpga      ),
	.IO_STROBE     (io_strobe    ),
	.IO_WAIT       (io_wait      ),
	.IO_DIN        (io_din       ),
	.IO_DOUT       (io_dout      ),

	//video
	._hsync        (hsync_n      ),
	._vsync        (vsync_n      ),
	._csync        (             ), // M2M generates its own csync after the OSM
	.field1        (field1       ),
	.lace          (lace         ),
	.hblank        (hblank       ),
	.vblank        (vblank       ),
	.red           (red          ),
	.green         (green        ),
	.blue          (blue         ),
	.ar            (             ),
	.scanline      (             ),
	.ce_pix        (ce_pix       ),
	.res           (res          ),
	.ntsc          (             ), // PAL only

	//audio
	.ldata         (ldata        ),
	.rdata         (rdata        ),
	.ldata_okk     (             ), // PWM-volume variant: unused
	.rdata_okk     (             ),
	.aud_mix       (             ),

	// Toccata audio: disabled in the AExp port (see minimig.v surgery)
	.toccata_ena   (1'b0         ),
	.toccata_base  (8'h00        ),
	.toccata_aud_left  (         ),
	.toccata_aud_right (         ),

	//user i/o: configs come from the amiga_config FSM via userio; the
	//cpu_wrapper inputs are tied constant in main.vhd (68000, no caches),
	//so these outputs are informational only
	.cpucfg        (             ),
	.cachecfg      (             ),
	.memcfg        (             ),
	.bootrom       (             ), // stays 0: we never host-write $F80000
	.ide_ena       (             ),

	// IDE/Gayle: disabled (cmd 0xF8 = 0)
	.ide_fast      (             ),
	.ide_ext_irq   (1'b0         ),
	.ide_req       (             ),
	.ide_address   (5'b0         ),
	.ide_write     (1'b0         ),
	.ide_writedata (16'h0000     ),
	.ide_read      (1'b0         ),
	.ide_readdata  (             )
);

endmodule
