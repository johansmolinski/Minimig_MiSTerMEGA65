//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
//                                                                          //
// Copyright (c) 2009-2011 Tobias Gubener                                   //
// Copyright (c) 2017-2019 Alexey Melnikov                                  //
// Subdesign fAMpIGA by TobiFlex                                            //
//                                                                          //
// This is the cpu wrapper to generate 68K Bus signals                      //
// and configure Zorro cards                                                //
//                                                                          //
// This source file is free software: you can redistribute it and/or modify //
// it under the terms of the GNU General Public License as published        //
// by the Free Software Foundation, either version 3 of the License, or     //
// (at your option) any later version.                                      //
//                                                                          //
// This source file is distributed in the hope that it will be useful,      //
// but WITHOUT ANY WARRANTY; without even the implied warranty of           //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            //
// GNU General Public License for more details.                             //
//                                                                          //
// You should have received a copy of the GNU General Public License        //
// along with this program.  If not, see <http://www.gnu.org/licenses/>.    //
//                                                                          //
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
//                                                                          //
// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026:                         //
// Reduced to the 68000 (fx68k) path running entirely on the single        //
// 28.375 MHz core clock:                                                  //
//  * TG68KdotC_Kernel (68020, VHDL) is no longer instantiated; its output //
//    wires are tied to safe constants so the cpucfg muxes still           //
//    elaborate. cpucfg MUST be driven with 2'b00 by the parent.           //
//  * The SDRAM/DDR3 cpu port (sdram_ctrl/ddram_ctrl + cpu cache in        //
//    MiSTer's Minimig.sv) is not ported - all Amiga memory is BRAM        //
//    behind minimig.v's chip bus. ramsel is tied to 0, so EVERY CPU       //
//    cycle (incl. stray accesses to $DD4000-$DD5FFF, sel_dd) goes to the  //
//    chip bus where minimig_m68k_bridge.v always generates _dtack. This   //
//    is also the defensive always-ack: in the original code a sel_dd      //
//    access would wait forever on ramready and hang the CPU.              //
//  * Toccata autoconfig is disabled (sound card logic not ported).        //
//  * Zorro II/III fastram autoconfig logic is kept but self-disables      //
//    with fastramcfg = 3'b000 (the only supported configuration).         //
//                                                                          //
// October 2026 (AExp fork, WIP-V2-A11-JS-01): Zorro II Fast RAM restored  //
// for the 68000. With fastramcfg = 3'b011 the autoconfig chain announces   //
// the 8 MB board at $200000, and its bus cycles leave the chip bus via     //
// the ram* port again (ramsel / ramaddr / ramdin / ramlds / ramuds and     //
// DTACKn from ramready). The parent serves that port from the board        //
// SDRAM (CORE/vhdl/fastram_sdram.vhd). The sel_dd always-ack defence and   //
// the rest of the June 2026 notes above stay in force.                     //
//                                                                          //
// October 2026 (AExp fork): a second autoconfig board, an IDE controller   //
// that is register-compatible with RIPPLE (manufacturer 5194, product 7),  //
// so that the open-source lide.device boot ROM drives it. ide_ena puts it  //
// into the chain after the Fast RAM board; its bus cycles leave the chip   //
// bus like the Fast RAM ones, through the new ext_* port (address, data    //
// and strobes are the chip_* outputs). The board itself is                 //
// CORE/vhdl/ide_board.vhd.                                                 //
// October 2026 (Megamiga): the 68020 is back. TG68KdotC_Kernel is        //
// instantiated again next to fx68k and selected with cpucfg (00 = 68000   //
// fx68k, 11 = 68020 TG68K), both on the 28 MHz clock as on MiSTer. The    //
// Zorro III boards of MiSTer (128/256 MB, DDR3) are replaced by ONE 16 MB //
// Zorro III board (z3ena, 68020 only, needs Kickstart 2.0+), whose cycles //
// leave on the ram* port with ramaddr[28:27] = 2'b10 (Zorro II = 2'b11).  //
// The RTG window ($02xxxxxx) is not decoded. For the 68020 the IDE board  //
// select drops for one clock after every completed access, because the   //
// board ends a bus cycle only when its select goes away (68000 AS).        //
//  * The port list is unchanged for compile compatibility with the way    //
//    MiSTer's Minimig.sv instantiates this module. Unused inputs must be  //
//    tied off by the parent, unused outputs left open.                    //
//  * Original code is kept as comments; all changes carry a               //
//    "MiSTer2MEGA65" provenance comment.                                  //
//                                                                          //
// October 2026 (Megamiga, branch cpu040, EXPERIMENTAL): CPU040 = 1 puts   //
// the AP68040 pipelined core (rtl/ap040, MC68040 with MMU, FPU and caches, //
// github.com/ranzbak/AP68040-pipelined 8ead239, GPLv3) where TG68K was.    //
// It is selected like TG68K (cpucfg = 11) and keeps TG68K's 16-bit bus      //
// (the core's own ap040_bus16_adapter), so everything downstream is the    //
// 68020 path. Two additions: the MMU's table walker borrows that bus       //
// (walker bridge below), and chipset writes to Chip RAM are snooped into   //
// the data cache (snoop_stb/snoop_addr), which caches Chip RAM.            //
//                                                                          //
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//

module cpu_wrapper
#(
	parameter CPU040 = 0,
	// Megamiga (cpu040, CPU040 = 2 only): 1 = the island's write buffer, 2 = also
	// the AP68040's own speed-ups (they lengthen its paths to ~27 ns)
	parameter CPU040_FAST = 0,
	// Megamiga (cpu040, CPU040 = 2 only): 1 = the island's Fast RAM port (fp_*) for
	// Zorro II/III RAM accesses and the core's line fills; cpu_cache_new is then off
	parameter CPU040_FASTRAM = 0,
	// Megamiga (cpu040, CPU040 = 2 only): the AP68040's speed-ups one by one, OR'ed with
	// CPU040_FAST = 2 (all but copyback): bit 0 POST_STORES, 1 STORE_BUF, 2 FWD, 3 RAS,
	// 4 PRECISE, 5 SB_MMU, 6 BTFN, 7 LDX, 8 COPYBACK (Zorro RAM, needs POST_STORES)
	parameter CPU040_OPTS = 0
)
(
	input             reset,
	output reg        reset_out,

	input             clk,
	// Megamiga, October 2026: the AP68040's own clock (CPU040 = 2 only)
	input             clk_cpu,
	input             ph1,
	input             ph2,

	input       [1:0] cpucfg,
	input       [2:0] fastramcfg,
	input       [2:0] cachecfg,
	input             bootrom,
	// Megamiga, October 2026: the Action Replay III cartridge (minimig.v / ar3.v).
	// cart_ar3: it is plugged in, so $400000-$47FFFF belongs to it and not to the
	// Zorro II RAM decode (the core announces a 2 MB board then). cart_ovl: its chip
	// RAM overlay is on, so chip RAM reads must take the chip bus, not the fast path.
	input             cart_ar3,
	input             cart_ovl,

	output reg [23:1] chip_addr,
	input      [15:0] chip_dout,
	output reg [15:0] chip_din,
	output reg        chip_as,
	output reg        chip_uds,
	output reg        chip_lds,
	output reg        chip_rw,
	input             chip_dtack,
	input       [2:0] chip_ipl,
	
	input      [15:0] fastchip_dout,
	output reg        fastchip_sel,
	output            fastchip_lds,
	output            fastchip_uds,
	output            fastchip_rnw,
	output reg        fastchip_lw,
	input             fastchip_selack,
	input             fastchip_ready,

	output            ramsel,
	output     [28:1] ramaddr,
	output     [15:0] ramdin,
	input      [15:0] ramdout,
	input             ramready,
	output            ramlds,
	output            ramuds,
	output            ramshared,

	// MiSTer2MEGA65 (AExp fork), October 2026: RIPPLE-compatible IDE board
	input             ide_ena,         // board present in the autoconfig chain
	// Megamiga, October 2026: the 16 MB Zorro III board (68020 only)
	input             z3ena,
	// Megamiga, October 2026: the network card (eth_card.vhd), also on ext_*
	input             eth_ena,         // board present in the autoconfig chain
	output            ext_eth,         // the ext_sel cycle is for the network card
	// Megamiga 0.6.1: the shared folder memory $DD4000-$DD5FFF (share_buf.vhd), also on ext_*
	output            ext_shr,         // the ext_sel cycle is for the shared folder memory
	output            ext_sel,         // bus cycle to the configured board
	input      [15:0] ext_dout,        // read data of the board
	input             ext_ready,       // DTACK of the board

	output            toccata_ena,
	output reg  [7:0] toccata_base,

	output reg  [1:0] cpustate,
	output reg  [3:0] cacr,
	output reg [31:0] nmi_addr,

	// Megamiga, October 2026 (CPU040): a chipset write to Chip RAM, this clock
	input             snoop_stb,
	input      [20:1] snoop_addr,
	// Megamiga, October 2026 (CPU040 != 0): 1 = the AP68040, 0 = TG68K (static between
	// resets: the profile's CPU radio, which cold-boots the Amiga when it changes)
	input             cpu040_sel,
	// Megamiga, October 2026 (CPU040): 512 KB Slow RAM at $C00000 (the 68040 caches it)
	input             slowram_ena,
	// Megamiga, October 2026 (CPU040 = 2, CPU040_FASTRAM = 1): the 68040's Fast RAM port
	// into sdram_ctrl (see ap040_island.v); fp_addr is the SDRAM long address
	output            fp_req,
	output            fp_we,
	output            fp_line,
	output     [24:2] fp_addr,
	output      [3:0] fp_be,
	output     [31:0] fp_wdata,
	input     [127:0] fp_rdata,
	input             fp_ack
);

// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: the SDRAM/DDR3 cpu port is
// not ported (no sdram_ctrl/ddram_ctrl; all memory is BRAM on the chip bus).
// ramsel is tied to 0 so all CPU cycles - including the otherwise never-acked
// $DD4000-$DD5FFF (sel_dd) range - are routed to the chip bus, where
// minimig_m68k_bridge.v always generates _dtack (defensive against bus hangs).
// With 68000 (cpucfg=00) + fastramcfg=000 the original expression could only
// ever assert via sel_dd anyway (sel_chipram/sel_kickram need turbo=cpucfg[1],
// sel_zram needs autoconfig'd fastram, sel_rtg needs a 32-bit address).
//assign ramsel       = cpu_req & ~sel_nmi_vector & (sel_zram | sel_chipram | sel_kickram | sel_dd | sel_rtg);
//assign ramshared    = sel_dd;
// MiSTer2MEGA65 (AExp fork), October 2026: Zorro II Fast RAM restored. The
// original expression minus sel_dd, which keeps the $DD4000-$DD5FFF always-ack
// defence above: with 68000 + fastramcfg=011 only sel_zram can assert
// (sel_chipram/sel_kickram need turbo, sel_rtg a 32-bit address, and the Z3
// selects need Zorro III autoconfig, which a 68000 never gets).
//assign ramsel       = 1'b0;
// Megamiga, October 2026: without sel_rtg (no RTG framebuffer in this core)
//assign ramsel       = cpu_req & ~sel_nmi_vector & (sel_zram | sel_chipram | sel_kickram | sel_rtg);
//assign ramsel       = cpu_req & ~sel_nmi_vector & (sel_zram | sel_chipram | sel_kickram);
// Megamiga, October 2026: sel_rtg back - the RTG board memory at $02000000 (68020 only,
// a 32-bit address), served by the parent from the MEGA65's HyperRAM (ramaddr[26] below)
//assign ramsel       = cpu_req & ~sel_nmi_vector & (sel_zram | sel_chipram | sel_kickram | sel_rtg);
// Megamiga, October 2026 (CPU040): the select inside this module is ramsel_i; the
// port drops for the clock in which the AP68040 path holds a registered RAM answer
// (ram_hold, see g_ap040), always 0 for TG68K and fx68k
wire ramsel_i       = cpu_req & ~sel_nmi_vector & (sel_zram | sel_chipram | sel_kickram | sel_rtg);
wire ram_hold;
assign ramsel       = ramsel_i & ~ram_hold;
assign ramshared    = 1'b0;

// MiSTer2MEGA65 (AExp fork), October 2026: the IDE board occupies 128 KB at
// the base Kickstart assigned during autoconfig (see AUTOCONFIG below).
wire sel_ide        = ide_configured && !cpu_addr[31:24] && (cpu_addr[23:17] == ide_base[7:1]);
// Megamiga, October 2026: for the 68020, the select drops for one clock after
// every completed access (ext_done): the board holds its ready until the
// select goes away, and TG68K, unlike the 68000, has no AS gap between two
// consecutive accesses (lide.device reads the data register with MOVEM).
//assign ext_sel      = cpu_req & ~sel_nmi_vector & sel_ide;
// Megamiga, October 2026: + sel_eth, the network card (64 KB at its autoconfig base)
//assign ext_sel      = cpu_req & ~sel_nmi_vector & sel_ide & ~ext_done;
reg       ac_eth;          // Megamiga: the network card is being configured
reg       eth_configured;  // Megamiga: the network card has its base address
reg [7:0] eth_base;        // Megamiga: A23..A16 of the network card
wire sel_eth        = eth_configured && !cpu_addr[31:24] && (cpu_addr[23:16] == eth_base);
// Megamiga 0.6.1: + sel_shr, the shared folder memory at $DD4000-$DD5FFF (MiSTer's
// shared memory for MiSTerFileSystem, sel_dd below), served by share_buf.vhd instead of
// the always-ack of the chip bus
//assign ext_sel      = cpu_req & ~sel_nmi_vector & (sel_ide | sel_eth) & ~ext_done;
wire sel_shr        = (cpu_addr[31:16] == 16'h00DD) && (cpu_addr[15:13] == 3'b010);
assign ext_sel      = cpu_req & ~sel_nmi_vector & (sel_ide | sel_eth | sel_shr) & ~ext_done;
assign ext_eth      = sel_eth;
assign ext_shr      = sel_shr;
reg ext_done;
always @(posedge clk) ext_done <= (cpucfg[1:0] != 2'b00) & ext_sel & ext_ready;

// NMI
always @(posedge clk) nmi_addr <= vbr + 32'h7c;

wire sel_z3ram0 = (cpu_addr[31:27] == z3ram_base0) && z3ram_ena0;
wire sel_z3ram1 = (cpu_addr[31:28] == z3ram_base1) && z3ram_ena1;
//wire sel_z2ram  = !cpu_addr[31:24] && (cpu_addr[23] ^ |cpu_addr[22:21]) && z2ram_ena; // addr[23:21] = 1..4
// Megamiga, October 2026: with the Action Replay III plugged in, the board is 2 MB
// ($200000-$3FFFFF), which keeps the cartridge window $400000-$47FFFF off the fast path.
// Decoded from addr[23:21] + cart_ar3 only (one input more than upstream): carving just the
// window out (a 5-bit compare) put a LUT level on the TG68K -> sdram_ctrl paths.
wire sel_z2ram  = !cpu_addr[31:24] && z2ram_ena &&
                  (cart_ar3 ? (cpu_addr[23:21] == 3'b001) : (cpu_addr[23] ^ |cpu_addr[22:21])); // addr[23:21] = 1..4 (AR3: 1)
// Megamiga, October 2026: + the 16 MB Zorro III board (base A31..A24 from autoconfig)
wire sel_z3ram16 = (cpu_addr[31:24] == z3_base) && z3_configured;
//wire sel_zram   = sel_z3ram0 | sel_z3ram1 | sel_z2ram;
wire sel_zram   = sel_z3ram0 | sel_z3ram1 | sel_z2ram | sel_z3ram16;
wire sel_dd     = (cpu_addr[31:16] == 16'h00DD) && (cpu_addr[15:13] == 'b010);
//wire sel_rtg    = (cpu_addr[31:24] == 8'h02);
// Megamiga, October 2026: + the memory half of the RTG autoconfig board
wire sel_rtgb   = rtgb_configured && (cpu_addr[31:23] == rtgb_base);
wire sel_rtgreg = sel_rtgb & cpu_addr[22];        // its register half
wire sel_rtg    = (cpu_addr[31:24] == 8'h02) | (sel_rtgb & ~cpu_addr[22]);

// don't sel_kickram when writing
wire sel_kickram   = !cpu_addr[31:24] && (&cpu_addr[23:19] || (cpu_addr[23:19] == 5'b11100)) && ckick && wr;	// $f8xxxx, e0xxxx
wire sel_kicklower = !cpu_addr[31:24] && (cpu_addr[23:18] == 6'b111110);
wire sel_chipram   = !cpu_addr[31:21] && cchip; 		             //$000000 - $1FFFFF

// we route everything hrtmon related through cart.v (needs a couple of signals to
// decide what to do, would not be good style to replicate that here). 
wire sel_nmi_vector = (cpu_addr[31:2] == nmi_addr[31:2]) && (cpustate == 2);

wire [15:0] ramdat;

// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: SDRAM cpu-port outputs tied
// inactive (ports kept only for compile compatibility - leave open in parent).
// MiSTer2MEGA65 (AExp fork), October 2026: restored for the Zorro II Fast RAM
// (the June 2026 tie-offs are kept below as comments). The strobes are the
// CPU's active-low UDS/LDS.
//assign ramlds = 1'b0;
//assign ramuds = 1'b0;
//assign ramdin = 16'h0000;
assign ramlds = sel_rtg ? uds_in : lds_in;
assign ramuds = sel_rtg ? lds_in : uds_in;
assign ramdin = sel_rtg ? {cpu_dout[7:0],cpu_dout[15:8]} : cpu_dout;
assign ramdat = sel_rtg ? {ramdout[7:0], ramdout[15:8]}  : ramdout;

//       Main  DDx  RTG  8M  128M  256M
//       ----  ---  ---  --  ----  ----
//        SDR  DDR  RTG  Z2  Z3_0  Z3_1
// 28      0    0    0   1    0     1
// 27      0    0    0   1    1     X
// 26      0    1    1   0    X     X
// 25-23   0   111  110  0    X     X
// supported configs: SDR + (Z2, Z3_1, Z3_0+Z3_1)

// This is the mapping to the sram
// map 00-1f to 00-1f (chipram), a0-ff to 20-7f. All non-fastram goes into the first
// 8M block(SDRAM). This map should be the same as in minimig_sram_bridge.v
// All Zorro RAM goes to DDR3
// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: SDRAM cpu-port address
// mapping tied to 0 - the SDRAM/DDR3 cpu port is not ported (see ramsel).
//assign ramaddr[28]    = sel_zram & ~sel_z3ram0;
//assign ramaddr[27]    = sel_zram & (~sel_z3ram1 | cpu_addr[27]);
//assign ramaddr[26:23] = (sel_z3ram0 | sel_z3ram1) ? cpu_addr[26:23]: (sel_rtg ? 4'b1110 : {4{sel_dd}});
//assign ramaddr[22:19] = {4{sel_dd}} | cpu_addr[22:19];
//assign ramaddr[18]    =    sel_dd   | (sel_kicklower & bootrom) | cpu_addr[18];
//assign ramaddr[17:16] = {2{sel_dd}} | cpu_addr[17:16];
//assign ramaddr[15:1]  = cpu_addr[15:1];
// MiSTer2MEGA65 (AExp fork), October 2026: original mapping restored for the
// Zorro II Fast RAM. For the window $200000-$9FFFFF it yields
// ramaddr[22:1] = cpu_addr[22:1], a 1:1 map onto 8 MB ($800000-$9FFFFF wraps
// onto the first 2 MB), which is all the parent uses.
//assign ramaddr[28:1]  = 28'd0;
// Megamiga, October 2026: the mapping the parent (main.vhd) expects:
// ramaddr[28:27] = 2'b11 Zorro II (8 MB, [22:1]), 2'b10 Zorro III (16 MB,
// [23:1]), 2'b00 everything else (turbo Chip/Kickstart, banked like
// minimig_sram_bridge.v), with ramaddr[26] = 1 for the RTG board memory.
// sel_dd never reaches the ram* port (see ramsel).
//assign ramaddr[28]    = sel_zram & ~sel_z3ram0;
//assign ramaddr[27]    = sel_zram & (~sel_z3ram1 | cpu_addr[27]);
//assign ramaddr[26:23] = (sel_z3ram0 | sel_z3ram1) ? cpu_addr[26:23]: (sel_rtg ? 4'b1110 : {4{sel_dd}});
//assign ramaddr[22:19] = {4{sel_dd}} | cpu_addr[22:19];
//assign ramaddr[18]    =    sel_dd   | (sel_kicklower & bootrom) | cpu_addr[18];
//assign ramaddr[17:16] = {2{sel_dd}} | cpu_addr[17:16];
//assign ramaddr[15:1]  = cpu_addr[15:1];
assign ramaddr[28]    = sel_z2ram | sel_z3ram16;
assign ramaddr[27]    = sel_z2ram;
// Megamiga, October 2026: ramaddr[26] = 1: the RTG board memory ($02xxxxxx, [22:1] the offset)
//assign ramaddr[26:24] = 3'b000;
assign ramaddr[26]    = sel_rtg;
assign ramaddr[25:24] = 2'b00;
assign ramaddr[23]    = sel_z3ram16 & cpu_addr[23];
assign ramaddr[22:19] = cpu_addr[22:19];
assign ramaddr[18]    = (sel_kicklower & bootrom) | cpu_addr[18];
assign ramaddr[17:1]  = cpu_addr[17:1];

assign fastchip_lds = lds_in;
assign fastchip_uds = uds_in;
assign fastchip_rnw = wr;

reg  [31:0] cpu_addr;
reg  [15:0] cpu_dout;
// MiSTer2MEGA65 (AExp fork), October 2026: + the IDE board (ext_*)
//wire [15:0] cpu_din = ramsel ? ramdat : fastchip_selack ? fastchip_dout : {sel_autoconfig ? autocfg_data : chip_data[15:12], chip_data[11:0]};
wire [15:0] cpu_din = ramsel_i ? ramdat : ext_sel ? ext_dout : fastchip_selack ? fastchip_dout : {sel_autoconfig ? autocfg_data : chip_data[15:12], chip_data[11:0]};
reg         wr;
reg         uds_in;
reg         lds_in;
reg  [15:0] chip_data;
reg  [31:0] vbr;

// Megamiga, October 2026: declared here, before the CPU generate block below
// (CPU040), which uses them: a forward reference from inside a generate block
// would declare a new, undriven net in the block's own scope.
wire       cpu_req = (cpustate != 1);
reg  [2:0] cpu_ipl;
reg        chipready;
reg        z2ram_ena;
reg        z3_configured;   // Megamiga: the Zorro III board has its base address
reg  [7:0] z3_base;         // Megamiga: A31..A24 of the Zorro III board
// Megamiga, October 2026 (cpu040 branch): the RTG card as an autoconfig Zorro III
// board (8 MB, I/O type, manufacturer 2011, product 102), so that an MMU setup
// maps it like every other board (Commodore's 68040.library maps the memory
// list and the configured boards and nothing else: with the card only at
// MiSTer's fixed addresses, Picasso96 took an access fault on a 68040).
// Board offset $000000-$3FFFFF = the board memory (as $02000000), $400000 and
// up = the register page of $B80000 ($B80100 registers, $B80400 palette,
// $B80800 blitter), repeated every 4 KB. The fixed addresses stay as well.
reg        ac_rtgb;         // the RTG board is being configured
reg        rtgb_configured; // the RTG board has its base address
reg  [8:0] rtgb_base;       // A31..A23 of the RTG board

always @* begin
	if(cpucfg[1:0]) begin
		cpu_dout     = cpu_dout_p;
		cpu_addr     = cpu_addr_p;
		cpustate     = cpustate_p;
		cacr         = cacr_p;
		vbr          = vbr_p;
		wr           = wr_p;
		uds_in       = uds_p;
		lds_in       = lds_p;
		reset_out    = reset_out_p;
		chip_as      = c_as;
		chip_rw      = c_rw;
		chip_uds     = c_uds;
		chip_lds     = c_lds;
		// Megamiga: the RTG board's register half reaches the fastchip port as $B80xxx
		//chip_addr    = cpu_addr_p[23:1];
		chip_addr    = sel_rtgreg ? {12'hB80, cpu_addr_p[11:1]} : cpu_addr_p[23:1];
		chip_din     = cpu_dout_p;
		chip_data    = chipdout_i;
		//fastchip_sel = cpu_req & !cpu_addr_p[31:24];
		fastchip_sel = cpu_req & (!cpu_addr_p[31:24] | sel_rtgreg);
		fastchip_lw  = longword;
	end
	else begin
		cpu_dout     = cpu_dout_o;
		cpu_addr     = {cpu_addr_o,1'b0};
		cpustate     = as_o ? 2'b01 : ~{wr_o,wr_o};
		cacr         = 1;
		vbr          = 0;
		wr           = wr_o;
		uds_in       = uds_o;
		lds_in       = lds_o;
		reset_out    = reset_out_o;
		chip_as      = ramsel_i | ext_sel | as_o;    // AExp: ext_sel added
		chip_rw      = wr_o;
		chip_uds     = uds_o;
		chip_lds     = lds_o;
		chip_addr    = cpu_addr_o[23:1];
		chip_din     = cpu_dout_o;
		chip_data    = chip_dout;
		fastchip_sel = 0;
		fastchip_lw  = 0;
	end
end

// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: the TG68KdotC_Kernel
// (68020 soft CPU, VHDL) is not ported - this core is 68000 (fx68k) only and
// cpucfg is always 2'b00. The instance is removed (it was instantiated
// unconditionally and would synthesize ~10k LUTs of dead logic and pull in
// the tg68k VHDL sources). Its former output wires are tied to safe inactive
// constants so the cpucfg muxes above still elaborate unchanged:
//   cpustate_p = 2'b01 ("no memaccess") so cpu_req would stay low,
//   nwr/nuds/nlds = 1 (active low, deasserted), nresetout = 1 (not in reset).
// Megamiga, October 2026: TG68K restored (the June 2026 tie-offs below are kept
// as comments). clkena also completes IDE board cycles (ext_ready).
//wire [15:0] cpu_dout_p  = 16'h0000;
//wire [31:0] cpu_addr_p  = 32'h00000000;
//wire  [1:0] cpustate_p  = 2'b01;
//wire  [3:0] cacr_p      = 4'b0000;
//wire [31:0] vbr_p       = 32'h00000000;
//wire        wr_p        = 1'b1;
//wire        uds_p       = 1'b1;
//wire        lds_p       = 1'b1;
//wire        reset_out_p = 1'b1;
//wire        longword    = 1'b0;
wire [15:0] cpu_dout_p;
wire [31:0] cpu_addr_p;
wire  [1:0] cpustate_p;
wire  [3:0] cacr_p;
wire [31:0] vbr_p;
wire        wr_p;
wire        uds_p;
wire        lds_p;
wire        reset_out_p;
wire        longword;

// Megamiga, October 2026: TG68K, or the AP68040 core (CPU040, see below)
generate if (CPU040 == 0) begin : g_tg68k
TG68KdotC_Kernel
#(
	.sr_read(2),        // 0=>user,   1=>privileged,    2=>switchable with CPU(0)
	.vbr_stackframe(2), // 0=>no,     1=>yes/extended,  2=>switchable with CPU(0)
	.extaddr_mode(2),   // 0=>no,     1=>yes,           2=>switchable with CPU(1)
	.mul_mode(2),       // 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no MUL,
	.div_mode(2),       // 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no DIV,
	.bitfield(2)        // 0=>no,     1=>yes,           2=>switchable with CPU(1)
)
cpu_inst_p
(
	.clk(clk),
	.nreset(reset),
	//.clkena_in(~cpu_req | chipready | ramready | fastchip_ready),
	.clkena_in(~cpu_req | chipready | ramready | fastchip_ready | ext_ready),
	.data_in(cpu_din),
	.ipl(cpu_ipl),
	.ipl_autovector(1),
	.regin_out(),
	.addr_out(cpu_addr_p),
	.data_write(cpu_dout_p),
	.nwr(wr_p),
	.nuds(uds_p),
	.nlds(lds_p),
	.nresetout(reset_out_p),
	.longword(longword),

	.cpu(cpucfg),
	.busstate(cpustate_p),		// 0: fetch code, 1: no memaccess, 2: read data, 3: write data
	.cacr_out(cacr_p),
	.vbr_out(vbr_p)
);
assign ram_hold = 1'b0;
assign {fp_req, fp_we, fp_line, fp_addr, fp_be, fp_wdata} = 0;
end
else begin : g_ap040
// Megamiga, October 2026: TG68K AND the AP68040, chosen at run time by cpu040_sel (the
// profile's CPU radio; a change cold-boots the Amiga). The core not selected is held in
// reset; its outputs are multiplexed out below (f_* = the 68040, t_* = TG68K).
wire [15:0] f_dout;
wire [31:0] f_addr;
wire  [1:0] f_state;
wire  [3:0] f_cacr;
wire [31:0] f_vbr;
wire        f_wr, f_uds, f_lds, f_rst_out, f_lw;
// The AP68040 pipelined core with TG68K's bus (AP040_BUS16 = 1: its
// ap040_bus16_adapter splits every transfer into TG68K-style 16-bit cycles
// with an idle cycle between them, completed by the same clock enable). The
// MMU's table walker has its own 32-bit port; the walker bridge below runs it
// as two 16-bit cycles on the same bus while the adapter is idle, and freezes
// the core (clock enable low) for the length of those cycles, so the adapter
// can neither start nor complete anything under them. First bring-up
// configuration: synchronous stores (no posted stores, no store buffer), none
// of the optional speed-ups, the fill channel off.
wire [15:0] a_dout;
wire [31:0] a_addr, a_cacr;
wire  [1:0] a_state;
wire        a_wr, a_uds, a_lds, a_lw;
wire        a_maint;
wire        wk_hold;      // the walker bridge owns the bus (CPU040 = 1 only)

// The RAM answer comes from the SDRAM controller's clock (113.5 MHz): taken
// combinationally into the clock enable it reached the core's bus unit through
// 19 logic levels in one fast period (build cpu040-a, -0.41 ns). Registered here
// instead, at the cost of one CPU clock per RAM access, like apolkosnik's MiSTer
// wrapper. amiga_sdram ends a 68020 cycle by dropping its chip select on the CPU
// clock in which it signals ready; the port select stays low in the clock that
// holds the registered answer, so the select does not come back for a second
// access before the adapter has gone idle.
reg        ram_rdy_r;
reg [15:0] ram_dat_r;
always @(posedge clk) begin
	if (!reset) ram_rdy_r <= 0;
	else        ram_rdy_r <= ~ram_rdy_r & cpu_req & ramsel_i & ramready;
	if (ramready) ram_dat_r <= ramdat;
end
//assign ram_hold = ram_rdy_r;
assign ram_hold = ram_rdy_r & cpu040_sel;   // TG68K takes ramready as it is
wire [15:0] a_din = ram_rdy_r ? ram_dat_r : cpu_din;
// the completion TG68K's clock enable uses, with the registered RAM answer
//wire bus_done = chipready | ramready | fastchip_ready | ext_ready;
wire bus_done = chipready | ram_rdy_r | fastchip_ready | ext_ready;

// physical-bus watchdog (2^20 clocks = 37 ms: Chip RAM may wait long for a slot,
// but every target on this bus answers, so it only fires on a real fault)
wire bus_berr;
ap040_bus_timeout #(.COUNTER_BITS(20)) bus_timeout (
	.clk(clk),
	.nreset(reset & f_rst_out),
	.req(cpu_req & ~wk_hold),
	.complete(bus_done),
	.berr(bus_berr)
);

// chipset writes to Chip RAM, registered once, into the data cache's snoop port
reg        snp_stb;
reg [20:1] snp_addr;
always @(posedge clk) begin
	snp_stb  <= snoop_stb;
	snp_addr <= snoop_addr;
end

// 040 CACR: bit 15 = instruction cache. Minimig's cpu_cache_new takes bit 0 =
// enable and clears on a rising bit 3: CINV/CPUSH clear it as well.
// With the Fast RAM port that cache is off: the port bypasses it, so it would go stale.
//assign f_cacr     = {a_maint, 2'b00, a_cacr[15]};
assign f_cacr     = {a_maint, 2'b00, a_cacr[15] & ~(CPU040 == 2 && CPU040_FASTRAM != 0)};

if (CPU040 == 1) begin : g_bus
// CPU040 = 1: the core on the bus clock
wire        wk_req, wk_we;
wire [31:0] wk_addr, wk_wdat;
reg         wk_ack, wk_berr;
reg  [31:0] wk_data;

// walker bridge: IDLE -> HI (bits 31:16) -> GAP (bus idle) -> LO (15:0) -> DONE
// (ack held until the MMU drops its request, which it does for at least one
// clock between two descriptor transactions)
localparam WK_IDLE = 3'd0, WK_HI = 3'd1, WK_GAP = 3'd2, WK_LO = 3'd3, WK_DONE = 3'd4;
reg  [2:0] wk_st;
wire wk_start = (wk_st == WK_IDLE) && wk_req && (a_state == 2'b01);
wire wk_bus   = (wk_st == WK_HI) || (wk_st == WK_LO);
assign wk_hold = wk_start || wk_bus || (wk_st == WK_GAP);

always @(posedge clk) begin
	if (!reset) begin
		wk_st   <= WK_IDLE;
		wk_ack  <= 0;
		wk_berr <= 0;
	end
	else case (wk_st)
		WK_IDLE: if (wk_start) begin
				if (wk_addr[1:0] != 2'b00) begin   // a descriptor is longword-aligned
					wk_berr <= 1;
					wk_st   <= WK_DONE;
				end
				else wk_st <= WK_HI;
			end
		WK_HI:   if (bus_done) begin wk_data[31:16] <= a_din; wk_st <= WK_GAP; end
		WK_GAP:  wk_st <= WK_LO;
		WK_LO:   if (bus_done) begin wk_data[15:0] <= a_din; wk_ack <= 1; wk_st <= WK_DONE; end
		WK_DONE: if (!wk_req) begin wk_ack <= 0; wk_berr <= 0; wk_st <= WK_IDLE; end
		default: wk_st <= WK_IDLE;
	endcase
end

assign f_addr = wk_bus ? {wk_addr[31:2], wk_st == WK_LO, 1'b0} : a_addr;
assign f_dout = wk_bus ? ((wk_st == WK_LO) ? wk_wdat[15:0] : wk_wdat[31:16]) : a_dout;
assign f_state = wk_bus ? (wk_we ? 2'b11 : 2'b10) : (wk_st == WK_GAP) ? 2'b01 : a_state;
assign f_wr       = wk_bus ? ~wk_we : a_wr;
assign f_uds      = wk_bus ? 1'b0 : a_uds;
assign f_lds      = wk_bus ? 1'b0 : a_lds;
assign f_lw       = wk_bus ? 1'b0 : a_lw;

ap040_pipe_tg68k_compat #(
	.AP040_HAS_MMU(1),
	.AP040_HAS_FPU(1),
	.AP040_ENABLE_CACHE(1),
	.AP040_POST_STORES(0),
	.AP040_FILL_CHANNEL(0),
	.AP040_BUS16(1),
	.AP040_IFP(1)
) cpu_inst_p (
	.clk(clk),
	//.nreset(reset),
	.nreset(reset & cpu040_sel),   // held in reset while TG68K is selected
	.clkena_in(~wk_hold & (~cpu_req | bus_done | bus_berr)),
	.cache_allow_all(1'b0),
	.cache_snoop_stb(snp_stb),
	.cache_snoop_addr({11'd0, snp_addr, 1'b0}),
	.cache_z2_ena(z2ram_ena),
	// the 16 MB Zorro III board: its 128 MB-aligned window (nothing else is there)
	.cache_z3_base0(z3_base),   // exactly the 16 MB board (rtl/ap040 compat, Megamiga)
	.cache_z3_ena0(z3_configured),
	.cache_z3_base1(4'd0),
	.cache_z3_ena1(1'b0),
	.cache_rom_ena(1'b1),
	.cache_slow_ena(slowram_ena),
	.data_in(a_din),
	.ipl(cpu_ipl),
	.ipl_autovector(1'b1),
	.berr(bus_berr),

	.addr_out(a_addr),
	.data_write(a_dout),
	.nwr(a_wr),
	.nuds(a_uds),
	.nlds(a_lds),
	.busstate(a_state),
	.longword(a_lw),
	.nresetout(f_rst_out),
	.fc(),
	.nmi_ack_toggle(),
	.fill_ena_zorro(1'b0),
	.fill_ena_chip(1'b0),
	.fill_req(),
	.fill_addr(),
	.fill_data(128'd0),
	.fill_ack(1'b0),
	.fill_err(1'b0),
	.cache_maint_req(a_maint),
	.cache_maint_ic(),
	.cache_maint_dc(),

	.mmu_addr_log(),
	.mmu_addr_phys(),
	.mmu_cache_inhibit(),

	.walker_req(wk_req),
	.walker_we(wk_we),
	.walker_addr(wk_addr),
	.walker_wdat(wk_wdat),
	.walker_ack(wk_ack),
	.walker_data(wk_data),
	.walker_berr(wk_berr),

	.cache_req(),
	.cache_addr(),
	.cache_data(16'd0),
	.cache_ack(1'b0),
	.cache_burst(),
	.cache_burst_len(),
	.cache_ramaddr(),

	.cacr_out(a_cacr),
	.vbr_out(f_vbr),
	.debug_busy(),
	.debug_fault(),
	.debug_halted(),
	.debug_status(),
	.debug_status2(),

	.m_req(),
	.m_write(),
	.m_instr(),
	.m_size(),
	.m_addr(),
	.m_wdata(),
	.m_fc(),
	.m_ack(1'b0),
	.m_rdata(32'd0)
);
assign {fp_req, fp_we, fp_line, fp_addr, fp_be, fp_wdata} = 0;
end
else begin : g_island
// CPU040 = 2: the core on its own clock (clk_cpu), behind ap040_island's request
// channel; its table walks use the same channel, so there is no walker bridge
assign wk_hold = 1'b0;
ap040_island #(
	// CPU040_FAST: 1 = the island's write buffer, 2 = also the core's own speed-ups
	//.POST_STORES(CPU040_FAST >= 2), .STORE_BUF(CPU040_FAST >= 2), .FWD(CPU040_FAST >= 2),
	//.RAS(CPU040_FAST >= 2), .PRECISE(CPU040_FAST >= 2), .SB_MMU(CPU040_FAST >= 2),
	//.BTFN(CPU040_FAST >= 2), .LDX(CPU040_FAST >= 2),
	.POST_STORES(CPU040_FAST >= 2 || CPU040_OPTS[0]), .STORE_BUF(CPU040_FAST >= 2 || CPU040_OPTS[1]),
	.FWD(CPU040_FAST >= 2 || CPU040_OPTS[2]), .RAS(CPU040_FAST >= 2 || CPU040_OPTS[3]),
	.PRECISE(CPU040_FAST >= 2 || CPU040_OPTS[4]), .SB_MMU(CPU040_FAST >= 2 || CPU040_OPTS[5]),
	.BTFN(CPU040_FAST >= 2 || CPU040_OPTS[6]), .LDX(CPU040_FAST >= 2 || CPU040_OPTS[7]),
	.COPYBACK(CPU040_OPTS[8]),
	.WRITE_BUF(CPU040_FAST >= 1),
	.FASTRAM(CPU040_FASTRAM)
) cpu_inst_p (
	.clk_cpu(clk_cpu),
	.clk(clk),
	//.nreset(reset),
	.nreset(reset & cpu040_sel),   // held in reset while TG68K is selected
	.clkena_in(~cpu_req | bus_done | bus_berr),
	.berr(bus_berr),
	.data_in(a_din),
	.addr_out(f_addr),
	.data_write(f_dout),
	.nwr(f_wr),
	.nuds(f_uds),
	.nlds(f_lds),
	.busstate(f_state),
	.longword(f_lw),
	.ipl(cpu_ipl),
	.snoop_stb(snp_stb),
	.snoop_addr({11'd0, snp_addr, 1'b0}),
	.cache_z2_ena(z2ram_ena),
	.cache_z3_base0(z3_base),
	.cache_z3_ena0(z3_configured),
	.cache_rom_ena(1'b1),
	.cache_slow_ena(slowram_ena),
	.fp_req(fp_req),
	.fp_we(fp_we),
	.fp_line(fp_line),
	.fp_addr(fp_addr),
	.fp_be(fp_be),
	.fp_wdata(fp_wdata),
	.fp_rdata(fp_rdata),
	.fp_ack(fp_ack),
	.nresetout(f_rst_out),
	.cacr_out(a_cacr),
	.vbr_out(f_vbr),
	.cache_maint(a_maint)
);
end

// TG68K, as in g_tg68k, for the 68020 setting
wire [15:0] t_dout;
wire [31:0] t_addr;
wire  [1:0] t_state;
wire  [3:0] t_cacr;
wire [31:0] t_vbr;
wire        t_wr, t_uds, t_lds, t_rst_out, t_lw;
TG68KdotC_Kernel
#(
	.sr_read(2),
	.vbr_stackframe(2),
	.extaddr_mode(2),
	.mul_mode(2),
	.div_mode(2),
	.bitfield(2)
)
tg68k_inst
(
	.clk(clk),
	.nreset(reset & ~cpu040_sel),   // held in reset while the 68040 is selected
	.clkena_in(~cpu_req | chipready | ramready | fastchip_ready | ext_ready),
	.data_in(cpu_din),
	.ipl(cpu_ipl),
	.ipl_autovector(1),
	.regin_out(),
	.addr_out(t_addr),
	.data_write(t_dout),
	.nwr(t_wr),
	.nuds(t_uds),
	.nlds(t_lds),
	.nresetout(t_rst_out),
	.longword(t_lw),
	.cpu(cpucfg),
	.busstate(t_state),
	.cacr_out(t_cacr),
	.vbr_out(t_vbr)
);
assign cpu_addr_p  = cpu040_sel ? f_addr    : t_addr;
assign cpu_dout_p  = cpu040_sel ? f_dout    : t_dout;
assign cpustate_p  = cpu040_sel ? f_state   : t_state;
assign cacr_p      = cpu040_sel ? f_cacr    : t_cacr;
assign vbr_p       = cpu040_sel ? f_vbr     : t_vbr;
assign wr_p        = cpu040_sel ? f_wr      : t_wr;
assign uds_p       = cpu040_sel ? f_uds     : t_uds;
assign lds_p       = cpu040_sel ? f_lds     : t_lds;
assign reset_out_p = cpu040_sel ? f_rst_out : t_rst_out;
assign longword    = cpu040_sel ? f_lw      : t_lw;
end endgenerate

//TG68KdotC_Kernel
//#(
//	.sr_read(2),        // 0=>user,   1=>privileged,    2=>switchable with CPU(0)
//	.vbr_stackframe(2), // 0=>no,     1=>yes/extended,  2=>switchable with CPU(0)
//	.extaddr_mode(2),   // 0=>no,     1=>yes,           2=>switchable with CPU(1)
//	.mul_mode(2),       // 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no MUL,
//	.div_mode(2),       // 0=>16Bit,  1=>32Bit,         2=>switchable with CPU(1),  3=>no DIV,
//	.bitfield(2)        // 0=>no,     1=>yes,           2=>switchable with CPU(1)
//)
//cpu_inst_p
//(
//  .clk(clk),
//  .nreset(reset),
//  .clkena_in(~cpu_req | chipready | ramready | fastchip_ready),
//  .data_in(cpu_din),
//  .ipl(cpu_ipl),
//  .ipl_autovector(1),
//  .regin_out(),
//  .addr_out(cpu_addr_p),
//  .data_write(cpu_dout_p),
//  .nwr(wr_p),
//  .nuds(uds_p),
//  .nlds(lds_p),
//  .nresetout(reset_out_p),
//  .longword(longword),
//
//  .cpu(cpucfg),
//  .busstate(cpustate_p),		// 0: fetch code, 1: no memaccess, 2: read data, 3: write data
//  .cacr_out(cacr_p),
//  .vbr_out(vbr_p)
//);

wire [15:0] cpu_dout_o;
wire [23:1] cpu_addr_o;
wire  [2:0] fc_o;
wire        wr_o;
wire        as_o;
wire        uds_o;
wire        lds_o;
wire        reset_out_o;

fx68k cpu_inst_o
(
	.clk(clk),
	.enPhi1(ph1),
	.enPhi2(ph2),

	.extReset(~reset),
	.pwrUp(~reset),
	.oRESETn(reset_out_o),
	.HALTn(1),

	.eRWn(wr_o),
	.ASn(as_o),
	.LDSn(lds_o),
	.UDSn(uds_o),
	// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: ramsel is tied to 0
	// (no SDRAM cpu port), so DTACKn always comes from the chip bus, which
	// minimig_m68k_bridge.v acknowledges for every address - no bus hangs.
	// MiSTer2MEGA65 (AExp fork), October 2026: original restored for the
	// Zorro II Fast RAM. ramsel no longer includes sel_dd, so every non-Fast-RAM
	// cycle still gets its DTACK from the chip bus.
	//.DTACKn(chip_dtack),
	// MiSTer2MEGA65 (AExp fork), October 2026: + the IDE board (ext_ready)
	.DTACKn(ramsel_i ? ~ramready : ext_sel ? ~ext_ready : chip_dtack),

	.FC0(fc_o[0]),
	.FC1(fc_o[1]),
	.FC2(fc_o[2]), 

	.VPAn(~&fc_o),
	.BERRn(1),
	.BRn(1),
	.BGACKn(1),
	.IPL0n(chip_ipl[0]),
	.IPL1n(chip_ipl[1]),
	.IPL2n(chip_ipl[2]),
	.iEdb(cpu_din),
	.oEdb(cpu_dout_o),
	.eab(cpu_addr_o)
);

// Megamiga, October 2026: declared above the CPU generate block (CPU040)
//wire cpu_req = (cpustate != 1);

//wire cchip = turbochip_d & (!cpustate | dcache_d);
// Megamiga, October 2026: no chip RAM fast path while the Action Replay III overlay is on
// (Minimig 1.x switched turbo off the same way while "int7 | ovr")
wire cchip = turbochip_d & (!cpustate | dcache_d) & ~cart_ovl;
wire ckick = turbokick_d & (!cpustate | dcache_d);

// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: turbo chipram/kickstart
// selects kept unchanged - they self-disable with cpucfg[1]=0 (68000), so
// cchip/ckick and thus sel_chipram/sel_kickram are constant 0 in this port.
reg turbochip_d;
reg turbokick_d;
reg dcache_d;
always @(posedge clk) begin
	if (~reset | ~reset_out) begin
		turbochip_d <= 0;
		turbokick_d <= 0;
		dcache_d    <= 0;
	end
	else if (~cpu_req) begin	// No mem access, so safe to switch chipram access mode
		turbochip_d <= cachecfg[0] & cpucfg[1];
		turbokick_d <= cachecfg[1] & cpucfg[1];
		dcache_d    <= cachecfg[2];
	end
end

reg       chipreq;
//reg [2:0] cpu_ipl;     // Megamiga: declared above the CPU generate block
always @(posedge clk) begin
	// Megamiga, October 2026: + ~ext_sel (IDE board cycles do not touch the chip bus)
	//chipreq <= cpu_req & ~ramsel & ~fastchip_selack;
	// Megamiga, October 2026: + the network card
	//chipreq <= cpu_req & ~ramsel & ~fastchip_selack & ~ext_sel & ~(sel_ide & ext_done);
	// Megamiga 0.6.1: + the shared folder memory
	//chipreq <= cpu_req & ~ramsel_i & ~fastchip_selack & ~ext_sel & ~((sel_ide | sel_eth) & ext_done);
	chipreq <= cpu_req & ~ramsel_i & ~fastchip_selack & ~ext_sel & ~((sel_ide | sel_eth | sel_shr) & ext_done);
	cpu_ipl <= ipl_i;
end

reg ph1n, ph2n;
always @(posedge clk) begin
	ph1n <= ph1;
	ph2n <= ph2;
end

//reg        chipready;  // Megamiga: declared above the CPU generate block
reg [15:0] chipdout_i;
reg  [2:0] ipl_i;
reg        c_as,c_rw,c_uds,c_lds;
// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: chip-bus FSM kept per
// porting plan. NOTE: its outputs (c_as/c_rw/c_uds/c_lds, chipready,
// chipdout_i, ipl_i) are only consumed by the cpucfg!=0 (TG68K) branch of
// the mux above; the fx68k path drives the chip bus directly. With cpucfg
// tied to 2'b00 in the parent, Vivado constant-propagates this FSM away.
// This is the only negedge-clk logic in the core (constrain accordingly).
// Vivado fix (see .research/c64_mister-diff.md B1): local reg declarations
// require a NAMED block in plain Verilog mode.
always @(negedge clk, negedge reset) begin : chipbus_fsm
	reg [1:0] stage;
	reg waitm;
	reg ready;

	if(~reset) begin
		stage <= 0;
		c_as <= 1;
		c_rw <= 1;
		c_uds <= 1;
		c_lds <= 1;
		ready <= 0;
	end
	else begin
		if (ph2n) begin
			waitm <= chip_dtack;
			if(~stage[0]) ipl_i <= chip_ipl;
		end

		chipready <= 0;
		if (ph1n) begin
			chipready <= ready;
			ready <= 0;
			case (stage)
				0: if (chipreq) begin
						c_as <= 0;
						c_rw <= wr;
						c_uds <= uds_in;
						c_lds <= lds_in;
						stage <= 1;
					end
				1: stage <= 2;
				2: begin
						chipdout_i <= chip_dout;
						if (~waitm) begin
							c_as <= 1;
							c_rw <= 1;
							c_uds <= 1;
							c_lds <= 1;
							ready <= 1;
							stage <= 3;
						end
					end
				3: stage <= 0;
			endcase
		end
	end
end

///////////////////// AUTOCONFIG ////////////////////////////

reg       ac_toccata;
reg [2:0] ac_memcard;
reg       ac_ide;          // AExp: the IDE board is the board being configured
reg       ac_z3;           // Megamiga: the 16 MB Zorro III board is being configured
//reg       z3_configured;   // Megamiga: declared above the CPU generate block
//reg [7:0] z3_base;         // Megamiga: declared above the CPU generate block
reg       ide_configured;  // AExp: the IDE board has its base address
reg [7:0] ide_base;        // AExp: A23..A16 of the IDE board
reg [3:0] autocfg_data;

always @(*) begin
	autocfg_data = 4'b1111;

	// Zorro II RAM (Up to 8 meg at 0x200000). It has a fixed base, so it must be first in the chain.
	if (~ac_memcard[2] && ac_memcard[1:0]) begin
		case (chip_addr[6:1])
			6'b000000: autocfg_data = 4'b1110;	// Zorro-II card, add mem, no ROM
			6'b000001:
				case (ac_memcard[1:0])
							1: autocfg_data = 4'b0110; // 2MB
							2: autocfg_data = 4'b0111; // 4MB
					default: autocfg_data = 4'b0000; // 8MB
				endcase
			6'b001000: autocfg_data = 4'b1110;	// Manufacturer ID: 0x139c
			6'b001001: autocfg_data = 4'b1100;
			6'b001010: autocfg_data = 4'b0110;
			6'b001011: autocfg_data = 4'b0011;
			6'b010011: autocfg_data = 4'b1110; //serial=1
			  default:;
		endcase
	end
	// Megamiga, October 2026: the 16 MB Zorro III board (68020 + Kickstart 2.0+).
	// MiSTer's 128/256 MB entries below with the size code 16 MB (extended
	// size: 000). Registers 0x00/0x02 are not inverted, all others are.
	else if(ac_z3) begin
		case (chip_addr[6:1])
			6'b000000: autocfg_data = 4'b1010;	// Zorro-III card, add mem, no ROM
			6'b000001: autocfg_data = 4'b0000;	// 16 MB (extended size code 000)
			6'b000010: autocfg_data = 4'b1110;	// ProductID=0x10 (only setting upper nibble)
			6'b000100: autocfg_data = 4'b0000;	// Memory card, not silenceable, Extended size, reserved.
			6'b000101: autocfg_data = 4'b1111;	// logical size matches physical size
			6'b001000: autocfg_data = 4'b1110;	// Manufacturer ID: 0x139c
			6'b001001: autocfg_data = 4'b1100;
			6'b001010: autocfg_data = 4'b0110;
			6'b001011: autocfg_data = 4'b0011;
			6'b010011: autocfg_data = 4'b1101;	// serial=2
			  default:;
		endcase
	end
	// MiSTer2MEGA65 (AExp fork), October 2026: the IDE board, values of RIPPLE
	// (github.com/LIV2/RIPPLE-IDE RTL/autoconfig.v): Zorro II I/O board with
	// a DiagArea ROM at offset 0x0008, 128 KB, product 7, manufacturer 5194.
	// Registers 0x00/0x02 are not inverted, all others are.
	// Megamiga, October 2026: the RTG board (68020/68040): Zorro III I/O board, 8 MB,
	// no ROM, product 102, manufacturer 2011; flags: not silenceable, Zorro III.
	// Registers 0x00/0x02 are not inverted, all others are.
	else if(ac_rtgb) begin
		case (chip_addr[6:1])
			6'h00: autocfg_data = 4'b1000; // Zorro III, not memory, no ROM
			6'h01: autocfg_data = 4'b0000; // 8 MB (size code 000, not extended)
			6'h02: autocfg_data = ~4'h6;   // product 102 = 0x66
			6'h03: autocfg_data = ~4'h6;
			6'h04: autocfg_data = ~4'b0101; // er_Flags: NOSHUTUP, ZORRO_III
			6'h08: autocfg_data = ~4'h0;   // manufacturer 0x07DB = 2011
			6'h09: autocfg_data = ~4'h7;
			6'h0A: autocfg_data = ~4'hD;
			6'h0B: autocfg_data = ~4'hB;
			default: ;                     // 4'b1111 = inverted 0
		endcase
	end
	else if(ac_ide) begin
		case (chip_addr[6:1])
			6'h00: autocfg_data = 4'b1101; // Zorro II, not memory, DiagArea valid
			6'h01: autocfg_data = 4'b0010; // 128 KB
			6'h02: autocfg_data = ~4'h0;   // product 7
			6'h03: autocfg_data = ~4'h7;
			6'h08: autocfg_data = ~4'h1;   // manufacturer 0x144A = 5194
			6'h09: autocfg_data = ~4'h4;
			6'h0A: autocfg_data = ~4'h4;
			6'h0B: autocfg_data = ~4'hA;
			6'h17: autocfg_data = ~4'h8;   // er_InitDiagVec = 0x0008
			default: ;                     // 4'b1111 = inverted 0
		endcase
	end
	// Megamiga, October 2026: the network card (eth_card.vhd): Zorro II I/O
	// board, 64 KB, no ROM, product 101, manufacturer 2011 ($07DB, the
	// experimental-board ID). Registers 0x00/0x02 are not inverted.
	else if(ac_eth) begin
		case (chip_addr[6:1])
			6'h00: autocfg_data = 4'b1100; // Zorro II, not memory, no ROM
			6'h01: autocfg_data = 4'b0001; // 64 KB
			6'h02: autocfg_data = ~4'h6;   // product 101 = 0x65
			6'h03: autocfg_data = ~4'h5;
			6'h08: autocfg_data = ~4'h0;   // manufacturer 0x07DB = 2011
			6'h09: autocfg_data = ~4'h7;
			6'h0A: autocfg_data = ~4'hD;
			6'h0B: autocfg_data = ~4'hB;
			default: ;                     // 4'b1111 = inverted 0
		endcase
	end
	// Zorro II other cards
	else if(ac_toccata) begin
		case (chip_addr[6:1])
			6'h0: autocfg_data = 4'b1100; // Zorro-II card, no link, no ROM
			6'h1: autocfg_data = 4'b0001; // Next board not related, size 'h64k
			// Inverted from here on
			6'h3: autocfg_data = 4'b0011; // Lower byte product number
			//6'h5: autocfg_data = 4'b1101; // logical size 64k -- commented out -> logical size == physical size. Issue with KS1.3?
			6'h8: autocfg_data = 4'b1011; // Manufacturer ID: 0x4754
			6'h9: autocfg_data = 4'b1000;
			6'ha: autocfg_data = 4'b1010;
			6'hb: autocfg_data = 4'b1011;
			default: ;
		endcase
	end 
	// Zorro III RAM 128MB/256MB/384MB
	else if(ac_memcard[2]) begin
		case (chip_addr[6:1])
			6'b000000: autocfg_data = 4'b1010;	// Zorro-III card, add mem, no ROM
			6'b000001: autocfg_data = ac_memcard[1] ? 4'b0011 : 4'b0100; // 128MB or 256MB, extended
			6'b000010: autocfg_data = 4'b1110;	// ProductID=0x10 (only setting upper nibble)
			6'b000100: autocfg_data = 4'b0000;	// Memory card, not silenceable, Extended size, reserved.
			6'b000101: autocfg_data = 4'b1111;	// 0000 - logical size matches physical size TODO change this to 0001, so it is autosized by the OS, WHEN it will be 24MB.
			6'b001000: autocfg_data = 4'b1110;	// Manufacturer ID: 0x139c
			6'b001001: autocfg_data = 4'b1100;
			6'b001010: autocfg_data = 4'b0110;
			6'b001011: autocfg_data = 4'b0011;
			6'b010011: autocfg_data = {2'b11, ~ac_memcard[1], ac_memcard[1]};	// serial=1/2
			  default:;
		endcase
	end
end

// MiSTer2MEGA65 (AExp fork), October 2026: + ac_ide
//wire sel_autoconfig = (chip_addr[23:16] == 8'b11101000) && (ac_memcard || ac_toccata); //$E80000 - $E8FFFF
// Megamiga, October 2026: + ac_z3
//wire sel_autoconfig = (chip_addr[23:16] == 8'b11101000) && (ac_memcard || ac_ide || ac_toccata); //$E80000 - $E8FFFF
// Megamiga, October 2026: + ac_eth
//wire sel_autoconfig = (chip_addr[23:16] == 8'b11101000) && (ac_memcard || ac_z3 || ac_ide || ac_toccata); //$E80000 - $E8FFFF
//wire sel_autoconfig = (chip_addr[23:16] == 8'b11101000) && (ac_memcard || ac_z3 || ac_ide || ac_eth || ac_toccata); //$E80000 - $E8FFFF
// Megamiga, October 2026: + ac_rtgb
wire sel_autoconfig = (chip_addr[23:16] == 8'b11101000) && (ac_memcard || ac_z3 || ac_rtgb || ac_ide || ac_eth || ac_toccata); //$E80000 - $E8FFFF

//reg       z2ram_ena;  // Megamiga: declared above the CPU generate block
reg [4:0] z3ram_base0;
reg [3:0] z3ram_base1;
reg       z3ram_ena0;
reg       z3ram_ena1;
// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026:
//  * Named block (Vivado B1 fix: local reg declaration in unnamed block).
//  * ac_toccata reset to 0: the Toccata sound card (fpga-toccata) is not
//    ported, so we must not advertise a phantom Zorro-II board to the
//    Kickstart expansion library.
//  * With fastramcfg tied to 3'b000 by the parent, ac_memcard resets to 0 -
//    the whole Zorro autoconfig block is then constant and synthesizes away,
//    which is why it is kept in source (cheap and self-disabling).
//    October 2026 (AExp fork): the parent now drives 3'b011 (8 MB Zorro II)
//    or 3'b000 from the OSM, so this block is live again.
always @(posedge clk) begin : autoconfig_blk
	reg old_uds;
	old_uds <= chip_uds;

	if (~reset | ~reset_out) begin
		// Megamiga, October 2026: Zorro II only here (fastramcfg[1:0]); the
		// Zorro III board is ac_z3 (z3ena, 68020 only)
		//ac_memcard  <= cpucfg[1] ? fastramcfg : fastramcfg[2] ? 3'd3 : {1'b0, fastramcfg[1:0]};
		ac_memcard  <= {1'b0, fastramcfg[1:0]};
		ac_z3          <= z3ena & cpucfg[1];
		ac_rtgb         <= cpucfg[1];        // Megamiga: the RTG board (68020/68040)
		rtgb_configured <= 0;
		rtgb_base       <= 9'h000;
		z3_configured  <= 0;
		z3_base        <= 8'h00;
		//ac_toccata  <= 1; // MiSTer2MEGA65 (AExp): Toccata not ported, see above
		ac_toccata  <= 0;
		toccata_base <= 8'h00; // MiSTer2MEGA65 (AExp): give the (now never written) reg a defined value
		ac_ide         <= ide_ena;
		ide_configured <= 0;
		ide_base       <= 8'h00;
		ac_eth         <= eth_ena;
		eth_configured <= 0;
		eth_base       <= 8'h00;
		z2ram_ena   <= 0;
		z3ram_ena0  <= 0;
		z3ram_ena1  <= 0;
		z3ram_base0 <= 1;
		z3ram_base1 <= 1;
	end
	else if (sel_autoconfig && ~chip_rw && ~chip_uds && old_uds) begin
		if(~ac_memcard[2] && ac_memcard[1:0]) begin
			if (chip_addr[6:1] == 6'b100100) begin // Register 0x48 - config, ZII RAM
				z2ram_ena <= 1;
				ac_memcard <= 0;
			end
		end
		// Megamiga, October 2026: the Zorro III board. Kickstart writes the
		// base address A31..A16 as a word to 0x44; a 16 MB board needs A31..A24.
		// 0x4C = "shut up" (no room, e.g. Kickstart 1.3): stays unconfigured.
		else if(ac_z3) begin
			if (chip_addr[6:1] == 6'b100010) begin // Register 0x44 - base address
				z3_base       <= cpu_dout[15:8];
				z3_configured <= 1;
				ac_z3         <= 0;
			end
			else if (chip_addr[6:1] == 6'b100110) begin // Register 0x4C - shut up
				ac_z3         <= 0;
			end
		end
		// MiSTer2MEGA65 (AExp fork), October 2026: the IDE board. Kickstart
		// writes the base address byte to 0x48 (and its low nibble to 0x4A
		// before, which we do not need: a byte write to an even address puts
		// the byte on both halves of the data bus). 0x4C = "shut up": there is
		// no room for the board, it stays unconfigured.
		// Megamiga, October 2026: the RTG board, base A31..A16 as a word to 0x44 like
		// the Zorro III RAM board; an 8 MB board needs A31..A23
		else if(ac_rtgb) begin
			if (chip_addr[6:1] == 6'b100010) begin // Register 0x44 - base address
				rtgb_base       <= cpu_dout[15:7];
				rtgb_configured <= 1;
				ac_rtgb         <= 0;
			end
			else if (chip_addr[6:1] == 6'b100110) begin // Register 0x4C - shut up
				ac_rtgb         <= 0;
			end
		end
		else if(ac_ide) begin
			if (chip_addr[6:1] == 6'b100100) begin // Register 0x48 - config
				ide_base       <= cpu_dout[15:8];
				ide_configured <= 1;
				ac_ide         <= 0;
			end
			else if (chip_addr[6:1] == 6'b100110) begin // Register 0x4C - shut up
				ac_ide         <= 0;
			end
		end
		// Megamiga, October 2026: the network card, configured like the IDE board
		else if(ac_eth) begin
			if (chip_addr[6:1] == 6'b100100) begin // Register 0x48 - config
				eth_base       <= cpu_dout[15:8];
				eth_configured <= 1;
				ac_eth         <= 0;
			end
			else if (chip_addr[6:1] == 6'b100110) begin // Register 0x4C - shut up
				ac_eth         <= 0;
			end
		end
		else if(ac_toccata) begin
			if (chip_addr[6:1] == 6'b100100) begin // Register 0x48 - config, Toccata card in ZII io space ($E90000)
				toccata_base <= cpu_dout[7:0];
				ac_toccata<=0;
			end		
		end
		else if(ac_memcard[2]) begin
			if(chip_addr[6:1] == 6'b100010) begin // Register 0x44, assign base address to ZIII RAM.
				if(~ac_memcard[1]) begin
					z3ram_base1 <= cpu_dout[15:12]; //256MB chunk
					z3ram_ena1 <= 1;
					ac_memcard <= {ac_memcard[0], ac_memcard[0], 1'b0};
				end
				else begin
					z3ram_base0 <= cpu_dout[15:11]; //128MB chunk
					z3ram_ena0 <= 1;
					ac_memcard <= 0;
				end
			end
		end
	end
end

// MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: Toccata is not ported.
// With ac_toccata now reset to 0 the original expression would wrongly
// report the card as enabled, so the output is tied to 0 instead.
//assign toccata_ena = ~ac_toccata;
assign toccata_ena = 1'b0;

endmodule
