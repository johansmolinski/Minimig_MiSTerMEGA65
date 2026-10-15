// Copyright 2008, 2009 by Jakub Bednarski
//
// This file is part of Minimig
//
// Minimig is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// Minimig is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// Action Replay III cartridge (Datel, Amiga 500). This is Jakub Bednarski's
// ActionReplay3.v of Minimig 1.x (module "ActionReplay", as in
// mist-devel/mist-board cores/minimig/rtl/minimig/ActionReplay3.v), ported to the
// Minimig-AGA clocking for Megamiga (MEGA65), October 2026:
//   * every register runs on clk with the clk7_en enable, like cart.v (the original
//     mixed clk, negedge clk, cpu_clk and posedge _cpu_as)
//   * the ROM is not uploaded through the CPU while a bootloader runs: the core's
//     loader writes it into the cartridge bank of the SDRAM, and "aron" (the
//     cartridge is plugged in and has a ROM) is an input. CPU writes to the ROM
//     area never reach the memory (the original allowed them only while "boot").
//   * the break-circuit's "previous cycle was in $000-$3FF" flag is taken at the
//     end of each bus cycle (_cpu_as rising, sampled) instead of clocking on _cpu_as
// The memory map, the registers and the freeze sequence are unchanged:
//
//   $400000-$43FFFF  256 KB ROM (selmem for reads); $400000-$400003 read: status
//                    (bits 1:0 = cause: 00 freeze button, 01 breakpoint, 11 reset),
//                    $400001 write: mode (bit 1 = breakpoint circuit on), a write to
//                    $400000/1 leaves the active state, a write to $400006/7 turns the
//                    chip RAM overlay off
//   $440000-$47FFFF  RAM, except $44F000-$44F1FF: read-only shadow of the custom
//                    registers (every write on the register bus, by CPU or DMA)
//   freeze           INT7 (level 7, autovector). The interrupt acknowledge turns on the
//                    chip RAM overlay: CPU reads of $000000-$07FFFF come from the
//                    cartridge bank at the same offset, so the level-7 vector at $7C is
//                    taken from ROM offset $7C.
//   reset            the first access to $000008 after a reset raises INT7 as well; the
//                    ROM checks the PC for Kickstart 1.3 ($FC0144) or 2.04 ($F80160), so
//                    only those Kickstarts get the reset hook.
//   breakpoints      an access to $BFE001 right after a bus cycle in $000000-$0003FF
//                    (the AR plants "TST.B $BFE001" stubs there) while mode[1] is set.
//
// Megamiga: the core plugs it in only with the 68000 (fx68k): the ROM expects the 68000's
// exception frame and its IACK cycle (int7_ack), and the 68040 core caches the Zorro II
// window and sends it to its Fast RAM port (mega65.vhd).
//
// The cartridge bank is selected by "selmem" (the bank mapper's cart input); the
// address offset into the bank is the CPU address offset ($400000 and $000000 overlay
// alike), so the bank is a plain 512 KB SRAM block from the bank mapper's point of view.

module ar3
(
	input             clk,
	input             clk7_en,
	input             reset,           // CPU reset (cart.v: cpu_rst)
	input             aron,            // the cartridge is plugged in and its ROM is loaded
	input      [23:1] cpu_address_in,  // CPU bus address (minimig_m68k_bridge address_out)
	input             _cpu_as,
	input             cpu_rd,
	input             cpu_hwr,
	input             cpu_lwr,
	input      [15:0] data_in,         // CPU write data
	input       [8:1] reg_address_in,  // register bus address
	input      [15:0] reg_data_in,     // register bus data
	input             dbr,             // DMA owns the bus
	input             freeze,          // freeze button (level)
	output     [15:0] data_out,
	output reg        int7 = 1'b0,
	output            selmem,          // cartridge bank select (bank mapper "cart")
	output            ovr              // chip RAM overlay active (reads)
);

reg        freeze_del = 1'b0;
wire       freeze_req;
wire       int7_req;
wire       int7_ack;
reg        l_int7_req = 1'b0;
reg        l_int7_ack = 1'b0;
reg        l_int7 = 1'b0;
wire       reset_req;
wire       break_req;
reg        after_reset = 1'b1;
reg  [1:0] mode = 2'b11;
reg  [1:0] status = 2'b11;
reg        ram_ovl = 1'b0;   // override chip memory and show the ROM for the int7 vector
reg        active = 1'b0;    // the cartridge is active (frozen)

wire       sel_cart;         // cartridge CPU space ($400000-$47FFFF)
wire       sel_rom;          // ROM ($400004-$43FFFF)
wire       sel_ram;          // RAM ($440000-$47FFFF) - ($44F000-$44F1FF)
wire       sel_custom;       // custom register shadow ($44F000-$44F1FF)
wire       sel_status;       // status register $400000-$400003 (repeated twice)
wire       sel_mode;         // mode register $400000/1
wire       sel_ovl;          // chip RAM overlay

wire [15:0] custom_out;
wire [15:0] status_out;

assign sel_cart   = aron & ~dbr & (cpu_address_in[23:19]==5'b0100_0);
assign sel_rom    = sel_cart & ~cpu_address_in[18] & |cpu_address_in[17:2];
assign sel_ram    = sel_cart & cpu_address_in[18] & (cpu_address_in[17:9]!=9'b001111_000);
assign sel_custom = sel_cart & cpu_address_in[18] & (cpu_address_in[17:9]==9'b001111_000) & cpu_rd;
assign sel_mode   = sel_cart & ~|cpu_address_in[18:1];
assign sel_status = sel_cart & ~|cpu_address_in[18:2] & cpu_rd;
assign sel_ovl    = aron & ram_ovl & ~dbr & (cpu_address_in[23:19]==5'b0000_0) & cpu_rd;   // Megamiga: + ~dbr (DMA reads stay in chip RAM)
// original: assign selmem = (sel_rom & boot) | ((sel_rom & cpu_rd) | sel_ram | sel_ovl);
assign selmem     = (sel_rom & cpu_rd) | sel_ram | sel_ovl;

// delayed signal for edge detection
always @(posedge clk) if (clk7_en) freeze_del <= freeze;

// freeze button has been pressed
assign freeze_req = freeze & ~freeze_del & ~active;

// int7 request
assign int7_req = aron & (freeze_req | reset_req | break_req);

// level 7 interrupt acknowledge cycle (A[23:4] all high, A[3:1] = level)
assign int7_ack = &cpu_address_in & ~_cpu_as;

// level 7 interrupt request
always @(posedge clk) begin
	if (clk7_en) begin
		     if (reset)    int7 <= 1'b0;
		else if (int7_req) int7 <= 1'b1;
		else if (int7_ack) int7 <= 1'b0;
	end
end

always @(posedge clk) begin
	if (clk7_en) begin
		l_int7_req <= int7_req;
		l_int7_ack <= int7_ack;
	end
end

always @(posedge clk) begin
	if (clk7_en) begin
		     if (reset)                 l_int7 <= 1'b0;
		else if (l_int7_req)            l_int7 <= 1'b1;
		else if (l_int7_ack && cpu_rd)  l_int7 <= 1'b0;
	end
end

// INT7 at the first access to $000008 after a reset (the ROM's reset hook)
assign reset_req = aron && cpu_address_in[23:1]==23'h04 && !_cpu_as && after_reset;

always @(posedge clk) begin
	if (clk7_en) begin
		     if (reset)    after_reset <= 1'b1;
		else if (int7_ack) after_reset <= 1'b0;
	end
end

// chip RAM overlay: set by the INT7 acknowledge, cleared by a write to $400006/7
always @(posedge clk) begin
	if (clk7_en) begin
		     if (reset)                                                    ram_ovl <= 1'b0;
		else if (l_int7 && l_int7_ack && cpu_rd)                           ram_ovl <= 1'b1;
		else if (sel_rom && (cpu_address_in[2:1]==2'b11) && (cpu_hwr|cpu_lwr)) ram_ovl <= 1'b0;
	end
end

// active (frozen): set by the INT7 acknowledge, cleared by a write to $400000/1. Without
// the CPU's FC lines code and data accesses look alike, so the ROM and RAM stay visible.
always @(posedge clk) begin
	if (clk7_en) begin
		     if (reset)                                                     active <= 1'b0;
		else if (l_int7 && l_int7_ack && cpu_rd)                            active <= 1'b1;
		else if (sel_mode && (cpu_address_in[2:1]==2'b00) && (cpu_hwr|cpu_lwr)) active <= 1'b0;
	end
end

assign ovr = aron & ram_ovl;

// mode[1] enables the breakpoint circuit
always @(posedge clk) begin
	if (clk7_en) begin
		     if (reset)              mode <= 2'b11;
		else if (sel_mode && cpu_lwr) mode <= data_in[1:0];
	end
end

always @(posedge clk) begin
	if (clk7_en) begin
		     if (reset)      status <= 2'b11;
		else if (freeze_req) status <= 2'b00;
		else if (break_req)  status <= 2'b01;
	end
end

assign status_out = sel_status ? {14'h00, status} : 16'h0000;

// custom register shadow - every write on the register bus, by the CPU or by DMA.
// Registered read like cart.v's custom mirror.
reg [15:0] custom[0:255];
reg [15:0] custom_q;
always @(posedge clk) begin
	if (clk7_en)
		custom[reg_address_in] <= reg_data_in;
	custom_q <= custom[cpu_address_in[8:1]];
end

assign custom_out = sel_custom ? custom_q : 16'h0000;

assign data_out = custom_out | status_out;

// breakpoints: an access to $BFE001 right after a bus cycle in $000000-$0003FF
// original: always @(posedge _cpu_as) cpu_address_hit <= cpu_address[23:10]==14'h00;
reg        as_del = 1'b1;
reg        cpu_address_hit = 1'b0;
reg [23:10] last_address = 14'h3FFF;
always @(posedge clk) begin
	if (clk7_en) begin
		as_del <= _cpu_as;
		if (!_cpu_as)
			last_address <= cpu_address_in[23:10];
		if (_cpu_as && !as_del)                  // end of a bus cycle
			cpu_address_hit <= (last_address == 14'h00);
	end
end

assign break_req = ~active && aron && mode[1] && cpu_address_hit &&
                   cpu_address_in == (24'hBFE001 >> 1) && !_cpu_as;

endmodule
