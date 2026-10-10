// Megamiga, October 2026: the AP68040 cores' true-dual-port RAM, renamed from
// "dpram" to "ap040_dpram" (Minimig's rtl/bram.vhd already has a dpram entity,
// with other generics). This is MinimigAGA_TC64's rtl/cpu040/dpram.v
// (github.com/ranzbak/MinimigAGA_TC64, 2b6695f, GPLv3), which replaces the
// core's simulation model: that one writes both ports in one process, which
// Vivado cannot map to block RAM (the cache tag row and the MMU's ATC then
// cost ~43k LUTs). Its notes, unchanged:
//
// ONE BEHAVIOURAL DIFFERENCE, deliberate and tested.  The original does both
// writes before both reads in a single process, so a read of a row another
// port writes in the same cycle returns the NEW data.  Two processes -- and
// any real RAMB primitive -- return the OLD data on such a collision.  The
// cache uses port B to invalidate while port A serves lookups, so a same-cycle
// collision on one row would see stale validity for a cycle.
// tb_ap040_cache_snoop.v is the test for that path and passes with this file.

module ap040_dpram #(parameter AW = 8, parameter DW = 8) (
	input clock,
	input [AW-1:0] address_a, input [DW-1:0] data_a, input wren_a, output reg [DW-1:0] q_a,
	input [AW-1:0] address_b, input [DW-1:0] data_b, input wren_b, output reg [DW-1:0] q_b
);
	(* ram_style = "block" *) reg [DW-1:0] mem [0:(1<<AW)-1];
	always @(posedge clock) begin
		if (wren_a) mem[address_a] <= data_a;
		q_a <= mem[address_a];
	end
	always @(posedge clock) begin
		if (wren_b) mem[address_b] <= data_b;
		q_b <= mem[address_b];
	end
endmodule
