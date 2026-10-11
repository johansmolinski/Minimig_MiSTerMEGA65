//--------------------------------------------------------------------------//
// ap040_island.v - Megamiga, October 2026 (branch cpu040, EXPERIMENTAL)    //
//                                                                          //
// The AP68040 pipelined core on its own clock (clk_cpu), behind one        //
// request queue to the 28 MHz bus (clk). On the CPU side the core's        //
// 32-bit master port (AP040_BUS16 = 0) and the MMU's table-walk port share //
// the queue, an asynchronous FIFO (Gray-coded pointers); the bus side runs //
// each entry through ap040_bus16_adapter as TG68K-style 16-bit cycles (the //
// same bus contract as the core's own BUS16 mode), back to back, and       //
// answers a read with a toggle. A write is acknowledged to the core as     //
// soon as it is queued (WRITE_BUF = 1, a write buffer like the 68040's own //
// - this bus never answers with an error, so nothing is lost); reads queue //
// behind the writes before them, so the order of all accesses is kept. The core runs free (clock enable 1): a cache hit  //
// costs no bus clock at all, and only a miss or an uncached access waits   //
// for the 28 MHz bus.                                                      //
//                                                                          //
// Crossings (constrained in CORE.xdc by the clocks, see the cpu040 entry   //
// in doc/developers/history.md):                                           //
//   CPU -> bus  request FIFO: Gray write pointer (2 FF), entries written   //
//               before the pointer moves                                   //
//   bus -> CPU  FIFO read pointer (Gray, 2 FF)                             //
//   bus -> CPU  answer toggle (2 FF) + read data (held until next request) //
//   bus -> CPU  snoop toggle (2 FF) + address (held until the next snoop:  //
//               the chipset writes at most every 4 bus clocks)             //
//   bus -> CPU  ipl, reset, cache windows (2 FF; static or level)          //
//   CPU -> bus  cacr, vbr, nresetout, cache maintenance (2 FF; levels)     //
//                                                                          //
// The bus never answers with an error here: a timeout (the wrapper's       //
// watchdog) completes the access with all ones, as the 68020 path does,   //
// except for a table walk, which gets walker_berr.                         //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_island
#(
	// the core's speed-ups (ap040_pipe_tg68k_compat), 0 = off; MinimigAGA_TC64
	// ships all of them on but COPYBACK (which it uses only for its DDR3 board)
	parameter POST_STORES = 0,
	parameter STORE_BUF   = 0,
	parameter FWD         = 0,
	parameter RAS         = 0,
	parameter PRECISE     = 0,
	parameter SB_MMU      = 0,
	parameter BTFN        = 0,
	parameter LDX         = 0,
	// 1: writes are acknowledged when queued (see above); 0: when done
	parameter WRITE_BUF   = 0
)
(
	input             clk_cpu,
	input             clk,
	input             nreset,       // clk domain, active low

	// bus side (clk): TG68K-shaped, completed by clkena_in like TG68K
	input             clkena_in,
	input             berr,         // the wrapper's watchdog
	input      [15:0] data_in,
	output     [31:0] addr_out,
	output     [15:0] data_write,
	output            nwr,
	output            nuds,
	output            nlds,
	output      [1:0] busstate,
	output            longword,

	input       [2:0] ipl,
	input             snoop_stb,    // a chipset write to Chip RAM, this clock
	input      [31:0] snoop_addr,
	input             cache_z2_ena,
	input       [7:0] cache_z3_base0,
	input             cache_z3_ena0,

	output reg        nresetout,
	output reg [31:0] cacr_out,
	output reg [31:0] vbr_out,
	output reg        cache_maint
);

//--------------------------------------------------------------------------
// CPU side
//--------------------------------------------------------------------------
(* ASYNC_REG = "TRUE" *) reg [1:0] c_rst_s;
always @(posedge clk_cpu) c_rst_s <= {c_rst_s[0], nreset};
wire c_nreset = c_rst_s[1];

(* ASYNC_REG = "TRUE" *) reg [2:0] c_ipl_s0, c_ipl_s1;
(* ASYNC_REG = "TRUE" *) reg [1:0] c_ack_s;
(* ASYNC_REG = "TRUE" *) reg [1:0] c_snp_s;
(* ASYNC_REG = "TRUE" *) reg [9:0] c_win_s0, c_win_s1;
reg        c_ack_p, c_snp_p;
always @(posedge clk_cpu) begin
	c_ipl_s0 <= ipl;            c_ipl_s1 <= c_ipl_s0;
	c_win_s0 <= {cache_z2_ena, cache_z3_ena0, cache_z3_base0};
	c_win_s1 <= c_win_s0;
end

wire        m_req, m_write, m_instr;
wire  [1:0] m_size;
wire [31:0] m_addr, m_wdata;
wire  [2:0] m_fc;
reg         m_ack;
reg  [31:0] m_rdata;
wire        wk_req, wk_we;
wire [31:0] wk_addr, wk_wdat;
reg         wk_ack, wk_berr;
reg  [31:0] wk_data;
wire [31:0] c_cacr, c_vbr;
wire        c_resetout, c_maint;

// the request queue: 8 entries of {answer, walk, write, instr, size, fc, addr, wdata}
localparam QW = 1 + 1 + 1 + 1 + 2 + 3 + 32 + 32;
reg  [QW-1:0] q_mem [0:7];
reg     [3:0] c_wp, c_wpg;                // write pointer, binary and Gray
(* ASYNC_REG = "TRUE" *) reg [3:0] c_rpg_s0, c_rpg_s1;   // the bus side's read pointer (Gray)
reg     [3:0] b_rp, b_rpg;            // the bus side's read pointer (bus clock)
function [3:0] bin2gray(input [3:0] b); bin2gray = b ^ (b >> 1); endfunction
wire    [3:0] c_wp1  = c_wp + 4'd1;
wire          c_full = (bin2gray(c_wp) == {~c_rpg_s1[3:2], c_rpg_s1[1:0]});

reg         c_busy, c_walk;               // waiting for a read's answer
wire        b_ack_t;                      // bus side: toggles once per answer
wire [31:0] b_rdata;
wire        b_err;
reg         c_snp_stb;
reg  [31:0] c_snp_addr;
wire        b_snp_t;
wire [31:0] b_snp_addr;

// a request to queue: the walker first. An entry wants an answer unless it is a
// write and WRITE_BUF is set; then it is acknowledged when queued.
wire        n_walk  = wk_req;
wire        n_write = n_walk ? wk_we : m_write;
wire        n_ans   = !(n_write && WRITE_BUF != 0);
wire [QW-1:0] n_ent = n_walk ? {n_ans, 1'b1, wk_we, 1'b0, `AP040_SZ_L, 3'd5, wk_addr, wk_wdat}
                             : {n_ans, 1'b0, m_write, m_instr, m_size, m_fc, m_addr, m_wdata};

always @(posedge clk_cpu) begin
	c_ack_s  <= {c_ack_s[0], b_ack_t};
	c_snp_s  <= {c_snp_s[0], b_snp_t};
	c_rpg_s0 <= b_rpg;
	c_rpg_s1 <= c_rpg_s0;
	if (!c_nreset) begin
		c_busy  <= 0;
		c_walk  <= 0;
		c_wp    <= 0;
		c_wpg   <= 0;
		c_ack_p <= c_ack_s[1];
		c_snp_p <= c_snp_s[1];
		m_ack   <= 0;
		wk_ack  <= 0;
		wk_berr <= 0;
		c_snp_stb <= 0;
	end
	else begin
		m_ack     <= 0;
		wk_ack    <= 0;
		wk_berr   <= 0;
		c_snp_stb <= 0;
		c_ack_p   <= c_ack_s[1];
		c_snp_p   <= c_snp_s[1];
		if (c_snp_s[1] != c_snp_p) begin
			c_snp_stb  <= 1;
			c_snp_addr <= b_snp_addr;    // held by the bus side since its toggle
		end
		if (c_busy) begin
			if (c_ack_s[1] != c_ack_p) begin
				c_busy <= 0;
				if (c_walk) begin
					wk_data <= b_rdata;
					wk_ack  <= ~b_err;
					wk_berr <= b_err;
				end
				else begin
					m_rdata <= b_rdata;
					m_ack   <= 1;
				end
			end
		end
		// a new request; not in the clock of an answer, whose requester only
		// drops its request at the edge that consumes the answer
		else if (!m_ack && !wk_ack && !wk_berr && (wk_req || m_req) && !c_full) begin
			q_mem[c_wp[2:0]] <= n_ent;
			c_wp  <= c_wp1;
			c_wpg <= bin2gray(c_wp1);
			if (n_ans) begin
				c_busy <= 1;
				c_walk <= n_walk;
			end
			else if (n_walk) wk_ack <= 1;          // a queued write is done for the core
			else             m_ack  <= 1;
		end
	end
end

ap040_pipe_tg68k_compat #(
	.AP040_HAS_MMU(1),
	.AP040_HAS_FPU(1),
	.AP040_ENABLE_CACHE(1),
	.AP040_POST_STORES(POST_STORES),
	.AP040_FILL_CHANNEL(0),
	.AP040_BUS16(0),
	.AP040_IFP(1),
	.AP040_STORE_BUF(STORE_BUF),
	.AP040_FWD(FWD),
	.AP040_RAS(RAS),
	.AP040_PRECISE(PRECISE),
	.AP040_SB_MMU(SB_MMU),
	.AP040_BTFN(BTFN),
	.AP040_LDX(LDX)
) core (
	.clk(clk_cpu),
	.nreset(c_nreset),
	.clkena_in(1'b1),
	.cache_allow_all(1'b0),
	.cache_snoop_stb(c_snp_stb),
	.cache_snoop_addr(c_snp_addr),
	.cache_z2_ena(c_win_s1[9]),
	.cache_z3_base0(c_win_s1[7:0]),
	.cache_z3_ena0(c_win_s1[8]),
	.cache_z3_base1(4'd0),
	.cache_z3_ena1(1'b0),
	.data_in(16'd0),
	.ipl(c_ipl_s1),
	.ipl_autovector(1'b1),
	.berr(1'b0),

	.addr_out(),
	.data_write(),
	.nwr(),
	.nuds(),
	.nlds(),
	.busstate(),
	.longword(),
	.nresetout(c_resetout),
	.fc(),
	.nmi_ack_toggle(),
	.fill_ena_zorro(1'b0),
	.fill_ena_chip(1'b0),
	.fill_req(),
	.fill_addr(),
	.fill_data(128'd0),
	.fill_ack(1'b0),
	.fill_err(1'b0),
	.cache_maint_req(c_maint),
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

	.cacr_out(c_cacr),
	.vbr_out(c_vbr),
	.debug_busy(),
	.debug_fault(),
	.debug_halted(),
	.debug_status(),
	.debug_status2(),

	.m_req(m_req),
	.m_write(m_write),
	.m_instr(m_instr),
	.m_size(m_size),
	.m_addr(m_addr),
	.m_wdata(m_wdata),
	.m_fc(m_fc),
	.m_ack(m_ack),
	.m_rdata(m_rdata)
);

// CPU -> bus levels, registered once on the CPU clock before they cross
reg [31:0] c_cacr_r, c_vbr_r;
reg        c_resetout_r, c_maint_r;
always @(posedge clk_cpu) begin
	c_cacr_r     <= c_cacr;
	c_vbr_r      <= c_vbr;
	c_resetout_r <= c_resetout;
	c_maint_r    <= c_maint;
end

//--------------------------------------------------------------------------
// bus side
//--------------------------------------------------------------------------
(* ASYNC_REG = "TRUE" *) reg [3:0] b_wpg_s0, b_wpg_s1;   // the CPU side's write pointer (Gray)
(* ASYNC_REG = "TRUE" *) reg [65:0] b_lvl_s0;
reg         b_req;
reg         b_ans;                        // the entry in progress wants an answer
reg         b_walk;
reg         r_write, r_instr;
reg   [1:0] r_size;
reg  [31:0] r_addr, r_wdata;
reg   [2:0] r_fc;
reg         b_ack_tr;
reg  [31:0] b_rdata_r;
reg         b_err_r;
reg         b_snp_tr;
reg         b_snp_p;
reg  [31:0] b_snp_addr_r;
wire        a_ack;
wire [31:0] a_rdata;
wire  [3:0] b_rp1   = b_rp + 4'd1;
wire        b_empty = (b_rpg == b_wpg_s1);
wire [QW-1:0] b_ent = q_mem[b_rp[2:0]];

always @(posedge clk) begin
	b_lvl_s0 <= {c_cacr_r[31], c_cacr_r[15], c_vbr_r, c_resetout_r, c_maint_r, 30'd0};
	{cacr_out[31], cacr_out[15]} <= b_lvl_s0[65:64];
	vbr_out     <= b_lvl_s0[63:32];
	nresetout   <= b_lvl_s0[31];
	cache_maint <= b_lvl_s0[30];
	{cacr_out[30:16], cacr_out[14:0]} <= 30'd0;
end

always @(posedge clk) begin
	b_wpg_s0 <= c_wpg;
	b_wpg_s1 <= b_wpg_s0;
	if (!nreset) begin
		b_req    <= 0;
		b_rp     <= 0;
		b_rpg    <= 0;
		b_ack_tr <= 0;
		b_err_r  <= 0;
		b_snp_tr <= 0;
		b_snp_p  <= 0;
	end
	else begin
		// once per chipset write: the strobe is a level for the length of the
		// write (several bus clocks), and an even number of toggles would look
		// like none to the CPU side. Writes are at least one 7 MHz cycle (4 bus
		// clocks) apart, the CPU side takes a toggle within 3 of its clocks.
		b_snp_p <= snoop_stb;
		if (snoop_stb && (!b_snp_p || snoop_addr != b_snp_addr_r)) begin
			b_snp_tr     <= ~b_snp_tr;
			b_snp_addr_r <= snoop_addr;
		end
		if (!b_req && !b_empty) begin
			// the next entry; it was written before the pointer that shows it
			{b_ans, b_walk, r_write, r_instr, r_size, r_fc, r_addr, r_wdata} <= b_ent;
			b_rp  <= b_rp1;
			b_rpg <= bin2gray(b_rp1);
			b_req <= 1;
		end
		else if (b_req && a_ack) begin
			b_req     <= 0;
			b_rdata_r <= a_rdata;
			b_err_r   <= 0;
			if (b_ans) b_ack_tr <= ~b_ack_tr;
		end
		else if (b_req && berr) begin
			b_req     <= 0;
			b_rdata_r <= 32'hFFFFFFFF;
			b_err_r   <= b_walk;
			if (b_ans) b_ack_tr <= ~b_ack_tr;
		end
	end
end
assign b_ack_t    = b_ack_tr;
assign b_rdata    = b_rdata_r;
assign b_err      = b_err_r;
assign b_snp_t    = b_snp_tr;
assign b_snp_addr = b_snp_addr_r;

ap040_bus16_adapter bus16 (
	.clk(clk),
	.nreset(nreset),
	.clkena_in(clkena_in),
	.mem_req(b_req),
	.mem_berr(berr),
	.mem_write(r_write),
	.mem_instr(r_instr),
	.mem_size(r_size),
	.mem_addr(r_addr),
	.mem_wdata(r_wdata),
	.mem_fc(r_fc),
	.mem_ack(a_ack),
	.mem_rdata(a_rdata),
	.data_in(data_in),
	.addr_out(addr_out),
	.data_write(data_write),
	.nwr(nwr),
	.nuds(nuds),
	.nlds(nlds),
	.busstate(busstate),
	.longword(longword),
	.fc()
);

endmodule
