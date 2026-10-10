//--------------------------------------------------------------------------//
// AP040_PIPE - MC68040-style pipelined core                                //
//                                                                          //
// ap040_pipe_bcu.v - bus controller: one memory port for IF, EA-fetch and  //
// the stores WB commits (Minimig plan M5)                                  //
//                                                                          //
// Memory side: the reference core's request port (lib/AP68040 ap040_core.v //
// mem_*): mem_req with {write, instr, size, addr, wdata, fc} held stable   //
// until mem_ack (a one-clock pulse, mem_rdata valid with it); mem_req is   //
// dropped in the ack clock, so it is low at the next enabled edge before  //
// a new request ("Changing an address while req remains asserted can make  //
// a completed request look like a duplicate transaction", ap040_core.v    //
// S_EPF_GAP).  Everything runs on the core's clock enable (the Minimig    //
// kernel is gated by clkena, ap040_tg68k_compat.v).                        //
//                                                                          //
// Clients:                                                                 //
//   stores  WB's committed store enters a posted-store FIFO (SB_N deep);   //
//           sb_full makes WB hold the store.  Stores go first: they are    //
//           older than any read outstanding.                               //
//   data    EA-fetch's read (rd_req a one-clock pulse, one outstanding),   //
//           answered by rd_ack.  It waits until every older store is in   //
//           memory: none in the FIFO, none committing in WB, none in EX    //
//           (older_st) -- so a read never needs store forwarding here.     //
//   fetch   IF's request (f_req held until f_gnt), answered by f_ack.      //
//           Lowest priority.                                               //
//                                                                          //
// mem_flt (an access error, M6) ends the transfer like an ack; rd_err /    //
// f_err flag the answer.  A store's error is fatal (st_err: nothing can be //
// restarted -- the reference's post_err).                                  //
//                                                                          //
// With translation on (tc_e), a data transfer that crosses a page boundary //
// is issued one byte at a time, most significant first, so each byte is   //
// translated through its own page (the reference's S_MRD_B/S_MWR_B,        //
// ap040_core.v; M68040UM 3.5 -- a misaligned operand's pieces are separate //
// accesses).  A fault on any byte ends the transfer with the error; the   //
// requester reports the transfer's first byte as FA, as the reference.    //
//--------------------------------------------------------------------------//

`include "ap040_pipe_defs.svh"

module ap040_pipe_bcu
	import ap040_pipe_pkg::*;
#(
	parameter SB_N = 4,
	// 0: synchronous stores -- WB holds a store until memory has taken it, so
	//    an access error on it is precise (M6; the reference core's stores are
	//    synchronous at the core too).  1: posted through the SB_N FIFO; an
	//    error on a posted store is fatal (st_err, the reference's post_err).
	parameter POST = 0,
	// (POST = 0) 1: WB's synchronous store completes the clock AFTER memory
	// acknowledges it (st_done & co. are registered).  The acknowledge is a
	// clk_114 signal registered on the edge right before the core's clk_38
	// edge (TG68K.vhd x_ack_r), and through st_done it reached WB's hold, EX's
	// stall and the whole stall chain back to IF's request -- 8.8 ns, the
	// design's one structural timing family (PLAN M11.1).  Registering it
	// ends that family at this module's flip-flops, for one clock per store.
	parameter DONE_REG = 0,
	// (POST = 0) 1: mixed stores (findings/storebuf/plan.md).  A store WB
	// hands over with sp_v is POSTED: it enters the SB_N FIFO and WB is
	// released while there is room (sb_full).  A store on st_v is SYNC, as
	// with MIX = 0, and goes out only once the FIFO is empty (the posted ones
	// are older).  The core posts only stores that cannot fault (RAM window,
	// translation off); a bus error on one is fatal (st_err).  0: the FIFO
	// is never used and everything is as before, clock for clock.
	parameter MIX = 0,
	// findings/catchup/plan.md step 1: store-to-load forwarding.  1: at the
	// data read path's lookup, the read is compared with WB's store and the
	// FIFO's, youngest first; if the youngest one it overlaps covers it
	// whole (same FC, not a locked write), fw_ok says so the next clock
	// with the read's value in fw_data, and the core may answer the slot
	// from it (rd_fwd) instead of waiting for the store to reach memory.
	parameter FWD = 0,
	// (findings/catchup/plan.md) 1: a misaligned data transfer is split
	// into aligned pieces -- a longword at 2 mod 4 into two words, any other
	// misaligned word or longword into bytes -- so each piece fits one
	// longword: the cache serves it (a misaligned read bypassed it) and a
	// store updates the line (a misaligned store cleared its row).  The
	// 68040 splits a misaligned operand into several accesses too (M68040UM
	// 7.x).  0: only a page-crossing transfer with translation on is split.
	parameter MISPLIT = 0
)
(
	input             clk,
	input             nreset,
	input             ce,

	// stores (WB commit)
	input             st_v,       // commits this clock
	input      [31:0] st_addr,
	input       [1:0] st_size,
	input      [31:0] st_data,
	input       [2:0] st_fc,
	input             st_rb,      // (a CAS/CAS2 locked write-back: carried to mem_rb for the benches)
	input             sp_v,       // (MIX) WB posts its store this enabled clock (st_* as for st_v)
	input             sp_wait,    // (MIX) WB holds a store it will post (in the FIFO from the next clock)
	output            sb_full,    // WB must hold a store (POST = 1)
	output            st_done,    // (POST = 0) WB's store completes this clock ...
	output            st_ferr,    // ... with an access error
	output            st_fatc,    // ... from the MMU
	output            sb_busy,    // a posted store is not in memory yet
	input             older_st,   // EX or WB holds a store not yet committed
	// (MIX, findings/storebuf/plan.md stage 2) the data read path's lookup
	// (lk_v: launched this enabled clock, at lk_addr/lk_size): sb_ovl, the
	// next clock, says a store in the FIFO touches one of its longwords
	// (page offset only, addr[11:2]: an alias only costs the fast answer)
	input             lk_v,
	input      [31:0] lk_addr,
	input       [1:0] lk_size,
	output reg        sb_ovl,
	output            k_fifo_o,   // (MIX) the store on the port is the FIFO's
	output            st_sync_ok, // a synchronous store completed whole this clock (no fault, not split)
	output     [31:0] dq_a_o,     // the data read slot's address and size (the core's
	output      [1:0] dq_s_o,     // answer-clock overlap check, PRECISE)
	// (FWD) forwarding: the lookup's function code, WB's store as it stands
	// (wbs_ok: one a read may take its bytes from), the verdict, the answer
	input       [2:0] lk_fc,
	// EX's BSR/JSR push (the youngest store; its address and data are EX's
	// input register, not ALU output): a return address the RTS reads
	input             exs_v,
	input      [31:0] exs_addr,
	input      [31:0] exs_data,
	input       [2:0] exs_fc,
	input             wbs_v,
	input             wbs_ok,
	input      [31:0] wbs_addr,
	input       [1:0] wbs_size,
	input      [31:0] wbs_data,
	input       [2:0] wbs_fc,
	output reg        fw_ok,
	output reg [31:0] fw_data,
	input             rd_fwd,

	// data reads (EA-fetch)
	input             rd_req,
	input      [31:0] rd_addr,
	input       [1:0] rd_size,
	input       [2:0] rd_fc,
	input             rd_lk,      // a locked RMW's read (the MMU checks W on it)
	output reg        rd_ack,
	output reg [31:0] rd_data,
	output reg        rd_err,
	// plan M14 step 3: the data read path serves the read in the slot this
	// clock (rd_fast, with its data): it is answered like a completed read
	// and never goes on the port.  Tie low without the path.
	input             rd_fast,
	input      [31:0] rd_fast_data,
	// (findings/loadstore/plan.md step 1, LDX) the answer the slot read gets
	// THIS clock if it is answered without the port -- the value rd_data
	// takes at this edge -- and whether it is (combinational, for EA-fetch's
	// late-operand dispatch: data path only, never a stall condition)
	output            rd_fast_now,
	output     [31:0] rd_data_c,

	// instruction fetch (IF)
	input             f_req,
	input      [31:0] f_addr,
	input             f_long,
	input       [2:0] f_fc,
	output            f_gnt,
	output reg        f_ack,
	output reg [31:0] f_data,
	output reg        f_err,

	output reg        st_err,

	// memory port
	output reg        mem_req,
	output reg        mem_write,
	output reg        mem_instr,
	output reg  [1:0] mem_size,
	output reg [31:0] mem_addr,
	output reg [31:0] mem_wdata,
	output reg  [2:0] mem_fc,
	output reg        mem_lock,   // the read in progress is a locked RMW's read
	output reg        mem_rb,     // the store in progress is a locked write-back (debug)
	input             mem_ack,
	input      [31:0] mem_rdata,
	input             mem_flt,
	input             mem_atc,    // (with mem_flt) the MMU's fault, not the bus's (M7)
	output reg        rd_atc,
	output reg        f_atc,
	output reg        rd_ma,      // (with rd_err) the fault hit a split read's second page
	output            st_fma,     // (with st_ferr) ... a split store's
	input             tc_e,       // TC.E: translation on (M7)
	input             tc_p        // TC.P: 8K pages
);

//--------------------------------------------------------------- store FIFO
localparam PW = (SB_N <= 2) ? 1 : (SB_N <= 4) ? 2 : 3;
reg [31:0] sb_a [0:SB_N-1];
reg [31:0] sb_d [0:SB_N-1];
reg  [1:0] sb_s [0:SB_N-1];
reg  [2:0] sb_f [0:SB_N-1];
reg        sb_b [0:SB_N-1];
reg        sb_vld [0:SB_N-1];   // (MIX) the entry holds a store not yet in memory
reg  [9:0] sb_lo [0:SB_N-1];    // ... its first and last byte's longword (addr[11:2])
reg  [9:0] sb_hi [0:SB_N-1];
reg [PW-1:0] sb_rp, sb_wp;
reg [PW:0]   sb_cnt;

assign sb_full = (POST || MIX) ? (sb_cnt == SB_N[PW:0]) : 1'b0;
assign sb_busy = (sb_cnt != 0) || (mem_req && mem_write);

//--------------------------------------------------------------- data read slot
reg        dq_v;
reg [31:0] dq_a;
reg  [1:0] dq_s;
reg  [2:0] dq_f;
reg        dq_l;

//--------------------------------------------------------------- the port
localparam [1:0] K_ST = 2'd0, K_RD = 2'd1, K_IF = 2'd2;
reg  [1:0] kind;           // what the transfer in progress is
reg        k_fifo;         // (kind K_ST) the store came from the FIFO, not from WB
assign k_fifo_o = k_fifo;
assign dq_a_o = dq_a;
assign st_sync_ok = done && (kind == K_ST) && !k_fifo && !sp_on && !mem_flt;
assign dq_s_o = dq_s;

// page-crossing split state (see below)
reg        sp_on;          // the transfer in progress is split into bytes
reg        sp_w;           // ... into two words (a longword at 2 mod 4, MISPLIT)
reg  [1:0] sp_i;           // the byte on the bus
reg  [1:0] sp_s;           // the whole transfer's size
reg [31:0] sp_a;           // ... its address
reg [31:0] sp_d;           // ... its store data
reg [23:0] sp_acc;         // the bytes read so far
wire idle    = !mem_req;         // (a new request never starts in the ack clock: one low enabled edge)
// the FIFO's oldest store (always first: it is older than WB's), or WB's
// synchronous store once the FIFO is empty
wire go_fifo = (POST || MIX) && idle && !sp_on && (sb_cnt != 0);
wire go_sync = !POST && idle && !sp_on && st_v && !((DONE_REG != 0) && st_done_q) && (sb_cnt == 0);
wire go_st   = go_fifo || go_sync;
// A read goes from the slot, never in the clock it is requested: EA-fetch
// may be sending the older instruction's store to EX in that same clock
// (an early read), and older_st sees it only once it is there.
wire go_rd   = idle && !sp_on && (sb_cnt == 0) && !st_v && !older_st && dq_v && !rd_fast && !rd_fwd;
assign rd_fast_now = (rd_fast || rd_fwd) && dq_v;
assign rd_data_c   = rd_fwd ? fw_data : rd_fast_data;
// (MIX) nor while WB holds a store it posts: it is older than any fetch,
// and a refetch after self-modifying code (WB's wb_smc, in that very
// clock) must not read the code before the store lands (smc.s)
wire go_if   = idle && !sp_on && !go_st && !go_rd && f_req && !(MIX && sp_wait);
assign f_gnt = go_if;

wire done    = mem_req && (mem_ack || mem_flt);

//--------------------------------------------------------------- page-crossing split
// (sp_on .. sp_acc: declared with the port logic above)
function automatic logic crosses(input logic e, input logic p, input logic [31:0] a, input logic [1:0] sz);
	logic [13:0] off;
	off = {1'b0, p & a[12], a[11:0]} + ((sz == SZ_L) ? 14'd4 : (sz == SZ_W) ? 14'd2 : 14'd1);
	return e && (sz != SZ_B) && (p ? (off > 14'h2000) : (off > 14'h1000));
endfunction
function automatic logic [7:0] sp_byte(input logic [31:0] d, input logic [1:0] sz, input logic [1:0] i);
	if (sz == SZ_W) return i[0] ? d[7:0] : d[15:8];
	case (i)
		2'd0: return d[31:24];
		2'd1: return d[23:16];
		2'd2: return d[15:8];
		default: return d[7:0];
	endcase
endfunction
// (MISPLIT) misaligned: split; a longword at 2 mod 4 into words
function automatic logic misal(input logic [31:0] a, input logic [1:0] sz);
	return (MISPLIT != 0) && ((sz == SZ_L && a[1:0] != 2'b00) || (sz == SZ_W && a[0]));
endfunction
function automatic logic wsplit(input logic [31:0] a, input logic [1:0] sz);
	return (MISPLIT != 0) && sz == SZ_L && a[1:0] == 2'b10;
endfunction
wire [1:0] sp_nm1 = (sp_w || sp_s != SZ_L) ? 2'd1 : 2'd3;    // the last piece's index
wire       sp_fin = !sp_on || mem_flt || (sp_i == sp_nm1);   // the transfer ends with this answer
wire       go_sp  = idle && sp_on;                           // the next byte (first priority)
wire [31:0] sp_rd = sp_w ? {sp_acc[15:0], mem_rdata[15:0]} :
                   (sp_s == SZ_L) ? {sp_acc, mem_rdata[7:0]} : {16'd0, sp_acc[7:0], mem_rdata[7:0]};

wire   st_done_c = !POST && done && (kind == K_ST) && sp_fin && !k_fifo;
wire   st_fma_c, st_ferr_c, st_fatc_c;
reg    st_done_q, st_ferr_q, st_fatc_q, st_fma_q;
assign st_done = (DONE_REG != 0) ? st_done_q : st_done_c;
// SSW.MA: the fault is on the far side of the page boundary (the reference's aer_ma)
wire   sp_ma   = sp_on && (tc_p ? (mem_addr[31:13] != sp_a[31:13]) : (mem_addr[31:12] != sp_a[31:12]));
assign st_fma_c  = sp_ma;
assign st_ferr_c = st_done_c && mem_flt;
assign st_fatc_c = st_ferr_c && mem_atc;
assign st_fma  = (DONE_REG != 0) ? st_fma_q  : st_fma_c;
assign st_ferr = (DONE_REG != 0) ? st_ferr_q : st_ferr_c;
assign st_fatc = (DONE_REG != 0) ? st_fatc_q : st_fatc_c;
always @(posedge clk)
	if (!nreset) begin st_done_q <= 1'b0; st_ferr_q <= 1'b0; st_fatc_q <= 1'b0; st_fma_q <= 1'b0; end
	else if (ce) begin
		st_done_q <= st_done_c; st_ferr_q <= st_ferr_c; st_fatc_q <= st_fatc_c; st_fma_q <= st_fma_c;
	end

// the longword (addr[11:2]) of an access's last byte
function automatic logic [9:0] lw_last(input logic [31:0] a, input logic [1:0] sz);
	logic [31:0] e;
	e = a + ((sz == SZ_L) ? 32'd3 : (sz == SZ_W) ? 32'd1 : 32'd0);
	return e[11:2];
endfunction
wire [9:0] lk_lo = lk_addr[11:2];
wire [9:0] lk_hi = lw_last(lk_addr, lk_size);
reg        lk_hit;
integer    q;
always @* begin
	lk_hit = 1'b0;
	for (q = 0; q < SB_N; q = q + 1)
		if (sb_vld[q] && (sb_lo[q] == lk_lo || sb_lo[q] == lk_hi || sb_hi[q] == lk_lo || sb_hi[q] == lk_hi))
			lk_hit = 1'b1;
end

// (FWD) byte ranges: does [a2, a2+n2) meet [a1, a1+n1), does it lie inside
// it, and the read's value taken out of the store's bytes (right-aligned)
function automatic logic [2:0] nby(input logic [1:0] sz);
	return (sz == SZ_L) ? 3'd4 : (sz == SZ_W) ? 3'd2 : 3'd1;
endfunction
// (timing: 12-bit arithmetic on the page offset, never a 33-bit add)
// f_ovl: may the two meet -- a longword index (addr[11:2]) of one equals one
// of the other's; conservative (an alias in another page counts)
function automatic logic [11:0] f_end(input logic [31:0] a, input logic [1:0] sz);
	return a[11:0] + ((sz == SZ_L) ? 12'd3 : (sz == SZ_W) ? 12'd1 : 12'd0);
endfunction
function automatic logic f_ovl(input logic [31:0] a1, input logic [1:0] s1, input logic [31:0] a2, input logic [1:0] s2);
	logic [11:0] e1, e2;
	e1 = f_end(a1, s1); e2 = f_end(a2, s2);
	return (a1[11:2] == a2[11:2]) || (a1[11:2] == e2[11:2]) || (e1[11:2] == a2[11:2]) || (e1[11:2] == e2[11:2]);
endfunction
// f_cov: does [a2, a2+n2) lie inside [a1, a1+n1) -- exact: the same page,
// neither crossing it, and the byte range inside
function automatic logic f_cov(input logic [31:0] a1, input logic [1:0] s1, input logic [31:0] a2, input logic [1:0] s2);
	logic [11:0] e1, e2;
	e1 = f_end(a1, s1); e2 = f_end(a2, s2);
	return (a1[31:12] == a2[31:12]) && (e1 >= a1[11:0]) && (e2 >= a2[11:0]) &&
	       (a2[11:0] >= a1[11:0]) && (e2 <= e1);
endfunction
function automatic logic [31:0] f_ext(input logic [31:0] d, input logic [31:0] a1, input logic [1:0] s1,
                                      input logic [31:0] a2, input logic [1:0] s2);
	logic [31:0] r; logic [2:0] n1, n2; logic [1:0] off; integer j;
	n1 = nby(s1); n2 = nby(s2); off = a2[1:0] - a1[1:0]; r = 32'd0;
	// (a byte outside the store reads 0: never used when the store covers
	// the read, and no X in simulation when it does not)
	for (j = 0; j < 4; j = j + 1)
		if (j < n2 && off + j < n1) r[8 * (n2 - 1 - j) +: 8] = d[8 * (n1 - 1 - (off + j)) +: 8];
	return r;
endfunction
reg        fw_hit;
reg [31:0] fw_d;
reg        fw_done;
reg [PW-1:0] fw_i;
integer    qq;
always @* begin
	fw_hit = 1'b0; fw_d = 32'd0; fw_done = 1'b0; fw_i = '0;
	// EX's push is the youngest
	if (exs_v && f_ovl(exs_addr, SZ_L, lk_addr, lk_size)) begin
		fw_done = 1'b1;
		if (exs_fc == lk_fc && f_cov(exs_addr, SZ_L, lk_addr, lk_size)) begin
			fw_hit = 1'b1; fw_d = f_ext(exs_data, exs_addr, SZ_L, lk_addr, lk_size);
		end
	end
	// then WB's store
	if (!fw_done && wbs_v && f_ovl(wbs_addr, wbs_size, lk_addr, lk_size)) begin
		fw_done = 1'b1;
		if (wbs_ok && wbs_fc == lk_fc && f_cov(wbs_addr, wbs_size, lk_addr, lk_size)) begin
			fw_hit = 1'b1; fw_d = f_ext(wbs_data, wbs_addr, wbs_size, lk_addr, lk_size);
		end
	end
	// then the FIFO, newest entry first
	for (qq = 1; qq <= SB_N; qq = qq + 1) begin
		fw_i = sb_wp - qq[PW-1:0];
		if (!fw_done && sb_vld[fw_i] && f_ovl(sb_a[fw_i], sb_s[fw_i], lk_addr, lk_size)) begin
			fw_done = 1'b1;
			if (sb_f[fw_i] == lk_fc && f_cov(sb_a[fw_i], sb_s[fw_i], lk_addr, lk_size)) begin
				fw_hit = 1'b1; fw_d = f_ext(sb_d[fw_i], sb_a[fw_i], sb_s[fw_i], lk_addr, lk_size);
			end
		end
	end
end

integer k;
always @(posedge clk) begin
	if (!nreset) begin
		mem_req <= 1'b0; mem_write <= 1'b0; mem_instr <= 1'b0; mem_size <= SZ_L;
		mem_addr <= 32'd0; mem_wdata <= 32'd0; mem_fc <= 3'd5;
		kind <= K_ST; k_fifo <= 1'b0;
		sb_rp <= '0; sb_wp <= '0; sb_cnt <= '0;
		dq_v <= 1'b0; dq_a <= 32'd0; dq_s <= SZ_L; dq_f <= 3'd5; dq_l <= 1'b0; mem_lock <= 1'b0;
		rd_ack <= 1'b0; rd_data <= 32'd0; rd_err <= 1'b0; rd_atc <= 1'b0; f_atc <= 1'b0; rd_ma <= 1'b0;
		f_ack <= 1'b0; f_data <= 32'd0; f_err <= 1'b0;
		st_err <= 1'b0;
		for (k = 0; k < SB_N; k = k + 1) begin
			sb_a[k] <= 32'd0; sb_d[k] <= 32'd0; sb_s[k] <= SZ_L; sb_f[k] <= 3'd5; sb_b[k] <= 1'b0;
			sb_vld[k] <= 1'b0; sb_lo[k] <= 10'd0; sb_hi[k] <= 10'd0;
		end
		sb_ovl <= 1'b0; fw_ok <= 1'b0; fw_data <= 32'd0;
		mem_rb <= 1'b0;
		sp_on <= 1'b0; sp_w <= 1'b0; sp_i <= 2'd0; sp_s <= SZ_L; sp_a <= 32'd0; sp_d <= 32'd0; sp_acc <= 24'd0;
	end else if (ce) begin
		rd_ack <= 1'b0; f_ack <= 1'b0;
		// the lookup's overlap verdict, against the FIFO before this edge (a
		// store WB posts at this edge is older than the read, but it was in
		// WB in this clock, and the core's st_quiet refuses the fast answer
		// for that)
		if (lk_v) sb_ovl <= MIX && lk_hit;
		if (lk_v) begin fw_ok <= (FWD != 0) && fw_hit; fw_data <= fw_d; end
		// a store committed by WB (posted)
		if ((POST && st_v) || (MIX && sp_v)) begin
			sb_vld[sb_wp] <= 1'b1; sb_lo[sb_wp] <= st_addr[11:2]; sb_hi[sb_wp] <= lw_last(st_addr, st_size);
			sb_a[sb_wp] <= st_addr; sb_d[sb_wp] <= st_data; sb_s[sb_wp] <= st_size; sb_f[sb_wp] <= st_fc;
			sb_b[sb_wp] <= st_rb;
			sb_wp <= sb_wp + 1'b1;
		end
		if (POST || MIX)
			sb_cnt <= sb_cnt + (((POST && st_v) || (MIX && sp_v)) ? 1'b1 : 1'b0)
			                 - ((done && kind == K_ST && sp_fin && k_fifo) ? 1'b1 : 1'b0);
		// a data read request waits in the slot
		if (rd_req) begin dq_v <= 1'b1; dq_a <= rd_addr; dq_s <= rd_size; dq_f <= rd_fc; dq_l <= rd_lk; end
		// ... unless the data read path answers it (M14 step 3)
		if ((rd_fast || rd_fwd) && dq_v) begin
			dq_v <= 1'b0;
			rd_ack <= 1'b1; rd_data <= rd_fwd ? fw_data : rd_fast_data;
			rd_err <= 1'b0; rd_atc <= 1'b0; rd_ma <= 1'b0;
		end
		// completion
		if (done && !sp_fin) begin
			// a byte of a split transfer: on to the next
			mem_req <= 1'b0;
			sp_i <= sp_i + 2'd1;
			sp_acc <= sp_w ? {8'd0, mem_rdata[15:0]} : {sp_acc[15:0], mem_rdata[7:0]};
		end else if (done) begin
			mem_req <= 1'b0;
			sp_on <= 1'b0;
			case (kind)
				K_ST: begin
					if (k_fifo) begin
						sb_rp <= sb_rp + 1'b1;   // (sp_fin: the whole store)
						sb_vld[sb_rp] <= 1'b0;
						if (mem_flt) st_err <= 1'b1;
					end
				end
				K_RD: begin rd_ack <= 1'b1; rd_data <= sp_on ? sp_rd : mem_rdata; rd_err <= mem_flt; rd_atc <= mem_flt && mem_atc; rd_ma <= mem_flt && sp_ma; end
				default: begin                // a word fetch answers right-aligned: IF wants it on top
					f_ack <= 1'b1; f_err <= mem_flt; f_atc <= mem_flt && mem_atc;
					f_data <= (mem_size == SZ_W) ? {mem_rdata[15:0], 16'h4E71} : mem_rdata;
				end
			endcase
		end
		// a new transfer
		if (go_sp) begin
			// the next piece of a split transfer (write/fc/kind stand)
			mem_req <= 1'b1; mem_size <= sp_w ? SZ_W : SZ_B;
			mem_addr <= sp_w ? sp_a + {29'd0, sp_i, 1'b0} : sp_a + {30'd0, sp_i};
			mem_wdata <= sp_w ? {16'd0, sp_i[0] ? sp_d[15:0] : sp_d[31:16]} : {24'd0, sp_byte(sp_d, sp_s, sp_i)};
		end else if (go_st) begin : g_st
			// the store WB holds (stable until st_done), or the FIFO's oldest
			logic [31:0] a, d; logic [1:0] z;
			a = go_fifo ? sb_a[sb_rp] : st_addr; d = go_fifo ? sb_d[sb_rp] : st_data; z = go_fifo ? sb_s[sb_rp] : st_size;
			mem_req <= 1'b1; kind <= K_ST; k_fifo <= go_fifo; mem_write <= 1'b1; mem_instr <= 1'b0; mem_lock <= 1'b0;
			mem_fc <= go_fifo ? sb_f[sb_rp] : st_fc; mem_rb <= go_fifo ? sb_b[sb_rp] : st_rb;
			if (crosses(tc_e, tc_p, a, z) || misal(a, z)) begin
				sp_on <= 1'b1; sp_w <= wsplit(a, z); sp_i <= 2'd0; sp_s <= z; sp_a <= a; sp_d <= d;
				mem_addr <= a;
				if (wsplit(a, z)) begin mem_size <= SZ_W; mem_wdata <= {16'd0, d[31:16]}; end
				else begin mem_size <= SZ_B; mem_wdata <= {24'd0, sp_byte(d, z, 2'd0)}; end
			end else begin
				mem_addr <= a; mem_size <= z; mem_wdata <= d;
			end
		end else if (go_rd) begin
			mem_req <= 1'b1; kind <= K_RD; mem_write <= 1'b0; mem_instr <= 1'b0;
			mem_addr <= dq_a; mem_fc <= dq_f; mem_lock <= dq_l;
			if (crosses(tc_e, tc_p, dq_a, dq_s) || misal(dq_a, dq_s)) begin
				sp_on <= 1'b1; sp_w <= wsplit(dq_a, dq_s); sp_i <= 2'd0; sp_s <= dq_s; sp_a <= dq_a; sp_acc <= 24'd0;
				mem_size <= wsplit(dq_a, dq_s) ? SZ_W : SZ_B;
			end else mem_size <= dq_s;
			dq_v <= 1'b0;
		end else if (go_if) begin
			mem_req <= 1'b1; kind <= K_IF; mem_write <= 1'b0; mem_instr <= 1'b1; mem_lock <= 1'b0;
			mem_addr <= f_addr; mem_size <= f_long ? SZ_L : SZ_W; mem_fc <= f_fc;
		end
	end
end

endmodule
