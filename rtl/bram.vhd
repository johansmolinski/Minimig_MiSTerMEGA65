--------------------------------------------------------------
-- Block RAM wrappers (spram, spram_sz, dpram, dpram_dif, dpram_difclk)
--
-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026:
-- The original file wrapped the Altera altsyncram megafunction
-- (library altera_mf), which does not exist in Vivado/Xilinx. All five
-- wrappers have been rewritten as portable, behavioral VHDL from which
-- Vivado infers block RAM (UG901 templates; see also the proven
-- M2M/vhdl/tdp_ram.vhd and 2port2clk_ram.vhd patterns in the framework).
-- Entity names, generics and ports (including their default values) are
-- IDENTICAL to the original, so no caller changes are needed. The
-- original altsyncram generic/port maps are kept in comments inside each
-- architecture for reference.
--
-- Consumers in the AExp Vivado file list: ONLY rtl/ide.v, which
-- instantiates "dpram #(12,16)" twice (io_buf0/io_buf1, equal widths on
-- both ports, single clock, enable_*/cs_* left unconnected = defaults).
-- rtl/cpu_cache_new.v also used dpram but is NOT part of the AExp build.
-- spram, spram_sz and dpram_difclk are unused in the AExp build; they
-- were ported anyway for completeness.
--
-- Replicated altsyncram semantics (identical for all wrappers):
--   * READ LATENCY = 1 enabled clock edge. The original used a
--     registered read address with UNREGISTERED output data
--     (outdata_reg = UNREGISTERED), i.e. q changes only as a consequence
--     of a clock edge and is stable in between. Modeled here as a
--     synchronous read into a data register (the BRAM output latch).
--   * Same-port read-during-write = "NEW_DATA_NO_NBE_READ": a port that
--     writes also presents the NEW (just written) data on its q output
--     after that clock edge ("write first"). Modeled with the UG901
--     shared-variable write-first template (write before read).
--   * Mixed-port (port A vs port B) read-during-write was left at the
--     altsyncram default "DONT_CARE" by the original; the replacement
--     makes no guarantee either (in simulation the process execution
--     order decides; in hardware Xilinx TDP BRAM collision rules apply).
--   * Memory powers up as all zeros (power_up_uninitialized = FALSE);
--     replicated with initial values, honored by Vivado.
--   * mem_init_file (.mif) and mem_name (Altera In-System Memory Content
--     Editor) are NOT supported in this port; no caller in the Minimig
--     code base uses them. Guarded by elaboration-time assertions.
--------------------------------------------------------------


--------------------------------------------------------------
-- Single port Block RAM
--------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: altera_mf removed
--LIBRARY altera_mf;
--USE altera_mf.altera_mf_components.all;

ENTITY spram IS
	generic (
		addr_width    : integer := 8;
		data_width    : integer := 8;
		mem_init_file : string := " ";
		mem_name      : string := "MEM" -- for InSystem Memory content editor.
	);
	PORT
	(
		clock   : in  STD_LOGIC;
		address : in  STD_LOGIC_VECTOR (addr_width-1 DOWNTO 0);
		data    : in  STD_LOGIC_VECTOR (data_width-1 DOWNTO 0) := (others => '0');
		enable  : in  STD_LOGIC := '1';
		wren    : in  STD_LOGIC := '0';
		q       : out STD_LOGIC_VECTOR (data_width-1 DOWNTO 0);
		cs      : in  std_logic := '1'
	);
END spram;


ARCHITECTURE SYN OF spram IS
BEGIN
	spram_sz : work.spram_sz
	generic map(addr_width, data_width, 2**addr_width, mem_init_file, mem_name)
	port map(clock,address,data,enable,wren,q,cs);
END SYN;


--------------------------------------------------------------
-- Single port Block RAM with specific size
--
-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: rewritten as inferred
-- BRAM (UG901 single-port write-first template).
-- Latency: read data valid 1 clock after the address is presented.
-- Same-port read-during-write: NEW data (write first), as the original
-- read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ".
-- NOTE: like the original (clock_enable_input_a => "BYPASS" and clocken0
-- left unconnected), the "enable" port is IGNORED; it only exists for
-- interface compatibility. "cs" gates writes and forces q to all ones.
--------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: altera_mf removed
--LIBRARY altera_mf;
--USE altera_mf.altera_mf_components.all;

ENTITY spram_sz IS
	generic (
		addr_width    : integer := 8;
		data_width    : integer := 8;
		numwords      : integer := 2**8;
		mem_init_file : string := " ";
		mem_name      : string := "MEM" -- for InSystem Memory content editor.
	);
	PORT
	(
		clock   : in  STD_LOGIC;
		address : in  STD_LOGIC_VECTOR (addr_width-1 DOWNTO 0);
		data    : in  STD_LOGIC_VECTOR (data_width-1 DOWNTO 0) := (others => '0');
		enable  : in  STD_LOGIC := '1';
		wren    : in  STD_LOGIC := '0';
		q       : out STD_LOGIC_VECTOR (data_width-1 DOWNTO 0);
		cs      : in  std_logic := '1'
	);
END ENTITY;

ARCHITECTURE SYN OF spram_sz IS
	type ram_t is array (0 to numwords-1) of std_logic_vector(data_width-1 downto 0);
	signal ram : ram_t := (others => (others => '0'));   -- power-up: all zeros
	signal q0  : std_logic_vector((data_width - 1) downto 0) := (others => '0');
BEGIN
	-- .mif initialization files are not supported in the Vivado port
	-- (no Minimig caller uses them)
	assert (mem_init_file = " ") or (mem_init_file = "") or (mem_init_file = "UNUSED")
		report "spram_sz (AExp Vivado port): mem_init_file is not supported"
		severity failure;

	q<= q0 when cs = '1' else (others => '1');

	-- 1-cycle read latency, write-first on the (single) port
	process (clock)
	begin
		if rising_edge(clock) then
			if to_integer(unsigned(address)) < numwords then
				if wren = '1' and cs = '1' then
					ram(to_integer(unsigned(address))) <= data;
					q0 <= data;   -- NEW_DATA_NO_NBE_READ (write first)
				else
					q0 <= ram(to_integer(unsigned(address)));
				end if;
			end if;
		end if;
	end process;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: original Altera
-- altsyncram instantiation, kept for reference:
--
--	altsyncram_component : altsyncram
--	GENERIC MAP (
--		clock_enable_input_a => "BYPASS",
--		clock_enable_output_a => "BYPASS",
--		intended_device_family => "Cyclone V",
--		lpm_hint => "ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME="&mem_name,
--		lpm_type => "altsyncram",
--		numwords_a => numwords,
--		operation_mode => "SINGLE_PORT",
--		outdata_aclr_a => "NONE",
--		outdata_reg_a => "UNREGISTERED",
--		power_up_uninitialized => "FALSE",
--		read_during_write_mode_port_a => "NEW_DATA_NO_NBE_READ",
--		init_file => mem_init_file,
--		widthad_a => addr_width,
--		width_a => data_width,
--		width_byteena_a => 1
--	)
--	PORT MAP (
--		address_a => address,
--		clock0 => clock,
--		data_a => data,
--		wren_a => wren and cs,
--		q_a => q0
--	);

END SYN;

--------------------------------------------------------------
-- Dual port Block RAM same parameters on both ports
--------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.all;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: altera_mf removed
--LIBRARY altera_mf;
--USE altera_mf.altera_mf_components.all;

entity dpram is
	generic (
		addr_width    : integer := 8;
		data_width    : integer := 8;
		mem_init_file : string := " "
	);
	PORT
	(
		clock			: in  STD_LOGIC;

		address_a	: in  STD_LOGIC_VECTOR (addr_width-1 DOWNTO 0);
		data_a		: in  STD_LOGIC_VECTOR (data_width-1 DOWNTO 0) := (others => '0');
		enable_a		: in  STD_LOGIC := '1';
		wren_a		: in  STD_LOGIC := '0';
		q_a			: out STD_LOGIC_VECTOR (data_width-1 DOWNTO 0);
		cs_a        : in  std_logic := '1';

		address_b	: in  STD_LOGIC_VECTOR (addr_width-1 DOWNTO 0) := (others => '0');
		data_b		: in  STD_LOGIC_VECTOR (data_width-1 DOWNTO 0) := (others => '0');
		enable_b		: in  STD_LOGIC := '1';
		wren_b		: in  STD_LOGIC := '0';
		q_b			: out STD_LOGIC_VECTOR (data_width-1 DOWNTO 0);
		cs_b        : in  std_logic := '1'
	);
end entity;


ARCHITECTURE SYN OF dpram IS
BEGIN
	ram : work.dpram_dif generic map(addr_width,data_width,addr_width,data_width,mem_init_file)
	port map(clock,address_a,data_a,enable_a,wren_a,q_a,cs_a,address_b,data_b,enable_b,wren_b,q_b,cs_b);
END SYN;

--------------------------------------------------------------
-- Dual port Block RAM different parameters on ports
--
-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: rewritten as inferred
-- true dual port BRAM (UG901 shared-variable template, write-first).
-- Latency per port: read data valid 1 enabled clock edge after the
-- address is presented (enable_a/enable_b replicate the original
-- clocken0/clocken1, which gated the whole port input stage).
-- Same-port read-during-write: NEW data ("NEW_DATA_NO_NBE_READ").
-- Mixed-port read-during-write: undefined (original: altsyncram default
-- "DONT_CARE").
-- RESTRICTION: only equal port geometries are supported
-- (addr_width_a = addr_width_b, data_width_a = data_width_b). Altera's
-- altsyncram supported mixed-width ports, but the only AExp consumer,
-- rtl/ide.v via entity dpram, uses 12/16 on both ports. Guarded by an
-- elaboration-time assertion.
--------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: altera_mf removed
--LIBRARY altera_mf;
--USE altera_mf.altera_mf_components.all;

entity dpram_dif is
	generic (
		addr_width_a  : integer := 8;
		data_width_a  : integer := 8;
		addr_width_b  : integer := 8;
		data_width_b  : integer := 8;
		mem_init_file : string := " "
	);
	PORT
	(
		clock			: in  STD_LOGIC;

		address_a	: in  STD_LOGIC_VECTOR (addr_width_a-1 DOWNTO 0);
		data_a		: in  STD_LOGIC_VECTOR (data_width_a-1 DOWNTO 0) := (others => '0');
		enable_a		: in  STD_LOGIC := '1';
		wren_a		: in  STD_LOGIC := '0';
		q_a			: out STD_LOGIC_VECTOR (data_width_a-1 DOWNTO 0);
		cs_a        : in  std_logic := '1';

		address_b	: in  STD_LOGIC_VECTOR (addr_width_b-1 DOWNTO 0) := (others => '0');
		data_b		: in  STD_LOGIC_VECTOR (data_width_b-1 DOWNTO 0) := (others => '0');
		enable_b		: in  STD_LOGIC := '1';
		wren_b		: in  STD_LOGIC := '0';
		q_b			: out STD_LOGIC_VECTOR (data_width_b-1 DOWNTO 0);
		cs_b        : in  std_logic := '1'
	);
end entity;


ARCHITECTURE SYN OF dpram_dif IS

	type ram_t is array (0 to 2**addr_width_a - 1) of std_logic_vector(data_width_a - 1 downto 0);
	shared variable ram : ram_t := (others => (others => '0'));   -- power-up: all zeros

	signal q0 : std_logic_vector((data_width_a - 1) downto 0) := (others => '0');
	signal q1 : std_logic_vector((data_width_b - 1) downto 0) := (others => '0');

BEGIN
	assert (addr_width_a = addr_width_b) and (data_width_a = data_width_b)
		report "dpram_dif (AExp Vivado port): mixed-width ports are not supported"
		severity failure;

	-- .mif initialization files are not supported in the Vivado port
	-- (no Minimig caller uses them)
	assert (mem_init_file = " ") or (mem_init_file = "") or (mem_init_file = "UNUSED")
		report "dpram_dif (AExp Vivado port): mem_init_file is not supported"
		severity failure;

	q_a<= q0 when cs_a = '1' else (others => '1');
	q_b<= q1 when cs_b = '1' else (others => '1');

	-- port A: 1-cycle read latency, write-first
	port_a : process (clock)
	begin
		if rising_edge(clock) then
			if enable_a = '1' then
				if wren_a = '1' and cs_a = '1' then
					ram(to_integer(unsigned(address_a))) := data_a;
				end if;
				q0 <= ram(to_integer(unsigned(address_a)));   -- after write: NEW data
			end if;
		end if;
	end process port_a;

	-- port B: 1-cycle read latency, write-first
	port_b : process (clock)
	begin
		if rising_edge(clock) then
			if enable_b = '1' then
				if wren_b = '1' and cs_b = '1' then
					ram(to_integer(unsigned(address_b))) := data_b;
				end if;
				q1 <= ram(to_integer(unsigned(address_b)));   -- after write: NEW data
			end if;
		end if;
	end process port_b;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: original Altera
-- altsyncram instantiation, kept for reference:
--
--	altsyncram_component : altsyncram
--	GENERIC MAP (
--		address_reg_b => "CLOCK1",
--		clock_enable_input_a => "NORMAL",
--		clock_enable_input_b => "NORMAL",
--		clock_enable_output_a => "BYPASS",
--		clock_enable_output_b => "BYPASS",
--		indata_reg_b => "CLOCK1",
--		intended_device_family => "Cyclone V",
--		lpm_type => "altsyncram",
--		numwords_a => 2**addr_width_a,
--		numwords_b => 2**addr_width_b,
--		operation_mode => "BIDIR_DUAL_PORT",
--		outdata_aclr_a => "NONE",
--		outdata_aclr_b => "NONE",
--		outdata_reg_a => "UNREGISTERED",
--		outdata_reg_b => "UNREGISTERED",
--		power_up_uninitialized => "FALSE",
--		read_during_write_mode_port_a => "NEW_DATA_NO_NBE_READ",
--		read_during_write_mode_port_b => "NEW_DATA_NO_NBE_READ",
--		init_file => mem_init_file,
--		widthad_a => addr_width_a,
--		widthad_b => addr_width_b,
--		width_a => data_width_a,
--		width_b => data_width_b,
--		width_byteena_a => 1,
--		width_byteena_b => 1,
--		wrcontrol_wraddress_reg_b => "CLOCK1"
--	)
--	PORT MAP (
--		address_a => address_a,
--		address_b => address_b,
--		clock0 => clock,
--		clock1 => clock,
--		clocken0 => enable_a,
--		clocken1 => enable_b,
--		data_a => data_a,
--		data_b => data_b,
--		wren_a => wren_a and cs_a,
--		wren_b => wren_b and cs_b,
--		q_a => q0,
--		q_b => q1
--	);

END SYN;


--------------------------------------------------------------
-- Dual port Block RAM different parameters and clocks on ports
--
-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: rewritten as inferred
-- true dual port, dual clock BRAM (UG901 shared-variable template,
-- write-first). Same semantics as dpram_dif above, but port A runs on
-- clock0 and port B on clock1. NOT instantiated by any file in the AExp
-- Vivado file list; ported for completeness. Same equal-geometry
-- restriction as dpram_dif.
--------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: altera_mf removed
--LIBRARY altera_mf;
--USE altera_mf.altera_mf_components.all;

entity dpram_difclk is
	generic (
		addr_width_a  : integer := 8;
		data_width_a  : integer := 8;
		addr_width_b  : integer := 8;
		data_width_b  : integer := 8;
		mem_init_file : string := " "
	);
	PORT
	(
		clock0		: in  STD_LOGIC;
		clock1		: in  STD_LOGIC;

		address_a	: in  STD_LOGIC_VECTOR (addr_width_a-1 DOWNTO 0);
		data_a		: in  STD_LOGIC_VECTOR (data_width_a-1 DOWNTO 0) := (others => '0');
		enable_a		: in  STD_LOGIC := '1';
		wren_a		: in  STD_LOGIC := '0';
		q_a			: out STD_LOGIC_VECTOR (data_width_a-1 DOWNTO 0);
		cs_a        : in  std_logic := '1';

		address_b	: in  STD_LOGIC_VECTOR (addr_width_b-1 DOWNTO 0) := (others => '0');
		data_b		: in  STD_LOGIC_VECTOR (data_width_b-1 DOWNTO 0) := (others => '0');
		enable_b		: in  STD_LOGIC := '1';
		wren_b		: in  STD_LOGIC := '0';
		q_b			: out STD_LOGIC_VECTOR (data_width_b-1 DOWNTO 0);
		cs_b        : in  std_logic := '1'
	);
end entity;


ARCHITECTURE SYN OF dpram_difclk IS

	type ram_t is array (0 to 2**addr_width_a - 1) of std_logic_vector(data_width_a - 1 downto 0);
	shared variable ram : ram_t := (others => (others => '0'));   -- power-up: all zeros

	signal q0 : std_logic_vector((data_width_a - 1) downto 0) := (others => '0');
	signal q1 : std_logic_vector((data_width_b - 1) downto 0) := (others => '0');

BEGIN
	assert (addr_width_a = addr_width_b) and (data_width_a = data_width_b)
		report "dpram_difclk (AExp Vivado port): mixed-width ports are not supported"
		severity failure;

	-- .mif initialization files are not supported in the Vivado port
	-- (no Minimig caller uses them)
	assert (mem_init_file = " ") or (mem_init_file = "") or (mem_init_file = "UNUSED")
		report "dpram_difclk (AExp Vivado port): mem_init_file is not supported"
		severity failure;

	q_a<= q0 when cs_a = '1' else (others => '1');
	q_b<= q1 when cs_b = '1' else (others => '1');

	-- port A on clock0: 1-cycle read latency, write-first
	port_a : process (clock0)
	begin
		if rising_edge(clock0) then
			if enable_a = '1' then
				if wren_a = '1' and cs_a = '1' then
					ram(to_integer(unsigned(address_a))) := data_a;
				end if;
				q0 <= ram(to_integer(unsigned(address_a)));   -- after write: NEW data
			end if;
		end if;
	end process port_a;

	-- port B on clock1: 1-cycle read latency, write-first
	port_b : process (clock1)
	begin
		if rising_edge(clock1) then
			if enable_b = '1' then
				if wren_b = '1' and cs_b = '1' then
					ram(to_integer(unsigned(address_b))) := data_b;
				end if;
				q1 <= ram(to_integer(unsigned(address_b)));   -- after write: NEW data
			end if;
		end if;
	end process port_b;

-- MiSTer2MEGA65 (AExp Amiga 500 port), June 2026: original Altera
-- altsyncram instantiation, kept for reference:
--
--	altsyncram_component : altsyncram
--	GENERIC MAP (
--		address_reg_b => "CLOCK1",
--		clock_enable_input_a => "NORMAL",
--		clock_enable_input_b => "NORMAL",
--		clock_enable_output_a => "BYPASS",
--		clock_enable_output_b => "BYPASS",
--		indata_reg_b => "CLOCK1",
--		intended_device_family => "Cyclone V",
--		lpm_type => "altsyncram",
--		numwords_a => 2**addr_width_a,
--		numwords_b => 2**addr_width_b,
--		operation_mode => "BIDIR_DUAL_PORT",
--		outdata_aclr_a => "NONE",
--		outdata_aclr_b => "NONE",
--		outdata_reg_a => "UNREGISTERED",
--		outdata_reg_b => "UNREGISTERED",
--		power_up_uninitialized => "FALSE",
--		read_during_write_mode_port_a => "NEW_DATA_NO_NBE_READ",
--		read_during_write_mode_port_b => "NEW_DATA_NO_NBE_READ",
--		init_file => mem_init_file,
--		widthad_a => addr_width_a,
--		widthad_b => addr_width_b,
--		width_a => data_width_a,
--		width_b => data_width_b,
--		width_byteena_a => 1,
--		width_byteena_b => 1,
--		wrcontrol_wraddress_reg_b => "CLOCK1"
--	)
--	PORT MAP (
--		address_a => address_a,
--		address_b => address_b,
--		clock0 => clock0,
--		clock1 => clock1,
--		clocken0 => enable_a,
--		clocken1 => enable_b,
--		data_a => data_a,
--		data_b => data_b,
--		wren_a => wren_a and cs_a,
--		wren_b => wren_b and cs_b,
--		q_a => q0,
--		q_b => q1
--	);

END SYN;
