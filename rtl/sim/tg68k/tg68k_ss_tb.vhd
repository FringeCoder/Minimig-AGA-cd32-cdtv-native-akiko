-------------------------------------------------------------------------------
-- TG68KdotC_Kernel savestate restore testbench.
--
-- Why this exists: the savestate restore write ports (ss_wr_*, ss_pc_wr,
-- ss_sr_wr, ss_usp_wr, ss_vbr_wr, ss_cacr_wr) were added to the kernel with no
-- simulation behind them at all, and on hardware a restore garbles the screen
-- and then resets the core. This bench runs the real kernel against a flat
-- one-cycle memory so a restore can be watched instruction by instruction.
--
-- Three phases:
--
--   SANITY   the CPU boots from the reset vectors and runs the test program.
--            Nothing below is worth anything if this does not pass.
--
--   RED      restore every architectural register at FREEZE_N successive
--            freeze points across one pass of the main loop, without pulsing
--            ss_resume, and count how many of them resume correctly. This is
--            a scan rather than a single case on purpose: whether a restore
--            survives depends entirely on where inside an instruction the CPU
--            happened to be parked, which is why the hardware symptom is
--            "garbage, then a reset" rather than a clean, repeatable hang.
--            The bench asserts that at least one freeze point fails, so a
--            resume implementation that quietly does nothing cannot pass.
--
--   GREEN    the same scan with ss_resume pulsed. Every freeze point must
--            resume at the restored PC, execute the instruction the assembler
--            put there, and write the restored register values to memory.
--
-- On the freeze points: cpu_wrapper computes ss_cpu_hold as (ss_arm &
-- ~cpu_req). "No bus request pending" is not "at an instruction boundary" --
-- a multi-word instruction has gaps between its bus cycles. So freezing at
-- arbitrary cycles, as this bench does, is a faithful model of the hardware
-- and not a pessimistic one.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use std.textio.all;

entity tg68k_ss_tb is
	generic (
		-- Relative to the directory ghdl -r is run from (the repo root).
		PROG      : string  := "rtl/sim/tg68k/tg68k_ss_prog.hex";
		-- Number of successive freeze points to scan. One pass of the main
		-- loop is about 70 clocks, so 80 covers all of it and then some.
		FREEZE_N  : integer := 80;
		-- Clocks spent in the main loop before the first freeze point.
		FREEZE_LO : integer := 200
	);
end entity;

architecture sim of tg68k_ss_tb is

	constant MEMW : integer := 16384;			-- words, i.e. 32 KB
	-- Words put back between scan iterations: the program plus the sentinels.
	constant RELOADW : integer := 1024;

	type mem_t is array (0 to MEMW-1) of std_logic_vector(15 downto 0);

	-- Program layout, from tg68k_ss_prog.s.
	constant ADDR_MAIN     : std_logic_vector(31 downto 0) := X"00000400";
	constant ADDR_RESTORED : std_logic_vector(31 downto 0) := X"00000500";
	constant OPC_MAIN      : std_logic_vector(15 downto 0) := X"203C";	-- move.l #imm,d0
	constant OPC_RESTORED  : std_logic_vector(15 downto 0) := X"21C3";	-- move.l d3,$0604
	constant MAIN_MARK     : integer := 16#600#;
	constant RES_D3        : integer := 16#604#;
	constant RES_A2        : integer := 16#608#;
	constant RES_TAG       : integer := 16#60C#;

	-- Values pushed in through the savestate write ports.
	constant SS_PC_VAL  : std_logic_vector(31 downto 0) := ADDR_RESTORED;
	constant SS_SR_VAL  : std_logic_vector(15 downto 0) := X"2700";	-- supervisor, IPL 7
	constant SS_USP_VAL : std_logic_vector(31 downto 0) := X"00006000";
	constant SS_A7_VAL  : std_logic_vector(31 downto 0) := X"00007FF0";
	constant EXP_D3     : std_logic_vector(31 downto 0) := X"5EED0003";
	constant EXP_A2     : std_logic_vector(31 downto 0) := X"5EED000A";
	constant EXP_TAG    : std_logic_vector(31 downto 0) := X"DEADBEEF";

	function nib(c : character) return std_logic_vector is
	begin
		case c is
			when '0'       => return x"0";
			when '1'       => return x"1";
			when '2'       => return x"2";
			when '3'       => return x"3";
			when '4'       => return x"4";
			when '5'       => return x"5";
			when '6'       => return x"6";
			when '7'       => return x"7";
			when '8'       => return x"8";
			when '9'       => return x"9";
			when 'a' | 'A' => return x"A";
			when 'b' | 'B' => return x"B";
			when 'c' | 'C' => return x"C";
			when 'd' | 'D' => return x"D";
			when 'e' | 'E' => return x"E";
			when 'f' | 'F' => return x"F";
			when others    => return "XXXX";
		end case;
	end function;

	-- One 16-bit word per line, most significant nibble first. Anything that is
	-- not a hex digit ends the line, so a CRLF file loads unchanged.
	impure function load_hex(fname : string) return mem_t is
		file     f   : text;
		variable st  : file_open_status;
		variable l   : line;
		variable m   : mem_t := (others => (others => '0'));
		variable idx : integer := 0;
		variable w   : std_logic_vector(15 downto 0);
		variable n   : std_logic_vector(3 downto 0);
		variable k   : integer;
	begin
		file_open(st, f, fname, read_mode);
		assert st = open_ok
			report "tg68k_ss_tb: cannot open " & fname severity failure;
		while not endfile(f) loop
			readline(f, l);
			w := (others => '0');
			k := 0;
			for i in l.all'range loop
				n := nib(l.all(i));
				exit when n(0) = 'X';
				w := w(11 downto 0) & n;
				k := k + 1;
			end loop;
			if k > 0 and idx < MEMW then
				m(idx) := w;
				idx    := idx + 1;
			end if;
		end loop;
		file_close(f);
		assert idx > 0 report "tg68k_ss_tb: " & fname & " is empty" severity failure;
		return m;
	end function;

	constant IMAGE : mem_t := load_hex(PROG);

	signal mem : mem_t := IMAGE;

	signal clk         : std_logic := '0';
	signal nReset      : std_logic := '0';
	signal clkena_in   : std_logic := '1';
	signal data_in     : std_logic_vector(15 downto 0) := (others => '0');
	signal addr_out    : std_logic_vector(31 downto 0);
	signal data_write  : std_logic_vector(15 downto 0);
	signal nWr         : std_logic;
	signal nUDS        : std_logic;
	signal nLDS        : std_logic;
	signal busstate    : std_logic_vector(1 downto 0);
	signal longword    : std_logic;
	signal nResetOut   : std_logic;
	signal FC          : std_logic_vector(2 downto 0);
	signal clr_berr    : std_logic;
	signal skipFetch   : std_logic;
	signal regin_out   : std_logic_vector(31 downto 0);
	signal CACR_out    : std_logic_vector(3 downto 0);
	signal D_CACHE_out : std_logic;
	signal VBR_out     : std_logic_vector(31 downto 0);

	signal ss_reg_index : std_logic_vector(3 downto 0) := (others => '0');
	signal ss_reg_data  : std_logic_vector(31 downto 0);
	signal ss_pc        : std_logic_vector(31 downto 0);
	signal ss_sr        : std_logic_vector(15 downto 0);
	signal ss_usp       : std_logic_vector(31 downto 0);

	signal ss_wr_index : std_logic_vector(3 downto 0)  := (others => '0');
	signal ss_wr_data  : std_logic_vector(31 downto 0) := (others => '0');
	signal ss_wr_en    : std_logic := '0';
	signal ss_pc_wr    : std_logic := '0';
	signal ss_sr_wr    : std_logic := '0';
	signal ss_usp_wr   : std_logic := '0';
	signal ss_vbr_wr   : std_logic := '0';
	signal ss_cacr_wr  : std_logic := '0';
	signal ss_resume   : std_logic := '0';

	-- Testbench back door into the memory model, so a scan iteration can undo
	-- whatever the previous one's runaway CPU scribbled.
	signal poke_en   : std_logic := '0';
	signal poke_idx  : integer := 0;
	signal poke_data : std_logic_vector(15 downto 0) := (others => '0');

	signal running : boolean := true;
	signal errors  : integer := 0;

begin

	clk <= (not clk) after 5 ns when running else '0';

	dut : entity work.TG68KdotC_Kernel
		generic map (
			SR_Read        => 2,
			VBR_Stackframe => 2,
			extAddr_Mode   => 2,
			MUL_Mode       => 2,
			DIV_Mode       => 2,
			BitField       => 2,
			BarrelShifter  => 1,
			MUL_Hardware   => 1
		)
		port map (
			clk            => clk,
			nReset         => nReset,
			clkena_in      => clkena_in,
			data_in        => data_in,
			IPL            => "111",			-- active low: no interrupt pending
			IPL_autovector => '0',
			berr           => '0',
			CPU            => "11",				-- as cpu_wrapper instantiates it
			addr_out       => addr_out,
			data_write     => data_write,
			nWr            => nWr,
			nUDS           => nUDS,
			nLDS           => nLDS,
			busstate       => busstate,
			longword       => longword,
			nResetOut      => nResetOut,
			FC             => FC,
			clr_berr       => clr_berr,
			skipFetch      => skipFetch,
			regin_out      => regin_out,
			CACR_out       => CACR_out,
			D_CACHE_out    => D_CACHE_out,
			VBR_out        => VBR_out,
			ss_reg_index   => ss_reg_index,
			ss_reg_data    => ss_reg_data,
			ss_pc          => ss_pc,
			ss_sr          => ss_sr,
			ss_usp         => ss_usp,
			ss_wr_index    => ss_wr_index,
			ss_wr_data     => ss_wr_data,
			ss_wr_en       => ss_wr_en,
			ss_pc_wr       => ss_pc_wr,
			ss_sr_wr       => ss_sr_wr,
			ss_usp_wr      => ss_usp_wr,
			ss_vbr_wr      => ss_vbr_wr,
			ss_cacr_wr     => ss_cacr_wr,
			ss_resume      => ss_resume
		);

	-- Flat, one-cycle, zero-wait-state memory. That is exactly how the kernel
	-- behaves inside the core whenever clkena_in is high, so no wait-state
	-- modelling is needed to make the instruction stream realistic.
	mem_read : process (addr_out, mem)
	begin
		if is_x(addr_out(14 downto 1)) then
			data_in <= (others => '0');
		else
			data_in <= mem(conv_integer(addr_out(14 downto 1)));
		end if;
	end process;

	mem_write : process (clk)
		variable a : integer;
	begin
		if rising_edge(clk) then
			if poke_en = '1' then
				mem(poke_idx) <= poke_data;
			elsif clkena_in = '1' and busstate = "11"
			      and not is_x(addr_out(14 downto 1)) then
				a := conv_integer(addr_out(14 downto 1));
				if nUDS = '0' then
					mem(a)(15 downto 8) <= data_write(15 downto 8);
				end if;
				if nLDS = '0' then
					mem(a)(7 downto 0) <= data_write(7 downto 0);
				end if;
			end if;
		end if;
	end process;

	stim : process

		variable ok       : boolean;
		variable ok2      : boolean;
		variable f_pc     : std_logic_vector(31 downto 0);
		variable f_opc    : std_logic_vector(15 downto 0);
		variable w_opc    : std_logic_vector(15 downto 0);
		variable red_pass : integer := 0;
		variable grn_pass : integer := 0;
		variable summary  : string(1 to 256) := (others => ' ');

		function hex(v : std_logic_vector) return string is
			constant digits : string(1 to 16) := "0123456789ABCDEF";
			variable nvec   : std_logic_vector(v'length-1 downto 0) := v;
			variable s      : string(1 to (v'length+3)/4);
			variable d      : integer;
			variable bad    : boolean;
		begin
			for i in s'range loop
				d   := 0;
				bad := false;
				for b in 3 downto 0 loop
					d := d * 2;
					case nvec(nvec'left - (i-1)*4 - (3-b)) is
						when '1' | 'H' => d := d + 1;
						when '0' | 'L' => null;
						when others    => bad := true;
					end case;
				end loop;
				if bad then
					s(i) := 'x';
				else
					s(i) := digits(d + 1);
				end if;
			end loop;
			return s;
		end function;

		impure function meml(byteaddr : integer) return std_logic_vector is
			variable r : std_logic_vector(31 downto 0);
		begin
			r(31 downto 16) := mem(byteaddr/2);
			r(15 downto 0)  := mem(byteaddr/2 + 1);
			return r;
		end function;

		procedure note(s : string) is
			variable ln : line;
		begin
			write(ln, s);
			writeline(output, ln);
		end procedure;

		procedure step(n : integer) is
		begin
			for i in 1 to n loop
				wait until rising_edge(clk);
			end loop;
		end procedure;

		procedure check(cond : boolean; what : string) is
		begin
			if cond then
				note("  ok   : " & what);
			else
				note("  FAIL : " & what);
				errors <= errors + 1;
				wait for 0 ns;
			end if;
		end procedure;

		-- Put the program and the sentinels back the way the assembler left
		-- them. A restore that goes wrong writes wherever it likes.
		procedure reload_image is
		begin
			for i in 0 to RELOADW-1 loop
				poke_idx  <= i;
				poke_data <= IMAGE(i);
				poke_en   <= '1';
				wait until rising_edge(clk);
			end loop;
			poke_en <= '0';
		end procedure;

		-- Drive the savestate write ports the way ss_ctrl does: the CPU clock
		-- enable is already low, one port strobed at a time.
		procedure ss_write_regs is
			variable v : std_logic_vector(31 downto 0);
		begin
			for i in 0 to 15 loop
				if i = 15 then
					v := SS_A7_VAL;					-- A7 has to stay a usable stack
				else
					v := X"5EED00" & conv_std_logic_vector(i, 8);
				end if;
				ss_wr_index <= conv_std_logic_vector(i, 4);
				ss_wr_data  <= v;
				ss_wr_en    <= '1';
				wait until rising_edge(clk);
				ss_wr_en    <= '0';
				wait until rising_edge(clk);
			end loop;

			ss_wr_data <= SS_USP_VAL;
			ss_usp_wr  <= '1';
			wait until rising_edge(clk);
			ss_usp_wr  <= '0';

			ss_wr_data <= X"0000" & SS_SR_VAL;
			ss_sr_wr   <= '1';
			wait until rising_edge(clk);
			ss_sr_wr   <= '0';

			ss_wr_data <= X"00000000";
			ss_vbr_wr  <= '1';
			wait until rising_edge(clk);
			ss_vbr_wr  <= '0';

			ss_wr_data <= X"00000000";
			ss_cacr_wr <= '1';
			wait until rising_edge(clk);
			ss_cacr_wr <= '0';

			ss_wr_data <= SS_PC_VAL;
			ss_pc_wr   <= '1';
			wait until rising_edge(clk);
			ss_pc_wr   <= '0';
			wait until rising_edge(clk);
		end procedure;

		-- Step the clock, printing the first `n` code-fetch cycles seen, and
		-- hand back the very first one. busstate="00" is the kernel's code
		-- space; the address is whatever the fetch pipeline put on the bus.
		-- `want` is watched over the whole window, not just the printed part,
		-- so a caller can dump the trace and assert on an address inside it
		-- without the dump swallowing the fetch it wanted to check.
		procedure trace_fetches(n         : integer;
		                        timeout   : integer;
		                        want      : std_logic_vector(31 downto 0);
		                        pc        : out std_logic_vector(31 downto 0);
		                        opc       : out std_logic_vector(15 downto 0);
		                        found     : out boolean;
		                        want_opc  : out std_logic_vector(15 downto 0);
		                        want_seen : out boolean) is
			variable seen : integer := 0;
			variable ws   : boolean := false;
			variable wo   : std_logic_vector(15 downto 0) := (others => '0');
		begin
			found := false;
			pc    := (others => '0');
			opc   := (others => '0');
			for i in 1 to timeout loop
				wait until rising_edge(clk);
				if clkena_in = '1' and busstate = "00" then
					if seen = 0 then
						pc    := addr_out;
						opc   := data_in;
						found := true;
					end if;
					if seen < n then
						note("       fetch[" & integer'image(seen) & "] $" &
						     hex(addr_out) & " -> $" & hex(data_in));
					end if;
					if addr_out = want and not ws then
						wo := data_in;
						ws := true;
					end if;
					seen := seen + 1;
					exit when seen >= n and ws;
				end if;
			end loop;
			want_opc  := wo;
			want_seen := ws;
		end procedure;

		-- Wait for a code fetch at a specific address, and stop there.
		procedure wait_fetch_at(want    : std_logic_vector(31 downto 0);
		                        timeout : integer;
		                        opc     : out std_logic_vector(15 downto 0);
		                        found   : out boolean) is
		begin
			found := false;
			opc   := (others => '0');
			for i in 1 to timeout loop
				wait until rising_edge(clk);
				if clkena_in = '1' and busstate = "00" and addr_out = want then
					opc   := data_in;
					found := true;
					exit;
				end if;
			end loop;
		end procedure;

		-- One complete experiment: boot, run into the main loop, freeze
		-- `freeze` clocks later, push the whole architectural state in,
		-- optionally pulse ss_resume, let go, and decide whether the machine
		-- came back.
		procedure do_restore(freeze     : integer;
		                     use_resume : boolean;
		                     verbose    : integer;
		                     pass       : out boolean;
		                     fpc        : out std_logic_vector(31 downto 0);
		                     fopc       : out std_logic_vector(15 downto 0)) is
			variable lpc  : std_logic_vector(31 downto 0);
			variable lopc : std_logic_vector(15 downto 0);
			variable wo   : std_logic_vector(15 downto 0);
			variable f    : boolean;
			variable w    : boolean;
		begin
			nReset    <= '0';
			clkena_in <= '1';
			ss_resume <= '0';
			step(6);
			reload_image;
			step(4);
			nReset <= '1';
			wait_fetch_at(ADDR_MAIN, 400, lopc, f);
			assert f report "tg68k_ss_tb: the CPU never reached $400"
				severity failure;
			step(60 + freeze);

			clkena_in <= '0';
			step(4);
			ss_write_regs;

			if use_resume then
				ss_resume <= '1';
				wait until rising_edge(clk);
				ss_resume <= '0';
			end if;
			step(2);
			clkena_in <= '1';

			trace_fetches(verbose, 400, ADDR_RESTORED, lpc, lopc, f, wo, w);
			step(800);

			fpc  := lpc;
			fopc := lopc;
			pass := f and (lpc = ADDR_RESTORED) and (lopc = OPC_RESTORED)
			        and (meml(RES_D3) = EXP_D3) and (meml(RES_A2) = EXP_A2)
			        and (meml(RES_TAG) = EXP_TAG);
		end procedure;

	begin
		note("");
		note("=== tg68k_ss_tb ===");

		----------------------------------------------------------------------
		note("");
		note("PHASE 1 -- sanity: does the bench run the CPU at all?");
		----------------------------------------------------------------------
		nReset <= '0';
		step(10);
		nReset <= '1';

		-- The kernel's reset seeds a synthesised "movea.l (0).l,a7 / jmp nn.l"
		-- with PC=4, so the reset vectors are read first and the entry point
		-- held at address 4 is the first real instruction fetch.
		trace_fetches(6, 400, ADDR_MAIN, f_pc, f_opc, ok, w_opc, ok2);
		check(ok, "the CPU produces code fetches after reset");
		check(f_pc = X"00000004",
		      "the first code fetch is address 4 -- reset's synthesised jmp " &
		      "reading its target");
		check(ok2, "a code fetch lands on the reset vector target $00000400");
		check(w_opc = OPC_MAIN,
		      "the word fetched there is move.l #imm,d0 ($203C)");

		step(80);
		note("       MAIN_MARK = $" & hex(meml(MAIN_MARK)));
		check(meml(MAIN_MARK)(31 downto 8) = X"111111",
		      "the main loop writes its heartbeat to $600");
		check(meml(MAIN_MARK) /= X"11111111",
		      "the heartbeat has been incremented, so the instructions at " &
		      "$400 really executed");

		----------------------------------------------------------------------
		note("");
		note("PHASE 2 -- RED: restore at " & integer'image(FREEZE_N) &
		     " freeze points, ss_resume never pulsed");
		----------------------------------------------------------------------
		-- One verbose case first, so there is a readable trace of the failure.
		note("       freeze point 0, first code fetches after the release:");
		do_restore(FREEZE_LO, false, 10, ok, f_pc, f_opc);
		note("       RES_D3 = $" & hex(meml(RES_D3)) &
		     "  RES_A2 = $" & hex(meml(RES_A2)) &
		     "  RES_TAG = $" & hex(meml(RES_TAG)));
		note("       PC now = $" & hex(ss_pc));

		for i in 0 to FREEZE_N-1 loop
			do_restore(FREEZE_LO + i, false, 0, ok, f_pc, f_opc);
			if ok then
				red_pass     := red_pass + 1;
				summary(i+1) := '.';
			else
				summary(i+1) := 'X';
			end if;
		end loop;
		note("       per freeze point (. = restored, X = did not): " &
		     summary(1 to FREEZE_N));
		note("       " & integer'image(red_pass) & " of " &
		     integer'image(FREEZE_N) &
		     " freeze points restored correctly without ss_resume");
		check(red_pass < FREEZE_N,
		      "at least one freeze point fails without ss_resume " &
		      "(a failure here means ss_resume is not load-bearing)");

		----------------------------------------------------------------------
		note("");
		note("PHASE 3 -- GREEN: the same scan with ss_resume pulsed");
		----------------------------------------------------------------------
		note("       freeze point 0, first code fetches after the resume:");
		do_restore(FREEZE_LO, true, 10, ok, f_pc, f_opc);
		note("       RES_D3 = $" & hex(meml(RES_D3)) &
		     "  RES_A2 = $" & hex(meml(RES_A2)) &
		     "  RES_TAG = $" & hex(meml(RES_TAG)));
		note("       PC now = $" & hex(ss_pc));
		check(f_pc = ADDR_RESTORED,
		      "the first code fetch is the restored PC $00000500");
		check(f_opc = OPC_RESTORED,
		      "the first opcode word is move.l d3,$0604 ($21C3)");
		check(meml(RES_D3) = EXP_D3, "the restored D3 reached memory intact");
		check(meml(RES_A2) = EXP_A2, "the restored A2 reached memory intact");
		check(meml(RES_TAG) = EXP_TAG,
		      "the third instruction ran (the fixed tag was written)");

		summary := (others => ' ');
		for i in 0 to FREEZE_N-1 loop
			do_restore(FREEZE_LO + i, true, 0, ok, f_pc, f_opc);
			if ok then
				grn_pass     := grn_pass + 1;
				summary(i+1) := '.';
			else
				summary(i+1) := 'X';
				note("       freeze point " & integer'image(i) &
				     " did not resume: first fetch $" & hex(f_pc) &
				     " -> $" & hex(f_opc));
			end if;
		end loop;
		note("       per freeze point (. = restored, X = did not): " &
		     summary(1 to FREEZE_N));
		check(grn_pass = FREEZE_N,
		      "every freeze point resumes correctly with ss_resume (" &
		      integer'image(grn_pass) & "/" & integer'image(FREEZE_N) & ")");

		----------------------------------------------------------------------
		note("");
		if errors = 0 then
			note("RUN: PASS");
		else
			note("RUN: FAIL (" & integer'image(errors) & " checks failed)");
		end if;
		running <= false;
		wait for 100 ns;
		assert errors = 0
			report "tg68k_ss_tb: " & integer'image(errors) & " checks failed"
			severity failure;
		wait;
	end process;

end architecture;
