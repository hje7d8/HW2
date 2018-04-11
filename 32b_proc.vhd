-- NAME: Hunter Earnest
-- CpE5120
-- 32 bit MIPS Processor

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.STD_LOGIC_ARITH.UNSIGNED;
use IEEE.STD_LOGIC_UNSIGNED.all;
use IEEE.NUMERIC_STD.UNSIGNED;
use STD.TEXTIO.all;

package proc is
	type Proc_op 				is (Add, Sub, Mpy, Comp, Not32, And32, Or32, Xor32, Lw, Sw, jmp, Beq, Bne, Slt, Nop);	-- Char op codes
	type Proc_state 		    is (fet, dec, exec, store, ret);						-- State machine types
	type stor100				is Array(0 to 1000) of std_logic_vector(31 downto 0);	-- Type for making blocks of memory
	type reg					is Array (0 to 31) of std_logic_vector(31 downto 0);  	-- Type for making proc reg
end package proc;

package body proc is
end package body proc;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
use work.proc.all;
use work.all;


-- ALU 32bit --
entity ALU_32 is
  port (	clk, reset: in std_logic;
			A_bus, B_bus: std_logic_vector(31 downto 0);
			Q_bus: inout std_logic_vector(63 downto 0); 
			Opcode: in Proc_op; 
			Proc_ready: out std_logic);
end entity ALU_32;

architecture Behavior of ALU_32 is
	signal A_temp, B_temp: 		std_logic_vector(32 downto 0):=(others => '0');
	signal A_buf, B_buf: 		std_logic_vector(31 downto 0):=(others => '0');
	signal Q: 			std_logic_vector(63 downto 0):=(others => '0');
	signal Overflow: 	std_logic;
	signal Status_code: std_logic_vector (0 to 1);

	procedure action (Bus_A, Bus_B: in std_logic_vector (31 downto 0); signal Bus_Q: inout std_logic_vector (63 downto 0); B_Opcode: in Proc_op) is
		variable zeros: 	std_logic_vector(31 downto 0):=(others => '0');
		variable ones: 		std_logic_vector(31 downto 0):=(others => '1');
		variable TempValue: std_logic_vector(31 downto 0);
	begin
		case B_Opcode is
			when Lw => Bus_Q <= zeros(31 downto 0) & std_logic_vector(signed(Bus_A) + signed(Bus_B));
			when Sw => Bus_Q <= zeros(31 downto 0) & std_logic_vector(signed(Bus_A) + signed(Bus_B));
			when jmp => Bus_Q <= zeros(31 downto 0) & Bus_A;
			when Beq =>
				if Bus_A = Bus_B then
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 0);
				end if;
			when Bne =>
				if Bus_A /= Bus_B then
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else
					Bus_Q <= zeros(31 downto 0) & zeros(31 downto 0);
				end if;
			when Slt =>
				if Bus_A < Bus_B then Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "1";
				else Bus_Q <= zeros(31 downto 0) & zeros(31 downto 1) & "0";
				end if;
			when others => 	Bus_Q <= Bus_Q;
		end case;
	end procedure action;

	begin
		A_temp<= '0' & A_bus;
		B_temp<= '0' & B_bus;
		A_buf<= A_bus;
		B_buf<= B_bus;

		ALU_Exec: process(clk, reset, Opcode)
			variable zeros: std_logic_vector(31 downto 0):=(others => '0');
		begin
			if reset='0' and clk'event and clk='1' then
				case Opcode is
					when Add=> Q <= zeros(30 downto 0) &  std_logic_vector(signed(A_temp) + signed(B_temp));
						if Q(32) = '1' then
							overflow<='1';
							assert not(overflow='1') report "Overflow occurred during Addition" severity warning;
						end if;
					when Sub=> Q <= zeros(30 downto 0) & std_logic_vector(signed(A_temp) - signed(B_temp));
						if signed(B_bus) > signed(A_bus)  then
							overflow<='1';
							assert not(overflow='1') report "Overflow occurred during Subtraction" severity warning;
						end if;
					when Mpy=> Q <= std_logic_vector(signed(A_buf) * signed(B_buf));
					when Comp=>
						if (A_buf<B_buf) then Status_code  <= "01"; 	
						elsif (A_buf=B_buf) then Status_code <= "00"; 	
						else Status_code <= "10"; 				
						end if;
						Q <= zeros(31 downto 0) & zeros(31 downto 2) & Status_code;
					when And32 => 	Q<= zeros(31 downto 0) & (A_buf and B_buf);
					when Or32 => 	Q<= zeros(31 downto 0) & (A_buf or B_buf);
					when Xor32 => 	Q<= zeros(31 downto 0) & (A_buf xor B_buf);
					when Not32 => 	Q<= zeros(31 downto 0 ) & not A_buf;
					when Lw => 		action (A_buf, B_buf, Q, Opcode);
					when Sw => 		action (A_buf, B_buf, Q, Opcode);
					when jmp => 	action (A_buf, B_buf, Q, Opcode);
					when Bne => 	action (A_buf, B_buf, Q, Opcode);
					when Beq => 	action (A_buf, B_buf, Q, Opcode);
					when Slt => 	action (A_buf, B_buf, Q, Opcode);
					when others => 	Q <= zeros(31 downto 0)&zeros(31 downto 0);
				end case;
			end if;
		Q_bus <= Q;
		Proc_ready <= '1';
	end process;
end architecture behavior;

Library IEEE;
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
use work.proc.all;
use work.all;

-- proc ---------------

entity proc_32 is
	port (	CLK, reset	: in std_logic);
end entity proc_32;
--
architecture mip_proc of proc_32 is
	
Component ALU_32
   port (A_bus, B_bus: std_logic_vector(31 downto 0);Q_bus: inout std_logic_vector(63 downto 0); Opcode: in Proc_op; Proc_ready: out std_logic; clk, reset: in std_logic);
end Component ALU_32;



	signal STATE		: 		Proc_state 		:= fet;
	signal A 				: 		std_logic_vector (31 downto 0):= (others => '0');
	signal B 				:		std_logic_vector (31 downto 0):= (others => '0');
	signal Q				: 		std_logic_vector (63 downto 0):=(others => '0');
	signal reg_32			: 		reg := ((others=> (others=>'0')));						-- Proc 32 32bit reg. don't write to (0).
	signal pc 				:  		std_logic_vector (31 downto 0) := (others => '0');
	signal current_instr	:		std_logic_vector (31 downto 0) := (others => '0');
	signal read_prog		:		std_logic;
	signal read_requ		:		std_logic;
	signal write_requ 		: 		std_logic;
	signal Addr				: 		std_logic_vector (31 downto 0);
	signal read_in			: 		std_logic_vector (31 downto 0);
	signal write_out		: 		std_logic_vector (31 downto 0);
	signal Opcode			: 		Proc_op;
	signal instruction_op	: 		Proc_op;
	signal Proc_ready		: 		std_logic;
	signal instruction_rs	: 		std_logic_vector (4 downto 0);
	signal instruction_rt	: 		std_logic_vector (4 downto 0);
	signal instruction_rd	: 		std_logic_vector (4 downto 0);
	signal instruction_sham	: 		std_logic_vector (4 downto 0);
	signal instruction_func	: 		std_logic_vector (5 downto 0);
	signal inst				:		stor100;
	signal var_storage		:		stor100;


for ALU_32C: ALU_32 use entity work.ALU_32(behavior);
begin

-- ALU init
ALU_32C: ALU_32 port map (A_bus=> A, B_bus=> B, Q_bus=>Q, Opcode=>Opcode, Proc_ready=>Proc_ready, CLK=>CLK, reset=>reset);
	
	DataMem: process(reset, Addr, read_requ, write_requ)
		begin
		if reset = '1' then 
				var_storage(500) <= "00000000000000000000000000000000";  -- made up values
				var_storage(501) <= "11111111111111111111111111111111";
				var_storage(505) <= "00000000000000000000000000000001";
				var_storage(510) <= "00000000000000000000000000000010";
				var_storage(511) <= "00000000000000000000000000000011";
				var_storage(512) <= "00000000000000000000000000001111";
		elsif reset = '0'then
			if (read_requ ='1') then 
				read_in <= var_storage(to_integer(unsigned(Addr)));
			elsif(write_requ = '1')
				then var_storage(to_integer(unsigned(Addr))) <= write_out;
			end if;
		end if;
	end process DataMem;
	
	
	instuctions: Process(reset, read_prog, pc)
		begin
		if reset = '1' then -- Write Program Here --
			inst(0)			<= "100011" & "00000" & "00001" & "0000000111111001"; --Lw, 505 to Reg1
			inst(1)			<= "100011" & "00000" & "00010" & "0000000111111110"; --Lw, 510 to Reg2
			inst(2)			<= "000000" & "00001" & "00010" & "00011" & "00000" & "100000"; --Add, Reg1 + Reg2 = Reg3
			inst(3)			<= "000000" & "00010" & "00001" & "00100" & "00000" & "100010"; --Sub, Reg2 - Reg1 = Reg4
			inst(4)			<= "100011" & "00000" & "00101" & "0000000111110100"; --LW, 500 to Reg5
			inst(5)			<= "100011" & "00000" & "00110" & "0000000111110101"; --Lw, 501 to Reg6
			inst(6)			<= "000000" & "00101" & "00110" & "00111" & "00000" & "100100"; --And, Reg5 and Reg6, output to Reg7
			inst(7)			<= "000000" & "00001" & "00010" & "01000" & "00000" & "011000"; --Mpy, Reg1 * Reg2 = Reg8
			inst(8)			<= "000000" & "00001" & "00010" & "01001" & "00000" & "100101"; --Or, Reg1 or Reg2, output to Reg9
			inst(9)  		<= "000000" & "00001" & "00010" & "01010" & "00000" & "100110"; --Xor, Reg1 xor Reg2, output to Reg10
			inst(10)  		<= "000000" & "00101" & "00000" & "01011" & "00000" & "101000"; --Not, Not Reg5, output to Reg11
			inst(11)		<= "101011" & "00000" & "00110" & "0000001000000011"; --Sw, Reg2 to 515
			inst(12)		<= "100011" & "00000" & "01100" & "0000000111111001"; --Lw, 505 to Reg12
			inst(13)		<= "000100" & "00001" & "01100" & "0000000001001111"; --Beq, Reg1 = Reg12 PC = 93
			inst(93)		<= "000101" & "00001" & "00010" & "1111111110110000"; --Bne, Reg1 /= Reg2 PC = 14
			inst(14)		<= "000010" & "00000000000000000000000000"; 		  --jmp, loop back to start
			current_instr 	<= "00000000000000000000000000000000";				  -- reset current instr if reset toggled
		elsif reset = '0' then	
			if read_prog = '1' then
			   current_instr <= inst(to_integer(unsigned(pc)));
			end if;
		end if;
	end Process instuctions;
	
	
	pcontrol: Process
		variable temp : std_logic_vector(31 downto 0):=(others => '0');
	begin
		Wait until (reset = '0' and CLK'event and CLK='1');
		
		case STATE is
			when fet   => 
				read_prog 	<= 	'1'; 			-- Grabs the next instruction and puts it into the current_instr
				write_requ 	<=	'0';
				read_requ 	<=	'0';
                STATE 	<=	dec;
				
			when dec  =>  					-- Grabs the opcode from the instr and puts it into a var the ALU can use.
				read_prog <= '0';
					case current_instr(31 downto 26) is
						when "000000" =>  					
							case current_instr(5 downto 0) is 
								when "101010" =>
									instruction_op <= Slt;
								when "100000" =>
									instruction_op <= Add;
								when "100010" =>
									instruction_op <= Sub;
								when "011000" =>
									instruction_op <= Mpy;
								when "100100" =>
									instruction_op <= And32;
								when "100101" =>
									instruction_op <= Or32;
								when "100110" =>
									instruction_op <= Xor32;
								when "101000" =>
									instruction_op <= Not32;
								when "101100" =>
									instruction_op <= Comp;
								when others =>
									instruction_op <= Nop;
							end case;
						when "100011" =>
							instruction_op <= Lw;
						when "101011" =>
							instruction_op <= Sw;
						when "000010" =>
							instruction_op <= jmp;
						when "000100" =>
							instruction_op <= Beq;
						when "000101" =>
							instruction_op <= Bne;
						when others =>
							instruction_op <= Nop;
				end case;
				
				instruction_rs		<= current_instr(25 downto 21);			-- Seperating other parts of instr to save time when needed
				instruction_rt		<= current_instr(20 downto 16);
				instruction_rd		<= current_instr(15 downto 11);
				instruction_sham	<= current_instr(10 downto 6);
				instruction_func	<= current_instr(5 downto 0);	
				Proc_ready	<= '0';
				STATE			<= exec;
				
			when exec =>  
				Proc_ready	<= '1';
				if (current_instr(31 downto 26) = "000000") then															-- Loads RS and RT for R type
					A 	<= 	reg_32(to_integer(unsigned(instruction_rs)));
					B 	<= 	reg_32(to_integer(unsigned(instruction_rt)));
				elsif (instruction_op = jmp) then
					A 	<= 	reg_32(0)(31 downto 26) & current_instr(25 downto 0);											-- load where to jump
				elsif (instruction_op = Lw or instruction_op = Sw) then						
					A 	<= 	reg_32(to_integer(unsigned(instruction_rs)));													-- Prep to load or store word
					if( current_instr(15) = '1' ) then												
						B <= "1111111111111111" & current_instr(15 downto 0);												-- sign extend for 32 bit ALU
					else
						B <= "0000000000000000" & current_instr(15 downto 0);												-- sign extend for 32 bit ALU
					end if;
				elsif (instruction_op = Beq or instruction_op = Bne or instruction_op = Nop) then							-- Rest of ops
					A <= reg_32(to_integer(unsigned(instruction_rs)));														
					B <= reg_32(to_integer(unsigned(instruction_rt)));														
				end if;
				Opcode <= instruction_op;			-- Sets opcode for ALU to know which op to perform
				if (Proc_ready='1') then			
					STATE <= store;
				end if;
				
              	
			when store =>
				if (instruction_op = Lw) then										-- Set up for reading word from mem
					read_requ 	<=	'1';											-- request from mem to read data
					Addr		<=	Q(31 downto 0);									-- sets Addr to read from
					STATE	<= 	ret;												-- send to ret to actually load word
				elsif (instruction_op = Sw) then
					write_requ 	<=	'1';											-- tell mem we're goint to write
					Addr		<=	Q(31 downto 0);									-- Set write Addr
					write_out	<=	reg_32(to_integer(unsigned(instruction_rt))); 	-- send data to mem	
					pc 			<= 	std_logic_vector(unsigned(pc) + 1);
					STATE 	<= 	fet;
				elsif (current_instr(31 downto 26) = "000000") then					-- for ALU/R ops
					reg_32(to_integer(unsigned(instruction_rd))) <= Q(31 downto 0);	-- Pull from ALU output and place into reg
					pc 	<= 	std_logic_vector(unsigned(pc) + 1);						--	inc pc
					STATE 	<= 	fet;
				elsif (instruction_op = beq or instruction_op = bne) then			-- Handle Branch
					if (unsigned(Q) = 1) then										-- Handle Branch True
						if (current_instr(15) = '0') then							-- checking to see if going forward or back		
							pc 	<= 	std_logic_vector( unsigned(pc) + unsigned(current_instr(15 downto 0)) + 1 ); -- simple add
						else
							temp := "1111111111111111" & current_instr(15 downto 0);	-- going backwards
							pc 	<= 	std_logic_vector( unsigned(pc) + unsigned(temp) + 1 );
						end if;
					else
						pc 	<= 	std_logic_vector(unsigned(pc) + 1); 				-- no branch just incr
					end if;
					STATE 	<= 	fet;
				elsif (instruction_op = jmp)  then									-- jmp P counter
					pc 	<= 	Q(31 downto 0);											--	Set pc to jmp number
					STATE 	<= 	fet;
				else STATE	<= 	ret;
				end if;
			
			when ret  => 
				if (instruction_op = Lw) then 										-- Load words
					reg_32(to_integer(unsigned(instruction_rt))) <= read_in;		-- load word into reg
					pc 	<= 	std_logic_vector(unsigned(pc) + 1);						-- inc pc
				end if;
				STATE 	<= 	fet; 													-- Restart cycle
		end case;
	end process pcontrol;
end architecture mip_proc;

--End Proc--------

--- Begin TB -----

Library IEEE;
Use IEEE.std_logic_1164.all;
Use IEEE.STD_LOGIC_ARITH.UNSIGNED;
Use IEEE.STD_LOGIC_UNSIGNED.all;
Use IEEE.NUMERIC_STD.UNSIGNED;
Use work.proc.all;
Use STD.TEXTIO.all;
Use work.all;

entity proc_tb is
end entity proc_tb;

architecture test of proc_tb is
Component proc_32
 port (CLK, reset: in std_logic);
end Component proc_32;

Signal reset, CLK: std_logic;

for test_proc: proc_32 use entity work.proc_32(mip_proc);
begin



reset <= '1', '0' after 100 ps;

in_clk: process			-- Simple clock toggle
	begin
	   CLK <= '0';
	   wait for 5 ps;
	   CLK <= '1';
	   wait for 5 ps;
end process;

test_proc: proc_32 port map (CLK, reset);

end architecture TEST;


