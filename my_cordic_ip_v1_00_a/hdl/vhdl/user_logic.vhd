----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------cordic-------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;


entity cordic is 
	generic(
		WIDTH 	: natural := 16;
		PIPEID	: natural := 15
	);
	port(
		clk		: in std_logic;
		reset	: in std_logic;
		ena		: in std_logic;
		---------------------p2r module--------------------------
		Ain			: in signed(15 downto 0);
		
		sin_val		: out signed(15 downto 0);
		sin_sign	: out std_logic;	-- 0 --> positive    1 --> negative
		cos_val		: out signed(15 downto 0);
		cos_sign	: out std_logic;	-- 0 --> positive    1 --> negative
		
		
		---------------------r2p module--------------------------
		Xin		: in signed(15 downto 0);
		Yin 	: in signed(15 downto 0);
		
		Rout	: out unsigned(19 downto 0);
		Aout	: out signed(19 downto 0)
	);
end entity cordic;

architecture top of cordic is
	component p2r_Cordic
	generic(
		PIPEID : integer := 15;
		WIDTH    : integer := 16);
	port(
		clk 		: in std_logic;
		reset	: in std_logic;
		ena 		: in std_logic;
		
		Ain			: in signed(15 downto 0);
		sin_val		: out signed(15 downto 0);
		sin_sign	: out std_logic;	-- 0 --> positive    1 --> negative
		cos_val		: out signed(15 downto 0);
		cos_sign	: out std_logic);	-- 0 --> positive    1 --> negative

	end component p2r_Cordic;
 
 
component r2p_corproc
	port(
		clk 		: in std_logic;
		ena 		: in std_logic;
		
		Xin	: in signed(15 downto 0);
		Yin : in signed(15 downto 0);
 
		Rout	: out unsigned(19 downto 0);
		Aout	: out signed(19 downto 0));

	end component r2p_corproc;
 
 --
	--	ACHITECTURE BODY
	--
begin

p2r: p2r_Cordic 
			generic map(WIDTH => WIDTH, PIPEID => PIPEID)
			port map ( clk, reset,ena, Ain ,sin_val ,sin_sign , cos_val, cos_sign );
			
r2p: r2p_corproc
			port map ( clk => clk, ena => ena, Xin => Xin, Yin => Yin, Rout => Rout, Aout => Aout );
			
end top;












----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------p2r_CordicPipe-----------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;

--For n=0 to [inf]
--If (Z(n) >= 0) then
--	X(n + 1) := X(n) - (Yn/2^n);
--	Y(n + 1) := Y(n) + (Xn/2^n);
--	Z(n + 1) := Z(n) - atan(1/2^n);
--Else
--	X(n + 1) := X(n) + (Yn/2^n);
--	Y(n + 1) := Y(n) - (Xn/2^n);
--	Z(n + 1) := Z(n) + atan(1/2^n);
--End if;
--End for;

entity p2r_CordicPipe is 
	generic(
		WIDTH 	: natural := 16;
		PIPEID	: natural := 1
	);
	port(
		clk				   : in std_logic;
		reset			   : in std_logic;
		ena				   : in std_logic;
 
		Xi				   : in signed(WIDTH -1 downto 0); 
		Yi				   : in signed(WIDTH -1 downto 0);
		Zi				   : in signed(19 downto 0);
 
		Xo				   : out signed(WIDTH -1 downto 0);
		Yo				   : out signed(WIDTH -1 downto 0);
		Zo				   : out signed(19 downto 0);
		sin_sign_inter_in  : in std_logic;
		cos_sign_inter_in  : in std_logic;
		sin_sign_inter_out : out std_logic;
		cos_sign_inter_out : out std_logic
	);
end entity p2r_CordicPipe;
 
architecture dataflow of p2r_CordicPipe is
 
	-- Function CATAN to calculate tan ^-1.
	-- 'n' is the number of the pipe
	-- return is a 20bit arc-tangent value.
	-- The numbers are calculated as follows: Z(n) = tan^-1(1/2^n)
	-- examples:
	-- 20bit values => 2^20 = 2pi(rad)
	--                 1(rad) = 2^20/2pi = 166886.053....
	--                 1(rad) = 2^19/2pi = 83,443.026803763621799596530531049
	-- n:0, atan(1/1) = 0.7853...(rad)
	--      0.7853... * 166886.053... = 131072(dec) = 20000(hex)
	-- n:1, atan(1/2) = 0.4636...(rad)
	--      0.4636... * 166886.053... = 77376.32(dec) = 12E40(hex)
	--
	function CATAN(n :natural) return integer is
	variable result	:integer;
	begin
		case n is
			when 0 => result := 16#020000#;
			when 1 => result := 16#012E40#;
			when 2 => result := 16#09FB4#;
			when 3 => result := 16#05111#;
			when 4 => result := 16#028B1#;
			when 5 => result := 16#0145D#;
			when 6 => result := 16#0A2F#;
			when 7 => result := 16#0518#;
			when 8 => result := 16#028C#;
			when 9 => result := 16#0146#;
			when 10 => result := 16#0A3#;
			when 11 => result := 16#051#;
			when 12 => result := 16#029#;
			when 13 => result := 16#014#;
			when 14 => result := 16#0A#;
			when 15 => result := 16#05#;
			when 16 => result := 16#03#;
			when 17 => result := 16#01#;
			when others => result := 16#0#;
		end case;
		return result;
	end CATAN;
 
	function AddSub(dataa, datab : in signed; add_sub : in std_logic) return signed is
	begin
		if (add_sub = '1') then
			return dataa + datab;
		else
			return dataa - datab;
		end if;
	end;
 
	--
	--	ARCHITECTURE BODY
	--
	signal dX, X_interm	: signed(WIDTH -1 downto 0);
	signal dY, Y_interm	: signed(WIDTH -1 downto 0);
	signal angle, remaining_angle	: signed(19 downto 0);

	signal Zneg, Zpos	: std_logic;

begin
	dX <= shift_right(Xi, PIPEID); --shift right arithmetic
	dY <= shift_right(Yi, PIPEID); --shift right arithmetic
	angle <= to_signed( catan(PIPEID), 20);
 
	-- generate adder structures
	Zneg <= Zi(19);
	Zpos <= not Zi(19);
 
	-- xadd
	X_interm <= AddSub(Xi, dY, Zneg);
 
	-- yadd 
	Y_interm <= AddSub(Yi, dX, Zpos);
 
	-- zadd
	remaining_angle <= AddSub(Zi, angle, Zneg);
 

    
	gen_regs: process(clk , reset)
	begin
 		if (reset = '1') then  --Asynchronous clear input
				Xo <= (others => '0');
				Yo <= (others => '0');
				Zo <= (others => '0');
				sin_sign_inter_out <= '0';
				cos_sign_inter_out <= '0';
		else
			if (rising_edge(CLK) ) then
				if (ena = '1') then
					Xo <= X_interm;
					Yo <= Y_interm;
					Zo <= remaining_angle;
					sin_sign_inter_out <= sin_sign_inter_in;
					cos_sign_inter_out <= cos_sign_inter_in;
				end if;
			end if;
		end if;
	end process;
 
end architecture dataflow;


----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------p2r_cordic---------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;

--thita_new = thita * 2^16 / 360 = thita * 182	-- range from 0 upto 360
--sin_value = sin_value_output * 1 / 2^15  = sin_value_output / 32768	-- range from -1 upto 1
--cos_value = cos_value_output * 1 / 2^15  = cos_value_output / 32768	-- range from -1 upto 1

-- 20*182 = 3640   sin = 11207/32768 cos = 30788/32768 	 --> sin = 	 0.342020143325668733044099614682262   cos =   0.93969262078590838405410927732473 
-- 160*182 = 29120 sin = 11207/32768 cos = 30788/32768   --> sin = 	 0.342020143325668733044099614682262   cos = - 0.93969262078590838405410927732473
-- 200*182 = 36400 sin = 11207/32768 cos = 30788/32768   --> sin = - 0.342020143325668733044099614682262   cos = - 0.93969262078590838405410927732473
-- 340*182 = 61880 sin = 11207/32768 cos = 30788/32768   --> sin = - 0.342020143325668733044099614682262   cos =   0.93969262078590838405410927732473




entity p2r_cordic is 
	generic(
		PIPEID : integer := 15;
		WIDTH    : integer := 16);
	port(
		clk 		: in std_logic;
		reset		: in std_logic;
		ena 		: in std_logic;
		
		Ain			: in signed(15 downto 0);
		sin_val		: out signed(15 downto 0);
		cos_val		: out signed(15 downto 0);
		sin_sign	: out std_logic;	-- 0 --> positive    1 --> negative
		cos_sign	: out std_logic);	-- 0 --> positive    1 --> negative

end entity p2r_Cordic;
 
 
architecture dataflow of p2r_cordic is

	signal thita		    : signed(15 downto 0);
	signal sin              : signed(15 downto 0);
	signal cos              : signed(15 downto 0);
	signal quadrant         : signed (1 downto 0 );	
	
	type XY_sign_Vector is array(PIPEID downto 0) of std_logic;
	type XYVector is array(PIPEID downto 0) of signed(WIDTH -1 downto 0);
	type ZVector is array(PIPEID downto 0) of signed(19 downto 0);
 
	signal sin_sign_inter, cos_sign_inter : XY_sign_Vector;	-- 0 --> positive    1 --> negative
 	signal X, Y	: XYVector;
	signal Z	: ZVector;
	
	component p2r_CordicPipe
	generic(
		WIDTH 	: natural := 16;
		PIPEID	: natural := 1
	);
	port(
		clk		: in std_logic;
		reset			   : in std_logic;
		ena		: in std_logic;
 
		Xi		: in signed(WIDTH -1 downto 0); 
		Yi		: in signed(WIDTH -1 downto 0);
		Zi		: in signed(19 downto 0);
 
		Xo				   : out signed(WIDTH -1 downto 0);
		Yo				   : out signed(WIDTH -1 downto 0);
		Zo				   : out signed(19 downto 0);
		sin_sign_inter_in  : in std_logic;
		cos_sign_inter_in  : in std_logic;
		sin_sign_inter_out : out std_logic;
		cos_sign_inter_out : out std_logic
	);
	end component p2r_CordicPipe;
 
 
	--
	--	ACHITECTURE BODY
	--
begin
	-- fill first nodes
quadrant <= Ain(15 downto 14);
process(quadrant,sin,cos,Ain)

	--- ___01___|___00___
    ---    10   |   11
begin
	CASE quadrant IS 
		when "00" =>
			thita <= Ain;
			sin_sign_inter(0) <= '0';
			cos_sign_inter(0) <= '0';
		when "01" =>
			thita <= "0111111111111000" - Ain;   -- thita = 180 - Ain
			sin_sign_inter(0) <= '0';
			cos_sign_inter(0) <= '1';
		when "10" =>
			thita <= Ain - "0111111111111000" ;
			sin_sign_inter(0) <= '1';
			cos_sign_inter(0) <= '1';
		when others =>
			thita <= "1111111111110000" - Ain;
			sin_sign_inter(0) <= '1';
			cos_sign_inter(0) <= '0';
		--when others => flag := '0' ;  
	end case;
end process;

	-- fill X
	X(0) <= x"4dba"; -- K = 1/P;
 
	-- fill Y
	Y(0) <= x"0000";
 
	-- fill Z
	Z(0)(19 downto 4) <= thita;
	Z(0)(3 downto 0) <= (others => '0');
 
	--
	-- generate pipeline
	--
	gen_pipe1: for n in 1 to PIPEID generate
		Pipe: p2r_CordicPipe generic map(WIDTH => WIDTH, PIPEID => n -1)
			port map ( clk,reset, ena, X(n-1), Y(n-1), Z(n-1), X(n), Y(n), Z(n), sin_sign_inter(n-1), cos_sign_inter(n-1), sin_sign_inter(n), cos_sign_inter(n) );
	end generate gen_pipe1;
 
	cos <= X(PIPEID);
	sin <= Y(PIPEID);
	sin_sign <= sin_sign_inter(PIPEID);
	cos_sign <= cos_sign_inter(PIPEID);
	cos_val <= cos;
	sin_val <= sin;

end dataflow;





----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------r2p_pre------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------


--
-- r2p_pre.vhd
--
-- Cordic pre-processing block
--
--
-- step 1:	determine quadrant and generate absolute value of X and Y
--		Q1: Xnegative
--		Q2: Ynegative
--
-- step 2:	swap X and Y values if Y>X
--		Q3: swapped (Y > X)
--
-- Rev. 1.1  June 4th, 2001. Richard Herveille. Revised entire core.
--
 
library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;
 
entity r2p_pre is
	port(
		clk	: in std_logic;
		ena	: in std_logic;
 
		Xi	: in signed(15 downto 0);
		Yi	: in signed(15 downto 0);
 
		Xo	: out unsigned(14 downto 0);
		Yo	: out unsigned(14 downto 0);
		Q	: out std_logic_vector(2 downto 0));
end entity r2p_pre;
 
architecture dataflow of r2p_pre is
	signal Xint1, Yint1		: unsigned(14 downto 0);
	signal Xneg, Yneg, swap	: std_logic;
 
begin
	--
	-- step 1: Determine absolute value of X and Y, set Xneg and Yneg
	--         Loose the sign-bit.
	Step1: process(clk, Xi, Yi)
	begin
		if (clk'event and clk = '1') then
			if (ena = '1') then
				Xint1 <= to_unsigned(to_integer(abs(Xi)),15);
				Xneg <= Xi(Xi'left);
 
				Yint1 <= to_unsigned(to_integer(abs(Yi)),15);--((unsigned(std_logic_vector(abs(Yi))))(14 downto 0));
				Yneg <= Yi(Yi'left);
			end if;
		end if;
	end process Step1;
 
	--
	-- step 2: Swap X and Y if Y>X
	--
	Step2: process(clk, Xint1, Yint1)
		variable Xint2, Yint2	: unsigned(14 downto 0);
	begin
		if (Yint1 > Xint1) then
			swap <= '1';
			Xint2 := Yint1;
			Yint2 := Xint1;
		else
			swap <= '0';
			Xint2 := Xint1;
			Yint2 := Yint1;
		end if;
 
		if(clk'event and clk = '1') then
			if (ena = '1') then
				Xo <= Xint2;
				Yo <= Yint2;
 
				Q <= (Yneg, Xneg, swap);
			end if;
		end if;
		end process Step2;
 
end architecture dataflow;






----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------r2p_CordicPipe-----------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------


--
-- file: r2p_CordicPipe.vhd
-- author: Richard Herveille
-- rev. 1.0 initial release
-- rev. 1.1 March 19th, 2001. Richard Herveille. Changed function Delta, it is compatible with Xilinx WebPack software now
-- rev. 1.2 May   18th, 2001. Richard Herveille. Added documentation to function ATAN (by popular request).
-- rev. 1.3 June   4th, 2001. Richard Herveille. Revised design (made it simpler and easier to understand). 
 
library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;
 
entity r2p_CordicPipe is 
	generic(
		WIDTH 	: natural := 16;
		PIPEID	: natural := 1
	);
	port(
		clk		: in std_logic;
		ena		: in std_logic;
 
		Xi		: in signed(WIDTH -1 downto 0); 
		Yi		: in signed(WIDTH -1 downto 0);
		Zi		: in signed(19 downto 0);
		Xo		: out signed(WIDTH -1 downto 0);
		Yo		: out signed(WIDTH -1 downto 0);
		Zo		: out signed(19 downto 0)
	);
end entity r2p_CordicPipe;
 
architecture dataflow of r2p_CordicPipe is
 
	--
	-- functions
	--
 
	-- Function CATAN (constante arc-tangent).
	-- This is a lookup table containing pre-calculated arc-tangents.
	-- 'n' is the number of the pipe, returned is a 20bit arc-tangent value.
	-- The numbers are calculated as follows: Z(n) = atan(1/2^n)
	-- examples:
	-- 20bit values => 2^20 = 2pi(rad)
	--                 1(rad) = 2^20/2pi = 166886.053....
	-- n:1, atan(1/2) = 0.4636...(rad)
	--      0.4636... * 166886.053... = 77376.32(dec) = 12E40(hex)
	-- n:2, atan(1/4) = 0.2449...(rad)
	--      0.2449... * 166886.053... = 40883.52(dec) = 9FB3(hex)
	-- n:3, atan(1/8) = 0.1243...(rad)
	--      0.1243... * 166886.053... = 20753.11(dec) = 5111(hex)
	--
	function CATAN(n :natural) return integer is
	variable result	:integer;
	begin
		case n is
			when 0 => result := 16#020000#;
			when 1 => result := 16#012E40#;
			when 2 => result := 16#09FB4#;
			when 3 => result := 16#05111#;
			when 4 => result := 16#028B1#;
			when 5 => result := 16#0145D#;
			when 6 => result := 16#0A2F#;
			when 7 => result := 16#0518#;
			when 8 => result := 16#028C#;
			when 9 => result := 16#0146#;
			when 10 => result := 16#0A3#;
			when 11 => result := 16#051#;
			when 12 => result := 16#029#;
			when 13 => result := 16#014#;
			when 14 => result := 16#0A#;
			when 15 => result := 16#05#;
			when 16 => result := 16#03#;
			when 17 => result := 16#01#;
			when others => result := 16#0#;
		end case;
		return result;
	end CATAN;
 
 
	function AddSub(dataa, datab : in signed; add_sub : in std_logic) return signed is
	begin
		if (add_sub = '1') then
			return dataa + datab;
		else
			return dataa - datab;
		end if;
	end;
 
	--
	--	ARCHITECTURE BODY
	--
	signal dX, Xresult	: signed(WIDTH -1 downto 0);
	signal dY, Yresult	: signed(WIDTH -1 downto 0);
	signal atan, Zresult	: signed(19 downto 0);
 
	signal Yneg, Ypos	: std_logic;
 
begin
 
	dX <= shift_right(Xi, PIPEID);
	dY <= shift_right(Yi, PIPEID);
	atan <= to_signed( catan(PIPEID), 20); -- Angle can not be negative, catan never returns a negative value, so conv_signed can be used
 
	-- generate adder structures
	Yneg <= Yi(WIDTH -1);
	Ypos <= not Yi(WIDTH -1);
 
	-- xadd
    Xresult <= AddSub(Xi, dY, YPos);
 
	-- yadd 
	Yresult <= AddSub(Yi, dX, Yneg);
 
	-- zadd
	Zresult <= AddSub(Zi, atan, Ypos);
 
	gen_regs: process(clk)
	begin
		if(clk'event and clk='1') then
			if (ena = '1') then
				Xo <= Xresult;
				Yo <= Yresult;
				Zo <= Zresult;
			end if;
		end if;
	end process;
 
end architecture dataflow;







----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------r2p_cordic---------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------



--
--	VHDL implementation of cordic algorithm
--
-- File: cordic.vhd
-- author: Richard Herveille
-- rev. 1.0 initial release
-- rev. 1.1 changed CordicPipe component declaration, Xilinx WebPack issue
-- rev. 1.2 Revised entire core. Made is simpler and easier to understand.
 
library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;
 
entity r2p_cordic is 
	generic(
		PIPELINE      : integer := 15;
		WIDTH         : integer := 16;
		EXT_PRECISION : integer := 4
	);
	port(
		clk	: in std_logic;
		ena : in std_logic;
 
		Xi : in signed(WIDTH-1 downto 0);
		Yi : in signed(WIDTH-1 downto 0);
		Zi : in signed(19 downto 0) := (others => '0');
 
		Xo : out signed(WIDTH + EXT_PRECISION -1 downto 0);
		Zo : out signed(19 downto 0)
	);
end r2p_cordic;
 
 
architecture dataflow of r2p_cordic is
 
	--
	--	TYPE defenitions
	--
	type XYVector is array(PIPELINE downto 0) of signed(WIDTH + EXT_PRECISION -1 downto 0);
	type ZVector is array(PIPELINE downto 0) of signed(19 downto 0);
 
	--
	--	COMPONENT declarations
	--
	component r2p_CordicPipe
	generic(
		WIDTH 	: natural := 16;
		PIPEID	: natural := 1
	);
	port(
		clk		: in std_logic;
		ena		: in std_logic;
 
		Xi		: in signed(WIDTH -1 downto 0); 
		Yi		: in signed(WIDTH -1 downto 0);
		Zi		: in signed(19 downto 0);
 
		Xo		: out signed(WIDTH -1 downto 0);
		Yo		: out signed(WIDTH -1 downto 0);
		Zo		: out signed(19 downto 0)
	);
	end component r2p_CordicPipe;
 
	--
	--	SIGNALS
	--
	signal X, Y	: XYVector;
	signal Z	: ZVector;
 
	--
	--	ACHITECTURE BODY
	--
begin
	-- fill first nodes
 
	X(0)(WIDTH + EXT_PRECISION -1 downto EXT_PRECISION) <= Xi;   -- fill MSBs with input data
	X(0)(EXT_PRECISION -1 downto 0) <= (others => '0');          -- fill LSBs with '0'
 
	Y(0)(WIDTH + EXT_PRECISION -1 downto EXT_PRECISION) <= Yi;   -- fill MSBs with input data
	Y(0)(EXT_PRECISION -1 downto 0) <= (others => '0');          -- fill LSBs with '0'
 
	Z(0) <= Zi;
 
	--
	-- generate pipeline
	--
	gen_pipe2:
	for n in 1 to PIPELINE generate
		Pipe: r2p_CordicPipe 
			generic map(WIDTH => WIDTH+EXT_PRECISION, PIPEID => n -1)
			port map ( clk, ena, X(n-1), Y(n-1), Z(n-1), X(n), Y(n), Z(n) );
	end generate gen_pipe2;
 
	--
	-- assign outputs
	--
	Xo <= X(PIPELINE);
	Zo <= Z(PIPELINE);
end dataflow;







----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------r2p_post---------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------



--
--	post.vhd
--
--	Cordic post-processing block
--
-- Compensate cordic algorithm K-factor; divide Radius by 1.6467, or multiply by 0.60725. 
-- Approximation:  Ra = Ri/2 + Ri/8 - Ri/64 - Ri/512
--                 Radius = Ra - Ra/4096 = Ri * 0.60727. This is a 0.0034% error.
-- Implementation: Ra = (Ri/2 + Ri/8) - (Ri/64 + Ri/512)
--                 Radius = Ra - Ra/4096
--	Position calculated angle in correct quadrant.
--
 
library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;
 
entity r2p_post is
	port(
		clk	: in std_logic;
		ena	: in std_logic;
 
		Ai	: in signed(19 downto 0);
		Ri	: in unsigned(19 downto 0);
		Q	: in std_logic_vector(2 downto 0);
 
		Ao	: out signed(19 downto 0);
		Ro	: out unsigned(19 downto 0));
end entity r2p_post;
 
architecture dataflow of r2p_post is
begin
	radius: block
		signal RadA, RadB, RadC : unsigned(19 downto 0);
	begin
		process(clk)
		begin
			if (clk'event and clk = '1') then
				if (ena = '1') then
					RadA <= ('0' & Ri(19 downto 1)) + ("000" & Ri(19 downto 3));
					RadB <= ("000000" & Ri(19 downto 6)) + ("000000000" & Ri(19 downto 9));
					RadC <= RadA - RadB;
 
					Ro <= RadC - RadC(19 downto 12);
				end if;
			end if;
		end process;
	end block radius;
 
	angle: block
		constant const_PI2 : signed(19 downto 0) := to_signed(16#40000#, 20); -- PI / 2
		constant const_PI : signed(19 downto 0) := to_signed(16#80000#, 20);  -- PI
		constant const_2PI : signed(19 downto 0) := (others => '0');            -- 2PI
 
		signal dQ : std_logic_vector(2 downto 1);
		signal ddQ : std_logic;
		signal AngStep1 : signed(19 downto 0);
		signal AngStep2 : signed(19 downto 0);
	begin
		angle_step1: process(clk, Ai, Q)
			variable overflow : std_logic;
			variable AngA, AngB, Ang : signed(19 downto 0);
		begin
			-- check if angle is negative, if so set it to zero
			overflow := Ai(19); --and Ai(18);
 
			if (overflow = '1') then
				AngA := (others => '0');
			else
				AngA := Ai;
			end if;
 
			-- step 1: Xabs and Yabs are swapped
			-- Calculated angle is the angle between vector and Y-axis.
			-- ActualAngle = PI/2 - CalculatedAngle
		 	AngB := const_PI2 - AngA;
 
			if (Q(0) = '1') then
				Ang := AngB;
			else
				Ang := AngA;
			end if;
 
			if (clk'event and clk = '1') then
				if (ena = '1') then
					AngStep1 <= Ang;
					dQ <= q(2 downto 1);
				end if;
			end if;
		end process angle_step1;
 
 
		angle_step2: process(clk, AngStep1, dQ)
			variable AngA, AngB, Ang : signed(19 downto 0);
		begin
			AngA := AngStep1;
 
			-- step 2: Xvalue is negative
			-- Actual angle is in the second or third quadrant
			-- ActualAngle = PI - CalculatedAngle
			AngB := const_PI - AngA;
 
			if (dQ(1) = '1') then
				Ang := AngB;
			else
				Ang := AngA;
			end if;
 
			if (clk'event and clk = '1') then
				if (ena = '1') then
					AngStep2 <= Ang;
					ddQ <= dQ(2);
				end if;
			end if;
		end process angle_step2;
 
 
		angle_step3: process(clk, AngStep2, ddQ)
			variable AngA, AngB, Ang : signed(19 downto 0);
		begin
			AngA := AngStep2;
 
			-- step 3: Yvalue is negative
			-- Actual angle is in the third or fourth quadrant
			-- ActualAngle = 2PI - CalculatedAngle
			AngB := const_2PI - AngA;
 
			if (ddQ = '1') then
				Ang := AngB;
			else
				Ang := AngA;
			end if;
 
			if (clk'event and clk = '1') then
				if (ena = '1') then
					Ao <= Ang;
				end if;
			end if;
		end process angle_step3;
	end block angle;
end;







----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------r2p_corproc--------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------------



--
-- file: r2p_corproc.vhd
--
-- XY to RA coordinate / rectangular to polar coordinates processor 
--
-- uses: r2p_pre.vhd, r2p_cordic.vhd, r2p_post.vhd
--
-- rev. 1.1 June 4th, 2001. Richard Herveille. Completely revised core.
--
 
library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;
 
entity r2p_corproc is
	port(
		clk	: in std_logic;
		ena	: in std_logic;
		Xin	: in signed(15 downto 0);
		Yin : in signed(15 downto 0);
 
		Rout	: out unsigned(19 downto 0);
		Aout	: out signed(19 downto 0)
	);
end entity r2p_corproc;
 
architecture dataflow of r2p_corproc is
	constant PipeLength : natural := 15;
	component r2p_pre is
	port(
		clk	: in std_logic;
		ena	: in std_logic;
		Xi	: in signed(15 downto 0);
		Yi	: in signed(15 downto 0);
 
		Xo	: out unsigned(14 downto 0);
		Yo	: out unsigned(14 downto 0);
		Q	: out std_logic_vector(2 downto 0)
	);
	end component r2p_pre;
 
	component r2p_cordic is
	generic(
		PIPELINE      : integer;
		WIDTH         : integer;
		EXT_PRECISION	: integer
	);
	port(
		clk	: in std_logic;
		ena	: in std_logic;
		Xi	: in signed(WIDTH-1 downto 0);
		Yi	: in signed(WIDTH-1 downto 0);
		Zi : in signed(19 downto 0) := (others => '0');		
 
		Xo	: out signed(WIDTH + EXT_PRECISION -1 downto 0);
		Zo	: out signed(19 downto 0));
	end component r2p_cordic;
 
	component r2p_post is
	port(
		clk	: in std_logic;
		ena	: in std_logic;
 
		Ai	: in signed(19 downto 0);
		Ri	: in unsigned(19 downto 0);
		Q	: in std_logic_vector(2 downto 0);
 
		Ao	: out signed(19 downto 0);
		Ro	: out unsigned(19 downto 0));
	end component r2p_post;
 
	signal Xpre, Ypre : unsigned(15 downto 0);
	signal Acor, Rcor : signed(19 downto 0);
	signal Q, dQ : std_logic_vector(2 downto 0);
 
begin
 
	-- instantiate components
	u1:	r2p_pre port map(clk => clk, ena => ena, Xi => Xin, Yi => Yin, Xo => Xpre(14 downto 0), Yo => Ypre(14 downto 0), Q => Q);
	Xpre(15) <= '0';
	Ypre(15) <= '0';
 
	u2:	r2p_cordic	
			generic map(PIPELINE => PipeLength, WIDTH => 16, EXT_PRECISION => 4)
			port map(clk => clk, ena => ena, Xi => signed(Xpre), Yi => signed(Ypre), Xo => Rcor, Zo => Acor);
 
	delay: block
		type delay_type is array(PipeLength -1 downto 0) of std_logic_vector(2 downto 0);
		signal delay_pipe :delay_type;
	begin
		process(clk, Q)
		begin
			if (clk'event and clk = '1') then
				if (ena = '1') then
					delay_pipe(0) <= Q;
					for n in 1 to PipeLength -1 loop
						delay_pipe(n) <= delay_pipe(n -1);
					end loop;
				end if;
			end if;
		end process;
		dQ <= delay_pipe(PipeLength -1);
	end block delay;
 
	u3:	r2p_post port map(clk => clk,  ena => ena, Ri => unsigned(Rcor), Ai => Acor, Q => dQ, Ao => Aout, Ro => Rout);
end architecture dataflow;








------------------------------------------------------------------------------
-- user_logic.vhd - entity/architecture pair
------------------------------------------------------------------------------
--
-- ***************************************************************************
-- ** Copyright (c) 1995-2012 Xilinx, Inc.  All rights reserved.            **
-- **                                                                       **
-- ** Xilinx, Inc.                                                          **
-- ** XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS"         **
-- ** AS A COURTESY TO YOU, SOLELY FOR USE IN DEVELOPING PROGRAMS AND       **
-- ** SOLUTIONS FOR XILINX DEVICES.  BY PROVIDING THIS DESIGN, CODE,        **
-- ** OR INFORMATION AS ONE POSSIBLE IMPLEMENTATION OF THIS FEATURE,        **
-- ** APPLICATION OR STANDARD, XILINX IS MAKING NO REPRESENTATION           **
-- ** THAT THIS IMPLEMENTATION IS FREE FROM ANY CLAIMS OF INFRINGEMENT,     **
-- ** AND YOU ARE RESPONSIBLE FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE      **
-- ** FOR YOUR IMPLEMENTATION.  XILINX EXPRESSLY DISCLAIMS ANY              **
-- ** WARRANTY WHATSOEVER WITH RESPECT TO THE ADEQUACY OF THE               **
-- ** IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OR        **
-- ** REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE FROM CLAIMS OF       **
-- ** INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS       **
-- ** FOR A PARTICULAR PURPOSE.                                             **
-- **                                                                       **
-- ***************************************************************************
--
------------------------------------------------------------------------------
-- Filename:          user_logic.vhd
-- Version:           1.00.a
-- Description:       User logic.
-- Date:              Sun Dec 17 14:15:02 2023 (by Create and Import Peripheral Wizard)
-- VHDL Standard:     VHDL'93
------------------------------------------------------------------------------
-- Naming Conventions:
--   active low signals:                    "*_n"
--   clock signals:                         "clk", "clk_div#", "clk_#x"
--   reset signals:                         "rst", "rst_n"
--   generics:                              "C_*"
--   user defined types:                    "*_TYPE"
--   state machine next state:              "*_ns"
--   state machine current state:           "*_cs"
--   combinatorial signals:                 "*_com"
--   pipelined or register delay signals:   "*_d#"
--   counter signals:                       "*cnt*"
--   clock enable signals:                  "*_ce"
--   internal version of output port:       "*_i"
--   device pins:                           "*_pin"
--   ports:                                 "- Names begin with Uppercase"
--   processes:                             "*_PROCESS"
--   component instantiations:              "<ENTITY_>I_<#|FUNC>"
------------------------------------------------------------------------------

-- DO NOT EDIT BELOW THIS LINE --------------------
library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;
use ieee.std_logic_unsigned.all;

library proc_common_v3_00_a;
use proc_common_v3_00_a.proc_common_pkg.all;

-- DO NOT EDIT ABOVE THIS LINE --------------------

--USER libraries added here

------------------------------------------------------------------------------
-- Entity section
------------------------------------------------------------------------------
-- Definition of Generics:
--   C_NUM_REG                    -- Number of software accessible registers
--   C_SLV_DWIDTH                 -- Slave interface data bus width
--
-- Definition of Ports:
--   Bus2IP_Clk                   -- Bus to IP clock
--   Bus2IP_Resetn                -- Bus to IP reset
--   Bus2IP_Data                  -- Bus to IP data bus
--   Bus2IP_BE                    -- Bus to IP byte enables
--   Bus2IP_RdCE                  -- Bus to IP read chip enable
--   Bus2IP_WrCE                  -- Bus to IP write chip enable
--   IP2Bus_Data                  -- IP to Bus data bus
--   IP2Bus_RdAck                 -- IP to Bus read transfer acknowledgement
--   IP2Bus_WrAck                 -- IP to Bus write transfer acknowledgement
--   IP2Bus_Error                 -- IP to Bus error response
------------------------------------------------------------------------------

entity user_logic is
  generic
  (
    -- ADD USER GENERICS BELOW THIS LINE ---------------
    --USER generics added here
    -- ADD USER GENERICS ABOVE THIS LINE ---------------

    -- DO NOT EDIT BELOW THIS LINE ---------------------
    -- Bus protocol parameters, do not add to or delete
    C_NUM_REG                      : integer              := 11;
    C_SLV_DWIDTH                   : integer              := 32
    -- DO NOT EDIT ABOVE THIS LINE ---------------------
  );
  port
  (
    -- ADD USER PORTS BELOW THIS LINE ------------------
    --USER ports added here
    -- ADD USER PORTS ABOVE THIS LINE ------------------

    -- DO NOT EDIT BELOW THIS LINE ---------------------
    -- Bus protocol ports, do not add to or delete
    Bus2IP_Clk                     : in  std_logic;
    Bus2IP_Resetn                  : in  std_logic;
    Bus2IP_Data                    : in  std_logic_vector(C_SLV_DWIDTH-1 downto 0);
    Bus2IP_BE                      : in  std_logic_vector(C_SLV_DWIDTH/8-1 downto 0);
    Bus2IP_RdCE                    : in  std_logic_vector(C_NUM_REG-1 downto 0);
    Bus2IP_WrCE                    : in  std_logic_vector(C_NUM_REG-1 downto 0);
    IP2Bus_Data                    : out std_logic_vector(C_SLV_DWIDTH-1 downto 0);
    IP2Bus_RdAck                   : out std_logic;
    IP2Bus_WrAck                   : out std_logic;
    IP2Bus_Error                   : out std_logic
    -- DO NOT EDIT ABOVE THIS LINE ---------------------
  );

  attribute MAX_FANOUT : string;
  attribute SIGIS : string;

  attribute SIGIS of Bus2IP_Clk    : signal is "CLK";
  attribute SIGIS of Bus2IP_Resetn : signal is "RST";

end entity user_logic;

------------------------------------------------------------------------------
-- Architecture section
------------------------------------------------------------------------------

architecture IMP of user_logic is

--------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------
component cordic is
	generic(
		WIDTH 	: natural := 16;
		PIPEID	: natural := 15
	);
	port(
		clk		: in std_logic;
		reset	: in std_logic;
		ena		: in std_logic;
		---------------------p2r module--------------------------
		Ain			: in signed(15 downto 0);
		
		sin_val		: out signed(15 downto 0);
		sin_sign	: out std_logic;	-- 0 --> positive    1 --> negative
		cos_val		: out signed(15 downto 0);
		cos_sign	: out std_logic;	-- 0 --> positive    1 --> negative
		
		
		---------------------r2p module--------------------------
		Xin		: in signed(15 downto 0);
		Yin 	: in signed(15 downto 0);
		
		Rout	: out unsigned(19 downto 0);
		Aout	: out signed(19 downto 0)
	);
end component cordic;

--------------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------------
  --USER signal declarations added here, as needed for user logic
  signal slv_reg00                       : std_logic;
  signal slv_reg11                       : std_logic;
  signal slv_reg22                       : signed (15 downto 0);
  signal slv_reg33                       : signed (15 downto 0);
  signal slv_reg44                       : std_logic;
  signal slv_reg55                       : signed (15 downto 0);
  signal slv_reg66                       : std_logic;
  signal slv_reg77                       : signed (15 downto 0);
  signal slv_reg88                       : signed (15 downto 0);
  signal slv_reg99                       : unsigned (19 downto 0);
  signal slv_reg1010                     : signed (19 downto 0);


  ------------------------------------------
  -- Signals for user logic slave model s/w accessible register example
  ------------------------------------------
  signal slv_reg0                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg1                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg2                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg3                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg4                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg5                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg6                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg7                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg8                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg9                       : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg10                      : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_reg_write_sel              : std_logic_vector(10 downto 0);
  signal slv_reg_read_sel               : std_logic_vector(10 downto 0);
  signal slv_ip2bus_data                : std_logic_vector(C_SLV_DWIDTH-1 downto 0);
  signal slv_read_ack                   : std_logic;
  signal slv_write_ack                  : std_logic;

begin

  --USER logic implementation added here

  ------------------------------------------
  -- Example code to read/write user logic slave model s/w accessible registers
  -- 
  -- Note:
  -- The example code presented here is to show you one way of reading/writing
  -- software accessible registers implemented in the user logic slave model.
  -- Each bit of the Bus2IP_WrCE/Bus2IP_RdCE signals is configured to correspond
  -- to one software accessible register by the top level template. For example,
  -- if you have four 32 bit software accessible registers in the user logic,
  -- you are basically operating on the following memory mapped registers:
  -- 
  --    Bus2IP_WrCE/Bus2IP_RdCE   Memory Mapped Register
  --                     "1000"   C_BASEADDR + 0x0
  --                     "0100"   C_BASEADDR + 0x4
  --                     "0010"   C_BASEADDR + 0x8
  --                     "0001"   C_BASEADDR + 0xC
  -- 
  ------------------------------------------
  slv_reg_write_sel <= Bus2IP_WrCE(10 downto 0);
  slv_reg_read_sel  <= Bus2IP_RdCE(10 downto 0);
  slv_write_ack     <= Bus2IP_WrCE(0) or Bus2IP_WrCE(1) or Bus2IP_WrCE(2) or Bus2IP_WrCE(3) or Bus2IP_WrCE(4) or Bus2IP_WrCE(5) or Bus2IP_WrCE(6) or Bus2IP_WrCE(7) or Bus2IP_WrCE(8) or Bus2IP_WrCE(9) or Bus2IP_WrCE(10);
  slv_read_ack      <= Bus2IP_RdCE(0) or Bus2IP_RdCE(1) or Bus2IP_RdCE(2) or Bus2IP_RdCE(3) or Bus2IP_RdCE(4) or Bus2IP_RdCE(5) or Bus2IP_RdCE(6) or Bus2IP_RdCE(7) or Bus2IP_RdCE(8) or Bus2IP_RdCE(9) or Bus2IP_RdCE(10);

  -- implement slave model software accessible register(s)
  SLAVE_REG_WRITE_PROC : process( Bus2IP_Clk ) is
  begin

    if Bus2IP_Clk'event and Bus2IP_Clk = '1' then
      if Bus2IP_Resetn = '0' then
        slv_reg0 <= (others => '0');
        slv_reg1 <= (others => '0');
        slv_reg2 <= (others => '0');
        
        slv_reg7 <= (others => '0');
        slv_reg8 <= (others => '0');
      
      else
        case slv_reg_write_sel is
          when "10000000000" =>
            for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              if ( Bus2IP_BE(byte_index) = '1' ) then
                slv_reg0(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              end if;
            end loop;
          when "01000000000" =>
            for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              if ( Bus2IP_BE(byte_index) = '1' ) then
                slv_reg1(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              end if;
            end loop;
          when "00100000000" =>
            for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              if ( Bus2IP_BE(byte_index) = '1' ) then
                slv_reg2(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              end if;
            end loop;
         
          when "00000001000" =>
            for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              if ( Bus2IP_BE(byte_index) = '1' ) then
                slv_reg7(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              end if;
            end loop;
          when "00000000100" =>
            for byte_index in 0 to (C_SLV_DWIDTH/8)-1 loop
              if ( Bus2IP_BE(byte_index) = '1' ) then
                slv_reg8(byte_index*8+7 downto byte_index*8) <= Bus2IP_Data(byte_index*8+7 downto byte_index*8);
              end if;
            end loop;


          when others => null;
        end case;
      end if;
    end if;

  end process SLAVE_REG_WRITE_PROC;

  -- implement slave model software accessible register(s) read mux
  SLAVE_REG_READ_PROC : process( slv_reg_read_sel, slv_reg0, slv_reg1, slv_reg2, slv_reg3, slv_reg4, slv_reg5, slv_reg6, slv_reg7, slv_reg8, slv_reg9, slv_reg10 ) is
  begin

    case slv_reg_read_sel is
      when "10000000000" => slv_ip2bus_data <= slv_reg0;
      when "01000000000" => slv_ip2bus_data <= slv_reg1;
      when "00100000000" => slv_ip2bus_data <= slv_reg2;
      when "00010000000" => slv_ip2bus_data <= slv_reg3;
      when "00001000000" => slv_ip2bus_data <= slv_reg4;
      when "00000100000" => slv_ip2bus_data <= slv_reg5;
      when "00000010000" => slv_ip2bus_data <= slv_reg6;
      when "00000001000" => slv_ip2bus_data <= slv_reg7;
      when "00000000100" => slv_ip2bus_data <= slv_reg8;
      when "00000000010" => slv_ip2bus_data <= slv_reg9;
      when "00000000001" => slv_ip2bus_data <= slv_reg10;
      when others => slv_ip2bus_data <= (others => '0');
    end case;

  end process SLAVE_REG_READ_PROC;

  ------------------------------------------
  -- Example code to drive IP to Bus signals

slv_reg00 <= slv_reg0(0);
slv_reg11 <= slv_reg1(0);

slv_reg22 <= signed(slv_reg2(15 downto 0));
slv_reg3(15 downto 0) <= std_logic_vector(slv_reg33);     slv_reg3(31 downto 16) <= (others => '0');
slv_reg4(0) <= slv_reg44;    				  slv_reg4(31 downto 1) <= (others => '0');
slv_reg5(15 downto 0) <= std_logic_vector(slv_reg55);     slv_reg5(31 downto 16) <= (others => '0');
slv_reg6(0) <= slv_reg66;    				  slv_reg6(31 downto 1) <= (others => '0');

slv_reg77 <= signed(slv_reg7(15 downto 0));
slv_reg88 <= signed(slv_reg8(15 downto 0));
slv_reg9(19 downto 0) <= std_logic_vector(slv_reg99);       slv_reg9(31 downto 20) <= (others => '0');
slv_reg10(19 downto 0) <= std_logic_vector(slv_reg1010);    slv_reg10(31 downto 20) <= (others => '0');

cordic_inst:cordic generic map ( WIDTH => 16,PIPEID => 15)  port map (Bus2IP_Clk,slv_reg00,slv_reg11 ,slv_reg22 ,slv_reg33 ,slv_reg44 ,slv_reg55 ,slv_reg66 ,slv_reg77 ,slv_reg88 ,slv_reg99 ,slv_reg1010 );


  ------------------------------------------
  IP2Bus_Data  <= slv_ip2bus_data when slv_read_ack = '1' else
                  (others => '0');

  IP2Bus_WrAck <= slv_write_ack;
  IP2Bus_RdAck <= slv_read_ack;
  IP2Bus_Error <= '0';

end IMP;
