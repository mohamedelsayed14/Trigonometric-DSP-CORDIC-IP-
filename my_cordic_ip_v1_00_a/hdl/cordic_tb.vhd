--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   10:00:05 12/15/2023
-- Design Name:   
-- Module Name:   /home/ise/Desktop/fpga_FP/cordic/cordic_tb.vhd
-- Project Name:  cordic
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: cordic
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY cordic_tb IS
END cordic_tb;
 
ARCHITECTURE behavior OF cordic_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT cordic
    PORT(
         clk : IN  std_logic;
         reset: IN  std_logic;
         ena : IN  std_logic;
         Ain : IN  signed(15 downto 0);
         sin_val : OUT  signed(15 downto 0);
         sin_sign : OUT  std_logic;
         cos_val : OUT signed(15 downto 0);
         cos_sign : OUT  std_logic;

         Xin : IN  signed(15 downto 0);
         Yin : IN  signed(15 downto 0);
         Rout : OUT  unsigned(19 downto 0);
         Aout : OUT signed(19 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic ;
   signal reset : std_logic ;
   signal ena : std_logic ;
   signal Ain : signed(15 downto 0);
   signal Xin : signed(15 downto 0);
   signal Yin : signed(15 downto 0);

 	--Outputs
   signal sin_val : signed(15 downto 0);
   signal sin_sign : std_logic;
   signal cos_val : signed(15 downto 0);
   signal cos_sign : std_logic;
   signal Rout : unsigned(19 downto 0);
   signal Aout : signed(19 downto 0);

   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: cordic PORT MAP (
          clk => clk,
          reset=>reset,
          ena => ena,
          Ain => Ain,
          sin_val => sin_val,
          sin_sign => sin_sign,
          cos_val => cos_val,
          cos_sign => cos_sign,
          Xin => Xin,
          Yin => Yin,
          Rout => Rout,
          Aout => Aout
        );

   -- Clock process definitions
   clk_process :process
   begin
		clk <= '0';
		wait for clk_period/2;
		clk <= '1';
		wait for clk_period/2;
   end process;



   -- Stimulus process
   stim_proc: process
   begin		
    -- insert stimulus here 
	-- polar to rect simulation 
         reset<='1';
         ena<='0';
		 wait for clk_period;
		 reset<='0';
         ena<='1';

	 Ain <= to_signed(3640,16);
         wait for clk_period;
         Ain <= to_signed(29120,16);
         wait for clk_period;
         Ain <= to_signed(36400,16);
         wait for clk_period;
         Ain <= to_signed(61880,16);

	 wait for 13*clk_period;
         ASSERT (sin_val = to_signed(11207,16)) REPORT "wrong sin_val, expected sin_val=11207" SEVERITY warning;
         ASSERT (sin_sign = '0') REPORT "wrong sin_sign, expected sin_sign=0" SEVERITY warning;
         ASSERT (cos_val = to_signed(30788,16)) REPORT "wrong cos_val, expected cos_val=30788" SEVERITY warning;
         ASSERT (cos_sign = '0') REPORT "wrong cos_sign, expected cos_sign=0" SEVERITY warning;

	wait for 14*clk_period;
         ASSERT (sin_val = to_signed(11207,16)) REPORT "wrong sin_val, expected sin_val=11207" SEVERITY warning;
         ASSERT (sin_sign = '0') REPORT "wrong sin_sign, expected sin_sign=0" SEVERITY warning;
         ASSERT (cos_val = to_signed(30788,16)) REPORT "wrong cos_val, expected cos_val=30788" SEVERITY warning;
         ASSERT (cos_sign = '1') REPORT "wrong cos_sign, expected cos_sign=1" SEVERITY warning;

	wait for 15*clk_period;
         ASSERT (sin_val = to_signed(11207,16)) REPORT "wrong sin_val, expected sin_val=11207" SEVERITY warning;
         ASSERT (sin_sign = '1') REPORT "wrong sin_sign, expected sin_sign=1" SEVERITY warning;
         ASSERT (cos_val = to_signed(30788,16)) REPORT "wrong cos_val, expected cos_val=-30788" SEVERITY warning;
         ASSERT (cos_sign = '1') REPORT "wrong cos_sign, expected cos_sign=1" SEVERITY warning;

	wait for 16*clk_period;
         ASSERT (sin_val = to_signed(11207,16)) REPORT "wrong sin_val, expected sin_val=11207" SEVERITY warning;
         ASSERT (sin_sign = '1') REPORT "wrong sin_sign, expected sin_sign=1" SEVERITY warning;
         ASSERT (cos_val = to_signed(30788,16)) REPORT "wrong cos_val, expected cos_val=30788" SEVERITY warning;
         ASSERT (cos_sign = '0') REPORT "wrong cos_sign, expected cos_sign=0" SEVERITY warning;
			


      wait;
   end process;

END;
