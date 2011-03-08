--
-- Copyright (C) 2009 Chris McClelland
--
-- This program is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program.  If not, see <http://www.gnu.org/licenses/>.
--
library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity fx2fpga is
	generic(
		COUNTER_WIDTH: natural := 18
	);
	port(
		reset    : in std_logic;
		ifclk    : in std_logic;

		-- Unused connections must be configured as inputs
		flagA    : in std_logic;
		flagB    : in std_logic;
		int0     : in std_logic;                     -- PA0
		int1     : in std_logic;                     -- PA1
		pa3      : in std_logic;                     -- PA3
		pa7      : in std_logic;                     -- PA7
		clkout   : in std_logic;
		pd       : in std_logic_vector(7 downto 0);
		dummy    : out std_logic;                    -- Dummy output, not connected to FX2

		-- Data & control from the FX2
		fd       : inout std_logic_vector(7 downto 0);
		gotData  : in std_logic;                     -- FLAGC

		-- Control to the FX2
		sloe     : out std_logic;                    -- PA2
		slrd     : out std_logic;
		slwr     : out std_logic;
		fifoAddr : out std_logic_vector(1 downto 0); -- PA4 & PA5
		pktEnd   : out std_logic;                    -- PA6

		-- Onboard peripherals
		sseg     : out std_logic_vector(7 downto 0);
		anode    : out std_logic_vector(3 downto 0);
		sw       : in std_logic_vector(2 downto 0);
		led_out  : out std_logic_vector(7 downto 0)
	);
end fx2fpga;

architecture arch of fx2fpga is
	type StateType is (
		STATE_IDLE,
		STATE_READ,
		STATE_WRITE
	);
	signal state, state_next       : StateType;
	signal counter, counter_next   : unsigned(COUNTER_WIDTH-1 downto 0);
	signal hex                     : std_logic_vector(3 downto 0);
	signal checksum, checksum_next : unsigned(15 downto 0);
	signal led, led_next           : std_logic_vector(7 downto 0);
	signal wcount, wcount_next     : unsigned(3 downto 0);
begin
	process(ifclk, reset)
	begin
		if ( reset = '1' ) then
			state    <= STATE_IDLE;
			checksum <= (others => '0');
			led      <= (others => '0');
			counter  <= (others => '0');
			wcount   <= (others => '0');
		elsif ( ifclk'event and ifclk = '1' ) then
			state    <= state_next;
			checksum <= checksum_next;
			led      <= led_next;
			counter  <= counter_next;
			wcount   <= wcount_next;
		end if;
	end process;

	-- Next state logic
	process(state, fd, gotData, checksum, led, wcount)
	begin
		state_next    <= STATE_IDLE;
		checksum_next <= checksum;
		led_next      <= led;
		wcount_next   <= wcount;
		fd            <= (others => 'Z');
		case state is
			when STATE_IDLE =>
				fifoAddr <= "10"; -- EP6OUT
				sloe     <= '0';
				slrd     <= '0';
				slwr     <= '1';
				pktEnd   <= '1';
				if ( gotData = '1' ) then
					-- There will be some data available on the next clock edge.
					if ( fd = x"FF" ) then
						state_next <= STATE_IDLE;  -- NOP
					elsif ( fd = x"FE" ) then
						state_next <= STATE_WRITE;
						wcount_next <= x"9";
					else
						state_next <= STATE_READ;
						checksum_next <= checksum + unsigned(fd);
						led_next <= fd;
					end if;
				else
					state_next <= STATE_IDLE;
				end if;
			when STATE_READ =>
				fifoAddr <= "10"; -- EP6OUT
				sloe     <= '0';
				slrd     <= '0';
				slwr     <= '1';
				pktEnd   <= '1';
				if ( gotData = '1' ) then
					-- There will be some data available on the next clock edge.
					state_next <= STATE_READ;
					checksum_next <= checksum + unsigned(fd);
					led_next <= fd;
				else
					state_next <= STATE_IDLE;
				end if;

			when STATE_WRITE =>
				fifoAddr    <= "00"; -- EP2IN
				fd          <= x"A9";
				sloe        <= '1';
				slrd        <= '1';
				slwr        <= '0';
				pktEnd      <= '1';
				wcount_next <= wcount - 1;
				if ( wcount = 0 ) then
					pktEnd      <= '0';
					state_next <= STATE_IDLE;
				else
					state_next <= STATE_WRITE;
				end if;
		end case;
	end process;
	
	-- binary counter
	counter_next <= counter + 1;
	
	-- Tri-stating doesn't seem to work...set them all as inputs
	dummy <=
		flagA and flagB and int0 and int1 and pa3 and pa7 and clkout and
		pd(0) and pd(1) and pd(1) and pd(2) and pd(3) and
		pd(4) and pd(5) and pd(6) and pd(7) and sw(0) and sw(1) and sw(2);
	
	led_out <= led;

	--fifoAddr <= sw(1 downto 0) when sw(2) = '1' else (others => 'Z');
	--sloe <= '0'                when sw(2) = '1' else 'Z';
	--slrd <= '0'                when sw(2) = '1' else 'Z';
	--slwr <= '1'                when sw(2) = '1' else 'Z';
	--pktEnd <= '1'              when sw(2) = '1' else 'Z';

	-- process to choose which 7-seg display to light
	process(counter(17 downto 16), checksum)
	begin
		case counter(17 downto 16) is
			when "00" =>
				anode <= "1110";
				hex <= std_logic_vector(checksum(3 downto 0));
				sseg(7) <= '1';
			when "01" =>
				anode <= "1101";
				hex <= std_logic_vector(checksum(7 downto 4));
				sseg(7) <= '1';
			when "10" =>
				anode <= "1011";
				hex <= std_logic_vector(checksum(11 downto 8));
				sseg(7) <= '1';
			when others =>
				anode <= "0111";
				hex <= std_logic_vector(checksum(15 downto 12));
				sseg(7) <= '1';
		end case;
	end process;

	-- combinatorial logic to display the correct pattern based
	-- on the output of the selector process above.
	with hex select
		sseg(6 downto 0) <=
			"0000001" when "0000",
			"1001111" when "0001",
			"0010010" when "0010",
			"0000110" when "0011",
			"1001100" when "0100",
			"0100100" when "0101",
			"0100000" when "0110",
			"0001111" when "0111",
			"0000000" when "1000",
			"0000100" when "1001",
			"0001000" when "1010",  -- a
			"1100000" when "1011",  -- b
			"0110001" when "1100",  -- c
			"1000010" when "1101",  -- d
			"0110000" when "1110",  -- e
			"0111000" when others;  -- f
end arch;
