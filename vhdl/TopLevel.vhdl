--
-- Copyright (C) 2009-2011 Chris McClelland
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

entity TopLevel is
	port(
		reset_in     : in std_logic;
		ifclk_in     : in std_logic;

		-- Unused connections must be configured as inputs
		flagA_in     : in std_logic;
		flagB_in     : in std_logic;
		int0_in      : in std_logic;                     -- PA0
		int1_in      : in std_logic;                     -- PA1
		pa3_in       : in std_logic;                     -- PA3
		pa7_in       : in std_logic;                     -- PA7
		clkout_in    : in std_logic;
		pd_in        : in std_logic_vector(7 downto 0);
		dummy_out    : out std_logic;                    -- Dummy output, not connected to FX2

		-- Data & control from the FX2
		fifodata_io  : inout std_logic_vector(7 downto 0);
		gotdata_in   : in std_logic;                     -- FLAGC=EF (active-low), so '1' when there's data

		-- Control to the FX2
		sloe_out     : out std_logic;                    -- PA2
		slrd_out     : out std_logic;
		slwr_out     : out std_logic;
		fifoaddr_out : out std_logic_vector(1 downto 0); -- PA4 & PA5
		pktend_out   : out std_logic;                    -- PA6

		-- Onboard peripherals
		sseg_out     : out std_logic_vector(7 downto 0);
		anode_out    : out std_logic_vector(3 downto 0);
		led_out      : out std_logic_vector(7 downto 0)
	);
end TopLevel;

architecture Behavioural of TopLevel is
	type StateType is (
		STATE_IDLE,
		STATE_READ,
		STATE_WRITE
	);
	signal state, state_next       : StateType;
	signal wcount, wcount_next     : unsigned(3 downto 0);  -- Write count
	signal checksum, checksum_next : unsigned(15 downto 0);
	signal led, led_next           : std_logic_vector(7 downto 0);
	constant OUT_FIFO              : std_logic_vector(1 downto 0) := "10"; -- EP6OUT
	constant IN_FIFO               : std_logic_vector(1 downto 0) := "11"; -- EP8IN
begin
	process(ifclk_in, reset_in)
	begin
		if ( reset_in = '1' ) then
			state    <= STATE_IDLE;
			checksum <= (others => '0');
			led      <= (others => '0');
			wcount   <= (others => '0');
		elsif ( ifclk_in'event and ifclk_in = '1' ) then
			state    <= state_next;
			checksum <= checksum_next;
			led      <= led_next;
			wcount   <= wcount_next;
		end if;
	end process;

	-- Next state logic
	process(state, fifodata_io, gotdata_in, checksum, led, wcount)
	begin
		state_next    <= STATE_IDLE;
		checksum_next <= checksum;
		led_next      <= led;
		wcount_next   <= wcount;
		fifodata_io   <= (others => 'Z');  -- Tristated unless explicitly driven
		case state is
			when STATE_IDLE =>
				fifoaddr_out <= OUT_FIFO;
				sloe_out     <= '0';
				slrd_out     <= '0';
				slwr_out     <= '1';
				pktend_out   <= '1';
				if ( gotdata_in = '1' ) then
					-- There will be some data available on the next clock edge.
					if ( fifodata_io = x"FF" ) then
						state_next    <= STATE_IDLE;  -- NOP
					elsif ( fifodata_io = x"FE" ) then
						state_next    <= STATE_WRITE;
						wcount_next   <= x"9";
					else
						state_next    <= STATE_READ;
						checksum_next <= checksum + unsigned(fifodata_io);
						led_next      <= fifodata_io;
					end if;
				else
					state_next <= STATE_IDLE;
				end if;

			when STATE_READ =>
				fifoaddr_out <= OUT_FIFO;
				sloe_out     <= '0';
				slrd_out     <= '0';
				slwr_out     <= '1';
				pktend_out   <= '1';
				if ( gotdata_in = '1' ) then
					-- There will be some data available on the next clock edge.
					state_next    <= STATE_READ;
					checksum_next <= checksum + unsigned(fifodata_io);
					led_next      <= fifodata_io;
				else
					state_next <= STATE_IDLE;
				end if;

			when STATE_WRITE =>
				fifoaddr_out <= IN_FIFO;
				fifodata_io  <= x"A9";
				sloe_out     <= '1';
				slrd_out     <= '1';
				slwr_out     <= '0';
				pktend_out   <= '1';
				wcount_next  <= wcount - 1;
				if ( wcount = 0 ) then
					pktend_out <= '0';
					state_next <= STATE_IDLE;
				else
					state_next <= STATE_WRITE;
				end if;
		end case;
	end process;
	
	-- Mop up all the unused inputs to prevent synthesis warnings
	dummy_out <=
		clkout_in and flagA_in and flagB_in and int0_in and int1_in and pa3_in and pa7_in
		and pd_in(0) and pd_in(1) and pd_in(2) and pd_in(3)
		and pd_in(4) and pd_in(5) and pd_in(6) and pd_in(7);
	
	led_out     <= led;
	sseg_out(7) <= '1';  -- Decimal point off

	sevenSeg : entity work.SevenSeg
		port map(
			clk    => ifclk_in,
			data   => std_logic_vector(checksum),
			segs   => sseg_out(6 downto 0),
			anodes => anode_out
		);

end Behavioural;
