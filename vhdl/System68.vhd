--===========================================================================--
--
--  S Y N T H E Z I A B L E    System68   System On a Chip
--
--  www.OpenCores.Org - December 2002
--  This core adheres to the GNU public license  
--
-- File name      : system68.vhd
--
-- Purpose        : Top level file for a 6800 compatible system on a chip
--                  Designed for the Burch ED B3-Spartan II board with
--                  X2S200 FPGA, 128 x 16 Word SRAM module and CPU I/O module
--                  Using mimiUart from open cores modified to look like a 6850
--                  
-- Dependencies   : ieee.Std_Logic_1164
--                  ieee.std_logic_unsigned
--                  ieee.std_logic_arith
--                  ieee.numeric_std
--
-- Uses           : miniuart.vhd, rxunit.vhd, tx_unit.vhd, clkunit.vhd
--                  swtbug.vhd (6800 SWTBUG ROM)
--                  datram.vhd (Dynamic address translation registers)
--                  ioport.vhd (4 x 8 port parallel I/O )
--                  cpu68.vhd  (6800 compatible CPU core)
--                  timer.vhd  (timer module)
--
-- Author         : John E. Kent      
--
--===========================================================================----
--
-- Revision History:
--===========================================================================--
--
-- Date:                Revision   Author
-- 22 September 2002    0.1        John Kent
-- Initial design.
--
-------------------------------------------------------------------------------
--
-- Memory Map:
--
-- $0000 - $7FFF RAM
-- $8000 - $9FFF IO
--     $8000 - $8003 Timer
--     $8004 - $8007 MiniUart / Acia
--     $8008 - $800F IO Port
-- $A000 - $DFFF RAM
-- $E000 - $FFFF ROM (read) & DAT (write)
--
library ieee;
   use ieee.std_logic_1164.all;
   use IEEE.STD_LOGIC_ARITH.ALL;
   use IEEE.STD_LOGIC_UNSIGNED.ALL;
   use ieee.numeric_std.all;

entity System68 is
  port(
    SysClk      : in  Std_Logic;  -- System Clock input
	 Reset_n     : in  Std_logic;  -- Master Reset input (active low)
    LED         : out std_logic;  -- Diagnostic LED Flasher

    -- Memory Interface signals
    ram_csn     : out Std_Logic;  -- RAM Chip select (active low)
    ram_wrln    : out Std_Logic;  -- lower byte write strobe (active low)
    ram_wrun    : out Std_Logic;  -- upper byte write strobe (active low)
    ram_addr    : out Std_Logic_Vector(16 downto 0);   -- RAM Address bus
    ram_data    : inout Std_Logic_Vector(15 downto 0); -- RAM Data bus

	 -- Stuff on the peripheral board
--  aux_clock   : in  Std_Logic;  -- FPGA-CPU-IO clock

	 -- PS/2 Mouse interface
--	 mouse_clock : in  Std_Logic;
--	 mouse_data  : in  Std_Logic;

	 -- Uart Interface
    rxbit       : in  Std_Logic; -- UART receive data
	 txbit       : out Std_Logic; -- UART transmit data
    rts_n       : out Std_Logic; -- Request to send (active low)
    cts_n       : in  Std_Logic; -- Clear to send (active low)

	 -- CRTC output signals
	 -- Signal defined on B3-CPU-IO Module
	 -- Not currently used.
	 v_drive     : out Std_Logic;
    h_drive     : out Std_Logic;
    blue_lo     : out std_logic;
    blue_hi     : out std_logic;
    green_lo    : out std_logic;
    green_hi    : out std_logic;
    red_lo      : out std_logic;
    red_hi      : out std_logic;
	 buzzer      : out std_logic;

-- I/O Ports
    PortA        : inout std_logic_vector(7 downto 0);
    PortB        : inout std_logic_vector(7 downto 0);
    PortC        : inout std_logic_vector(7 downto 0);
    PortD        : inout std_logic_vector(7 downto 0);

-- Timer I/O
	 timer_out    : out std_logic;

-- Test Pins
    uart_csn    : out std_logic;  -- Uart chip select out (active low)
    test_rw     : out std_logic;  -- Read / Write signal
	 test_d0     : out std_logic;  -- Uart Chip select ANDed with Receive Data Ready bit
	 test_d1     : out std_logic;  -- Uart Chip select ANDed with Transmit Data Ready bit

	 test_alu    : out std_logic_vector(15 downto 0); -- ALU output for timing constraints
	 test_cc     : out std_logic_vector(7 downto 0)   -- Condition Code Outputs for timing constraints
	 );
end System68;

-------------------------------------------------------------------------------
-- Architecture for memio Controller Unit
-------------------------------------------------------------------------------
architecture my_computer of System68 is
  -----------------------------------------------------------------------------
  -- Signals
  -----------------------------------------------------------------------------
  -- BOOT ROM
  signal rom_data_out  : Std_Logic_Vector(7 downto 0);

  -- UART Interface signals
  signal uart_data_out : Std_Logic_Vector(7 downto 0);  
  signal uart_cs       : Std_Logic;
  signal uart_irq      : Std_Logic;

  -- timer
  signal timer_data_out : std_logic_vector(7 downto 0);
  signal timer_cs    : std_logic;
  signal timer_irq   : std_logic;

  -- i/o port
  signal ioport_data_out : std_logic_vector(7 downto 0);
  signal ioport_cs   : std_logic;

  -- RAM
  signal ram_cs      : std_logic; -- memory chip select
  signal ram_wrl     : std_logic; -- memory write lower
  signal ram_wru     : std_logic; -- memory write upper
  signal ram_data_out    : std_logic_vector(7 downto 0);

  -- CPU Interface signals
  signal cpu_reset   : Std_Logic;
  signal cpu_clk     : Std_Logic;
  signal cpu_rw      : std_logic;
  signal cpu_vma     : std_logic;
  signal cpu_irq     : std_logic;
  signal cpu_nmi     : std_logic;
  signal cpu_addr    : Std_Logic_Vector(15 downto 0);
  signal cpu_data_in : Std_Logic_Vector(7 downto 0);
  signal cpu_data_out: Std_Logic_Vector(7 downto 0);

  -- Dynamic Address Translation RAM
  signal dat_cs      : std_logic;
  signal dat_addr    : std_logic_vector(7 downto 0);

  -- Flashing Led test signals
  signal countL      : std_logic_vector(23 downto 0);


-----------------------------------------------------------------
--
-- Open Cores Mini UART
--
-----------------------------------------------------------------

component miniUART is
  port (
     SysClk   : in  Std_Logic;  -- System Clock
     rst      : in  Std_Logic;  -- Reset input
     cs       : in  Std_Logic;
     rw       : in  Std_Logic;
     RxD      : in  Std_Logic;
     TxD      : out Std_Logic;
     CTS_n    : in  Std_Logic;
     RTS_n    : out Std_Logic;
     Irq      : out Std_logic;
     Addr     : in  Std_Logic;
     DataIn   : in  Std_Logic_Vector(7 downto 0); -- 
     DataOut  : out Std_Logic_Vector(7 downto 0)); -- 
end component;

--------------------------------------
--
-- Three port parallel I/O
--
---------------------------------------

component ioport is
  port (
     clk      : in std_logic;
	  rst      : in std_logic;
	  cs       : in std_logic;
	  rw       : in std_logic;
	  addr     : in std_logic_vector(2 downto 0);
	  data_in  : in std_logic_vector(7 downto 0);
	  data_out : out std_logic_vector(7 downto 0);
	  porta_io : inout std_logic_vector(7 downto 0);
	  portb_io : inout std_logic_vector(7 downto 0);
	  portc_io : inout std_logic_vector(7 downto 0);
	  portd_io : inout std_logic_vector(7 downto 0)
	  );
end component;

----------------------------------------
--
-- Timer module
--
----------------------------------------

component timer is
  port (
     clk       : in std_logic;
	  rst       : in std_logic;
	  cs        : in std_logic;
	  rw        : in std_logic;
	  addr      : in std_logic;
	  data_in   : in std_logic_vector(7 downto 0);
	  data_out  : out std_logic_vector(7 downto 0);
	  irq       : out std_logic;
     timer_in  : in std_logic;
	  timer_out : out std_logic
	  );
end component timer;

component cpu68 is
  port (    
	 clk:	     in	std_logic;
    rst:      in	std_logic;
    rw:	     out	std_logic;		-- Asynchronous memory interface
    vma:	     out	std_logic;
    address:  out	std_logic_vector(15 downto 0);
    data_in:  in	std_logic_vector(7 downto 0);
	 data_out: out std_logic_vector(7 downto 0);
	 irq:      in  std_logic;
	 nmi:      in  std_logic;
	 test_alu: out std_logic_vector(15 downto 0);
	 test_cc:  out std_logic_vector(7 downto 0)
  );
end component cpu68;

component dat_ram is
  port (
    clk:      in  std_logic;
	 rst:      in  std_logic;
	 cs:       in  std_logic;
	 rw:       in  std_logic;
	 addr_lo:  in  std_logic_vector(3 downto 0);
	 addr_hi:  in  std_logic_vector(3 downto 0);
    data_in:  in  std_logic_vector(7 downto 0);
	 data_out: out std_logic_vector(7 downto 0)
	 );
end component dat_ram;

component boot_rom is
  port (
    addr  : in  Std_Logic_Vector(9 downto 0);  -- 1K byte boot rom
	 data  : out Std_Logic_Vector(7 downto 0)
  );
end component boot_rom;

begin
  -----------------------------------------------------------------------------
  -- Instantiation of internal components
  -----------------------------------------------------------------------------

my_uart  : miniUART port map (
    SysClk    => SysClk,
	 rst       => cpu_reset,
    cs        => uart_cs,
	 rw        => cpu_rw,
	 RxD       => rxbit,
	 TxD       => txbit,
	 CTS_n     => cts_n,
	 RTS_n     => rts_n,
    Irq       => uart_irq,
    Addr      => cpu_addr(0),
	 Datain    => cpu_data_out,
	 DataOut   => uart_data_out
	 );

my_ioport  : ioport port map (
    clk       => SysClk,
	 rst       => cpu_reset,
    cs        => ioport_cs,
	 rw        => cpu_rw,
    addr      => cpu_addr(2 downto 0),
	 data_in   => cpu_data_out,
	 data_out  => ioport_data_out,
	 porta_io  => porta,
	 portb_io  => portb,
	 portc_io  => portc,
	 portd_io  => portd
    );

my_timer  : timer port map (
    clk       => SysClk,
	 rst       => cpu_reset,
    cs        => timer_cs,
	 rw        => cpu_rw,
    addr      => cpu_addr(0),
	 data_in   => cpu_data_out,
	 data_out  => timer_data_out,
    irq       => timer_irq,
	 timer_in  => CountL(5),
	 timer_out => timer_out
    );

my_cpu : cpu68  port map (    
	 clk	     => SysClk,
    rst       => cpu_reset,
    rw	     => cpu_rw,
    vma       => cpu_vma,
    address   => cpu_addr(15 downto 0),
    data_in   => cpu_data_in,
	 data_out  => cpu_data_out,
	 irq       => cpu_irq,
	 nmi       => cpu_nmi,
	 test_alu  => test_alu,
	 test_cc   => test_cc
  );


my_dat : dat_ram port map (
    clk        => SysClk,
	 rst        => cpu_reset,
	 cs         => dat_cs,
	 rw         => cpu_rw,
	 addr_hi    => cpu_addr(15 downto 12),
	 addr_lo    => cpu_addr(3 downto 0),
    data_in    => cpu_data_out,
	 data_out   => dat_addr(7 downto 0)
	 );

my_rom : boot_rom port map (
	 addr       => cpu_addr(9 downto 0),
    data       => rom_data_out
	 );
	 
----------------------------------------------------------------------
--
--  Processes to read and write memory based on bus signals
--
----------------------------------------------------------------------

memory: process( SysClk, Reset_n,
                 cpu_addr, cpu_rw, cpu_vma, cpu_data_out,
                 ram_cs, ram_wrl, ram_wru, dat_addr,
					  rom_data_out, ram_data_out,
					  ioport_data_out, timer_data_out, uart_data_out )
begin
    case cpu_addr(15 downto 13) is
		when "111" => -- $E000 - $FFFF
 		   cpu_data_in <= rom_data_out;
			dat_cs    <= cpu_vma;
			ram_cs    <= '0';
			uart_cs   <= '0';
			ioport_cs <= '0';
			timer_cs  <= '0';
		when "100" => -- $8000 - $9FFF
			dat_cs    <= '0';
			ram_cs    <= '0';
		   case cpu_addr(5 downto 2) is
			when "0000" => -- $8000
           cpu_data_in <= timer_data_out;
			  uart_cs   <= '0';
			  ioport_cs <= '0';
           timer_cs  <= cpu_vma;
			when "0001" => -- $8004
		     cpu_data_in <= uart_data_out;
			  uart_cs     <= cpu_vma;
			  ioport_cs   <= '0';
			  timer_cs    <= '0';
			when "0010" | "0011" => -- $8008/$800C
           cpu_data_in <= ioport_data_out;
			  uart_cs     <= '0';
           ioport_cs   <= cpu_vma;
			  timer_cs    <= '0';
			when others => -- $8010 to $9FFF
           cpu_data_in <= "00000000";
			  uart_cs     <= '0';
			  ioport_cs   <= '0';
			  timer_cs    <= '0';
		   end case;
		when others =>
		  cpu_data_in <= ram_data_out;
		  ram_cs     <= cpu_vma;
		  dat_cs     <= '0';
		  uart_cs    <= '0';
		  ioport_cs  <= '0';
		  timer_cs   <= '0';
	 end case;
 	 cpu_reset <= not Reset_n; -- CPU reset is active high
    ram_csn <= not( ram_cs and Reset_n );
	 ram_wrl  <= (not dat_addr(5)) and (not cpu_rw) and SysClk;
	 ram_wrln <= not ram_wrl;
    ram_wru  <= dat_addr(5) and (not cpu_rw) and SysClk;
	 ram_wrun <= not ram_wru;
	 ram_addr(16 downto 12) <= dat_addr(4 downto 0);
	 ram_addr(11 downto 0) <= cpu_addr(11 downto 0);

    if ram_wrl = '1' then
		ram_data(7 downto 0) <= cpu_data_out;
	 else
      ram_data(7 downto 0)  <= "ZZZZZZZZ";
	 end if;

	 if ram_wru = '1' then
		ram_data(15 downto 8) <= cpu_data_out;
	 else
      ram_data(15 downto 8)  <= "ZZZZZZZZ";
    end if;

	 if dat_addr(5) = '1' then
      ram_data_out <= ram_data(15 downto 8);
	 else
      ram_data_out <= ram_data(7 downto 0);
    end if;
	 -- test pins
	 uart_csn <= not uart_cs;
	 test_rw <= cpu_rw;
	 test_d0 <= uart_data_out(0) and uart_cs;
	 test_d1 <= uart_data_out(1) and uart_cs;
end process;

--
-- tie together interrupts
--
interrupts : process( timer_irq, uart_irq )
begin
    cpu_irq <= uart_irq;
	 cpu_nmi <= timer_irq;
end process;

--
--
--clock_gen : process( SysClk, e_clk )
--begin
--  if SysClk'event and SysClk='1' then
--    e_clk <= not e_clk;
--  end if;
--end process;

  --
  -- flash led to indicate code is working
  --
  increment: process (SysClk, CountL )
  begin
    if(SysClk'event and SysClk = '1') then
      countL <= countL + 1;			 
    end if;
	 LED <= countL(21);
  end process;

  uncommitted: process( SysClk )
  begin
  --
  -- CRTC output signals
  --
	 v_drive     <= '0';
    h_drive     <= '0';
    blue_lo     <= '0';
    blue_hi     <= '0';
    green_lo    <= '0';
    green_hi    <= '0';
    red_lo      <= '0';
    red_hi      <= '0';
	 buzzer      <= '0';
 end process;

  
end my_computer; --===================== End of architecture =======================--

