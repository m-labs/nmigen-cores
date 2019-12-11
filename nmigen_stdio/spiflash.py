from nmigen import *
from nmigen.lib.cdc import FFSynchronizer
from nmigen.lib.io import Pin
from nmigen.build.dsl import Subsignal, Pins
from nmigen.build.plat import Platform
from nmigen.utils import bits_for, log2_int


__all__ = ["SPIFlashFastReader"]



class SPIFlashFastReader(Elaboratable):
    """An SPI flash controller module for fast-reading
    """
    CMD_WIDTH = 8
    CMD_DICT = {
        "extended": 0x0b,
        "dual"    : 0x3b,
        "quad"    : 0x6b
    }


    def _format_cmd(self, cmd_value):
        """
        Returns the values on all the DQ lines that corresponds to the command.

        Since everything is transmitted on all DQ lines (command, address and data),
        the input cmd_value is extended/interleaved to full DQ width
        even if DQ1-DQ3 are "don't care" during the command phase:
        
        eg:    1    1    1    0    1    0    1    1   (extended SPI mode)
              11   11   11   10   11   10   11   11   (dual I/O SPI mode)
            1111 1111 1111 1110 1111 1110 1111 1111   (quad I/O SPI mode)
        """
        fcmd = 2**(SPIFlashFastReader.CMD_WIDTH*self.spi_width) - 1
        for bit in range(SPIFlashFastReader.CMD_WIDTH):
            if (cmd_value >> bit)%2 == 0:
                fcmd &= ~(1 << (bit*self.spi_width))
        return fcmd


    def __init__(self, *, protocol, addr_width, data_width, dummy_cycles,
                 divisor=1, device=None, pins=None):
        if protocol not in ["extended", "dual", "quad"]:
            raise ValueError("Invalid SPI protocol {!r}; must be one of \"extended\", \"dual\", or \"quad\""
                             .format(protocol))
        self._protocol = protocol
        self._addr_width = addr_width
        self._data_width = data_width
        self._dummy_cycles = dummy_cycles

        if divisor < 1:
            raise ValueError("Invalid divisor value; must be at least 1"
                             .format(divisor))
        self.divisor = Signal(bits_for(divisor), reset=divisor)
        self._divisor_val = divisor + 1

        supported_devices = ["lattice_ecp5"]
        if device is not None and device not in supported_devices:
            raise ValueError("Invalid FPGA device name {!r}; must be one of {}"
                             .format(device, supported_devices))
        self._device = device
        if self._device is not None and pins is None:
            raise ValueError("Pins parameter is missing for this FPGA device {}"
                             .format(self._device))
        self._pins = pins

        if self._protocol == "extended":
            self.spi_width = 1
        elif self._protocol == "dual":
            self.spi_width = 2
        elif self._protocol == "quad":
            self.spi_width = 4

        self.cs = Signal(reset=0)       # Equivalent to ~CS_N
        self.clk = Signal()

        if protocol == "extended":
            self.mosi = Signal()
            self.miso = Signal()
        elif protocol == "dual":
            self.dq = Pin(2, "io")
        elif protocol == "quad":
            self.dq = Pin(4, "io")

        self.rdy = Signal()             # Internal rdy signal, set at idle state
        self.ack = Signal()             # External ack signal that begins a read
        self.addr = Signal(addr_width)
        self.r_data = Signal(self._data_width)
        self.r_rdy = Signal()           # 1 if r_data is valid, 0 otherwise

        self.cmd_width = SPIFlashFastReader.CMD_WIDTH
        self._fcmd_width = self.cmd_width * self.spi_width
        # A two-way register storing the current value on the DQ I/O pins
        # (Note: DQ pins are both input/output only for Dual or Quad SPI protocols)
        self.shreg = Signal(max(self._fcmd_width, self._addr_width, self._data_width, self._dummy_cycles))
        self.counter = Signal.like(self.divisor)


    def _add_clk_primitive(self, platform, module):
        """Add a submodule whose instantiation is required by certain devices
        when choosing a user clock as SPI clock.

        Note that the CLK pin is not connected outside of this function, 
        as certain devices does not need to request the CLK pin.
        """
        # Lattice ECP5:
        # "The ECP5 and ECP5-5G devices provide a solution for users 
        # to choose any user clock as MCLK under this scenario 
        # by instantiating USRMCLK macro in your Verilog or VHDL."
        # (see Section 6.1.2 of FPGA-TN-02039-1.7, 
        #  "ECP5 and ECP5-5G sysCONFIG Usage Guide Technical Note")
        if self._device == "lattice_ecp5":
            usrmclk = Signal.like(self.clk)
            module.d.comb += usrmclk.eq(self.clk)
            module.submodules += Instance("USRMCLK",
                                          i_USRMCLKI=usrmclk,
                                          i_USRMCLKTS=0)


    def elaborate(self, platform):
        m = Module()

        shreg = self.shreg
        counter = self.counter

        if self._pins is not None:
            self._add_clk_primitive(platform, m)
            m.d.comb += [
                self._pins.cs.o.eq(self.cs),
                self._pins.wp.eq(1),
                self._pins.hold.eq(1)
            ]
            if self._protocol == "extended":
                m.submodules += FFSynchronizer(self._pins.miso.i, self.miso)
                m.d.comb += self._pins.mosi.o.eq(self.mosi)
            elif self._protocol in ["dual", "quad"]:
                dq_oe = Signal()
                m.submodules.dq = platform.get_tristate(self.dq, self._pins.dq, None, False)
        # If the user doesn't give pins, create dq Pins for Dual & Quad
        else:
            if self._protocol in ["dual", "quad"]:
                dq_oe = Signal()
                if self._protocol == "dual":
                    dq_pins = Record([
                        ("dq", Pin(width=2, dir="io", xdr=0).layout)
                    ])
                elif self._protocol == "quad":
                    dq_pins = Record([
                        ("dq", Pin(width=4, dir="io", xdr=0).layout)
                    ])
                tristate_submodule = Module()
                tristate_submodule.submodules += Instance("$tribuf",
                    p_WIDTH=self.dq.width,
                    i_EN=self.dq.oe,
                    i_A=Platform._invert_if(False, self.dq.o),
                    o_Y=dq_pins
                )
                m.submodules.dq = tristate_submodule

        # r_data always reads from shreg
        m.d.comb += self.r_data.eq(shreg)

        # Countdown logic for counter based on divisor
        # Also implements MISO logic
        dq_i = Signal(self.spi_width)
        ## When countdown is half-way done, clock edge goes up (positive);
        ##     MISO starts latching bit/byte from slave
        with m.If((counter == self._divisor_val >> 1) & self.cs):
            m.d.sync += self.clk.eq(1)
            if self._protocol == "extended":
                m.d.sync += dq_i.eq(self.miso)
            elif self._protocol in ["dual", "quad"]:
                m.d.sync += dq_i.eq(self.dq.i)
        ## When countdown reaches 0, clock edge goes down (negative)
        ##     shreg latches from MISO for r_data to read
        with m.If((counter == 0) & self.cs):
            m.d.sync += [
                self.clk.eq(0),
                counter.eq(self.divisor)
            ]
            # MOSI latches from shreg by "pushing" old data out from the left of shreg
            m.d.sync += shreg.eq(Cat(dq_i, shreg[:-self.spi_width]))
        ## Normal countdown
        with m.Elif(self.cs):
            m.d.sync += counter.eq(counter - 1)

        # MOSI logic for Extended SPI protocol:
        # MOSI always output the leftmost bit of shreg
        if self._protocol == "extended":
            m.d.comb += self.mosi.eq(shreg[-1])
        # MOSI logic for Dual and Quad SPI protocols:
        # Whenever DQ output should be enabled, 
        # DQ always output the leftmost `spi_width`-wide bits of shreg
        elif self._protocol in ["dual", "quad"]:
            m.d.comb += [
                self.dq.o.eq(shreg[-self.spi_width:]),
                self.dq.oe.eq(dq_oe)
            ]

        # fcmd: get formatted command based on cmd_dict
        fcmd = self._format_cmd(SPIFlashFastReader.CMD_DICT[self._protocol])
        # addr: convert bus address to byte-sized address
        byte_addr = Cat(Repl(0, log2_int(self._data_width//8)), self.addr)
        # FSM
        with m.FSM() as fsm:
            state_durations = {
                "FASTREAD-CMD"     : self._divisor_val*(self.cmd_width//self.spi_width),
                "FASTREAD-ADDR"    : self._divisor_val*(self._addr_width//self.spi_width),
                "FASTREAD-WAITREAD": self._divisor_val*(self._dummy_cycles+
                                                        self._data_width//self.spi_width),
                "FASTREAD-RDYWAIT" : 1+self._divisor_val
            }
            max_duration = max([dur for state,dur in state_durations.items()])
            # A "count-up" counter for each state of the command
            state_counter = Signal(range(max_duration))
            # State: Idling
            with m.State("FASTREAD-IDLE"):
                m.d.comb += self.rdy.eq(1)
                with m.If(self.ack):
                    m.d.sync += [
                        counter.eq((self._divisor_val >> 1) - 1),
                        self.clk.eq(1)
                    ]
                    m.next = "FASTREAD-CS"
            # State: Chip select
            with m.State("FASTREAD-CS"):
                with m.If(counter == 0):
                    m.d.sync += [
                        state_counter.eq(0),
                        shreg[-self.cmd_width:].eq(fcmd)
                    ]
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += dq_oe.eq(1)
                    m.next = "FASTREAD-CMD"
            # State: Command, MOSI
            with m.State("FASTREAD-CMD"):
                with m.If(state_counter == state_durations["FASTREAD-CMD"] - 1):
                    m.d.sync += [
                        state_counter.eq(0),
                        shreg[-self._addr_width:].eq(byte_addr)
                    ]
                    m.next = "FASTREAD-ADDR"
                with m.Else():
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Address, MOSI
            with m.State("FASTREAD-ADDR"):
                with m.If(state_counter == state_durations["FASTREAD-ADDR"] - 1):
                    #m.d.sync += state_counter.eq(0)
                    #if self._protocol in ["dual", "quad"]:
                    #    m.d.sync += dq_oe.eq(0)
                    m.d.sync += [
                        state_counter.eq(0),
                        shreg[-self._dummy_cycles:].eq(Repl(0, self._dummy_cycles))
                    ]
                    m.next = "FASTREAD-WAITREAD"
                with m.Else():
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Dummy cycles (waiting), and then Read (MISO)
            with m.State("FASTREAD-WAITREAD"):
                with m.If(state_counter == state_durations["FASTREAD-WAITREAD"] - 1):
                    m.d.sync += [
                        state_counter.eq(0),
                        self.r_rdy.eq(1)
                    ]
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += dq_oe.eq(0)
                    m.next = "FASTREAD-RDYWAIT"
                with m.Else():
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Send r_rdy (for 1 clock period), and then Wait
            with m.State("FASTREAD-RDYWAIT"):
                with m.If(state_counter == 0):
                    m.d.sync += self.r_rdy.eq(0)
                # Early check to skip 1 clock period of doing nothing
                with m.If(state_counter == state_durations["FASTREAD-RDYWAIT"] - 2):
                    m.d.sync += state_counter.eq(0)
                    m.next = "FASTREAD-IDLE"
                with m.Else():
                    m.d.sync += state_counter.eq(state_counter + 1)
        # 
        with m.If(~fsm.ongoing("FASTREAD-IDLE") & 
                  ~fsm.ongoing("FASTREAD-RDYWAIT")):
            m.d.comb += self.cs.eq(1)
        with m.Else():
            m.d.comb += self.cs.eq(0)

        """
        ### SCRAPPED: other commands should be implemented in a separate core

        # Command: WRITE ENABLE
        if "WRITE_ENHANCED_VOLATILE_CONFIG_REG" in CMD_DICT:
            # fcmd: get formatted command based on cmd_dict
            fcmd = self._format_cmd(CMD_DICT[self._protocol])
            # FSM
            with m.FSM() as fsm:
                #
                with m.State("WR-EN-CMD"):
                    m.d.comb += self.rdy.eq(1)
                    with m.If

        # Command: WRITE ENHANCED VOLATILE CONFIGURATION REGISTER
        if "WRITE_ENHANCED_VOLATILE_CONFIG_REG" in CMD_DICT:
            # fcmd: get formatted command based on cmd_dict
            fcmd = self._format_cmd(CMD_DICT[self._protocol])
            # FSM
            with m.FSM() as fsm:
                #
                with m.State("WR-EN-CMD"):
                    m.d.comb += self.rdy.eq(1)
                    with m.If
        """

        return m