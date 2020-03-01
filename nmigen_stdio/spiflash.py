from nmigen import *
from nmigen.lib.cdc import FFSynchronizer
from nmigen.lib.io import Pin
from nmigen.build.dsl import Subsignal, Pins
from nmigen.build.plat import Platform
from nmigen.utils import bits_for, log2_int


__all__ = ["SPIFlashSlowReader", "SPIFlashFastReader"]


class _SPIFlashReaderBase:
    """A base class for an SPI flash controller that is to perform any READ operations
    """
    CMD_WIDTH = 8
    CMD_DICT = {
        "standard": None,
        "dual"    : None,
        "quad"    : None
    }

    def _format_cmd(self):
        """
        Returns the values on all the DQ lines that corresponds to the command.

        Since everything is transmitted on all DQ lines (command, address and data),
        the input cmd_value is extended/interleaved to full DQ width
        even if DQ1-DQ3 are "don't care" during the command phase:

        eg:    1    1    1    0    1    0    1    1   (standard SPI mode)
              11   11   11   10   11   10   11   11   (dual I/O SPI mode)
            1111 1111 1111 1110 1111 1110 1111 1111   (quad I/O SPI mode)
        """
        cmd_value = type(self).CMD_DICT.get(self._protocol)
        if cmd_value is None:
            raise KeyError("Reader class {} does not support {} SPI protocol"
                           .format(type(self).__name__, self._protocol))
        fcmd = 2**(type(self).CMD_WIDTH*self.spi_width) - 1
        for bit in range(type(self).CMD_WIDTH):
            if (cmd_value >> bit) % 2 == 0:
                fcmd &= ~(1 << (bit*self.spi_width))
        return fcmd

    def __init__(self, *, protocol, data_width=32,
                 divisor=1, divisor_bits=None, pins=None):
        if protocol not in ["standard", "dual", "quad"]:
            raise ValueError("Invalid SPI protocol {!r}; must be one of "
                             "\"standard\", \"dual\", or \"quad\""
                             .format(protocol))
        self._protocol = protocol
        if not isinstance(data_width, int) or data_width < 1:
            raise ValueError("Bus data width must be a positive integer, not {}"
                             .format(repr(data_width)))
        if data_width < 8 or data_width % 8:
            raise ValueError("Data width must be divisible by 8 ({} % 8 != 0)"
                             .format(data_width))
        self._data_width = data_width

        if divisor < 1:
            raise ValueError("Invalid divisor value; must be at least 1"
                             .format(divisor))
        self.divisor = Signal(divisor_bits or bits_for(divisor), reset=divisor)

        self._pins = pins

        if self._protocol == "standard":
            self.spi_width = 1
        elif self._protocol == "dual":
            self.spi_width = 2
        elif self._protocol == "quad":
            self.spi_width = 4

        self.cs = Signal(reset=0)       # Equivalent to ~CS_N
        self.clk = Signal()

        if protocol == "standard":
            self.mosi = Signal()
            self.miso = Signal()
        elif protocol == "dual":
            self.dq = Pin(2, "io")
        elif protocol == "quad":
            self.dq = Pin(4, "io")

        self.rdy = Signal()             # Internal rdy signal, set at idle state
        self.ack = Signal()             # External ack signal that begins a read
        self.r_data = Signal(self._data_width)
        self.r_rdy = Signal()           # 1 if r_data is valid, 0 otherwise

        self.cmd_width = type(self).CMD_WIDTH
        self._fcmd_width = self.cmd_width * self.spi_width
        # A two-way register storing the current value on the DQ I/O pins
        # (Note: DQ pins are both input/output only for Dual or Quad SPI protocols)
        self.shreg = None
        self.counter = Signal.like(self.divisor)
        self.counter_en = Signal()
        self.clk_posedge_next = Signal()
        self.clk_negedge_next = Signal()

    def _add_spi_hardware_logic(self, platform, module):
        """Add the foundamental hardware logic for controlling all SPI pins to be used
        """
        if self.shreg is None:
            raise NotImplementedError("Shift register shreg has not been defined!")
        shreg = self.shreg
        counter = self.counter
        counter_en = self.counter_en

        if self._pins is not None:
            module.d.comb += self._pins.cs.o.eq(self.cs)
            if hasattr(self._pins, "wp"):
                module.d.comb += self._pins.wp.eq(0)
            if hasattr(self._pins, "hold"):
                module.d.comb += self._pins.hold.eq(0)
            # Platforms that require declaration of a user SPI clock signal
            # (e.g. by instantiating a USRMCLK Instance) must NOT pass a CLK on the SPI flash
            if hasattr(self._pins, "clk"):
                module.d.comb += self._pins.clk.o.eq(self.clk)
            if self._protocol == "standard":
                module.submodules += FFSynchronizer(self._pins.miso.i, self.miso)
                module.d.comb += self._pins.mosi.o.eq(self.mosi)
            elif self._protocol in ["dual", "quad"]:
                self.dq_oe = Signal()
                self.dq = self._pins.dq
        # If the user doesn't give pins, create dq Pins for Dual & Quad
        else:
            if self._protocol in ["dual", "quad"]:
                self.dq_oe = Signal()
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
                    i_A=self.dq.o,
                    o_Y=dq_pins
                )
                module.submodules.dq = tristate_submodule

        # r_data always reads from shreg
        module.d.comb += self.r_data.eq(shreg)

        # Countdown logic for counter based on divisor
        # Also implements MISO logic
        dq_i = Signal(self.spi_width)
        # When counter is enabled:
        with module.If(counter_en):
            # When countdown is half-way done, clock edge goes up (positive):
            # - MISO latches bit/byte from slave
            # - slave has been latching from MOSI mid-way (since previous negedge)
            with module.If(counter == (self.divisor+1) >> 1):
                # Enable SCLK only when CS:
                with module.If(self.cs):
                    module.d.sync += self.clk.eq(1)
                # Indicate imminent posedge
                module.d.comb += self.clk_posedge_next.eq(1)
                if self._protocol == "standard":
                    module.d.sync += dq_i.eq(self.miso)
                elif self._protocol in ["dual", "quad"]:
                    module.d.sync += dq_i.eq(self.dq.i)
            # When countdown reaches 0, clock edge goes down (negative):
            # - MOSI latches from shreg by "pushing" old data out from the left of shreg;
            # - slave has been outputting MISO mid-way (since previous posedge)
            with module.If(counter == 0):
                # Enable SCLK only when CS:
                with module.If(self.cs):
                    module.d.sync += self.clk.eq(0)
                module.d.sync += counter.eq(self.divisor)
                # Indicate imminent negedge
                module.d.comb += self.clk_negedge_next.eq(1)
                module.d.sync += shreg.eq(Cat(dq_i, shreg[:-self.spi_width]))
            # Normal countdown
            with module.Else():
                module.d.sync += counter.eq(counter - 1)
        # When counter is disabled, reset:
        with module.Else():
            module.d.sync += counter.eq(self.divisor)

        # MOSI logic for Standard SPI protocol:
        # MOSI always output the leftmost bit of shreg
        if self._protocol == "standard":
            module.d.comb += self.mosi.eq(shreg[-1])
        # MOSI logic for Dual and Quad SPI protocols:
        # Whenever DQ output should be enabled,
        # DQ always output the leftmost `spi_width`-wide bits of shreg
        elif self._protocol in ["dual", "quad"]:
            module.d.comb += [
                self.dq.o.eq(shreg[-self.spi_width:]),
                self.dq.oe.eq(self.dq_oe)
            ]

        # r_rdy is asserted for only 1 clock cycle
        with module.If(self.r_rdy):
            module.d.sync += self.r_rdy.eq(0)


class SPIFlashSlowReader(_SPIFlashReaderBase, Elaboratable):
    """An SPI flash controller module for normal reading
    (i.e. only available in Standard protocol and a lower frequency,
    but no dummy cycles of waiting are required).
    """
    CMD_DICT = {
        "standard": 0x03,
        "dual"    : None,
        "quad"    : None
    }

    def __init__(self, addr_width, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._addr_width = addr_width
        self.addr = Signal(addr_width)
        self.shreg = Signal(max(self._fcmd_width, self._data_width, self._addr_width))

    def elaborate(self, platform):
        m = Module()
        self._add_spi_hardware_logic(platform, m)

        shreg = self.shreg
        counter = self.counter
        counter_en = self.counter_en

        # fcmd: get formatted command based on cmd_dict
        fcmd = self._format_cmd()
        # addr: convert bus address to byte-sized address
        byte_addr = Cat(Repl(0, log2_int(self._data_width // 8)), self.addr)
        # FSM
        with m.FSM() as fsm:
            state_durations = {
                "SLOWREAD-CMD" : self.cmd_width//self.spi_width,
                "SLOWREAD-ADDR": self._addr_width//self.spi_width,
                "SLOWREAD-READ": self._data_width//self.spi_width
            }
            max_duration = max([dur for state, dur in state_durations.items()])
            # A "count-up" counter for each state of the command
            state_counter = Signal(range(max_duration))
            # State: Idling
            with m.State("SLOWREAD-IDLE"):
                m.d.comb += self.rdy.eq(1)
                with m.If(self.ack):
                    m.d.sync += [
                        counter.eq(0),
                        counter_en.eq(1),
                        self.clk.eq(0)
                    ]
                    m.next = "SLOWREAD-CS"
            # State: Chip select
            with m.State("SLOWREAD-CS"):
                m.d.sync += self.cs.eq(1)
                with m.If(self.clk_negedge_next):
                    m.d.sync += [
                        state_counter.eq(0),
                        shreg[-self.cmd_width:].eq(fcmd)
                    ]
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += self.dq_oe.eq(1)
                    m.next = "SLOWREAD-CMD"
            # State: Command, MOSI
            with m.State("SLOWREAD-CMD"):
                with m.If((state_counter == state_durations["SLOWREAD-CMD"] - 1) &
                          self.clk_negedge_next):
                    m.d.sync += [
                        state_counter.eq(0),
                        shreg[-self._addr_width:].eq(byte_addr)
                    ]
                    m.next = "SLOWREAD-ADDR"
                with m.Elif(self.clk_negedge_next):
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Address, MOSI
            with m.State("SLOWREAD-ADDR"):
                with m.If((state_counter == state_durations["SLOWREAD-ADDR"] - 1) &
                          self.clk_posedge_next):
                    m.d.sync += state_counter.eq(0)
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += self.dq_oe.eq(0)
                    m.next = "SLOWREAD-READ"
                with m.Elif(self.clk_negedge_next):
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Read, MISO
            with m.State("SLOWREAD-READ"):
                with m.If((state_counter == state_durations["SLOWREAD-READ"] - 1) &
                          self.clk_posedge_next):
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += self.dq_oe.eq(0)
                    m.next = "SLOWREAD-RDYWAIT"
                with m.Elif(self.clk_posedge_next):
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Send r_rdy (for 1 clock period), and then Wait
            with m.State("SLOWREAD-RDYWAIT"):
                with m.If(self.clk_negedge_next):
                    m.d.sync += self.cs.eq(0)
                with m.If(self.clk_posedge_next):
                    m.d.sync += [
                        self.r_rdy.eq(1),
                        counter_en.eq(0)
                    ]
                with m.If(self.r_rdy):
                    m.next = "SLOWREAD-IDLE"

        return m


class SPIFlashFastReader(_SPIFlashReaderBase, Elaboratable):
    """An SPI flash controller module for fast-reading
    (i.e. available in all SPI protocols and a higher frequency,
    but dummy cycles of waiting are required)
    """
    CMD_DICT = {
        "standard": 0x0b,
        "dual"    : 0x3b,
        "quad"    : 0x6b
    }

    def __init__(self, addr_width, dummy_cycles, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._addr_width = addr_width
        self._dummy_cycles = dummy_cycles
        self.addr = Signal(addr_width)
        self.shreg = Signal(max(self._fcmd_width, self._data_width, self._addr_width))

    def elaborate(self, platform):
        m = Module()
        self._add_spi_hardware_logic(platform, m)

        shreg = self.shreg
        counter = self.counter
        counter_en = self.counter_en

        # fcmd: get formatted command based on cmd_dict
        fcmd = self._format_cmd()
        # addr: convert bus address to byte-sized address
        byte_addr = Cat(Repl(0, log2_int(self._data_width // 8)), self.addr)
        # FSM
        with m.FSM() as fsm:
            state_durations = {
                "FASTREAD-CMD"     : self.cmd_width//self.spi_width,
                "FASTREAD-ADDR"    : self._addr_width//self.spi_width,
                "FASTREAD-WAITREAD": self._dummy_cycles+self._data_width//self.spi_width
            }
            max_duration = max([dur for state, dur in state_durations.items()])
            # A "count-up" counter for each state of the command
            state_counter = Signal(range(max_duration))
            # State: Idling
            with m.State("FASTREAD-IDLE"):
                m.d.comb += self.rdy.eq(1)
                with m.If(self.ack):
                    m.d.sync += [
                        counter.eq(0),
                        counter_en.eq(1),
                        self.clk.eq(0)
                    ]
                    m.next = "FASTREAD-CS"
            # State: Chip select
            with m.State("FASTREAD-CS"):
                m.d.sync += self.cs.eq(1)
                with m.If(self.clk_negedge_next):
                    m.d.sync += [
                        state_counter.eq(0),
                        shreg[-self.cmd_width:].eq(fcmd)
                    ]
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += self.dq_oe.eq(1)
                    m.next = "FASTREAD-CMD"
            # State: Command, MOSI
            with m.State("FASTREAD-CMD"):
                with m.If((state_counter == state_durations["FASTREAD-CMD"] - 1) &
                          self.clk_negedge_next):
                    m.d.sync += [
                        state_counter.eq(0),
                        shreg[-self._addr_width:].eq(byte_addr)
                    ]
                    m.next = "FASTREAD-ADDR"
                with m.Elif(self.clk_negedge_next):
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Address, MOSI
            with m.State("FASTREAD-ADDR"):
                with m.If((state_counter == state_durations["FASTREAD-ADDR"] - 1) &
                          self.clk_posedge_next):
                    m.d.sync += state_counter.eq(0)
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += self.dq_oe.eq(0)
                    m.next = "FASTREAD-WAITREAD"
                with m.Elif(self.clk_negedge_next):
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Dummy cycles (waiting), and then Read (MISO)
            with m.State("FASTREAD-WAITREAD"):
                with m.If((state_counter == state_durations["FASTREAD-WAITREAD"] - 1) &
                          self.clk_posedge_next):
                    if self._protocol in ["dual", "quad"]:
                        m.d.sync += self.dq_oe.eq(0)
                    m.next = "FASTREAD-RDYWAIT"
                with m.Elif(self.clk_posedge_next):
                    m.d.sync += state_counter.eq(state_counter + 1)
            # State: Send r_rdy (for 1 clock period), and then Wait
            with m.State("FASTREAD-RDYWAIT"):
                with m.If(self.clk_negedge_next):
                    m.d.sync += self.cs.eq(0)
                with m.If(self.clk_posedge_next):
                    m.d.sync += [
                        self.r_rdy.eq(1),
                        counter_en.eq(0)
                    ]
                with m.If(self.r_rdy):
                    m.next = "FASTREAD-IDLE"

        return m
