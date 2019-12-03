from nmigen import *
from nmigen.lib.cdc import FFSynchronizer
from nmigen.lib.io import Pin
from nmigen.build.dsl import Subsignal, Pins
from nmigen.build.plat import Platform
from nmigen.utils import bits_for, log2_int


__all__ = ["SPIFlash"]



class SPIFlash(Elaboratable):


    def _check_cmd_dict(self, cmd_dict):
        cmd_names = [
            "FAST_READ",
            "WRITE_ENABLE",
            "WRITE_ENHANCED_VOLATILE_CONFIG_REG"
            # TODO: Add more supported commands
        ]
        for cmd in cmd_dict:
            if cmd not in cmd_names:
                raise ValueError("Invalid SPI command {!r}; must be one of {}"
                                 .format(cmd, cmd_names))
            if cmd_dict[cmd] > 2**self.cmd_width:
                raise ValueError("Invalid SPI command value {!r}; must not be more than {}-bit wide"
                                 .format(cmd_dict[cmd], 2**self.cmd_width))


    def _check_dummy_cycles_dict(self, dummy_cycles_dict):
        cmd_names = [
            "FAST_READ"
            # TODO: Add more commands that have dummy cycles
        ]
        for cmd in dummy_cycles_dict:
            if cmd not in cmd_names:
                raise ValueError("Invalid SPI command {!r}; must be one of {}"
                                 .format(cmd, cmd_names))
        for required_cmd in cmd_names:
            if required_cmd not in dummy_cycles_dict and required_cmd in cmd_names:
                raise KeyError("Missing dummy cycle value for SPI command {!r}"
                               .format(required_cmd))


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
        fcmd = 2**(self.cmd_width*self.spi_width) - 1
        for bit in range(self.cmd_width):
            if (cmd_value >> bit)%2 == 0:
                fcmd &= ~(1 << (bit*self.spi_width))
        return fcmd


    def __init__(self, *, protocol, addr_width, data_width, cmd_width, cmd_dict, dummy_cycles_dict,
                 divisor=1, device=None, pins=None):
        if protocol not in ["extended", "dual", "quad"]:
            raise ValueError("Invalid SPI protocol {!r}; must be one of \"extended\", \"dual\", or \"quad\""
                             .format(protocol))
        self._protocol = protocol
        self.addr_width = addr_width
        self.data_width = data_width
        self.cmd_width = cmd_width
        # Dict that pairs an SPI command (specific to the designated protocol) with its name
        if not isinstance(cmd_dict, dict):
            raise TypeError("Invalid command dict {!r}; must be a dict where each key-value pair is (command name, command value)"
                            .format(cmd_dict))
        self._check_cmd_dict(cmd_dict)
        self._cmd_dict = cmd_dict
        # Dict that pairs the number of dummy cycles (specific to the designated protocl) with the SPI command name
        if not isinstance(dummy_cycles_dict, dict):
            raise TypeError("Invalid dummy cycle dict {!r}; must be a dict where each key-value pair is (command name, number of dummy cycles)"
                            .format(dummy_cycles_dict))
        self._check_dummy_cycles_dict(dummy_cycles_dict)
        self._dummy_cycles_dict = dummy_cycles_dict

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
            # TODO: Add wp & hold pins
        elif protocol == "dual":
            self.dq = Pin(2, "io")
        elif protocol == "quad":
            self.dq = Pin(4, "io")

        self.rdy = Signal()             # Internal rdy signal, set at idle state
        self.ack = Signal()             # External ack signal that begins a read
        self.cmd = Signal(cmd_width)    # set to the corresponding value of the command
        self.addr = Signal(addr_width)
        self.r_data = Signal(self.data_width)
        self.r_rdy = Signal()           # 1 if r_data is valid, 0 otherwise

        self.fcmd_width = self.cmd_width * self.spi_width
        # A two-way register storing the current value on the DQ I/O pins
        # (Note: DQ pins are both input/output only for Dual or Quad SPI protocols)
        self.shreg = Signal(max(self.fcmd_width, self.addr_width, self.data_width))
        self.counter = Signal.like(self.divisor)


    def _add_clk_primitive(self, module):
        """Add a submodule whose instantiation is required by certain devices
        when choosing a user clock as SPI clock
        """
        # Lattice ECP5:
        # "The ECP5 and ECP5-5G devices provide a solution for users 
        # to choose any user clock as MCLK under this scenario 
        # by instantiating USRMCLK macro in your Verilog or VHDL."
        # (see Section 6.1.2 of FPGA-TN-02039-1.7, 
        #  "ECP5 and ECP5-5G sysCONFIG Usage Guide Technical Note")
        if self._device == "lattice_ecp5":
            module.submodules += Instance("USRMCLK",
                                          i_USRMCLKI=self._pins.clk,
                                          i_USRMCLKTS=self._pins.cs.o)


    def elaborate(self, platform):
        m = Module()

        shreg = self.shreg
        counter = self.counter

        if self._pins is not None:
            self._add_clk_primitive(m)
            m.d.comb += [
                self._pins.cs.o.eq(self.cs),
                self._pins.clk.eq(self.clk)
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
        with m.If(counter == self._divisor_val >> 1):
            m.d.sync += self.clk.eq(1)
            if self._protocol == "extended":
                m.d.sync += dq_i.eq(self.miso)
            elif self._protocol in ["dual", "quad"]:
                m.d.sync += dq_i.eq(self.dq.i)
        ## When countdown reaches 0, clock edge goes down (negative)
        ##     shreg latches from MISO for r_data to read
        with m.If(counter == 0):
            m.d.sync += [
                self.clk.eq(0),
                self.counter.eq(self.divisor)
            ]
            m.d.sync += shreg.eq(Cat(dq_i, shreg[:-self.spi_width]))    # "pushing" old data out from the left
        ## Normal countdown
        with m.Else():
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

        # Command: FAST READ
        if "FAST_READ" in self._cmd_dict:
            # fcmd: get formatted command based on cmd_dict
            fcmd = self._format_cmd(self._cmd_dict["FAST_READ"])
            # dummy_cycles: get from dummy_cycles_dict
            dummy_cycles = self._dummy_cycles_dict["FAST_READ"]
            # addr: convert bus address to byte-sized address
            byte_addr = Cat(Repl(0, log2_int(self.data_width//8)), self.addr)
            # FSM
            with m.FSM() as fsm:
                state_durations = {
                    "FASTREAD-CMD"     : self._divisor_val*(self.cmd_width//self.spi_width),
                    "FASTREAD-ADDR"    : self._divisor_val*(self.addr_width//self.spi_width),
                    "FASTREAD-WAITREAD": self._divisor_val*(dummy_cycles+
                                                            self.data_width//self.spi_width),
                    "FASTREAD-RDYWAIT" : 1+self._divisor_val
                }
                max_duration = max([dur for state,dur in state_durations.items()])
                # A "count-up" counter for each state of the command
                state_counter = Signal(range(max_duration))
                # State: Idling
                with m.State("FASTREAD-IDLE"):
                    m.d.comb += self.rdy.eq(1)
                    with m.If((self.cmd == self._cmd_dict["FAST_READ"]) &
                              self.ack & (counter == 0)):
                        m.d.sync += [
                            state_counter.eq(0),
                            self.cs.eq(1),
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
                            shreg[-self.addr_width:].eq(byte_addr)
                        ]
                        m.next = "FASTREAD-ADDR"
                    with m.Else():
                        m.d.sync += state_counter.eq(state_counter + 1)
                # State: Address, MOSI
                with m.State("FASTREAD-ADDR"):
                    with m.If(state_counter == state_durations["FASTREAD-ADDR"] - 1):
                        m.d.sync += state_counter.eq(0)
                        if self._protocol in ["dual", "quad"]:
                            m.d.sync += dq_oe.eq(0)
                        m.next = "FASTREAD-WAITREAD"
                    with m.Else():
                        m.d.sync += state_counter.eq(state_counter + 1)
                # State: Dummy cycles (waiting), and then Read (MISO)
                with m.State("FASTREAD-WAITREAD"):
                    with m.If(state_counter == state_durations["FASTREAD-WAITREAD"] - 1):
                        m.d.sync += [
                            state_counter.eq(0),
                            self.cs.eq(0),
                            self.r_rdy.eq(1)
                        ]
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

        """
        # Command: WRITE ENABLE
        if "WRITE_ENHANCED_VOLATILE_CONFIG_REG" in self._cmd_dict:
            # fcmd: get formatted command based on cmd_dict
            fcmd = self._format_cmd(self._cmd_dict["WRITE_ENHANCED_VOLATILE_CONFIG_REG"])
            # FSM
            with m.FSM() as fsm:
                #
                with m.State("WR-EN-CMD"):
                    m.d.comb += self.rdy.eq(1)
                    with m.If
        """

        """
        # Command: WRITE ENHANCED VOLATILE CONFIGURATION REGISTER
        if "WRITE_ENHANCED_VOLATILE_CONFIG_REG" in self._cmd_dict:
            # fcmd: get formatted command based on cmd_dict
            fcmd = self._format_cmd(self._cmd_dict["WRITE_ENHANCED_VOLATILE_CONFIG_REG"])
            # FSM
            with m.FSM() as fsm:
                #
                with m.State("WR-EN-CMD"):
                    m.d.comb += self.rdy.eq(1)
                    with m.If
        """

        return m