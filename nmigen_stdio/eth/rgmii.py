import warnings

from nmigen import *
from nmigen.lib.io import Pin

from .endpoint import *


__all__ = ["EthRGMIIRX", "EthRGMIITX"]


class EthRGMIIRX(Elaboratable):
    def __init__(self, *, pins=None):
        self.data_width = 8
        self._pins = pins

        self.rx_ctl    = Signal()
        self.rx_ctl_u  = Signal.like(self.rx_ctl)
        self.rx_data   = Signal(data_width)
        self.rx_data_u = Signal.like(self.rx_data)

        self.source = PHYEndpoint(data_width=data_width)


    def _add_primitives(self, module):
        """Add submodules as required by certain devices
        """
        # Lattice ECP5:
        # (with reference to: https://github.com/enjoy-digital/liteeth/blob/master/liteeth/phy/ecp5rgmii.py)
        if self._device == "lattice_ecp5":
            # Add a DELAYF and a IDDRX1F primitive on latching rx_ctl:
            # * DELAYF: dynamic input delay module, setting DDR SCLK input edge aligned to delayed data output
            # * IDDRX1F: generic DDR input 1(IN):2(OUT) gearbox
            # (see Section 8 of FPGA-TN-02035-1.2, 
            #  "ECP5 and ECP5-5G High-Speed I/O Interface")
            rx_ctl_in = Signal()
            module.submodules += [
                Instance("DELAYF",
                         p_DEL_MODE="SCLK_ALIGNED",
                         i_A=self._pins.rx_ctl,
                         i_LOADN=1,
                         i_MOVE=0,                  # set DIRECTION as constant
                         i_DIRECTION=0,             # increase delay
                         o_Z=rx_ctl_in),
                Instance("IDDRX1F",
                         i_D=rx_ctl_in,
                         i_SCLK=ClockSignal("eth_rx"),
                         i_RST=ResetSignal("eth_rx"),
                         o_Q0=self.rx_ctl_u)
            ]
            # Add a DELAYF and a IDDRX1F primitive on latching each of the 4 pins in rx_data:
            rx_data_in = Signal(self.data_width//2)
            for i in range(self.data_width//2):
                module.submodules += [
                    Instance("DELAYF",
                             p_DEL_MODE="SCLK_ALIGNED",
                             i_A=self._pins.rx_data[i],
                             i_LOADN=1,
                             i_MOVE=0,                  # set DIRECTION as constant
                             i_DIRECTION=0,             # increase delay
                             o_Z=rx_data_in[i]),
                    Instance("IDDRX1F",
                             i_D=rx_data_in[i],
                             i_SCLK=ClockSignal("eth_rx"),
                             i_RST=ResetSignal("eth_rx"),
                             o_Q0=self.rx_data_u[i],
                             o_Q1=self.rx_data_u[i+4])
                ]

        self.source = PHYEndpoint(data_width=self.data_width)

    def elaborate(self, platform):
        m = Module()

        if self._pins is None:
            raise NotImplementedError("Currently does not support simulation")

        rx_ctl = Signal()
        rx_data = Signal(8)

        m.d.eth_rx += [
            rx_ctl.eq(self.rx_ctl_u),
            rx_data.eq(self.rx_data_u)
        ]

        rx_ctl_d = Signal()
        m.d.eth_rx += [
            rx_ctl_d.eq(rx_ctl),
            self.source.stb.eq(rx_ctl),
            self.source.payload.data.eq(rx_data)
        ]
        m.d.comb += self.source.eop.eq(~rx_ctl & rx_ctl_d)

        return m


class EthRGMIITX(Elaboratable):
    def __init__(self, *, clk_domain=None, pins=None):
        """A generic transmitter module for RGMII.

        Note that this module does not implement any DDR modules, but provides
        :class:`nmigen.lib.io.Pins` objects representing the signals to be used as
        the inputs of those DDR modules. Using these signals are recommended.

        This module also has `o_clk`, representing the clock to be used as
        the clock of the DDRs. However, the user should consider whether
        using `o_clk` on all the TX-related DDRs is appropriate, especially because
        some platforms require a skew between the TX clock and the TX data pins.
        """
        # Lattice ECP5:
        # (with reference to: https://github.com/enjoy-digital/liteeth/blob/master/liteeth/phy/ecp5rgmii.py)
        if self._device == "lattice_ecp5":
            # Add a ODDRX1F and a DELAYF primitive on latching rx_ctl:
            # * ODDRX1F: generic DDR output 2(IN):1(OUT) gearbox
            # * DELAYF: dynamic output delay module, setting DDR SCLK input edge aligned to delayed data input
            # (see Section 8 of FPGA-TN-02035-1.2, 
            #  "ECP5 and ECP5-5G High-Speed I/O Interface")
            tx_ctl_out = Signal()
            module.submodules += [
                Instance("ODDRX1F",
                         i_D0=self.sink.stb,
                         i_D1=self.sink.stb,
                         i_SCLK=ClockSignal("eth_tx"),
                         i_RST=ResetSignal("eth_tx"),
                         o_Q=tx_ctl_out),
                Instance("DELAYF",
                         p_DEL_MODE="SCLK_ALIGNED",
                         i_A=tx_ctl_out,
                         i_LOADN=1,
                         i_MOVE=0,                  # set DIRECTION as constant
                         i_DIRECTION=0,             # increase delay
                         o_Z=self._pins.tx_ctl)
            ]
            # Add a ODDRX1F and a DELAYF primitive on latching each of the 4 pins in tx_data:
            tx_data_out = Signal(self.data_width//2)
            for i in range(self.data_width//2):
                module.submodules += [
                    Instance("ODDRX1F",
                             i_D0=self.sink.data[i],
                             i_D1=self.sink.data[4+i],
                             i_SCLK=ClockSignal("eth_tx"),
                             i_RST=ResetSignal("eth_tx"),
                             o_Q=tx_data_out[i]),
                    Instance("DELAYF",
                             p_DEL_MODE="SCLK_ALIGNED",
                             i_A=tx_data_out[i],
                             i_LOADN=1,
                             i_MOVE=0,                  # set DIRECTION as constant
                             i_DIRECTION=0,             # increase delay
                             o_Z=self._pins.tx_data[i])
                ]

        self.tx_clk_oddr = Pin(1, "o", xdr=2)
        self.tx_ctl_oddr = Pin(1, "o", xdr=2)
        self.tx_data_oddr = Pin(self.data_width//2, "o", xdr=2)

        self.sink = PHYEndpoint(data_width=self.data_width)

    def elaborate(self, platform):
        m = Module()

        if self._pins is None:
            raise NotImplementedError("Currently does not support simulation")

        m.d.comb += [
            # DDR output for tx_clk
            # (Note: often the device requires this output to be delayed
            #        from the CTL/DATA clocks; the user should implement
            #        a delay on the `o_clk` of this Pin, or
            #        a delay on the output of a DDR module they write)
            self.tx_clk_oddr.o_clk.eq(self.o_clk),
            self.tx_clk_oddr.o0.eq(1),
            self.tx_clk_oddr.o1.eq(0),
            # DDR output for tx_ctl
            self.tx_ctl_oddr.o_clk.eq(self.o_clk),
            self.tx_ctl_oddr.o0.eq(self.sink.stb),
            self.tx_ctl_oddr.o1.eq(self.sink.stb),
            # DDR output for tx_data[3:0]
            self.tx_data_oddr.o_clk.eq(self.o_clk),
            self.tx_data_oddr.o0.eq(self.sink.payload.data[:self.data_width//2]),
            self.tx_data_oddr.o1.eq(self.sink.payload.data[self.data_width//2:])
        ]

        m.d.comb += self.sink.rdy.eq(1)

        return m


class EthRGMII(Elaboratable):
    def __init__(self, *, data_width=8, pins=None, **kwargs):
        self._pins = pins

        self.rx     = EthRGMIIRX(data_width=data_width,
                                 device=device,
                                 pins=pins, **kwargs)
        self.tx     = EthRGMIITX(data_width=data_width,
                                 device=device,
                                 pins=pins, **kwargs)
        self.sink   = None
        self.source = None


    def _add_primitives(self, module):
        """Add submodules as required by certain devices
        """
        # Lattice ECP5:
        # (with reference to: https://github.com/enjoy-digital/liteeth/blob/master/liteeth/phy/ecp5rgmii.py)
        if self._device == "lattice_ecp5":
            # Add a ODDRX1F and a DELAYF primitive on latching tx_clk:
            tx_clk_out = Signal()
            module.submodules += [
                Instance("ODDRX1F",
                         i_D0=1,
                         i_D1=0,
                         i_SCLK=ClockSignal("eth_tx"),
                         i_RST=ResetSignal("eth_tx"),
                         o_Q=tx_clk_out),
                Instance("DELAYF",
                         p_DEL_MODE="SCLK_ALIGNED",
                         i_A=tx_clk_out,
                         i_LOADN=1,
                         i_MOVE=0,                  # set DIRECTION as constant
                         i_DIRECTION=0,             # increase delay
                         o_Z=self._pins.tx_clk)
            ]


    def elaborate(self, platform):
        m = Module()
        
        m.submodules.rx = rx = self.rx
        m.submodules.tx = tx = self.tx

        return m
