from nmigen import *
from nmigen.lib.io import Pin
from .endpoint import *


__all__ = ["EthRGMIIRX", "EthRGMIITX"]


class EthRGMIIRX(Elaboratable):
    def __init__(self, *, data_width=8, device=None, pins=None):
        self.data_width = data_width

        supported_devices = ["lattice_ecp5"]
        if device is not None and device not in supported_devices:
            raise ValueError("Invalid FPGA device name {!r}; must be one of {}"
                             .format(device, supported_devices))
        self._device = device
        if self._device is not None and pins is None:
            raise ValueError("Pins parameter is missing for this FPGA device {}"
                             .format(self._device))
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


    def elaborate(self, platform):
        m = Module()

        if self._pins is not None:
            self._add_primitives(m)
            m.domains.eth_rx = cd_eth_rx = ClockDomain()
            m.d.comb += cd_eth_rx.clk.eq(self._pins.rx_clk)

        # If the user doesn't give pins, create rx_ctl & rx_data DDR Pins
        else:
            rx_ctl_i    = Signal()
            rx_ctl_iddr = Pin(1, dir="i", xdr=2)
            m.d.comb += self.rx_ctl_u.eq(rx_ctl_iddr.i0)
            # TODO: add DDR logic (latch i0 on posedge, i1 on negedge)
            rx_data_i    = Signal(self.data_width//2)
            rx_data_iddr = Pin(self.data_width//2, dir="i", xdr=2)
            for i in range(self.data_width//2):
                m.d.comb += [
                    self.rx_data_u[i].eq(rx_data_iddr.i0[i]),
                    self.rx_data_u[i+4].eq(rx_data_iddr.i1[i])
                ]
            # TODO: add DDR logic (latch i0 on posedge, i1 on negedge)

        source = self.source

        #
        m.d.sync += [
            self.rx_ctl.eq(self.rx_ctl_u),
            self.rx_data.eq(self.rx_data_u)
        ]

        rx_ctl_d = Signal()
        m.d.sync += [
            rx_ctl_d.eq(self.rx_ctl),
            source.stb.eq(self.rx_ctl),
            source.payload.data.eq(
                Cat(self.rx_data[:self.data_width//2],
                    self.rx_data[self.data_width//2:])
            )
        ]
        m.d.comb += source.eop.eq(~self.rx_ctl & rx_ctl_d)

        return m


class EthRGMIITX(Elaboratable):
    def __init__(self, *, data_width=8, device=None, pins=None):
        self.data_width = data_width

        supported_devices = ["lattice_ecp5"]
        if device is not None and device not in supported_devices:
            raise ValueError("Invalid FPGA device name {!r}; must be one of {}"
                             .format(device, supported_devices))
        self._device = device
        if self._device is not None and pins is None:
            raise ValueError("Pins parameter is missing for this FPGA device {}"
                             .format(self._device))
        self._pins = pins

        self.tx_ctl    = Signal()
        self.tx_ctl_u  = Signal.like(self.tx_ctl)
        self.tx_data   = Signal(data_width)
        self.tx_data_u = Signal.like(self.tx_data)

        self.sink = PHYEndpoint(data_width=data_width)


    def _add_primitives(self, module):
        """Add submodules as required by certain devices
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


    def elaborate(self, platform):
        m = Module()

        if self._pins is not None:
            self._add_primitives(m)
        # If the user doesn't give pins, create rx_ctl & rx_data DDR Pins
        else:
            tx_ctl_o    = Signal()
            tx_ctl_oddr = Pin(1, dir="o", xdr=2)
            m.d.comb += [
                tx_ctl_oddr.o0.eq(self.sink.stb),
                tx_ctl_oddr.o1.eq(self.sink.stb)
            ]
            # TODO: add DDR logic (latch o0 on posedge, o1 on negedge)
            tx_data_o    = Signal(self.data_width//2)
            tx_data_oddr = Pin(self.data_width//2, dir="o", xdr=2)
            for i in range(self.data_width//2):
                m.d.comb += [
                    tx_data_oddr.o0[i].eq(self.tx_data_u[i]),
                    tx_data_oddr.o1[i].eq(self.tx_data_u[i+4])
                ]
            # TODO: add DDR logic (latch o0 on posedge, o1 on negedge)

        #
        sink = self.sink
        m.d.comb += sink.rdy.eq(1)

        return m


class EthRGMII(Elaboratable):
    def __init__(self, *, data_width=8, device=None, pins=None, **kwargs):
        supported_devices = ["lattice_ecp5"]
        if device is not None and device not in supported_devices:
            raise ValueError("Invalid FPGA device name {!r}; must be one of {}"
                             .format(device, supported_devices))
        self._device = device
        if self._device is not None and pins is None:
            raise ValueError("Pins parameter is missing for this FPGA device {}"
                             .format(self._device))
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
        self.source = rx.source
        self.sink = tx.sink

        m.domains.eth_tx = cd_eth_tx = ClockDomain()
        cd_eth_tx.clk.eq(ClockSignal("eth_rx"))

        if self._pins is not None:
            self._add_primitives(m)
        # If the user doesn't give pins, create rx_ctl & rx_data DDR Pins
        else:
            tx_clk_o    = Signal()
            tx_clk_oddr = Pin(1, dir="o", xdr=2)
            m.d.comb += [
                tx_clk_oddr.o0.eq(1),
                tx_clk_oddr.o1.eq(0)
            ]
            # TODO: add DDR logic (latch o0 on posedge, o1 on negedge)

        # TODO: implement reset

        return m
