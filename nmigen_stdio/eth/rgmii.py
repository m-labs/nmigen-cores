import warnings

from nmigen import *
from nmigen.lib.io import Pin

from .endpoint import *


__all__ = ["EthRGMIIRX", "EthRGMIITX"]


class EthRGMIIRX(Elaboratable):
    """A generic receiver module for RGMII.

    Note that this module does not implement any DDR modules, but provides
    :class:`nmigen.lib.io.Pins` objects representing the signals to be used as
    the outputs of those DDR modules. Using these signals are recommended.

    This module also has `i_clk`, representing the clock to be used as
    the clock of the DDRs. However, the user should consider whether
    using `i_clk` on all the RX-related DDRs is appropriate, especially because
    some platforms require a skew between the RX clock and the RX data pins.

    Parameters
    ----------
    clk_domain : str or None
        Clock domain to be used as the DDR clock. Optional and defaults to None,
        meaning the user must connect the DDR clock signal to an external signal.

    Attributes
    ----------
    data_width : int
        Number of bits per Ethernet packet. Equals to 8.
    source : :class:`PHYEndpoint`
        Representation of the status and data of the packet received. See :class:`PHYEndPoint`.
    i_clk : Signal()
        Clock signal used as input for the DDR outputs.
        If clk_domain has been specified, it is the clock signal of that domain.
        Otherwise, it can be externally driven by other signals.
    rx_clk_oddr : :class:`Pin`
        DDR input representation for the RX CLK. See below for its attributes.
    rx_ctl_oddr : :class:`Pin`
        DDR input representation for the RX CTL. See below for its attributes.
    rx_data_oddr : :class:`Pin`
        DDR input representation for the RX DATA. See below for its attributes.
        Its i0 represents the first 4 bits, and i1 represents the last 4 bits.

    Each of the DDR inputs above contains these attributes:
    i_clk : Signal()
        Clock signal for the DDR. Internally driven.
    i0 : Signal
        Input value at the rising edge of i_clk. Internally driven.
    i1 : Signal
        Input value at the falling edge of i_clk. Internally driven.
    """
    def __init__(self, *, clk_domain=None):
        self.data_width = 8
        if clk_domain:
            # If a clock domain name is given, use its clock signal for the DDR;
            # This also assumes that the clock domain exists
            if isinstance(clk_domain, str):
                self.i_clk = ClockSignal(clk_domain)
                self.clk_domain = clk_domain
            else:
                raise TypeError("Clock domain must be a string, not {!r}"
                                .format(clk_domain))
        else:
            # Otherwise, create a signal ready for connections from the outside
            self.i_clk = Signal()
            warnings.warn("Since no clock domain is specified, "
                          "to use the DDR pins provided by this module, "
                          "you must connect a clock signal to `i_clk`.")

        self.rx_ctl_iddr = Pin(1, "i", xdr=2)
        self.rx_data_iddr = Pin(self.data_width//2, "i", xdr=2)

        self.source = PHYEndpoint(data_width=self.data_width)

    def elaborate(self, platform):
        m = Module()

        rx_ctl = Signal()
        rx_data = Signal(8)

        m.d.comb += [
            # DDR input clocks for rx_ctl and rx_data[3:0]
            self.rx_ctl_iddr.i_clk.eq(self.i_clk),
            self.rx_data_iddr.i_clk.eq(self.i_clk)
        ]

        m.d[self.clk_domain] += [
            rx_ctl.eq(self.rx_ctl_iddr.i0),
            rx_data.eq(Cat(self.rx_data_iddr.i0,
                           self.rx_data_iddr.i1))
        ]

        rx_ctl_d = Signal()
        m.d[self.clk_domain] += [
            rx_ctl_d.eq(rx_ctl),
            self.source.stb.eq(rx_ctl),
            self.source.payload.data.eq(rx_data)
        ]
        m.d.comb += self.source.eop.eq(~rx_ctl & rx_ctl_d)

        return m


class EthRGMIITX(Elaboratable):
    """A generic transmitter module for RGMII.

    Note that this module does not implement any DDR modules, but provides
    :class:`nmigen.lib.io.Pins` objects representing the signals to be used as
    the inputs of those DDR modules. Using these signals are recommended.

    This module also has `o_clk`, representing the clock to be used as
    the clock of the DDRs. However, the user should consider whether
    using `o_clk` on all the TX-related DDRs is appropriate, especially because
    some platforms require a skew between the TX clock and the TX data pins.

    Parameters
    ----------
    clk_domain : str or None
        Clock domain to be used as the DDR clock. Optional and defaults to None,
        meaning the user must connect the DDR clock signal to an external signal.

    Attributes
    ----------
    data_width : int
        Number of bits per Ethernet packet. Equals to 8.
    sink : :class:`PHYEndpoint`
        Representation of the status and data of the packet to transmit. See :class:`PHYEndPoint`.
    o_clk : Signal()
        Clock signal used as input for the DDR outputs.
        If clk_domain has been specified, it is the clock signal of that domain.
        Otherwise, it can be externally driven by other signals.
    tx_clk_oddr : :class:`Pin`
        DDR output representation for the TX CLK. See below for its attributes.
    tx_ctl_oddr : :class:`Pin`
        DDR output representation for the TX CTL. See below for its attributes.
    tx_data_oddr : :class:`Pin`
        DDR output representation for the TX DATA. See below for its attributes.
        Its o0 represents the first 4 bits, and o1 represents the last 4 bits.

    Each of the DDR outputs above contains these attributes:
    o_clk : Signal()
        Clock signal for the DDR. Internally driven.
    o0 : Signal
        Value to output at the rising edge of o_clk. Internally driven.
    o1 : Signal
        Value to output at the falling edge of o_clk. Internally driven.
    """
    def __init__(self, *, clk_domain=None):
        self.data_width = 8
        if clk_domain:
            # If a clock domain name is given, use its clock signal for the DDR
            if isinstance(clk_domain, str):
                self.o_clk = ClockSignal(clk_domain)
            else:
                raise TypeError("Clock domain must be a string, not {!r}"
                                .format(clk_domain))
        else:
            # Otherwise, create a signal ready for connections from the outside
            self.o_clk = Signal()
            warnings.warn("Since no clock domain is specified, "
                          "to use the DDR pins provided by this module, "
                          "you must connect a clock signal to `o_clk`.")

        self.tx_clk_oddr = Pin(1, "o", xdr=2)
        self.tx_ctl_oddr = Pin(1, "o", xdr=2)
        self.tx_data_oddr = Pin(self.data_width//2, "o", xdr=2)

        self.sink = PHYEndpoint(data_width=self.data_width)

    def elaborate(self, platform):
        m = Module()

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
    """A generic transmitter/receiver module for RGMII.

    Note that this module does not implement any DDR modules, but its TX and RX modules 
    provide :class:`nmigen.lib.io.Pins` objects representing the signals to be used as
    the inputs or outputs of those DDR modules. Using these signals are recommended.

    Parameters
    ----------
    rx_clk_domain : str or None
        Clock domain to be used as the DDR clock for RX. Optional and defaults to None,
        meaning the user must connect the DDR clock signal to an external signal.
    tx_clk_domain : str or None
        Clock domain to be used as the DDR clock for TX. Optional and defaults to None,
        meaning the user must connect the DDR clock signal to an external signal.

    Attributes
    ----------
    rx : EthRGMIIRX
        RGMII receiver. See :class:`EthRGMIIRX`.
    tx : EthRGMIITX
        RGMII transmitter. See :class:`EthRGMIITX`.
    source : :class:`PHYEndpoint`
        Representation of the status and data of the packet received by the RX. See :class:`PHYEndPoint`.
    sink : :class:`PHYEndpoint`
        Representation of the status and data of the packet to transmit by the TX. See :class:`PHYEndPoint`.

    """
    def __init__(self, *args, rx_clk_domain=None, tx_clk_domain=None, **kwargs):
        self.rx     = EthRGMIIRX(*args, clk_domain=rx_clk_domain, **kwargs)
        self.tx     = EthRGMIITX(*args, clk_domain=tx_clk_domain, **kwargs)
        self.source = self.rx.source
        self.sink   = self.tx.sink

    def elaborate(self, platform):
        m = Module()

        m.submodules.rx = self.rx = self.rx
        m.submodules.tx = self.tx = self.tx

        return m
