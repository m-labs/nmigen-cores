from nmigen import *
from nmigen.lib.cdc import FFSynchronizer
from nmigen.utils import bits_for


__all__ = ["AsyncSerialRX", "AsyncSerialTX", "AsyncSerial"]


def _check_parity(parity):
    choices = ("none", "mark", "space", "even", "odd")
    if parity not in choices:
        raise ValueError("Invalid parity {!r}; must be one of {}"
                         .format(parity, ", ".join(choices)))


def _compute_parity_bit(data, parity):
    if parity == "none":
        return C(0, 0)
    if parity == "mark":
        return C(1, 1)
    if parity == "space":
        return C(0, 1)
    if parity == "even":
        return data.xor()
    if parity == "odd":
        return ~data.xor()
    assert False


def _wire_layout(data_bits, parity="none"):
    return [
        ("start",  1),
        ("data",   data_bits),
        ("parity", 0 if parity == "none" else 1),
        ("stop",   1)
    ]


class AsyncSerialRX(Elaboratable):
    """An UART receiver module. Receives the LSB first and MSB last.
    """
    def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none", pins=None):
        _check_parity(parity)
        self._parity = parity

        self.divisor = Signal(divisor_bits or bits_for(divisor), reset=divisor)

        self.data = Signal(data_bits)
        self.err  = Record([
            ("overflow", 1),
            ("frame",    1),
            ("parity",   1),
        ])
        # rx.ack: Externally driven, to indicate if transfer of received data is enabled
        self.ack  = Signal()
        # rx.busy: Internally driven, to indicate if there is an ongoing data transfer
        self.busy = Signal()
        # rx.r_rdy: Internally driven, to indicate if received data is ready to be read
        self.r_rdy  = Signal()

        self.i    = Signal(reset=1)
        self._pins = pins

        self.timer = Signal.like(self.divisor)
        self.shreg = Record(_wire_layout(len(self.data), self._parity))
        self.bits_left = Signal(range(len(self.shreg)))
        self.done = Signal()

    def elaborate(self, platform):
        m = Module()

        timer = self.timer
        shreg = self.shreg
        bits_left = self.bits_left
        done = self.done

        if self._pins is not None:
            m.submodules += FFSynchronizer(self._pins.rx.i, self.i, reset=1)

        with m.FSM() as fsm:
            with m.State("IDLE"):
                with m.If(~self.i):
                    m.d.sync += [
                        bits_left.eq(len(shreg) - 1),
                        timer.eq(self.divisor >> 1)
                    ]
                    m.next = "BUSY"

            with m.State("BUSY"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer - 1)
                with m.Else():
                    m.d.sync += [
                        shreg.eq(Cat(shreg[1:], self.i)),
                        bits_left.eq(bits_left - 1),
                        timer.eq(self.divisor)
                    ]
                    with m.If(((self.divisor != 0) & (bits_left == 0)) |
                              ((self.divisor == 0) & (bits_left == 1))):
                        m.next = "DONE"

            with m.State("DONE"):
                with m.If(timer == self.divisor):
                    with m.If(self.ack):
                        m.d.sync += [
                            self.data.eq(shreg.data),
                            self.err.frame .eq(~((shreg.start == 0) & (shreg.stop == 1))),
                            self.err.parity.eq(~(shreg.parity ==
                                                 _compute_parity_bit(shreg.data, self._parity)))
                        ]
                    m.d.sync += [
                        self.err.overflow.eq(~self.ack),
                        self.r_rdy.eq(1)
                    ]
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer - 1)
                with m.Else():
                    # Check if this next bit is start bit
                    # (useful for divisor == 0)
                    with m.If(~self.i):
                        m.d.sync += [
                            shreg.eq(Cat(shreg[1:], self.i)),
                            bits_left.eq(len(shreg) - 2),
                            timer.eq(self.divisor)
                        ]
                        m.next = "BUSY"
                    with m.Else():
                        m.next = "IDLE"

        m.d.comb += self.busy.eq(fsm.ongoing("BUSY"))
        with m.If(self.r_rdy):
            m.d.sync += self.r_rdy.eq(0)

        return m


class AsyncSerialTX(Elaboratable):
    """An UART transmitter module. Transmits the LSB first and MSB last.
    """
    def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none", pins=None):
        _check_parity(parity)
        self._parity = parity

        self.divisor = Signal(divisor_bits or bits_for(divisor), reset=divisor)
        # tx.continuous: Externally driven; no breaks between transmission of "packets" if asserted
        #                Useful if ACK stays high until the entire transmission has ended
        self.continuous = Signal(reset=0)

        self.data = Signal(data_bits)
        # tx.ack: Externally driven, to indicate if transfer of data to transmit is enabled
        self.ack  = Signal()
        # tx.busy: Internally driven, to indicate if there is an ongoing data transfer
        self.busy = Signal()
        # tx.w_done: Internally driven, to indicate if data has been transmitted
        self.w_done  = Signal()

        self.o    = Signal(reset=1)
        self._pins = pins

        self.timer = Signal.like(self.divisor)
        self.shreg = Record(_wire_layout(len(self.data), self._parity))
        self.bits_left = Signal(range(len(self.shreg) + 1))

    def elaborate(self, platform):
        m = Module()

        timer = self.timer
        shreg = self.shreg
        bits_left = self.bits_left

        if self._pins is not None:
            m.d.comb += self._pins.tx.o.eq(self.o)

        with m.FSM() as fsm:
            with m.State("IDLE"):
                m.d.sync += self.o.eq(1)
                with m.If(self.ack):
                    m.d.sync += [
                        self.o.eq(0),       # Issue start bit ASAP
                        bits_left.eq(len(self.shreg)),
                        timer.eq(self.divisor)
                    ]
                    # In continous mode, data is valid only at beginning of transmission
                    # and after ACK is asserted
                    with m.If(self.continuous):
                        m.next = "LATCH-DATA"
                    # In normal mode, data is valid at the same time as ACK is asserted
                    with m.Else():
                        m.d.sync += [
                            shreg.start .eq(0),
                            shreg.data  .eq(self.data),
                            shreg.parity.eq(_compute_parity_bit(self.data, self._parity)),
                            shreg.stop  .eq(1),
                        ]
                        m.next = "BUSY"

            with m.State("LATCH-DATA"):
                # In continuous mode, data is valid only at beginning of transmission;
                # Latch the data either before timer reaches 0 on the first bit,
                #   or, if timer is always 0 (i.e. divisor is 0),
                #   latch the first data bit to output for the next clock directly
                with m.If(self.divisor == 0):
                    # On the next clock, do:
                    m.d.sync += [
                        self.o.eq(self.data[0]),    # Send data[0], and
                        shreg.eq(Cat(               # Store to shreg:
                            self.data[1:],                                  # data[1:]
                            _compute_parity_bit(self.data, self._parity),   # parity
                            1, 0, 0                                         # stop bit
                        )) 
                    ]
                with m.Else():
                    # On the next clock, latch the data to shreg and keep counting down
                    m.d.sync += [
                        shreg.start .eq(0),
                        shreg.data  .eq(self.data),
                        shreg.parity.eq(_compute_parity_bit(self.data, self._parity)),
                        shreg.stop  .eq(1),
                        timer.eq(timer - 1)
                    ]
                # Resume BUSY state
                m.next = "BUSY"

            with m.State("BUSY"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer - 1)
                with m.If(((timer == 0) & (bits_left != 1)) | ((timer == 1) & (bits_left == 1))):
                    m.d.sync += bits_left.eq(bits_left - 1),
                    with m.If(bits_left == len(self.shreg)):
                        m.d.sync += [
                            self.o.eq(shreg[1]),            # Start bit has already been issued
                            shreg.eq(Cat(shreg[2:], 0, 0))  # Skip it during transfer
                        ]
                    with m.Elif(bits_left != 1):
                        m.d.sync += [
                            self.o.eq(shreg[0]),
                            shreg.eq(Cat(shreg[1:], 0)),
                        ]
                    with m.If(bits_left != 1):
                        m.d.sync += timer.eq(self.divisor)
                    with m.Else():
                        m.next = "DONE"

            with m.State("DONE"):
                # In continuous mode, accept next sequence if ACK is still high
                with m.If(self.ack & self.continuous):
                    m.d.sync += [
                        self.o.eq(0),       # Issue start bit ASAP
                        bits_left.eq(len(self.shreg)),
                        timer.eq(self.divisor)
                    ]
                    m.next = "LATCH-DATA"
                # Set back to IDLE if ACK is low
                with m.Else():
                    m.d.sync += self.o.eq(1)
                    m.next = "IDLE"

        m.d.comb += [
            self.w_done.eq(fsm.ongoing("DONE")),
            self.busy.eq(fsm.ongoing("LATCH-DATA") | fsm.ongoing("BUSY"))
        ]

        return m


class AsyncSerial(Elaboratable):
    def __init__(self, *, divisor, divisor_bits=None, **kwargs):
        self.divisor = Signal(divisor_bits or bits_for(divisor), reset=divisor)

        self.rx = AsyncSerialRX(divisor=divisor, divisor_bits=divisor_bits, **kwargs)
        self.tx = AsyncSerialTX(divisor=divisor, divisor_bits=divisor_bits, **kwargs)

    def elaborate(self, platform):
        m = Module()
        m.submodules.rx = self.rx
        m.submodules.tx = self.tx
        m.d.comb += [
            self.rx.divisor.eq(self.divisor),
            self.tx.divisor.eq(self.divisor),
        ]
        return m
