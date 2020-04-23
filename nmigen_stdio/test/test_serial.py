import unittest
from nmigen import *
from nmigen.lib.fifo import SyncFIFO
from nmigen.back.pysim import *

# Formal Verification
from nmigen.test.utils import *
from nmigen.asserts import *

from ..serial import *


def simulation_test(dut, process):
    with Simulator(dut, vcd_file=open("test.vcd", "w")) as sim:
        sim.add_clock(1e-6)
        sim.add_sync_process(process)
        sim.run()


class AsyncSerialRXTestCase(unittest.TestCase):
    def tx_period(self):
        for _ in range((yield self.dut.divisor) + 1):
            yield

    def tx_bits(self, bits):
        for bit in bits:
            yield from self.tx_period()
            yield self.dut.i.eq(bit)

    def rx_test(self, bits, *, data=None, errors=None):
        def process():
            self.assertFalse((yield self.dut.r_rdy))
            yield self.dut.ack.eq(1)
            yield from self.tx_bits(bits)
            while not (yield self.dut.r_rdy):
                yield
            if data is not None:
                self.assertFalse((yield self.dut.err))
                self.assertEqual((yield self.dut.data), data)
            if errors is not None:
                self.assertTrue((yield self.dut.err))
                for error in errors:
                    self.assertTrue((yield getattr(self.dut.err, error)))
        simulation_test(self.dut, process)

    def rx_test_multiple(self, bits, *, data_bits, parity, dataset=None, errorset=None, continuous=False):
        def process():
            tx_bitlen = (data_bits+(2 if parity=="none" else 3))
            if len(bits) % tx_bitlen != 0:
                raise ValueError("Total number of bits {} is not a multiple of number of bits per transmission {}"
                                 .format(bits, tx_bitlen))
            self.assertFalse((yield self.dut.r_rdy))
            rx_count = 0
            for bit in bits:
                yield self.dut.ack.eq(1)
                for _ in range((yield self.dut.divisor) + 1):
                    if (yield self.dut.r_rdy):
                        if dataset is not None:
                            data = dataset[rx_count]
                            self.assertFalse((yield self.dut.err))
                            self.assertEqual((yield self.dut.data), data)
                        if errorset is not None:
                            errors = errorset[rx_count]
                            self.assertTrue((yield self.dut.err))
                            for error in errors:
                                self.assertTrue((yield getattr(self.dut.err, error)))
                        rx_count += 1
                        if not continuous:
                            for __ in range((yield self.dut.divisor) + 1):
                                yield
                    yield
                yield self.dut.i.eq(bit)
        simulation_test(self.dut, process)

    def test_8n1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="none")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1], data=0b01110101)

    def test_8n1_multiple_continuous(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="none")
        self.rx_test_multiple(
            [0, 0,1,0,1,0,1,0,1, 1,
             0, 1,0,1,0,1,0,1,0, 1], data_bits=8, parity="none",
            dataset=[0b10101010, 0b01010101],
            continuous=True
        )

    def test_8n1_multiple_notcontinuous(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="none")
        self.rx_test_multiple(
            [0, 0,1,0,1,0,1,0,1, 1,
             0, 1,0,1,0,1,0,1,0, 1], data_bits=8, parity="none",
            dataset=[0b10101010, 0b01010101],
            continuous=False
        )

    def test_16n1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=16, parity="none")
        self.rx_test([0, 1,0,1,0,1,1,1,0,1,1,1,1,0,0,0,0, 1],
                     data=0b0000111101110101)

    def test_8m1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="mark")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], data=0b01110101)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 1, 1], data=0b00110101)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], errors={"parity"})

    def test_8s1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="space")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], data=0b01110101)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 0, 1], data=0b00110101)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], errors={"parity"})

    def test_8e1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="even")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], data=0b01110101)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 0, 1], data=0b00110101)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], errors={"parity"})

    def test_8o1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="odd")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], data=0b01110101)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 1, 1], data=0b00110101)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], errors={"parity"})

    def test_err_frame(self):
        self.dut = AsyncSerialRX(divisor=7)
        self.rx_test([0, 0,0,0,0,0,0,0,0, 0], errors={"frame"})

    def test_err_overflow(self):
        self.dut = AsyncSerialRX(divisor=7)

        def process():
            self.assertFalse((yield self.dut.r_rdy))
            yield from self.tx_bits([0, 0,0,0,0,0,0,0,0, 1])
            yield from self.tx_period()
            self.assertFalse((yield self.dut.r_rdy))
            self.assertTrue((yield self.dut.err.overflow))

        simulation_test(self.dut, process)


class AsyncSerialTXTestCase(unittest.TestCase):
    def tx_period(self):
        for _ in range((yield self.dut.divisor) + 1):
            yield

    def tx_test(self, *, data):
        def process():
            yield self.dut.ack.eq(1)
            yield self.dut.data.eq(data)
            yield
            yield self.dut.ack.eq(0)
            for _ in range(10):
                yield from self.tx_period()
            yield from self.tx_period()     # Check 1 more period
        simulation_test(self.dut, process)

    def tx_test_multiple(self, *, dataset, continuous=False):
        def process():
            if continuous:
                yield self.dut.continuous.eq(1)
            yield self.dut.ack.eq(1)
            yield self.dut.data.eq(dataset[0])
            yield
            for _ in range(10):
                yield from self.tx_period()
            for data in dataset[1:]:
                if not continuous:
                    yield
                yield self.dut.data.eq(data)
                yield
                for _ in range(10):
                    yield from self.tx_period()
            yield from self.tx_period()     # Check 1 more period
        simulation_test(self.dut, process)

    def test_8n1(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=8, parity="none")
        self.tx_test(data=0b10101010)

    def test_8n1_multiple_continuous(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=8, parity="none")
        self.tx_test_multiple(dataset=[0b10101010, 0b01010101], continuous=True)

    def test_8n1_multiple_notcontinuous(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=8, parity="none")
        self.tx_test_multiple(dataset=[0b10101010, 0b01010101], continuous=False)

    def test_16n1(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=16, parity="none")
        self.tx_test(data=0b0101011011110100)

    def test_8m1(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=8, parity="mark")
        self.tx_test(data=0b00111100)

    def test_8s1(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=8, parity="space")
        self.tx_test(data=0b11011010)

    def test_8e1(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=8, parity="even")
        self.tx_test(data=0b00101110)

    def test_8o1(self):
        self.dut = AsyncSerialTX(divisor=7, data_bits=8, parity="odd")
        self.tx_test(data=0b01110011)

    def tx_bits(self, data_bits):
        for _ in range(data_bits):
            yield from self.tx_period()
            yield self.dut.o


class AsyncSerialLoopbackSpec(Elaboratable):
    """This FV model is to verify the behaviours of the transmitter and receiver with data of 
    variable width. The test case contains the following independent variables:

    * Data width: the number of bits to transmit per unit of transmission
    * Divisor: the value of the divisor register
    * Parity: the parity mode

    Only when different arguments are passed to the unit test function will these independent 
    variables change. 
    """
    def __init__(self, *, divisor, data_bits, parity):
        self.rx = AsyncSerialRX(divisor=divisor, data_bits=data_bits, parity=parity)
        self.tx = AsyncSerialTX(divisor=divisor, data_bits=data_bits, parity=parity)
        self.data_bits = data_bits
        self.parity = parity

    def elaborate(self, platform):
        m = Module()
        m.submodules.rx = rx = self.rx
        m.submodules.tx = tx = self.tx

        # RX FIFO: stores data received by RX
        m.submodules.rx_fifo = rx_fifo = SyncFIFO(width=self.data_bits, depth=1)
        # TX FIFO: stores data to be sent by TX
        m.submodules.tx_fifo = tx_fifo = SyncFIFO(width=self.data_bits, depth=1)

        m.d.comb += [
            # Connect TX to the TX FIFO
            tx.ack.eq(tx_fifo.r_rdy),
            tx_fifo.r_en.eq(~tx.busy),
            tx.data.eq(tx_fifo.r_data),
            # Connect TX output to RX input
            rx.i.eq(tx.o),
            # Connect RX to the RX FIFO
            rx.ack.eq(rx_fifo.w_rdy),
            rx_fifo.w_en.eq(rx.r_rdy),
            rx_fifo.w_data.eq(rx.data)
        ]

        m.domains += ClockDomain("sync")
        m.d.comb += ResetSignal("sync").eq(0)

        # Assumptions for TX
        fv_tx_data = AnyConst(self.data_bits)
        m.d.comb += Assume(Stable(tx.divisor))
        # Set up an FSM for TX such that only 1 data frame is sent per test
        with m.FSM() as tx_fsm:
            with m.State("TX-1"):
                with m.If(tx_fifo.w_rdy):
                    m.d.comb += [
                        tx_fifo.w_en.eq(1),
                        tx_fifo.w_data.eq(fv_tx_data),
                    ]
                    m.next = "TX-2"
            with m.State("TX-2"):
                with m.If((tx.bits_left == 0) & (tx.timer == 0)):
                    m.next = "DONE"
        tx_fsm.state.name = "fv_tx_fsm_state"

        # Assumptions for RX
        fv_rx_data = Signal(self.data_bits)
        m.d.comb += Assume(Stable(rx.divisor))
        # Set up an FSM for RX such that it expects 1 data frame from TX
        with m.FSM() as rx_fsm:
            with m.State("RX-1"):
                with m.If(~rx_fifo.w_rdy):
                    m.d.sync += rx_fifo.r_en.eq(1)
                    m.next = "RX-LATCH"
            with m.State("RX-LATCH"):
                with m.If(rx_fifo.r_rdy):
                    m.d.sync += [
                        fv_rx_data.eq(rx_fifo.r_data),
                        rx_fifo.r_en.eq(0)
                    ]
                    m.next = "CHECK"
            with m.State("CHECK"):
                with m.If((fv_rx_data == fv_tx_data) &
                          (rx.err == 0)):
                    m.next = "DONE"
        rx_fsm.state.name = "fv_rx_fsm_state"

        # Assume initial FSM states
        with m.If(Initial()):
            m.d.comb += [
                Assume(tx_fsm.ongoing("TX-1")),
                Assume(rx_fsm.ongoing("RX-1"))
            ]
        # Assertions
        with m.If((Past(rx_fsm.state) == rx_fsm.encoding["CHECK"]) &
                  ~Stable(rx_fsm.state)):
            m.d.comb += [
                Assert(rx_fsm.ongoing("DONE")),
                Assert(tx_fsm.ongoing("DONE"))
            ]
        # RX r_rdy
        with m.If(Past(rx.busy, 2) & ~Stable(rx.busy, 1)):
            m.d.comb += Assert(rx.r_rdy)
        with m.If(Stable(rx.r_rdy) & rx.r_rdy):
            m.d.comb += Assert(Stable(rx.data) &
                               Stable(rx.err.overflow) &
                               Stable(rx.err.frame) &
                               Stable(rx.err.parity))
        # TX w_done
        with m.If(Past(tx.busy) & ~Stable(tx.busy)):
            m.d.comb += Assert(tx.w_done)

        return m


class AsyncSerialLoopbackTestCase(FHDLTestCase):
    def check_formal(self, *, divisor, data_bits, parity):
        depth = (divisor+1) * (data_bits+(3 if parity!="none" else 2) + 2)
        self.assertFormal(AsyncSerialLoopbackSpec(divisor=divisor,
                                                  data_bits=data_bits,
                                                  parity=parity),
                          mode="bmc", depth=depth)

    def test_all_div7(self):
        list_data_bits = range(5, 9)
        list_parity = ["none", "mark", "space", "even", "odd"]
        for data_bits in list_data_bits:
            for parity in list_parity:
                with self.subTest(data_bits=data_bits, parity=parity):
                    self.check_formal(divisor=7, data_bits=data_bits, parity=parity)


class AsyncSerialBitstreamSpec(Elaboratable):
    """This FV model is to verify the continuous behaviour of the receiver with data of 
    variable width. The test case contains the following independent variables:

    * Data width: the number of bits to transmit per unit of transmission
    * Divisor: the value of the divisor register
    * Parity: the parity mode

    Only when different arguments are passed to the unit test function will these independent 
    variables change.
    """
    def __init__(self, *, divisor, data_bits, parity):
        self.rx = AsyncSerialRX(divisor=divisor, data_bits=data_bits, parity=parity)
        self.divisor = divisor
        self.data_bits = data_bits
        self.parity = parity

    def elaborate(self, platform):
        m = Module()
        m.submodules.rx = rx = self.rx

        len_bitstream = self.data_bits+(3 if self.parity!="none" else 2)
        # RX FIFO: stores data (the first sequence) received by RX
        m.submodules.rx_fifo = rx_fifo = SyncFIFO(width=self.data_bits, depth=1)
        # TX bit FIFO: stores the bit sequence generated and sends it to RX bit by bit
        m.submodules.txbit_fifo = txbit_fifo = SyncFIFO(width=1, depth=len_bitstream+2)
        # A const flag to determine if an extra bit of 0 or 1 should be sent
        fv_tx_extra_bit = AnyConst(1)
        # A flag to indicate if an extra bit is being sent
        fv_tx_extra_send = Signal()
        # A flag to indicate if the TX FIFO has started to be read
        fv_txfifo_start = Signal()
        # A flag to indicate if the Tx FIFO has ended
        fv_txfifo_end = Signal()
        with m.If(fv_txfifo_start):
            m.d.sync += rx.i.eq(
                Mux(fv_tx_extra_send, fv_tx_extra_bit, txbit_fifo.r_data)
            )
        with m.Else():
            m.d.sync += rx.i.eq(1)
        m.d.comb += [
            # Connect RX to the RX FIFO
            rx.ack.eq(rx_fifo.w_rdy),
            rx_fifo.w_en.eq(rx.r_rdy),
            rx_fifo.w_data.eq(rx.data)
        ]

        m.domains += [
            ClockDomain("sync"),
            ClockDomain("txclk")
        ]
        m.d.comb += [
            ResetSignal("sync").eq(0),
            ResetSignal("txclk").eq(0)
        ]

        # Assumptions for TX
        fv_tx_bitstream_val = AnyConst(len_bitstream)
        # Assume the bitstream doesn't have 1-bit delay
        m.d.comb += Assume(fv_tx_bitstream_val[0] != 1)
        fv_tx_bitstream = Signal(len_bitstream)
        # Delay for issuing an extra bit after the bitstream
        fv_tx_end_delay = Signal(range(self.divisor + 2))
        m.d.comb += [
            Assume(Stable(fv_tx_end_delay)),
            Assume((fv_tx_end_delay >= 0) & (fv_tx_end_delay <= rx.divisor + 1))
        ]
        # A const flag to determine if the rx_fifo is always full
        fv_tx_overflow = AnyConst(1)
        fv_tx_bitno = Signal(range(len_bitstream+1))
        fv_tx_timer = Signal.like(rx.divisor)
        fv_rx_data = Signal(self.data_bits)
        # A flag to indicate if the extra bit (0) has been issued
        fv_tx_extra_done = Signal()
        # FSM for storing the bit sequence generated
        fv_txfifo_num_bits = Signal(range(len_bitstream+2))
        with m.FSM(domain="txclk") as txfifo_fsm:
            with m.State("WRITE-PREP"):
                m.d.txclk += [
                    fv_tx_bitstream.eq(Cat(fv_tx_bitstream_val)),
                    fv_txfifo_num_bits.eq(0),
                    txbit_fifo.w_en.eq(1)
                ]
                m.next = "WRITE-BITSTREAM"
            with m.State("WRITE-BITSTREAM"):
                with m.If(fv_txfifo_num_bits != len_bitstream+1):
                    m.d.txclk += [
                        txbit_fifo.w_data.eq(fv_tx_bitstream[0]),
                        fv_tx_bitstream.eq(Cat(fv_tx_bitstream[1:], 0)),
                        fv_txfifo_num_bits.eq(fv_txfifo_num_bits + 1)
                    ]
                with m.Else():
                    m.d.txclk += txbit_fifo.w_en.eq(0)
                    m.next = "DONE"
        txfifo_fsm.state.name = "fv_txfifo_fsm_state"
        # FSM for sending the bit sequence bit by bit
        with m.FSM(domain="txclk") as tx_fsm:
            with m.State("TX-PREP"):
                m.d.txclk += txbit_fifo.r_en.eq(0)
                with m.If(txbit_fifo.r_rdy):
                    m.d.txclk += [
                        fv_tx_bitno.eq(len_bitstream),
                        fv_tx_timer.eq(1)
                    ]
                    m.next = "TX-SENDBIT"
            with m.State("TX-SENDBIT"):
                m.d.txclk += txbit_fifo.r_en.eq(0)
                with m.If(fv_tx_timer != 0):
                    m.d.txclk += fv_tx_timer.eq(fv_tx_timer - 1)
                    with m.If((fv_tx_timer == 1) & (fv_tx_bitno != 0)):
                        m.d.txclk += txbit_fifo.r_en.eq(1)
                        with m.If(fv_tx_bitno == len_bitstream):
                            m.d.txclk += fv_txfifo_start.eq(1)
                with m.Else():
                    m.d.txclk += [
                        fv_tx_bitno.eq(fv_tx_bitno - 1),
                        fv_tx_timer.eq(self.divisor),
                    ]
                    with m.If(fv_tx_bitno == 0):
                        m.d.txclk += fv_txfifo_end.eq(1)
                        with m.If(fv_tx_end_delay == 0):
                            m.d.txclk += [
                                fv_tx_timer.eq(self.divisor),
                                fv_tx_extra_send.eq(1)
                            ]
                            m.next = "TX-SENDEXTRA"
                        with m.Else():
                            m.d.txclk += fv_tx_timer.eq(fv_tx_end_delay)
                            m.next = "TX-ENDDELAY"
            with m.State("TX-ENDDELAY"):
                with m.If(fv_tx_timer != 0):
                    m.d.txclk += fv_tx_timer.eq(fv_tx_timer - 1)
                with m.Else():
                    m.d.txclk += [
                        fv_tx_timer.eq(self.divisor),
                        fv_tx_extra_send.eq(1)
                    ]
                    m.next = "TX-SENDEXTRA"
            with m.State("TX-SENDEXTRA"):
                with m.If(fv_tx_timer != 0):
                    m.d.txclk += fv_tx_timer.eq(fv_tx_timer - 1)
                with m.Else():
                    m.d.txclk += fv_tx_extra_done.eq(1)
                    m.next = "DONE"
        tx_fsm.state.name = "fv_tx_fsm_state"

        # Assumptions for RX
        m.d.comb += Assume(Stable(rx.divisor))
        # Connect RX to the RX FIFO
        with m.If(fv_tx_overflow):
            m.d.comb += Assume(rx_fifo.w_rdy == 0)
        with m.Else():
            m.d.comb += [
                rx_fifo.w_en.eq(rx.r_rdy),
                rx_fifo.w_data.eq(rx.data)
            ]
        # FSM for storing the received data
        with m.FSM() as rx_fsm:
            with m.State("RX-1"):
                with m.If(~rx_fifo.w_rdy):
                    m.d.sync += rx_fifo.r_en.eq(1)
                    m.next = "RX-LATCH"
            with m.State("RX-LATCH"):
                with m.If(rx_fifo.r_rdy):
                    m.d.sync += [
                        fv_rx_data.eq(rx_fifo.r_data),
                        rx_fifo.r_en.eq(0)
                    ]
                    m.next = "CHECK"
            with m.State("CHECK"):
                with m.If((rx.err != 0)):
                    m.next = "ERROR"
                with m.Else():
                    m.next = "DONE"
        rx_fsm.state.name = "fv_rx_fsm_state"

        # Assume initial FSM states
        with m.If(Initial()):
            m.d.comb += [
                Assume(txfifo_fsm.ongoing("WRITE-PREP")),
                Assume(tx_fsm.ongoing("TX-PREP")),
                Assume(fv_txfifo_start == 0),
                Assume(fv_tx_extra_done == 0),
                Assume(rx_fsm.ongoing("RX-1"))
            ]

        # Assertions
        with m.If(Past(rx_fsm.state) == rx_fsm.encoding["CHECK"]):
            m.d.comb += Assert(rx_fsm.ongoing("DONE") | rx_fsm.ongoing("ERROR"))
        # RX r_rdy
        with m.If(Past(rx.busy, 2) & ~Stable(rx.busy, 1)):
            m.d.comb += Assert(rx.r_rdy)
        with m.If(Stable(rx.r_rdy) & rx.r_rdy):
            m.d.comb += Assert(Stable(rx.data) &
                               Stable(rx.err.overflow) &
                               Stable(rx.err.frame) &
                               Stable(rx.err.parity))

        with m.If(rx_fsm.ongoing("DONE")):
            m.d.comb += Assert((fv_tx_bitstream_val[0] == 0) & (fv_tx_bitstream_val[-1] == 1))
            if self.parity == "mark":
                m.d.comb += Assert((fv_tx_bitstream_val[-2] == 1))
            elif self.parity == "space":
                m.d.comb += Assert((fv_tx_bitstream_val[-2] == 0))
            elif self.parity == "even":
                m.d.comb += Assert((fv_tx_bitstream_val[-2] == fv_rx_data.xor()))
            elif self.parity == "odd":
                m.d.comb += Assert((fv_tx_bitstream_val[-2] == ~fv_rx_data.xor()))
            m.d.comb += Assert(~fv_tx_overflow)
            if self.parity == "none":
                m.d.comb += Assert(fv_rx_data == fv_tx_bitstream_val[1:-1])
            else:
                m.d.comb += Assert(fv_rx_data == fv_tx_bitstream_val[1:-2])

        with m.Elif(rx_fsm.ongoing("ERROR")):
            with m.If(rx.err.frame):
                m.d.comb += Assert((fv_tx_bitstream_val[0] != 0) | (fv_tx_bitstream_val[-1] != 1))
            if self.parity == "none":
                m.d.comb += Assert(~rx.err.parity)
            else:
                with m.If(rx.err.parity):
                    if self.parity == "mark":
                        m.d.comb += Assert((fv_tx_bitstream_val[-2] != 1))
                    elif self.parity == "space":
                        m.d.comb += Assert((fv_tx_bitstream_val[-2] != 0))
                    elif self.parity == "even":
                        m.d.comb += Assert((fv_tx_bitstream_val[-2] != fv_rx_data.xor()))
                    elif self.parity == "odd":
                        m.d.comb += Assert((fv_tx_bitstream_val[-2] != ~fv_rx_data.xor()))
            with m.If(rx.err.overflow):
                m.d.comb += Assert(fv_tx_overflow)

        with m.If(fv_tx_extra_done):
            m.d.comb += [
                Assert(rx.i == fv_tx_extra_bit),
                Assert(rx.busy == ~fv_tx_extra_bit)
            ]

        return m


class AsyncSerialBitstreamTestCase(FHDLTestCase):
    def check_formal(self, *, divisor, data_bits, parity):
        depth = (divisor+1) * (data_bits+(3 if parity!="none" else 2) + 2) + 6
        self.assertFormal(AsyncSerialBitstreamSpec(divisor=divisor,
                                                   data_bits=data_bits,
                                                   parity=parity),
                          mode="bmc", depth=depth)

    def test_all_div7(self):
        list_data_bits = range(5, 9)
        list_parity = ["none", "mark", "space", "even", "odd"]
        for data_bits in list_data_bits:
            for parity in list_parity:
                with self.subTest(data_bits=data_bits, parity=parity):
                    self.check_formal(divisor=7, data_bits=data_bits, parity=parity)
