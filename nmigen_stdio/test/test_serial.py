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
            self.assertFalse((yield self.dut.rdy))
            yield self.dut.ack.eq(1)
            yield from self.tx_bits(bits)
            while not (yield self.dut.rdy):
                yield
            if data is not None:
                self.assertFalse((yield self.dut.err))
                self.assertEqual((yield self.dut.data), data)
            if errors is not None:
                self.assertTrue((yield self.dut.err))
                for error in errors:
                    self.assertTrue((yield getattr(self.dut.err, error)))
        simulation_test(self.dut, process)

    def test_8n1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="none")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1], data=0b10101110)

    def test_16n1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=16, parity="none")
        self.rx_test([0, 1,0,1,0,1,1,1,0,1,1,1,1,0,0,0,0, 1],
                     data=0b1010111011110000)

    def test_8m1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="mark")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], data=0b10101110)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 1, 1], data=0b10101100)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], errors={"parity"})

    def test_8s1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="space")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], data=0b10101110)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 0, 1], data=0b10101100)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], errors={"parity"})

    def test_8e1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="even")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], data=0b10101110)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 0, 1], data=0b10101100)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], errors={"parity"})

    def test_8o1(self):
        self.dut = AsyncSerialRX(divisor=7, data_bits=8, parity="odd")
        self.rx_test([0, 1,0,1,0,1,1,1,0, 0, 1], data=0b10101110)
        self.rx_test([0, 1,0,1,0,1,1,0,0, 1, 1], data=0b10101100)
        self.rx_test([0, 1,0,1,0,1,1,1,0, 1, 1], errors={"parity"})

    def test_err_frame(self):
        self.dut = AsyncSerialRX(divisor=7)
        self.rx_test([0, 0,0,0,0,0,0,0,0, 0], errors={"frame"})

    def test_err_overflow(self):
        self.dut = AsyncSerialRX(divisor=7)
        def process():
            self.assertFalse((yield self.dut.rdy))
            yield from self.tx_bits([0, 0,0,0,0,0,0,0,0, 1])
            yield from self.tx_period()
            self.assertFalse((yield self.dut.rdy))
            self.assertTrue((yield self.dut.err.overflow))
        simulation_test(self.dut, process)


class AsyncSerialTXTestCase(unittest.TestCase):
    def tx_period(self):
        for _ in range((yield self.dut.divisor) + 1):
            yield

    def tx_dummy_test(self):
        self.dut = AsyncSerialTX(divisor=7)
        m = Module()
        m.submodules.tx = self.dut
        def process():
            yield self.dut.ack.eq(1)
            yield self.dut.data.eq(0xAA)
            for _ in range(10):
                yield from self.tx_period()
            yield
            yield
        simulation_test(m, process)

    def tx_bits(self, data_bits):
        for _ in range(data_bits):
            yield from self.tx_period()
            yield self.dut.o


class AsyncSerialLoopbackSpec(Elaboratable):
    def __init__(self, *, divisor, data_bits, parity):
        self.rx = AsyncSerialRX(divisor=divisor, data_bits=data_bits, parity=parity)
        self.tx = AsyncSerialTX(divisor=divisor, data_bits=data_bits, parity=parity)
        self.data_bits = data_bits
        self.parity = parity

    def elaborate(self, platform):
        m = Module()
        m.submodules.rx = rx = self.rx
        m.submodules.tx = tx = self.tx

        m.submodules.rx_fifo = rx_fifo = SyncFIFO(width=self.data_bits, depth=1)
        m.submodules.tx_fifo = tx_fifo = SyncFIFO(width=self.data_bits, depth=1)

        m.d.comb += [
            #
            tx.ack.eq(tx_fifo.r_rdy),
            tx_fifo.r_en.eq(tx.rdy),
            tx.data.eq(tx_fifo.r_data),
            #
            rx.i.eq(tx.o),
            #
            rx.ack.eq(rx_fifo.w_rdy),
            rx_fifo.w_en.eq(rx.rdy),
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
                with m.If((tx.bitno == 0) & (tx.timer == 0)):
                    m.next = "DONE"
        tx_fsm.state.name = "fv_tx_fsm_state"

        # Assumptions for RX
        fv_rx_data = Signal(self.data_bits)
        m.d.comb += Assume(Stable(rx.divisor))
        # Set up an FSM for RX such that it expects 1 data frame from TX
        with m.FSM() as rx_fsm:
            with m.State("RX-1"):
                with m.If((rx.bitno == 0) & (rx.timer == 0)):
                    m.d.sync += rx_fifo.r_en.eq(1)
                    m.next = "RX-LATCH"
            with m.State("RX-LATCH"):
                with m.If(rx_fifo.r_rdy):
                    m.d.sync += fv_rx_data.eq(rx_fifo.r_data)
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
        with m.If(Past(rx_fsm.state) == rx_fsm.encoding["CHECK"]):
            m.d.comb += [
                Assert(rx_fsm.ongoing("DONE")),
                Assert(tx_fsm.ongoing("DONE"))
            ]

        return m


class AsyncSerialLoopbackTestCase(FHDLTestCase):
    def check_formal(self, *, divisor, data_bits, parity):
        depth = (divisor+1) * (data_bits+(3 if parity!="none" else 2)) + 5
        self.assertFormal(AsyncSerialLoopbackSpec(divisor=divisor, data_bits=data_bits, parity=parity),
                          mode="bmc", depth=depth)

    def check_all(self):
        # Commonly-used divisors, from "Embedded Systems Design with 8051 Microcontrollers: Hardware and Software", p5-54, Fig 6.11
        list_divisor = [0x417, 0x30, 0x18, 0xC, 0x6, 0x3, 0x2, 0x1]
        list_data_bits = range(5, 9)
        list_parity = ["none", "mark", "space", "even", "odd"]
        for divisor in list_divisor:
            for data_bits in list_data_bits:
                for parity in list_parity:
                    with self.subTest(divisor=divisor, data_bits=data_bits, parity=parity):
                        self.check_formal(divisor=divisor, data_bits=data_bits, parity=parity)

    def check_all_div6(self):
        list_data_bits = range(5, 9)
        list_parity = ["none", "mark", "space", "even", "odd"]
        for data_bits in list_data_bits:
            for parity in list_parity:
                with self.subTest(data_bits=data_bits, parity=parity):
                    self.check_formal(divisor=6, data_bits=data_bits, parity=parity)


class AsyncSerialBitstreamSpec(Elaboratable):
    def __init__(self, *, divisor, data_bits, parity):
        self.rx = AsyncSerialRX(divisor=divisor, data_bits=data_bits, parity=parity)
        self.divisor = divisor
        self.data_bits = data_bits
        self.parity = parity

    def elaborate(self, platform):
        m = Module()
        m.submodules.rx = rx = self.rx

        len_bitstream = self.data_bits+(3 if self.parity!="none" else 2)
        m.submodules.rx_fifo = rx_fifo = SyncFIFO(width=self.data_bits, depth=1)
        m.submodules.txbit_fifo = txbit_fifo = SyncFIFO(width=1, depth=len_bitstream+1)

        fv_txfifo_start = Signal()
        m.d.txclk += fv_txfifo_start.eq(fv_txfifo_start)
        with m.If(fv_txfifo_start):
            m.d.comb += rx.i.eq(txbit_fifo.r_data)
        with m.Else():
            m.d.comb += rx.i.eq(1)
        m.d.comb += [
            #
            rx.ack.eq(rx_fifo.w_rdy),
            rx_fifo.w_en.eq(rx.rdy),
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
        m.d.comb += Assume(fv_tx_bitstream_val[-1] != 1)
        fv_tx_bitstream = Signal(len_bitstream+1)
        fv_tx_extra_bit = AnyConst(1)       # A const bit representing the extra bit after the bitstream
        fv_tx_overflow = AnyConst(1)        # A const flag to determine if the rx_fifo is always full
        fv_tx_bitno = Signal(range(len(fv_tx_bitstream)))
        fv_tx_timer = Signal.like(rx.divisor)
        fv_rx_data = Signal(self.data_bits)
        # 
        fv_txfifo_num_bits = Signal(range(len(fv_tx_bitstream)))
        with m.FSM(domain="txclk") as txfifo_fsm:
            with m.State("WRITE-PREP"):
                m.d.txclk += [
                    fv_tx_bitstream.eq(Cat(fv_tx_extra_bit, fv_tx_bitstream_val)),
                    fv_txfifo_num_bits.eq(len(fv_tx_bitstream)),
                ]
                with m.If(fv_txfifo_num_bits == len(fv_tx_bitstream)):
                    m.d.comb += txbit_fifo.w_en.eq(1)
                    m.next = "WRITE-BITSTREAM"
            with m.State("WRITE-BITSTREAM"):
                m.d.comb += txbit_fifo.w_en.eq(1)
                with m.If(fv_txfifo_num_bits != 0):
                    m.d.comb += txbit_fifo.w_data.eq(fv_tx_bitstream[-1])
                    m.d.txclk += [
                        fv_tx_bitstream.eq(Cat(0, fv_tx_bitstream[:-1])),
                        fv_txfifo_num_bits.eq(fv_txfifo_num_bits - 1)
                    ]
                with m.Else():
                    m.next = "DONE"
        txfifo_fsm.state.name = "fv_txfifo_fsm_state"
        # 
        # Assume a variable timer initial value for bit 0
        fv_tx_bit0_divisor = AnyConst(range(self.divisor))
        m.d.comb += Assume(fv_tx_bit0_divisor > (self.divisor >> 1))
        fv_tx_done = Signal()
        m.d.txclk += fv_tx_done.eq(fv_tx_done)
        with m.FSM(domain="txclk") as tx_fsm:
            with m.State("TX-PREP"):
                with m.If(txbit_fifo.r_rdy):
                    m.d.txclk += txbit_fifo.r_en.eq(1)
                with m.If(txbit_fifo.r_en):
                    m.d.txclk += [
                        txbit_fifo.r_en.eq(0),
                        fv_tx_bitno.eq(len(fv_tx_bitstream) - 1),
                        fv_tx_timer.eq(fv_tx_bit0_divisor),
                        fv_txfifo_start.eq(1)
                    ]
                    m.next = "TX-SENDBIT"
            with m.State("TX-SENDBIT"):
                m.d.txclk += txbit_fifo.r_en.eq(0)
                with m.If(fv_tx_timer != 0):
                    m.d.txclk += fv_tx_timer.eq(fv_tx_timer - 1)
                    with m.If((fv_tx_timer == 1) & (fv_tx_bitno != 0)):
                        m.d.txclk += txbit_fifo.r_en.eq(1)
                with m.Else():
                    m.d.txclk += [
                        fv_tx_bitno.eq(fv_tx_bitno - 1),
                        fv_tx_timer.eq(self.divisor),
                    ]
                    with m.If(fv_tx_bitno == 0):
                        m.d.txclk += fv_tx_done.eq(1)
                        m.next = "DONE"
        tx_fsm.state.name = "fv_tx_fsm_state"

        # Assumptions for RX
        m.d.comb += Assume(Stable(rx.divisor))
        #
        with m.If(fv_tx_overflow):
            m.d.comb += Assume(rx_fifo.w_rdy == 0)
        with m.Else():
            m.d.comb += [
                rx_fifo.w_en.eq(rx.rdy),
                rx_fifo.w_data.eq(rx.data)
            ]
        #
        with m.FSM() as rx_fsm:
            with m.State("RX-1"):
                with m.If((rx.bitno == 0) & (rx.timer == 0)):
                    m.d.sync += rx_fifo.r_en.eq(1)
                    m.next = "RX-LATCH"
            with m.State("RX-LATCH"):
                with m.If(rx_fifo.r_rdy):
                    m.d.sync += fv_rx_data.eq(rx_fifo.r_data)
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
                Assume(fv_tx_done == 0),
                Assume(rx_fsm.ongoing("RX-1"))
            ]

        # Assertions
        with m.If(Past(rx_fsm.state) == rx_fsm.encoding["CHECK"]):
            m.d.comb += Assert(rx_fsm.ongoing("DONE") | rx_fsm.ongoing("ERROR"))

        with m.If(rx_fsm.ongoing("DONE")):
            m.d.comb += Assert((fv_tx_bitstream_val[-1] == 0) & (fv_tx_bitstream_val[0] == 1))
            if self.parity == "mark":
                m.d.comb += Assert((fv_tx_bitstream_val[1] == 1))
            elif self.parity == "space":
                m.d.comb += Assert((fv_tx_bitstream_val[1] == 0))
            elif self.parity == "even":
                m.d.comb += Assert((fv_tx_bitstream_val[1] == fv_rx_data.xor()))
            elif self.parity == "odd":
                m.d.comb += Assert((fv_tx_bitstream_val[1] == ~fv_rx_data.xor()))
            m.d.comb += Assert(~fv_tx_overflow)

        with m.Elif(rx_fsm.ongoing("ERROR")):
            with m.If(rx.err.frame):
                m.d.comb += Assert((fv_tx_bitstream_val[-1] != 0) | (fv_tx_bitstream_val[0] != 1))
            if self.parity == "none":
                m.d.comb += Assert(~rx.err.parity)
            else:
                with m.If(rx.err.parity):
                    if self.parity == "mark":
                        m.d.comb += Assert((fv_tx_bitstream_val[1] != 1))
                    elif self.parity == "space":
                        m.d.comb += Assert((fv_tx_bitstream_val[1] != 0))
                    elif self.parity == "even":
                        m.d.comb += Assert((fv_tx_bitstream_val[1] != fv_rx_data.xor()))
                    elif self.parity == "odd":
                        m.d.comb += Assert((fv_tx_bitstream_val[1] != ~fv_rx_data.xor()))
            with m.If(rx.err.overflow):
                m.d.comb += Assert(fv_tx_overflow)

        with m.If(~fv_tx_extra_bit & fv_tx_done):
            m.d.comb += [
                Assert(rx.shreg.stop == 0),
                Assert(rx.busy == 1)
            ]
        with m.Elif(fv_tx_extra_bit & fv_tx_done):
            m.d.comb += [
                Assert(rx.i == 1),
                Assert(rx.busy == 0)
            ]

        return m


class AsyncSerialBitstreamTestCase(FHDLTestCase):
    def check_formal(self, *, divisor, data_bits, parity):
        depth = (divisor+2) * (data_bits+(3 if parity!="none" else 2)+1) + 5
        self.assertFormal(AsyncSerialBitstreamSpec(divisor=divisor, data_bits=data_bits, parity=parity),
                          mode="bmc", depth=depth)

    def check_all(self):
        # Commonly-used divisors, from "Embedded Systems Design with 8051 Microcontrollers: Hardware and Software", p5-54, Fig 6.11
        list_divisor = [0x417, 0x30, 0x18, 0xC, 0x6, 0x3, 0x2, 0x1]
        list_data_bits = range(5, 9)
        list_parity = ["none", "mark", "space", "even", "odd"]
        for divisor in list_divisor:
            for data_bits in list_data_bits:
                for parity in list_parity:
                    with self.subTest(divisor=divisor, data_bits=data_bits, parity=parity):
                        self.check_formal(divisor=divisor, data_bits=data_bits, parity=parity)

    def check_all_div6(self):
        list_data_bits = range(5, 9)
        list_parity = ["none", "mark", "space", "even", "odd"]
        for data_bits in list_data_bits:
            for parity in list_parity:
                with self.subTest(data_bits=data_bits, parity=parity):
                    self.check_formal(divisor=6, data_bits=data_bits, parity=parity)
