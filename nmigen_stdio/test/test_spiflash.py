import unittest
from nmigen import *
from nmigen.lib.fifo import SyncFIFO
from nmigen.back.pysim import *

from ..spiflash import *


def simulation_test(dut, process):
    with Simulator(dut, vcd_file=open("test.vcd", "w")) as sim:
        sim.add_clock(1e-6)
        sim.add_sync_process(process)
        sim.run()


class SPIFlashFastReadTestCase(unittest.TestCase):
    def divisor_period(self):
        for _ in range((yield self.dut.divisor) + 1):
            yield

    class SuperNaiveFlash(Elaboratable):
        def __init__(self, *, dut, data, data_size, dummy_cycles):
            self.dut = dut
            self.data = data
            self.data_size = data_size
            self.dummy_cycles = dummy_cycles
        def elaborate(self, platform):
            m = Module()
            data_sig = Signal(self.data_size)
            recv_data = SyncFIFO(width=self.dut.spi_width,
                                 depth=(self.dut.cmd_width+self.dut.addr_width)//self.dut.spi_width)
            stored_data = SyncFIFO(width=self.dut.spi_width,
                                   depth=self.dut.data_width//self.dut.spi_width)
            stored_data_num_sent = Signal(range(stored_data.depth+1),
                                          reset=stored_data.depth)
            dummy_counter = Signal(range(self.dummy_cycles),
                                   reset=self.dummy_cycles - 1)
            m.submodules.recv_data = recv_data
            m.submodules.stored_data = stored_data
            if self.dut.spi_width == 1:
                m.d.comb += recv_data.w_data.eq(self.dut.mosi)
            else:
                m.d.comb += recv_data.w_data.eq(self.dut.dq.o)
            with m.If(stored_data.r_rdy & stored_data.r_en):
                if self.dut.spi_width == 1:
                    m.d.sync += self.dut.miso.eq(stored_data.r_data)
                else:
                    m.d.sync += self.dut.dq.i.eq(stored_data.r_data)
            with m.FSM() as fsm:
                with m.State("INIT"):
                    m.d.sync += data_sig.eq(self.data)
                    with m.If(self.dut.cs):
                        # Set w_en 1 clock earlier
                        with m.If(self.dut.counter == 1):
                            m.d.comb += recv_data.w_en.eq(1)
                        with m.Else():
                            m.d.comb += recv_data.w_en.eq(0)
                        with m.If(~recv_data.w_rdy & stored_data.w_rdy):
                            m.next = "PUT-DATA"
                with m.State("PUT-DATA"): 
                    with m.If(stored_data_num_sent != 0):
                        m.d.comb += [
                            stored_data.w_en.eq(1),
                            stored_data.w_data.eq(data_sig[-self.dut.spi_width:])
                        ]
                        m.d.sync += [
                            stored_data_num_sent.eq(stored_data_num_sent+1),
                            data_sig.eq(Cat(Repl(0, self.dut.spi_width), data_sig[:-self.dut.spi_width]))
                        ]
                    with m.Else():
                        m.d.comb += stored_data.w_en.eq(0)
                    with m.If((dummy_counter != 0) & (self.dut.counter == 0)):
                        m.d.sync += dummy_counter.eq(dummy_counter - 1)
                    with m.Elif((dummy_counter == 0) &
                                (self.dut.counter == self.dut._divisor_val>>1+1)):
                        m.d.comb += stored_data.r_en.eq(1)
                        m.next = "RETURN-DATA"
                with m.State("RETURN-DATA"):
                    with m.If(self.dut.counter == self.dut._divisor_val>>1+1):
                        m.d.comb += stored_data.r_en.eq(1)
                    with m.Else():
                        m.d.comb += stored_data.r_en.eq(0)
            return m

    def test_n25q128a_extended(self):
        self.cmd_dict = {"FAST_READ": 0x0b}
        self.dummy_cycles_dict = {"FAST_READ": 8}
        self.dut = SPIFlash(protocol="extended", 
                            addr_width=24,
                            data_width=8,
                            cmd_width=8,
                            cmd_dict=self.cmd_dict,
                            dummy_cycles_dict=self.dummy_cycles_dict)
        self.flash = (SPIFlashFastReadTestCase.
                      SuperNaiveFlash(dut=self.dut,
                                      data=0xAA,
                                      data_size=8,
                                      dummy_cycles=8))
        self.simple_test()

    def test_n25q128a_dual(self):
        self.cmd_dict = {"FAST_READ": 0xbb}
        self.dummy_cycles_dict = {"FAST_READ": 8}
        self.dut = SPIFlash(protocol="dual", 
                            addr_width=24,
                            data_width=8,
                            cmd_width=8,
                            cmd_dict=self.cmd_dict,
                            dummy_cycles_dict=self.dummy_cycles_dict)
        self.flash = (SPIFlashFastReadTestCase.
                      SuperNaiveFlash(dut=self.dut,
                                      data=0xAA,
                                      data_size=8,
                                      dummy_cycles=8))
        self.simple_test()

    def test_n25q128a_quad(self):
        self.cmd_dict = {"FAST_READ": 0xeb}
        self.dummy_cycles_dict = {"FAST_READ": 10}
        self.dut = SPIFlash(protocol="quad", 
                            addr_width=24,
                            data_width=8,
                            cmd_width=8,
                            cmd_dict=self.cmd_dict,
                            dummy_cycles_dict=self.dummy_cycles_dict)
        self.flash = (SPIFlashFastReadTestCase.
                      SuperNaiveFlash(dut=self.dut,
                                      data=0xAA,
                                      data_size=8,
                                      dummy_cycles=10))
        self.simple_test()

    def simple_test(self):
        m = Module()
        m.submodules.master = self.dut
        m.submodules.slave  = self.flash
        def process():
            yield self.dut.ack.eq(1)
            yield self.dut.cmd.eq(self.cmd_dict["FAST_READ"])
            while (yield self.dut.rdy):
                yield       # Wait until it enters CMD state
            yield self.dut.ack.eq(0)
            while not (yield self.dut.r_rdy):
                yield       # Wait until it enters RDYWAIT state
            self.assertEqual((yield self.dut.r_data), self.flash.data)
        simulation_test(m, process)
