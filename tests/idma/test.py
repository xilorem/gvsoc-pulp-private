# Copyright (C) 2026 Fondazione Chips-IT
#
# SPDX-License-Identifier: Apache-2.0

"""Regression test for reordered AXI completions in the legacy iDMA model."""

import gvsoc.runner
import gvsoc.systree
import vp.clock_domain

from pulp.idma.reg32_3d_dma import Reg32_3dDma


class IDmaInterleavingTester(gvsoc.systree.Component):
    def __init__(self, parent, name):
        super().__init__(parent, name)
        self.add_sources(['test.cpp'])

    def o_CTRL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('ctrl', itf, signature='io')

    def i_TCDM(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'tcdm', signature='io')

    def i_AXI(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'axi', signature='io')

    def i_EVENT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'event', signature='wire<bool>')


class Testbench(gvsoc.systree.Component):
    def __init__(self, parent, name=None):
        super().__init__(parent, name)

        clock = vp.clock_domain.Clock_domain(self, 'clock', frequency=100_000_000)

        # Keep only two burst slots. The broken implementation leaks both after
        # reordered completions, leaving the third descriptor permanently pending.
        idma = Reg32_3dDma(
            self,
            'idma',
            nb_cores=1,
            nb_pe_ports=0,
            nb_streams=1,
            multireg_count=2,
            global_queue_depth=4,
            transfer_queue_size=4,
            burst_queue_size=2,
            burst_size=0,
            loc_base=0x10000000,
            loc_size=0x10000,
            tcdm_width=32,
        )
        tester = IDmaInterleavingTester(self, 'tester')

        clock.o_CLOCK(idma.i_CLOCK())
        clock.o_CLOCK(tester.i_CLOCK())

        tester.o_CTRL(idma.i_CTRL(0))
        idma.o_TCDM(tester.i_TCDM())
        idma.o_AXI(tester.i_AXI())
        idma.o_EVENT(0, tester.i_EVENT())


class Target(gvsoc.runner.Target):
    gapy_description = 'legacy iDMA reordered AXI completion regression test'
    model = Testbench
    name = 'test'
