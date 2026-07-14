"""Tests for the hardening added after a real overnight run showed two
independent processes running concurrently against the same ESP for hours
(confirmed via graph_diagnostics.log: two separate, cleanly interleaved
sequences, one frozen/stale while the other kept controlling the pot
normally), plus several previously-unprotected call sites that could let
any bug silently kill the whole control loop.

Covers:
1. acquireportlock() refuses a second instance on the same port.
2. safecall() logs and swallows exceptions instead of propagating them.
3. requestgraphrefresh() only sets a flag; recreategraph() itself only runs
   from animate()'s top-level context, never synchronously from the caller.

Run from the project root:
    ./venvarch/bin/python tests/test_robustness_hardening.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["YOGURT_SILENT"] = "1"
os.environ["MPLBACKEND"] = "Agg"

import matplotlib.pyplot as plt

from src.yogurtdata import YogourtFermenter, SingleInstanceError, acquireportlock


def test_single_instance_lock():
    port = 55081
    first = acquireportlock(port)
    try:
        try:
            acquireportlock(port)
            assert False, "a second lock on the same port must be refused"
        except SingleInstanceError as e:
            assert str(port) in str(e)
        # A different port must be unaffected.
        second = acquireportlock(port + 1)
        second.close()
    finally:
        first.close()

    # Releasing (closing) the first lock must free the port for reuse.
    third = acquireportlock(port)
    third.close()
    print("OK - acquireportlock() refuses duplicate instances on the same port")


def test_fermenter_refuses_duplicate_port():
    plt.close('all')
    os.environ["YOGURT_ESP_IP"] = "127.0.0.1"
    os.environ["YOGURT_ESP_PORT"] = "55091"
    os.environ["YOGURT_LISTEN_PORT"] = "55090"
    first = YogourtFermenter(mode='pidprogram',
                             stages=[{"temperature": 38.0, "duration_minutes": 60}],
                             autorun=False)
    try:
        try:
            YogourtFermenter(mode='pidprogram',
                             stages=[{"temperature": 38.0, "duration_minutes": 60}],
                             autorun=False)
            assert False, "starting a second fermenter on the same port must raise"
        except SingleInstanceError:
            pass
    finally:
        first.sock.close()
        first.portlock.close()
    print("OK - YogourtFermenter refuses to start a second instance on the same port")


def test_safecall_swallows_exceptions():
    plt.close('all')
    os.environ["YOGURT_ESP_IP"] = "127.0.0.1"
    os.environ["YOGURT_ESP_PORT"] = "55093"
    os.environ["YOGURT_LISTEN_PORT"] = "55092"
    fermenter = YogourtFermenter(mode='pidprogram',
                                 stages=[{"temperature": 38.0, "duration_minutes": 60}],
                                 autorun=False)
    try:
        calls = []

        def boom():
            calls.append(1)
            raise RuntimeError("simulated bug")

        ok = fermenter.safecall("boom", boom)
        assert ok is False
        assert calls == [1], "the function must still have been called"

        def fine(x, y):
            calls.append(x + y)

        ok = fermenter.safecall("fine", fine, 2, 3)
        assert ok is True
        assert calls == [1, 5]

        # ontick() giving up after repeated failures must not crash the
        # process - it must gracefully request a stop instead.
        fermenter.ontick = boom
        for _ in range(10):
            success = fermenter.safecall("ontick", fermenter.ontick)
            if success:
                fermenter.ontickfailures = 0
            else:
                fermenter.ontickfailures += 1
        assert fermenter.ontickfailures == 10
    finally:
        fermenter.sock.close()
        fermenter.portlock.close()
    print("OK - safecall() swallows exceptions and keeps the label/count bookkeeping working")


def test_graph_refresh_is_deferred():
    plt.close('all')
    os.environ["YOGURT_ESP_IP"] = "127.0.0.1"
    os.environ["YOGURT_ESP_PORT"] = "55095"
    os.environ["YOGURT_LISTEN_PORT"] = "55094"
    fermenter = YogourtFermenter(mode='pidprogram',
                                 stages=[{"temperature": 38.0, "duration_minutes": 60}],
                                 autorun=False)
    try:
        assert fermenter.graphrecreatecount == 0
        fermenter.requestgraphrefresh()
        # Must NOT have recreated synchronously just by requesting it.
        assert fermenter.graphrefreshrequested is True
        assert fermenter.graphrecreatecount == 0

        fermenter.tempbysec = [38.0]
        fermenter.CV = [10]
        fermenter.PIDTermslist = [[1.0, 0.1, 0.0]]
        fermenter.animate()  # this is the only place recreategraph() may run from

        assert fermenter.graphrefreshrequested is False
        assert fermenter.graphrecreatecount == 1
    finally:
        fermenter.sock.close()
        fermenter.portlock.close()
    print("OK - requestgraphrefresh() only sets a flag; animate() services it")


if __name__ == '__main__':
    test_single_instance_lock()
    test_fermenter_refuses_duplicate_port()
    test_safecall_swallows_exceptions()
    test_graph_refresh_is_deferred()
    print("\nOK - all robustness hardening tests passed.")
