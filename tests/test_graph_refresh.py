"""Tests the periodic graph-recreation watchdog (proactive fix for graphs
that silently stop updating on some window managers/compositors after a long
time - see yogurtdata.py's recreategraph()).

Uses a real interactive matplotlib backend (not Agg) against a fake ESP on
loopback, with a short refresh interval so the recreation is observed within
a few real seconds instead of waiting the default 30 minutes.

Run from the project root (needs a display):
    ./venvarch/bin/python tests/test_graph_refresh.py
"""

import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["YOGURT_SILENT"] = "1"
os.environ["YOGURT_ESP_IP"] = "127.0.0.1"
os.environ["YOGURT_ESP_PORT"] = "55041"
os.environ["YOGURT_LISTEN_PORT"] = "55040"
os.environ["YOGURT_GRAPH_REFRESH_SECONDS"] = "2"

from src.yogurtdata import YogourtFermenter


def main():
    esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp.bind(("127.0.0.1", 55041))
    stop = threading.Event()

    def fakeesp():
        t = 0.0
        while not stop.is_set():
            esp.sendto(("Sensor temp: " + str(30 + (t % 10))).encode(), ("127.0.0.1", 55040))
            esp.sendto(b"Output:80.0", ("127.0.0.1", 55040))
            esp.sendto(b"PID Terms: 1.0,0.1,0.0", ("127.0.0.1", 55040))
            esp.sendto(b"SetPoint:38.0", ("127.0.0.1", 55040))
            t += 0.25
            time.sleep(0.25)

    thread = threading.Thread(target=fakeesp, daemon=True)
    thread.start()

    fermenter = YogourtFermenter(mode='pidprogram',
                                 stages=[{"temperature": 38.0, "duration_minutes": 60}],
                                 ontick=None, autorun=False)

    # Run the loop for ~9 real seconds (>= 4x the 2s refresh interval) on a
    # timer, then request stop.
    def stopper():
        time.sleep(9)
        fermenter.stoprequested = True

    threading.Thread(target=stopper, daemon=True).start()
    fermenter.listeningloop()
    stop.set()
    esp.close()

    print("Graph recreate count:", fermenter.graphrecreatecount)
    assert fermenter.graphrecreatecount >= 3, \
        "expected several recreations in 9s with a 2s refresh interval, got " + str(fermenter.graphrecreatecount)
    print("OK - graph watchdog recreated the window " + str(fermenter.graphrecreatecount) + " times, as expected.")


if __name__ == '__main__':
    main()
