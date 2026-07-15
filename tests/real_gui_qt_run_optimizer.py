"""Launches the REAL GUI (real interactive graph, not headless) against the
real ESP: starts the staged program using whatever is currently saved in
yogurt_settings.json (stages + active profile), and once it's genuinely
holding at target, auto-clicks "Start optimizer" with maxswing=3.0 (the
user's explicit safety requirement).

After that it behaves like a normal GUI session - app.exec_() takes over,
the window stays open and interactive; the optimizer can be stopped at any
time via the "Stop optimizer" button, same as if a human had clicked it.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from src.YogurtGUIQt import YogurtGUIQt

MAXSWING = float(os.environ.get('OPTIMIZER_MAXSWING', 3.0))

app = QtWidgets.QApplication(sys.argv)
gui = YogurtGUIQt()
gui.show()

print("Will start the program with stages", gui.settings["stages"],
      "profile", gui.settings["active_profile"], "then auto-start the "
      "PID optimizer (maxswing=" + str(MAXSWING) + ") once it's holding.")


def waitthenstartoptimizer():
    fermenter = gui.fermenter
    if fermenter is None or not hasattr(fermenter, 'pidprogram'):
        QTimer.singleShot(500, waitthenstartoptimizer)
        return
    if not fermenter.pidprogram.temperaturehasbeenreached:
        QTimer.singleShot(500, waitthenstartoptimizer)
        return
    print("PROGRAM is holding", fermenter.currentSP, "C (current",
          fermenter.currenttemp, "C) - starting PID optimizer now.")
    gui.optimizermaxswingentry.setText(str(MAXSWING))
    gui.toggleoptimizer()


def autorun():
    QTimer.singleShot(500, waitthenstartoptimizer)
    gui.startprogram()  # blocks (pumps its own events) until stopped


QTimer.singleShot(500, autorun)
app.exec_()
