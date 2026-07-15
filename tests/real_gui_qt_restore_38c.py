"""Restores the real ferment to its normal 38 C hold using the currently
saved stages/profile in yogurt_settings.json, via the new PyQt GUI. No
autotuning, no experiments - just the plain staged program, same as the
Tkinter GUI would run.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from src.YogurtGUIQt import YogurtGUIQt

app = QtWidgets.QApplication(sys.argv)
gui = YogurtGUIQt()
gui.show()
QTimer.singleShot(500, gui.startprogram)
app.exec_()
