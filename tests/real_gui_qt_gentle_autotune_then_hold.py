"""Same as real_gui_gentle_autotune_then_hold.py but using the new PyQt5 +
pyqtgraph GUI (YogurtGUIQt) instead of the Tkinter one, so the graph is
faster-drawing and single-toolkit end to end.

Launches the REAL GUI (real interactive window, not headless/offscreen)
against the real ESP: runs the relay autotune at a gentler target than the
actual fermentation setpoint (e.g. 32 C instead of 38 C) with a tightened
safety margin, so the "heater on" excursions during tuning stay lower -
meant for autotuning near a living culture rather than plain water. Once
done, saves the profile and resumes holding the REAL fermentation stage(s)
from yogurt_settings.json with the new tunings.

After that it behaves like a normal GUI session - app.exec_() takes over,
the window stays open and interactive.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# No QT_QPA_PLATFORM override: use the real display so the graph is visible.

from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from src.YogurtGUIQt import YogurtGUIQt

AUTOTUNE_TARGET = float(os.environ.get('GENTLE_AUTOTUNE_TARGET', 32.0))
AUTOTUNE_MARGIN = float(os.environ.get('GENTLE_AUTOTUNE_MARGIN', 7.0))
PROFILE_LABEL = os.environ.get('GENTLE_AUTOTUNE_LABEL', 'turc-tuned-32c')

app = QtWidgets.QApplication(sys.argv)
gui = YogurtGUIQt()
resumestages = list(gui.settings["stages"])
if not resumestages:
    print("No stages configured in yogurt_settings.json - nothing to resume into.")
    sys.exit(1)

print("Will autotune at", AUTOTUNE_TARGET, "C (safety ceiling",
      min(95.0, AUTOTUNE_TARGET + AUTOTUNE_MARGIN), "C), then resume:", resumestages)


def onautotunedone(result):
    gui.saveautotuneresult(PROFILE_LABEL, result)
    if result is None:
        print("GENTLE AUTOTUNE ABORTED - not resuming automatically. Check tunings before restarting.")
        return
    print("GENTLE AUTOTUNE DONE - resuming the real program with profile '" + PROFILE_LABEL + "'.")
    gui.stop(heateroff=False)
    QTimer.singleShot(300, waitthenresume)


def waitthenresume():
    if gui.running:
        QTimer.singleShot(200, waitthenresume)
        return
    tunings = tuple(gui.settings["pid_profiles"][PROFILE_LABEL][k] for k in ("Kp", "Ki", "Kd"))
    gui.runfermenter("Program resumed with gentle-autotune tunings (profile '" + PROFILE_LABEL + "')...",
                     mode='pidprogram', stages=resumestages, tunings=tunings)


def startautotune():
    gui.runfermenter("Gentle autotune at " + str(AUTOTUNE_TARGET) + " C (margin "
                     + str(AUTOTUNE_MARGIN) + " C, profile '" + PROFILE_LABEL + "')...",
                     mode='relayautotune', autotunetarget=AUTOTUNE_TARGET,
                     autotunesafetymargin=AUTOTUNE_MARGIN,
                     onautotunedone=onautotunedone)


gui.show()
QTimer.singleShot(500, startautotune)
app.exec_()
