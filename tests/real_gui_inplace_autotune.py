"""Launches the REAL GUI (real interactive graph, not headless) against the
real ESP, using whatever stages/profile are currently saved in
yogurt_settings.json, auto-clicks Start, and once the program is actually
holding its target, auto-clicks "Autotune here (resume after)".

After that it behaves exactly like a normal GUI session - mainloop() takes
over, the window stays open and interactive (Stop, Refresh graph, etc. all
work normally), Carl can watch the graph the whole time. This script's only
job is to perform the two clicks that would otherwise require driving the
GUI by hand.

Only run this with nothing else already bound to the ESP's port (the
single-instance lock will refuse to start otherwise, but check first: the
whole point is to avoid ever having two processes control the real ESP at
once).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# No MPLBACKEND override: use the real interactive backend so the graph is visible.

from src.YogurtGUI import YogurtGUI

gui = YogurtGUI()


def autorun():
    if not gui.settings["stages"]:
        print("AUTORUN: no stages configured, nothing to start.")
        return
    print("AUTORUN: starting program with stages", gui.settings["stages"],
          "profile", gui.settings["active_profile"])
    gui.root.after(500, waitthenautotune)
    gui.startprogram()  # blocks (pumps its own events) until stopped


def waitthenautotune():
    fermenter = gui.fermenter
    if fermenter is None or not hasattr(fermenter, 'pidprogram'):
        gui.root.after(500, waitthenautotune)
        return
    if not fermenter.pidprogram.temperaturehasbeenreached:
        gui.root.after(500, waitthenautotune)
        return
    print("AUTORUN: program is holding", fermenter.currentSP,
          "C (current", fermenter.currenttemp, "C) - starting in-place autotune now.")
    gui.autotuneinplace()


gui.root.after(500, autorun)
gui.root.mainloop()
