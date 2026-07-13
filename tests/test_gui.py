"""GUI tests: drives the real tkinter widgets programmatically.

Uses an isolated settings file and loopback UDP ports - never touches the
real ESP. A window appears briefly on screen while the test runs.

Run from the project root:
    ./venvarch/bin/python tests/test_gui.py
"""

import json
import os
import socket
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

TMPDIR = tempfile.mkdtemp(prefix="yogurtgui")
SETTINGSFILE = os.path.join(TMPDIR, "settings.json")
os.environ["YOGURT_SETTINGS_FILE"] = SETTINGSFILE
os.environ["YOGURT_SILENT"] = "1"
os.environ["MPLBACKEND"] = "Agg"          # no graph window during the test
os.environ["YOGURT_ESP_IP"] = "127.0.0.1"  # never the real device
os.environ["YOGURT_ESP_PORT"] = "55031"
os.environ["YOGURT_LISTEN_PORT"] = "55030"

import src.YogurtGUI as yogurtgui
from src.YogurtGUI import YogurtGUI

# Any validation popup would block the test; record it instead.
popups = []
yogurtgui.messagebox.showerror = lambda title, msg: popups.append((title, msg))
yogurtgui.messagebox.showinfo = lambda title, msg: popups.append((title, msg))


def storedsettings():
    with open(SETTINGSFILE) as f:
        return json.load(f)


def test_stages(gui):
    gui.stagetempentry.insert(0, "82")
    gui.stageminutesentry.insert(0, "10")
    gui.addstage()
    gui.stagetempentry.delete(0, "end")
    gui.stageminutesentry.delete(0, "end")
    gui.stagetempentry.insert(0, "40")
    gui.stageminutesentry.insert(0, "480")
    gui.addstage()
    stored = storedsettings()["stages"]
    assert stored[-2:] == [{"temperature": 82.0, "duration_minutes": 10.0},
                           {"temperature": 40.0, "duration_minutes": 480.0}], stored

    # Remove the default first stage; the two new ones must remain.
    firstitem = gui.stagetree.get_children()[0]
    gui.stagetree.selection_set(firstitem)
    gui.removestage()
    stored = storedsettings()["stages"]
    assert stored == [{"temperature": 82.0, "duration_minutes": 10.0},
                      {"temperature": 40.0, "duration_minutes": 480.0}], stored

    # Invalid input -> popup, nothing saved.
    popups.clear()
    gui.stagetempentry.delete(0, "end")
    gui.stagetempentry.insert(0, "abc")
    gui.addstage()
    assert popups, "invalid stage input was not rejected"
    assert len(storedsettings()["stages"]) == 2
    print("OK - stages add/remove/save")


def test_stageprograms(gui):
    # test_stages left stages = [82/10, 40/480]; the default greek-yogurt
    # program must also be offered.
    assert "greek-yogurt" in gui.programbox["values"]

    gui.programbox.set("my-program")
    gui.saveprogram()
    assert storedsettings()["stage_programs"]["my-program"] == \
        [{"temperature": 82.0, "duration_minutes": 10.0},
         {"temperature": 40.0, "duration_minutes": 480.0}]

    # Mangle the working stages, then restore from the saved program.
    gui.stagetree.selection_set(gui.stagetree.get_children()[0])
    gui.removestage()
    assert len(storedsettings()["stages"]) == 1
    gui.loadprogram()
    assert storedsettings()["stages"] == \
        [{"temperature": 82.0, "duration_minutes": 10.0},
         {"temperature": 40.0, "duration_minutes": 480.0}]

    # Loading must give an independent copy: editing stages afterwards must
    # not silently change the saved program.
    gui.stagetempentry.delete(0, "end")
    gui.stageminutesentry.delete(0, "end")
    gui.stagetempentry.insert(0, "50")
    gui.stageminutesentry.insert(0, "5")
    gui.addstage()
    assert len(storedsettings()["stage_programs"]["my-program"]) == 2

    gui.deleteprogram()
    assert "my-program" not in storedsettings()["stage_programs"]

    # Restore the two-stage working set for the following tests.
    gui.programbox.set("greek-yogurt")
    gui.loadprogram()
    print("OK - stage programs save/load/delete")


def test_profiles(gui):
    gui.profilebox.set("water-test")
    for key, value in (("Kp", "55.5"), ("Ki", "0.111"), ("Kd", "0")):
        gui.pidentries[key].delete(0, "end")
        gui.pidentries[key].insert(0, value)
    gui.saveprofile()
    stored = storedsettings()
    assert stored["pid_profiles"]["water-test"] == {"Kp": 55.5, "Ki": 0.111, "Kd": 0.0}
    assert stored["active_profile"] == "water-test"

    # Switching profiles loads its values into the entries.
    gui.profilebox.set("medium-pot-5-jars")
    gui.onprofileselected()
    assert gui.pidentries["Kp"].get() == "20.0"
    assert storedsettings()["active_profile"] == "medium-pot-5-jars"

    # Autotune completion saves a profile under the label.
    fakeresult = {"applied": "ziegler_nichols_pi",
                  "ziegler_nichols_pi": {"Kp": 77.7, "Ki": 0.222, "Kd": 0.0}}
    gui.saveautotuneresult("autotune-82C", fakeresult)
    stored = storedsettings()
    assert stored["pid_profiles"]["autotune-82C"] == {"Kp": 77.7, "Ki": 0.222, "Kd": 0.0}
    assert stored["active_profile"] == "autotune-82C"
    # Aborted autotune saves nothing.
    before = storedsettings()["pid_profiles"]
    gui.saveautotuneresult("should-not-exist", None)
    assert storedsettings()["pid_profiles"] == before

    gui.profilebox.set("water-test")
    gui.deleteprofile()
    assert "water-test" not in storedsettings()["pid_profiles"]
    print("OK - PID profiles save/select/delete/autotune-save")


def test_start_stop(gui):
    """Full cycle: Start program -> fake ESP traffic -> Stop & heater off."""
    esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp.bind(("127.0.0.1", 55031))
    esp.settimeout(0.2)
    received = []
    stopflag = []

    def fakeesp():
        endtime = time.time() + 15
        while time.time() < endtime and not stopflag:
            esp.sendto(b"Sensor temp: 35.0", ("127.0.0.1", 55030))
            try:
                while True:
                    received.append(esp.recvfrom(4096)[0].decode())
            except socket.timeout:
                pass

    espthread = threading.Thread(target=fakeesp, daemon=True)
    espthread.start()
    # Stop the fermenter (with heater off) after 3 s of running; scheduled
    # before start because startprogram() blocks until the loop ends.
    gui.root.after(3000, lambda: gui.stop(heateroff=True))
    gui.startprogram()
    stopflag.append(True)
    espthread.join()
    esp.close()

    assert any("SetTuningsagg" in r for r in received), "tunings were never sent"
    assert any("SetSP(82" in r for r in received), "first stage SP was never sent"
    assert any("SetSP(1)" in r for r in received), "'Stop & heater off' did not send SetSP(1)"
    assert not gui.running and gui.fermenter is None
    assert gui.startbutton.instate(["!disabled"]), "start button not re-enabled after stop"
    print("OK - start program / stop & heater off cycle")


def main():
    gui = YogurtGUI()
    gui.root.update()
    try:
        test_stages(gui)
        test_stageprograms(gui)
        test_profiles(gui)
        test_start_stop(gui)
    finally:
        gui.root.destroy()
    print("\nOK - all GUI tests passed.")


if __name__ == '__main__':
    main()
