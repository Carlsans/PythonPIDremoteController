"""Tests for the PyQt5 + pyqtgraph GUI (YogurtGUIQt). Mirrors test_gui.py's
coverage for the Tkinter version.

Uses an isolated settings file, loopback UDP ports, and Qt's offscreen
platform plugin - never touches the real ESP or a real display.

Run from the project root:
    ./venvarch/bin/python tests/test_gui_qt.py
"""

import json
import os
import socket
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

TMPDIR = tempfile.mkdtemp(prefix="yogurtguiqt")
SETTINGSFILE = os.path.join(TMPDIR, "settings.json")
os.environ["YOGURT_SETTINGS_FILE"] = SETTINGSFILE
os.environ["YOGURT_SILENT"] = "1"
os.environ["QT_QPA_PLATFORM"] = "offscreen"  # no real display needed
os.environ["YOGURT_ESP_IP"] = "127.0.0.1"    # never the real device
os.environ["YOGURT_ESP_PORT"] = "55131"
os.environ["YOGURT_LISTEN_PORT"] = "55130"

from PyQt5 import QtWidgets
import src.YogurtGUIQt as yogurtguiqt
from src.YogurtGUIQt import YogurtGUIQt

# Any modal popup would block the test; record it instead.
popups = []
QtWidgets.QMessageBox.critical = staticmethod(lambda parent, title, msg: popups.append((title, msg)))
QtWidgets.QMessageBox.information = staticmethod(lambda parent, title, msg: popups.append((title, msg)))


def storedsettings():
    with open(SETTINGSFILE) as f:
        return json.load(f)


def test_stages(gui):
    gui.stagetempentry.setText("82")
    gui.stageminutesentry.setText("10")
    gui.addstage()
    gui.stagetempentry.setText("40")
    gui.stageminutesentry.setText("480")
    gui.addstage()
    stored = storedsettings()["stages"]
    assert stored[-2:] == [{"temperature": 82.0, "duration_minutes": 10.0},
                           {"temperature": 40.0, "duration_minutes": 480.0}], stored

    gui.stagetable.selectRow(0)
    gui.removestage()
    stored = storedsettings()["stages"]
    assert stored == [{"temperature": 82.0, "duration_minutes": 10.0},
                      {"temperature": 40.0, "duration_minutes": 480.0}], stored

    popups.clear()
    gui.stagetempentry.setText("abc")
    gui.addstage()
    assert popups, "invalid stage input was not rejected"
    assert len(storedsettings()["stages"]) == 2
    print("OK - stages add/remove/save")


def test_stageprograms(gui):
    values = [gui.programbox.itemText(i) for i in range(gui.programbox.count())]
    assert "greek-yogurt" in values

    gui.programbox.setCurrentText("my-program")
    gui.saveprogram()
    assert storedsettings()["stage_programs"]["my-program"] == \
        [{"temperature": 82.0, "duration_minutes": 10.0},
         {"temperature": 40.0, "duration_minutes": 480.0}]

    gui.stagetable.selectRow(0)
    gui.removestage()
    assert len(storedsettings()["stages"]) == 1
    gui.loadprogram()
    assert storedsettings()["stages"] == \
        [{"temperature": 82.0, "duration_minutes": 10.0},
         {"temperature": 40.0, "duration_minutes": 480.0}]

    gui.stagetempentry.setText("50")
    gui.stageminutesentry.setText("5")
    gui.addstage()
    assert len(storedsettings()["stage_programs"]["my-program"]) == 2

    gui.deleteprogram()
    assert "my-program" not in storedsettings()["stage_programs"]

    gui.programbox.setCurrentText("greek-yogurt")
    gui.loadprogram()
    print("OK - stage programs save/load/delete")


def test_profiles(gui):
    gui.profilebox.setCurrentText("water-test")
    values = {"Kp": "55.5", "Ki": "0.111", "Kd": "0"}
    for key, value in values.items():
        gui.pidentries[key].setText(value)
    gui.saveprofile()
    stored = storedsettings()
    assert stored["pid_profiles"]["water-test"] == {"Kp": 55.5, "Ki": 0.111, "Kd": 0.0}
    assert stored["active_profile"] == "water-test"

    gui.profilebox.setCurrentText("medium-pot-5-jars")
    gui.onprofileselected(0)
    assert gui.pidentries["Kp"].text() == "20.0"
    assert storedsettings()["active_profile"] == "medium-pot-5-jars"

    fakeresult = {"applied": "ziegler_nichols_pi",
                  "ziegler_nichols_pi": {"Kp": 77.7, "Ki": 0.222, "Kd": 0.0}}
    gui.saveautotuneresult("autotune-82C", fakeresult)
    stored = storedsettings()
    assert stored["pid_profiles"]["autotune-82C"] == {"Kp": 77.7, "Ki": 0.222, "Kd": 0.0}
    assert stored["active_profile"] == "autotune-82C"
    before = storedsettings()["pid_profiles"]
    gui.saveautotuneresult("should-not-exist", None)
    assert storedsettings()["pid_profiles"] == before

    gui.profilebox.setCurrentText("water-test")
    gui.deleteprofile()
    assert "water-test" not in storedsettings()["pid_profiles"]
    print("OK - PID profiles save/select/delete/autotune-save")


def pump(app, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline:
        app.processEvents()
        time.sleep(0.02)


def test_start_stop(app, gui):
    """Full cycle: Start program -> fake ESP traffic -> Stop & heater off."""
    esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp.bind(("127.0.0.1", 55131))
    esp.settimeout(0.2)
    received = []
    stopflag = []

    def fakeesp():
        endtime = time.time() + 15
        while time.time() < endtime and not stopflag:
            esp.sendto(b"Sensor temp: 35.0", ("127.0.0.1", 55130))
            try:
                while True:
                    received.append(esp.recvfrom(4096)[0].decode())
            except socket.timeout:
                pass

    espthread = threading.Thread(target=fakeesp, daemon=True)
    espthread.start()

    capturedprogress = []
    from PyQt5.QtCore import QTimer
    QTimer.singleShot(1500, lambda: capturedprogress.append(gui.progresslabel.text()))
    QTimer.singleShot(3000, lambda: gui.stop(heateroff=True))
    gui.startprogram()  # blocks (pumps events via ontick) until stopped
    stopflag.append(True)
    espthread.join()
    esp.close()

    assert any("SetTuningsagg" in r for r in received), "tunings were never sent"
    assert any("SetSP(82" in r for r in received), "first stage SP was never sent"
    assert any("SetSP(1)" in r for r in received), "'Stop & heater off' did not send SetSP(1)"
    assert not gui.running and gui.fermenter is None
    assert gui.startbutton.isEnabled(), "start button not re-enabled after stop"

    assert capturedprogress, "progress display was never captured while running"
    progresstext = capturedprogress[0]
    assert "Stage 1/2" in progresstext, progresstext
    assert "heating" in progresstext.lower(), progresstext
    assert "Total elapsed" in progresstext, progresstext
    assert gui.progresslabel.text() == "Idle - nothing running", \
        "progress display did not reset after stop"
    print("OK - start program / stop & heater off cycle, progress display")


def test_autotune_in_place(app, gui):
    """Full cycle: hold 38 C with too-aggressive tunings -> click 'Autotune
    here (resume after)' -> auto-stop, autotune at 38 C, save profile,
    auto-resume the same stage with the new tunings."""
    from tests.simulator import SimulatedPot

    port_esp, port_listen = 55133, 55132
    os.environ["YOGURT_ESP_PORT"] = str(port_esp)
    os.environ["YOGURT_LISTEN_PORT"] = str(port_listen)

    gui.settings["stages"] = [{"temperature": 38.0, "duration_minutes": 9999.0}]
    gui.refreshstages()
    badtunings = {"Kp": 60.0, "Ki": 0.05, "Kd": 0.0}
    gui.settings["pid_profiles"]["inplace-start"] = dict(badtunings)
    gui.settings["active_profile"] = "inplace-start"
    gui.store.save(gui.settings)
    gui.refreshprofiles()

    esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp.bind(("127.0.0.1", port_esp))
    esp.settimeout(0.1)
    stopflag = []
    lastsp = [1.0]

    def fakeesp():
        pot = SimulatedPot(ambient=30.0, fullpowerrise=20.0, tau=3.0, deadtime=1.0)
        while not stopflag:
            try:
                while True:
                    data, _ = esp.recvfrom(4096)
                    msg = data.decode()
                    if msg.startswith("SetSP("):
                        lastsp[0] = float(msg[6:-1])
            except socket.timeout:
                pass
            sp = lastsp[0]
            if sp > pot.temp + 0.3:
                output = 255.0
            elif sp < pot.temp - 0.3:
                output = 0.0
            else:
                output = 55.0
            pot.step(output)
            esp.sendto(("Sensor temp: " + str(round(pot.temp, 2))).encode(), ("127.0.0.1", port_listen))
            esp.sendto(("Output:" + str(output)).encode(), ("127.0.0.1", port_listen))
            esp.sendto(b"PID Terms: 1.0,0.1,0.0", ("127.0.0.1", port_listen))
            esp.sendto(("SetPoint: " + str(sp)).encode(), ("127.0.0.1", port_listen))
            time.sleep(0.1)

    espthread = threading.Thread(target=fakeesp, daemon=True)
    espthread.start()

    events = {"clicked": False, "resumed": False, "timeout": False}

    from PyQt5.QtCore import QTimer

    def waitthenclick():
        if gui.fermenter is None or gui.fermenter.currenttemp < 37.5:
            QTimer.singleShot(300, waitthenclick)
            return
        events["clicked"] = True
        assert gui.autotuneherebutton.isEnabled(), \
            "'Autotune here' should be enabled once the program is holding"
        gui.autotuneinplace()

    def resumed():
        return (gui.running and gui.fermenter is not None and gui.fermenter.mode == 'pidprogram'
               and "inplace-38.0C" in gui.settings["pid_profiles"])

    # Same reasoning as the Tkinter version's test_start_stop/autotune-in-
    # place test: gui.startprogram() only blocks for as long as its OWN
    # fermenter run is active; later runs (autotune, then resume) are only
    # driven by whatever keeps pumping the Qt event loop afterwards. Without
    # an enclosing app.exec_(), this test has to pump manually - and once a
    # later run is active it is ITSELF blocking inside one of those pumps,
    # so "detect resumed() and stop it" has to happen from inside via
    # ontick(), not from this function's own loop.
    originalontick = gui.ontick

    def testontick():
        originalontick()
        if not events["resumed"] and resumed():
            events["resumed"] = True
            gui.fermenter.stoprequested = True

    gui.ontick = testontick
    try:
        QTimer.singleShot(500, waitthenclick)
        gui.startprogram()

        deadline = time.time() + 90
        while time.time() < deadline and not events["resumed"]:
            app.processEvents()
            time.sleep(0.05)
        events["timeout"] = not events["resumed"]
    finally:
        gui.ontick = originalontick

    if not events["resumed"] and gui.fermenter is not None:
        gui.fermenter.stoprequested = True
    stopdeadline = time.time() + 10
    while gui.running and time.time() < stopdeadline:
        app.processEvents()
        time.sleep(0.05)

    stopflag.append(True)
    espthread.join(timeout=5)
    esp.close()

    assert events["clicked"], "never reached the holding temperature to click 'Autotune here'"
    assert not events["timeout"], "in-place autotune + resume did not complete within 90s"
    assert events["resumed"], "program never auto-resumed after the in-place autotune"
    assert not gui.running and gui.fermenter is None

    stored = storedsettings()
    assert "inplace-38.0C" in stored["pid_profiles"]
    newtunings = stored["pid_profiles"]["inplace-38.0C"]
    assert newtunings != badtunings, "autotune did not actually change the tunings"
    assert stored["active_profile"] == "inplace-38.0C"
    assert stored["stages"] == [{"temperature": 38.0, "duration_minutes": 9999.0}], \
        "resumed stage list should match what was running before autotune"
    assert not gui.autotuneherebutton.isEnabled(), \
        "'Autotune here' should be disabled again once stopped"
    print("OK - Autotune here (resume after) full cycle:", newtunings)


def test_pid_optimizer_toggle(gui):
    """Start program at 38 C -> once holding, click 'Start optimizer' -> it
    settles, runs windows, and nudges tunings -> click 'Stop optimizer' ->
    tunings freeze and gui.fermenter.pidoptimizer is cleared."""
    from tests.simulator import SimulatedPot, SimulatedAdaptiveMCUPID
    from PyQt5.QtCore import QTimer

    os.environ["YOGURT_OPTIMIZER_WINDOW_SECONDS"] = "6"
    os.environ["YOGURT_OPTIMIZER_SETTLE_SECONDS"] = "2"

    port_esp, port_listen = 55137, 55136
    os.environ["YOGURT_ESP_PORT"] = str(port_esp)
    os.environ["YOGURT_LISTEN_PORT"] = str(port_listen)

    gui.settings["stages"] = [{"temperature": 38.0, "duration_minutes": 9999.0}]
    gui.refreshstages()
    starttunings = {"Kp": 14.0, "Ki": 0.02, "Kd": 0.0}
    gui.settings["pid_profiles"]["optimizer-start"] = dict(starttunings)
    gui.settings["active_profile"] = "optimizer-start"
    gui.store.save(gui.settings)
    gui.refreshprofiles()

    esp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp.bind(("127.0.0.1", port_esp))
    esp.settimeout(0.1)
    stopflag = []

    def fakeesp():
        # Real closed-loop simulated MCU (applies whatever cons/agg tunings
        # are sent), same as test_gui.py's version - a fixed-output hack
        # cannot reach a useful steady state and gets stuck below target.
        pot = SimulatedPot(ambient=37.85, fullpowerrise=20.0, tau=3.0, deadtime=1.0)
        pid = SimulatedAdaptiveMCUPID(outmax=255.0)
        pid.settuningsagg(100.0, 0.0, 0.0)
        pid.settuningscons(14.0, 0.02, 0.0)
        pid.setpoint = 1.0
        while not stopflag:
            try:
                while True:
                    data, _ = esp.recvfrom(4096)
                    msg = data.decode()
                    if msg.startswith("SetSP("):
                        pid.setpoint = float(msg[6:-1])
                    elif msg.startswith("SetTuningscons("):
                        kp, ki, kd = (float(v) for v in msg[len("SetTuningscons("):-1].split(","))
                        pid.settuningscons(kp, ki, kd)
                    elif msg.startswith("SetTuningsagg("):
                        kp, ki, kd = (float(v) for v in msg[len("SetTuningsagg("):-1].split(","))
                        pid.settuningsagg(kp, ki, kd)
            except socket.timeout:
                pass
            measured = round(pot.temp * 4) / 4.0
            output = pid.compute(measured)
            pot.step(output)
            esp.sendto(("Sensor temp: " + str(round(pot.temp, 2))).encode(), ("127.0.0.1", port_listen))
            esp.sendto(("Output:" + str(output)).encode(), ("127.0.0.1", port_listen))
            esp.sendto(b"PID Terms: 1.0,0.1,0.0", ("127.0.0.1", port_listen))
            esp.sendto(("SetPoint: " + str(pid.setpoint)).encode(), ("127.0.0.1", port_listen))
            time.sleep(0.1)

    espthread = threading.Thread(target=fakeesp, daemon=True)
    espthread.start()

    events = {"started": False, "windowscompleted": False, "stopped": False}

    def waitthenstart():
        if gui.fermenter is None or gui.fermenter.currenttemp < 37.5:
            QTimer.singleShot(300, waitthenstart)
            return
        assert gui.optimizerbutton.isEnabled(), \
            "optimizer button should be enabled once the program is holding"
        gui.toggleoptimizer()
        events["started"] = gui.fermenter.pidoptimizer is not None
        QTimer.singleShot(300, waitforwindows)

    def waitforwindows(deadline=None):
        if deadline is None:
            deadline = time.time() + 60
        fermenter = gui.fermenter
        if fermenter is None or fermenter.pidoptimizer is None:
            return
        if fermenter.pidoptimizer.getprogress()["windows_done"] >= 2:
            events["windowscompleted"] = True
            events["tunings"] = dict(fermenter.pidoptimizer.currenttunings)
            gui.toggleoptimizer()
            events["stopped"] = fermenter.pidoptimizer is None
            QTimer.singleShot(300, lambda: gui.stop(heateroff=True))
            return
        if time.time() > deadline:
            QTimer.singleShot(300, lambda: gui.stop(heateroff=True))
            return
        QTimer.singleShot(200, lambda: waitforwindows(deadline))

    QTimer.singleShot(500, waitthenstart)
    gui.startprogram()
    stopflag.append(True)
    espthread.join(timeout=5)
    esp.close()

    assert events["started"], "clicking 'Start optimizer' did not attach a PIDOptimizer"
    assert events["windowscompleted"], "optimizer never completed 2 windows within 60s"
    assert events["stopped"], "clicking 'Stop optimizer' did not detach the PIDOptimizer"
    assert events["tunings"] != starttunings, "tunings never changed across 2 windows"
    assert not gui.optimizerbutton.isEnabled(), \
        "optimizer button should be disabled again once the program stopped"
    print("OK - PID optimizer start/stop toggle, tunings changed:", events["tunings"])


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    gui = YogurtGUIQt()
    gui.show()
    app.processEvents()
    try:
        test_stages(gui)
        test_stageprograms(gui)
        test_profiles(gui)
        test_start_stop(app, gui)
        test_autotune_in_place(app, gui)
        test_pid_optimizer_toggle(gui)
    finally:
        gui.close()
    print("\nOK - all Qt GUI tests passed.")


if __name__ == '__main__':
    main()
