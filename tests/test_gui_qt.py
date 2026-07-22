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


def pump_until(app, predicate, timeout_s=10.0):
    """Run the REAL Qt event loop until predicate() is true or timeout_s
    elapses; returns predicate()'s final value.

    The GUI now runs under app.exec_() with the fermenter stepped by a
    QTimer (the production fix for the multi-day window freeze), so tests
    drive that event loop rather than calling a blocking startprogram() and
    hand-pumping processEvents() - which is exactly the fragile pattern the
    fix removed."""
    from PyQt5.QtCore import QTimer
    deadline = time.time() + timeout_s
    poll = QTimer()
    poll.setInterval(20)

    def check():
        if predicate() or time.time() > deadline:
            poll.stop()
            app.quit()

    poll.timeout.connect(check)
    poll.start()
    app.exec_()
    poll.stop()
    return bool(predicate())


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
    gui.startprogram()  # returns immediately; the QTimer pump drives it now
    pump_until(app, lambda: not gui.running, timeout_s=12)
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

    # fast_approach so the pot reaches 38 quickly and, crucially, the live
    # setpoint is a settled 38.0 the whole time (no slow ramp holding SP
    # below target) - "Autotune here" names the saved profile after the
    # current SP, so this keeps it deterministically "inplace-38.0C".
    gui.settings["stages"] = [{"temperature": 38.0, "duration_minutes": 9999.0, "fast_approach": True}]
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

    # The GUI now runs under the real Qt event loop, so the autotune ->
    # resume chain (which itself schedules QTimer.singleShot steps) just
    # progresses while we pump the loop until it has resumed.
    QTimer.singleShot(500, waitthenclick)
    gui.startprogram()
    events["resumed"] = pump_until(app, resumed, timeout_s=90)
    events["timeout"] = not events["resumed"]

    if gui.fermenter is not None:
        gui.fermenter.stoprequested = True
    pump_until(app, lambda: not gui.running, timeout_s=10)

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
    assert stored["stages"] == [{"temperature": 38.0, "duration_minutes": 9999.0, "fast_approach": True}], \
        "resumed stage list should match what was running before autotune"
    assert not gui.autotuneherebutton.isEnabled(), \
        "'Autotune here' should be disabled again once stopped"
    print("OK - Autotune here (resume after) full cycle:", newtunings)


def test_event_loop_stays_responsive_while_running(app, gui):
    """Direct regression test for the multi-day window freeze. Its root
    cause (proven by a py-spy dump of a frozen instance) was that the GUI
    blocked Qt's event loop inside listeningloop() and hand-cranked repaints
    via processEvents(); over long runs the window silently stopped
    updating. The fix keeps app.exec_() running and steps the fermenter from
    a QTimer. Here we start a run (with no ESP answering, the strictest
    case - nothing external drives the loop), install an independent 100 ms
    heartbeat timer, and confirm it keeps firing near its real rate: proof
    the loop is dispatching normally rather than being starved."""
    from PyQt5.QtCore import QTimer
    os.environ["YOGURT_ESP_PORT"] = "55141"
    os.environ["YOGURT_LISTEN_PORT"] = "55140"
    gui.settings["stages"] = [{"temperature": 38.0, "duration_minutes": 9999.0}]
    gui.refreshstages()

    beats = []
    last = [time.time()]
    hb = QTimer()
    hb.setInterval(100)

    def beat():
        now = time.time()
        beats.append(now - last[0])
        last[0] = now

    hb.timeout.connect(beat)
    gui.startprogram()
    assert gui.running, "startprogram() should have started a run"
    assert gui.fermenter is not None
    hb.start()
    last[0] = time.time()
    beats.clear()
    pump_until(app, lambda: False, timeout_s=3.0)  # just spin the real loop 3 s
    hb.stop()

    gui.stop(heateroff=False)
    pump_until(app, lambda: not gui.running, timeout_s=10)

    assert len(beats) >= 20, \
        "event loop starved: only " + str(len(beats)) + " heartbeats in 3s (expected ~30)"
    worst = max(beats)
    assert worst < 0.5, \
        "event loop stalled for " + str(round(worst * 1000)) + " ms - the blocking-loop freeze is back"
    print("OK - event loop stays responsive while running:", len(beats),
          "beats, worst gap", str(round(worst * 1000)) + " ms")


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
        test_event_loop_stays_responsive_while_running(app, gui)
        test_plot_redraw_stays_fast_on_long_runs(app, gui)
    finally:
        gui.close()
    print("\nOK - all Qt GUI tests passed.")


def test_plot_redraw_stays_fast_on_long_runs(app, gui):
    """Regression test for the long-run GUI freeze: updateplots() used to
    hand every curve the FULL history as Python lists with antialiased
    wide pens - measured ~1 s per redraw at 5 h of data and ~4 s at the
    20 h cap, at which point the once-per-second redraw starved the event
    loop completely and the window froze. Feed a full 20 h of fake history
    and verify (a) downsampling actually bounds what reaches the painted
    curves, and (b) a full update+render cycle stays far under the 1 s
    redraw interval."""
    import math
    class FakeFerm:
        pass
    f = FakeFerm()
    n = 20 * 3600  # cleanPlotlists()' cap
    f.tempbysec = [38 + 1.5 * math.sin(i / 40.0) for i in range(n)]
    f.CV = [30 + 25 * math.sin(i / 18.0) for i in range(n)]
    f.PIDTermslist = [(3 * math.sin(i / 25.0), 2.0, -0.7) for i in range(n)]
    f.errorlist = [(i, abs(math.sin(i / 40.0)) * 4) for i in range(n)]
    f.currenttemp = f.tempbysec[-1]
    f.currentSP = 38.0
    f.currentPIDSettings = [19.76, 0.03, 0.0]
    gui.fermenter = f
    try:
        gui.lastplotupdate = 0.0
        gui.updateplots()
        app.processEvents()
        gui.grab()  # warm-up render (offscreen widgets skip plain repaint())

        for name, line in (("templine", gui.templine), ("cvline", gui.cvline),
                           ("pline", gui.pline), ("errorline", gui.errorline)):
            ds = line._getDisplayDataset()
            assert ds is not None and len(ds.x) < n / 3, \
                name + " is painting " + str(0 if ds is None else len(ds.x)) \
                + " of " + str(n) + " points - downsampling is not active"

        best = None
        for _ in range(3):
            gui.lastplotupdate = 0.0
            t0 = time.time()
            gui.updateplots()
            app.processEvents()
            gui.grab()
            elapsed = time.time() - t0
            best = elapsed if best is None else min(best, elapsed)
        # Generous bound: measured ~0.15 s after the fix vs ~4.2 s before,
        # so 1 s catches any real regression without being timing-flaky.
        assert best < 1.0, \
            "full 20h update+render took " + str(round(best, 2)) + " s - the freeze regression is back"
        print("OK - 20h-history redraw stays fast:", str(round(best * 1000)) + " ms")
    finally:
        gui.fermenter = None


if __name__ == '__main__':
    main()
