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
    # Capture the live progress display partway through the run, then stop
    # the fermenter (with heater off); scheduled before start because
    # startprogram() blocks until the loop ends.
    capturedprogress = []
    buttonstateswhilerunning = []
    recreatecountbeforeclick = []
    recreatecountsoonafterclick = []
    requestflagimmediatelyafterclick = []

    def clickrefresh():
        buttonstateswhilerunning.append(gui.refreshgraphbutton.instate(["!disabled"]))
        recreatecountbeforeclick.append(gui.fermenter.graphrecreatecount)
        gui.refreshgraph()
        # The click must NOT recreate the graph synchronously (that nested,
        # nested-in-a-Tk-callback path is what showed a real hang) - it only
        # sets a flag serviced later from animate()'s safe top-level context.
        requestflagimmediatelyafterclick.append(gui.fermenter.graphrefreshrequested)

    def checkrecreated():
        recreatecountsoonafterclick.append(gui.fermenter.graphrecreatecount)

    gui.root.after(1500, lambda: capturedprogress.append(gui.progressvar.get()))
    gui.root.after(1700, clickrefresh)
    gui.root.after(2800, checkrecreated)  # a couple of animate() ticks later
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

    assert buttonstateswhilerunning == [True], "refresh graph button was not enabled while running"
    assert recreatecountbeforeclick == [0], "unexpected automatic graph recreation (should default to off)"
    assert requestflagimmediatelyafterclick == [True], \
        "clicking 'Refresh graph' must only set a request flag, not recreate synchronously"
    assert recreatecountsoonafterclick == [1], \
        "the requested graph refresh was never serviced by animate()"
    assert gui.refreshgraphbutton.instate(["disabled"]), "refresh graph button not disabled after stop"

    assert capturedprogress, "progress display was never captured while running"
    progresstext = capturedprogress[0]
    assert "Stage 1/2" in progresstext, progresstext
    assert "heating" in progresstext.lower(), progresstext  # fake ESP holds temp at 35, never reaches 82
    assert "Total elapsed" in progresstext, progresstext
    assert gui.progressvar.get() == "Idle - nothing running", \
        "progress display did not reset after stop"
    print("OK - start program / stop & heater off cycle, progress display")


def test_autotune_in_place(gui):
    """Full cycle: Start a 38 C hold with deliberately-too-aggressive tunings
    -> click 'Autotune here (resume after)' once it's holding -> the program
    auto-stops, autotunes at 38 C, saves a new profile, and auto-resumes the
    SAME stage with the new tunings - all over real (simulated, fast) UDP.
    """
    from tests.simulator import SimulatedPot

    port_esp, port_listen = 55033, 55032
    os.environ["YOGURT_ESP_PORT"] = str(port_esp)
    os.environ["YOGURT_LISTEN_PORT"] = str(port_listen)

    gui.settings["stages"] = [{"temperature": 38.0, "duration_minutes": 9999.0}]
    gui.refreshstages()
    badtunings = {"Kp": 60.0, "Ki": 0.05, "Kd": 0.0}  # deliberately too aggressive for this "pot"
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
        # Fast thermal model (compressed time constant) so a full 6-cycle
        # relay autotune completes in seconds of real/wall-clock time
        # instead of the ~15 min/cycle a real pot takes.
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
                output = 55.0  # modest holding power near equilibrium
            pot.step(output)
            esp.sendto(("Sensor temp: " + str(round(pot.temp, 2))).encode(), ("127.0.0.1", port_listen))
            esp.sendto(("Output:" + str(output)).encode(), ("127.0.0.1", port_listen))
            esp.sendto(b"PID Terms: 1.0,0.1,0.0", ("127.0.0.1", port_listen))
            esp.sendto(("SetPoint: " + str(sp)).encode(), ("127.0.0.1", port_listen))
            time.sleep(0.1)

    espthread = threading.Thread(target=fakeesp, daemon=True)
    espthread.start()

    events = {"clicked": False, "resumed": False, "timeout": False}

    def waitthenclick():
        if gui.fermenter is None or gui.fermenter.currenttemp < 37.5:
            gui.root.after(300, waitthenclick)
            return
        events["clicked"] = True
        assert gui.autotuneherebutton.instate(["!disabled"]), \
            "'Autotune here' should be enabled once the program is holding"
        gui.autotuneinplace()

    def resumed():
        return (gui.running and gui.fermenter is not None and gui.fermenter.mode == 'pidprogram'
               and "inplace-38.0C" in gui.settings["pid_profiles"])

    # gui.startprogram() only blocks for as long as ITS OWN fermenter run is
    # active; clicking "Autotune here" stops that run, so this first call
    # returns as soon as the click fires (well before the autotune has even
    # started). The two follow-up runs it schedules (autotune, then resume)
    # are only driven by whatever keeps pumping the Tk event loop afterwards
    # - in the real app that's root.mainloop(), which runs for the life of
    # the window regardless of any single button handler returning. This
    # test has no mainloop(), so it must keep pumping root.update() itself
    # for exactly the same reason.
    #
    # But once a later run (e.g. the resumed pidprogram) is active, it is
    # ITSELF blocking inside one of those root.update() calls - control does
    # not return to this function's own while loop until that nested call
    # unwinds. So "detect resumed() and stop it" cannot be done from out
    # here; it has to happen from inside, via the SAME ontick() hook the
    # fermenter already calls on every iteration regardless of nesting depth.
    originalontick = gui.ontick

    def testontick():
        originalontick()
        if not events["resumed"] and resumed():
            events["resumed"] = True
            gui.fermenter.stoprequested = True

    gui.ontick = testontick
    try:
        gui.root.after(500, waitthenclick)
        gui.startprogram()

        deadline = time.time() + 90
        while time.time() < deadline and not events["resumed"]:
            gui.root.update()
            time.sleep(0.05)
        events["timeout"] = not events["resumed"]
    finally:
        gui.ontick = originalontick

    if not events["resumed"] and gui.fermenter is not None:
        gui.fermenter.stoprequested = True
    stopdeadline = time.time() + 10
    while gui.running and time.time() < stopdeadline:
        gui.root.update()
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
    assert gui.autotuneherebutton.instate(["disabled"]), \
        "'Autotune here' should be disabled again once stopped"
    print("OK - Autotune here (resume after) full cycle:", newtunings)


def test_pid_optimizer_toggle(gui):
    """Start program at 38 C -> once holding, click 'Start optimizer' -> it
    settles, runs windows, and nudges tunings -> click 'Stop optimizer' ->
    tunings freeze and gui.fermenter.pidoptimizer is cleared."""
    from tests.simulator import SimulatedPot, SimulatedAdaptiveMCUPID

    # Short window/settle so this completes in seconds of real time instead
    # of the real GUI's default 15 min windows / 5 min settle.
    os.environ["YOGURT_OPTIMIZER_WINDOW_SECONDS"] = "6"
    os.environ["YOGURT_OPTIMIZER_SETTLE_SECONDS"] = "2"

    port_esp, port_listen = 55035, 55034
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
        # A REAL closed-loop simulated MCU (not a crude neutral-band hack):
        # it actually applies whatever cons/agg tunings are sent, exactly
        # like SimulatedAdaptiveMCUPID does for tests/test_pidoptimizer.py,
        # so this test can observe the optimizer's tuning changes actually
        # affecting tracking quality, and so it settles into a genuinely
        # stable hold (a fixed-output "neutral band" hack instead caused a
        # persistent oscillation that fought the fermenter's own approach
        # ramp and never let the optimizer settle).
        # Start already close to target (PIDProgram's approach ramp runs on
        # REAL wall-clock time at a fixed 0.5 C/min regardless of how fast
        # this simulated pot itself responds, so a cold start from far away
        # would make this test take many real minutes - starting within 0.25
        # C of target makes updateapproachramp() skip the ramp entirely).
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
            gui.root.after(300, waitthenstart)
            return
        assert gui.optimizerbutton.instate(["!disabled"]), \
            "optimizer button should be enabled once the program is holding"
        gui.toggleoptimizer()
        events["started"] = gui.fermenter.pidoptimizer is not None
        gui.root.after(300, waitforwindows)

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
            gui.root.after(300, lambda: gui.stop(heateroff=True))
            return
        if time.time() > deadline:
            gui.root.after(300, lambda: gui.stop(heateroff=True))
            return
        gui.root.after(200, lambda: waitforwindows(deadline))

    gui.root.after(500, waitthenstart)
    gui.startprogram()
    stopflag.append(True)
    espthread.join(timeout=5)
    esp.close()

    assert events["started"], "clicking 'Start optimizer' did not attach a PIDOptimizer"
    assert events["windowscompleted"], "optimizer never completed 2 windows within 60s"
    assert events["stopped"], "clicking 'Stop optimizer' did not detach the PIDOptimizer"
    assert events["tunings"] != starttunings, "tunings never changed across 2 windows"
    assert gui.optimizerbutton.instate(["disabled"]), \
        "optimizer button should be disabled again once the program stopped"
    print("OK - PID optimizer start/stop toggle, tunings changed:", events["tunings"])


def main():
    gui = YogurtGUI()
    gui.root.update()
    try:
        test_stages(gui)
        test_stageprograms(gui)
        test_profiles(gui)
        test_start_stop(gui)
        test_autotune_in_place(gui)
        test_pid_optimizer_toggle(gui)
    finally:
        gui.root.destroy()
    print("\nOK - all GUI tests passed.")


if __name__ == '__main__':
    main()
