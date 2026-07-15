"""PyQt5 + pyqtgraph control panel for the yogurt fermenter.

Run from the project root:
    python -m src.YogurtGUIQt

Same feature set as YogurtGUI.py (the Tkinter+matplotlib version): stage
editor, named PID profiles, autotune (including "Autotune here, resume
after"), start/stop, live progress display. The difference is the graph:
this GUI runs the fermenter with showgraph=False (it never touches
matplotlib) and instead reads tempbysec/CV/PIDTermslist/errorlist itself,
plotting them with pyqtgraph inside the SAME Qt window.

Why a separate GUI instead of embedding pyqtgraph in the Tkinter one:
pyqtgraph needs a running Qt event loop (QApplication), and running a
second GUI toolkit's event loop alongside Tkinter's in one process is
exactly the kind of cross-toolkit hazard this project spent a long
investigation chasing (see yogurtdata.py's diaglog / graph_diagnostics.log
history). Keeping this GUI 100% Qt avoids that entirely - one process, one
toolkit, one event loop.

Design note: like the Tkinter GUI, the fermenter's listening loop runs on
this same thread and calls back into ontick() every iteration, which pumps
QApplication.processEvents(). This keeps everything single-threaded.
"""

import copy
import sys
import time

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from src.SettingsStore import SettingsStore
from src.yogurtdata import YogourtFermenter, SingleInstanceError


def formatduration(seconds):
    if seconds is None:
        return "-"
    seconds = max(0, int(seconds))
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    return "%d:%02d:%02d" % (hours, minutes, seconds)


class YogurtGUIQt(QtWidgets.QMainWindow):
    def __init__(self, settingsstore=None):
        super().__init__()
        self.setWindowTitle("Yogurt Fermenter (Qt)")
        self.store = settingsstore if settingsstore is not None else SettingsStore()
        self.settings = self.store.load()
        self.fermenter = None
        self.running = False
        self.closing = False
        self._inplacepending = None
        self.lastplotupdate = 0.0
        self.lastprogressupdate = 0.0

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QHBoxLayout(central)
        controls = QtWidgets.QVBoxLayout()
        outer.addLayout(controls, stretch=0)
        outer.addWidget(self.buildplots(), stretch=1)

        controls.addWidget(self.buildstagesection())
        controls.addWidget(self.buildpidsection())
        controls.addWidget(self.buildautotunesection())
        controls.addWidget(self.buildrunsection())
        controls.addWidget(self.buildprogresssection())
        controls.addStretch(1)

        self.refreshstages()
        self.refreshprograms()
        self.refreshprofiles()
        self.resize(1200, 700)

    # ------------------------------------------------------------------
    # Program stages
    # ------------------------------------------------------------------
    def buildstagesection(self):
        box = QtWidgets.QGroupBox("Program stages (heat to temp, then hold for duration)")
        layout = QtWidgets.QVBoxLayout(box)

        self.stagetable = QtWidgets.QTableWidget(0, 2)
        self.stagetable.setHorizontalHeaderLabels(["Temperature (C)", "Hold (minutes)"])
        self.stagetable.horizontalHeader().setStretchLastSection(True)
        self.stagetable.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.stagetable.setMaximumHeight(120)
        layout.addWidget(self.stagetable)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Temp (C):"))
        self.stagetempentry = QtWidgets.QLineEdit()
        self.stagetempentry.setMaximumWidth(60)
        row.addWidget(self.stagetempentry)
        row.addWidget(QtWidgets.QLabel("Hold (min):"))
        self.stageminutesentry = QtWidgets.QLineEdit()
        self.stageminutesentry.setMaximumWidth(70)
        row.addWidget(self.stageminutesentry)
        addbutton = QtWidgets.QPushButton("Add stage")
        addbutton.clicked.connect(self.addstage)
        row.addWidget(addbutton)
        removebutton = QtWidgets.QPushButton("Remove selected")
        removebutton.clicked.connect(self.removestage)
        row.addWidget(removebutton)
        layout.addLayout(row)

        row2 = QtWidgets.QHBoxLayout()
        row2.addWidget(QtWidgets.QLabel("Saved programs:"))
        self.programbox = QtWidgets.QComboBox()
        self.programbox.setEditable(True)
        row2.addWidget(self.programbox)
        savebutton = QtWidgets.QPushButton("Save")
        savebutton.clicked.connect(self.saveprogram)
        row2.addWidget(savebutton)
        loadbutton = QtWidgets.QPushButton("Load")
        loadbutton.clicked.connect(self.loadprogram)
        row2.addWidget(loadbutton)
        deletebutton = QtWidgets.QPushButton("Delete")
        deletebutton.clicked.connect(self.deleteprogram)
        row2.addWidget(deletebutton)
        layout.addLayout(row2)
        return box

    def refreshstages(self):
        self.stagetable.setRowCount(0)
        for stage in self.settings["stages"]:
            r = self.stagetable.rowCount()
            self.stagetable.insertRow(r)
            self.stagetable.setItem(r, 0, QtWidgets.QTableWidgetItem(str(stage["temperature"])))
            self.stagetable.setItem(r, 1, QtWidgets.QTableWidgetItem(str(stage["duration_minutes"])))

    def addstage(self):
        try:
            temperature = float(self.stagetempentry.text())
            minutes = float(self.stageminutesentry.text())
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Invalid stage", "Temperature and hold minutes must be numbers.")
            return
        if not 0 < temperature <= 95:
            QtWidgets.QMessageBox.critical(self, "Invalid stage", "Temperature must be between 0 and 95 C.")
            return
        if minutes < 0:
            QtWidgets.QMessageBox.critical(self, "Invalid stage", "Hold minutes cannot be negative.")
            return
        self.settings["stages"].append({"temperature": temperature, "duration_minutes": minutes})
        self.store.save(self.settings)
        self.refreshstages()

    def removestage(self):
        rows = sorted({idx.row() for idx in self.stagetable.selectedIndexes()}, reverse=True)
        if not rows:
            QtWidgets.QMessageBox.information(self, "Remove stage", "Select a stage to remove first.")
            return
        for r in rows:
            del self.settings["stages"][r]
        self.store.save(self.settings)
        self.refreshstages()

    # ------------------------------------------------------------------
    # Saved stage programs
    # ------------------------------------------------------------------
    def refreshprograms(self):
        self.programbox.clear()
        self.programbox.addItems(sorted(self.settings["stage_programs"].keys()))
        self.programbox.setCurrentText("")

    def saveprogram(self):
        name = self.programbox.currentText().strip()
        if not name:
            QtWidgets.QMessageBox.critical(self, "Save program", "Give the program a name in the 'Saved programs' box.")
            return
        if not self.settings["stages"]:
            QtWidgets.QMessageBox.critical(self, "Save program", "Add at least one stage first.")
            return
        self.settings["stage_programs"][name] = copy.deepcopy(self.settings["stages"])
        self.store.save(self.settings)
        self.refreshprograms()
        self.programbox.setCurrentText(name)
        self.setstatus("Saved stage program '" + name + "'")

    def loadprogram(self):
        name = self.programbox.currentText().strip()
        program = self.settings["stage_programs"].get(name)
        if program is None:
            QtWidgets.QMessageBox.critical(self, "Load program", "No saved program named '" + name + "'.")
            return
        self.settings["stages"] = copy.deepcopy(program)
        self.store.save(self.settings)
        self.refreshstages()
        self.setstatus("Loaded stage program '" + name + "'")

    def deleteprogram(self):
        name = self.programbox.currentText().strip()
        if name not in self.settings["stage_programs"]:
            return
        del self.settings["stage_programs"][name]
        self.store.save(self.settings)
        self.refreshprograms()
        self.setstatus("Deleted stage program '" + name + "'")

    # ------------------------------------------------------------------
    # PID profiles
    # ------------------------------------------------------------------
    def buildpidsection(self):
        box = QtWidgets.QGroupBox("PID tunings (named profiles)")
        layout = QtWidgets.QVBoxLayout(box)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Profile:"))
        self.profilebox = QtWidgets.QComboBox()
        self.profilebox.setEditable(True)
        self.profilebox.currentIndexChanged.connect(self.onprofileselected)
        row.addWidget(self.profilebox)
        layout.addLayout(row)

        row2 = QtWidgets.QHBoxLayout()
        self.pidentries = {}
        for name in ("Kp", "Ki", "Kd"):
            row2.addWidget(QtWidgets.QLabel(name + ":"))
            entry = QtWidgets.QLineEdit()
            entry.setMaximumWidth(90)
            row2.addWidget(entry)
            self.pidentries[name] = entry
        layout.addLayout(row2)

        row3 = QtWidgets.QHBoxLayout()
        savebutton = QtWidgets.QPushButton("Save profile")
        savebutton.clicked.connect(self.saveprofile)
        row3.addWidget(savebutton)
        deletebutton = QtWidgets.QPushButton("Delete profile")
        deletebutton.clicked.connect(self.deleteprofile)
        row3.addWidget(deletebutton)
        layout.addLayout(row3)
        return box

    def refreshprofiles(self):
        names = sorted(self.settings["pid_profiles"].keys())
        self.profilebox.blockSignals(True)
        self.profilebox.clear()
        self.profilebox.addItems(names)
        active = self.settings.get("active_profile")
        if active not in self.settings["pid_profiles"]:
            active = names[0] if names else ""
            self.settings["active_profile"] = active
        self.profilebox.setCurrentText(active)
        self.profilebox.blockSignals(False)
        self.loadprofileentries(active)

    def loadprofileentries(self, name):
        profile = self.settings["pid_profiles"].get(name)
        if profile is None:
            return
        for key, entry in self.pidentries.items():
            entry.setText(str(profile[key]))

    def onprofileselected(self, index):
        name = self.profilebox.currentText()
        if name not in self.settings["pid_profiles"]:
            return
        self.settings["active_profile"] = name
        self.loadprofileentries(name)
        self.store.save(self.settings)

    def currenttunings(self):
        return tuple(float(self.pidentries[k].text()) for k in ("Kp", "Ki", "Kd"))

    def saveprofile(self):
        name = self.profilebox.currentText().strip()
        if not name:
            QtWidgets.QMessageBox.critical(self, "Save profile", "Give the profile a name in the Profile box.")
            return
        try:
            kp, ki, kd = self.currenttunings()
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Save profile", "Kp, Ki and Kd must be numbers.")
            return
        self.settings["pid_profiles"][name] = {"Kp": kp, "Ki": ki, "Kd": kd}
        self.settings["active_profile"] = name
        self.store.save(self.settings)
        self.refreshprofiles()
        self.setstatus("Saved profile '" + name + "'")

    def deleteprofile(self):
        name = self.profilebox.currentText().strip()
        if name not in self.settings["pid_profiles"]:
            return
        if len(self.settings["pid_profiles"]) == 1:
            QtWidgets.QMessageBox.critical(self, "Delete profile", "Cannot delete the last profile.")
            return
        del self.settings["pid_profiles"][name]
        self.store.save(self.settings)
        self.refreshprofiles()
        self.setstatus("Deleted profile '" + name + "'")

    def saveautotuneresult(self, label, result):
        if result is None:
            self.setstatus("Autotune aborted - nothing saved. Check the terminal output.")
            return
        tunings = result[result.get("applied", "ziegler_nichols_pi")]
        self.settings["pid_profiles"][label] = {
            "Kp": tunings["Kp"], "Ki": tunings["Ki"], "Kd": tunings["Kd"]}
        self.settings["active_profile"] = label
        self.store.save(self.settings)
        self.refreshprofiles()
        self.setstatus("Autotune done: saved profile '" + label + "'. Now holding the target temperature.")

    # ------------------------------------------------------------------
    # Autotune
    # ------------------------------------------------------------------
    def buildautotunesection(self):
        box = QtWidgets.QGroupBox("Autotune (relay method - use water, not milk)")
        layout = QtWidgets.QVBoxLayout(box)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Target (C):"))
        self.autotunetargetentry = QtWidgets.QLineEdit("40")
        self.autotunetargetentry.setMaximumWidth(60)
        row.addWidget(self.autotunetargetentry)
        row.addWidget(QtWidgets.QLabel("Margin (C):"))
        self.autotunemarginentry = QtWidgets.QLineEdit("12")
        self.autotunemarginentry.setMaximumWidth(50)
        self.autotunemarginentry.setToolTip(
            "How far above target the relay's 'heater on' setpoint is allowed to swing.\n"
            "Tighten this (e.g. 5-7) when autotuning near a live, temperature-sensitive culture.")
        row.addWidget(self.autotunemarginentry)
        row.addWidget(QtWidgets.QLabel("Save as profile:"))
        self.autotunelabelentry = QtWidgets.QLineEdit()
        row.addWidget(self.autotunelabelentry)
        self.autotunebutton = QtWidgets.QPushButton("Start autotune")
        self.autotunebutton.clicked.connect(self.startautotune)
        row.addWidget(self.autotunebutton)
        layout.addLayout(row)

        self.autotuneherebutton = QtWidgets.QPushButton("Autotune here (resume after)")
        self.autotuneherebutton.setEnabled(False)
        self.autotuneherebutton.clicked.connect(self.autotuneinplace)
        layout.addWidget(self.autotuneherebutton)
        return box

    def autotunemargin(self):
        try:
            return float(self.autotunemarginentry.text())
        except ValueError:
            return 12.0

    def startautotune(self):
        if self.running:
            return
        try:
            target = float(self.autotunetargetentry.text())
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Autotune", "Target temperature must be a number.")
            return
        if not 20 <= target <= 90:
            QtWidgets.QMessageBox.critical(self, "Autotune", "Target must be between 20 and 90 C.")
            return
        label = self.autotunelabelentry.text().strip()
        if not label:
            label = "autotune-" + str(target) + "C"
        self.runfermenter("Autotuning at " + str(target) + " C (profile '" + label + "')...",
                          mode='relayautotune', autotunetarget=target,
                          autotunesafetymargin=self.autotunemargin(),
                          onautotunedone=lambda result: self.saveautotuneresult(label, result))

    def autotuneinplace(self):
        if not self.running or self.fermenter is None or self.fermenter.mode != 'pidprogram':
            QtWidgets.QMessageBox.information(self, "Autotune here",
                "Start the program first - this retunes at whatever it's "
                "currently holding, then resumes it automatically.")
            return
        target = self.fermenter.SP
        stageindex = self.fermenter.pidprogram.currentstage
        resumestages = list(self.settings["stages"][stageindex:]) or list(self.settings["stages"][-1:])
        label = self.autotunelabelentry.text().strip() or ("inplace-" + str(target) + "C")

        self.setstatus("Stopping the program to autotune in place at " + str(target) + " C...")
        self.autotuneherebutton.setEnabled(False)
        self.stop(heateroff=False)
        self._inplacepending = (target, resumestages, label)
        QtCore.QTimer.singleShot(300, self._inplacewaitthenautotune)

    def _inplacewaitthenautotune(self):
        if self.running:
            QtCore.QTimer.singleShot(200, self._inplacewaitthenautotune)
            return
        target, resumestages, label = self._inplacepending
        self._inplacepending = (resumestages, label)
        self.runfermenter("In-place autotune at " + str(target) + " C (profile '" + label + "')...",
                          mode='relayautotune', autotunetarget=target,
                          autotunesafetymargin=self.autotunemargin(),
                          onautotunedone=self._inplaceautotunedone)

    def _inplaceautotunedone(self, result):
        resumestages, label = self._inplacepending
        self.saveautotuneresult(label, result)
        if result is None:
            self.setstatus("In-place autotune aborted - program NOT auto-resumed. "
                           "Check the tunings, then press Start program yourself.")
            return
        self.setstatus("In-place autotune done - resuming the program with the new tunings...")
        self.stop(heateroff=False)
        QtCore.QTimer.singleShot(300, self._inplacewaitthenresume)

    def _inplacewaitthenresume(self):
        if self.running:
            QtCore.QTimer.singleShot(200, self._inplacewaitthenresume)
            return
        resumestages, label = self._inplacepending
        tunings = tuple(self.settings["pid_profiles"][label][k] for k in ("Kp", "Ki", "Kd"))
        self.settings["stages"] = resumestages
        self.store.save(self.settings)
        self.refreshstages()
        self.runfermenter("Program resumed with in-place tunings (profile '" + label + "')...",
                          mode='pidprogram', stages=resumestages, tunings=tunings)

    # ------------------------------------------------------------------
    # Run / stop
    # ------------------------------------------------------------------
    def buildrunsection(self):
        box = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout(box)
        row = QtWidgets.QHBoxLayout()
        self.startbutton = QtWidgets.QPushButton("Start program")
        self.startbutton.clicked.connect(self.startprogram)
        row.addWidget(self.startbutton)
        self.stopbutton = QtWidgets.QPushButton("Stop (keep heating)")
        self.stopbutton.setEnabled(False)
        self.stopbutton.clicked.connect(lambda: self.stop(heateroff=False))
        row.addWidget(self.stopbutton)
        self.stopoffbutton = QtWidgets.QPushButton("Stop & heater off")
        self.stopoffbutton.setEnabled(False)
        self.stopoffbutton.clicked.connect(lambda: self.stop(heateroff=True))
        row.addWidget(self.stopoffbutton)
        layout.addLayout(row)
        self.statuslabel = QtWidgets.QLabel("Idle")
        self.statuslabel.setWordWrap(True)
        layout.addWidget(self.statuslabel)
        return box

    def setstatus(self, text):
        self.statuslabel.setText(text)

    def startprogram(self):
        if self.running:
            return
        if not self.settings["stages"]:
            QtWidgets.QMessageBox.critical(self, "Start program", "Add at least one stage first.")
            return
        try:
            tunings = self.currenttunings()
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Start program", "Kp, Ki and Kd must be numbers.")
            return
        self.store.save(self.settings)
        self.runfermenter("Program running (profile '" + self.profilebox.currentText() + "')...",
                          mode='pidprogram', stages=list(self.settings["stages"]),
                          tunings=tunings)

    def runfermenter(self, statustext, **kwargs):
        self.running = True
        self.startbutton.setEnabled(False)
        self.autotunebutton.setEnabled(False)
        self.stopbutton.setEnabled(True)
        self.stopoffbutton.setEnabled(True)
        self.autotuneherebutton.setEnabled(kwargs.get('mode') == 'pidprogram')
        self.setstatus(statustext)
        try:
            self.fermenter = YogourtFermenter(ontick=self.ontick, autorun=False, showgraph=False, **kwargs)
            self.fermenter.listeningloop()  # blocks; ontick keeps the GUI alive
        except SingleInstanceError as e:
            QtWidgets.QMessageBox.critical(self, "Another instance is already running", str(e))
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Fermenter stopped", str(e))
        finally:
            self.running = False
            self.fermenter = None
            if not self.closing:
                self.startbutton.setEnabled(True)
                self.autotunebutton.setEnabled(True)
                self.stopbutton.setEnabled(False)
                self.stopoffbutton.setEnabled(False)
                self.autotuneherebutton.setEnabled(False)
                self.setstatus("Stopped. The MCU keeps its last setpoint unless you used 'Stop & heater off'.")
                self.progresslabel.setText("Idle - nothing running")
        if self.closing:
            QtWidgets.QApplication.instance().quit()

    def stop(self, heateroff=False):
        if self.fermenter is None:
            return
        if heateroff:
            self.fermenter.setSP(1)
            self.setstatus("Heater off requested, stopping...")
        self.fermenter.stoprequested = True

    # ------------------------------------------------------------------
    # Progress display
    # ------------------------------------------------------------------
    def buildprogresssection(self):
        box = QtWidgets.QGroupBox("Progress")
        layout = QtWidgets.QVBoxLayout(box)
        self.progresslabel = QtWidgets.QLabel("Idle - nothing running")
        self.progresslabel.setWordWrap(True)
        layout.addWidget(self.progresslabel)
        return box

    def updateprogress(self):
        now = time.time()
        if now - self.lastprogressupdate < 1.0:
            return
        self.lastprogressupdate = now
        fermenter = self.fermenter
        if fermenter is None:
            return
        if fermenter.mode == 'pidprogram' and hasattr(fermenter, 'pidprogram'):
            progress = fermenter.pidprogram.getprogress()
            if progress["ended"]:
                text = "Program complete (held " + str(progress["target"]) + " C)"
            else:
                stagelabel = "Stage " + str(progress["stage_index"] + 1) + "/" + str(progress["stage_count"])
                target = str(progress["target"]) + " C"
                if progress["phase"] == "heating":
                    text = (stagelabel + ": heating to " + target
                           + " (current " + str(fermenter.currenttemp) + " C)")
                else:
                    text = (stagelabel + ": holding " + target
                           + "  -  elapsed " + formatduration(progress["stage_elapsed_s"])
                           + " / remaining " + formatduration(progress["stage_remaining_s"]))
            text += "\nTotal elapsed: " + formatduration(progress["total_elapsed_s"])
            text += "   Est. remaining (holds only): " + formatduration(progress["total_remaining_s"])
            self.progresslabel.setText(text)
        elif fermenter.mode == 'relayautotune' and hasattr(fermenter, 'relayautotune'):
            progress = fermenter.relayautotune.getprogress()
            text = ("Autotune at " + str(progress["target"]) + " C: " + progress["phase"]
                   + "  -  cycle " + str(progress["cycles_done"]) + "/" + str(progress["cycles_needed"])
                   + "\nElapsed: " + formatduration(progress["elapsed_s"]))
            self.progresslabel.setText(text)

    # ------------------------------------------------------------------
    # Plots
    # ------------------------------------------------------------------
    def buildplots(self):
        pg.setConfigOptions(antialias=True)
        widget = pg.GraphicsLayoutWidget()

        self.tempplot = widget.addPlot(row=0, col=0, title="Temperature / Output")
        self.tempplot.addLegend()
        self.templine = self.tempplot.plot([], [], pen=pg.mkPen('r', width=2), name="Temp")
        self.cvline = self.tempplot.plot([], [], pen=pg.mkPen('b', width=1), name="Output %")

        self.pidplot = widget.addPlot(row=0, col=1, title="PID terms")
        self.pidplot.addLegend()
        self.pline = self.pidplot.plot([], [], pen=pg.mkPen('r'), name="P")
        self.iline = self.pidplot.plot([], [], pen=pg.mkPen('g'), name="I")
        self.dline = self.pidplot.plot([], [], pen=pg.mkPen('b'), name="D")

        self.zoomplot = widget.addPlot(row=1, col=0, title="Latest temperature")
        self.zoomline = self.zoomplot.plot([], [], pen=pg.mkPen('r', width=2))

        self.errorplot = widget.addPlot(row=1, col=1, title="Seconds since temperature last changed")
        self.errorline = self.errorplot.plot([], [], pen=pg.mkPen('m'))

        return widget

    def updateplots(self):
        now = time.time()
        if now - self.lastplotupdate < 1.0:
            return
        self.lastplotupdate = now
        fermenter = self.fermenter
        if fermenter is None or not fermenter.tempbysec:
            return
        n = min(len(fermenter.tempbysec), len(fermenter.CV), len(fermenter.PIDTermslist))
        x = list(range(n))

        self.templine.setData(x, fermenter.tempbysec[:n])
        self.cvline.setData(x, fermenter.CV[:n])
        self.tempplot.setTitle("MCU target=" + str(fermenter.currentSP))

        self.pline.setData(x, [t[0] for t in fermenter.PIDTermslist[:n]])
        self.iline.setData(x, [t[1] for t in fermenter.PIDTermslist[:n]])
        self.dline.setData(x, [t[2] for t in fermenter.PIDTermslist[:n]])
        settings = fermenter.currentPIDSettings
        self.pidplot.setTitle("P=" + str(settings[0]) + " I=" + str(settings[1]) + " D=" + str(settings[2]))

        zoomx = x[-100:] if n > 100 else x
        zoomy = fermenter.tempbysec[n - 100:n] if n > 100 else fermenter.tempbysec[:n]
        self.zoomline.setData(zoomx, zoomy)
        self.zoomplot.setTitle("Current temp=" + str(fermenter.currenttemp))

        if fermenter.errorlist:
            ex = list(range(len(fermenter.errorlist)))
            ey = [e[1] for e in fermenter.errorlist]
            self.errorline.setData(ex, ey)

    # ------------------------------------------------------------------
    def ontick(self):
        if self.closing:
            if self.fermenter is not None:
                self.fermenter.stoprequested = True
            return
        self.updateprogress()
        self.updateplots()
        QtWidgets.QApplication.instance().processEvents()

    def closeEvent(self, event):
        self.closing = True
        if not self.running:
            event.accept()
        else:
            # ontick() will stop the loop; runfermenter() quits the app on
            # the way out, which ends the Qt event loop cleanly.
            event.ignore()
            if self.fermenter is not None:
                self.fermenter.stoprequested = True


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = YogurtGUIQt()
    gui.show()
    app.exec_()


if __name__ == '__main__':
    main()
