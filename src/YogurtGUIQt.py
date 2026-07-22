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

Design note: unlike the Tkinter GUI (whose fermenter blocks in
listeningloop() and hand-cranks Tk via ontick -> root.update()), this GUI
keeps Qt's own event loop (app.exec_) in charge and steps the fermenter
from a QTimer (see runfermenter/_pumpfermenter). The blocking-loop +
QApplication.processEvents() pump the Tkinter design mirrors was found to
silently wedge this window on multi-day runs - a py-spy dump of a frozen
instance showed the main thread parked in listeningloop()'s recvfrom() with
the real event loop suspended behind it. Letting Qt paint natively fixes
that. Still single-threaded: the pump runs on the GUI thread.
"""

import copy
import os
import sys
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

# HiDPI/4K support. Qt's own auto-scaling depends on the X server/
# compositor reporting the screen's real DPI, which many Linux setups get
# wrong even on a 4K panel (falls back to 96 DPI, making every point-sized
# font and every fixed-pixel widget width look tiny). These attributes let
# Qt's auto-detection work when it can; QT_SCALE_FACTOR (set from
# YOGURT_UI_SCALE below, if given) is the manual override for when it
# can't - both must be set before a QApplication is constructed, so this
# runs at import time rather than inside main().
if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
if os.environ.get('YOGURT_UI_SCALE'):
    os.environ.setdefault('QT_SCALE_FACTOR', os.environ['YOGURT_UI_SCALE'])

from src.SettingsStore import SettingsStore
from src.yogurtdata import YogourtFermenter, SingleInstanceError

# Baseline Qt default is ~9pt, which reads as genuinely tiny text on a 4K
# screen even once DPI scaling is working - bump the whole app's font up,
# and scale the (otherwise fixed-pixel) widget widths by the same ratio so
# entry boxes grow to match instead of clipping the larger text.
DEFAULTFONTPT = 9.0
UIFONTPT = float(os.environ.get('YOGURT_UI_FONT_PT', 16))
UISCALE = UIFONTPT / DEFAULTFONTPT


def applyuifont(app):
    font = app.font()
    font.setPointSizeF(UIFONTPT)
    app.setFont(font)


def scaled(pixels):
    return int(round(pixels * UISCALE))


# Graph palette - a calmer dark theme with a distinguishable, low-fatigue
# color per series (see buildplots()).
BG_COLOR = '#20232a'
FG_COLOR = '#c9d1d9'
COLOR_TEMP = '#ff6e6e'
COLOR_SETPOINT = '#8b95a5'
COLOR_OUTPUT = '#5ec9e2'
COLOR_P = '#ff6e6e'
COLOR_I = '#63d9a6'
COLOR_D = '#f5c94f'
COLOR_ERROR = '#c792ea'

# pyqtgraph's titles/axis labels/legend text do NOT inherit the
# QApplication font at all - they're drawn through pyqtgraph's own
# QGraphicsTextItem-based labels with their own small hardcoded default
# sizes, so without this the graph's titles stay tiny even once every
# other widget in the window has scaled up.
TITLEPT = round(UIFONTPT * 1.15)
TICKPT = max(UIFONTPT - 2, 8)
TITLEROWPX = scaled(36)


def fillcolor(hexcolor, alpha=40):
    return pg.mkColor(hexcolor + format(alpha, '02x'))


def tickfont():
    font = QtGui.QFont()
    font.setPointSize(int(TICKPT))
    return font


def settitle(plot, text):
    plot.setTitle(text, size=str(TITLEPT) + 'pt')
    # setTitle() hardcodes a 30px-tall title row internally, too short for
    # a bigger font and clipping the text - widen it back out every time.
    plot.titleLabel.setMaximumHeight(TITLEROWPX)
    plot.layout.setRowFixedHeight(0, TITLEROWPX)


def styleplot(plot, xlabel='Elapsed'):
    plot.showGrid(x=True, y=True, alpha=0.2)
    # No units on the x label: the values are elapsed seconds, but the
    # TimeAxisItem below renders the ticks as h/d, and pyqtgraph's units=
    # would otherwise SI-prefix them ('ks').
    plot.setLabel('bottom', xlabel, **{'font-size': str(UIFONTPT) + 'pt'})
    plot.getAxis('bottom').setTickFont(tickfont())
    plot.getAxis('left').setTickFont(tickfont())


class TimeAxisItem(pg.AxisItem):
    """X-axis that renders elapsed-seconds tick values as human-readable
    durations (s / m / h / d) instead of pyqtgraph's SI-prefixed seconds
    ('ks', 'Ms'). The resolution shown follows the tick spacing, so a
    multi-day run reads in days while the zoom plot reads in seconds."""

    def tickStrings(self, values, scale, spacing):
        return [self._fmt(v, spacing) for v in values]

    @staticmethod
    def _fmt(value, spacing):
        v = int(round(value))
        sign = '-' if v < 0 else ''
        v = abs(v)
        d, r = divmod(v, 86400)
        h, r = divmod(r, 3600)
        m, s = divmod(r, 60)
        if spacing >= 86400:          # ticks a day or more apart
            return sign + "%dd" % d
        if spacing >= 3600:           # hours
            return sign + (("%dd%dh" % (d, h)) if d else ("%dh" % h))
        if spacing >= 60:             # minutes
            if d:
                return sign + "%dd%dh%02dm" % (d, h, m)
            return sign + (("%dh%02dm" % (h, m)) if h else ("%dm" % m))
        if d or h:                    # seconds, but hours/days present
            return sign + "%dh%02dm%02ds" % (d * 24 + h, m, s)
        return sign + (("%dm%02ds" % (m, s)) if m else ("%ds" % s))


def formatduration(seconds):
    if seconds is None:
        return "-"
    seconds = max(0, int(seconds))
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    return "%d:%02d:%02d" % (hours, minutes, seconds)


def formatpid(value):
    """Kp/Ki/Kd values often come from autotune with a dozen+ decimal digits
    of floating-point noise (e.g. 19.75716534933873) - 5 decimal digits is
    already far more precision than the tuning process or the firmware's own
    resolution can use."""
    return str(round(float(value), 5))


# Dark "instrument panel" theme for the control widgets, so they read as one
# piece with the already-dark pyqtgraph plots (panels share the plots'
# #20232a). The subject is heat/fermentation, so the one warm accent (amber,
# used only on the primary Start action and on the numeric readouts) is the
# "heat"; everything else stays quiet so the live graphs remain the hero.
# Signature: numeric fields (Kp/Ki/Kd, temp, hold) and the live readouts are
# set in a monospace face, tinted warm - an instrument's digits.
THEME = {
    'bg': '#191c22', 'panel': '#20232a', 'raised': '#262b34', 'line': '#333a45',
    'text': '#d7dde5', 'muted': '#8b95a5', 'accent': '#ff8a5c', 'cool': '#5ec9e2',
    'danger': '#e0685f', 'digit': '#ffcfa6',
}
MONO = '"JetBrains Mono","DejaVu Sans Mono","Consolas",monospace'


def _caretpath(color):
    """Draw a small downward caret to a PNG and return its path. Styling a
    QComboBox via QSS makes Qt stop drawing its native dropdown arrow, and
    the pure-QSS border-triangle trick renders unreliably - so we paint a
    real arrow image with Qt itself (no shipped asset needed) and point the
    stylesheet's QComboBox::down-arrow at it."""
    import tempfile
    scale = 2  # oversample so it stays crisp on HiDPI
    w, h = 12 * scale, 8 * scale
    pm = QtGui.QPixmap(w, h)
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    p.setRenderHint(QtGui.QPainter.Antialiasing, True)
    p.setPen(QtCore.Qt.NoPen)
    p.setBrush(QtGui.QColor(color))
    m = scale
    p.drawPolygon(QtGui.QPolygon([
        QtCore.QPoint(m, m + scale), QtCore.QPoint(w - m, m + scale),
        QtCore.QPoint(w // 2, h - m)]))
    p.end()
    path = os.path.join(tempfile.gettempdir(),
                        "yogurt_caret_" + color.lstrip('#') + ".png")
    pm.save(path, "PNG")
    return path.replace("\\", "/")


def applytheme(app):
    app.setStyle('Fusion')
    t = THEME
    caret = _caretpath(t['muted'])
    app.setStyleSheet(("""
        QWidget {{ background: {bg}; color: {text}; }}
        QMainWindow, QDialog {{ background: {bg}; }}
        QLabel, QCheckBox {{ background: transparent; }}
        QGroupBox {{
            background: {panel}; border: 1px solid {line}; border-radius: 9px;
            margin-top: 16px; padding: 12px 12px 10px 12px; font-weight: 600;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin; subcontrol-position: top left;
            left: 12px; padding: 1px 6px; color: {muted};
        }}
        QPushButton {{
            background: {raised}; color: {text}; border: 1px solid {line};
            border-radius: 7px; padding: 6px 13px;
        }}
        QPushButton:hover {{ background: #2e3540; border-color: #4a5360; }}
        QPushButton:pressed {{ background: #1f242c; }}
        QPushButton:disabled {{ background: #1e2229; color: #59626f; border-color: #2a303a; }}
        QPushButton#primaryButton {{
            background: {accent}; color: #241503; border: none; font-weight: 700;
        }}
        QPushButton#primaryButton:hover {{ background: #ff9d75; }}
        QPushButton#primaryButton:pressed {{ background: #f07c4e; }}
        QPushButton#primaryButton:disabled {{ background: #3a2f28; color: #7c6c5e; }}
        QPushButton#dangerButton {{
            background: transparent; color: {danger}; border: 1px solid #7a3a37;
        }}
        QPushButton#dangerButton:hover {{ background: #2a1d1c; border-color: {danger}; }}
        QPushButton#dangerButton:disabled {{ color: #644745; border-color: #382a29; }}
        QLineEdit, QComboBox {{
            background: {raised}; color: {text}; border: 1px solid {line};
            border-radius: 6px; padding: 4px 8px;
            selection-background-color: {cool}; selection-color: #0f141a;
        }}
        QLineEdit:focus, QComboBox:focus {{ border-color: {accent}; }}
        QLineEdit:disabled, QComboBox:disabled {{ color: #59626f; background: #1e2229; }}
        QComboBox::drop-down {{
            subcontrol-origin: padding; subcontrol-position: center right;
            width: 22px; border: none; border-left: 1px solid {line};
        }}
        /* Styling QComboBox via QSS drops its native arrow; point it at a
           caret we painted (see _caretpath) so it still reads as a dropdown. */
        QComboBox::down-arrow {{
            image: url("{caret}"); width: 11px; height: 7px; margin-right: 7px;
        }}
        QComboBox QAbstractItemView {{
            background: {panel}; color: {text}; border: 1px solid {line};
            selection-background-color: #2e3540; outline: none;
        }}
        QLineEdit[role="digit"] {{ font-family: {mono}; color: {digit}; }}
        QTableWidget {{
            background: {panel}; alternate-background-color: #23272f;
            gridline-color: #2b313b; border: 1px solid {line}; border-radius: 6px;
            selection-background-color: #2e3a44; selection-color: {text};
        }}
        QHeaderView::section {{
            background: {raised}; color: {muted}; padding: 5px 6px; border: none;
            border-right: 1px solid #2b313b; border-bottom: 1px solid #2b313b;
        }}
        QTableCornerButton::section {{ background: {raised}; border: none; }}
        QCheckBox::indicator {{
            width: 16px; height: 16px; border: 1px solid #4a5360;
            border-radius: 4px; background: {raised};
        }}
        QCheckBox::indicator:checked {{ background: {accent}; border-color: {accent}; }}
        QLabel#readout {{ font-family: {mono}; color: #cbd3dd; }}
        QScrollBar:vertical {{ background: {bg}; width: 12px; margin: 0; }}
        QScrollBar::handle:vertical {{ background: {line}; border-radius: 6px; min-height: 28px; }}
        QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
        QToolTip {{ background: {panel}; color: {text}; border: 1px solid {line}; }}
    """).format(mono=MONO, caret=caret, **t))


class YogurtGUIQt(QtWidgets.QMainWindow):
    def __init__(self, settingsstore=None):
        super().__init__()
        app = QtWidgets.QApplication.instance()
        if app is not None:
            applyuifont(app)
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
        self.plotwidget = self.buildplots()
        outer.addWidget(self.plotwidget, stretch=1)

        controls.addWidget(self.buildstagesection())
        controls.addWidget(self.buildpidsection())
        controls.addWidget(self.buildautotunesection())
        controls.addWidget(self.buildrunsection())
        controls.addWidget(self.buildprogresssection())
        controls.addStretch(1)

        self.refreshstages()
        self.refreshprograms()
        self.refreshprofiles()
        self.resize(scaled(1600), scaled(800))

        # Watchdog against the Wayland presentation freeze (see
        # _forcerepaint). YOGURT_QT_REPAINT_SECONDS sets the interval;
        # 0 disables it.
        self.repaintwatchdogseconds = float(os.environ.get('YOGURT_QT_REPAINT_SECONDS', 2.0))
        self.repaintwatchdog = QtCore.QTimer(self)
        self.repaintwatchdog.timeout.connect(self._forcerepaint)
        if self.repaintwatchdogseconds > 0:
            self.repaintwatchdog.start(int(self.repaintwatchdogseconds * 1000))

    # ------------------------------------------------------------------
    # Program stages
    # ------------------------------------------------------------------
    def buildstagesection(self):
        box = QtWidgets.QGroupBox("Program stages (heat to temp, then hold for duration)")
        layout = QtWidgets.QVBoxLayout(box)

        self.stagetable = QtWidgets.QTableWidget(0, 4)
        self.stagetable.setHorizontalHeaderLabels(["Temperature (C)", "Hold (minutes)", "PID profile", "Approach"])
        self.stagetable.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.stagetable.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.stagetable.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.stagetable.verticalHeader().setVisible(False)
        self.stagetable.setMaximumHeight(scaled(120))
        layout.addWidget(self.stagetable)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Temp (C):"))
        self.stagetempentry = QtWidgets.QLineEdit()
        self.stagetempentry.setProperty("role", "digit")
        self.stagetempentry.setMaximumWidth(scaled(60))
        row.addWidget(self.stagetempentry)
        row.addWidget(QtWidgets.QLabel("Hold (min):"))
        self.stageminutesentry = QtWidgets.QLineEdit()
        self.stageminutesentry.setProperty("role", "digit")
        self.stageminutesentry.setMaximumWidth(scaled(70))
        row.addWidget(self.stageminutesentry)
        addbutton = QtWidgets.QPushButton("Add stage")
        addbutton.clicked.connect(self.addstage)
        row.addWidget(addbutton)
        removebutton = QtWidgets.QPushButton("Remove selected")
        removebutton.clicked.connect(self.removestage)
        row.addWidget(removebutton)
        layout.addLayout(row)

        rowprofile = QtWidgets.QHBoxLayout()
        rowprofile.addWidget(QtWidgets.QLabel("Stage's PID profile:"))
        self.stageprofilebox = QtWidgets.QComboBox()
        self.stageprofilebox.setEditable(True)
        rowprofile.addWidget(self.stageprofilebox)
        assignbutton = QtWidgets.QPushButton("Assign to selected")
        assignbutton.clicked.connect(self.assignstageprofile)
        rowprofile.addWidget(assignbutton)
        rowprofile.addWidget(QtWidgets.QLabel("(blank = use the active profile below)"))
        layout.addLayout(rowprofile)

        rowapproach = QtWidgets.QHBoxLayout()
        self.fastapproachcheck = QtWidgets.QCheckBox("Fast approach (allow overshoot)")
        rowapproach.addWidget(self.fastapproachcheck)
        toggleapproachbutton = QtWidgets.QPushButton("Toggle for selected")
        toggleapproachbutton.clicked.connect(self.togglestagefastapproach)
        rowapproach.addWidget(toggleapproachbutton)
        rowapproach.addWidget(QtWidgets.QLabel("(skips the gentle setpoint ramp - faster, but can overshoot)"))
        layout.addLayout(rowapproach)

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
            self.stagetable.setItem(r, 2, QtWidgets.QTableWidgetItem(stage.get("profile") or "(active profile)"))
            self.stagetable.setItem(r, 3, QtWidgets.QTableWidgetItem(
                "fast" if stage.get("fast_approach") else "gentle"))

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
        stage = {"temperature": temperature, "duration_minutes": minutes}
        profile = self.stageprofilebox.currentText().strip()
        if profile:
            stage["profile"] = profile
        if self.fastapproachcheck.isChecked():
            stage["fast_approach"] = True
        self.settings["stages"].append(stage)
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

    def assignstageprofile(self):
        rows = sorted({idx.row() for idx in self.stagetable.selectedIndexes()})
        if not rows:
            QtWidgets.QMessageBox.information(self, "Assign profile", "Select one or more stages first.")
            return
        profile = self.stageprofilebox.currentText().strip()
        if profile and profile not in self.settings["pid_profiles"]:
            QtWidgets.QMessageBox.critical(self, "Assign profile", "No saved PID profile named '" + profile + "'.")
            return
        for r in rows:
            if profile:
                self.settings["stages"][r]["profile"] = profile
            else:
                self.settings["stages"][r].pop("profile", None)
        self.store.save(self.settings)
        self.refreshstages()

    def togglestagefastapproach(self):
        """Sets selected stages' fast_approach flag to whatever the
        checkbox currently shows (not a per-row toggle - the checkbox is
        the value being applied, matching "Assign to selected" above)."""
        rows = sorted({idx.row() for idx in self.stagetable.selectedIndexes()})
        if not rows:
            QtWidgets.QMessageBox.information(self, "Fast approach", "Select one or more stages first.")
            return
        fast = self.fastapproachcheck.isChecked()
        for r in rows:
            if fast:
                self.settings["stages"][r]["fast_approach"] = True
            else:
                self.settings["stages"][r].pop("fast_approach", None)
        self.store.save(self.settings)
        self.refreshstages()

    def resolvestagetunings(self, stages):
        """(Kp, Ki, Kd) per stage, or None where the stage has no profile of
        its own - PIDProgram falls back to the globally selected profile."""
        result = []
        for stage in stages:
            name = stage.get("profile")
            profile = self.settings["pid_profiles"].get(name) if name else None
            result.append((profile["Kp"], profile["Ki"], profile["Kd"]) if profile else None)
        return result

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
            entry.setProperty("role", "digit")
            entry.setMaximumWidth(scaled(90))
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
        self.applylivebutton = QtWidgets.QPushButton("Apply now (live)")
        self.applylivebutton.setEnabled(False)
        self.applylivebutton.clicked.connect(self.applylivetunings)
        row3.addWidget(self.applylivebutton)
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
        self.stageprofilebox.clear()
        self.stageprofilebox.addItems([""] + names)
        self.stageprofilebox.setCurrentText("")

    def loadprofileentries(self, name):
        profile = self.settings["pid_profiles"].get(name)
        if profile is None:
            return
        for key, entry in self.pidentries.items():
            entry.setText(formatpid(profile[key]))

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

    def applylivetunings(self):
        """Push the Kp/Ki/Kd entries to the running controller immediately,
        without stopping the program or waiting for the next stage change -
        for tweaking a profile by hand while watching the graph."""
        if self.fermenter is None:
            return
        try:
            kp, ki, kd = self.currenttunings()
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Apply now", "Kp, Ki and Kd must be numbers.")
            return
        self.fermenter.setconsPIDvalues(kp, ki, kd)
        self.setstatus("Applied live tunings: Kp=" + str(kp) + " Ki=" + str(ki) + " Kd=" + str(kd))

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
        self.autotunetargetentry.setProperty("role", "digit")
        self.autotunetargetentry.setMaximumWidth(scaled(60))
        row.addWidget(self.autotunetargetentry)
        row.addWidget(QtWidgets.QLabel("Margin (C):"))
        self.autotunemarginentry = QtWidgets.QLineEdit("12")
        self.autotunemarginentry.setProperty("role", "digit")
        self.autotunemarginentry.setMaximumWidth(scaled(50))
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
                          mode='pidprogram', stages=resumestages, tunings=tunings,
                          stagetunings=self.resolvestagetunings(resumestages))

    # ------------------------------------------------------------------
    # Run / stop
    # ------------------------------------------------------------------
    def buildrunsection(self):
        box = QtWidgets.QGroupBox()
        layout = QtWidgets.QVBoxLayout(box)
        row = QtWidgets.QHBoxLayout()
        self.startbutton = QtWidgets.QPushButton("Start program")
        self.startbutton.setObjectName("primaryButton")
        self.startbutton.clicked.connect(self.startprogram)
        row.addWidget(self.startbutton)
        self.stopbutton = QtWidgets.QPushButton("Stop (keep heating)")
        self.stopbutton.setEnabled(False)
        self.stopbutton.clicked.connect(lambda: self.stop(heateroff=False))
        row.addWidget(self.stopbutton)
        self.stopoffbutton = QtWidgets.QPushButton("Stop & heater off")
        self.stopoffbutton.setObjectName("dangerButton")
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
        stages = list(self.settings["stages"])
        self.runfermenter("Program running (profile '" + self.profilebox.currentText() + "')...",
                          mode='pidprogram', stages=stages,
                          tunings=tunings, stagetunings=self.resolvestagetunings(stages))

    def runfermenter(self, statustext, **kwargs):
        self.running = True
        self.startbutton.setEnabled(False)
        self.autotunebutton.setEnabled(False)
        self.stopbutton.setEnabled(True)
        self.stopoffbutton.setEnabled(True)
        self.autotuneherebutton.setEnabled(kwargs.get('mode') == 'pidprogram')
        self.applylivebutton.setEnabled(kwargs.get('mode') == 'pidprogram')
        self.setstatus(statustext)
        # Crucially, do NOT pass ontick / do NOT call listeningloop() here.
        # The old design blocked in listeningloop() and hand-cranked the GUI
        # via QApplication.processEvents() from ontick; a py-spy dump of a
        # frozen instance proved that suspends Qt's real event loop and the
        # window silently stops repainting on long runs. Instead we keep
        # app.exec_() running and step the fermenter from a QTimer, so Qt
        # owns painting the normal, robust way.
        try:
            self.fermenter = YogourtFermenter(autorun=False, showgraph=False, **kwargs)
        except SingleInstanceError as e:
            QtWidgets.QMessageBox.critical(self, "Another instance is already running", str(e))
            self._fermenterstopped()
            return
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Fermenter stopped", str(e))
            self._fermenterstopped()
            return
        self._lastpumpdiag = 0.0
        self.looptimer = QtCore.QTimer(self)
        self.looptimer.timeout.connect(self._pumpfermenter)
        self.looptimer.start(100)  # 10 Hz: poll the socket + refresh the UI

    def _pumpfermenter(self):
        """One control-loop step, driven by Qt's event loop instead of
        blocking it. Runs on the GUI thread, so touching widgets here is
        safe."""
        fermenter = self.fermenter
        if fermenter is None:
            return
        if self.closing:
            fermenter.stoprequested = True
        if fermenter.stoprequested:
            self.looptimer.stop()
            try:
                fermenter.closelistening()
            except Exception as e:
                print("closelistening error (ignored):", repr(e))
            print("Listening loop stopped.")
            self._fermenterstopped()
            if self.closing:
                QtWidgets.QApplication.instance().quit()
            return
        try:
            fermenter.steponce(blocking=False, runontick=False)
        except Exception as e:
            print("Fermenter step error (ignored):", repr(e))
        self.updateprogress()
        self.updateplots()
        # Heartbeat to the diagnostics log so a future freeze is diagnosable
        # at a glance: if the window ever stops updating while these keep
        # being written, the timer is still firing (a compositor/WM-side
        # freeze); if they stop, the pump itself wedged.
        now = time.time()
        if now - self._lastpumpdiag >= 60:
            self._lastpumpdiag = now
            try:
                fermenter.diaglog("QT_PUMP_HEARTBEAT temp=" + str(fermenter.currenttemp)
                                  + " sp=" + str(fermenter.currentSP)
                                  + " points=" + str(len(fermenter.tempbysec)))
            except Exception:
                pass

    def _fermenterstopped(self):
        self.running = False
        self.fermenter = None
        if not self.closing:
            self.startbutton.setEnabled(True)
            self.autotunebutton.setEnabled(True)
            self.stopbutton.setEnabled(False)
            self.stopoffbutton.setEnabled(False)
            self.autotuneherebutton.setEnabled(False)
            self.applylivebutton.setEnabled(False)
            self.setstatus("Stopped. The MCU keeps its last setpoint unless you used 'Stop & heater off'.")
            self.progresslabel.setText("Idle - nothing running")

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
        self.progresslabel.setObjectName("readout")
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
        # A calmer dark theme reads better for a graph that may be watched
        # for hours: pyqtgraph's default black-on-white grid is harsh, and
        # plain red/green/blue lines fight each other for attention.
        #
        # antialias stays globally OFF: Qt's raster engine has no fast path
        # for antialiased pens wider than 1px, making each dense curve
        # ~10-40x more expensive to stroke (measured 40+ ms per curve per
        # redraw at this window size) - across 7 curves redrawn every
        # second that was a major part of the GUI freezing on long runs.
        # The short zoom curve opts back in below (100 points, negligible).
        pg.setConfigOptions(antialias=False, background=BG_COLOR, foreground=FG_COLOR)
        widget = pg.GraphicsLayoutWidget()
        widget.setBackground(BG_COLOR)

        labelstyle = {'font-size': str(UIFONTPT) + 'pt'}

        self.tempplot = widget.addPlot(row=0, col=0,
                                       axisItems={'bottom': TimeAxisItem(orientation='bottom')})
        settitle(self.tempplot, "Temperature / Output")
        styleplot(self.tempplot)
        self.tempplot.setLabel('left', 'Temperature', units='°C', **labelstyle)
        self.tempplot.getAxis('left').setPen(pg.mkPen(COLOR_TEMP))
        self.tempplot.getAxis('left').setTextPen(pg.mkPen(COLOR_TEMP))
        templegend = self.tempplot.addLegend(offset=(-10, 10))
        templegend.setLabelTextSize(str(TICKPT) + 'pt')
        self.templine = self.tempplot.plot([], [], pen=pg.mkPen(COLOR_TEMP, width=2.5),
                                           fillLevel=0, brush=pg.mkBrush(fillcolor(COLOR_TEMP)),
                                           name="Temp")
        self.spline = pg.InfiniteLine(angle=0, pen=pg.mkPen(COLOR_SETPOINT, width=1.5,
                                                             style=QtCore.Qt.DashLine))
        self.tempplot.addItem(self.spline, ignoreBounds=True)

        # Output shares the same plot but not the same scale (0-100% vs a
        # temperature range) - a second Y axis keeps both readable instead
        # of one flattening the other.
        self.outputviewbox = pg.ViewBox()
        self.tempplot.showAxis('right')
        self.tempplot.scene().addItem(self.outputviewbox)
        self.tempplot.getAxis('right').linkToView(self.outputviewbox)
        self.outputviewbox.setXLink(self.tempplot)
        self.tempplot.getAxis('right').setLabel('Output', units='%', **labelstyle)
        self.tempplot.getAxis('right').setTickFont(tickfont())
        self.tempplot.getAxis('right').setPen(pg.mkPen(COLOR_OUTPUT))
        self.tempplot.getAxis('right').setTextPen(pg.mkPen(COLOR_OUTPUT))
        # PlotDataItem (not PlotCurveItem): only PlotDataItem supports the
        # auto-downsampling used by downsampled() below.
        self.cvline = pg.PlotDataItem(pen=pg.mkPen(COLOR_OUTPUT, width=1.5))
        self.outputviewbox.addItem(self.cvline)
        templegend.addItem(self.cvline, "Output %")

        def syncoutputview():
            self.outputviewbox.setGeometry(self.tempplot.vb.sceneBoundingRect())
            self.outputviewbox.linkedViewChanged(self.tempplot.vb, self.outputviewbox.XAxis)
        syncoutputview()
        self.tempplot.vb.sigResized.connect(syncoutputview)

        self.pidplot = widget.addPlot(row=0, col=1,
                                      axisItems={'bottom': TimeAxisItem(orientation='bottom')})
        settitle(self.pidplot, "PID terms")
        styleplot(self.pidplot)
        pidlegend = self.pidplot.addLegend(offset=(10, 10))
        pidlegend.setLabelTextSize(str(TICKPT) + 'pt')
        self.pline = self.pidplot.plot([], [], pen=pg.mkPen(COLOR_P, width=1.5), name="P")
        self.iline = self.pidplot.plot([], [], pen=pg.mkPen(COLOR_I, width=1.5), name="I")
        self.dline = self.pidplot.plot([], [], pen=pg.mkPen(COLOR_D, width=1.5), name="D")

        self.zoomplot = widget.addPlot(row=1, col=0,
                                       axisItems={'bottom': TimeAxisItem(orientation='bottom')})
        settitle(self.zoomplot, "Latest temperature")
        styleplot(self.zoomplot)
        self.zoomplot.setLabel('left', 'Temperature', units='°C', **labelstyle)
        self.zoomline = self.zoomplot.plot([], [], pen=pg.mkPen(COLOR_TEMP, width=2.5),
                                           fillLevel=0, brush=pg.mkBrush(fillcolor(COLOR_TEMP)),
                                           antialias=True)
        self.zoomspline = pg.InfiniteLine(angle=0, pen=pg.mkPen(COLOR_SETPOINT, width=1.5,
                                                                 style=QtCore.Qt.DashLine))
        self.zoomplot.addItem(self.zoomspline, ignoreBounds=True)

        self.errorplot = widget.addPlot(row=1, col=1)
        settitle(self.errorplot, "Seconds since temperature last changed")
        styleplot(self.errorplot, xlabel='Samples')
        self.errorplot.setLabel('left', 'Staleness', units='s', **labelstyle)
        self.errorline = self.errorplot.plot([], [], pen=pg.mkPen(COLOR_ERROR, width=1.5))

        # The long-history curves are redrawn every second for the whole
        # run. Without downsampling, painting cost grows linearly with run
        # length (measured: ~1 s per redraw at 5 h of data, ~4 s at the
        # 20 h cap) - past a few hours the loop spends its whole time
        # painting and the window freezes-by-starvation. 'peak' keeps the
        # min/max envelope per screen pixel, so spikes stay visible;
        # clipToView skips whatever is panned/zoomed out of sight.
        for line in (self.templine, self.cvline, self.pline, self.iline,
                     self.dline, self.errorline):
            line.setDownsampling(auto=True, method='peak')
            line.setClipToView(True)

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
        started = time.perf_counter()
        # One numpy conversion per list (a single C pass) instead of Python
        # list slicing/comprehensions every second over the full history.
        # Each sample is one second; offset by trimmedseconds (the history
        # dropped by cleanPlotlists once past the 20 h cap) so the time axis
        # keeps showing true elapsed time - into days - rather than resetting.
        offset = getattr(fermenter, 'trimmedseconds', 0)
        x = np.arange(n) + offset
        pidterms = np.asarray(fermenter.PIDTermslist[:n], dtype=float)

        self.templine.setData(x, np.asarray(fermenter.tempbysec[:n], dtype=float))
        self.cvline.setData(x, np.asarray(fermenter.CV[:n], dtype=float))
        self.spline.setValue(fermenter.currentSP)
        settitle(self.tempplot, "MCU target=" + str(fermenter.currentSP))

        self.pline.setData(x, pidterms[:, 0])
        self.iline.setData(x, pidterms[:, 1])
        self.dline.setData(x, pidterms[:, 2])
        settings = fermenter.currentPIDSettings
        settitle(self.pidplot, "P=" + formatpid(settings[0]) + " I=" + formatpid(settings[1])
                 + " D=" + formatpid(settings[2]))

        zoomstart = max(0, n - 100)
        self.zoomline.setData(x[zoomstart:n], np.asarray(fermenter.tempbysec[zoomstart:n], dtype=float))
        self.zoomspline.setValue(fermenter.currentSP)
        settitle(self.zoomplot, "Current temp=" + str(fermenter.currenttemp))

        if fermenter.errorlist:
            ey = np.asarray([e[1] for e in fermenter.errorlist], dtype=float)
            self.errorline.setData(np.arange(len(ey)), ey)

        # Leave evidence if this ever creeps toward starving the event loop
        # again - same idea as yogurtdata's SLOW_REDRAW diagnostics.
        elapsedms = (time.perf_counter() - started) * 1000
        if elapsedms > 250 and hasattr(fermenter, 'diaglog'):
            fermenter.diaglog("QT_SLOW_PLOT ms=" + str(round(elapsedms, 1)) + " points=" + str(n))

    def _forcerepaint(self):
        """Watchdog against the Wayland/XWayland presentation freeze.

        On a Wayland session (here niri, with XWayland provided by
        xwayland-satellite) this GUI runs through XWayland - the venv's
        bundled Qt has no native Wayland platform plugin. When the window is
        left where the compositor isn't compositing it (a non-visible niri
        workspace, or an idle-blanked output) the compositor stops sending
        frame callbacks, and via xwayland-satellite the surface can stop
        presenting new frames and fail to resume when shown again - the
        window looks frozen even though the process is completely healthy
        (proven on a live frozen instance: a py-spy dump showed the main
        thread idle in the event loop while QT_PUMP_HEARTBEAT lines and the
        control loop kept running normally).

        This is the same 'compositor silently stops redrawing a long-lived
        window' failure the matplotlib GUI guards against by recreating its
        window (YOGURT_GRAPH_REFRESH_SECONDS / recreategraph). Here we force
        a *synchronous* repaint on a timer: repaint() does not wait for a
        frame callback, so it commits a fresh buffer regardless - when the
        window becomes visible again the latest frame is presented within
        one interval instead of staying frozen. Interval:
        YOGURT_QT_REPAINT_SECONDS (default 2 s; 0 disables).

        The real fix is to not go through XWayland at all (a Qt with the
        Wayland platform plugin), or a newer xwayland-satellite; this is the
        in-app mitigation for the current setup."""
        try:
            self.plotwidget.viewport().repaint()
        except Exception:
            pass

    def closeEvent(self, event):
        self.closing = True
        if not self.running:
            event.accept()
        else:
            # The pump timer (_pumpfermenter) sees stoprequested on its next
            # tick, tears the fermenter down cleanly, and quits the app -
            # which ends app.exec_() and closes the window.
            event.ignore()
            if self.fermenter is not None:
                self.fermenter.stoprequested = True


def main():
    app = QtWidgets.QApplication(sys.argv)
    applytheme(app)
    gui = YogurtGUIQt()
    gui.show()
    app.exec_()


if __name__ == '__main__':
    main()
