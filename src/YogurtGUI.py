"""Control panel for the yogurt fermenter.

Run from the project root:
    python -m src.YogurtGUI

Features:
- edit the fermentation program (stages: temperature + hold duration),
  saved to yogurt_settings.json,
- save/select/delete named PID tunings,
- run a relay autotune and save the resulting tunings under a chosen name,
- start/stop the fermentation program (the matplotlib graph opens as usual).

Design note: the fermenter's listening loop runs in this same thread and
calls back into ontick() several times per second, which pumps tkinter
events. This keeps everything single-threaded (tkinter and matplotlib both
dislike background threads).
"""

import copy
import os
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, messagebox


# HiDPI/4K support. `ttk.Entry`/`Combobox` widths are character-based and
# scale on their own once the named fonts below grow, but Tk's own DPI
# probe (winfo_fpixels) is often wrong on Linux (many X servers/compositors
# misreport it), leaving fonts tiny on a 4K screen even though the widgets
# around them look normal-sized. YOGURT_UI_SCALE, if set, overrides the
# computed Tk scaling factor directly; YOGURT_UI_FONT_PT sets the base
# point size applied to every named Tk font (default 13, vs Tk's ~9-10
# default - genuinely small on a 4K panel).
UIFONTPT = int(os.environ.get('YOGURT_UI_FONT_PT', 16))


def applyuiscaling(root):
    override = os.environ.get('YOGURT_UI_SCALE')
    if override:
        root.tk.call('tk', 'scaling', float(override))
    else:
        dpi = root.winfo_fpixels('1i')
        root.tk.call('tk', 'scaling', dpi / 72.0)
    for fontname in ("TkDefaultFont", "TkTextFont", "TkHeadingFont", "TkMenuFont",
                     "TkFixedFont", "TkIconFont", "TkCaptionFont", "TkSmallCaptionFont"):
        try:
            tkfont.nametofont(fontname).configure(size=UIFONTPT)
        except tk.TclError:
            pass


def scaledpx(pixels):
    return int(round(pixels * UIFONTPT / 9.0))


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

from src.SettingsStore import SettingsStore
from src.yogurtdata import YogourtFermenter, SingleInstanceError


class YogurtGUI:
    def __init__(self, settingsstore=None):
        self.store = settingsstore if settingsstore is not None else SettingsStore()
        self.settings = self.store.load()
        self.fermenter = None
        self.running = False
        self.closing = False
        self._inplacepending = None

        self.root = tk.Tk()
        applyuiscaling(self.root)
        self.root.title("Yogurt Fermenter")
        self.root.protocol("WM_DELETE_WINDOW", self.onclose)
        body = ttk.Frame(self.root, padding=10)
        body.grid(sticky="nsew")
        self.buildstagesection(body)
        self.buildpidsection(body)
        self.buildautotunesection(body)
        self.buildrunsection(body)
        self.buildprogresssection(body)
        self.lastprogressupdate = 0.0
        self.refreshstages()
        self.refreshprograms()
        self.refreshprofiles()

    # ------------------------------------------------------------------
    # Program stages
    # ------------------------------------------------------------------
    def buildstagesection(self, parent):
        frame = ttk.LabelFrame(parent, text="Program stages (heat to temp, then hold for duration)", padding=8)
        frame.grid(row=0, column=0, sticky="ew", pady=4)
        self.stagetree = ttk.Treeview(frame, columns=("temp", "minutes", "profile", "approach"),
                                      show="headings", height=4)
        self.stagetree.heading("temp", text="Temperature (C)")
        self.stagetree.heading("minutes", text="Hold (minutes)")
        self.stagetree.heading("profile", text="PID profile")
        self.stagetree.heading("approach", text="Approach")
        self.stagetree.column("temp", width=scaledpx(140), anchor="center")
        self.stagetree.column("minutes", width=scaledpx(140), anchor="center")
        self.stagetree.column("profile", width=scaledpx(160), anchor="center")
        self.stagetree.column("approach", width=scaledpx(110), anchor="center")
        self.stagetree.grid(row=0, column=0, columnspan=6, sticky="ew")

        ttk.Label(frame, text="Temp (C):").grid(row=1, column=0, pady=4)
        self.stagetempentry = ttk.Entry(frame, width=7)
        self.stagetempentry.grid(row=1, column=1)
        ttk.Label(frame, text="Hold (min):").grid(row=1, column=2)
        self.stageminutesentry = ttk.Entry(frame, width=7)
        self.stageminutesentry.grid(row=1, column=3)
        ttk.Button(frame, text="Add stage", command=self.addstage).grid(row=1, column=4, padx=4)
        ttk.Button(frame, text="Remove selected", command=self.removestage).grid(row=1, column=5, padx=4)

        ttk.Label(frame, text="Stage's PID profile:").grid(row=2, column=0, pady=4)
        self.stageprofilebox = ttk.Combobox(frame, width=18)
        self.stageprofilebox.grid(row=2, column=1, sticky="w", padx=4)
        ttk.Button(frame, text="Assign to selected", command=self.assignstageprofile).grid(
            row=2, column=2, columnspan=2, padx=2, sticky="w")
        ttk.Label(frame, text="(blank = use the active profile above)").grid(
            row=2, column=4, columnspan=2, sticky="w")

        self.fastapproachvar = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="Fast approach (allow overshoot)",
                        variable=self.fastapproachvar).grid(row=3, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Button(frame, text="Toggle for selected", command=self.togglestagefastapproach).grid(
            row=3, column=2, padx=2, sticky="w")
        ttk.Label(frame, text="(skips the gentle setpoint ramp - faster, but can overshoot)").grid(
            row=3, column=3, columnspan=3, sticky="w")

        ttk.Label(frame, text="Saved programs:").grid(row=4, column=0, pady=4)
        self.programbox = ttk.Combobox(frame, width=18)
        self.programbox.grid(row=4, column=1, columnspan=2, sticky="w", padx=4)
        ttk.Button(frame, text="Save", command=self.saveprogram).grid(row=4, column=3, padx=2)
        ttk.Button(frame, text="Load", command=self.loadprogram).grid(row=4, column=4, padx=2)
        ttk.Button(frame, text="Delete", command=self.deleteprogram).grid(row=4, column=5, padx=2)

    def refreshstages(self):
        self.stagetree.delete(*self.stagetree.get_children())
        for stage in self.settings["stages"]:
            self.stagetree.insert("", "end", values=(
                stage["temperature"], stage["duration_minutes"],
                stage.get("profile") or "(active profile)",
                "fast" if stage.get("fast_approach") else "gentle"))

    def addstage(self):
        try:
            temperature = float(self.stagetempentry.get())
            minutes = float(self.stageminutesentry.get())
        except ValueError:
            messagebox.showerror("Invalid stage", "Temperature and hold minutes must be numbers.")
            return
        if not 0 < temperature <= 95:
            messagebox.showerror("Invalid stage", "Temperature must be between 0 and 95 C.")
            return
        if minutes < 0:
            messagebox.showerror("Invalid stage", "Hold minutes cannot be negative.")
            return
        stage = {"temperature": temperature, "duration_minutes": minutes}
        profile = self.stageprofilebox.get().strip()
        if profile:
            stage["profile"] = profile
        if self.fastapproachvar.get():
            stage["fast_approach"] = True
        self.settings["stages"].append(stage)
        self.store.save(self.settings)
        self.refreshstages()

    def removestage(self):
        selected = self.stagetree.selection()
        if not selected:
            messagebox.showinfo("Remove stage", "Select a stage to remove first.")
            return
        indexes = sorted((self.stagetree.index(item) for item in selected), reverse=True)
        for index in indexes:
            del self.settings["stages"][index]
        self.store.save(self.settings)
        self.refreshstages()

    def assignstageprofile(self):
        selected = self.stagetree.selection()
        if not selected:
            messagebox.showinfo("Assign profile", "Select one or more stages first.")
            return
        profile = self.stageprofilebox.get().strip()
        if profile and profile not in self.settings["pid_profiles"]:
            messagebox.showerror("Assign profile", "No saved PID profile named '" + profile + "'.")
            return
        for item in selected:
            index = self.stagetree.index(item)
            if profile:
                self.settings["stages"][index]["profile"] = profile
            else:
                self.settings["stages"][index].pop("profile", None)
        self.store.save(self.settings)
        self.refreshstages()

    def togglestagefastapproach(self):
        """Sets selected stages' fast_approach flag to whatever the
        checkbox currently shows (not a per-row toggle - the checkbox is
        the value being applied, matching "Assign to selected" above)."""
        selected = self.stagetree.selection()
        if not selected:
            messagebox.showinfo("Fast approach", "Select one or more stages first.")
            return
        fast = self.fastapproachvar.get()
        for item in selected:
            index = self.stagetree.index(item)
            if fast:
                self.settings["stages"][index]["fast_approach"] = True
            else:
                self.settings["stages"][index].pop("fast_approach", None)
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
        self.programbox["values"] = sorted(self.settings["stage_programs"].keys())

    def saveprogram(self):
        name = self.programbox.get().strip()
        if not name:
            messagebox.showerror("Save program", "Give the program a name in the 'Saved programs' box.")
            return
        if not self.settings["stages"]:
            messagebox.showerror("Save program", "Add at least one stage first.")
            return
        self.settings["stage_programs"][name] = copy.deepcopy(self.settings["stages"])
        self.store.save(self.settings)
        self.refreshprograms()
        self.setstatus("Saved stage program '" + name + "'")

    def loadprogram(self):
        name = self.programbox.get().strip()
        program = self.settings["stage_programs"].get(name)
        if program is None:
            messagebox.showerror("Load program", "No saved program named '" + name + "'.")
            return
        self.settings["stages"] = copy.deepcopy(program)
        self.store.save(self.settings)
        self.refreshstages()
        self.setstatus("Loaded stage program '" + name + "'")

    def deleteprogram(self):
        name = self.programbox.get().strip()
        if name not in self.settings["stage_programs"]:
            return
        del self.settings["stage_programs"][name]
        self.store.save(self.settings)
        self.refreshprograms()
        self.programbox.set("")
        self.setstatus("Deleted stage program '" + name + "'")

    # ------------------------------------------------------------------
    # PID profiles
    # ------------------------------------------------------------------
    def buildpidsection(self, parent):
        frame = ttk.LabelFrame(parent, text="PID tunings (named profiles)", padding=8)
        frame.grid(row=1, column=0, sticky="ew", pady=4)
        ttk.Label(frame, text="Profile:").grid(row=0, column=0)
        self.profilebox = ttk.Combobox(frame, width=24)
        self.profilebox.grid(row=0, column=1, columnspan=3, sticky="w", padx=4)
        self.profilebox.bind("<<ComboboxSelected>>", self.onprofileselected)

        self.pidentries = {}
        for column, name in enumerate(("Kp", "Ki", "Kd")):
            ttk.Label(frame, text=name + ":").grid(row=1, column=column * 2)
            entry = ttk.Entry(frame, width=10)
            entry.grid(row=1, column=column * 2 + 1, padx=4, pady=4)
            self.pidentries[name] = entry
        ttk.Button(frame, text="Save profile", command=self.saveprofile).grid(row=2, column=1, pady=4)
        ttk.Button(frame, text="Delete profile", command=self.deleteprofile).grid(row=2, column=3, pady=4)
        self.applylivebutton = ttk.Button(frame, text="Apply now (live)", state="disabled",
                                          command=self.applylivetunings)
        self.applylivebutton.grid(row=2, column=4, pady=4)

    def refreshprofiles(self):
        names = sorted(self.settings["pid_profiles"].keys())
        self.profilebox["values"] = names
        self.stageprofilebox["values"] = [""] + names
        active = self.settings.get("active_profile")
        if active not in self.settings["pid_profiles"]:
            active = names[0] if names else ""
            self.settings["active_profile"] = active
        self.profilebox.set(active)
        self.loadprofileentries(active)

    def loadprofileentries(self, name):
        profile = self.settings["pid_profiles"].get(name)
        if profile is None:
            return
        for key, entry in self.pidentries.items():
            entry.delete(0, "end")
            entry.insert(0, formatpid(profile[key]))

    def onprofileselected(self, event=None):
        name = self.profilebox.get()
        self.settings["active_profile"] = name
        self.loadprofileentries(name)
        self.store.save(self.settings)

    def currenttunings(self):
        return tuple(float(self.pidentries[k].get()) for k in ("Kp", "Ki", "Kd"))

    def saveprofile(self):
        name = self.profilebox.get().strip()
        if not name:
            messagebox.showerror("Save profile", "Give the profile a name in the Profile box.")
            return
        try:
            kp, ki, kd = self.currenttunings()
        except ValueError:
            messagebox.showerror("Save profile", "Kp, Ki and Kd must be numbers.")
            return
        self.settings["pid_profiles"][name] = {"Kp": kp, "Ki": ki, "Kd": kd}
        self.settings["active_profile"] = name
        self.store.save(self.settings)
        self.refreshprofiles()
        self.setstatus("Saved profile '" + name + "'")

    def deleteprofile(self):
        name = self.profilebox.get().strip()
        if name not in self.settings["pid_profiles"]:
            return
        if len(self.settings["pid_profiles"]) == 1:
            messagebox.showerror("Delete profile", "Cannot delete the last profile.")
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
            messagebox.showerror("Apply now", "Kp, Ki and Kd must be numbers.")
            return
        self.fermenter.setconsPIDvalues(kp, ki, kd)
        self.setstatus("Applied live tunings: Kp=" + str(kp) + " Ki=" + str(ki) + " Kd=" + str(kd))

    def saveautotuneresult(self, label, result):
        """Store an autotune result as a named profile (called when it finishes)."""
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
    def buildautotunesection(self, parent):
        frame = ttk.LabelFrame(parent, text="Autotune (relay method - use water, not milk)", padding=8)
        frame.grid(row=2, column=0, sticky="ew", pady=4)
        ttk.Label(frame, text="Target (C):").grid(row=0, column=0)
        self.autotunetargetentry = ttk.Entry(frame, width=7)
        self.autotunetargetentry.insert(0, "40")
        self.autotunetargetentry.grid(row=0, column=1, padx=4)
        ttk.Label(frame, text="Margin (C):").grid(row=0, column=2)
        self.autotunemarginentry = ttk.Entry(frame, width=5)
        self.autotunemarginentry.insert(0, "12")
        self.autotunemarginentry.grid(row=0, column=3, padx=4)
        ttk.Label(frame, text="Save as profile:").grid(row=0, column=4)
        self.autotunelabelentry = ttk.Entry(frame, width=18)
        self.autotunelabelentry.grid(row=0, column=5, padx=4)
        self.autotunebutton = ttk.Button(frame, text="Start autotune", command=self.startautotune)
        self.autotunebutton.grid(row=0, column=6, padx=4)
        self.autotuneherebutton = ttk.Button(frame, text="Autotune here (resume after)",
                                             state="disabled", command=self.autotuneinplace)
        self.autotuneherebutton.grid(row=1, column=0, columnspan=7, pady=(4, 0))

    def autotunemargin(self):
        try:
            return float(self.autotunemarginentry.get())
        except ValueError:
            return 12.0

    def startautotune(self):
        if self.running:
            return
        try:
            target = float(self.autotunetargetentry.get())
        except ValueError:
            messagebox.showerror("Autotune", "Target temperature must be a number.")
            return
        if not 20 <= target <= 90:
            messagebox.showerror("Autotune", "Target must be between 20 and 90 C.")
            return
        label = self.autotunelabelentry.get().strip()
        if not label:
            label = "autotune-" + str(target) + "C"
        self.runfermenter("Autotuning at " + str(target) + " C (profile '" + label + "')...",
                          mode='relayautotune', autotunetarget=target,
                          autotunesafetymargin=self.autotunemargin(),
                          onautotunedone=lambda result: self.saveautotuneresult(label, result))

    def autotuneinplace(self):
        """Retune without formally stopping the ferment: stop the current
        program run (heater keeps its last setpoint the whole time, same as
        "Stop (keep heating)"), run the relay autotune at whatever the
        program was actually holding right now, then automatically resume
        the program from its current stage onward using the new tunings.

        Only meaningful while a staged program is running - autotuning
        itself has no "current stage" to resume into.
        """
        if not self.running or self.fermenter is None or self.fermenter.mode != 'pidprogram':
            messagebox.showinfo("Autotune here",
                                "Start the program first - this retunes at whatever it's "
                                "currently holding, then resumes it automatically.")
            return
        # SP (what we've commanded), not currentSP (what the ESP has echoed
        # back so far) - SP updates immediately when the program sets a
        # stage target, currentSP only once a round-trip confirms it, which
        # would make this depend on ESP echo timing for no good reason.
        target = self.fermenter.SP
        stageindex = self.fermenter.pidprogram.currentstage
        resumestages = list(self.settings["stages"][stageindex:]) or list(self.settings["stages"][-1:])
        label = self.autotunelabelentry.get().strip() or ("inplace-" + str(target) + "C")

        self.setstatus("Stopping the program to autotune in place at " + str(target) + " C...")
        self.autotuneherebutton["state"] = "disabled"
        self.stop(heateroff=False)
        self._inplacepending = (target, resumestages, label)
        self.root.after(300, self._inplacewaitthenautotune)

    def _inplacewaitthenautotune(self):
        if self.running:
            # The previous run hasn't finished unwinding yet - starting a new
            # one now would race with it (see requestgraphrefresh()'s
            # docstring for why nested/racing calls here are dangerous).
            self.root.after(200, self._inplacewaitthenautotune)
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
        # The autotune keeps holding its target indefinitely once done; stop
        # it the same way, then wait for it to unwind before resuming.
        self.stop(heateroff=False)
        self.root.after(300, self._inplacewaitthenresume)

    def _inplacewaitthenresume(self):
        if self.running:
            self.root.after(200, self._inplacewaitthenresume)
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
    def buildrunsection(self, parent):
        frame = ttk.Frame(parent, padding=8)
        frame.grid(row=3, column=0, sticky="ew", pady=4)
        self.startbutton = ttk.Button(frame, text="Start program", command=self.startprogram)
        self.startbutton.grid(row=0, column=0, padx=4)
        self.stopbutton = ttk.Button(frame, text="Stop (keep heating)", state="disabled",
                                     command=lambda: self.stop(heateroff=False))
        self.stopbutton.grid(row=0, column=1, padx=4)
        self.stopoffbutton = ttk.Button(frame, text="Stop & heater off", state="disabled",
                                        command=lambda: self.stop(heateroff=True))
        self.stopoffbutton.grid(row=0, column=2, padx=4)
        self.refreshgraphbutton = ttk.Button(frame, text="Refresh graph", state="disabled",
                                             command=self.refreshgraph)
        self.refreshgraphbutton.grid(row=0, column=3, padx=4)
        self.statusvar = tk.StringVar(value="Idle")
        ttk.Label(frame, textvariable=self.statusvar).grid(row=1, column=0, columnspan=4, sticky="w", pady=4)

    def setstatus(self, text):
        self.statusvar.set(text)

    def refreshgraph(self):
        if self.fermenter is None:
            return
        # Not a direct recreategraph() call: see requestgraphrefresh()'s
        # docstring - doing the actual work from inside this nested button
        # callback is what a real run showed can hang the whole process.
        self.fermenter.requestgraphrefresh()

    def startprogram(self):
        if self.running:
            return
        if not self.settings["stages"]:
            messagebox.showerror("Start program", "Add at least one stage first.")
            return
        try:
            tunings = self.currenttunings()
        except ValueError:
            messagebox.showerror("Start program", "Kp, Ki and Kd must be numbers.")
            return
        self.store.save(self.settings)
        stages = list(self.settings["stages"])
        self.runfermenter("Program running (profile '" + self.profilebox.get() + "')...",
                          mode='pidprogram', stages=stages,
                          tunings=tunings, stagetunings=self.resolvestagetunings(stages))

    def runfermenter(self, statustext, **kwargs):
        self.running = True
        self.startbutton["state"] = "disabled"
        self.autotunebutton["state"] = "disabled"
        self.stopbutton["state"] = "normal"
        self.stopoffbutton["state"] = "normal"
        self.refreshgraphbutton["state"] = "normal"
        # Only meaningful while a staged program (not an autotune) is active.
        self.autotuneherebutton["state"] = "normal" if kwargs.get('mode') == 'pidprogram' else "disabled"
        self.applylivebutton["state"] = "normal" if kwargs.get('mode') == 'pidprogram' else "disabled"
        self.setstatus(statustext)
        try:
            self.fermenter = YogourtFermenter(ontick=self.ontick, autorun=False, **kwargs)
            self.fermenter.listeningloop()  # blocks; ontick keeps the GUI alive
        except SingleInstanceError as e:
            messagebox.showerror("Another instance is already running", str(e))
        except Exception as e:
            messagebox.showerror("Fermenter stopped", str(e))
        finally:
            self.running = False
            self.fermenter = None
            if not self.closing:
                self.startbutton["state"] = "normal"
                self.autotunebutton["state"] = "normal"
                self.stopbutton["state"] = "disabled"
                self.stopoffbutton["state"] = "disabled"
                self.refreshgraphbutton["state"] = "disabled"
                self.autotuneherebutton["state"] = "disabled"
                self.applylivebutton["state"] = "disabled"
                self.setstatus("Stopped. The MCU keeps its last setpoint unless you used 'Stop & heater off'.")
                self.progressvar.set("Idle - nothing running")
        if self.closing:
            self.root.destroy()

    # ------------------------------------------------------------------
    # Progress display
    # ------------------------------------------------------------------
    def buildprogresssection(self, parent):
        frame = ttk.LabelFrame(parent, text="Progress", padding=8)
        frame.grid(row=4, column=0, sticky="ew", pady=4)
        self.progressvar = tk.StringVar(value="Idle - nothing running")
        ttk.Label(frame, textvariable=self.progressvar, justify="left").grid(row=0, column=0, sticky="w")

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
            self.progressvar.set(text)
        elif fermenter.mode == 'relayautotune' and hasattr(fermenter, 'relayautotune'):
            progress = fermenter.relayautotune.getprogress()
            text = ("Autotune at " + str(progress["target"]) + " C: " + progress["phase"]
                   + "  -  cycle " + str(progress["cycles_done"]) + "/" + str(progress["cycles_needed"])
                   + "\nElapsed: " + formatduration(progress["elapsed_s"]))
            self.progressvar.set(text)

    def ontick(self):
        if self.closing:
            if self.fermenter is not None:
                self.fermenter.stoprequested = True
            return
        self.updateprogress()
        self.root.update()

    def stop(self, heateroff=False):
        if self.fermenter is None:
            return
        if heateroff:
            self.fermenter.setSP(1)
            self.setstatus("Heater off requested, stopping...")
        self.fermenter.stoprequested = True

    def onclose(self):
        self.closing = True
        if not self.running:
            self.root.destroy()
        # If running, ontick() will stop the loop and runfermenter() destroys
        # the window on the way out.


if __name__ == '__main__':
    gui = YogurtGUI()
    gui.root.mainloop()
