This program is intended to be used in conjunction with this codebase :
https://github.com/Carlsans/ESP8266RemotePIDController
You need to setup your mcu first and then run the python program to access your mcu.
They both talk to each other through web serial.
Run yogurtdata.py

## GUI

Two control panels exist with the same feature set; pick one:

- **Tkinter + matplotlib** (`src/YogurtGUI.py`) - the original, no extra
  dependencies beyond what's already installed.
- **PyQt5 + pyqtgraph** (`src/YogurtGUIQt.py`) - a faster-drawing graph and a
  single GUI toolkit end to end (the Tkinter version's graph runs through
  matplotlib's own window management, which is a second, independent Tk
  interpreter in the same process - see "Diagnosing a frozen graph" below).
  Needs `pyqtgraph` (`./venvarch/bin/python -m pip install pyqtgraph`; PyQt5
  itself is already a dependency of matplotlib's Qt backend).

Run from the project root:

    PYTHONPATH=. python -m src.YogurtGUI
    PYTHONPATH=. python -m src.YogurtGUIQt

### Running the Qt GUI natively on Wayland (recommended on a Wayland desktop)

On a Wayland desktop (e.g. niri) the normal venv's PyQt5 has no Wayland
platform plugin, so `src.YogurtGUIQt` runs through **XWayland** - which on
this setup (niri + `xwayland-satellite`) stops presenting the window when it
is left off a visible workspace or on an idle-blanked output, so the window
"freezes" while the process stays perfectly healthy (see "long-run freeze,
part 3" below). Run it as a **native Wayland** client instead:

    ./run_gui_wayland.sh

That launcher uses a separate `venvwayland` virtualenv - a
`--system-site-packages` venv on the system Python, so it picks up the
**system Qt** (which *does* ship the Wayland platform plugin) plus system
numpy/matplotlib, with `pyqtgraph` and `chime` pip-installed on top. It
forces `QT_QPA_PLATFORM=wayland` (falling back to `xcb` if there is no
Wayland display), overriding any global `QT_QPA_PLATFORM=xcb` the session
sets. To (re)create the venv:

    /usr/bin/python3 -m venv --system-site-packages venvwayland
    ./venvwayland/bin/pip install pyqtgraph chime

Verify it is running natively: while it is up, `xdotool search --name
"Yogurt Fermenter"` finds **no** window (a native Wayland window is not an X
window), and the process' `QApplication.platformName()` is `wayland`.

Both scale for HiDPI/4K screens: `YOGURT_UI_FONT_PT` sets the base font
size applied everywhere (default `13`, vs. the ~9pt Tk/Qt/matplotlib
default that reads as genuinely tiny on a 4K panel), and widget widths /
figure size scale with it automatically. `YOGURT_UI_SCALE` (Qt GUI only)
overrides Qt's own DPI auto-detection directly (`QT_SCALE_FACTOR`) for
setups where the X server/compositor misreports the screen's real DPI -
try this first if things still look wrong at the default font size. The Qt
GUI's graph also got a visual pass: a dark theme, a distinguishable color
per line, a filled temperature curve, a dashed setpoint reference line, and
a second Y axis for Output so it no longer fights the temperature curve's
scale.

- **Program stages**: add/remove stages (heat to a temperature, then hold it for a
  duration). Example for Greek yogurt: 82 C held 10 min (texture + sanitizing),
  then 40 C held 480 min. Stages are saved to `yogurt_settings.json`.
  The current stage list can be saved under a name (e.g. "greek-yogurt") and
  restored later with the Save/Load/Delete buttons.
- **Per-stage PID profile**: each stage can use its own named PID profile
  (e.g. a stiff one for an 82 C sanitizing stage, a gentle one for a 38 C
  hold) - pick a profile in the "Stage's PID profile" box and click "Assign
  to selected" on one or more selected stage rows. Leaving it blank keeps a
  stage on whatever profile is selected below at program start (the
  original, single-profile-for-the-whole-run behaviour).
- **Fast approach**: a stage can skip the gentle setpoint ramp (see "Note on
  approaches" below) entirely - check "Fast approach (allow overshoot)" and
  click "Toggle for selected" on one or more stage rows. The full target is
  sent immediately instead of being capped to trickle in behind the
  measured temperature, so the firmware's aggressive full-power profile
  drives the whole climb - faster, at the cost of the stored-heat overshoot
  the ramp exists to prevent. Good for a stage where a few degrees of
  overshoot are harmless (e.g. an 82 C sanitizing hold) but not for one
  holding near a live, temperature-sensitive culture. Off by default.
- **PID tunings**: named profiles (save/select/delete). The selected profile is
  applied when the program starts. "Apply now (live)" pushes whatever is
  currently typed in the Kp/Ki/Kd boxes straight to the running controller
  without stopping the program or waiting for the next stage change - for
  tweaking a profile by hand while watching the graph.
- **Autotune**: enter a target temperature and a profile name, press Start
  autotune. When it finishes, the tunings are saved under that name and the pot
  keeps holding the target. Use water, not milk. "Margin (C)" bounds how far
  above target the relay's "heater on" setpoint can swing (default 12 C);
  tighten it (e.g. 5-7) when autotuning near a live, temperature-sensitive
  culture rather than plain water.
- **Autotune here (resume after)**: retune without stopping the ferment for
  good - available while the staged program is running. It stops the current
  run (the MCU keeps its last setpoint the whole time, same as "Stop (keep
  heating)"), runs the relay autotune at whatever temperature the program was
  actually holding, saves the result as a profile named `inplace-<temp>C`,
  and automatically resumes the program from its current stage onward using
  the new tunings. Note the relay method still needs to swing the output
  fully on/off to measure anything (the firmware has no direct duty-cycle
  control, only setpoint/tunings/an on-off override), so the temperature will
  oscillate by a few degrees for the ~10-30 min the autotune takes - this is
  for fixing tunings that are visibly wrong (e.g. persistent oscillation from
  a profile tuned at a very different temperature or in a different medium),
  not something to run routinely.
- **Start program / Stop**: "Stop (keep heating)" leaves the MCU holding its last
  setpoint (the MCU is autonomous); "Stop & heater off" sends SetSP(1) first.
- **Refresh graph**: requests the graph window be closed and reopened; the
  actual work happens on the next tick from a safe context, not synchronously
  from the button click (see "Only one instance at a time" below for why).
  Automatic periodic refresh is off by default (see
  `YOGURT_GRAPH_REFRESH_SECONDS` below) since it raises/refocuses the window,
  which is disruptive on its own.

## Only one instance at a time

Starting a second `yogurtdata.py`/GUI process against the same ESP (port) is
now refused with a clear error instead of silently succeeding. This was added
after a real overnight run showed two independent processes running for
*hours* against the same ESP: both had bound the same UDP port (Python's
`SO_REUSEADDR` allows that), so the kernel silently delivered each incoming
packet to only one of them at a time - one process kept controlling the pot
normally while the other sat frozen on stale data, and both were sending
their own `SetSP`/`SetTunings` commands to the real ESP the whole time. This
went unnoticed because neither process actually crashed; a window that looks
frozen (see "Diagnosing a frozen graph" below) is not the same as a dead
process, and starting a fresh instance without confirming the old one is
really gone leaves it running in the background indefinitely.

If you ever see "Another instance is already running" and you're sure
nothing legitimate is using it, find and close it first:

    ps aux | grep -i yogurt

Also: `applyProgram()`, `checkconnection()`, `relayautotune.update()`, and the
GUI's `ontick()` are now wrapped so a bug in any of them can no longer take
the whole run down silently (they used to run unprotected). If `ontick()`
keeps failing (e.g. the GUI window is gone), the loop stops itself after 10
consecutive failures rather than running invisibly forever.

## Modes

The mode is selected with the `YOGURT_MODE` environment variable (default: `pidprogram`):

- `pidprogram` - runs the fermentation temperature program (PIDProgram.py).
- `relayautotune` - **recommended autotuner**: relay-feedback (Astrom-Hagglund) autotune.
  Makes the pot oscillate around the target temperature for ~6 cycles, then computes
  and applies PID tunings (Ziegler-Nichols PI by default - validated against the
  real pot; other variants are printed and appended to `autotune_results.txt`,
  and the firmware's 160/255 output clamp is measured automatically). Target temperature
  comes from `YOGURT_AUTOTUNE_TARGET` (default 40). Do this with a pot of water,
  not with milk you care about.

Example:

    YOGURT_MODE=relayautotune YOGURT_AUTOTUNE_TARGET=40 python src/yogurtdata.py

## Configuration environment variables

- `YOGURT_ESP_IP` - ESP8266 address (default `192.168.0.4`, see NetworkConfiguration.py)
- `YOGURT_ESP_PORT` - port the ESP listens on (default `5000`)
- `YOGURT_LISTEN_PORT` - port this program listens on (default: same as `YOGURT_ESP_PORT`)
- `YOGURT_CONNECTION_TIMEOUT` - seconds without any packet before the UDP socket is
  recreated (default `30`)
- `YOGURT_GRAPH_REFRESH_SECONDS` - if set above `0`, the graph window is
  proactively closed and reopened this often (default `0`, disabled). Some
  window managers/compositors can in theory silently stop delivering redraws
  to a long-lived window without any error, which this would bound - but it
  also raises/refocuses the window on every reload, which is disruptive on
  its own, so it is off unless you set it. Use the GUI's "Refresh graph"
  button (or `fermenter.recreategraph()`) to do it manually instead.
- `YOGURT_GRAPH_DIAG_LOG` - path to the graph diagnostics log (default
  `graph_diagnostics.log` in the working directory; empty string disables it)
- `YOGURT_GRAPH_DIAG_HEARTBEAT_SECONDS` - how often a heartbeat line is
  written (default `300`)
- `YOGURT_GRAPH_DIAG_SLOW_MS` - a single redraw call taking longer than this
  is logged immediately (default `250`)

## Diagnosing a frozen graph

Every run writes `graph_diagnostics.log`: a `GRAPH_CREATED` line at startup
and after every proactive refresh (with the matplotlib backend and Qt
platform in use), a `HEARTBEAT` line every 5 minutes (temperature, setpoint,
redraw count/timing, point count, `fig_stale`), and an immediate
`SLOW_REDRAW` line if any single redraw call takes more than 250 ms. This
distinguishes two very different failure modes: a slow/hung redraw *call*
(a real Python-level problem, would show as `SLOW_REDRAW` or a gap in
heartbeats) versus the window silently no longer being repainted on screen
while Python sees nothing wrong (no anomaly in the log at all - the redraw
calls keep returning normally and quickly, they just aren't reaching the
screen). If the graph looks frozen during a run, **note the wall-clock time**
and check the log afterwards: heartbeats with fast, steady `max_redraw_ms`
around that time mean the freeze was invisible to the process (a
window-manager/compositor-side issue - the proactive refresh should recover
it within `YOGURT_GRAPH_REFRESH_SECONDS`), while a gap in heartbeats or a
`SLOW_REDRAW`/exception would point at a real in-process hang instead.

### The Qt GUI's long-run freeze (fixed)

A third failure mode, found the hard way on a real multi-hour run of the Qt
GUI: neither a hung redraw call nor a compositor problem, but the redraw
being *legitimately* slow enough to starve the event loop. `updateplots()`
redraws every curve once a second over the **whole** run history, so its
cost grew with run length - measured at this window size, a full
update+render took ~0.15 s at 1 h of data but ~4.2 s at the 20 h cap
(`cleanPlotlists()`). Once one redraw takes longer than the one-second
redraw interval, the loop does nothing but paint and the window stops
responding to input - indistinguishable from a crash from the outside,
which is exactly how it was first reported.

Two causes, both needed fixing (measured with `tests/test_gui_qt.py`'s
`test_plot_redraw_stays_fast_on_long_runs`, and end-to-end via event-loop
lag against a fake ESP - median lag went from **2580 ms to 1 ms**):

- **No downsampling**: every one of ~72 000 points was handed to Qt to
  stroke, most landing on the same screen pixel. Now
  `setDownsampling(auto=True, method='peak')` + `setClipToView(True)`: the
  min/max envelope per pixel is kept (spikes stay visible) and off-screen
  points are skipped.
- **Antialiasing on wide pens**: Qt's raster engine has no fast path for
  antialiased pens wider than 1 px - ~40 ms per dense curve per redraw,
  across 7 curves. Antialiasing is now off globally for the graph; the
  100-point zoom curve opts back in (`antialias=True`), where it costs
  nothing and the smoothing is actually visible.

`updateplots()` also logs a `QT_SLOW_PLOT` line to the diagnostics log if it
ever exceeds 250 ms again, so a future regression leaves evidence instead of
just looking like another mystery freeze.

### The Qt GUI's long-run freeze, part 2: the blocking event loop (fixed)

The redraw fix above was necessary but not sufficient - the window still
froze on a multi-day run. This time a `py-spy dump` of the live frozen
instance was decisive: the main thread was parked in
`listeningloop()`'s `recvfrom()`, with Qt's real event loop (`app.exec_`)
suspended on the stack behind it. The Qt GUI had inherited the Tkinter
design where the fermenter *blocks* in `listeningloop()` and the GUI is
repainted only by `QApplication.processEvents()` calls hand-cranked from
inside that loop (once per socket read). That manual pump is fragile over
long runs: the window silently stops repainting while the control loop
keeps cycling, so from outside it looks like a crash even though the
process is healthy and low-CPU (~10%, mostly asleep in `poll` - nothing
like the CPU-pegged redraw-starvation above).

The fix is architectural: the Qt GUI no longer blocks. `runfermenter()`
creates the fermenter with `autorun=False`, then drives it from a `QTimer`
(`_pumpfermenter`, 10 Hz) while Qt's own `app.exec_()` stays in charge of
painting the normal, robust way. `listeningloop()` was factored into
`steponce()` / `receiveone()` / `drainincoming()` / `closelistening()` so
the blocking version still serves the Tkinter GUI and headless mode
unchanged, while the Qt pump calls `steponce(blocking=False,
runontick=False)` and lets Qt paint natively. Because the pump is timer-
driven, the window now stays responsive **regardless of packet arrival** -
verified on a real display with 20 h of history while the fake ESP goes
silent mid-run (event-loop lag stayed at ~0 ms median). Regression test:
`test_event_loop_stays_responsive_while_running`. The pump also writes a
`QT_PUMP_HEARTBEAT` line to the diagnostics log every 60 s, so if the
window ever freezes again the log immediately distinguishes "timer still
firing" (a genuine compositor/WM-side freeze) from "pump wedged".

### The Qt GUI's long-run freeze, part 3: XWayland presentation (mitigated)

Even after part 2 the window still froze after a couple of hours - and this
time the `QT_PUMP_HEARTBEAT` line the part-2 fix added settled it
immediately: on the live frozen instance the heartbeats were **still being
written every 60 s** (temperature and point-count advancing normally), and a
`py-spy dump` showed the main thread idle in Qt's event loop. So the process
and the whole control loop were completely healthy - only the on-screen
window was frozen. This is the "compositor silently stops redrawing a
long-lived window" mode the matplotlib GUI already guards against with
`recreategraph` (see above); the Qt GUI just never had an equivalent.

The environment is the reason: the desktop is **niri** (a Wayland
compositor) and the venv's bundled Qt has no native Wayland platform plugin
(only `xcb`), so the GUI runs through **XWayland**, provided by
**`xwayland-satellite`**. When the window is somewhere the compositor stops
compositing it (a non-visible niri workspace, or an idle-blanked output),
frame callbacks stop and the surface can stop presenting new frames and fail
to resume when shown again. It is not reproducible on a short timescale
(monitor DPMS-off, and a nested-niri window hidden for 15 s, both recovered
fine); it takes hours, so it could not be validated by fast reproduction.

Mitigation (`_forcerepaint`, `YOGURT_QT_REPAINT_SECONDS`, default 2 s): a
watchdog `QTimer` calls `self.plotwidget.viewport().repaint()` - a
*synchronous* repaint that does not wait for a frame callback, so the app
commits a fresh buffer every couple of seconds regardless. For the window to
stay frozen the compositor would have to ignore our commits outright, not
merely withhold frame-callback requests (the common XWayland-stall cause).
It runs cleanly through the real XWayland path (verified in a nested niri +
xwayland-satellite) and costs one extra plot repaint every 2 s.

The **definitive** fix is to not go through XWayland at all: run the GUI on
a Qt that has the Wayland platform plugin. `./run_gui_wayland.sh` does this
(system Qt via `venvwayland`, forced `QT_QPA_PLATFORM=wayland` - see
"Running the Qt GUI natively on Wayland" above); a native Wayland window is
not subject to the XWayland presentation stall at all. A newer
`xwayland-satellite` may also fix it. The repaint watchdog stays as the
in-app best effort for anyone still running through XWayland.

## Tests

Automated tests run against a simulated pot / fake ESP on localhost and never touch
the real device:

    ./venvarch/bin/python tests/test_relayautotune.py
    ./venvarch/bin/python tests/test_connection_robustness.py
    ./venvarch/bin/python tests/test_settings_and_program.py
    ./venvarch/bin/python tests/test_gui.py
    ./venvarch/bin/python tests/test_gui_qt.py
    ./venvarch/bin/python tests/test_overshoot_fix.py
    ./venvarch/bin/python tests/test_robustness_hardening.py

`tests/test_graph_refresh.py` needs a real display (it drives the actual
interactive graph window, not the headless Agg backend used by the tests
above) - run it manually when checking graph-refresh behaviour:

    ./venvarch/bin/python tests/test_graph_refresh.py

Two scripts talk to the real device (run them only with water in the pot; both
turn the heater off at the end):

- `tests/real_autotune_82c.py` - autotune at 82 C, saves the 'water-82c' profile.
- `tests/real_program_stage.py` - runs one program stage and measures overshoot
  and hold quality (`YOGURT_TEST_TARGET`, `YOGURT_TEST_HOLD_MINUTES`,
  `YOGURT_TEST_PROFILE` env vars).

Note on tunings: the program writes the selected profile to the firmware's
*conservative* slot (active within 4.5 C of the setpoint) and a no-windup pure-P
profile (100, 0, 0) to the *aggressive* slot used during climbs. Writing the
same PI values to both slots causes several degrees of integral-windup
overshoot when heating to a low setpoint like 38 C.

Note on approaches: the pot stores several degrees' worth of heat in its
element/bottom during a full-power climb, which arrives minutes after the PID
cuts its output - no tuning can see it. PIDProgram therefore ramps the setpoint
over the last 10 C of any upward approach (0.5 C/min, never leading the
measured temperature by more than ~2 C), which lets the stored heat arrive
during the approach instead of after it.
