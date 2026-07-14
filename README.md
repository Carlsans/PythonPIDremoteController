This program is intended to be used in conjunction with this codebase :
https://github.com/Carlsans/ESP8266RemotePIDController
You need to setup your mcu first and then run the python program to access your mcu.
They both talk to each other through web serial.
Run yogurtdata.py

## GUI

The control panel starts, configures and stops everything (run from the project root):

    PYTHONPATH=. python -m src.YogurtGUI

- **Program stages**: add/remove stages (heat to a temperature, then hold it for a
  duration). Example for Greek yogurt: 82 C held 10 min (texture + sanitizing),
  then 40 C held 480 min. Stages are saved to `yogurt_settings.json`.
  The current stage list can be saved under a name (e.g. "greek-yogurt") and
  restored later with the Save/Load/Delete buttons.
- **PID tunings**: named profiles (save/select/delete). The selected profile is
  applied when the program starts.
- **Autotune**: enter a target temperature and a profile name, press Start
  autotune. When it finishes, the tunings are saved under that name and the pot
  keeps holding the target. Use water, not milk.
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

## Tests

Automated tests run against a simulated pot / fake ESP on localhost and never touch
the real device:

    ./venvarch/bin/python tests/test_relayautotune.py
    ./venvarch/bin/python tests/test_connection_robustness.py
    ./venvarch/bin/python tests/test_settings_and_program.py
    ./venvarch/bin/python tests/test_gui.py
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
