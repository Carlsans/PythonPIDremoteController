#!/usr/bin/env bash
# Launch the PyQt fermenter GUI as a NATIVE WAYLAND client (no XWayland).
#
# Why: on a Wayland desktop (e.g. niri) the normal venv's PyQt5 has no
# Wayland platform plugin, so it runs through XWayland (xwayland-satellite),
# which stops presenting the window when it is left off a visible workspace
# or on an idle-blanked output - the window "freezes" while the process
# stays perfectly healthy (see README, "long-run freeze, part 3").
#
# This uses ./venvwayland instead - a --system-site-packages venv on the
# system Python, so it picks up the SYSTEM Qt (which DOES ship the Wayland
# platform plugin) plus system numpy/matplotlib, with pyqtgraph + chime
# pip-installed on top. QT_QPA_PLATFORM=wayland then runs it as a real
# Wayland client, so the XWayland presentation freeze cannot happen.
#
# Falls back to xcb (XWayland) automatically if no Wayland display is
# present (e.g. run over SSH or on an X session).
cd "$(dirname "$0")" || exit 1
# Force native Wayland (fall back to xcb only if there is no Wayland
# display). This is set UNCONDITIONALLY on purpose: many Wayland sessions
# export QT_QPA_PLATFORM=xcb globally as a compat default, and honouring
# that here is exactly what sends this GUI back through XWayland and into
# the presentation freeze. Override it with YOGURT_GUI_PLATFORM if you
# really need a different platform for this app specifically.
export QT_QPA_PLATFORM="${YOGURT_GUI_PLATFORM:-wayland;xcb}"
exec env PYTHONPATH=. ./venvwayland/bin/python -m src.YogurtGUIQt "$@"
