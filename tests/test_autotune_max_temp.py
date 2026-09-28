"""Regression test for the explicit 'max sensor temp' autotune safety
override in yogurtdata.py.

A relay tune oscillates and some overshoot past target is normal, not a
fault - the old min(95, target+margin) ceiling left little to no headroom
above the relay's own "heater on" setpoint (which is itself capped by the
same maxsafetemp), so ordinary oscillation could trip a safety abort.
autotunemaxtemp (and YOGURT_AUTOTUNE_MAX_TEMP) let a caller set an
absolute ceiling - "95 C is known safe for this pot" - independent of
target and margin.

Run from the project root:
    ./venvarch/bin/python tests/test_autotune_max_temp.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["YOGURT_SILENT"] = "1"
os.environ["MPLBACKEND"] = "Agg"
os.environ["YOGURT_ESP_IP"] = "127.0.0.1"

import matplotlib.pyplot as plt

from src.yogurtdata import YogourtFermenter


def makefermenter(port, **kwargs):
    plt.close('all')
    os.environ["YOGURT_ESP_PORT"] = str(port + 1)
    os.environ["YOGURT_LISTEN_PORT"] = str(port)
    fermenter = YogourtFermenter(mode='relayautotune', autorun=False, showgraph=False, **kwargs)
    return fermenter


def test_default_behaviour_unchanged():
    """No autotunemaxtemp given: falls back to min(95, target+margin), same
    as before this override existed."""
    fermenter = makefermenter(55160, autotunetarget=65.0, autotunesafetymargin=12.0)
    try:
        assert fermenter.relayautotune.maxsafetemp == 65.0 + 12.0
    finally:
        fermenter.closelistening()
    print("OK - default (no override) behaviour is unchanged: min(95, target+margin)")


def test_default_behaviour_still_caps_at_95():
    fermenter = makefermenter(55162, autotunetarget=82.0, autotunesafetymargin=20.0)
    try:
        assert fermenter.relayautotune.maxsafetemp == 95.0, \
            "the default fallback must still cap at 95 C even with a large margin"
    finally:
        fermenter.closelistening()
    print("OK - default fallback still caps at 95 C regardless of margin")


def test_explicit_override_replaces_margin_math_entirely():
    fermenter = makefermenter(55164, autotunetarget=65.0, autotunesafetymargin=12.0, autotunemaxtemp=95.0)
    try:
        assert fermenter.relayautotune.maxsafetemp == 95.0, \
            "an explicit autotunemaxtemp must be used as-is, ignoring the margin"
    finally:
        fermenter.closelistening()
    print("OK - an explicit autotunemaxtemp overrides the margin-based calculation entirely")


def test_env_var_override():
    os.environ["YOGURT_AUTOTUNE_MAX_TEMP"] = "95"
    try:
        fermenter = makefermenter(55166, autotunetarget=65.0)
        try:
            assert fermenter.relayautotune.maxsafetemp == 95.0
        finally:
            fermenter.closelistening()
    finally:
        del os.environ["YOGURT_AUTOTUNE_MAX_TEMP"]
    print("OK - YOGURT_AUTOTUNE_MAX_TEMP env var also overrides the margin-based calculation")


def test_explicit_kwarg_wins_over_env_var():
    os.environ["YOGURT_AUTOTUNE_MAX_TEMP"] = "70"
    try:
        fermenter = makefermenter(55168, autotunetarget=65.0, autotunemaxtemp=95.0)
        try:
            assert fermenter.relayautotune.maxsafetemp == 95.0, \
                "an explicit constructor argument must win over the env var"
        finally:
            fermenter.closelistening()
    finally:
        del os.environ["YOGURT_AUTOTUNE_MAX_TEMP"]
    print("OK - an explicit autotunemaxtemp argument wins over the env var")


def main():
    test_default_behaviour_unchanged()
    test_default_behaviour_still_caps_at_95()
    test_explicit_override_replaces_margin_math_entirely()
    test_env_var_override()
    test_explicit_kwarg_wins_over_env_var()
    print("\nOK - autotune max-sensor-temp override verified.")


if __name__ == '__main__':
    main()
