import copy
import json
import os


DEFAULTSETTINGS = {
    # Each stage: heat to `temperature`, then hold for `duration_minutes`
    # after the temperature is first reached.
    "stages": [
        {"temperature": 39.0, "duration_minutes": 24 * 60},
    ],
    # Named PID tunings. "medium-pot-5-jars" is the hand tuning that was
    # hardcoded in PIDProgram.py.
    "pid_profiles": {
        "medium-pot-5-jars": {"Kp": 20.0, "Ki": 0.02, "Kd": 0.4},
    },
    "active_profile": "medium-pot-5-jars",
}


class SettingsStore:
    """Loads/saves the GUI settings (program stages and PID profiles).

    Default location is yogurt_settings.json in the project root; the
    YOGURT_SETTINGS_FILE environment variable (or the constructor argument)
    overrides it, which the automated tests use to stay isolated.
    """

    def __init__(self, path=None):
        if path is None:
            path = os.environ.get('YOGURT_SETTINGS_FILE')
        if path is None:
            path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "yogurt_settings.json")
        self.path = path

    def load(self):
        settings = copy.deepcopy(DEFAULTSETTINGS)
        try:
            with open(self.path) as f:
                stored = json.load(f)
            if not isinstance(stored, dict):
                raise ValueError("settings root must be an object")
            settings.update(stored)
        except FileNotFoundError:
            pass
        except (OSError, ValueError) as e:
            print("Could not read settings from", self.path, "- using defaults:", e)
        return settings

    def save(self, settings):
        tmppath = self.path + ".tmp"
        with open(tmppath, 'w') as f:
            json.dump(settings, f, indent=2)
        os.replace(tmppath, self.path)
