"""Manual gains for the standard torque controller, kept in /data so updates never touch them.

The file is optional. Each value is validated against LIMITS; anything missing or out of range
falls back to the constant in latcontrol_torque.py. Read once per controller start (next drive).
"""
import json
import math
import os

PATH = os.environ.get("STARPILOT_MANUAL_GAINS_PATH", "/data/starpilot_tune.json")

# name: (minimum, maximum, step)
LIMITS = {
  "KP": (0.2, 1.5, 0.05),
  "KI": (0.0, 1.0, 0.05),
  "JERK_GAIN": (0.0, 0.6, 0.05),
  "FRICTION_THRESHOLD": (0.005, 0.2, 0.005),
  "LAT_DELAY": (0.0, 0.6, 0.01),  # seconds; 0 = use the learned steering delay
  "LAT_OFFSET": (-1.0, 1.0, 0.05),  # m/s^2 added to feedforward; set to 0 after the rear axle repair
}

LABELS = {
  "KP": "Highway P gain",
  "KI": "I gain",
  "JERK_GAIN": "Jerk gain",
  "FRICTION_THRESHOLD": "Friction threshold",
  "LAT_DELAY": "Steering delay (0 = learned)",
  "LAT_OFFSET": "Lateral offset (0 after repair)",
}


def _valid(name: str, value) -> float | None:
  try:
    number = float(value)
  except (TypeError, ValueError):
    return None
  low, high, _ = LIMITS[name]
  if not math.isfinite(number) or not low <= number <= high:
    return None
  return number


def read_raw() -> bytes | None:
  try:
    with open(PATH, "rb") as f:
      return f.read()
  except OSError:
    return None


def load(defaults: dict[str, float]) -> dict[str, float]:
  """Return defaults overlaid with any valid values from the file."""
  values = dict(defaults)
  raw = read_raw()
  if raw is None:
    return values
  try:
    data = json.loads(raw.decode("utf-8"))
  except (ValueError, UnicodeDecodeError):
    return values
  if not isinstance(data, dict):
    return values
  for name in LIMITS:
    number = _valid(name, data.get(name))
    if number is not None:
      values[name] = number
  return values


def save(values: dict[str, float]) -> bytes:
  """Write only valid known values, atomically. Returns the bytes written."""
  clean = {name: number for name in LIMITS if (number := _valid(name, values.get(name))) is not None}
  raw = (json.dumps(clean, indent=2, sort_keys=True) + "\n").encode("utf-8")
  tmp = PATH + ".tmp"
  with open(tmp, "wb") as f:
    f.write(raw)
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, PATH)
  return raw
