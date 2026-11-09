"""Settings rows for the standard torque controller gains stored in /data/starpilot_tune.json."""
from __future__ import annotations

import math

from openpilot.starpilot.lateral import manual_gains
from openpilot.starpilot.ui.feature_settings_state import FeatureRow, FeatureSettingsRequest

PREFIX = "manual_gains:"
REASON = "Standard controller only. Applies next drive. Steering delay 0 = use the learned value."


class ManualGainsFeature:
  def __init__(self, owner):
    self.owner = owner

  def _defaults(self) -> dict[str, float]:
    from openpilot.selfdrive.controls.lib.latcontrol_torque import KP, KI, JERK_GAIN
    return {"KP": KP, "KI": KI, "JERK_GAIN": JERK_GAIN, "FRICTION_THRESHOLD": 0.05, "LAT_DELAY": 0.0, "LAT_OFFSET": 0.0}

  def rows(self, allowed: bool) -> list[FeatureRow]:
    defaults = self._defaults()
    values = manual_gains.load(defaults)
    raw = manual_gains.read_raw()
    rows = []
    for name, (low, high, step) in manual_gains.LIMITS.items():
      rows.append(FeatureRow(PREFIX + name, manual_gains.LABELS[name], f"{values[name]:g}", source=raw,
                             step=step, minimum=low, maximum=high, available=allowed,
                             default_value=f"{defaults[name]:g}", reason=REASON))
    return rows

  def apply(self, request: FeatureSettingsRequest) -> bool:
    name = request.key[len(PREFIX):]
    if name not in manual_gains.LIMITS or not self.owner.authority("torque") or request.direction:
      return False
    if request.expected != manual_gains.read_raw():
      return False
    try:
      number = float(request.value)
    except ValueError:
      return False
    low, high, step = manual_gains.LIMITS[name]
    if not math.isfinite(number) or not low <= number <= high:
      return False
    values = manual_gains.load(self._defaults())
    values[name] = round(round(number / step) * step, 4)
    try:
      manual_gains.save(values)
    except OSError:
      return False
    return True
