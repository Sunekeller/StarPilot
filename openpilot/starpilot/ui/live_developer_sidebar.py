import math
from dataclasses import dataclass

from openpilot.starpilot.ui.raylib import rl

from openpilot.starpilot.ui.layout_preview_sidebar import metric_rects
from openpilot.starpilot.ui.presentation import FontRole


@dataclass(frozen=True)
class DeveloperMetric:
  label: str
  value: str


def finite(value):
  try:
    number = float(value)
    return number if math.isfinite(number) else None
  except (TypeError, ValueError, OverflowError):
    return None


class DeveloperMetrics:
  def __init__(self):
    self.drive = None
    self.maximum = None
    self.driver_torque = None  # smoothed column torque sensor reading (raw MDPS units)

  def observe(self, *, drive, car, delay, torque, parameters, metric, cp=None):
    if drive != self.drive:
      self.drive, self.maximum, self.driver_torque = drive, None, None
    # Driver torque: ~1 s running average so a constant offset is readable while driving straight
    raw_torque = finite(getattr(car, 'steeringTorque', None)) if car is not None else None
    if raw_torque is not None:
      self.driver_torque = raw_torque if self.driver_torque is None else self.driver_torque + 0.05 * (raw_torque - self.driver_torque)
    steer_angle = finite(getattr(car, 'steeringAngleDeg', None)) if car is not None else None
    acceleration = finite(getattr(car, 'aEgo', None)) if car is not None else None
    if acceleration is not None and drive is not None and not bool(getattr(car, 'gasPressed', False)):
      self.maximum = max(0., acceleration) if self.maximum is None else max(self.maximum, acceleration)
    unit, conversion = ('m/s²', 1.) if metric else ('ft/s²', 3.280839895)

    def shown(value, decimals=5, suffix=''):
      number = finite(value)
      return '—' if number is None else f'{number:.{decimals}f}{suffix}'

    tune = None
    if cp is not None:
      try:
        tune = cp.lateralTuning.torque if cp.lateralTuning.which() == 'torque' else None
      except (AttributeError, TypeError, ValueError):
        pass
    live_torque = torque is not None and bool(getattr(torque, 'useParams', False))
    return (
      DeveloperMetric('DRIVER TORQUE', shown(None if car is None else self.driver_torque, 1)),
      DeveloperMetric('STEER ANGLE', shown(steer_angle, 1, ' deg')),
      DeveloperMetric('LEARNED DELAY' if delay is not None else 'BASE DELAY',
                      shown(getattr(delay, 'lateralDelay', None) if delay is not None else getattr(cp, 'steerActuatorDelay', None))),
      DeveloperMetric('LEARNED FRICTION' if live_torque else 'BASE FRICTION',
                      shown(getattr(torque, 'frictionCoefficientFiltered', None) if live_torque else getattr(tune, 'friction', None))),
      DeveloperMetric('LEARNED LAT ACCEL' if live_torque else 'BASE LAT ACCEL',
                      shown(getattr(torque, 'latAccelFactorFiltered', None) if live_torque else getattr(tune, 'latAccelFactor', None))),
      DeveloperMetric('STEER RATIO', shown(getattr(parameters, 'steerRatio', None))),
      DeveloperMetric('STEER STIFF', shown(getattr(parameters, 'stiffnessFactor', None))),
    )


def render_sidebar(fonts, frame, metrics):
  rl.draw_rectangle_rec(frame, rl.BLACK)
  for rect, metric in zip(metric_rects(frame, len(metrics)), metrics, strict=True):
    rl.draw_rectangle_rounded_lines_ex(rect, .3, 10, 2, rl.Color(255, 255, 255, 85))
    for line, y in ((metric.label, rect.y + 24), (metric.value, rect.y + 68)):
      size = 30
      measured = fonts.measure(line, FontRole.SEMI_BOLD, size)
      while measured.width > rect.width - 22 and size > 16:
        size -= 1
        measured = fonts.measure(line, FontRole.SEMI_BOLD, size)
      fonts.draw(line, FontRole.SEMI_BOLD, size, rect.x + (rect.width - measured.width) / 2, y, rl.WHITE)
