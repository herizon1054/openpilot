from dataclasses import dataclass
import pyray as rl

from opendbc.car.structs import car


@dataclass
class RadarElement:
  trackId: int  # uint

  dRel: float
  yRel: float
  vRel: float

  color: rl.Color


class RadarData:
  def __init__(self):
    self.Points: list[RadarElement] = []

  def update(self, radar: 'car.RadarData.RadarPoint') -> list[RadarElement]:
    self.Points = []

    for point in radar:

      # 根據距離決定顏色
      color = self.radar_point_color(point.dRel, point.yRel)

      # 初始化 RadarElement
      radar_element = RadarElement(
        trackId=point.trackId,
        dRel=point.dRel,
        yRel=-point.yRel,
        vRel=point.vRel,
        color=color,
      )

      self.Points.append(radar_element)

    return self.Points

  def radar_point_color(self, dRel: float, yRel: float) -> rl.Color:
    abs_y = abs(yRel)

    # --------------------------------------------------
    # 1️⃣ 危險程度 t（0=綠 → 0.5=黃 → 1=紅）
    # --------------------------------------------------
    if dRel > 50 and abs_y > 3:
      t = 0.0
    elif dRel > 50 and abs_y <= 3:
      t = 0.33
    elif dRel < 10 or abs_y < 2:
      t = 1.0
    else:
      t = 0.66

    # --------------------------------------------------
    # 2️⃣ 顏色漸變（直接算，不用 helper）
    # --------------------------------------------------
    if t <= 0.5:
      # 綠 → 黃
      r = int(255 * (t * 2))
      g = 255
      b = 0
    else:
      # 黃 → 紅
      r = 255
      g = int(255 * (1 - (t - 0.5) * 2))
      b = 0

    # --------------------------------------------------
    # 3️⃣ 透明度漸變（40% ~ 80%）
    # --------------------------------------------------
    d = max(0.0, min(dRel, 50.0))
    alpha = int(255 * (0.8 - (d / 50.0) * 0.4))

    return rl.Color(r, g, b, alpha)
