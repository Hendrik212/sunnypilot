"""
Onroad tuning panel for dynamic lane centering (comma 3X / tizi layout).

Shown only while LaneCenteringOnroadPanel is on. Collapsed it is an "LC" pill under the
experimental-mode button with a one-line status strip beside it, so what lane centering is
doing (and why, when the clearance guard limits it) is always visible; an amber bar on the
pill's side marks a tracked radar wall. Tapping opens a panel with an on/off button, a live
readout from lateralTuneStateSP (lane position, applied offset, spare clearance per side,
walls) and -/+ rows for LaneCenterOffset and LaneCenteringGain (trim mode) or
LaneCenteringMinClearance (camera mode). The buttons only write Params -- controlsd picks
them up on its next params read (PARAMS_UPDATE_PERIOD) -- and the readout comes from the
published telemetry, so the panel shows what is actually applied.

One Widget covering the whole panel (hit-testing its own sub-rects) so that any touch on it
reports is_pressed and the road view does not also toggle the sidebar.
"""
import time

import pyray as rl

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.lane_centering import LaneCenteringStatus as St
from openpilot.sunnypilot.selfdrive.controls.lib.dynamic_lane_centering import LimitCause as Lc
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

POSITION_NAMES = {0: "--", 1: "SINGLE", 2: "RIGHT", 3: "MIDDLE", 4: "LEFT"}

PILL_W, PILL_H = 192, 110
PANEL_W = 620
ROW_H = 120
BTN_W, BTN_H = 130, 100
PAD = 24
PARAM_REFRESH_S = 1.0
STRIP_GAP = 16
STRIP_FONT = 40
WALL_BAR_W = 12

# param, label, step, min, max, format
OFFSET_KNOB = ("LaneCenterOffset", "Offset", 0.01, 0.0, 0.30, lambda v: f"{v * 100:.0f} cm")
GAIN_KNOB = ("LaneCenteringGain", "Gain", 0.05, 0.0, 0.60, lambda v: f"{v:.2f}")
MARGIN_KNOB = ("LaneCenteringMinClearance", "Margin", 0.05, 0.0, 0.60, lambda v: f"{v * 100:.0f} cm")
ALL_KNOBS = (OFFSET_KNOB, GAIN_KNOB, MARGIN_KNOB)
N_KNOBS = 2

BG = rl.Color(0, 0, 0, 180)
BTN_BG = rl.Color(255, 255, 255, 40)
ON_BG = rl.Color(0x17, 0x86, 0x44, 230)
OFF_BG = rl.Color(0x80, 0x80, 0x80, 160)
WHITE = rl.Color(255, 255, 255, 255)
GREY = rl.Color(200, 200, 200, 255)
AMBER = rl.Color(0xFF, 0xB0, 0x20, 255)
GREEN = rl.Color(0x40, 0xD0, 0x70, 255)

STATUS = {
  St.OFF: ("OFF", GREY),
  St.STANDBY: ("STANDBY", GREY),
  St.PAUSED: ("PAUSED", AMBER),
  St.NO_LINES: ("NO LINES", AMBER),
  St.CENTERED: ("CENTERED", GREEN),
  St.NUDGE_LEFT: ("< NUDGING LEFT", GREEN),
  St.NUDGE_RIGHT: ("NUDGING RIGHT >", GREEN),
  St.SHIFT_LEFT: ("< SHIFTING LEFT", GREEN),
  St.SHIFT_RIGHT: ("SHIFTING RIGHT >", GREEN),
  St.HOLD_LEFT: ("HOLDING LEFT", GREEN),
  St.HOLD_RIGHT: ("HOLDING RIGHT", GREEN),
  St.LIMITED: ("LIMITED", AMBER),
}
ARROW_LEFT = (St.NUDGE_LEFT, St.SHIFT_LEFT)
ARROW_RIGHT = (St.NUDGE_RIGHT, St.SHIFT_RIGHT)
# camera offset mode: no curvature trim, the right-hand readout shows the offset instead
CAMERA_MODE = (St.SHIFT_LEFT, St.SHIFT_RIGHT, St.HOLD_LEFT, St.HOLD_RIGHT, St.LIMITED)

BOUNDARY = {Lc.RIGHT_LINE: ("RIGHT LINE", 1), Lc.LEFT_LINE: ("LEFT LINE", -1),
            Lc.RIGHT_EDGE: ("RIGHT ROAD EDGE", 1), Lc.LEFT_EDGE: ("LEFT ROAD EDGE", -1),
            Lc.RIGHT_WALL: ("RIGHT RADAR WALL", 1), Lc.LEFT_WALL: ("LEFT RADAR WALL", -1)}


def describe(status: St, cause: int, offset: float) -> tuple[str, rl.Color]:
  """One line saying what lane centering is doing right now, and why when it is limited."""
  if status != St.LIMITED:
    name, color = STATUS[status]
    if status in (St.SHIFT_LEFT, St.SHIFT_RIGHT, St.HOLD_LEFT, St.HOLD_RIGHT):
      name += f"  {abs(offset) * 100:.0f} cm"
    return name, color
  try:
    cause = Lc(cause)
  except ValueError:
    return "LIMITED", AMBER
  if cause in BOUNDARY:
    name, side = BOUNDARY[cause]
    away = side * offset < -0.01  # already pushed past the lane centre, away from it
    if cause in (Lc.RIGHT_WALL, Lc.LEFT_WALL) and not away:
      return f"KEEPING {name.split()[0]} WALL DISTANCE", AMBER
    return ("MOVING AWAY FROM " if away else "KEEPING OFF ") + name, AMBER
  if cause == Lc.WALL_NO_SHIFT_RIGHT:
    return "RADAR WALL RIGHT: NO SHIFT", AMBER
  if cause == Lc.WALL_NO_SHIFT_LEFT:
    return "RADAR WALL LEFT: NO SHIFT", AMBER
  if cause == Lc.NARROW:
    return "NARROW LANE: CENTERED", AMBER
  return "LIMITED", AMBER


class LaneCenteringPanel(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._expanded = False
    self._show = False
    self._enabled = False
    self._camera_mode = True
    self._values = {k[0]: k[3] for k in ALL_KNOBS}
    self._last_refresh = 0.0
    self._hit: dict[str, rl.Rectangle] = {}

  def _refresh_params(self) -> None:
    now = time.monotonic()
    if now - self._last_refresh < PARAM_REFRESH_S:
      return
    self._last_refresh = now
    self._show = self._params.get_bool("LaneCenteringOnroadPanel")
    self._enabled = self._params.get_bool("LaneCentering")
    try:
      self._camera_mode = int(self._params.get("LaneCenteringOffsetMode", return_default=True)) == 1
    except (TypeError, ValueError):
      self._camera_mode = True
    for key, _, _, lo, hi, _ in ALL_KNOBS:
      try:
        self._values[key] = min(max(float(self._params.get(key, return_default=True)), lo), hi)
      except (TypeError, ValueError):
        pass

  def _update_state(self) -> None:
    self._refresh_params()
    self.set_visible(self._show and ui_state.started)
    if not self.is_visible:
      self._expanded = False

  @property
  def _knobs(self):
    return (OFFSET_KNOB, MARGIN_KNOB if self._camera_mode else GAIN_KNOB)

  def _status(self) -> tuple[St, str, rl.Color]:
    st = ui_state.sm["lateralTuneStateSP"]
    status = St(st.laneCentreStatus) if st.laneCentreStatus in St._value2member_map_ else St.OFF
    if not self._enabled:
      status = St.OFF
    text, color = describe(status, st.laneCentreLimitCause, st.laneCentreOffset)
    return status, text, color

  def layout_rect(self, hud_rect: rl.Rectangle) -> rl.Rectangle:
    right = hud_rect.x + hud_rect.width - UI_CONFIG.border_size
    top = hud_rect.y + UI_CONFIG.border_size + UI_CONFIG.button_size + 30
    if not self._expanded:
      _, text, _ = self._status()
      strip_w = measure_text_cached(self._font_bold, text, STRIP_FONT).x + 2 * PAD
      w = PILL_W + STRIP_GAP + strip_w
      return rl.Rectangle(right - w, top, w, PILL_H)
    height = PILL_H + 3 * ROW_H + N_KNOBS * ROW_H + PAD
    return rl.Rectangle(right - PANEL_W, top, PANEL_W, height)

  def _handle_mouse_release(self, pos) -> None:
    for name, r in self._hit.items():
      if rl.check_collision_point_rec(pos, r):
        self._on_tap(name)
        return

  def _on_tap(self, name: str) -> None:
    if name == "pill":
      self._expanded = not self._expanded
      return
    # Params.put is asynchronous in this tree: keep the new value locally and hold off the
    # periodic re-read, or it reads the old value back before the write has landed.
    self._last_refresh = time.monotonic()
    if name == "toggle":
      self._enabled = not self._enabled
      self._params.put_bool("LaneCentering", self._enabled)
      return
    for key, _, step, lo, hi, _ in self._knobs:
      if name in (key + "-", key + "+"):
        v = self._values[key] + (step if name.endswith("+") else -step)
        v = round(min(max(v, lo), hi), 3)
        self._params.put(key, v)
        self._values[key] = v
        return

  def _button(self, name: str, r: rl.Rectangle, text: str, bg: rl.Color, size: int = 52) -> None:
    self._hit[name] = r
    rl.draw_rectangle_rounded(r, 0.3, 10, bg)
    sz = measure_text_cached(self._font_bold, text, size)
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(r.x + (r.width - sz.x) / 2, r.y + (r.height - sz.y) / 2), size, 0, WHITE)

  @staticmethod
  def _arrow(pill: rl.Rectangle, left: bool) -> None:
    """Direction triangle inside the pill edge while the controller is nudging."""
    cy, h, w = pill.y + pill.height / 2, 26, 22
    if left:
      tip = pill.x + 18
      rl.draw_triangle(rl.Vector2(tip, cy), rl.Vector2(tip + w, cy + h), rl.Vector2(tip + w, cy - h), WHITE)
    else:
      tip = pill.x + pill.width - 18
      rl.draw_triangle(rl.Vector2(tip, cy), rl.Vector2(tip - w, cy - h), rl.Vector2(tip - w, cy + h), WHITE)

  def _text(self, text: str, x: float, y: float, size: int = 44, color: rl.Color = WHITE) -> None:
    rl.draw_text_ex(self._font, text, rl.Vector2(x, y), size, 0, color)

  def _wall_bars(self, pill: rl.Rectangle, st) -> None:
    if st.laneCentreWallLeft:
      rl.draw_rectangle_rec(rl.Rectangle(pill.x - WALL_BAR_W - 4, pill.y + 8, WALL_BAR_W, pill.height - 16), AMBER)
    if st.laneCentreWallRight:
      rl.draw_rectangle_rec(rl.Rectangle(pill.x + pill.width - WALL_BAR_W, pill.y + 8, WALL_BAR_W, pill.height - 16), AMBER)

  def _render(self, rect: rl.Rectangle) -> None:
    self._hit = {}
    st = ui_state.sm["lateralTuneStateSP"]
    status, text, color = self._status()
    pill = rl.Rectangle(rect.x + rect.width - PILL_W, rect.y, PILL_W, PILL_H)

    if not self._expanded:
      strip = rl.Rectangle(rect.x, rect.y, rect.width - PILL_W - STRIP_GAP, PILL_H)
      self._hit["pill"] = rect  # the strip opens the panel too
      rl.draw_rectangle_rounded(strip, 0.4, 10, BG)
      sz = measure_text_cached(self._font_bold, text, STRIP_FONT)
      rl.draw_text_ex(self._font_bold, text, rl.Vector2(strip.x + PAD, strip.y + (PILL_H - sz.y) / 2), STRIP_FONT, 0, color)
      self._button("pill", pill, "LC", ON_BG if self._enabled else OFF_BG, 60)
      self._hit["pill"] = rect
      if status in ARROW_LEFT + ARROW_RIGHT:
        self._arrow(pill, status in ARROW_LEFT)
      self._wall_bars(pill, st)
      return

    rl.draw_rectangle_rounded(rect, 0.12, 10, BG)
    self._button("pill", pill, "LC", ON_BG if self._enabled else OFF_BG, 60)
    if status in ARROW_LEFT + ARROW_RIGHT:
      self._arrow(pill, status in ARROW_LEFT)
    self._wall_bars(pill, st)

    x0 = rect.x + PAD
    self._text("LANE CENTER", x0, rect.y + (PILL_H - 50) / 2, 50)
    toggle = rl.Rectangle(x0, rect.y + PILL_H + 10, 200, BTN_H)
    self._button("toggle", toggle, "ON" if self._enabled else "OFF", ON_BG if self._enabled else OFF_BG)

    pos = POSITION_NAMES.get(int(st.laneCentrePosition), "--")
    self._text(f"lane {pos}", x0 + 230, rect.y + PILL_H + 12, 42)
    self._text(f"offset {st.laneCentreOffset * 100:+.0f} cm", x0 + 230, rect.y + PILL_H + 60, 42, GREY)

    # status: shrink to fit the long guard reasons
    y_status = rect.y + PILL_H + ROW_H + 30
    size = 46
    while size > 30 and measure_text_cached(self._font, text, size).x > rect.width - 2 * PAD:
      size -= 2
    self._text(text, x0, y_status, size, color)

    y_info = rect.y + PILL_H + 2 * ROW_H + 20
    if self._camera_mode:
      # spare clearance beyond the margin per side (cm), amber when the guard is cutting in
      x = x0
      parts = [("spare", GREY)]
      for side, slack in (("L", st.laneCentreSlackLeft), ("R", st.laneCentreSlackRight)):
        parts.append((f"{side} --" if slack != slack else f"{side} {slack * 100:+.0f}", AMBER if slack == slack and slack < 0 else GREY))
      for txt, col in parts:
        self._text(txt, x, y_info, 40, col)
        x += measure_text_cached(self._font, txt, 40).x + 24
      walls = " ".join(t for t, on in (("WALL L", st.laneCentreWallLeft), ("WALL R", st.laneCentreWallRight)) if on)
      if walls:
        sz = measure_text_cached(self._font_bold, walls, 40)
        rl.draw_text_ex(self._font_bold, walls, rl.Vector2(rect.x + rect.width - PAD - sz.x, y_info), 40, 0, AMBER)
    else:
      # correction as the lateral accel it adds: curvature * v^2
      accel = abs(st.laneCentreCorrection) * ui_state.sm["carState"].vEgo ** 2
      # the UI font has no "²" glyph: draw a raised small "2"
      self._text(f"trim {accel:.2f} m/s", x0, y_info, 40, GREY)
      self._text("2", x0 + measure_text_cached(self._font, f"trim {accel:.2f} m/s", 40).x, y_info - 6, 26, GREY)
    self._render_knobs(rect, x0)

  def _render_knobs(self, rect: rl.Rectangle, x0: float) -> None:
    y = rect.y + PILL_H + 3 * ROW_H
    for key, label, _, _, _, fmt in self._knobs:
      self._text(label, x0, y + (BTN_H - 44) / 2, 44)
      minus = rl.Rectangle(rect.x + rect.width - PAD - 2 * BTN_W - 170, y, BTN_W, BTN_H)
      plus = rl.Rectangle(rect.x + rect.width - PAD - BTN_W, y, BTN_W, BTN_H)
      self._button(key + "-", minus, "-", BTN_BG)
      self._button(key + "+", plus, "+", BTN_BG)
      val = fmt(self._values[key])
      sz = measure_text_cached(self._font_bold, val, 44)
      mid = minus.x + BTN_W + (plus.x - minus.x - BTN_W - sz.x) / 2
      rl.draw_text_ex(self._font_bold, val, rl.Vector2(mid, y + (BTN_H - sz.y) / 2), 44, 0, WHITE)
      y += ROW_H
