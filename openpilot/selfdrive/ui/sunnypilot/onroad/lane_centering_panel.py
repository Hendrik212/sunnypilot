"""
Onroad tuning panel for dynamic lane centering (comma 3X / tizi layout).

Shown only while LaneCenteringOnroadPanel is on. Collapsed it is an "LC" pill under the
experimental-mode button; tapping it opens a panel with an on/off button, a live readout
from lateralTuneStateSP (lane position, applied offset, correction) and -/+ rows for
LaneCenterOffset and LaneCenteringGain. The buttons only write Params -- controlsd picks
them up on its next params read (PARAMS_UPDATE_PERIOD) -- and the readout comes from the
published telemetry, so the panel shows what is actually applied.

One Widget covering the whole panel (hit-testing its own sub-rects) so that any touch on it
reports is_pressed and the road view does not also toggle the sidebar.
"""
import time

import pyray as rl

from openpilot.common.params import Params
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

KNOBS = (
  # param, label, step, min, max, format
  ("LaneCenterOffset", "Offset", 0.01, 0.0, 0.30, "{:.2f} m"),
  ("LaneCenteringGain", "Gain", 0.05, 0.0, 0.60, "{:.2f}"),
)

BG = rl.Color(0, 0, 0, 180)
BTN_BG = rl.Color(255, 255, 255, 40)
ON_BG = rl.Color(0x17, 0x86, 0x44, 230)
OFF_BG = rl.Color(0x80, 0x80, 0x80, 160)
WHITE = rl.Color(255, 255, 255, 255)
GREY = rl.Color(200, 200, 200, 255)


class LaneCenteringPanel(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._expanded = False
    self._show = False
    self._enabled = False
    self._values = {k[0]: k[3] for k in KNOBS}
    self._last_refresh = 0.0
    self._hit: dict[str, rl.Rectangle] = {}

  def _refresh_params(self) -> None:
    now = time.monotonic()
    if now - self._last_refresh < PARAM_REFRESH_S:
      return
    self._last_refresh = now
    self._show = self._params.get_bool("LaneCenteringOnroadPanel")
    self._enabled = self._params.get_bool("LaneCentering")
    for key, _, _, lo, hi, _ in KNOBS:
      try:
        self._values[key] = min(max(float(self._params.get(key, return_default=True)), lo), hi)
      except (TypeError, ValueError):
        pass

  def _update_state(self) -> None:
    self._refresh_params()
    self.set_visible(self._show and ui_state.started)
    if not self.is_visible:
      self._expanded = False

  def layout_rect(self, hud_rect: rl.Rectangle) -> rl.Rectangle:
    right = hud_rect.x + hud_rect.width - UI_CONFIG.border_size
    top = hud_rect.y + UI_CONFIG.border_size + UI_CONFIG.button_size + 30
    if not self._expanded:
      return rl.Rectangle(right - PILL_W, top, PILL_W, PILL_H)
    height = PILL_H + 2 * ROW_H + len(KNOBS) * ROW_H + PAD
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
    for key, _, step, lo, hi, _ in KNOBS:
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

  def _text(self, text: str, x: float, y: float, size: int = 44, color: rl.Color = WHITE) -> None:
    rl.draw_text_ex(self._font, text, rl.Vector2(x, y), size, 0, color)

  def _render(self, rect: rl.Rectangle) -> None:
    self._hit = {}
    rl.draw_rectangle_rounded(rect, 0.12 if self._expanded else 0.4, 10, BG)

    pill = rl.Rectangle(rect.x + rect.width - PILL_W, rect.y, PILL_W, PILL_H)
    self._button("pill", pill, "LC", ON_BG if self._enabled else OFF_BG, 60)
    if not self._expanded:
      return

    x0 = rect.x + PAD
    self._text("LANE CENTER", x0, rect.y + (PILL_H - 50) / 2, 50)
    toggle = rl.Rectangle(x0, rect.y + PILL_H + 10, 200, BTN_H)
    self._button("toggle", toggle, "ON" if self._enabled else "OFF", ON_BG if self._enabled else OFF_BG)

    st = ui_state.sm["lateralTuneStateSP"]
    pos = POSITION_NAMES.get(int(st.laneCentrePosition), "--")
    self._text(f"lane {pos}", x0 + 230, rect.y + PILL_H + 12, 42)
    self._text(f"off {st.laneCentreOffset:+.2f} m", x0 + 230, rect.y + PILL_H + 60, 42, GREY)
    self._text(f"corr {st.laneCentreCorrection * 1e4:+.1f}e-4 1/m{'  *' if st.laneCentreActive else ''}",
               x0, rect.y + PILL_H + ROW_H + 30, 42, GREY)

    y = rect.y + PILL_H + 2 * ROW_H
    for key, label, _, _, _, fmt in KNOBS:
      self._text(label, x0, y + (BTN_H - 44) / 2, 44)
      minus = rl.Rectangle(rect.x + rect.width - PAD - 2 * BTN_W - 170, y, BTN_W, BTN_H)
      plus = rl.Rectangle(rect.x + rect.width - PAD - BTN_W, y, BTN_W, BTN_H)
      self._button(key + "-", minus, "-", BTN_BG)
      self._button(key + "+", plus, "+", BTN_BG)
      val = fmt.format(self._values[key])
      sz = measure_text_cached(self._font_bold, val, 44)
      mid = minus.x + BTN_W + (plus.x - minus.x - BTN_W - sz.x) / 2
      rl.draw_text_ex(self._font_bold, val, rl.Vector2(mid, y + (BTN_H - sz.y) / 2), 44, 0, WHITE)
      y += ROW_H
