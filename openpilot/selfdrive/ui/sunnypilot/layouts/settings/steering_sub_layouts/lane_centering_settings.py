"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable
import pyray as rl

from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp, LineSeparatorSP
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget


class LaneCenteringSettingsLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=False, spacing=0)

  def _initialize_items(self):
    self._enable = toggle_item_sp(
      param="LaneCentering",
      title=lambda: tr("Lane Centering"),
      description=lambda: tr("Trim the model's path toward the centre of the lane lines, plus an offset chosen by " +
                             "the lane you are in: right lane (or a road without neighbour lanes) right of centre, " +
                             "middle lane dead centre, left lane left of centre."),
    )
    self._offset = option_item_sp(
      param="LaneCenterOffset",
      title=lambda: tr("Lane Offset"),
      description=lambda: tr("How far right (right lane) or left (left lane) of the lane centre to drive. " +
                             "Reduced automatically in narrow lanes."),
      min_value=0,
      max_value=30,
      value_change_step=1,
      use_float_scaling=True,
      label_callback=lambda v: f"{v / 100:.2f} m",
    )
    self._pause_on_signal = toggle_item_sp(
      param="LaneCenteringPauseOnSignal",
      title=lambda: tr("Pause Lane Centering with Blinker"),
      description=lambda: tr("Fade the lane centering correction out while a turn signal is on."),
    )
    self._onroad_panel = toggle_item_sp(
      param="LaneCenteringOnroadPanel",
      title=lambda: tr("Show Tuning Panel Onroad"),
      description=lambda: tr("Show an \"LC\" button on the driving screen that opens a panel to switch lane centering " +
                             "and adjust offset and gain while driving."),
    )

    return [
      self._enable,
      self._offset,
      LineSeparatorSP(40),
      self._pause_on_signal,
      LineSeparatorSP(40),
      self._onroad_panel,
    ]

  def _update_state(self):
    super()._update_state()
    enabled = self._enable.action_item.get_state()
    self._offset.set_visible(enabled)
    self._pause_on_signal.set_visible(enabled)

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()
    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40, rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._scroller.show_event()
