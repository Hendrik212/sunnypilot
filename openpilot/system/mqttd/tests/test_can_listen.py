from openpilot.system.mqttd.status import CAN_LISTEN_DEFAULT_S, listen_seconds


def test_listen_seconds_from_payload():
  assert listen_seconds("600") == 600
  assert listen_seconds("120.5") == 120
  assert listen_seconds("PRESS") == CAN_LISTEN_DEFAULT_S
  assert listen_seconds(None) == CAN_LISTEN_DEFAULT_S
  # clamped like pandad: the 12 V awake budget caps the window
  assert listen_seconds("5000") == 900
  assert listen_seconds("1") == 10
