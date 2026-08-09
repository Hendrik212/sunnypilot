#!/usr/bin/env python3
import time
import datetime

import cereal.messaging as messaging
from cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.utils import strip_deprecated_keys
from collections import defaultdict

from openpilot.system.mqttd import mqttd
from opendbc.car.hyundai import mqtt
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.power_monitoring import VBATT_PAUSE_CHARGING

# hardwared compares a low-passed car voltage against the shutdown threshold, not
# the instantaneous reading (power_monitoring.CAR_VOLTAGE_LOW_PASS_K, 45s tau).
# Mirror that here so the published value matches what actually triggers shutdown.
CAR_VOLTAGE_LOW_PASS_TAU_S = 45.0
LOW_VOLTAGE_SHUTDOWN_MAX = 12.5  # upper clamp applied by the LowVoltageShutdown toggle


def get_low_voltage_shutdown_threshold(params):
  """Effective shutdown threshold in volts, mirroring the starpilot toggle."""
  try:
    if not params.get_bool("DeviceManagement"):
      return VBATT_PAUSE_CHARGING
    value = float(params.get("LowVoltageShutdown"))
  except (TypeError, ValueError, AttributeError):
    return VBATT_PAUSE_CHARGING
  return min(max(value, VBATT_PAUSE_CHARGING), LOW_VOLTAGE_SHUTDOWN_MAX)


#f = open("mqtt_status_log.txt", "w")

def format_time(minutes):
  """Convert minutes to HH:MM format. Returns None if minutes < 0."""
  if minutes < 0:
    return None
  hours = minutes // 60
  mins = minutes % 60
  return f"{hours:02d}:{mins:02d}"

def get_device_info():
  """Build device info for HA discovery."""
  client_id = mqttd.client_id()
  return {
    "identifiers": [client_id],
    "name": "Ioniq 6",
    "manufacturer": "comma.ai",
    "model": HARDWARE.get_device_type(),
    "sw_version": "openpilot"
  }

def publish_sensor_discovery(pm, sensor_name, device_info, config_prefix):
  """Publish discovery config for a single sensor."""
  client_id = mqttd.client_id()
  # Fix: Use underscores instead of slashes in topic (HA requirement)
  unique_id = f"openpilot_{client_id}_{sensor_name}"
  object_id = f"openpilot_{client_id}_{sensor_name}"

  sensors = {
    "battery_level": {
      "name": "Battery Level",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.battery_level }}",
      "unit_of_measurement": "%",
      "device_class": "battery",
    },
    "range": {
      "name": "Range",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.range }}",
      "unit_of_measurement": "km",
      "device_class": "distance",
    },
    "pack_voltage": {
      "name": "Pack Voltage",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.pack_voltage }}",
      "unit_of_measurement": "V",
      "device_class": "voltage",
    },
    "charging_current": {
      "name": "Charging Current",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.charging_current }}",
      "unit_of_measurement": "A",
      "device_class": "current",
    },
    "charging_power": {
      "name": "Charging Power",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.charging_power }}",
      "unit_of_measurement": "kW",
      "device_class": "power",
    },
    "charging_time_remaining": {
      "name": "Charging Time Remaining",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.charging_time_remaining }}",
      "icon": "mdi:clock",
    },
    "charging_status": {
      "name": "Charging Status",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.charging_status }}",
      "icon": "mdi:ev-station",
    },
    "charge_limit_ac": {
      "name": "Charge Limit AC",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.charge_limit_ac }}",
      "unit_of_measurement": "%",
      "icon": "mdi:battery-charging-80",
    },
    "charge_limit_dc": {
      "name": "Charge Limit DC",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.charge_limit_dc }}",
      "unit_of_measurement": "%",
      "icon": "mdi:battery-charging-high",
    },
    "ac_power_limit_pct": {
      "name": "AC Power Limit",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.ac_power_limit_pct }}",
      "unit_of_measurement": "%",
      "icon": "mdi:flash",
    },
    "ac_current_limit": {
      "name": "AC Current Limit",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.ac_current_limit }}",
      "icon": "mdi:current-ac",
    },
    "car_voltage": {
      "name": "12V Battery Voltage",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.car_voltage }}",
      "unit_of_measurement": "V",
      "device_class": "voltage",
      "state_class": "measurement",
      "suggested_display_precision": 2,
    },
    "car_voltage_filtered": {
      "name": "12V Battery Voltage (Filtered)",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.car_voltage_filtered }}",
      "unit_of_measurement": "V",
      "device_class": "voltage",
      "state_class": "measurement",
      "suggested_display_precision": 2,
      "icon": "mdi:sine-wave",
    },
    "low_voltage_shutdown_threshold": {
      "name": "Low Voltage Shutdown Threshold",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ value_json.low_voltage_shutdown_threshold }}",
      "unit_of_measurement": "V",
      "device_class": "voltage",
      "suggested_display_precision": 2,
      "icon": "mdi:power-plug-off",
    },
  }

  if sensor_name not in sensors:
    return

  config = sensors[sensor_name].copy()
  config["unique_id"] = unique_id
  config["device"] = device_info
  config["availability_topic"] = f"openpilot/{client_id}/availability"
  config["payload_available"] = "online"
  config["payload_not_available"] = "offline"

  # Fix: Use underscores instead of slashes (HA discovery format requirement)
  topic = f"homeassistant/sensor/{object_id}/config"
  mqttd.publish(pm, topic, config, qos=1, retain=True)

def publish_binary_sensor_discovery(pm, sensor_name, device_info, config_prefix):
  """Publish discovery config for a single binary sensor."""
  client_id = mqttd.client_id()
  unique_id = f"openpilot_{client_id}_{sensor_name}"
  object_id = f"openpilot_{client_id}_{sensor_name}"

  binary_sensors = {
    "connector_connected": {
      "name": "Connector Connected",
      "state_topic": "openpilot/car_status",
      "value_template": "{{ 'ON' if value_json.connector_connected else 'OFF' }}",
      "device_class": "plug",
    },
  }

  if sensor_name not in binary_sensors:
    return

  config = binary_sensors[sensor_name].copy()
  config["unique_id"] = unique_id
  config["device"] = device_info
  config["availability_topic"] = f"openpilot/{client_id}/availability"
  config["payload_available"] = "online"
  config["payload_not_available"] = "offline"

  topic = f"homeassistant/binary_sensor/{object_id}/config"
  mqttd.publish(pm, topic, config, qos=1, retain=True)

def publish_button_discovery(pm, button_name, device_info, config_prefix):
  """Publish discovery config for a single button."""
  client_id = mqttd.client_id()
  unique_id = f"openpilot_{client_id}_{button_name}"
  object_id = f"openpilot_{client_id}_{button_name}"

  buttons = {
    "wake_can_bus": {
      "name": "Wake CAN Bus",
      "command_topic": "openpilot/command/wake",
      "payload_press": "PRESS",
      "icon": "mdi:car-electric",
    },
  }

  if button_name not in buttons:
    return

  config = buttons[button_name].copy()
  config["unique_id"] = unique_id
  config["device"] = device_info
  config["availability_topic"] = f"openpilot/{client_id}/availability"
  config["payload_available"] = "online"
  config["payload_not_available"] = "offline"

  topic = f"homeassistant/button/{object_id}/config"
  mqttd.publish(pm, topic, config, qos=1, retain=True)

def publish_ha_discovery(pm, count, config_prefix):
  """Publish discovery configs for all sensors."""
  device_info = get_device_info()
  sensors = [
    "battery_level",
    "range",
    "pack_voltage",
    "charging_current",
    "charging_power",
    "charging_time_remaining",
    "charging_status",
    "charge_limit_ac",
    "charge_limit_dc",
    "ac_power_limit_pct",
    "ac_current_limit",
    "car_voltage",
    "car_voltage_filtered",
    "low_voltage_shutdown_threshold",
  ]

  binary_sensors = [
    "connector_connected",
  ]

  buttons = [
    "wake_can_bus",
  ]

  for sensor in sensors:
    publish_sensor_discovery(pm, sensor, device_info, config_prefix)

  for sensor in binary_sensors:
    publish_binary_sensor_discovery(pm, sensor, device_info, config_prefix)

  for button in buttons:
    publish_button_discovery(pm, button, device_info, config_prefix)

def status_thread():
  config_prefix = 'openpilot'
  status_prefix = ''

  #time.sleep(30)
  #f.write(str(datetime.datetime.now()) + '--------------------------------------------\n')
  #f.write("mqtt status daemon started\n\n")
  #f.flush()
  pm = messaging.PubMaster(['mqttPubQueue'])
  sm = messaging.SubMaster(['mqttRecvQueue'])
  logcan = messaging.sub_sock('can', timeout=10)
  dat = defaultdict(int)

  panda_state_timeout = int(1000 * 2.5 * DT_CTRL)  # 2.5x the expected pandaState frequency
  panda_state_sock = messaging.sub_sock('pandaState', timeout=panda_state_timeout)
  peripheral_state_sock = messaging.sub_sock('peripheralState')
  location_sock = messaging.sub_sock('gpsLocationExternal')
  device_sock = messaging.sub_sock('deviceState')
  params = Params()

  publish_ha_discovery(pm, "FIRST", config_prefix)

  count = 0
  total_count = 0
  panda_prev = None
  location_prev = None
  device_prev = None
  soc_prev = None
  range_prev = None
  pack_voltage_prev = None
  charging_current_prev = None
  charging_power_prev = None
  charging_time_remaining_prev = None
  charging_status_prev = None
  sleeptimer = time.monotonic()
  car_voltage_instant = None
  car_voltage_filtered = None
  car_voltage_last_t = None

  while True:
    cur_time = time.monotonic()

    # get can messages
    can_recv = messaging.drain_sock(logcan)
    bus = 1
    mqtt.getParsedMessages(can_recv, bus, dat, pm)

    # sample the 12V line every iteration so the low-pass advances at the real
    # peripheralState rate rather than the 30s publish cadence
    peripheral_msgs = [m for m in messaging.drain_sock(peripheral_state_sock)
                       if m.peripheralState.pandaType != log.PandaState.PandaType.unknown]
    if peripheral_msgs:
      car_voltage_instant = peripheral_msgs[-1].peripheralState.voltage / 1e3
      dt = cur_time - car_voltage_last_t if car_voltage_last_t is not None else 0.0
      if car_voltage_filtered is None or dt <= 0.0:
        car_voltage_filtered = car_voltage_instant
      else:
        alpha = dt / (CAR_VOLTAGE_LOW_PASS_TAU_S + dt)
        car_voltage_filtered += alpha * (car_voltage_instant - car_voltage_filtered)
      car_voltage_last_t = cur_time

    if cur_time > sleeptimer + 30: # update mqtt all 30s
      #f.write(str(datetime.datetime.now()) + " entering loop...\n")
      #f.flush()
      sleeptimer = cur_time

      # mqtt message receiver - subscribe to both command topics
      mqttd.subscribe(pm, "openpilot/command")
      mqttd.subscribe(pm, "openpilot/command/wake")
      sm.update(1)
      if count == 10:
        count = 0
        publish_ha_discovery(pm, total_count, config_prefix)
      count = count + 1

      if sm.updated["mqttRecvQueue"]:
        message = sm["mqttRecvQueue"]
        topic = message.topic if hasattr(message, 'topic') else ''
        payload = message.payload if hasattr(message, 'payload') else ''
        print(f"[MQTT] Received message on {topic}: {payload}")
        with open("/tmp/mqtt_wake.log", "a") as f:
          f.write(f"{datetime.datetime.now()} - Received: {topic} = {payload}\n")

        # Handle wake CAN bus button press
        if 'wake' in topic or payload == 'PRESS':
          print("[MQTT] Wake CAN bus button pressed - triggering wake", flush=True)
          with open("/tmp/mqtt_wake.log", "a") as f:
            f.write(f"{datetime.datetime.now()} - WAKE TRIGGERED\n")
          try:
            result = mqtt.wakeCanBus()
            print(f"[MQTT] Wake result: {result}", flush=True)
            with open("/tmp/mqtt_wake.log", "a") as f:
              f.write(f"{datetime.datetime.now()} - Wake result: {result}\n")
          except Exception as e:
            print(f"[MQTT] Wake failed: {e}", flush=True)
            with open("/tmp/mqtt_wake.log", "a") as f:
              f.write(f"{datetime.datetime.now()} - Wake FAILED: {e}\n")


      panda = messaging.recv_sock(panda_state_sock)
      location = messaging.recv_sock(location_sock)
      device = messaging.recv_sock(device_sock)

      panda_prev = panda if panda else panda_prev
      device_prev = device if device else device_prev
      location_prev = location if location else location_prev

      topic = f"{status_prefix}/openpilot/status"
      content = {"panda_state": (strip_deprecated_keys(panda_prev.to_dict()) if panda_prev else None),
                }
      mqttd.publish(pm, topic, content)


      topic = f"openpilot/car_status"
      content = {"battery_level": mqtt.soc_out,
                 "range": mqtt.range_out,
                 "pack_voltage": mqtt.pack_voltage_out,
                 "charging_current": mqtt.charging_current_out,
                 "charging_power": mqtt.charging_power_out,
                 "charging_time_remaining_minutes": mqtt.charging_time_remaining_out,
                 "charging_time_remaining": format_time(mqtt.charging_time_remaining_out),
                 "charging_status": mqtt.charging_status_out,
                 "connector_connected": mqtt.connector_connected_out,
                 "charge_limit_ac": mqtt.charge_limit_ac_out,
                 "charge_limit_dc": mqtt.charge_limit_dc_out,
                 "ac_power_limit_pct": mqtt.ac_power_limit_pct_out,
                 "ac_current_limit": mqtt.ac_current_limit_out,
                 "car_voltage": round(car_voltage_instant, 2) if car_voltage_instant is not None else None,
                 "car_voltage_filtered": round(car_voltage_filtered, 2) if car_voltage_filtered is not None else None,
                 "low_voltage_shutdown_threshold": get_low_voltage_shutdown_threshold(params),
                }
      mqttd.publish(pm, topic, content)

      soc_prev = mqtt.soc_out
      range_prev = mqtt.range_out
      pack_voltage_prev = mqtt.pack_voltage_out
      charging_current_prev = mqtt.charging_current_out
      charging_power_prev = mqtt.charging_power_out
      charging_time_remaining_prev = mqtt.charging_time_remaining_out
      charging_status_prev = mqtt.charging_status_out


def main():
  status_thread()

if __name__ == "__main__":
  main()
