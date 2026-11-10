import time
from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.mqttd import mqttd


class TestPubQueue(OpenpilotTestCase):
  def test_discovery_burst_is_queued_whole(self):
    # status.py publishes one discovery config per sensor back to back; a SubMaster kept only the last one
    pm = messaging.PubMaster(['mqttPubQueue'])
    sock = messaging.sub_sock('mqttPubQueue', timeout=100)
    time.sleep(0.1)
    topics = [f"homeassistant/sensor/s{i}/config" for i in range(17)]
    for t in topics:
      mqttd.publish(pm, t, {"name": t}, qos=1, retain=True)
    client = SimpleNamespace(sub_dict={}, pub_list=[])
    for _ in range(5):
      for event in messaging.drain_sock(sock, wait_for_one=True):
        mqttd.queue_message(client, event.mqttPubQueue)
      if len(client.pub_list) == len(topics):
        break
    assert [m["topic"] for m in client.pub_list] == topics
    assert all(m["retain"] and m["qos"] == 1 for m in client.pub_list)


def test_subscribe_and_unsubscribe():
  client = SimpleNamespace(sub_dict={}, pub_list=[])
  sub = SimpleNamespace(subscribe=True, publish=False, topic="openpilot/command")
  mqttd.queue_message(client, sub)
  assert client.sub_dict["openpilot/command"] == {"server_state": False, "subscribe": True}
  mqttd.queue_message(client, SimpleNamespace(subscribe=False, publish=False, topic="openpilot/command"))
  assert client.sub_dict["openpilot/command"]["subscribe"] is False
