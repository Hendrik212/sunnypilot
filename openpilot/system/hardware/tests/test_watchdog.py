import json
import threading
import time

import pytest

from openpilot.system.hardware.watchdog import HardwareWatchdog


class Clock:
  def __init__(self):
    self.now = 100.

  def __call__(self):
    return self.now


def test_stall_stack_repeat_limit_and_recovery(tmp_path):
  clock = Clock()
  watchdog = HardwareWatchdog(str(tmp_path / 'watchdog.log'), clock=clock)
  assert watchdog.poll() == []  # startup has no publication deadline yet
  watchdog.arm()
  watchdog.stage('thermal_read')
  clock.now += 1.
  assert watchdog.poll() == []
  clock.now += .6
  event, = watchdog.poll()
  assert event['event'] == 'stalled'
  assert event['stage'] == 'thermal_read'
  assert event['publication_gap'] == pytest.approx(1.6)
  assert 'test_stall_stack_repeat_limit_and_recovery' in event['threads'][str(threading.get_ident())]['stack']
  clock.now += 1.
  assert watchdog.poll() == []
  clock.now += 4.
  assert watchdog.poll()[0]['event'] == 'stalled'
  watchdog.stage('publish_device_state')
  watchdog.published()
  event, = watchdog.poll()
  assert event['event'] == 'recovered'
  assert event['publication_gap'] == pytest.approx(6.6)
  assert event['slow_stages'] == (('thermal_read', pytest.approx(6.6)),)
  # A new incident can dump immediately, without waiting out the prior incident's limiter.
  clock.now += 2.
  assert watchdog.poll()[0]['event'] == 'stalled'


def test_recovery_survives_watchdog_not_running_during_stall(tmp_path):
  clock = Clock()
  watchdog = HardwareWatchdog(str(tmp_path / 'watchdog.log'), clock=clock)
  watchdog.arm()
  watchdog.stage('uptime_onroad_persist')
  clock.now += 8.
  watchdog.stage('publish_device_state')
  watchdog.published()
  event, = watchdog.poll()
  assert event['event'] == 'recovered'
  assert event['publication_gap'] == 8.
  assert event['slow_stages'] == (('uptime_onroad_persist', 8.),)


def test_healthy_publication_is_quiet_and_does_no_file_io(tmp_path):
  clock = Clock()
  watchdog = HardwareWatchdog(str(tmp_path / 'watchdog.log'), clock=clock)
  watchdog.arm()
  for _ in range(100):
    clock.now += .5
    watchdog.stage('publish_device_state')
    watchdog.published()
    assert watchdog.poll() == []
  assert watchdog.handler is None
  assert not list(tmp_path.iterdir())


def test_diagnostic_file_is_bounded_and_valid_json(tmp_path):
  watchdog = HardwareWatchdog(str(tmp_path / 'watchdog.log'))
  watchdog.MAX_BYTES = 2048
  try:
    for i in range(100):
      watchdog._write({'event': 'stalled', 'sequence': i, 'stack': 'x' * 400})
    files = list(tmp_path.iterdir())
    assert len(files) == 3
    assert sum(f.stat().st_size for f in files) <= 3 * 2048
    for f in files:
      assert all(json.loads(line)['event'] == 'stalled' for line in f.read_text().splitlines())
  finally:
    if watchdog.handler is not None:
      watchdog.handler.close()


def test_watchdog_captures_a_blocked_publisher_without_interfering(tmp_path):
  path = tmp_path / 'watchdog.log'
  watchdog = HardwareWatchdog(str(path))
  watchdog.STALL_SECONDS = .05
  stop = threading.Event()
  blocked = threading.Event()
  release = threading.Event()

  def blocked_operation():
    watchdog.arm()
    watchdog.stage('test_blocking_operation')
    blocked.set()
    release.wait(5.)
    watchdog.stage('publish_device_state')
    watchdog.published()

  publisher = threading.Thread(target=blocked_operation, name='test_publisher')
  diagnostic = threading.Thread(target=watchdog.run, args=(stop,))
  try:
    publisher.start()
    assert blocked.wait(2.)
    diagnostic.start()
    deadline = time.monotonic() + 3.
    while time.monotonic() < deadline:
      if path.exists() and '"event": "stalled"' in path.read_text():
        break
      time.sleep(.01)
    events = [json.loads(line) for line in path.read_text().splitlines()]
    stalled = next(e for e in events if e['event'] == 'stalled')
    assert stalled['stage'] == 'test_blocking_operation'
    assert 'blocked_operation' in stalled['threads'][str(publisher.ident)]['stack']
    release.set()
    publisher.join(2.)
    assert not publisher.is_alive()
    deadline = time.monotonic() + 3.
    while time.monotonic() < deadline:
      if '"event": "recovered"' in path.read_text():
        break
      time.sleep(.01)
    assert '"event": "recovered"' in path.read_text()
  finally:
    release.set()
    stop.set()
    publisher.join(2.)
    if diagnostic.ident is not None:
      diagnostic.join(2.)


def test_failed_diagnostic_storage_does_not_block_publisher(tmp_path):
  # A regular file where the log's parent directory should be makes initialization fail.
  parent = tmp_path / 'not_a_directory'
  parent.write_text('occupied')
  clock = Clock()
  watchdog = HardwareWatchdog(str(parent / 'watchdog.log'), clock=clock)
  stop = threading.Event()
  diagnostic = threading.Thread(target=watchdog.run, args=(stop,))
  diagnostic.start()
  try:
    watchdog.arm()
    for _ in range(10):
      clock.now += .5
      watchdog.stage('publish_device_state')
      watchdog.published()
    assert watchdog.snapshot.last_publication == 105.
    assert diagnostic.is_alive()
  finally:
    stop.set()
    diagnostic.join(2.)
  assert not diagnostic.is_alive()


def test_blocked_diagnostic_writer_does_not_block_publication(tmp_path, monkeypatch):
  clock = Clock()
  watchdog = HardwareWatchdog(str(tmp_path / 'watchdog.log'), clock=clock)
  entered = threading.Event()
  release = threading.Event()
  stop = threading.Event()

  def blocked_write(_event):
    entered.set()
    release.wait(5.)

  monkeypatch.setattr(watchdog, '_write', blocked_write)
  diagnostic = threading.Thread(target=watchdog.run, args=(stop,))
  diagnostic.start()
  try:
    assert entered.wait(2.)
    watchdog.arm()
    clock.now += 8.
    watchdog.stage('publish_device_state')
    watchdog.published()
    assert watchdog.snapshot.last_publication == 108.
    assert watchdog.recoveries[0]['publication_gap'] == 8.
  finally:
    stop.set()
    release.set()
    diagnostic.join(2.)
  assert not diagnostic.is_alive()
