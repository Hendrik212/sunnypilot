"""Params writes off the deviceState publishing thread.

A params write takes the params lock and fsyncs. When the storage stalls the write waits with
it: route 269 (Nov 4 2026, 15:38 UTC) had 8 s of iowait on /data (90% full, loggerd, mapd and
journald stalled too), hardware_thread sat 7.7 s in Params().remove for an offroad alert,
deviceState stopped and selfdrived raised commIssue. ParamWriter runs the writes in order in its
own thread; the publish loop only queues them.
"""
import queue
import threading
from collections.abc import Callable

from openpilot.common.swaglog import cloudlog


class ParamWriter:
  def __init__(self):
    self.q: queue.Queue[tuple[Callable, tuple, dict]] = queue.Queue()

  def submit(self, fn: Callable, *args, **kwargs) -> None:
    self.q.put((fn, args, kwargs))

  def _write_one(self, timeout: float) -> bool:
    try:
      fn, args, kwargs = self.q.get(timeout=timeout)
    except queue.Empty:
      return False
    try:
      fn(*args, **kwargs)
    except Exception:
      cloudlog.exception("hardwared param write failed")
    return True

  def run(self, end_event: threading.Event) -> None:
    while not end_event.is_set():
      self._write_one(timeout=0.5)
    # writes queued before the end still go out (uptime, DoShutdown)
    while self._write_one(timeout=0.0):
      pass
