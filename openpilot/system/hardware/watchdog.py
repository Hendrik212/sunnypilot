"""Diagnostics for a stalled deviceState publisher; never supplies a replacement heartbeat."""
import json
import logging
from logging.handlers import RotatingFileHandler
import os
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime, UTC
from typing import NamedTuple


class Snapshot(NamedTuple):
  stage: str
  stage_started: float
  last_publication: float
  slow_stages: tuple[tuple[str, float], ...] = ()


class HardwareWatchdog:
  STALL_SECONDS = 1.5
  REPEAT_SECONDS = 5.0
  SLOW_SECONDS = 0.25
  MAX_BYTES = 1024 * 1024
  BACKUP_COUNT = 2

  def __init__(self, path: str, clock=time.monotonic):
    self.path = path
    self.clock = clock
    self.snapshot: Snapshot | None = None
    self.thread_id: int | None = None
    self.native_id: int | None = None
    self.recoveries: deque[dict] = deque(maxlen=16)
    self.reported_publication: float | None = None
    self.next_dump = 0.0
    self.handler: RotatingFileHandler | None = None

  def arm(self) -> None:
    # Called by the publisher after initialization. Snapshot replacement is atomic under the
    # GIL; this thread never waits for a diagnostic lock or touches the diagnostic file.
    now = self.clock()
    self.thread_id = threading.get_ident()
    self.native_id = threading.get_native_id()
    self.snapshot = Snapshot('first_publication', now, now)

  def stage(self, name: str) -> None:
    previous = self.snapshot
    if previous is None:
      return
    now = self.clock()
    slow = previous.slow_stages
    duration = now - previous.stage_started
    if duration >= self.SLOW_SECONDS:
      slow = (*slow, (previous.stage, duration))[-16:]
    self.snapshot = Snapshot(name, now, previous.last_publication, slow)

  def published(self) -> None:
    previous = self.snapshot
    if previous is None:
      return
    now = self.clock()
    if now - previous.last_publication >= self.STALL_SECONDS:
      # Retain recovery even if the watchdog could not run while the GIL was held.
      self.recoveries.append(self._event('recovered', previous, now))
    self.snapshot = previous._replace(last_publication=now)

  def _event(self, event: str, snapshot: Snapshot, now: float) -> dict:
    return {'event': event, 'monotonic': now, 'utc': datetime.now(UTC).isoformat(), 'pid': os.getpid(),
            'publisher_thread': self.thread_id, 'publisher_native_thread': self.native_id,
            'stage': snapshot.stage, 'stage_started_monotonic': snapshot.stage_started,
            'last_publication_monotonic': snapshot.last_publication, 'stage_seconds': now - snapshot.stage_started,
            'publication_gap': now - snapshot.last_publication, 'slow_stages': snapshot.slow_stages}

  def poll(self) -> list[dict]:
    events = []
    while self.recoveries:
      events.append(self.recoveries.popleft())
    snapshot = self.snapshot
    if snapshot is None:
      return events
    now = self.clock()
    if now - snapshot.last_publication < self.STALL_SECONDS:
      self.reported_publication = None
      return events
    if self.reported_publication == snapshot.last_publication and now < self.next_dump:
      return events
    event = self._event('stalled', snapshot, now)
    names = {t.ident: (t.name, t.native_id) for t in threading.enumerate()}
    frames = sys._current_frames()
    # Include every Python thread, but bound the total stack text even with many threads.
    limit = min(8192, 128 * 1024 // max(1, len(frames)))
    event['threads'] = {
      str(tid): {'name': names.get(tid, ('unknown', None))[0], 'native_id': names.get(tid, ('unknown', None))[1],
                 'stack': ''.join(traceback.format_stack(frame, limit=32))[-limit:]}
      for tid, frame in frames.items()
    }
    del frames
    events.append(event)
    self.reported_publication = snapshot.last_publication
    self.next_dump = now + self.REPEAT_SECONDS
    return events

  def _write(self, event: dict) -> None:
    # Only the diagnostic thread performs file I/O. No fsync or message-bus dependency.
    if self.handler is None:
      os.makedirs(os.path.dirname(self.path), exist_ok=True)
      self.handler = RotatingFileHandler(self.path, maxBytes=self.MAX_BYTES,
                                         backupCount=self.BACKUP_COUNT, encoding='utf-8')
      self.handler.setFormatter(logging.Formatter('%(message)s'))
    self.handler.handle(logging.makeLogRecord({'msg': json.dumps(event), 'levelno': logging.INFO}))

  def run(self, end_event: threading.Event) -> None:
    started = False
    try:
      while not end_event.is_set():
        try:
          if not started:
            self._write({'event': 'watchdog_started', 'monotonic': self.clock(), 'utc': datetime.now(UTC).isoformat(),
                         'pid': os.getpid(), 'stall_seconds': self.STALL_SECONDS, 'repeat_seconds': self.REPEAT_SECONDS})
            started = True
          for event in self.poll():
            self._write(event)
        except Exception:
          # A diagnostic failure must not terminate hardwared or stop deviceState publication.
          if end_event.wait(5.):
            break
        if end_event.wait(0.25):
          break
    finally:
      if self.handler is not None:
        self.handler.close()
