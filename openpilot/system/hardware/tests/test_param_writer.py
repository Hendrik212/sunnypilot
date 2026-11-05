import threading

from openpilot.system.hardware.param_writer import ParamWriter


def test_submit_returns_while_a_write_is_stuck():
  writer, end = ParamWriter(), threading.Event()
  gate, done = threading.Event(), []
  t = threading.Thread(target=writer.run, args=(end,), daemon=True)
  t.start()
  writer.submit(lambda: (gate.wait(5.), done.append("remove")))
  writer.submit(done.append, "put")  # queued behind the stuck one, submit does not wait
  assert done == []
  gate.set()
  end.set()
  t.join(5.)
  assert done == ["remove", "put"]


def test_writes_queued_before_the_end_still_run_in_order():
  writer, end = ParamWriter(), threading.Event()
  done = []
  for k in ("IsEngaged", "UptimeOnroad", "DoShutdown"):
    writer.submit(done.append, k)
  end.set()
  writer.run(end)
  assert done == ["IsEngaged", "UptimeOnroad", "DoShutdown"]


def test_a_failing_write_does_not_stop_the_next():
  writer, end = ParamWriter(), threading.Event()
  done = []
  writer.submit(lambda: 1 / 0)
  writer.submit(done.append, "next")
  end.set()
  writer.run(end)
  assert done == ["next"]
