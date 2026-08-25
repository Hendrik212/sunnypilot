import re
import subprocess
import tempfile
from pathlib import Path

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized

import openpilot.cereal.services as services
from openpilot.cereal.services import SERVICE_LIST

# Files that build a SubMaster/PubMaster from an `*_services_ext` list. A service referenced
# here but absent from services.py crashes SubMaster.__init__ with a hard KeyError at boot --
# the 2026-08-25 static-logo regression, where lateralTuneStateSP was published and subscribed
# but never registered. test_services only iterates SERVICE_LIST.keys(), so it cannot see an
# unregistered service; this scan closes that gap.
_EXT_SERVICE_SOURCE_FILES = [
  "openpilot/selfdrive/ui/sunnypilot/ui_state.py",
  "openpilot/sunnypilot/selfdrive/controls/controlsd_ext.py",
]


def _ext_service_refs() -> set[str]:
  refs: set[str] = set()
  repo_root = Path(services.__file__).parents[2]
  for rel in _EXT_SERVICE_SOURCE_FILES:
    txt = (repo_root / rel).read_text()
    for m in re.finditer(r'(?:sm|pm)_services_ext\s*=\s*\[([^\]]*)\]', txt, re.S):
      refs.update(re.findall(r'"([A-Za-z0-9_]+)"', m.group(1)))
  return refs


class TestServices(OpenpilotTestCase):

  @parameterized.expand(SERVICE_LIST.keys())
  def test_services(self, s):
    service = SERVICE_LIST[s]
    assert service.frequency <= 104
    assert service.decimation != 0

  def test_generated_header(self):
    with tempfile.NamedTemporaryFile(suffix=".h") as f:
      ret = subprocess.run(f"python3 {services.__file__} > {f.name} && clang++ {f.name} -std=c++11", shell=True).returncode
      assert ret == 0, "generated services header is not valid C"

  def test_ext_services_registered(self):
    """Every service referenced in an `*_services_ext` list must be registered in services.py.

    SubMaster.__init__ reads SERVICE_LIST[s].frequency with a direct subscript, so an
    unregistered service raises KeyError at construction. The UI builds its SubMaster at boot,
    so one missing entry crashes the UI to a static comma logo before any alert can show.
    """
    missing = sorted(s for s in _ext_service_refs() if s not in SERVICE_LIST)
    assert not missing, f"ext services referenced but not registered in services.py: {missing}"
