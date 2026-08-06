import pytest

try:
  from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_lag_adjusted_curvature
except ModuleNotFoundError:
  # On this Windows PC, git checked out the `openpilot -> .` symlinks (common, selfdrive, ...)
  # as plain text files (core.symlinks=false), so `openpilot.<pkg>` submodules don't resolve.
  # Work around it with a synthetic namespace package rooted at the real repo root. This only
  # affects test collection here; the source files under selfdrive/ are untouched.
  #
  # `openpilot.common.realtime` additionally pulls in `openpilot.system.hardware` -> `cereal`,
  # and cereal's capnp schemas cannot be built on this PC (pycapnp). Stub that one leaf module
  # with its real constant values (DT_CTRL/DT_MDL, unchanged from common/realtime.py) so the
  # rest of drive_helpers.py's real imports resolve normally.
  import os, sys, types
  repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
  # The failed import above already registers the real (broken) `openpilot` package in
  # sys.modules before failing on `.selfdrive` (it resolves via the nested openpilot/openpilot/
  # dir, whose __path__ doesn't contain selfdrive/common/etc as real dirs). Force its __path__
  # to the actual repo root so submodule lookups (openpilot.selfdrive, openpilot.common, ...)
  # find the real packages instead.
  pkg = sys.modules.get("openpilot") or types.ModuleType("openpilot")
  pkg.__path__ = [repo_root]
  sys.modules["openpilot"] = pkg
  if "openpilot.common.realtime" not in sys.modules:
    stub = types.ModuleType("openpilot.common.realtime")
    stub.DT_CTRL = 0.01
    stub.DT_MDL = 0.05
    sys.modules["openpilot.common.realtime"] = stub
  from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_lag_adjusted_curvature


class _CP:
  class _CI: pass
  steerActuatorDelay = 0.2


# NOTE: get_lag_adjusted_curvature resets psis/curvatures/distances to all-zero whenever
# len(psis) != CONTROL_N (a safety guard against malformed model-plan input). CONTROL_N is 17
# here (matches carrot's original), not 10, so fixtures must be CONTROL_N-length or that guard
# always fires and every test degenerates to the zero-curvature case. The brief's Step 2 sample
# used len==10 fixtures; sized up to CONTROL_N here to actually exercise the ported logic.
def test_zero_curvature_stays_zero():
  psis = [0.0]*CONTROL_N
  curvs = [0.0]*CONTROL_N
  dists = [float(i) for i in range(CONTROL_N)]
  out = get_lag_adjusted_curvature(_CP(), 25.0, psis, curvs, 0.2, dists)
  assert abs(out) < 1e-6


def test_constant_curvature_preserved_sign():
  psis = [0.01*i for i in range(CONTROL_N)]
  curvs = [0.01]*CONTROL_N
  dists = [float(i) for i in range(CONTROL_N)]
  out = get_lag_adjusted_curvature(_CP(), 25.0, psis, curvs, 0.2, dists)
  assert out > 0
