"""Guard: vendored policy code must remain a BYTE-EXACT copy of upstream.

Operator requirement, 2026-08-27: the policy code is to be as similar to -- if
not an exact copy of -- what is in the NCP-on-Rosmaster repo. The whole value
of that is that it is *checkable*, so this checks it.

There are deliberately ZERO local divergences. The TG30 no-return fix that used
to live in `prepare_scan` was moved into `rm_policy.scan_resampler`, which is
our own code, precisely so this file could stay pristine. See vendor/VENDOR.md.

If this test fails, someone edited vendored code. Revert it and fix the problem
in our own modules instead -- that is the rule this test exists to enforce.

Pure stdlib: runs in the workstation tier. Skips only when no upstream checkout
is present (e.g. on the robot, which has no sibling clone).
"""

from __future__ import annotations

import hashlib
import os

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# Sibling checkout on the dev workstation. Not present on the robot.
_UPSTREAM = os.path.abspath(
    os.path.join(_REPO_ROOT, "..", "NCP-on-Rosmaster", "rosmaster", "policy.py")
)
_VENDORED = os.path.join(
    _REPO_ROOT, "src", "rm_policy", "rm_policy", "vendor", "ncp_policy.py"
)

# Recorded at vendoring time, from upstream commit
# bc1099d43831db3b228b2b7ea3baf4e267b9dc4c. This pins the copy even where no
# upstream checkout exists, so the robot tier still catches local edits.
#
# RE-VENDORED 2026-10-08 (was 69785c8 / sha da3830aa...) for interface 4.0.0:
# BEAMS 32 -> 720 and FOV_DEG 180 -> 270, the native BARN sensor layout the
# M7/M8 runs trained on. The actor architecture is unchanged; only its input
# width follows OBS_DIM (36 -> 724). weights/actor.pt is now the M8-v2
# snapshot s1436792. A 3.0.0 checkpoint cannot load into this shape (strict
# load_state_dict raises at startup), which is the intended behaviour.
#
# RE-VENDORED 2026-08-29 (was 0ad548c / sha 37c8cf2f...). The upstream actor
# architecture CHANGED when the trained checkpoint landed: `head` went from
# `nn.Linear(motor, 2)` to the SB3 stack `Linear-Tanh-Linear-Tanh-Linear`, and
# `obs_mean`/`obs_var` buffers were added for the checkpoint's VecNormalize
# statistics.
_EXPECTED_SHA256 = "784df5f455109fd3363f276cf369d2804449da79016c1e9b4d7ad76e211c0d71"


def _sha256(path: str) -> str:
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def test_vendored_policy_matches_recorded_upstream_hash() -> None:
    """Runs everywhere, including the robot. No upstream checkout needed."""
    assert os.path.isfile(_VENDORED), f"vendored policy missing: {_VENDORED}"
    actual = _sha256(_VENDORED)
    assert actual == _EXPECTED_SHA256, (
        "vendored ncp_policy.py has been MODIFIED.\n"
        f"  expected {_EXPECTED_SHA256}\n"
        f"  actual   {actual}\n"
        "Vendored code must stay byte-exact with upstream (see vendor/VENDOR.md).\n"
        "Fix the problem in our own modules -- scan_resampler, observation_builder,\n"
        "policy_node -- not in the vendored copy."
    )


def test_vendored_policy_is_byte_identical_to_upstream_checkout() -> None:
    """Stronger check, where a sibling upstream clone is available."""
    if not os.path.isfile(_UPSTREAM):
        pytest.skip(f"no upstream checkout at {_UPSTREAM}")
    with open(_UPSTREAM, "rb") as a, open(_VENDORED, "rb") as b:
        up, ven = a.read(), b.read()
    assert ven == up, (
        "vendored ncp_policy.py differs from the upstream working copy.\n"
        "Either vendored code was edited (revert it), or upstream moved on "
        "(re-vendor deliberately and update _EXPECTED_SHA256 here and the "
        "commit recorded in vendor/VENDOR.md)."
    )


def test_no_local_divergence_markers_in_vendored_tree() -> None:
    """The previous revision carried a 'LOCAL DIVERGENCE' block. Ensure none returns."""
    with open(_VENDORED, "r", encoding="utf-8") as fh:
        text = fh.read()
    for marker in ("LOCAL DIVERGENCE", "DIVERGES FROM UPSTREAM", "TG30_RANGE_MIN_M"):
        assert marker not in text, (
            f"vendored file contains {marker!r} -- it is no longer a pristine copy"
        )
