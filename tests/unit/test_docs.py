"""The documentation rules of CODING_PROMPT.md, enforced: every doc under
docs/ is indexed from docs/README.md, stays under the 600-word cap, and has
no em-dash. The docs that were over the cap before the rule applied are
listed as known exceptions until each is split; a new doc cannot join that
list.

The how-to for adding a policy profile must name every registration point
the profile tests check, so a fresh reader following it is led to each one.
"""

from __future__ import annotations

import glob
import os

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_DOCS = os.path.join(_REPO_ROOT, "docs")
WORD_CAP = 600

# Over the cap before the rule was enforced (2026-11-04). Shrinking this list
# is the only allowed change.
OVER_CAP_BEFORE_THE_RULE = {
    "architecture.md", "calibration.md", "deployment.md",
    "interface_contract_3.0.0_superseded.md", "operating_guide.md",
    "safety_validation.md",
}


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def _docs():
    return sorted(os.path.basename(p) for p in glob.glob(os.path.join(_DOCS, "*.md"))
                  if os.path.basename(p) != "README.md")


@pytest.mark.parametrize("name", _docs())
def test_every_doc_is_indexed(name) -> None:
    assert f"]({name})" in _read(os.path.join(_DOCS, "README.md")), (
        f"docs/{name} is not linked from docs/README.md; add its row to the index"
    )


@pytest.mark.parametrize("name", _docs())
def test_every_doc_is_under_the_cap_or_a_known_exception(name) -> None:
    words = len(_read(os.path.join(_DOCS, name)).split())
    if name in OVER_CAP_BEFORE_THE_RULE:
        pytest.skip(f"{name}: {words} words, over the cap before the rule; split it along Diátaxis lines")
    assert words <= WORD_CAP, f"docs/{name} is {words} words; the cap is {WORD_CAP} (wc -w)"


def test_the_exception_list_names_only_docs_that_exist() -> None:
    assert OVER_CAP_BEFORE_THE_RULE <= set(_docs())


@pytest.mark.parametrize("name", [n for n in _docs() if n not in OVER_CAP_BEFORE_THE_RULE])
def test_docs_have_no_em_dash(name) -> None:
    assert "—" not in _read(os.path.join(_DOCS, name)), f"docs/{name} has an em-dash"


def test_the_robot_how_to_names_every_fact_the_profile_holds() -> None:
    """What the robot profile tests pin, the how-to must send the reader to."""
    text = _read(os.path.join(_DOCS, "add_a_robot.md"))
    for needle in (
        "config/robots/<name>.yaml", "hostnames", "ros_domain_id", "model", "yaw_deg",
        "self_occlusion_zones_deg", "body_floor_m", "body_radius_m", "calibration",
        "encoder_sign", "robot_profile.py", "test_robot_profile.py", "scan_view.py",
        "per_motor.py", "calibration.md", "run_stack.sh", "deploy_to_robot.sh",
        "sign_fault", "ROS_DOMAIN_ID",
    ):
        assert needle in text, f"add_a_robot.md does not mention {needle!r}"


def test_the_profile_how_to_names_every_registration_point() -> None:
    """What the profile tests pin, the how-to must send the reader to."""
    text = _read(os.path.join(_DOCS, "add_a_policy_profile.md"))
    for needle in (
        "export_from_sb3.py", "max action diff vs SB3",
        "config/interface", "_SUPPORTED_INTERFACE_VERSIONS", "action.layout",
        "config/topics.yaml", "command_sources", "_PROFILES", "_DEFAULT_PROFILES",
        "deploy_upstream.sh", "CRITICAL", "record_run.sh",
        "test_lidar_contract.py", "test_source_per_policy_launch.py",
        "test_command_sources.py", "test_dashboard_sources.py", "test_action_contract.py",
        "interface_versions.md", "deploy_to_robot.sh", "/tmp/ncp_stack.log",
    ):
        assert needle in text, f"add_a_policy_profile.md does not mention {needle!r}"
