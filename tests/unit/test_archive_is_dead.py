"""archive/ holds retired code and docs and is not part of the codebase.

The cruft rule (Daniel's CODING_PROMPT.md): archived items stay readable but
nothing builds, imports, tests or links them, and every item is indexed in
archive/README.md. This pins all of that so cruft cannot creep back in
through a stale link or a forgotten package.xml.
"""

from __future__ import annotations

import os
import re

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_ARCHIVE = os.path.join(_REPO_ROOT, "archive")
_LIVE_DIRS = ("src", "tests", "scripts", "config")
_CURRENT_DOCS = ("README.md", "docs", os.path.join("src", "rm_safety"), os.path.join("src", "rm_policy"),
                 os.path.join("src", "rm_base"), os.path.join("src", "rm_bringup"),
                 os.path.join("src", "rm_dashboard"), os.path.join("src", "rm_waypoint"),
                 os.path.join("src", "rm_interfaces"))


def _files(root: str, suffixes):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in ("__pycache__", ".git", "build", "install", "log")]
        for f in filenames:
            if f.endswith(suffixes):
                yield os.path.join(dirpath, f)


def _archived_paths():
    out = []
    for dirpath, _dirnames, filenames in os.walk(_ARCHIVE):
        for f in filenames:
            rel = os.path.relpath(os.path.join(dirpath, f), _ARCHIVE)
            if rel not in ("README.md", "COLCON_IGNORE"):
                out.append(rel)
    return out


def test_colcon_never_sees_the_archive() -> None:
    assert os.path.isfile(os.path.join(_ARCHIVE, "COLCON_IGNORE"))


def test_every_archived_item_is_indexed() -> None:
    with open(os.path.join(_ARCHIVE, "README.md"), "r", encoding="utf-8") as fh:
        index = fh.read()
    for rel in _archived_paths():
        # An item is indexed by its own path or by the directory it lives in.
        parts = rel.split(os.sep)
        candidates = [rel] + [os.sep.join(parts[:i]) + "/" for i in range(1, len(parts))]
        assert any(c in index for c in candidates), f"{rel} is not listed in archive/README.md"


def test_nothing_live_imports_archived_code() -> None:
    offenders = []
    for d in _LIVE_DIRS:
        for path in _files(os.path.join(_REPO_ROOT, d), (".py", ".sh", ".xml", ".cfg", ".txt")):
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                text = fh.read()
            if re.search(r"^\s*(from|import)\s+rm_tools\b", text, re.M) or "archive/" in text \
                    and not path.endswith("test_archive_is_dead.py"):
                offenders.append(os.path.relpath(path, _REPO_ROOT))
    assert offenders == [], offenders


def test_no_current_doc_links_to_an_archived_item_by_its_old_path() -> None:
    offenders = []
    for d in _CURRENT_DOCS:
        root = os.path.join(_REPO_ROOT, d)
        paths = [root] if os.path.isfile(root) else list(_files(root, (".md",)))
        for path in paths:
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                text = fh.read()
            for old in ("src/rm_tools/", "scripts/check_interface.py",
                        "](interface_contract_2.0.0_superseded.md)", "](phase1_session_2026-08-08.md)"):
                if old in text:
                    offenders.append((os.path.relpath(path, _REPO_ROOT), old))
    assert offenders == [], offenders
