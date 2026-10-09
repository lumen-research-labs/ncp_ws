#!/usr/bin/env python3
"""NOT IMPLEMENTED — a stub. Do not rely on this script.

!!  THIS SCRIPT VALIDATES NOTHING.  !!

It exits non-zero with a "not implemented" message. It is kept as a
specification of the intended tool, not as a tool. The docstring below
describes what it WOULD do; nothing under it has been built.

Do not put it in a pre-session checklist, and do not read a passing exit
status from it as evidence about the interface — there is no passing exit
status. Checking the contract today means reading config/interface.yaml and
docs/interface_contract.md, and confirming interface_version at policy_node
startup (which DOES check, and refuses to start on a mismatch).

--- intended behaviour, unbuilt ---

Validate a running system (or a bag) against config/interface.yaml.

Asserts that every topic declared in the interface contract exists, publishes
the declared message type, and runs at or above the declared rate. Run this
before every hardware session — a silently renamed or slowed topic is the
cheapest possible failure to catch here and the most expensive to catch on the
floor.

Subscribes:  every topic declared in config/interface.yaml (introspection only)
Publishes:   nothing
Parameters:  --interface (path, default config/interface.yaml)
             --bag (path, optional — check a recorded bag instead of a live system)
             --duration (float, default 5.0) — sampling window for rate checks
Failure mode: exits non-zero with the list of violations. Never exits zero on a
             partial check; an unreachable topic is a failure, not a skip.

STATUS: STILL A STUB as of 2026-08-30. It was scheduled for Phase 3 task 4,
gated on the rm_interfaces schema loader and on Phase 1 measured rates being
written into config/interface.yaml. Both of those happened; this did not.
It exits 1, so it can never be mistaken for a passing check.
"""

import sys


def main() -> int:
    print(
        "check_interface.py is not implemented yet — Phase 3 task 4.\n"
        "It depends on the rm_interfaces schema loader and on Phase 1 measured\n"
        "rates being written into config/interface.yaml.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
