#!/usr/bin/env python3
"""Validate a running system (or a bag) against config/interface.yaml.

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

STATUS: stub. Implemented in Phase 3 task 4, once the schema loader
(rm_interfaces) exists and Phase 1 has established the measured rates that
interface.yaml declares.
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
