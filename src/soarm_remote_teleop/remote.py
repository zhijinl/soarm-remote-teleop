#!/usr/bin/env python3
"""Run a stock LeRobot CLI with the network leader active (CLI-based remote deployment).

The stock `lerobot-teleoperate` / `lerobot-record` CLIs build the leader internally, so there
is no in-process hook to inject the network leader. This launcher activates it (by
class-patching the SO leader) and then hands off to the named LeRobot console script, passing
all remaining arguments through unchanged. Gated on REMOTE_LEADER (a no-op when unset, so it
behaves exactly like the wrapped CLI).

    REMOTE_LEADER=127.0.0.1:5599 soarm-remote lerobot-teleoperate \
        --teleop.type=so101_leader --teleop.id=<id> \
        --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=<id>

Works for any LeRobot console script that builds an SO leader (teleoperate, record, ...).
Requires the leader calibration on this machine, keyed by the leader id (same as any other
deployment).
"""
from __future__ import annotations

import sys


def _activate_network_leader() -> None:
    from soarm_remote_teleop import activate_network_leader

    activate_network_leader()  # reads REMOTE_LEADER; no-op if unset


def _resolve_console_script(name: str):
    import importlib.metadata as md

    for ep in md.entry_points(group="console_scripts"):
        if ep.name == name:
            return ep.load()
    raise SystemExit(f"soarm-remote: console script '{name}' not found in this environment.")


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit(
            "usage: soarm-remote <lerobot-cli> [args...]\n"
            "  e.g. REMOTE_LEADER=127.0.0.1:5599 soarm-remote lerobot-teleoperate "
            "--teleop.type=so101_leader ..."
        )
    _activate_network_leader()
    cli_name = sys.argv[1]
    func = _resolve_console_script(cli_name)
    sys.argv = sys.argv[1:]  # the wrapped CLI sees itself as argv[0], its args as argv[1:]
    sys.exit(func())


if __name__ == "__main__":
    main()
