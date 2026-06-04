# soarm-remote-teleop

Teleoperate a remote LeRobot setup (e.g. a simulated SO-101 in Isaac Sim on a cloud GPU)
from a physical SO-100/SO-101 leader arm on your local machine, without installing
LeRobot/torch/Isaac locally.

The leader reading stays local; only calibrated joint values stream one-way to the remote,
where LeRobot reconstructs the exact leader action. The local side runs on Linux or macOS;
the remote is any SSH-reachable host.

Scope: leader -> remote, one-way. Driving a physical follower (round-trip) is out of scope.

## How it works

Tunneling the USB/serial device does not work: the Feetech bus is a synchronous
request/response protocol, so over a ~100-150 ms WAN round-trip every read blocks a full RTT
and trips the SDK timeouts. Streaming joint values keeps the protocol local, so network delay
becomes a small constant lag instead of per-read stalls.

```
[local]  leader USB --read--> soarm-local stream (TCP server)
                                  | ssh -R reverse tunnel (encrypted)
[remote] network_leader --decode+normalize via LeRobot bus--> leader.get_action()
```

## Install

Local machine (arm plugged in):

```
git clone <repo> && cd soarm-remote-teleop
./scripts/setup_local.sh          # venv + pip install -e ".[local]"  (pyserial, feetech-servo-sdk)
```

Remote machine (your LeRobot env):

```
pip install -e .                  # stdlib-only core; uses the LeRobot already in that env
```

Nothing is hardcoded: host, key, ports, serial port, and arm id are all arguments / env vars.

## Usage

1. Probe the arm. Find the port first (Linux: `ls /dev/ttyACM*`; macOS: `ls /dev/cu.usbmodem*`):

   ```
   soarm-local --port <PORT> probe
   ```

2. Calibrate. Writes homing offsets into the servos and emits a JSON. Power-cycle the arm
   first if a joint reads outside 0-4095 at center (clears multi-turn encoder state):

   ```
   soarm-local --port <PORT> calibrate --out leader_calibration.json
   ```

3. Deploy the calibration to the remote, named `<id>.json` (the id you pass as `--teleop.id`
   / `--robot_id`). Find the destination path with LeRobot:

   ```
   python -c "from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig; \
   print(SO101Leader(SO101LeaderConfig(port='/dev/null', id='<id>')).calibration_fpath)"
   ```

   It must match the homing offsets the calibrate run just wrote, so deploy that run's JSON.

4. Activate the network leader in your remote app (see "Activating" below).

5. Run:

   ```
   # local, terminal 1: stream the leader
   soarm-local --port <PORT> stream --port-tcp 5599
   # local, terminal 2: reverse SSH tunnel to the remote
   REMOTE_HOST=my.server REMOTE_KEY=~/.ssh/id_ed25519 ./scripts/tunnel.sh
   # remote: launch your normal LeRobot/Isaac teleop with the network leader active
   export LEADER_NET=127.0.0.1:5599
   ```

   Move the leader; the remote tracks it with a small constant lag. Unset `LEADER_NET` to
   return to normal local behavior.

## Activating the network leader (step 4)

Your remote app builds a LeRobot leader and calls `get_action()` in a loop. Make that leader
read from the network with one of the two options below. Both are gated on `LEADER_NET`, so
they are no-ops when it is unset. Both require the leader calibration on the remote (step 3).

Option A - class activator (recommended; works regardless of how the app builds the leader).
It must run before the leader's `connect()`/`get_action()` are called:

```
import os
from soarm_remote_teleop import attach_network_to_leader_class
if os.getenv("LEADER_NET"):
    from lerobot.teleoperators.so101_leader import SO101Leader
    attach_network_to_leader_class(SO101Leader, os.getenv("LEADER_NET"))
```

Put it at the top of your entry script, or in a launcher that runs your script unmodified:

```
# run_with_network_leader.py
import os, runpy
from soarm_remote_teleop import attach_network_to_leader_class
if os.getenv("LEADER_NET"):
    from lerobot.teleoperators.so101_leader import SO101Leader
    attach_network_to_leader_class(SO101Leader, os.getenv("LEADER_NET"))
runpy.run_path("path/to/your_agent.py", run_name="__main__")
```

Option B - instance wrap (if you control where the leader is built):

```
import os
from soarm_remote_teleop import wrap_leader_for_network
leader = make_your_lerobot_leader(...)
if os.getenv("LEADER_NET"):
    leader = wrap_leader_for_network(leader, os.getenv("LEADER_NET"))
```

Use A or B, not both.

## Measure tunnel latency (optional)

Separate from the teleop path; run instead of `stream`:

```
# local
soarm-local --port <PORT> latency-server --port-tcp 5599
./scripts/tunnel.sh
# remote
python -m soarm_remote_teleop.network_leader --addr 127.0.0.1:5599 --ping 100
```

Reports RTT min/median/p95/max; one-way lag is about median/2.

## Layout

```
src/soarm_remote_teleop/
  protocol.py        wire frame + SO-arm motor table + sign codecs (stdlib only)
  feetech_bus.py     local STS3215 bus (scservo_sdk)
  local.py           CLI: probe / calibrate / stream / latency-server
  network_leader.py  remote adapters + transport selftest + latency ping
scripts/             setup_local.sh, tunnel.sh
```

## Scope and limits

- SO-100 / SO-101 only (6x Feetech STS3215); the motor table lives in `protocol.py`.
- Reuses LeRobot bus internals (`_decode_sign` / `_normalize`), matched to LeRobot 0.4.3.
- Calibration is per-arm; each user generates their own (calibration JSONs are gitignored).
- The activator must run before the leader's methods are called; class patching also affects
  instances built later.
