# soarm-remote-teleop

Teleoperate a remote SO Arm follower using a local SO Arm leader:

- Local side requires minimal dependencies: only `python3`, `pyserial` and `feetech-servo-sdk`. Tested on Linux, Mac and Windows 11 (native, not WSL2).
- Remote side requires teleop application-specific dependencies (e.g. `lerobot`). Remote can be any `ssh`-accessible instance: cloud, NVIDIA Brev, workstation, etc. Remote SO Arm follower can be simulated (e.g. in Isaac Sim), or a physical one connected to the remote PC.

The leader is read locally and its joint values streamed over SSH to the remote, where the teleop loop runs. A near-constant delay is expected, primarily due to network latency.

## Install

### Local

Clone the repo:

```bash
git clone https://github.com/zhijinl/soarm-remote-teleop.git
```

Install the lib. The following example shows installation using `venv`. But you can also use `uv`.

```bash
cd soarm-remote-teleop

python3 -m venv .venv
source .venv/bin/activate # Windows: .venv\Scripts\activate

# Inside .venv
python3 -m pip install --upgrade pip
python3 -m pip install -e ".[local]"
```

### Remote

Clone the repo, and install it under the same env where you run teleop (with `lerobot` and dependencies):

```bash
git clone https://github.com/zhijinl/soarm-remote-teleop.git

pip install -e .
```

## Usage

### Local

If you already have a working calibration file for the leader arm and you already know which port the leader is connected to, you can skip steps 1 & 2 and jump directly to step 3.

1. Find out which port the SO Arm leader is connected to. You can use `pyserial` (part of the dependencies) to inspect ports, which works on Windows, Linux and Mac.

   ```bash
   python -m serial.tools.list_ports -v
   ```

   On Linux the port usually is one of `/dev/ttyACM*`; macOS: `/dev/cu.usbmodem*`; Windows `COM*`. You can probe the port using the command below:

   ```bash
   soarm-leader --teleop-port <PORT> probe
   ```

   A successful probe should return leader arm's raw ticks for each joint and print something like below:

   ```text
   Opened /dev/cu.usbmodem5AB01576901 @ 1000000 baud
     id  1 shoulder_pan   OK  model=777
     id  2 shoulder_lift  OK  model=777
     id  3 elbow_flex     OK  model=777
     id  4 wrist_flex     OK  model=777
     id  5 wrist_roll     OK  model=777
     id  6 gripper        OK  model=777
   Present_Position (raw ticks):
     shoulder_pan   = 1987
     shoulder_lift  = 856
     elbow_flex     = 3091
     wrist_flex     = 2815
     wrist_roll     = 920
     gripper        = 1554
   ```

2. Calibrate the leader arm with the following command - this runs an interactive `lerobot` equivalent calibration:

   ```bash
   soarm-leader --teleop-port <PORT> calibrate --out leader_calibration.json
   ```

   Follow the prompts: pose to center and ENTER, then sweep each joint and ENTER. If a joint reads out of range at center, power-cycle the arm and retry. Refer to this [video](https://huggingface.co/docs/lerobot/so101#calibration-video) for how to do the calibration.

   The generated `leader_calibration.json` needs to be deployed to remote later.

3. Open a terminal, stream the leader with the following command:

   ```bash
   soarm-leader --teleop-port <PORT> stream --port-tcp 5599
   ```

   In case of `permission denied` error on the leader port, run `sudo chmod 666 <PORT>`.

4. Open a new terminal, reverse SSH tunnel to the remote with the following command:

   ```bash
   ssh -i <REMOTE_KEY> -N -o ServerAliveInterval=15 -o ExitOnForwardFailure=yes \
       -R 5599:127.0.0.1:5599 <REMOTE_USER>@<REMOTE_HOST>
   ```

   Here we use `5599` for the streaming port, but this can be customized. Omit `-i <REMOTE_KEY>` to use your default SSH key / agent. Use `127.0.0.1` (not `localhost`) as the forward target: on Windows `localhost` may resolve to IPv6 `::1` and miss the IPv4 streamer; on Mac/Linux the two are equivalent.

### Remote

1. Deploy your leader calibration file (`leader_calibration.json`) to the remote. Rename it to `<id>.json`, where `<id>` is the value for `--teleop.id` in `lerobot` calibration.

   Move / copy this calibration file to where `lerobot` expects it for teleop. You can find this path with the following command:

   ```bash
   python -c "from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig; \
   print(SO101Leader(SO101LeaderConfig(port='/dev/null', id='<id>')).calibration_fpath)"
   ```

2. Run the `lerobot` teleop application. We need to patch the application to construct an `SO101Leader` class from network streamed bus values, instead of from serial USB. The patch depends on how you are launching the teleop application:

   - **Launching via CLI `lerobot-teleoperate`**

   Simply prepend the CLI as follows:

   `REMOTE_LEADER=127.0.0.1:5599 soarm-remote lerobot-teleoperate ...`

   - **Launching via `lerobot` Python APIs**

   Put the following at the beginning of your entry Python script, or anywhere before the leader class is constructed:

   ```python
   from soarm_remote_teleop import activate_network_leader
   activate_network_leader()  # no-op unless REMOTE_LEADER is set; finds the leader class itself
   ```

   Then run the entry script (assuming it to be `teleop.py` here):

   ```bash
   export REMOTE_LEADER=127.0.0.1:5599
   python3 teleop.py
   ```

   > **Note**:
   > 1. `export REMOTE_LEADER=...` is needed and will serve as a gate for whether the leader arm class takes network streamed values or serial USB values. Unsetting this env var returns to the normal behavior, where a physically connected leader is expected for teleop.
   > 2. Start the streamer and tunnel before launching remote teleop (the remote auto-connects/retries).
   > 3. The integration uses LeRobot internal APIs; verified with LeRobot 0.4.3 and 0.5.x. A future LeRobot refactor may require a small update.

### Testing

Once local and remote are set up, move the leader locally and the remote should track it with a lag that depends on your network connection.
