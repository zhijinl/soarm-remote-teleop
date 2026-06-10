"""soarm-remote-teleop — remote teleoperation for LeRobot SO-100/SO-101 arms.

The local machine (arms plugged in) keeps the Feetech serial protocol local and streams
calibrated joint values over the network; the remote machine (LeRobot / Isaac Sim) runs the
robot logic and reconstructs the exact LeRobot action/observation. See README.md and
INTEGRATION.md.

Core (this top-level import) is stdlib-only. The local CLI additionally needs
`pyserial` + `feetech-servo-sdk`; the remote side additionally needs LeRobot.
"""
from .protocol import MOTORS, IDS, ID_TO_NAME  # noqa: F401
from .network_leader import (  # noqa: F401
    wrap_leader_for_network,
    attach_network_to_leader_class,
    activate_network_leader,
    leader_calibration_path,
)

__version__ = "0.1.0"
