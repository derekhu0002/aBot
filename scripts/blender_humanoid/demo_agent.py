"""Demo: an agent on the local machine drives the Blender humanoid twin.

Prerequisite: the twin server is running in Blender, e.g.
    blender --python scripts/blender_humanoid/twin_server.py
    (or headless:  blender -b -P scripts/blender_humanoid/twin_server.py)

Run:
    python scripts/blender_humanoid/demo_agent.py
"""

import math
import sys
import time
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from twin_client import TwinClient


def main():
    twin = TwinClient()
    print("health:", twin.health())

    # 1. static poses
    for pose in ("relax", "tpose", "apose", "relax"):
        twin.set_pose(pose)
        time.sleep(1.0)

    # 2. time-based motion (wave)
    print("wave...")
    twin.start_motion("wave", 4.0)
    time.sleep(4.0)

    # 3. raw FK drive: raise both arms to the sides, then fold one forearm.
    # (Side convention, see humanoid_control.py: '.R' bones sit at world +X =
    # the robot's anatomical LEFT side; '.L' bones at world -X = anatomical
    # right. The calibrated anatomical-right wave is the "wave" motion above.)
    print("raw FK drive...")
    twin.drive_bones({
        "upper_arm.L": [0.0, 0.0, math.radians(90)],
        "upper_arm.R": [0.0, 0.0, math.radians(-90)],
    })
    time.sleep(1.0)
    twin.drive_bones({
        "upper_arm.R": [0.0, 0.0, math.radians(-90)],
        "forearm.R": [0.0, 0.0, math.radians(180)],
    })
    time.sleep(1.0)

    # 4. read back the model state (bone rotations)
    state = twin.get_state()
    print("state.motion:", state.get("motion"))
    print("state.bones:", {k: v for k, v in list(state.get("bones", {}).items())
                           if "upper_arm" in k or "forearm" in k})

    # 5. stop, back to relax
    twin.stop()
    twin.set_pose("relax")
    print("done. twin back to relax.")


if __name__ == "__main__":
    main()
