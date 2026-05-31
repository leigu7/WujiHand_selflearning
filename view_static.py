#!/usr/bin/env python3
"""Show the Wuji Hand in a static pose (no motion)."""
import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np

side = "right"

# Load the model
mjcf_path = (Path(__file__).parent / "wuji_hand_description" / "mjcf" / f"{side}.xml").resolve()
model = mujoco.MjModel.from_xml_path(str(mjcf_path))
data = mujoco.MjData(model)

# Set default pose: middle position for all joints
for i in range(model.nu):
    if model.actuator_ctrllimited[i]:
        ctrl_range = model.actuator_ctrlrange[i]
        data.ctrl[i] = (ctrl_range[0] + ctrl_range[1]) / 2  # middle position
    else:
        data.ctrl[i] = 0.0

# Step a few times to settle
for _ in range(100):
    mujoco.mj_step(model, data)

print("Wuji Hand - Static pose")
print("Close the viewer window to exit.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]
    
    while viewer.is_running():
        # Keep the same pose, just step physics
        mujoco.mj_step(model, data)
        viewer.sync()


