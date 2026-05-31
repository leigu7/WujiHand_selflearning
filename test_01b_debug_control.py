"""
TASK 1b: Debug why ctrl changes don't appear in viewer.

Tests:
    1. Directly set data.ctrl in the main loop (not callback)
    2. Check if data.qpos changes after mj_step
"""

import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
import time

SIDE = "right"

mjcf_path = (Path(__file__).parent / "wuji_hand_description" / "mjcf" / f"{SIDE}.xml").resolve()
model = mujoco.MjModel.from_xml_path(str(mjcf_path))
data = mujoco.MjData(model)

actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
TARGET_IDX = 0

# Set all to mid-range
for i in range(model.nu):
    if model.actuator_ctrllimited[i]:
        r = model.actuator_ctrlrange[i]
        data.ctrl[i] = (r[0] + r[1]) / 2
    else:
        data.ctrl[i] = 0.0

for _ in range(100):
    mujoco.mj_step(model, data)

print("=" * 60)
print("TASK 1b: Debug - Direct ctrl in main loop")
print("=" * 60)
print(f"Controlling: {actuator_names[TARGET_IDX]}")
print("The script will cycle through ctrl values automatically.")
print("Watch the finger in the viewer - it should move.")
print("=" * 60)

step = 0
phase = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]

    while viewer.is_running():
        # Directly set ctrl in main loop with a sine wave
        phase += 0.01
        data.ctrl[TARGET_IDX] = 0.5 * np.sin(phase)

        mujoco.mj_step(model, data)
        step += 1

        if step % 200 == 0:
            print(f"Step {step:5d} | ctrl={data.ctrl[TARGET_IDX]:+.3f}  qpos={data.qpos[TARGET_IDX]:+.3f}")

        time.sleep(0.001)

print(f"\nDone. Steps: {step}")
