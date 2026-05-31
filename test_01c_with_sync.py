"""
TASK 1c: Test with viewer.sync() like the original run_sim.py.
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
print("TASK 1c: Test with viewer.sync()")
print("=" * 60)
print("The script cycles finger1_joint1 with a sine wave.")
print("KEY DIFFERENCE: uses viewer.sync() after each step")
print("=" * 60)

step = 0
phase = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]

    while viewer.is_running():
        phase += 0.05
        data.ctrl[0] = 0.8 * np.sin(phase)

        mujoco.mj_step(model, data)
        viewer.sync()  # <-- THIS is what run_sim.py does!
        
        step += 1

        if step % 100 == 0:
            print(f"Step {step:5d} | ctrl={data.ctrl[0]:+.3f}  qpos={data.qpos[0]:+.3f}")

        time.sleep(0.001)

print(f"\nDone. Steps: {step}")
