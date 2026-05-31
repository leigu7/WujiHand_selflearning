"""
TASK 1d: Keyboard control WITH viewer.sync()

What to verify:
    - Press UP arrow → finger1_joint1 bends
    - Press DOWN arrow → finger1_joint1 straightens
    - Press Q → quit
"""

import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
import time

SIDE = "right"
JOINT_STEP = 0.05  # radians per keypress

mjcf_path = (Path(__file__).parent / "wuji_hand_description" / "mjcf" / f"{SIDE}.xml").resolve()
model = mujoco.MjModel.from_xml_path(str(mjcf_path))
data = mujoco.MjData(model)

TARGET_IDX = 0  # finger1_joint1

# Get ctrl range
if model.actuator_ctrllimited[TARGET_IDX]:
    ctrl_lo, ctrl_hi = model.actuator_ctrlrange[TARGET_IDX]
else:
    ctrl_lo, ctrl_hi = -1.57, 1.57

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
print("TASK 1d: Keyboard Control WITH viewer.sync()")
print("=" * 60)
print(f"Controlling joint {TARGET_IDX} (finger1_joint1)")
print(f"  UP arrow   → increase (+{JOINT_STEP} rad, max {ctrl_hi:.2f})")
print(f"  DOWN arrow → decrease (-{JOINT_STEP} rad, min {ctrl_lo:.2f})")
print(f"  Q          → quit")
print("=" * 60)


def key_callback(keycode):
    import glfw
    if keycode == glfw.KEY_UP:
        new_val = data.ctrl[TARGET_IDX] + JOINT_STEP
        data.ctrl[TARGET_IDX] = np.clip(new_val, ctrl_lo, ctrl_hi)
        print(f"  Ctrl={data.ctrl[TARGET_IDX]:+.3f}")
    elif keycode == glfw.KEY_DOWN:
        new_val = data.ctrl[TARGET_IDX] - JOINT_STEP
        data.ctrl[TARGET_IDX] = np.clip(new_val, ctrl_lo, ctrl_hi)
        print(f"  Ctrl={data.ctrl[TARGET_IDX]:+.3f}")
    elif keycode == glfw.KEY_Q or keycode == glfw.KEY_ESCAPE:
        pass  # viewer handles close


step = 0

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        step += 1

        if step % 500 == 0:
            print(f"Step {step:5d} | ctrl={data.ctrl[TARGET_IDX]:+.3f}  qpos={data.qpos[TARGET_IDX]:+.3f}")

        time.sleep(0.001)

print(f"Done. Steps: {step}")
