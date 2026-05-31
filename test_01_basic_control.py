"""
TASK 1: Basic MuJoCo viewer with manual arrow key control.

What to verify:
    - Open viewer with hand in mid-range
    - Press UP arrow → finger1_joint1 should bend
    - Press DOWN arrow → finger1_joint1 should straighten
    - Press Q → close viewer
"""

import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
import time

SIDE = "right"
JOINT_STEP = 0.05  # radians per keypress

# Load model
mjcf_path = (Path(__file__).parent / "wuji_hand_description" / "mjcf" / f"{SIDE}.xml").resolve()
model = mujoco.MjModel.from_xml_path(str(mjcf_path))
data = mujoco.MjData(model)

# Get actuator names
actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]

# We'll control only joint index 0 = finger1_joint1
TARGET_JOINT_IDX = 0
TARGET_JOINT_NAME = actuator_names[TARGET_JOINT_IDX]

# Get ctrl range for finger1_joint1
if model.actuator_ctrllimited[TARGET_JOINT_IDX]:
    ctrl_lo, ctrl_hi = model.actuator_ctrlrange[TARGET_JOINT_IDX]
else:
    ctrl_lo, ctrl_hi = -1.57, 1.57

print(f"Target joint: {TARGET_JOINT_NAME}")
print(f"Control range: [{ctrl_lo:.3f}, {ctrl_hi:.3f}]")

# Set all joints to mid-range
for i in range(model.nu):
    if model.actuator_ctrllimited[i]:
        r = model.actuator_ctrlrange[i]
        data.ctrl[i] = (r[0] + r[1]) / 2
    else:
        data.ctrl[i] = 0.0

# Step to settle
for _ in range(100):
    mujoco.mj_step(model, data)

print("=" * 60)
print("TASK 1: Basic Keyboard Control")
print("=" * 60)
print(f"Controlling: {TARGET_JOINT_NAME}")
print(f"  UP arrow   → increase position (+{JOINT_STEP} rad)")
print(f"  DOWN arrow → decrease position (-{JOINT_STEP} rad)")
print(f"  Q          → quit")
print("=" * 60)


def key_callback(keycode):
    import glfw

    if keycode == glfw.KEY_UP:
        new_val = data.ctrl[TARGET_JOINT_IDX] + JOINT_STEP
        data.ctrl[TARGET_JOINT_IDX] = np.clip(new_val, ctrl_lo, ctrl_hi)
        print(f"  Ctrl: {data.ctrl[TARGET_JOINT_IDX]:+.3f} rad")

    elif keycode == glfw.KEY_DOWN:
        new_val = data.ctrl[TARGET_JOINT_IDX] - JOINT_STEP
        data.ctrl[TARGET_JOINT_IDX] = np.clip(new_val, ctrl_lo, ctrl_hi)
        print(f"  Ctrl: {data.ctrl[TARGET_JOINT_IDX]:+.3f} rad")

    elif keycode == glfw.KEY_Q or keycode == glfw.KEY_ESCAPE:
        # We'll just set a flag - can't close viewer from callback directly
        print("  Q pressed - close viewer window to exit")


step = 0

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]

    while viewer.is_running():
        mujoco.mj_step(model, data)
        step += 1

        if step % 500 == 0:
            print(f"Step {step:5d} | {TARGET_JOINT_NAME}: ctrl={data.ctrl[TARGET_JOINT_IDX]:+.3f}  pos={data.qpos[TARGET_JOINT_IDX]:+.3f}")

        time.sleep(0.001)

print(f"\nDone. Steps: {step}")
