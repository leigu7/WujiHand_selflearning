"""
TASK 2: Add finger/joint selection with F and J keys.

How it works:
    - F key: cycle through fingers 1..5
    - J key: cycle through joints 1..4 within the selected finger
    - UP/DOWN arrows: move the selected joint
    - R key: reset all joints to home (mid-range)

What to verify:
    - Press F → switches between fingers (terminal shows which)
    - Press J → switches between joints within a finger
    - UP/DOWN → moves the selected joint only
    - R → resets all joints to mid-range
"""

import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
import time

SIDE = "right"
JOINT_STEP = 0.05

mjcf_path = (Path(__file__).parent / "wuji_hand_description" / "mjcf" / f"{SIDE}.xml").resolve()
model = mujoco.MjModel.from_xml_path(str(mjcf_path))
data = mujoco.MjData(model)

actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]

# Build finger/joint mapping
finger_joint_indices = {}  # (finger_num, joint_num) -> actuator index
for name in actuator_names:
    parts = name.split('_')
    f_num = int(parts[0].replace('finger', ''))
    j_num = int(parts[1].replace('joint', ''))
    finger_joint_indices[(f_num, j_num)] = actuator_names.index(name)

finger_to_joints = {f: [] for f in range(1, 6)}
for (f, j), idx in finger_joint_indices.items():
    finger_to_joints[f].append(j)

# Get control ranges
ctrl_ranges = {}
for i in range(model.nu):
    if model.actuator_ctrllimited[i]:
        ctrl_ranges[i] = model.actuator_ctrlrange[i]
    else:
        ctrl_ranges[i] = (-1.57, 1.57)

# Interactive state
selected_finger = 1
selected_joint = 1

def get_selected_idx():
    return finger_joint_indices.get((selected_finger, selected_joint), 0)

def set_ctrl(idx, val):
    lo, hi = ctrl_ranges[idx]
    data.ctrl[idx] = np.clip(val, lo, hi)

def reset_to_home():
    for i in range(model.nu):
        lo, hi = ctrl_ranges[i]
        data.ctrl[i] = (lo + hi) / 2

def print_status():
    idx = get_selected_idx()
    name = actuator_names[idx]
    print(f"  Finger {selected_finger} | Joint {selected_joint} | {name}: ctrl={data.ctrl[idx]:+.3f}")

# Set initial pose
reset_to_home()
for _ in range(100):
    mujoco.mj_step(model, data)

print("=" * 60)
print("TASK 2: Finger/Joint Selection")
print("=" * 60)
print("  [F]       → Cycle fingers: 1..5")
print("  [J]       → Cycle joints:  1..4 within finger")
print("  [UP]      → Increase selected joint position")
print("  [DOWN]    → Decrease selected joint position")
print("  [R]       → Reset all joints to home")
print("  Close window to quit")
print("=" * 60)
print_status()


def key_callback(keycode):
    global selected_finger, selected_joint
    import glfw

    idx = get_selected_idx()

    if keycode == glfw.KEY_F:
        selected_finger = (selected_finger % 5) + 1
        valid = finger_to_joints[selected_finger]
        if selected_joint not in valid:
            selected_joint = min(valid)
        print(f"\n--- Switched to Finger {selected_finger} ---")

    elif keycode == glfw.KEY_J:
        valid = sorted(finger_to_joints[selected_finger])
        ci = valid.index(selected_joint) if selected_joint in valid else 0
        selected_joint = valid[(ci + 1) % len(valid)]
        print(f"\n--- Switched to Joint {selected_joint} ---")

    elif keycode == glfw.KEY_UP:
        set_ctrl(idx, data.ctrl[idx] + JOINT_STEP)

    elif keycode == glfw.KEY_DOWN:
        set_ctrl(idx, data.ctrl[idx] - JOINT_STEP)

    elif keycode == glfw.KEY_R:
        reset_to_home()
        print("Reset to home position.")

    print_status()


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
            idx = get_selected_idx()
            print(f"Step {step:5d} | {actuator_names[idx]}: ctrl={data.ctrl[idx]:+.3f}")

        time.sleep(0.001)

print(f"Done. Steps: {step}")
