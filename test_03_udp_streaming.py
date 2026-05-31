"""
TASK 3: UDP Streaming to PlotJuggler (no auto motion, just manual control).

Streams ONLY pos/ and ctrl/ signals for the 20 joints.
No sine waves, no auto motion.
Keyboard controls same as Task 2.

How to test:
    1. Open PlotJuggler
       File -> Streaming -> UDP Server -> Port: 9870 -> Start
    2. Run this script
    3. In PlotJuggler, drag "pos/finger1_joint1" and "ctrl/finger1_joint1" into plot
    4. Press UP/DOWN in MuJoCo viewer
    5. Verify: flat lines that step up/down when you press arrow keys
"""

import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
import socket
import json
import time

SIDE = "right"
JOINT_STEP = 0.05
UDP_IP = "127.0.0.1"
UDP_PORT = 9870
SEND_EVERY_N_STEPS = 2

mjcf_path = (Path(__file__).parent / "wuji_hand_description" / "mjcf" / f"{SIDE}.xml").resolve()
model = mujoco.MjModel.from_xml_path(str(mjcf_path))
data = mujoco.MjData(model)

actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]

# Build finger/joint mapping
finger_joint_indices = {}
for name in actuator_names:
    parts = name.split('_')
    f_num = int(parts[0].replace('finger', ''))
    j_num = int(parts[1].replace('joint', ''))
    finger_joint_indices[(f_num, j_num)] = actuator_names.index(name)

finger_to_joints = {f: [] for f in range(1, 6)}
for (f, j), idx in finger_joint_indices.items():
    finger_to_joints[f].append(j)

ctrl_ranges = {}
for i in range(model.nu):
    if model.actuator_ctrllimited[i]:
        ctrl_ranges[i] = model.actuator_ctrlrange[i]
    else:
        ctrl_ranges[i] = (-1.57, 1.57)

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

reset_to_home()
for _ in range(100):
    mujoco.mj_step(model, data)

# Setup UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("=" * 60)
print("TASK 3: UDP Streaming to PlotJuggler")
print("=" * 60)
print("  Keyboard: F=cycle finger, J=cycle joint, UP/DOWN=move, R=reset")
print()
print("  PlotJuggler: File -> Streaming -> UDP Server -> Port 9870 -> Start")
print("  Then drag signals like: pos/finger1_joint1, ctrl/finger1_joint1")
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
        print(f"\n--- Finger {selected_finger} ---")

    elif keycode == glfw.KEY_J:
        valid = sorted(finger_to_joints[selected_finger])
        ci = valid.index(selected_joint) if selected_joint in valid else 0
        selected_joint = valid[(ci + 1) % len(valid)]
        print(f"\n--- Joint {selected_joint} ---")

    elif keycode == glfw.KEY_UP:
        set_ctrl(idx, data.ctrl[idx] + JOINT_STEP)

    elif keycode == glfw.KEY_DOWN:
        set_ctrl(idx, data.ctrl[idx] - JOINT_STEP)

    elif keycode == glfw.KEY_R:
        reset_to_home()
        print("Reset to home.")

    print_status()


step = 0
send_counter = 0

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        step += 1
        send_counter += 1

        if send_counter >= SEND_EVERY_N_STEPS:
            send_counter = 0
            payload = {"timestamp": data.time, "step": step}

            # Stream ONLY pos and ctrl (no forces, no tips yet)
            for i, name in enumerate(actuator_names):
                payload[f"pos/{name}"] = round(float(data.qpos[i]), 6)
                payload[f"ctrl/{name}"] = round(float(data.ctrl[i]), 6)

            try:
                msg = json.dumps(payload).encode("utf-8")
                sock.sendto(msg, (UDP_IP, UDP_PORT))
            except Exception as e:
                print(f"UDP error: {e}")

        if step % 500 == 0:
            idx = get_selected_idx()
            print(f"Step {step:5d} | {actuator_names[idx]}: ctrl={data.ctrl[idx]:+.3f}")

        time.sleep(0.001)

sock.close()
print(f"Done. Steps: {step}")
