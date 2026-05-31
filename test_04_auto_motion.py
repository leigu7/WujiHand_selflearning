"""
TASK 4: Add auto sine motion with Space toggle.

What's new:
    - Space bar: toggle auto-sinusoidal motion ON/OFF
    - When ON: all joints move with sine waves (you can see traces in PlotJuggler)
    - When OFF: joints stay at their last position (manual control works)
    - Arrow keys still override individual joints even when auto is ON

How to test:
    1. Open PlotJuggler -> Streaming -> UDP Server -> Port 9870 -> Start
    2. Drag "pos/finger1_joint1" into the plot
    3. Run this script
    4. Initially auto is ON → you should see a sine wave
    5. Press Space → sine wave freezes
    6. Press UP/DOWN → joint moves manually
    7. Press Space again → sine wave resumes but from current position
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

# Interactive state
selected_finger = 1
selected_joint = 1
auto_motion = True  # starts ON so you see traces immediately
t_start = time.time()

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
    print(f"  F{selected_finger} J{selected_joint} | {name}: ctrl={data.ctrl[idx]:+.3f} | Auto={'ON' if auto_motion else 'OFF'}")

reset_to_home()
for _ in range(100):
    mujoco.mj_step(model, data)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("=" * 60)
print("TASK 4: Auto Sine Motion (Space to toggle)")
print("=" * 60)
print("  [F]       Cycle fingers")
print("  [J]       Cycle joints")
print("  [UP/DOWN] Move selected joint")
print("  [Space]   Toggle auto motion ON/OFF")
print("  [R]       Reset to home")
print("=" * 60)
print_status()


def key_callback(keycode):
    global selected_finger, selected_joint, auto_motion, t_start
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

    elif keycode == glfw.KEY_SPACE:
        auto_motion = not auto_motion
        if auto_motion:
            t_start = time.time()  # restart phase so motion is smooth
        print(f"\nAuto motion: {'ON' if auto_motion else 'OFF'}")

    print_status()


step = 0
send_counter = 0

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]

    while viewer.is_running():
        # Auto motion: only runs when enabled
        if auto_motion:
            elapsed = time.time() - t_start
            for i, name in enumerate(actuator_names):
                if "_joint1" in name:
                    data.ctrl[i] = 0.3 * np.sin(0.4 * elapsed + 0.1 * i)
                elif "_joint2" in name:
                    data.ctrl[i] = 0.2 * np.sin(0.6 * elapsed + 0.3 * i)
                elif "_joint3" in name:
                    data.ctrl[i] = 0.3 * np.sin(0.5 * elapsed + 0.5 * i)
                elif "_joint4" in name:
                    data.ctrl[i] = 0.2 * np.sin(0.7 * elapsed + 0.7 * i)

        mujoco.mj_step(model, data)
        viewer.sync()
        step += 1
        send_counter += 1

        if send_counter >= SEND_EVERY_N_STEPS:
            send_counter = 0
            payload = {"timestamp": data.time, "step": step}

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
