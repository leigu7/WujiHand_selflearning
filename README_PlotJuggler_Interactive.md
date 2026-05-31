# Wuji Hand Interactive Visualization with PlotJuggler

This guide explains how to use the interactive visualization scripts for the **Wuji Hand** in MuJoCo. These scripts allow you to manually control the hand's fingers in real-time and stream data to **PlotJuggler** for live plotting.

## Quick Start

### 1. Run the visualization
```bash
cd mujoco-sim
python test_05_full_signals.py
```

### 2. Open PlotJuggler
- Open **PlotJuggler** desktop app
- Go to **File → Streaming → UDP Server**
- Set **Port: 9870** and click **Start**

### 3. Plot signals
In PlotJuggler, drag any signal name from the left panel into the plot area.

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| **F** | Cycle fingers: 1 (Thumb) → 2 → 3 → 4 → 5 |
| **J** | Cycle joints within the selected finger |
| **↑** | Increase selected joint position (+0.05 rad) |
| **↓** | Decrease selected joint position (−0.05 rad) |
| **Space** | Toggle auto sine motion ON/OFF |
| **R** | Reset all joints to home (mid-range) position |

---

## Available Signals in PlotJuggler

| Signal Pattern | Description |
|----------------|-------------|
| `pos/fingerN_jointN` | Actual joint position (radians) |
| `ctrl/fingerN_jointN` | Commanded control input (radians) |
| `vel/fingerN_jointN` | Joint velocity (rad/s) |
| `force_act/fingerN_jointN` | Actuator torque (Nm) |
| `force_pass/fingerN_jointN` | Passive forces (damping, friction) |
| `force_bias/fingerN_jointN` | Coriolis/gravity bias (Nm) |
| `tip/fingerN_x`, `tip/fingerN_y`, `tip/fingerN_z` | Fingertip Cartesian position (m) |
| `energy_kinetic` | Total kinetic energy (J) |
| `energy_potential` | Total potential energy (J) |
| `n_contacts` | Number of active contacts |
| `contact_N_force`, `contact_N_dist` | Contact force (N) and distance (m) |

---

## Sciript Learning Progression

| File | What It Teaches |
|------|-----------------|
| `test_01_basic_control.py` | Minimal viewer + UP/DOWN arrow on one joint |
| `test_01b_debug_control.py` | Sine wave in main loop (proves `viewer.sync()` needed) |
| `test_01c_with_sync.py` | Sine wave WITH `viewer.sync()` — motion works! |
| `test_01d_keyboard_with_sync.py` | Keyboard + `viewer.sync()` — manual control works |
| `test_02_finger_joint_selection.py` | F/J keys for finger/joint selection |
| `test_03_udp_streaming.py` | UDP streaming to PlotJuggler (pos/ + ctrl/) |
| `test_04_auto_motion.py` | Auto sine motion with Space toggle |
| `test_05_full_signals.py` | Full signal streaming (pos, ctrl, vel, forces, tips, contacts) |
| `view_plotjuggler_stream.py` | Final complete version |

---

## Key Technical Note

The MuJoCo passive viewer requires **`viewer.sync()`** after each `mujoco.mj_step()` call. Without it, changes to `data.ctrl[]` are computed internally but **not reflected visually** in the viewer window.

```python
# Correct pattern:
mujoco.mj_step(model, data)
viewer.sync()  # ← Required for visual updates!
```

This was discovered and verified in `test_01c_with_sync.py`.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Viewer opens but fingers don't move | Add `viewer.sync()` after each `mj_step()` |
| PlotJuggler shows no signals | Click **Start** in UDP Server dialog, check port 9870 |
| Keyboard not responding | Click on the MuJoCo viewer window to focus it |
| Unicode arrow errors on Windows | Run `chcp 65001` in terminal before Python scripts |
