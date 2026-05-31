# Wuji Hand — Project Ideas for MuJoCo

This file outlines 8 projects you can build with the Wuji Hand in MuJoCo, from beginner to advanced. Each will be implemented in its own subfolder.

---

## Project 1: Teleoperation (Record & Playback)
**Folder:** `projects/01_teleoperation/`

Record manual finger movements from the keyboard-controlled viewer, save as `.npy` trajectory files, and replay them.

**Key features:**
- Press **R** to start/stop recording joint positions
- Save to `data/recorded_trajectory.npy`
- Load and replay with `run_sim.py`
- Compare recorded vs replayed trajectory in PlotJuggler

**Learning goals:**
- Working with MuJoCo data arrays
- File I/O for NumPy arrays
- Basic state logging

---

## Project 2: Force Feedback & Virtual Grasping
**Folder:** `projects/02_force_feedback/`

Use contact and force signals to detect when fingers touch, provide visual/haptic feedback, and attempt to grasp a virtual object.

**Key features:**
- Detect finger contact (from `n_contacts` and contact forces)
- Display contact status overlay in the viewer
- Add a virtual ball/sphere to the simulation
- Try to pick it up with the hand
- Visualize grip force in PlotJuggler

**Learning goals:**
- Contact detection in MuJoCo
- Adding objects to the scene dynamically
- Force-based interaction

---

## Project 3: PID Controller Tuning & Custom Actuation
**Folder:** `projects/03_pid_control/`

The Wuji Hand's MJCF uses `biastype=USER`, meaning it expects a user-defined bias callback. Implement a PID controller and tune it.

**Key features:**
- Implement `set_mjcb_act_bias` callback with PID
- Adjustable P, I, D gains (via keyboard or config file)
- Compare position tracking (`ctrl` vs `pos`) in PlotJuggler
- Auto-tune by analyzing step response

**Learning goals:**
- MuJoCo actuator callbacks
- Control theory (PID)
- Performance optimization

---

## Project 4: Grasping Objects with Different Shapes
**Folder:** `projects/04_grasping/`

Drop various objects (sphere, cube, cylinder, irregular shape) in front of the hand and try to grasp them.

**Key features:**
- Load or procedurally generate objects
- Position objects within reach of the hand
- Close fingers around the object using position control
- Measure grasp success (contact count, object displacement)
- Visualize grasp quality metrics

**Learning goals:**
- Scene composition in MuJoCo
- Object manipulation
- Grasp quality metrics

---

## Project 5: Reinforcement Learning Environment
**Folder:** `projects/05_reinforcement_learning/`

Set up the hand as a Gymnasium-compatible RL environment for reaching and grasping tasks.

**Key features:**
- Gymnasium `Env` interface (step, reset, render)
- Reward function for reaching a target position
- Observation space: joint positions, velocities, fingertip positions
- Action space: target joint positions
- Integration with Stable-Baselines3 or SBX

**Learning goals:**
- RL environment design
- Reward shaping
- Training with PPO/SAC

---

## Project 6: Multi-Hand Coordination
**Folder:** `projects/06_multi_hand/`

Load both left and right hands simultaneously for bimanual manipulation tasks.

**Key features:**
- Load `left.xml` and `right.xml` into one model
- Control both hands independently
- Coordinate finger movements between hands
- Bimanual grasping of larger objects

**Learning goals:**
- Multi-body simulation in MuJoCo
- Bimanual coordination
- Complex scene composition

---

## Project 7: Custom Motion Primitives & Gestures
**Folder:** `projects/07_motion_primitives/`

Create a library of finger motion patterns and gestures that can be composed and blended.

**Key features:**
- Pre-defined gestures: Pinch, Point, Fist, Open Palm, Thumbs Up
- Smooth interpolation between gestures
- Gesture recorder UI
- Save/load gesture library as `.npy` files
- Sequence of gestures (e.g., wave gesture)

**Learning goals:**
- Motion interpolation (SLERP, linear)
- Keyframe animation concepts
- State machines for gesture sequencing

---

## Project 8: Web Dashboard for Real-Time Monitoring
**Folder:** `projects/08_web_dashboard/`

Replace or augment PlotJuggler with a custom web-based dashboard for real-time visualization.

**Key features:**
- Flask/FastAPI WebSocket server
- Browser-based real-time line charts
- 3D hand visualization with Three.js or Babylon.js
- Remote keyboard/mouse control from the browser
- Multi-client support

**Learning goals:**
- WebSocket communication
- Real-time data streaming over HTTP
- 3D rendering in the browser
- Full-stack development

---

## Project Roadmap

```
projects/
├── 01_teleoperation/        # Record & playback
├── 02_force_feedback/       # Virtual grasping
├── 03_pid_control/          # Custom actuator tuning
├── 04_grasping/             # Object manipulation
├── 05_reinforcement_learning/  # RL environment
├── 06_multi_hand/           # Bimanual control
├── 07_motion_primitives/    # Gesture library
└── 08_web_dashboard/        # Web visualization
```

Each project folder will contain:
- `README.md` — Detailed description and usage
- `requirements.txt` — Additional dependencies
- Main script(s)

**Start with Project 1 or 2 — they build directly on what you've already done with `test_05_full_signals.py`.**
