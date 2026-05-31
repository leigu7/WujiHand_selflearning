#!/usr/bin/env python3
import mujoco
import mujoco.viewer
from pathlib import Path
import os

side = 'right'
mjcf_path = (Path('wuji_hand_description') / 'mjcf' / f'{side}.xml').resolve()
model = mujoco.MjModel.from_xml_path(str(mjcf_path))
data = mujoco.MjData(model)

for i in range(model.nu):
    if model.actuator_ctrllimited[i]:
        ctrl_range = model.actuator_ctrlrange[i]
        data.ctrl[i] = (ctrl_range[0] + ctrl_range[1]) / 2

for _ in range(100):
    mujoco.mj_step(model, data)

names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
tips = ['finger1_link4','finger2_link4','finger3_link4','finger4_link4','finger5_link4']
tip_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in tips]

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 0.5
    viewer.cam.lookat[:] = [0, 0, 0.05]
    step = 0
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        step += 1
        if step % 100 == 0:
            os.system('cls')
            print(f'Time: {data.time:.4f}s  Step: {step}')
            print('--- Positions ---')
            for i in range(model.nu):
                print(f'  {names[i]}: pos={data.qpos[i]:.4f}  vel={data.qvel[i]:.4f}')
            print('--- Forces ---')
            for i in range(model.nu):
                print(f'  {names[i]}: act={data.qfrc_actuator[i]:.4f}  pass={data.qfrc_passive[i]:.4f}')
            print('--- Tips ---')
            for i, tid in enumerate(tip_ids):
                p = data.xpos[tid]
                print(f'  Finger{i+1}: ({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})')
            print(f'Energy: K={data.energy[0]:.8f}  P={data.energy[1]:.8f}')
            print(f'Contacts: {data.ncon}')

