import time
import mujoco
import mujoco.viewer
import yaml
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__),os.path.pardir))
import ModelGenerator

model_config_path = 'parametric_configs/WS/model_config.yaml'
motor_config_path = 'parametric_configs/WS/motor_config.yaml'

walter = ModelGenerator.GenerateModel(model_config_path, motor_config_path)

walter.gen_scene()

m = walter.spec.compile()
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        mujoco.mj_step(m, d)
        viewer.sync()

        time_until_next_step = m.opt.timestep - (time.time() - start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)