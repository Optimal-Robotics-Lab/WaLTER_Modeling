import argparse
import time
import mujoco
import mujoco.viewer
import os
import sys

# Allow running straight from a checkout (`python3 scripts/view_parametric.py`)
# without installing the package first. Harmless when it *is* installed.
sys.path.append(os.path.join(os.path.dirname(__file__), os.path.pardir))
from WaLTER_Modeling import parametric

parser = argparse.ArgumentParser(description="View the parametric WaLTER model")
parser.add_argument("--fs", action="store_true", help="View FS instead of WS")
args = parser.parse_args()

walter = parametric.build(full_scale=args.fs)

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
