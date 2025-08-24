import mujoco
import numpy as np

# Load your model
model = mujoco.MjModel.from_xml_path("wobl_sim/mjcf/robot.xml")  # or MJCF path
data = mujoco.MjData(model)

# Simulate a few steps (optional if you want to get a steady pose)
for _ in range(10):
    mujoco.mj_step(model, data)

# Function to compute the full-body Center of Mass
def compute_total_com(model, data):
    total_mass = 0.0
    com = np.zeros(3)

    for i in range(model.nbody):
        body_mass = model.body_mass[i]
        # Get world position of this body's CoM
        body_com = data.xipos[i]
        print(body_com)
        com += body_mass * body_com
        total_mass += body_mass

    return com / total_mass if total_mass > 0 else com

# Print the CoM
robot_com = compute_total_com(model, data)
print("Center of Mass (world frame):", robot_com * 1000)