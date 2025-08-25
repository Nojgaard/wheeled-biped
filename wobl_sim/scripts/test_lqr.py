from build.wobl_sim.build.lib.wobl_sim.application import Application
import numpy as np

from wobl_sim.balance_task import BalanceTask
from wobl_sim.robot import Robot
from scipy.spatial.transform import Rotation

# K = np.array([[-323573.82487269, -1272.68254765, -239663.34361485],
#                [-323573.82487269, -1272.68254765, -239663.34361485]])
# K = np.array([-6.34656745e+00, -5.66288177e-01, -1.53675426e-17,  2.23606798e+00])
K = np.array([-6.17048601, -0.62420939,  1.61091858,  0.81649658])
filtered_vel = 0
velocity_integral = 0.0
filtered_pitch_rate = 0


def policy(timestep):
    if np.linalg.norm(timestep.observation["robot/orientation"]) == 0:
        return np.zeros(4)
    quat = Rotation.from_quat(
        timestep.observation["robot/orientation"], scalar_first=True
    )
    equilibrium_pitch = 0.031
    pitch = quat.as_euler("XYZ", degrees=False)[1] - equilibrium_pitch
    pitch_rate = timestep.observation["robot/angular_velocity"][1]
    joint_vel = np.mean(timestep.observation["robot/joint_velocities"][2:])

    global filtered_vel
    filtered_vel = filtered_vel * 0.95 + 0.05 * (joint_vel * 0.04)

    global filtered_pitch_rate
    filtered_pitch_rate = filtered_pitch_rate * 0.80 + 0.2 * pitch_rate

    # In your policy function
    global velocity_integral
    velocity_integral += filtered_vel * 0.01  # Accumulate velocity error
    velocity_integral = np.clip(velocity_integral, -0.3, 0.3)  # Prevent windup

    # Add to your control law

    lqr_v = (
        K[0] * pitch
        + K[1] * filtered_pitch_rate
        + K[2] * -filtered_vel
        + K[3] * -velocity_integral
    )
    #print(integral_gain * velocity_integral, lqr_v)
    lqr_v = np.clip(-lqr_v / 0.04, -10, 10)
    return np.array([0, 0, lqr_v, lqr_v])


def main():

    robot = Robot()
    task = BalanceTask(robot)
    app = Application(task, policy)
    app.launch()


if __name__ == "__main__":
    main()
