import numpy as np
from dm_control.composer import Task
from dm_control.locomotion.arenas import Floor
from dm_control.mujoco import Physics

from dm_control.composer.variation import noises, distributions
from wobl_sim.robot import Robot
import math


def imu_noise():
    # ---- choose your model parameters ----
    fs = 200.0                  # Hz, your IMU sample rate
    fc = None                   # or e.g. 20.0 for a 1-pole LPF

    # Noise densities (ICM-20948 typicals) in SI per √Hz
    GYRO_ND = 0.015 * (math.pi / 180.0)   # rad/s/√Hz
    ACC_ND  = 230e-6 * 9.80665                      # m/s^2/√Hz

    # Effective noise bandwidth
    if fc is None:
        ENBW = fs / 2.0
    else:
        ENBW = 0.5 * math.pi * fc  # (π/2)*fc

    gyro_sigma = GYRO_ND * math.sqrt(ENBW)   # rad/s
    acc_sigma  = ACC_ND  * math.sqrt(ENBW)   # m/s^2
    return gyro_sigma, acc_sigma

class BalanceTask(Task):
    def __init__(self, robot: Robot, static_entity=False):
        super().__init__()

        self.robot = robot
        self._arena = Floor(reflectance=0.0)
        if static_entity:
            self._arena.attach(self.robot)
        else:
            self._arena.add_free_entity(self.robot)
        self.set_timesteps(control_timestep=0.01, physics_timestep=0.005)

        self.robot.observables.joint_positions.enabled = True
        self.robot.observables.joint_velocities.enabled = True
        self.robot.observables.joint_velocities.corruptor = noises.Multiplicative(
            distributions.LogNormal(sigma=0.03)
        )
        self.robot.observables.joint_velocities.delay = 1
        self.robot.observables.joint_efforts.enabled = True

        self.robot.observables.orientation.enabled = True
        self.robot.observables.orientation.delay = 1

        self.robot.observables.angular_velocity.enabled = True
        self.robot.observables.linear_acceleration.enabled = True

        gyro_sigma, acc_sigma = imu_noise()
        #print(f"IMU noise: gyro {gyro_sigma:.6f} rad/s, acc {acc_sigma:.6f} m/s²")
        self.robot.observables.angular_velocity.delay = 1
        self.robot.observables.linear_acceleration.delay = 1

        self.robot.observables.angular_velocity.corruptor = noises.Additive(
            distributions.Normal(scale=gyro_sigma)
        )

        self.robot.observables.linear_acceleration.enabled = True
        self.robot.observables.linear_acceleration.corruptor = noises.Additive(
            distributions.Normal(scale=acc_sigma)
        )

    def initialize_episode(self, physics: Physics, random_state):
        self.robot.set_pose(physics, np.array([0, 0, 0.24]), np.array([1, 0, 0, 0]))

    @property
    def root_entity(self):
        return self._arena

    def get_reward(self, physics: Physics):
        return 0
