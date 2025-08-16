import numpy as np
from dm_control.composer import Task
from dm_control.locomotion.arenas import Floor
from dm_control.mujoco import Physics

from dm_control.composer.variation import noises, distributions
from wobl_sim.robot import Robot


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

    def initialize_episode(self, physics: Physics, random_state):
        self.robot.set_pose(physics, np.array([0, 0, 0.24]), np.array([1, 0, 0, 0]))

    @property
    def root_entity(self):
        return self._arena

    def get_reward(self, physics: Physics):
        return 0
