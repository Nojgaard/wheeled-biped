from dm_control.composer import Task
from wb.sim.robot import Robot
from dm_control.locomotion.arenas import Floor
from dm_control.mujoco import Physics
import numpy as np

class BalanceTask(Task):
    def __init__(self):
        super().__init__()

        self.robot = Robot()
        self._arena = Floor(reflectance=0.0)
        self._arena.add_free_entity(self.robot)
        self.set_timesteps(control_timestep=0.02, physics_timestep=0.005)

    def initialize_episode(self, physics: Physics, random_state):
        self.robot.set_pose(
            physics, np.array([0, 0, 0.24]), np.array([1, 0, 0, 0])
        )

    @property
    def root_entity(self):
        return self._arena

    def get_reward(self, physics: Physics):
        return 0