from dm_control.composer import Task
from wobl.sim.robot import Robot
from dm_control.locomotion.arenas import Floor
from dm_control.mujoco import Physics
import numpy as np
from dm_control.composer.variation import distributions
from dm_control.composer.variation import noises
from dm_control.composer.variation import variation_values


class UnitVector:
    """Adds noise via variation, then re-normalizes the vector."""

    def __init__(self, noise_variation, eps=1e-8):
        self._variation = noise_variation
        self._eps = eps

    def __call__(self, initial_value=None, current_value=None, random_state=None):
        noise = variation_values.evaluate(
            self._variation, initial_value, current_value, random_state
        )
        return noise / np.linalg.norm(noise)


class BalanceTask(Task):
    def __init__(self, robot: Robot):
        super().__init__()

        self.robot = robot
        self._arena = Floor(reflectance=0.0)
        self._arena.add_free_entity(self.robot)
        self.set_timesteps(control_timestep=0.01, physics_timestep=0.005)

        self.robot.observables.joint_positions.enabled = True
        self.robot.observables.joint_velocities.enabled = True
        self.robot.observables.joint_efforts.enabled = True

        self.robot.observables.orientation.enabled = True
        #self.robot.observables.orientation.corruptor = noises.Additive(distributions.Normal(scale=0.0005))
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
