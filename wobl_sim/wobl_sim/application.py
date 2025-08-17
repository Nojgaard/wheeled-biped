import time
from enum import Enum
from typing import Callable

import dm_env
import mujoco.viewer
import numpy.typing as npt
from dm_control.composer import Environment, Task
from dm_control.mujoco import Physics
from dm_env import TimeStep


class State(Enum):
    RUNNING = 0
    RESET = 1
    STOPPED = 2


class Application:
    def __init__(self, task: Task, policy: Callable[[TimeStep], npt.ArrayLike]):
        self._task = task
        self._env = Environment(task, strip_singleton_obs_buffer_dim=True)
        self._step_data = self._env.reset()
        self._timestep = task.control_timestep
        self._state = State.RUNNING
        self._policy = policy

    def _step(self):
        match (self._state):
            case State.RUNNING:
                action = self._policy(self._step_data)
                self._step_data = self._env.step(action)
            case State.RESET:
                self._state = State.RUNNING
                self._step_data = self._restart()
            case State.STOPPED:
                self._env.physics.forward()

    def _restart(self):
        self._env.physics.reset()
        self._env._hooks.initialize_episode(
            self._env._physics_proxy, self._env._random_state
        )
        self._env._observation_updater.reset(
            self._env._physics_proxy, self._env._random_state
        )
        self._env.physics.forward()

        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=None,
            discount=None,
            observation=self._env._observation_updater.get_observation(),
        )

    def _key_callback(self, key: int):
        # print(key, chr(key))
        if ord("R") == key:
            self._state = State.RESET
        elif ord(" ") == key:
            self._state = (
                State.RUNNING if self._state == State.STOPPED else State.STOPPED
            )

    def launch_headless(self):
        """Launch the application in headless mode."""
        while True:
            step_start = time.time()
            self._step()

            time_until_next_step = self._timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def launch(self):
        physics: Physics = self._env.physics

        with mujoco.viewer.launch_passive(
            physics.model.ptr, physics.data.ptr, key_callback=self._key_callback
        ) as viewer:
            while viewer.is_running():
                step_start = time.time()
                self._step()
                viewer.sync()

                time_until_next_step = self._timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
        self._env.close()


if __name__ == "__main__":
    import numpy as np

    from wobl_sim.balance_task import BalanceTask
    from wobl_sim.robot import Robot

    robot = Robot()
    task = BalanceTask(robot)
    app = Application(task, lambda timestep: np.zeros(4))
    app.launch()
