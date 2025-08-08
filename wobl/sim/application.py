from enum import Enum
from typing import Callable
from dm_control.composer import Environment
from dm_control.composer import Task
from dm_control.mujoco import Physics
from dm_env import TimeStep
import time
import mujoco.viewer
import numpy as np
import numpy.typing as npt
import dm_env


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
        self._env._hooks.initialize_episode(
            self._env._physics_proxy, self._env._random_state
        )
        self._env._observation_updater.reset(
            self._env._physics_proxy, self._env._random_state
        )
        #self._env.physics.data.qpos[:] = 0
        self._env.physics.data.qvel[:] = 0
        self._env.physics.data.ctrl[:] = 0
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
