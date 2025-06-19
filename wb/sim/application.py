from enum import Enum
from dm_control.composer import Environment
from dm_control.composer import Task
from dm_control.mujoco import Physics
import time
import mujoco.viewer
import numpy as np


class State(Enum):
    RUNNING = 0
    RESET = 1
    STOPPED = 2


class Application:
    def __init__(self, task: Task):
        self._task = task
        self._env = Environment(task)
        self._env.reset()
        self._timestep = task.control_timestep
        self._state = State.RUNNING

    def _step(self):
        match (self._state):
            case State.RUNNING:
                self._env.step(np.zeros_like(self._env.action_spec().shape))
            case State.RESET:
                self._state = State.RUNNING
                self._task.initialize_episode(self._env.physics, self._env.random_state)
            case State.STOPPED:
                pass

    def _key_callback(self, key: int):
        #print(key, chr(key))
        if ord("R") == key:
            self._state = State.RESET
        elif ord(" ") == key:
            self._state = State.RUNNING if self._state == State.STOPPED else State.STOPPED

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
