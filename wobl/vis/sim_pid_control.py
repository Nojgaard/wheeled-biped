from wobl.control.pid_balancer import PidBalancer
from wobl.sim.balance_task import BalanceTask
from wobl.sim.application import Application
from dm_env import TimeStep, StepType
import numpy as np

def main():
    task = BalanceTask()
    controller = PidBalancer()
    def policy(timestep: TimeStep):
        if timestep.step_type == StepType.FIRST:
            controller.pitch_pid.reset()
        orientation = timestep.observation["robot/orientation"]
        linear_velocity = timestep.observation["robot/linear_velocity"]
        lvel, rvel = controller.update(task.control_timestep, orientation, linear_velocity)

        return np.array([0.0, 0.0, lvel, rvel])

    app = Application(task, policy)
    app.launch()
    print("Ending")


if __name__ == "__main__":
    main()
