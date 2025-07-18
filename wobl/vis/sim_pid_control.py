from wobl.control.pid_balancer import PidBalancer
from wobl.sim.balance_task import BalanceTask
from wobl.sim.application import Application
from dm_env import TimeStep, StepType
import numpy as np
import dearpygui.dearpygui as dpg
import threading

from wobl.sim.robot import Robot


class Gui:
    def __init__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        dpg.create_context()

        with dpg.window(tag="primary_window"):
            self._velocity = dpg.add_slider_float(
                label="Target Velocity", min_value=-0.5, max_value=0.5, default_value=0
            )
            self._yaw_rate = dpg.add_slider_float(
                label="Target Yaw Rate",
                min_value=-2,
                max_value=2,
                default_value=0,
            )

        dpg.create_viewport(title="Control Dashboard", width=500, height=300)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("primary_window", value=True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    @property
    def velocity(self):
        return dpg.get_value(self._velocity)
    
    @property
    def yaw_rate(self):
        return dpg.get_value(self._yaw_rate)
    
    def launch(self):
        self._thread.start()


def main():
    robot = Robot()
    task = BalanceTask(robot)
    controller = PidBalancer()
    gui = Gui()
    gui.launch()

    def policy(timestep: TimeStep):
        if timestep.step_type == StepType.FIRST:
            controller._pitch_pid.reset()
        joint_velocities = timestep.observation["robot/joint_velocities"]
        print(timestep.observation["robot/joint_efforts"])
        velocity = ((joint_velocities[2] +joint_velocities[3])  / 2) * 0.08

        #velocity = timestep.observation["robot/linear_velocity"][0]
        print("linvel", timestep.observation["robot/linear_velocity"][0])
        orientation = timestep.observation["robot/orientation"]
        #linear_velocity = timestep.observation["robot/linear_velocity"]
        controller.target_velocity = gui.velocity
        controller.target_yaw_rate = gui.yaw_rate
        lvel, rvel = controller.update(
            task.control_timestep, orientation, velocity, 
        )

        return np.array([0.0, 0.0, lvel, rvel])

    app = Application(task, policy)
    app.launch()
    print("Ending")


if __name__ == "__main__":
    main()
