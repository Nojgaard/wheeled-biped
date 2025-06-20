from simple_pid import PID

import numpy as np
import numpy.typing as npt


class PidBalancer:
    def __init__(self):
        self._pitch_pid = PID(40.0, 0.1, 1.5, setpoint=0.00, sample_time=None, output_limits=(-10, 10))
        self._vel_pid = PID(2, 1.0, 0, setpoint=0.00, sample_time=None, output_limits=(-0.17, 0.17))
        #self._pitch_pid = PID(50.0, 0.0, 0.5, setpoint=0.00, sample_time=None, output_limits=(-10, 10))
        #self._vel_pid = PID(60.0, 25.0, 0, setpoint=0.00, sample_time=None, output_limits=(-10, 10))
        self.target_velocity = 0
        self.target_yaw_rate = 0

    def update(self, dt: float, orientation: npt.ArrayLike, linear_velocity: npt.ArrayLike):
        pitch = orientation[1]
        fwd_vel = linear_velocity[1]
        
        self._vel_pid.setpoint = -self.target_velocity
        feedforward_pitch = 0.1 * self._vel_pid.setpoint
        self._pitch_pid.setpoint = self._vel_pid(fwd_vel, dt) + feedforward_pitch
        control = self._pitch_pid(-pitch, dt)
        #print(self.target_velocity, fwd_vel, self._pitch_pid.setpoint, pitch, control)
        #control = self._pitch_pid(-pitch, dt) - self._vel_pid(-fwd_vel, dt)
        return (control - self.target_yaw_rate, control + self.target_yaw_rate)
    