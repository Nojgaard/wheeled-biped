from simple_pid import PID

import numpy as np
import numpy.typing as npt


class PidBalancer:
    def __init__(self):
        self.pitch_pid = PID(50.0, 0.0, 0.5, setpoint=0.00, sample_time=None, output_limits=(-10, 10))
        self.vel_pid = PID(60.0, 25, 0, setpoint=0.00, sample_time=None, output_limits=(-10, 10))

    def update(self, dt: float, orientation: npt.ArrayLike, linear_velocity: npt.ArrayLike):
        pitch = orientation[1]
        fwd_vel = linear_velocity[1]
        control = self.pitch_pid(-pitch, dt) - self.vel_pid(-fwd_vel, dt)
        return (control, control)
    