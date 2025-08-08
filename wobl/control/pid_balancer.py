from simple_pid import PID

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation
from dm_control.composer.variation import distributions
from dm_control.composer.variation import noises


class PidBalancer:
    def __init__(self):
        self._pitch_pid = PID(10.0, 0.0, 5.0, setpoint=0.00, sample_time=None, output_limits=(-10, 10))
        self._vel_pid = PID(.5, 0.1, 0.0, setpoint=0.00, sample_time=None, output_limits=(-0.17, 0.17))
        #self._pitch_pid = PID(40.0, 0.1, 1.5, setpoint=0.00, sample_time=None, output_limits=(-10, 10))
        #self._vel_pid = PID(2, 1.0, 0, setpoint=0.00, sample_time=None, output_limits=(-0.17, 0.17))
        self.target_velocity = 0
        self.target_yaw_rate = 0
        self.velocities = 0

    def update(self, dt: float, quat: npt.ArrayLike, linear_velocity: float):
        if (quat[0] == 0 and quat[1] == 0 and quat[2] == 0 and quat[3] == 0):
            return (0, 0)
        orientation = Rotation.from_quat(quat, scalar_first=True).as_euler("XYZ")
        pitch = orientation[1]
        #pitch = noises.Additive(distributions.Normal(scale=0.01))(pitch)
        #self.velocities.append(linear_velocity)
        self.velocities = linear_velocity * 0.01 + self.velocities * 0.99
        #if len(self.velocities) > 5:
        #    self.velocities = self.velocities[1:]
        #fwd_vel = np.mean(self.velocities)
        fwd_vel = -self.velocities

        
        self._vel_pid.setpoint = -self.target_velocity
        feedforward_pitch = 0.1 * self._vel_pid.setpoint
        self._pitch_pid.setpoint = self._vel_pid(fwd_vel, dt)
        control = self._pitch_pid(-pitch, dt)
        print(self.target_velocity, fwd_vel, self._pitch_pid.setpoint, pitch, control)
        #control = self._pitch_pid(-pitch, dt) - self._vel_pid(-fwd_vel, dt)
        return (control - self.target_yaw_rate, control + self.target_yaw_rate)
    