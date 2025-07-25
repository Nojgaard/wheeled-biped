import os

from dm_control import composer, mjcf
from dm_control.composer import Entity, Observables
from dm_control.composer.observation import observable
from scipy.spatial.transform import Rotation


def quat_to_euler(quat):
    return Rotation.from_quat(quat, scalar_first=True).as_euler("XYZ")


class Robot(Entity):
    def __init__(self, assets_dir = "wobl_description/mjcf"):
        self._model_path = os.path.join(assets_dir, "robot.xml")
        super().__init__()

    def _build(self):
        self._model = mjcf.from_path(self._model_path)
        self.joint_names = ["L_hip", "R_hip", "L_foot", "R_foot"]
        self.mjcf_joints = [self.mjcf_model.find('joint', name) for name in self.joint_names]

    @property
    def mjcf_model(self):
        return self._model

    def _build_observables(self):
        return RobotObservables(self)

    def orientation(self, physics):
        framequat_element = self._entity.mjcf_model.sensor.framequat
        quat = physics.bind(framequat_element).sensordata
        return Rotation.from_quat(quat, scalar_first=True).as_euler("XYZ")

    @property
    def observables(self) -> "RobotObservables":
        return super().observables


class RobotObservables(Observables):
    _entity: Robot

    @composer.observable
    def joint_positions(self):
        return observable.MJCFFeature("qpos", self._entity.mjcf_joints)

    @composer.observable
    def joint_velocities(self):
        return observable.MJCFFeature("qvel", self._entity.mjcf_joints)
    
    @composer.observable
    def joint_efforts(self):
        return observable.Generic(lambda physics: physics.data.actuator_force)

    @composer.observable
    def angular_velocity(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.gyro,
        )

    @composer.observable
    def linear_acceleration(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.accelerometer
        )

    def read_orientation(self, physics):
        framequat_element = self._entity.mjcf_model.sensor.framequat
        quat = physics.bind(framequat_element).sensordata
        return quat_to_euler(quat)

    @composer.observable
    def orientation(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.framequat
        )
        #return observable.Generic(self.read_orientation)

    @composer.observable
    def linear_velocity(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.velocimeter,
        )
