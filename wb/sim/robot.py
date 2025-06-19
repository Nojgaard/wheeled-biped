from dm_control.composer import Entity, Observables
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control import mjcf
from scipy.spatial.transform import Rotation


class Robot(Entity):
    def __init__(self):
        self._model_path = "assets/robot.xml"
        super().__init__()

    def _build(self):
        self._model = mjcf.from_path(self._model_path)

    @property
    def mjcf_model(self):
        return self._model

    def _build_observables(self):
        return RobotObservables(self)

    def orientation(self, physics):
        framequat_element = self._entity.mjcf_model.sensor.framequat
        quat = physics.bind(framequat_element).sensordata
        return Rotation.from_quat(quat, scalar_first=True).as_euler("XYZ")


class RobotObservables(Observables):
    _entity: Robot

    @composer.observable
    def joint_positions(self):
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qpos", all_joints)

    @composer.observable
    def joint_velocities(self):
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qvel", all_joints)

    @composer.observable
    def attitude_velocity(self):
        return observable.MJCFFeature(
            "sensordata",
            self._entity.mjcf_model.sensor.gyro,
        )

    @composer.observable
    def accelerometer(self):
        return observable.MJCFFeature(
            "sensordata", self._entity.mjcf_model.sensor.accelerometer
        )
