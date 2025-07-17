import threading

import numpy as np
import rclpy
from dm_env import TimeStep
from rclpy.node import Node

from wobl.sim.application import Application
from wobl.sim.balance_task import BalanceTask
from wobl.sim.robot import Robot

from wobl_messages.msg import JointCommand
from sensor_msgs.msg import Imu, JointState


class MujocoBridgeNode(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        self._action = np.zeros(4)

        self._robot = Robot(assets_dir="wobl_description/mjcf")
        self._task = BalanceTask(self._robot)
        self._app = Application(self._task, self.update_sim)
        self._thread = threading.Thread(target=self._app.launch, daemon=True)
        self._thread.start()

        self.subscriber = self.create_subscription(
            JointCommand, "joint_commands", self.command_callback, 10
        )

        self._publish_imu_state = self.create_publisher(Imu, "imu/data", 10)
        self._publish_joint_state = self.create_publisher(JointState, "joint_states")

    def command_callback(self, msg: JointCommand):
        self._action[:] = msg.position

    def publish_joints(self, timestep: TimeStep):
        joint_state = JointState()
        joint_state.name = self._robot.joint_names()
        joint_state.position = timestep.observation["robot/joint_positions"]
        joint_state.velocity = timestep.observation["robot/joint_velocities"]
        joint_state.effort = timestep.observation["robot/joint_efforts"]
        self._publish_joint_state(joint_state)

    def publish_imu(self, timestep: TimeStep):
        imu = Imu()
        imu.orientation = timestep.observation["robot/orientation"]
        imu.angular_velocity = timestep.observation["robot/angular_velocity"]
        imu.linear_acceleration = timestep.observation["robot/linear_acceleration"]
        self._publish_imu_state(imu)

    def update_sim(self, timestep: TimeStep):
        self.publish_imu(timestep)
        self.publish_joints(timestep)
        return self._action

    def shutdown(self):
        """Gracefully shutdown the node"""
        self.get_logger().info("MuJoCo Viewer closed, shutting down...")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridgeNode()
    rclpy.spin(node)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
