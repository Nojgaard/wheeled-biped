import threading

import numpy as np
import rclpy
from dm_env import TimeStep
from rclpy.node import Node

from wobl_sim.application import Application
from wobl_sim.balance_task import BalanceTask
from wobl_sim.robot import Robot

from wobl_messages.msg import JointCommand
from sensor_msgs.msg import Imu, JointState


class MujocoBridgeNode(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        self._action = np.zeros(4)

        self._robot = Robot()
        self._task = BalanceTask(self._robot)
        self._app = Application(self._task, self.update_sim)
        self._thread = threading.Thread(target=self._app.launch, daemon=True)
        self._thread.start()

        self.subscriber = self.create_subscription(
            JointCommand, "joint_commands", self.command_callback, 1
        )

        self._publish_imu_state = self.create_publisher(Imu, "imu/data", 1)
        self._publish_joint_state = self.create_publisher(JointState, "joint_states", 1)

    def command_callback(self, msg: JointCommand):
        self._action[:2] = msg.position[:2]
        self._action[2:] = msg.velocity[2:]

    def publish_joints(self, timestep: TimeStep):
        joint_state = JointState()
        joint_state.name = self._robot.joint_names
        joint_state.position = timestep.observation["robot/joint_positions"]
        joint_state.velocity = timestep.observation["robot/joint_velocities"]
        joint_state.effort = timestep.observation["robot/joint_efforts"]
        self._publish_joint_state.publish(joint_state)

    def publish_imu(self, timestep: TimeStep):
        if np.linalg.norm(timestep.observation["robot/orientation"]) == 0:
            return

        imu = Imu()

        q = timestep.observation["robot/orientation"]
        imu.orientation.w = q[0]
        imu.orientation.x = q[1]
        imu.orientation.y = q[2]
        imu.orientation.z = q[3]
        
        avel = timestep.observation["robot/angular_velocity"]
        imu.angular_velocity.x = avel[0]
        imu.angular_velocity.y = avel[1]
        imu.angular_velocity.z = avel[2]

        lacc = timestep.observation["robot/linear_acceleration"] 
        imu.linear_acceleration.x = lacc[0]
        imu.linear_acceleration.y = lacc[1]
        imu.linear_acceleration.z = lacc[2]

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        self._publish_imu_state.publish(imu)

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
