import threading

import numpy as np
import rclpy
from dm_env import TimeStep
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from wobl.sim.application import Application
from wobl.sim.balance_task import BalanceTask
from wobl.sim.robot import Robot


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        self._action = np.zeros(4)

        robot = Robot(assets_dir="../assets")
        self._task = BalanceTask(robot)
        self._app = Application(self._task, self.update_sim)
        self._thread = threading.Thread(target=self._app.launch, daemon=True)
        self._thread.start()

        self.subscriber = self.create_subscription(
            Float32MultiArray, "joint_commands", self.command_callback, 10
        )
        self.publisher = self.create_publisher(Float32MultiArray, "orientation", 10)

    def command_callback(self, msg):
        self._action[:] = msg.data

    def publish_state(self):
        state_msg = Float32MultiArray()
        state_msg.data = self.sim.data.qpos.tolist()
        self.publisher.publish(state_msg)

    def update_sim(self, timestep: TimeStep):
        state_msg = Float32MultiArray()
        state_msg.data = timestep.observation["robot/orientation"]
        self.publisher.publish(state_msg)
        return self._action

    def shutdown(self):
        """Gracefully shutdown the node"""
        self.get_logger().info("MuJoCo Viewer closed, shutting down...")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    rclpy.spin(node)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
