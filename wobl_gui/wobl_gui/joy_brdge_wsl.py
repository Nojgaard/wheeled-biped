import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import struct

UDP_PORT = 5005

class JoyBridge(Node):
    def __init__(self):
        super().__init__('joy_bridge')
        self.pub = self.create_publisher(Joy, '/joy', 10)

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', UDP_PORT))

        self.get_logger().info(f'Listening for joystick on UDP port {UDP_PORT}')
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        self.sock.settimeout(0.001)
        try:
            data, _ = self.sock.recvfrom(1024)
            # Assume 6 axes + 11 buttons (adjust to match Windows side)
            axes = list(struct.unpack('6f11B', data))
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = axes[:6]
            msg.buttons = axes[6:]
            self.pub.publish(msg)
        except socket.timeout:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = JoyBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()