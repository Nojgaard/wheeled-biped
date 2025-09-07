#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from wobl_msgs.msg import Topics


class Joy2CmdNode(Node):
    def __init__(self):
        super().__init__('joy2cmd')
        
        # Create subscriber for joystick messages
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            Topics.VELOCITY_COMMAND,
            10
        )
        
        # Joystick configuration
        # Typically: Left stick Y-axis for linear velocity, Right stick X-axis for angular velocity
        self.linear_axis = 1   # Left stick Y-axis (up/down)
        self.angular_axis = 2  # Right stick X-axis (left/right)
        
        # Scaling factors for velocity commands
        self.max_linear_velocity = 0.2   # m/s
        self.max_angular_velocity = 1.0  # rad/s

        # Deadzone to prevent drift
        self.deadzone = 0.1
        
        self.get_logger().info(f'Joy2Cmd node started. Publishing to {Topics.VELOCITY_COMMAND}')
        self.get_logger().info(f'Linear axis: {self.linear_axis}, Angular axis: {self.angular_axis}')
        self.get_logger().info(f'Max linear vel: {self.max_linear_velocity} m/s, Max angular vel: {self.max_angular_velocity} rad/s')

    def apply_deadzone(self, value, deadzone):
        """Apply deadzone to joystick input to prevent drift."""
        if abs(value) < deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        """Convert joystick input to velocity commands."""
        try:
            # Extract axis values with bounds checking
            if len(msg.axes) <= max(self.linear_axis, self.angular_axis):
                self.get_logger().warn(f'Not enough joystick axes. Got {len(msg.axes)}, need at least {max(self.linear_axis, self.angular_axis) + 1}')
                return
            
            # Get raw joystick values
            linear_raw = msg.axes[self.linear_axis]
            angular_raw = msg.axes[self.angular_axis]
            
            # Apply deadzone
            linear_filtered = self.apply_deadzone(linear_raw, self.deadzone)
            angular_filtered = self.apply_deadzone(angular_raw, self.deadzone)
            
            # Scale to desired velocity ranges
            linear_velocity = -linear_filtered * self.max_linear_velocity
            angular_velocity = -angular_filtered * self.max_angular_velocity
            
            # Create and populate Twist message
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = angular_velocity
            
            # Publish the command
            self.cmd_vel_publisher.publish(twist_msg)
            
            # Log for debugging (only when non-zero to reduce spam)
            if abs(linear_velocity) > 0.01 or abs(angular_velocity) > 0.01:
                self.get_logger().debug(f'Publishing cmd_vel: linear.x={linear_velocity:.3f}, angular.z={angular_velocity:.3f}')
                
        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Joy2CmdNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()