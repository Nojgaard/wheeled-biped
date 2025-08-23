#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
import dearpygui.dearpygui as dpg
import threading
import yaml
import os
import sys
from ament_index_python.packages import get_package_share_directory
from wobl_msgs.msg import Topics
from geometry_msgs.msg import Twist


class TuneGainsNode(Node):
    """
    ROS 2 node for tuning PID controller gains with a GUI interface.

    This node creates a DearPyGui interface for adjusting PID gains and
    max_pitch parameters of a running controller node. Changes are
    batched and sent to the controller every 0.5 seconds.
    """

    def __init__(self):
        super().__init__("pid_gains_tuner")

        # Configuration
        self.controller_name = "wobl_controller"
        self.update_rate = 0.5  # seconds
        self.param_names = ["vel2pitch_gains", "pitch2vel_gains", "max_pitch"]

        # Store parameters, track changes
        self.params = {}
        self.changed_params = set()

        # Velocity command publisher
        self.vel_pub = self.create_publisher(Twist, Topics.VELOCITY_COMMAND, 10)

        # Load parameters from file and initialize services
        self._load_params_from_yaml()
        self._init_ros_services()

        # Create GUI and start update timer
        self._setup_gui()
        self.update_timer = self.create_timer(
            self.update_rate, self._send_param_updates
        )

        # Start the GUI in a separate thread
        self.gui_thread = threading.Thread(target=self._run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

        self.get_logger().info("PID Gains Tuner started")

    def _load_params_from_yaml(self):
        """Load parameter defaults from the controller's YAML file"""
        try:
            config_path = os.path.join("wobl_control", "config", "params.yaml")

            with open(config_path, "r") as file:
                yaml_data = yaml.safe_load(file)

            # Extract parameters from the YAML file
            controller_params = yaml_data.get("wobl_controller", {}).get(
                "ros__parameters", {}
            )

            # Check if required parameters exist
            missing_params = []
            for param in self.param_names:
                if param not in controller_params:
                    missing_params.append(param)

            if missing_params:
                self.get_logger().error(
                    f"Required parameters missing in {config_path}: {', '.join(missing_params)}"
                )
                sys.exit(1)

            # Initialize parameters from YAML
            self.params = {
                param: controller_params[param] for param in self.param_names
            }
            self.get_logger().info(f"Loaded parameter defaults from {config_path}")

        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found: {config_path}")
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f"Error loading parameters: {e}")
            sys.exit(1)

    def _init_ros_services(self):
        """Initialize ROS 2 parameter services"""
        # Create parameter service clients
        self.set_param_client = self.create_client(
            SetParameters, f"/{self.controller_name}/set_parameters"
        )

        # Wait for services with timeout
        if not self.set_param_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                f"Controller node {self.controller_name} not available"
            )
            sys.exit(1)

    def _setup_gui(self):
        """Setup the DearPyGui interface"""
        dpg.create_context()

        # Set window size
        width, height = 600, 500
        dpg.create_viewport(title="PID Gains Tuner", width=width, height=height)
        dpg.setup_dearpygui()

        # Create the main window
        with dpg.window(
            label="PID Gains Tuner", width=width, height=height
        ) as self.main_window:
            # Title
            dpg.add_text("Wheeled Biped PID Gains Tuner")
            dpg.add_separator()

            # Create tabs for different parameter groups
            with dpg.tab_bar():
                # Velocity to Pitch PID Gains
                with dpg.tab(label="Velocity to Pitch"):
                    dpg.add_text("Velocity to Pitch Controller Gains")
                    dpg.add_separator()

                    # Create sliders for kp, ki, kd
                    for i, gain_name in enumerate(["kp", "ki", "kd"]):
                        dpg.add_text(
                            f"{gain_name}: {['Proportional', 'Integral', 'Derivative'][i]} Gain"
                        )
                        slider_id = f"vel2pitch_{gain_name}_slider"

                        # Use a closure to capture the correct index for each slider
                        def create_callback(idx):
                            return lambda s, a: self._mark_param_changed(
                                "vel2pitch_gains", idx, a
                            )

                        setattr(
                            self,
                            slider_id,
                            dpg.add_slider_float(
                                label=f"{gain_name}##vel2pitch",
                                default_value=self.params["vel2pitch_gains"][i],
                                min_value=0.0,
                                max_value=[0.5, 0.5, 0.5][i],
                                callback=create_callback(i),
                                width=width - 50,
                            ),
                        )
                # Pitch to Velocity PID Gains
                with dpg.tab(label="Pitch to Velocity"):
                    dpg.add_text("Pitch to Velocity Controller Gains")
                    dpg.add_separator()

                    # Create sliders for kp, ki, kd
                    for i, gain_name in enumerate(["kp", "ki", "kd"]):
                        dpg.add_text(
                            f"{gain_name}: {['Proportional', 'Integral', 'Derivative'][i]} Gain"
                        )
                        slider_id = f"pitch2vel_{gain_name}_slider"

                        # Use a closure to capture the correct index for each slider
                        def create_callback(idx):
                            return lambda s, a: self._mark_param_changed(
                                "pitch2vel_gains", idx, a
                            )

                        setattr(
                            self,
                            slider_id,
                            dpg.add_slider_float(
                                label=f"{gain_name}##pitch2vel",
                                default_value=self.params["pitch2vel_gains"][i],
                                min_value=0.0,
                                max_value=[13.0, 1.0, 2.0][i],
                                callback=create_callback(i),
                                width=width - 50,
                            ),
                        )

                # Max Pitch Parameter
                with dpg.tab(label="Physical Parameters"):
                    dpg.add_text("Robot Physical Parameters")
                    dpg.add_separator()

                    # Slider for max_pitch
                    dpg.add_text("Maximum Pitch Angle (radians)")
                    self.max_pitch_slider = dpg.add_slider_float(
                        label="max_pitch",
                        default_value=self.params["max_pitch"],
                        min_value=0.0,
                        max_value=0.5,  # About 30 degrees
                        callback=lambda s, a: self._mark_param_changed(
                            "max_pitch", None, a
                        ),
                        width=width - 50,
                    )

                # Velocity Command Tab
                with dpg.tab(label="Velocity Command"):
                    dpg.add_text("Send Velocity Commands (Twist)")
                    dpg.add_separator()
                    self.linear_vel_slider = dpg.add_slider_float(
                        label="Linear Velocity (m/s)",
                        default_value=0.0,
                        min_value=-0.8,
                        max_value=0.8,
                        callback=lambda s, a: self._publish_twist(),
                        width=width - 50,
                    )
                    self.angular_vel_slider = dpg.add_slider_float(
                        label="Angular Velocity (rad/s)",
                        default_value=0.0,
                        min_value=-2.0,
                        max_value=2.0,
                        callback=lambda s, a: self._publish_twist(),
                        width=width - 50,
                    )

            # Buttons for save/reset
            with dpg.group(horizontal=True):
                dpg.add_button(label="Save Configuration", callback=self._save_config)
                dpg.add_button(
                    label="Reset to Defaults", callback=self._reset_to_defaults
                )
                # Status display1
                dpg.add_separator()
                self.status_text = dpg.add_text("Ready")
                self.update_status_text = dpg.add_text("No updates pending")

    def _publish_twist(self):
        """Publish a Twist message with values from sliders"""
        try:
            twist = Twist()
            twist.linear.x = dpg.get_value(self.linear_vel_slider)
            twist.angular.z = dpg.get_value(self.angular_vel_slider)
            self.vel_pub.publish(twist)
            dpg.set_value(
                self.status_text,
                f"Published Twist: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}",
            )
        except Exception as e:
            self.get_logger().error(f"Error publishing Twist: {e}")
            dpg.set_value(self.status_text, f"Error publishing Twist: {e}")

    def _run_gui(self):
        """Run the GUI event loop"""
        dpg.show_viewport()
        dpg.set_primary_window(self.main_window, True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def _mark_param_changed(self, param_name, index, value):
        """Mark a parameter as changed for the next update cycle"""
        # Update local copy
        if index is not None:
            # For array parameters (PID gains)
            self.params[param_name][index] = value

            # Log the full array when updating a component
            self.get_logger().debug(
                f"Updated {param_name}[{index}] to {value:.4f}, full array: {self.params[param_name]}"
            )
        else:
            # For scalar parameters
            self.params[param_name] = value

        # Mark as changed
        self.changed_params.add(param_name)

        # Update status text
        if index is not None:
            gain_name = ["kp", "ki", "kd"][index]
            dpg.set_value(
                self.status_text, f"Changed {param_name} {gain_name} to {value:.4f}"
            )
        else:
            dpg.set_value(self.status_text, f"Changed {param_name} to {value:.4f}")

        # Update pending changes text
        dpg.set_value(
            self.update_status_text,
            f"Pending updates: {', '.join(self.changed_params)}",
        )

    def _send_param_updates(self):
        """Send batched parameter updates to the controller"""
        if not self.changed_params:
            return

        self.get_logger().info(f"Updating parameters: {', '.join(self.changed_params)}")

        for param_name in self.changed_params:
            self._set_parameter(param_name, self.params[param_name])

        # Clear the changed parameters set
        self.changed_params.clear()

        # Update status
        dpg.set_value(self.update_status_text, "No updates pending")

    def _set_parameter(self, param_name, value):
        """Set a parameter on the controller node"""
        try:
            request = SetParameters.Request()

            # Create parameter
            parameter = Parameter()
            parameter.name = param_name

            # Create parameter value based on type
            param_value = ParameterValue()

            if isinstance(value, list):
                # For arrays (PID gains)
                param_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param_value.double_array_value = value
                self.get_logger().info(
                    f"Setting parameter {param_name} to array: {value}"
                )
            else:
                # For scalar values
                param_value.type = ParameterType.PARAMETER_DOUBLE
                param_value.double_value = value
                self.get_logger().info(
                    f"Setting parameter {param_name} to value: {value}"
                )

            parameter.value = param_value
            request.parameters = [parameter]

            # Send request and wait for it to complete
            self.set_param_client.call_async(request)

        except Exception as e:
            self.get_logger().error(f"Error setting parameter {param_name}: {str(e)}")
            import traceback

            self.get_logger().error(traceback.format_exc())

    def _save_config(self):
        """Save current configuration to the YAML file"""
        try:
            # Get the path to the params.yaml file
            config_path = os.path.join("wobl_control", "config", "params.yaml")

            # Load the YAML file to get the full structure
            with open(config_path, "r") as file:
                yaml_data = yaml.safe_load(file)

            # Update the parameters
            ros_params = yaml_data[self.controller_name]["ros__parameters"]
            for key, value in self.params.items():
                ros_params[key] = value

            # Write back to the file
            with open(config_path, "w") as file:
                yaml.dump(yaml_data, file, default_flow_style=False)

            self.get_logger().info(f"Saved configuration to {config_path}")
            dpg.set_value(self.status_text, f"Configuration saved")

        except Exception as e:
            self.get_logger().error(f"Error saving configuration: {e}")
            dpg.set_value(self.status_text, f"Error saving configuration: {e}")

    def _reset_to_defaults(self):
        """Reset parameters to original YAML values"""
        try:
            # Reload parameters from YAML
            self._load_params_from_yaml()

            # Mark all as changed to trigger updates
            self.changed_params = set(self.params.keys())

            # Update UI sliders
            for i, gain_name in enumerate(["kp", "ki", "kd"]):
                dpg.set_value(
                    getattr(self, f"vel2pitch_{gain_name}_slider"),
                    self.params["vel2pitch_gains"][i],
                )
                dpg.set_value(
                    getattr(self, f"pitch2vel_{gain_name}_slider"),
                    self.params["pitch2vel_gains"][i],
                )

            dpg.set_value(self.max_pitch_slider, self.params["max_pitch"])

            # Force immediate update
            self._send_param_updates()

            # Set velocity sliders to zero
            dpg.set_value(self.linear_vel_slider, 0.0)
            dpg.set_value(self.angular_vel_slider, 0.0)

            # Publish zero velocity Twist
            try:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.vel_pub.publish(twist)
                dpg.set_value(
                    self.status_text,
                    "Reset to default values and published zero velocity",
                )
            except Exception as e:
                self.get_logger().error(f"Error publishing zero Twist: {e}")
                dpg.set_value(self.status_text, f"Error publishing zero Twist: {e}")

        except Exception as e:
            self.get_logger().error(f"Error resetting to defaults: {e}")
            dpg.set_value(self.status_text, f"Error resetting to defaults: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        # Create the tuner node
        tuner = TuneGainsNode()

        # Run ROS loop in a separate thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(tuner)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Wait for threads to complete
        while executor_thread.is_alive():
            try:
                executor_thread.join(1.0)
            except KeyboardInterrupt:
                break

    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        # Clean shutdown
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
