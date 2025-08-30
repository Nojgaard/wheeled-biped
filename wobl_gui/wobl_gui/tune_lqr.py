#!/usr/bin/env python3

import dearpygui.dearpygui as dpg
import numpy as np
import control
from build.wobl_sim.build.lib.wobl_sim.robot import Robot
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import threading
import yaml
import os
from dm_control.locomotion.arenas import Floor
from dm_control import mjcf

def compute_mass(model):
    total_mass = 0.0
    for i in range(model.nbody):
        total_mass += model.body_mass[i]
    return total_mass

def compute_total_com(model, data):
    total_mass = 0.0
    com = np.zeros(3)

    for i in range(model.nbody):
        body_mass = model.body_mass[i]
        # Get world position of this body's CoM
        body_com = data.xipos[i]
        com += body_mass * body_com
        total_mass += body_mass

    return com / total_mass if total_mass > 0 else com

def find_equilibrium(robot: Robot, physics: mjcf.Physics):
    com = compute_total_com(physics.model, physics.data)
    com_x_offset = com[0]
    com_z_offset = com[2] - robot.mjcf_model.find("joint", "L_foot").pos[2]
    theta_eq = -com_x_offset / com_z_offset
    return theta_eq

def compute_com_height(robot: Robot, physics: mjcf.Physics):
    com = compute_total_com(physics.model, physics.data)[[0, 2]]
    wheel_pos = robot.mjcf_model.find("joint", "L_foot").pos[[0, 2]]

    return np.linalg.norm(com - wheel_pos)


class LQRTunerGUI:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        self.node = rclpy.create_node('lqr_tuner_gui')
        
        # ROS2 parameter client for wobl_controller
        self.param_client = self.node.create_client(SetParameters, '/wobl_controller/set_parameters')
        

        robot = Robot()
        arena = Floor(reflectance=0.0)
        arena.add_free_entity(robot)

        physics = mjcf.Physics.from_mjcf_model(arena.mjcf_model)

        self.g = 9.81  # gravity
        self.m = compute_mass(physics.model)   # total mass (kg)
        self.l = compute_com_height(robot, physics)   # length to the center of mass from wheel axis (m)
        self.r = 0.04  # Wheel radius (m)
        self.theta_eq = find_equilibrium(robot, physics)

        # Default values for tuning
        self.pitch_offset = 0.0313
        self.Q_diag = [3.0, 0.2, 1.5, 1.0]  # theta, theta_dot, x, x_dot
        self.R_value = 0.8
        self.K_values = [-6.96610096, -0.77409491, 2.1227282, 1.11803399]
        
        # Config file path
        self.config_path = "/home/nnoej/projects/wheeled-biped/wobl_control/config/params.yaml"
        
        # Start ROS spinning in a separate thread
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Initialize GUI
        self._setup_gui()
    
    def _ros_spin(self):
        """Spin ROS2 in a separate thread"""
        rclpy.spin(self.node)
    
    def _setup_gui(self):
        """Set up the dearpygui interface"""
        dpg.create_context()
        
        # Main window
        with dpg.window(label="LQR Gain Tuner", tag="primary_window", width=800, height=600):
            
            # System Parameters Section
            dpg.add_separator()
            dpg.add_text("System Parameters", color=[255, 255, 0])
            dpg.add_separator()
            
            with dpg.group(horizontal=True):
                dpg.add_text("Mass (kg):")
                dpg.add_input_double(tag="mass_input", default_value=self.m, width=100, format="%.3f")
                dpg.add_text("  COM Height (m):")
                dpg.add_input_double(tag="com_height_input", default_value=self.l, width=100, format="%.3f")
                dpg.add_text("  Wheel Radius (m):")
                dpg.add_input_double(tag="wheel_radius_input", default_value=self.r, width=100, format="%.3f")
            
            dpg.add_separator()
            
            # Pitch Offset Section
            dpg.add_text("Control Parameters", color=[255, 255, 0])
            dpg.add_separator()
            
            with dpg.group(horizontal=True):
                dpg.add_text("Pitch Offset (rad):")
                dpg.add_input_double(tag="pitch_offset_input", default_value=self.pitch_offset, width=150, format="%.6f")
            
            dpg.add_separator()
            
            # Q Matrix Section
            dpg.add_text("Q Matrix (State Weighting)", color=[255, 255, 0])
            dpg.add_text("Diagonal elements: [pitch, pitch_rate, velocity, position_integral]")
            dpg.add_separator()
            
            with dpg.group(horizontal=True):
                dpg.add_text("Q[0,0] (pitch):")
                dpg.add_input_double(tag="q0_input", default_value=self.Q_diag[0], width=100, format="%.3f")
                dpg.add_text("  Q[1,1] (pitch_rate):")
                dpg.add_input_double(tag="q1_input", default_value=self.Q_diag[1], width=100, format="%.3f")
            
            with dpg.group(horizontal=True):
                dpg.add_text("Q[2,2] (velocity):")
                dpg.add_input_double(tag="q2_input", default_value=self.Q_diag[2], width=100, format="%.3f")
                dpg.add_text("  Q[3,3] (position):")
                dpg.add_input_double(tag="q3_input", default_value=self.Q_diag[3], width=100, format="%.3f")
            
            dpg.add_separator()
            
            # R Matrix Section
            dpg.add_text("R Matrix (Control Weighting)", color=[255, 255, 0])
            dpg.add_separator()
            
            with dpg.group(horizontal=True):
                dpg.add_text("R[0,0] (control effort):")
                dpg.add_input_double(tag="r_input", default_value=self.R_value, width=100, format="%.3f")
            
            dpg.add_separator()
            
            # Compute Button
            dpg.add_button(label="Compute LQR Gains", callback=self._compute_lqr_gains, width=200, height=40)
            
            dpg.add_separator()
            
            # Results Section
            dpg.add_text("Computed LQR Gains (K)", color=[0, 255, 0])
            dpg.add_separator()
            
            with dpg.group(horizontal=True):
                dpg.add_text("K[0] (pitch):")
                dpg.add_input_double(tag="k0_output", default_value=self.K_values[0], width=120, format="%.8f", readonly=True)
                dpg.add_text("  K[1] (pitch_rate):")
                dpg.add_input_double(tag="k1_output", default_value=self.K_values[1], width=120, format="%.8f", readonly=True)
            
            with dpg.group(horizontal=True):
                dpg.add_text("K[2] (velocity):")
                dpg.add_input_double(tag="k2_output", default_value=self.K_values[2], width=120, format="%.8f", readonly=True)
                dpg.add_text("  K[3] (position):")
                dpg.add_input_double(tag="k3_output", default_value=self.K_values[3], width=120, format="%.8f", readonly=True)
            
            dpg.add_separator()
            
            # Action buttons
            with dpg.group(horizontal=True):
                dpg.add_button(label="Set ROS Parameters", callback=self._set_ros_parameters, width=150, height=30)
                dpg.add_button(label="Save to Config File", callback=self._save_to_config, width=150, height=30)
                dpg.add_button(label="Load from Config", callback=self._load_from_config, width=150, height=30)
            
            dpg.add_separator()
            
            # Status text
            dpg.add_text("Status: Ready", tag="status_text", color=[0, 255, 0])
    
    def _compute_lqr_gains(self):
        """Compute LQR gains using the control library"""
        try:
            # Update system parameters
            self.m = dpg.get_value("mass_input")
            self.l = dpg.get_value("com_height_input")
            self.r = dpg.get_value("wheel_radius_input")
            
            # System matrices (from the notebook)
            A = np.array([[0, 1, 0],
                          [self.g/self.l, 0, 0],
                          [-self.g/self.m, 0, 0]])

            B = np.array([[0],
                          [-1 / (self.m * (self.l ** 2))],
                          [1 / self.m]])
            
            # Get Q and R matrices from GUI
            q_values = [
                dpg.get_value("q0_input"),
                dpg.get_value("q1_input"),
                dpg.get_value("q2_input"),
                dpg.get_value("q3_input")
            ]
            Q = np.diag(q_values)
            R = np.array([[dpg.get_value("r_input")]])
            
            # Integral action matrix (integrate velocity)
            C_integral = np.array([[0, 0, 1]])
            
            # Compute LQR gains
            K, _, _ = control.lqr(A, B, Q, R, integral_action=C_integral)
            
            # Update the K values and GUI
            self.K_values = K[0].tolist()  # K is a 2D array, we want the first (and only) row
            
            dpg.set_value("k0_output", self.K_values[0])
            dpg.set_value("k1_output", self.K_values[1])
            dpg.set_value("k2_output", self.K_values[2])
            dpg.set_value("k3_output", self.K_values[3])
            
            dpg.set_value("status_text", "Status: LQR gains computed successfully!")
            
        except Exception as e:
            dpg.set_value("status_text", f"Status: Error computing gains: {str(e)}")
    
    def _set_ros_parameters(self):
        """Set the computed parameters to the ROS2 node"""
        try:
            if not self.param_client.wait_for_service(timeout_sec=2.0):
                dpg.set_value("status_text", "Status: ROS parameter service not available!")
                return
            
            # Create parameter list
            parameters = []
            
            # Pitch offset parameter
            pitch_offset_param = Parameter()
            pitch_offset_param.name = "offset_pitch"
            pitch_offset_param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=dpg.get_value("pitch_offset_input"))  # type 3 = double
            parameters.append(pitch_offset_param)
            
            # LQR K parameter
            lqr_k_param = Parameter()
            lqr_k_param.name = "lqr_K"
            lqr_k_param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=self.K_values)  # type 7 = double_array
            parameters.append(lqr_k_param)
            
            # Create service request
            request = SetParameters.Request()
            request.parameters = parameters
            
            # Call service
            future = self.param_client.call_async(request)
            
            # Wait for response (non-blocking)
            def check_result():
                if future.done():
                    try:
                        result = future.result()
                        if result.results[0].successful and result.results[1].successful:
                            dpg.set_value("status_text", "Status: ROS parameters set successfully!")
                        else:
                            reasons = [r.reason for r in result.results if not r.successful]
                            dpg.set_value("status_text", f"Status: Failed to set parameters: {', '.join(reasons)}")
                    except Exception as e:
                        dpg.set_value("status_text", f"Status: Error setting parameters: {str(e)}")
                else:
                    # Check again in 100ms
                    threading.Timer(0.1, check_result).start()
            
            check_result()
            
        except Exception as e:
            dpg.set_value("status_text", f"Status: Error calling ROS service: {str(e)}")
    
    def _save_to_config(self):
        """Save current parameters to the config file"""
        try:
            # Read existing config
            config_data = {}
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    config_data = yaml.safe_load(f) or {}
            
            # Ensure the structure exists
            if 'wobl_controller' not in config_data:
                config_data['wobl_controller'] = {}
            if 'ros__parameters' not in config_data['wobl_controller']:
                config_data['wobl_controller']['ros__parameters'] = {}
            
            params = config_data['wobl_controller']['ros__parameters']
            
            # Update parameters
            params['offset_pitch'] = dpg.get_value("pitch_offset_input")
            params['lqr_K'] = self.K_values
            
            # Write back to file
            with open(self.config_path, 'w') as f:
                yaml.dump(config_data, f, default_flow_style=False, indent=2)
            
            dpg.set_value("status_text", "Status: Configuration saved to file!")
            
        except Exception as e:
            dpg.set_value("status_text", f"Status: Error saving config: {str(e)}")
    
    def _load_from_config(self):
        """Load parameters from the config file"""
        try:
            if not os.path.exists(self.config_path):
                dpg.set_value("status_text", "Status: Config file not found!")
                return
            
            with open(self.config_path, 'r') as f:
                config_data = yaml.safe_load(f)
            
            params = config_data.get('wobl_controller', {}).get('ros__parameters', {})
            
            # Load pitch offset
            if 'offset_pitch' in params:
                dpg.set_value("pitch_offset_input", params['offset_pitch'])
            
            # Load LQR K values
            if 'lqr_K' in params:
                self.K_values = params['lqr_K']
                dpg.set_value("k0_output", self.K_values[0])
                dpg.set_value("k1_output", self.K_values[1])
                dpg.set_value("k2_output", self.K_values[2])
                dpg.set_value("k3_output", self.K_values[3])
            
            dpg.set_value("status_text", "Status: Configuration loaded from file!")
            
        except Exception as e:
            dpg.set_value("status_text", f"Status: Error loading config: {str(e)}")
    
    def run(self):
        """Run the GUI"""
        dpg.create_viewport(title="LQR Gain Tuner", width=850, height=650)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("primary_window", True)
        
        try:
            dpg.start_dearpygui()
        finally:
            dpg.destroy_context()
            self.node.destroy_node()
            rclpy.shutdown()

def main():
    """Main entry point"""
    try:
        gui = LQRTunerGUI()
        gui.run()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == "__main__":
    main()