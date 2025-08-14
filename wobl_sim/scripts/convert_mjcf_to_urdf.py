from mjcf_urdf_simple_converter import convert
import os

script_path = os.path.dirname(__file__)
mjcf_path = os.path.join(script_path, "../mjcf/plane_world.xml")
urdf_path = os.path.join(script_path, "../urdf/robot.urdf")

if not os.path.exists(mjcf_path):
    raise FileNotFoundError(f"MJCF file not found: {mjcf_path}")

convert(mjcf_path, urdf_path, asset_file_prefix="package://real/urdf/")
print(f"Converted MJCF to URDF: {urdf_path}")