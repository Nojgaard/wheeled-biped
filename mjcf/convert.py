from mjcf_urdf_simple_converter import convert
#convert("assets/robot.xml", "assets/robot.urdf")
# or, if you are using it in your ROS package and would like for the mesh directories to be resolved correctly, set meshfile_prefix, for example:
convert("assets/robot.xml", "assets/test/robot.urdf", asset_file_prefix="real")