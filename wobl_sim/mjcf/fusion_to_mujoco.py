import numpy as np
from dm_control import mjcf
from scipy.spatial.transform import Rotation as R


def rotate_pos(pos):
    """Rotate position vector -90° around Z (X,Y swapped)."""
    x, y, z = pos
    return np.array([y, -x, z])


def rotate_quat(quat):
    """Apply -90° rotation around Z to a quaternion."""
    if quat is None:
        return None
    r_orig = R.from_quat(quat)  # xyzw format
    r_rot = R.from_euler('z', -90, degrees=True)
    r_new = r_rot * r_orig
    return r_new.as_quat()  # xyzw


def process_body(geom):
    if geom.pos is not None:
        geom.pos = rotate_pos(np.array(geom.pos))
    if geom.quat is None:
        geom.quat = (1, 0, 0, 0)
    quat_wxyz = geom.quat
    quat_xyzw = np.roll(quat_wxyz, -1)
    new_quat_xyzw = rotate_quat(quat_xyzw)
    geom.quat = np.roll(new_quat_xyzw, 1)



# === Load MJCF ===
model = mjcf.from_path("wobl_description/mjcf/robot.xml")

# === Process all bodies ===
for body in model.worldbody.find_all('geom'):
    process_body(body)


# === Save the updated model ===
with open("wobl_description/mjcf/rotated_robot.xml", "w") as f:
    f.write(model.to_xml_string())

print("✅ Model updated and saved as rotated_model.xml")