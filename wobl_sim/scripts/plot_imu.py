import os
import plotly.express as px
from wobl_msgs.msg import Topics
import rosbag2_py
from rclpy.serialization import deserialize_message
import math
from rosidl_runtime_py.utilities import get_message
import pandas as pd
from plotly.subplots import make_subplots
import plotly.graph_objects as go


import webbrowser


class PrintUrl:
    def open(self, url, *args, **kwargs):
        print(url)


webbrowser.register("print", PrintUrl)


def read_messages(input_bag: str, topic_names=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic_names is None or topic in topic_names:
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
    del reader


def normalize_timestamps(data):
    if not data:
        return []
    t0 = data[0][0]
    return [((t - t0) / 1e9, msg) for t, msg in data]


def find_first_mcap_file(directory: str) -> str:
    """
    Returns the path to the first .mcap file found in the given directory.
    Raises FileNotFoundError if none found.
    """
    for entry in os.listdir(directory):
        if entry.endswith(".mcap"):
            return os.path.join(directory, entry)
    raise FileNotFoundError(f"No .mcap file found in {directory}")


def stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def collect_msgs(name, mcap_path):
    msgs = [(name, topic, msg) for topic, msg, t in read_messages(mcap_path)]
    t0 = stamp_to_seconds(msgs[0][2].header.stamp)
    for _, _, msg in msgs:
        msg.header.stamp = stamp_to_seconds(msg.header.stamp) - t0
    return msgs

def linear_filter(velocities, alpha):
    """Apply a simple linear filter to the velocity list."""
    filtered = []
    vel = velocities[0] if velocities else 0.0
    for new_vel in velocities:
        vel = alpha * vel + (1 - alpha) * new_vel
        filtered.append(vel)
    return filtered

def msgs_to_df(msgs):
    # Converts messages to a pandas DataFrame for easier plotting
    data = []
    last_time = None
    last_pitch = None
    for name, topic, msg in msgs:
        stamp = msg.header.stamp
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        ori = msg.orientation
        # Attitude (pitch, roll)
        pitch = math.atan2(
            2.0 * (ori.w * ori.y - ori.z * ori.x),
            1.0 - 2.0 * (ori.y**2 + ori.z**2),
        )
        roll = math.atan2(
            2.0 * (ori.w * ori.x + ori.y * ori.z),
            1.0 - 2.0 * (ori.x**2 + ori.y**2),
        )

        pitch_rate_dt = 0
        if last_time is not None:
            pitch_rate_dt = (pitch - last_pitch) / (stamp - last_time)
        last_time = stamp
        last_pitch = pitch

        data.append({
            "time": stamp,
            "name": name,
            "topic": topic,
            "acc_x": acc.x,
            "acc_y": acc.y,
            "acc_z": acc.z,
            "gyro_x": gyro.x,
            "gyro_y": gyro.y,
            "gyro_z": gyro.z,
            "pitch": pitch,
            "roll": roll,
            "pitch_rate_dt": pitch_rate_dt
        })
    return pd.DataFrame(data)

def plot_linear_acceleration(df, title):
    df_melt = df.melt(id_vars=["time", "name"], value_vars=["acc_x", "acc_y", "acc_z"], var_name="axis", value_name="acceleration")
    fig = px.line(
        df_melt,
        x="time",
        y="acceleration",
        color="axis",
        line_dash="name",
        labels={"time": "Time [s]", "acceleration": "Linear Acceleration [m/s²]", "axis": "Axis"},
        title=title,
    )
    #fig.update_traces(mode="lines+markers")
    fig.show(renderer="browser", using="print")

def plot_gyro(df, title):
    df_melt = df.melt(id_vars=["time", "name"], value_vars=["gyro_x", "gyro_y", "gyro_z"], var_name="axis", value_name="angular_velocity")
    fig = px.line(
        df_melt,
        x="time",
        y="angular_velocity",
        color="axis",
        line_dash="name",
        labels={"time": "Time [s]", "angular_velocity": "Angular Velocity [rad/s]", "axis": "Axis"},
        title=title,
    )
    #fig.update_traces(mode="lines+markers")
    fig.show(renderer="browser", using="print")

def plot_gyro_dt(df, title):
    df_melt = df.melt(id_vars=["time", "name"], value_vars=["pitch", "pitch_rate", "gyro_y"], var_name="axis", value_name="angular_velocity")
    fig = px.line(
        df_melt,
        x="time",
        y="angular_velocity",
        color="axis",
        line_dash="name",
        labels={"time": "Time [s]", "angular_velocity": "Angular Velocity [rad/s]", "axis": "Axis"},
        title=title,
    )
    #fig.update_traces(mode="lines+markers")
    fig.show(renderer="browser", using="print")

def plot_attitude(df, title):
    df_melt = df.melt(id_vars=["time", "name"], value_vars=["pitch", "roll"], var_name="attitude", value_name="angle")
    fig = px.line(
        df_melt,
        x="time",
        y="angle",
        color="attitude",
        line_dash="name",
        labels={"time": "Time [s]", "angle": "Angle [rad]", "attitude": "Attitude"},
        title=title,
    )
    #fig.update_traces(mode="lines+markers")
    fig.show(renderer="browser", using="print")


if __name__ == "__main__":
    real_mcap = find_first_mcap_file("data/bag_real_imu")
    sim_mcap = find_first_mcap_file("data/bag_sim_imu")
    msgs = collect_msgs("real", real_mcap)
    msgs += collect_msgs("sim", sim_mcap)
    df = msgs_to_df(msgs)
    df["pitch_rate"] = linear_filter(list(df["gyro_y"]), alpha=0.5)

    #plot_linear_acceleration(df, "Real IMU Linear Acceleration")
    #plot_gyro(df, "Real IMU Angular Velocity")
    plot_gyro_dt(df, "Real IMU Angular Velocity Rate of Change")
    #plot_attitude(df, "Real IMU Attitude (Pitch & Roll)")