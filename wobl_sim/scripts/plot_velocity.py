import os
import plotly.express as px
from wobl_msgs.msg import Topics
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from filterpy.kalman import KalmanFilter
import numpy as np


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
    # Find the index of the first JOINT_COMMAND message
    first_cmd_idx = next(
        (i for i, (_, topic, _) in enumerate(msgs) if topic == Topics.JOINT_COMMAND)
    )
    msgs = msgs[first_cmd_idx:]
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

def kalman_filter(velocities, process_variance, measurement_variance):
    """Apply a Kalman filter to the velocity list."""
    filtered = []
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0]])  # initial state (position and velocity)
    kf.P = np.eye(1)  # initial uncertainty
    kf.F = np.array([[1]])  # state transition matrix
    kf.H = np.array([[1]])  # measurement matrix
    kf.R = np.array([[measurement_variance]])  # measurement noise
    kf.Q = np.array([[process_variance]])  # process noise

    for z in velocities:
        kf.predict()
        kf.update(np.array([[z]]))
        filtered.append(kf.x[0, 0])
        print(filtered[-1], z)
    return filtered

def plot_velocity_comparison(msgs, title, alpha=0.8):
    msgs = [msg for msg in msgs if msg[1] == Topics.JOINT_STATE]
    names, topics, messages = zip(*msgs)
    stamps = [msg.header.stamp for msg in messages]
    velocities = [sum(msg.velocity[2:]) / 2.0 for msg in messages]
    #filtered_velocities = linear_filter(velocities, alpha)
    filtered_velocities = kalman_filter(velocities, process_variance=0.001, measurement_variance=0.02)
    #print(filtered_velocities)

    import pandas as pd

    df = pd.DataFrame(
        {
            "Time [s]": stamps,
            "Unfiltered Velocity [m/s]": velocities,
            "Filtered Velocity [m/s]": filtered_velocities,
            "Source": names,
            "Topic": topics,
        }
    )

    print(df)

    dfm = df.melt(
        id_vars=["Time [s]", "Source", "Topic"],
        value_vars=["Unfiltered Velocity [m/s]", "Filtered Velocity [m/s]"],
        var_name="Type",
        value_name="Velocity [m/s]",
    )

    fig = px.line(
        dfm,
        x="Time [s]",
        y="Velocity [m/s]",
        color="Type",
        line_dash="Source",
        title=title,
        markers=True,
    )

    fig.show(renderer="browser", using="print")


if __name__ == "__main__":
    real_mcap = find_first_mcap_file("data/bag_real_wheel_acc_0")
    sim_mcap = find_first_mcap_file("data/bag_sim_wheel")

    msgs = collect_msgs("real", real_mcap)
    msgs += collect_msgs("sim", sim_mcap)

    plot_velocity_comparison(msgs, "Linear velocity: Unfiltered vs. Filtered (alpha=0.8)")
    # plot_wheel_comparison(msgs, 3, "Right Wheel: JointState & JointCommand Comparison")
