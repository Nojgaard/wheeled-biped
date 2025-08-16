import os
import plotly.express as px
from wobl_msgs.msg import Topics
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


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


def plot_wheel_comparison(msgs, wheed_idx, title):
    names, topics, messages = zip(*msgs)
    stamps = [msg.header.stamp for msg in messages]
    velocities = [msg.velocity[wheed_idx] for msg in messages]
    fig = px.line(
        x=stamps,
        y=velocities,
        labels={"x": "Time [s]", "y": "Velocity [rad/s]"},
        title=title,
        markers=True,
        color=names,
        line_dash=topics,
    )
    fig.show(renderer="browser", using="print")


if __name__ == "__main__":
    real_mcap = find_first_mcap_file("data/bag_real_wheel_acc_0")
    sim_mcap = find_first_mcap_file("data/bag_sim_wheel")

    msgs = collect_msgs("real", real_mcap)
    msgs += collect_msgs("sim", sim_mcap)

    plot_wheel_comparison(msgs, 2, "Left Wheel: JointState & JointCommand Comparison")
    # plot_wheel_comparison(msgs, 3, "Right Wheel: JointState & JointCommand Comparison")
