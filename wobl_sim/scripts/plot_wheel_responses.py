from mcap.reader import make_reader
import numpy as np
import matplotlib.pyplot as plt
from wobl_msgs.msg import Topics
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_messages(input_bag: str):
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
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def compare_joint_states(real_data, sim_data):
    # Example: align by timestamp, plot velocity or position
    # You may need to interpolate or align timestamps
    # This is a simplified example
    real_times = [t for t, _ in real_data]
    real_vels = [msg.velocity for _, msg in real_data]
    sim_times = [t for t, _ in sim_data]
    sim_vels = [msg.velocity for _, msg in sim_data]

    plt.plot(real_times, [v[2] for v in real_vels], label="Real Left Wheel")
    plt.plot(sim_times, [v[2] for v in sim_vels], label="Sim Left Wheel")
    plt.legend()
    plt.show()


def normalize_timestamps(data):
    if not data:
        return []
    t0 = data[0][0]
    return [((t - t0) / 1e9, msg) for t, msg in data]

if __name__ == "__main__":
    real_mcap = "rosbag2_test/rosbag2_test.mcap"
    sim_mcap = "rosbag2_test/rosbag2_test.mcap"
    topic = Topics.JOINT_STATE
    real_data = [
        (t, msg)
        for topic, msg, t in read_messages(real_mcap)
        if topic == Topics.JOINT_STATE
    ]
    real_data = normalize_timestamps(real_data)
    sim_data = [
        (t, msg)
        for topic, msg, t in read_messages(sim_mcap)
        if topic == Topics.JOINT_STATE
    ]
    sim_data = normalize_timestamps(sim_data)
    compare_joint_states(real_data, sim_data)
