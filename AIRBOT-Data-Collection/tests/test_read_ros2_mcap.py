from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from mcap.reader import make_reader
from pprint import pprint


def read_messages(path: str):
    reader = make_reader(open(path, "rb"))

    all_topics = {channel.topic for channel in reader.get_summary().channels.values()}
    info_topics = {topic for topic in all_topics if "info" in topic}
    other_topics = all_topics - info_topics

    def iter_topics(topics: set):
        cur_topics = {}
        for schema, channel, message in reader.iter_messages(topics=topics):
            topic = channel.topic
            msg_type = get_message(schema.name)
            ros_msg = deserialize_message(message.data, msg_type)
            cur_topics[topic] = ros_msg
            if set(cur_topics.keys()) == topics:
                break
        pprint(cur_topics)

    iter_topics(info_topics)
    iter_topics(other_topics)


if __name__ == "__main__":
    import sys

    mcap_path = "/root/mcap_data_test/0.mcap"
    # mcap_path = sys.argv[1]

    read_messages(mcap_path)
