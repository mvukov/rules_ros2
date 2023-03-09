# Copyright 2023 Thomas Griffith
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import pathlib

import pytest
import rosbag2_py

SQLITE3_STORAGE_ID = 'sqlite3'
MCAP_STORAGE_ID = 'mcap'
ODOMETRY_MESSAGE_TYPE = 'nav_msgs/msg/Odometry'
ODOMETRY_TOPIC = '/foo'
SERIALIZATION_FORMAT = 'cdr'


def make_bag(bag_filepath: pathlib.Path, storage_id: str):
    """Make a bag with a single message on a single topic.

    Args:
        bag_filepath: The path to the bag file to create.
        storage_id: The storage format to use.
    """
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_filepath),
            storage_id=storage_id,
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format=SERIALIZATION_FORMAT,
            output_serialization_format=SERIALIZATION_FORMAT),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(name=ODOMETRY_TOPIC,
                                 type=ODOMETRY_MESSAGE_TYPE,
                                 serialization_format=SERIALIZATION_FORMAT))


def test_write_to_sqlite(tmp_path):
    bag_filepath = tmp_path / 'test.db3'
    make_bag(bag_filepath=bag_filepath, storage_id=SQLITE3_STORAGE_ID)


def test_write_to_mcap(tmp_path):
    bag_filepath = tmp_path / 'test.mcap'
    make_bag(bag_filepath=bag_filepath, storage_id=MCAP_STORAGE_ID)


if __name__ == '__main__':
    # TODO(mvukov) Make a pytest runner for regular tests.
    raise SystemExit(pytest.main([__file__]))
