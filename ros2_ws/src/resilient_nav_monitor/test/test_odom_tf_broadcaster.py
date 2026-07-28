# Copyright 2026 Kylian
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

from pathlib import Path
from types import SimpleNamespace

from nav_msgs.msg import Odometry

from resilient_nav_monitor.odom_tf_broadcaster import BASE_FOOTPRINT_FRAME
from resilient_nav_monitor.odom_tf_broadcaster import ODOM_FRAME
from resilient_nav_monitor.odom_tf_broadcaster import ODOM_TOPIC
from resilient_nav_monitor.odom_tf_broadcaster import odometry_to_transform
from resilient_nav_monitor.odom_tf_broadcaster import OdomTfBroadcaster


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def make_odometry() -> Odometry:
    """Return an odometry message with a distinctive pose and stamp."""
    message = Odometry()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.5
    message.pose.pose.position.z = 0.125
    message.pose.pose.orientation.x = 0.1
    message.pose.pose.orientation.y = 0.2
    message.pose.pose.orientation.z = 0.3
    message.pose.pose.orientation.w = 0.9
    return message


def test_odom_tf_contract_uses_standard_names():
    """The node should subscribe and publish the requested frame names."""
    assert ODOM_TOPIC == '/odom'
    assert ODOM_FRAME == 'odom'
    assert BASE_FOOTPRINT_FRAME == 'base_footprint'


def test_odometry_to_transform_copies_stamp_and_pose():
    """The transform should preserve the message stamp and complete pose."""
    message = make_odometry()

    transform = odometry_to_transform(message)

    assert transform.header.stamp == message.header.stamp
    assert transform.header.frame_id == ODOM_FRAME
    assert transform.child_frame_id == BASE_FOOTPRINT_FRAME
    assert transform.transform.translation.x == 1.25
    assert transform.transform.translation.y == -0.5
    assert transform.transform.translation.z == 0.125
    assert transform.transform.rotation.x == 0.1
    assert transform.transform.rotation.y == 0.2
    assert transform.transform.rotation.z == 0.3
    assert transform.transform.rotation.w == 0.9


def test_callback_sends_converted_transform():
    """Each odometry callback should send one transform."""

    class RecordingBroadcaster:
        def __init__(self):
            self.transforms = []

        def sendTransform(self, transform):
            self.transforms.append(transform)

    broadcaster = RecordingBroadcaster()
    receiver = SimpleNamespace(_tf_broadcaster=broadcaster)

    OdomTfBroadcaster._odom_callback(receiver, make_odometry())

    assert len(broadcaster.transforms) == 1
    assert broadcaster.transforms[0].header.stamp.sec == 12
    assert broadcaster.transforms[0].child_frame_id == 'base_footprint'


def test_setup_registers_odom_tf_broadcaster_entry_point():
    """The installed package should expose the new executable."""
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert "'odom_tf_broadcaster = '" in setup_source
    assert (
        "'resilient_nav_monitor.odom_tf_broadcaster:main'"
        in setup_source
    )
