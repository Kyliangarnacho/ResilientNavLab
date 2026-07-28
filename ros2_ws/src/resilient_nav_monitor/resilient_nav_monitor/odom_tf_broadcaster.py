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

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


ODOM_TOPIC = '/odom'
ODOM_FRAME = 'odom'
BASE_FOOTPRINT_FRAME = 'base_footprint'


def odometry_to_transform(message: Odometry) -> TransformStamped:
    """Convert an odometry pose to the project's odom transform."""
    transform = TransformStamped()
    transform.header.stamp = message.header.stamp
    transform.header.frame_id = ODOM_FRAME
    transform.child_frame_id = BASE_FOOTPRINT_FRAME

    position = message.pose.pose.position
    transform.transform.translation.x = position.x
    transform.transform.translation.y = position.y
    transform.transform.translation.z = position.z

    orientation = message.pose.pose.orientation
    transform.transform.rotation.x = orientation.x
    transform.transform.rotation.y = orientation.y
    transform.transform.rotation.z = orientation.z
    transform.transform.rotation.w = orientation.w
    return transform


class OdomTfBroadcaster(Node):
    """Publish the odometry pose as an odom-to-base-footprint TF."""

    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')
        self._tf_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odom_callback,
            10,
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._tf_broadcaster.sendTransform(
            odometry_to_transform(message)
        )


def main(args=None) -> None:
    """Run the odometry TF broadcaster."""
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
