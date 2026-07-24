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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HEARTBEAT_TOPIC = '/system_heartbeat'
HEARTBEAT_PERIOD_SECONDS = 1.0


def format_heartbeat(count: int) -> str:
    """Return the stable heartbeat payload for a publication count."""
    return f'alive count={count}'


class SystemHeartbeat(Node):
    """Publish a simple liveness heartbeat at one hertz."""

    def __init__(self) -> None:
        super().__init__('system_heartbeat')
        self._count = 0
        self._publisher = self.create_publisher(String, HEARTBEAT_TOPIC, 10)
        self._timer = self.create_timer(
            HEARTBEAT_PERIOD_SECONDS,
            self._publish_heartbeat,
        )

    def _publish_heartbeat(self) -> None:
        self._count += 1
        message = String()
        message.data = format_heartbeat(self._count)
        self._publisher.publish(message)
        self.get_logger().info(message.data)


def main(args=None) -> None:
    """Run the system heartbeat node."""
    rclpy.init(args=args)
    node = SystemHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
