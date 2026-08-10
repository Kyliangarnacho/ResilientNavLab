"""Republish the first valid camera frame with advancing ROS timestamps."""

from math import isfinite

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from resilient_nav_health_assessment.camera_health_calibrate import (
    image_message_to_numpy,
)
from sensor_msgs.msg import Image


DEFAULT_SOURCE_TOPIC = '/camera/c920/image_raw'
DEFAULT_OUTPUT_TOPIC = '/test/camera/image_frozen'
DEFAULT_PUBLISH_RATE_HZ = 10.0


class FrozenImageCache:
    """Cache one valid Image payload and reproduce it with a new stamp."""

    def __init__(self):
        self._template = None
        self.last_error = None

    @property
    def ready(self):
        """Return whether a valid source frame has been cached."""
        return self._template is not None

    def try_cache(self, message, bridge):
        """Cache the first structurally valid and convertible Image."""
        if self.ready:
            return False
        try:
            self._validate_metadata(message)
            image_message_to_numpy(message, bridge)
        except (TypeError, ValueError) as error:
            self.last_error = str(error)
            return False

        self._template = {
            'frame_id': str(message.header.frame_id),
            'height': int(message.height),
            'width': int(message.width),
            'encoding': str(message.encoding),
            'is_bigendian': int(message.is_bigendian),
            'step': int(message.step),
            'data': bytes(message.data),
        }
        self.last_error = None
        return True

    @staticmethod
    def _validate_metadata(message):
        if message.width <= 0 or message.height <= 0:
            raise ValueError('source Image width and height must be positive')
        if not message.encoding:
            raise ValueError('source Image encoding must not be empty')
        if message.step <= 0:
            raise ValueError('source Image step must be positive')
        minimum_data_size = int(message.step) * int(message.height)
        if len(message.data) < minimum_data_size:
            raise ValueError('source Image data is shorter than step * height')

    def make_message(self, stamp):
        """Return a pixel-identical Image with the supplied current stamp."""
        if not self.ready:
            return None
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self._template['frame_id']
        message.height = self._template['height']
        message.width = self._template['width']
        message.encoding = self._template['encoding']
        message.is_bigendian = self._template['is_bigendian']
        message.step = self._template['step']
        message.data = self._template['data']
        return message


class CameraFreezeSource(Node):
    """Publish a frozen copy without modifying the original camera topic."""

    def __init__(self):
        super().__init__('camera_freeze_source')
        self.declare_parameter('source_topic', DEFAULT_SOURCE_TOPIC)
        self.declare_parameter('output_topic', DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('publish_rate_hz', DEFAULT_PUBLISH_RATE_HZ)
        source_topic = str(self.get_parameter('source_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not source_topic or not output_topic:
            raise ValueError('source_topic and output_topic must not be empty')
        if source_topic == output_topic:
            raise ValueError('output_topic must differ from source_topic')
        if not isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError(
                'publish_rate_hz must be finite and greater than zero'
            )

        self._cache = FrozenImageCache()
        self._bridge = CvBridge()
        self._publisher = self.create_publisher(
            Image, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            Image,
            source_topic,
            self._on_source_image,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_frozen_image
        )
        self.get_logger().info(
            f'Waiting for the first valid Image on {source_topic}; frozen '
            f'copies will publish to {output_topic} at '
            f'{publish_rate_hz:.3f} Hz'
        )

    def _on_source_image(self, message):
        if self._cache.ready:
            return
        if self._cache.try_cache(message, self._bridge):
            self.get_logger().info(
                'Cached the first valid source Image; frozen publishing active'
            )
        else:
            self.get_logger().warning(
                f'Source Image rejected: {self._cache.last_error}'
            )

    def _publish_frozen_image(self):
        message = self._cache.make_message(
            self.get_clock().now().to_msg()
        )
        if message is not None:
            self._publisher.publish(message)


def main(args=None):
    """Run the frozen camera test source."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraFreezeSource()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
