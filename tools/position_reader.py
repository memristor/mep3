import math
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DisplayTransforms(Node):
    def __init__(self, parent_frame, child_frame):
        super().__init__('image_subscriber')

        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)
        while True:
            transform = None
            try:
                transform = self.__tf_buffer.lookup_transform(
                    child_frame, parent_frame, rclpy.time.Time())
            except Exception:  # noqa: E501
                # logging.exception("StaticTfListener"):
                pass

            if transform is not None:
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                q = transform.transform.rotation
                # Convert quaternion to yaw
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                print(x, y, math.degrees(yaw))


            rclpy.spin_once(self, timeout_sec=0.1)


# python3 position.py --ros-args -r /tf:=/big/tf -r /tf_static:=/big/tf_static
def main(args=None):
    rclpy.init(args=args)

    display_transforms = DisplayTransforms('base_link', 'map')
    rclpy.spin(display_transforms)
    display_transforms.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
