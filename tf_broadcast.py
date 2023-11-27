"""import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster
import tf2_ros

class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the camera to vehicle transformation
        self.camera_to_vehicle_transform = TransformStamped()
        self.camera_to_vehicle_transform.header.frame_id = 'camera_frame'
        self.camera_to_vehicle_transform.child_frame_id = 'vehicle_frame'
        self.camera_to_vehicle_transform.transform.translation.x = 2.0
        self.camera_to_vehicle_transform.transform.translation.y = 2.0
        self.camera_to_vehicle_transform.transform.translation.z = 2.0
        self.camera_to_vehicle_transform.transform.rotation.w = 1.0  # No rotation in this example

        # Publish the camera to vehicle transformation
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(self.camera_to_vehicle_transform)

    def transform_coordinates(self, input_point):
        # Wait for the transformation to be available
        self.tf_buffer.can_transform('vehicle_frame', 'camera_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))

        # Perform the transformation
        try:
            transformed_point = self.tf_buffer.transform(input_point, 'vehicle_frame')
            self.get_logger().info("Transformed Point: %s", transformed_point.point)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error("Transformation error: %s", str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()
    
    # Define the input point in the camera frame
    input_point = PointStamped()
    input_point.header.frame_id = 'camera_frame'
    input_point.point.x = 1.0
    input_point.point.y = 1.0
    input_point.point.z = 1.0

    # Transform the coordinates
    node.transform_coordinates(input_point)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster
import tf2_ros
from std_msgs.msg import Header  # Import Header to resolve the TypeException

class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the camera to vehicle transformation
        self.camera_to_vehicle_transform = TransformStamped()
        self.camera_to_vehicle_transform.header.frame_id = 'camera_frame'
        self.camera_to_vehicle_transform.child_frame_id = 'vehicle_frame'
        self.camera_to_vehicle_transform.transform.translation.x = 2.0
        self.camera_to_vehicle_transform.transform.translation.y = 2.0
        self.camera_to_vehicle_transform.transform.translation.z = 2.0
        self.camera_to_vehicle_transform.transform.rotation.w = 1.0  # No rotation in this example

        # Publish the camera to vehicle transformation
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(self.camera_to_vehicle_transform)

    def transform_coordinates(self, input_point):
        # Wait for the transformation to be available
        self.tf_buffer.can_transform('vehicle_frame', 'camera_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))

        # Perform the transformation
        try:
            transformed_point = self.tf_buffer.transform(input_point, 'vehicle_frame')
            self.get_logger().info("Transformed Point: %s", transformed_point.transform.translation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error("Transformation error: %s", str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()
    
    # Define the input point in the camera frame
    input_point = PointStamped()
    input_point.header = Header(frame_id='camera_frame')
    input_point.point.x = 1.0
    input_point.point.y = 1.0
    input_point.point.z = 1.0

    # Transform the coordinates
    node.transform_coordinates(input_point)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster
from std_msgs.msg import Header
import rosidl_runtime_py

class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the camera to vehicle transformation
        self.camera_to_vehicle_transform = TransformStamped()
        self.camera_to_vehicle_transform.header.frame_id = 'camera_frame'
        self.camera_to_vehicle_transform.child_frame_id = 'vehicle_frame'
        self.camera_to_vehicle_transform.transform.translation.x = 2.0
        self.camera_to_vehicle_transform.transform.translation.y = 2.0
        self.camera_to_vehicle_transform.transform.translation.z = 2.0
        self.camera_to_vehicle_transform.transform.rotation.w = 1.0  # No rotation in this example

        # Publish the camera to vehicle transformation
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(self.camera_to_vehicle_transform)

    def transform_coordinates(self, input_point):
        # Wait for the transformation to be available
        self.tf_buffer.can_transform('vehicle_frame', 'camera_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))

        # Perform the transformation
        try:
            transformed_point = self.tf_buffer.transform(input_point, 'vehicle_frame')
            self.get_logger().info("Transformed Point: %s", transformed_point.transform.translation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error("Transformation error: %s", str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()
    
    # Define the input point in the camera frame
    input_point = PointStamped()
    input_point.header = Header(frame_id='camera_frame')
    input_point.point.x = 1.0
    input_point.point.y = 1.0
    input_point.point.z = 1.0

    # Transform the coordinates
    node.transform_coordinates(input_point)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
