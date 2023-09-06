import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped

class FrameTransformer(Node):

    def __init__(self):
        super().__init__('frame_transformer')
        self.buffer = Buffer(self)
        self.tf_listener = TransformListener(self.buffer, self)

    def transform_point(self):
        world_point = PointStamped()
        world_point.header.frame_id = "world"  # Set the source frame
        world_point.point.x = 1.0  # Replace with your desired coordinates
        world_point.point.y = 2.0
        world_point.point.z = 3.0

        
        try:
            vehicle_point = self.buffer.transform(world_point, "vehicle")
            # "vehicle" should be the target frame
            self.get_logger().info(f"Transformed Point: {vehicle_point.point}")
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FrameTransformer()

    try:
        node.transform_point()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
