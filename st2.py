import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped  # Pose with ref frame and timestamp
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(GridMap, 'gridmap_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        gridmap = GridMap()
        gridmap.header.frame_id = 'map'
        gridmap.info.resolution = 1.0  # Grid resolution in meters
        gridmap.info.length_x = 10.0  # Number of cells in the x direction
        gridmap.info.length_y = 10.0  # Number of cells in the y direction
        gridmap.layers = ["test"]

        num_cells = 10*10
        # print(type(num_cells))
        msg_data = Float32MultiArray()
        msg_data.data = [1.0] * num_cells
        print(type(msg_data))

        # msg_lay = MultiArrayLayout
        # msg_lay.dim.append(MultiArrayDimension(label='length', size=5, stride=10))
        # msg_data.layout = [msg_lay]

        layout_msg = MultiArrayLayout()
    # Modify the layout_msg as required, setting the necessary fields
    # For example:
        layout_msg.dim = [
            MultiArrayDimension(label='length', size=10, stride=5),
            MultiArrayDimension(label='width', size=10, stride=5),
            MultiArrayDimension(label='height', size=2, stride=100)
        ]
        # layout_msg.is_row_major = True
        msg_data.layout = layout_msg

        gridmap.data = [msg_data]
        self.publisher_.publish(gridmap)
        self.get_logger().info('Publishing: "%s"' % gridmap)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
