import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap, GridMapInfo
from builtin_interfaces.msg import Time
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

class GridMapPublisher(Node):
    def __init__(self):
        super().__init__('gridmap_publisher')
        self.publisher_ = self.create_publisher(GridMap, 'gridmap_topic', 10)

    def publish_gridmap(self):
        gridmap = GridMap()
        gridmap.header.stamp = self.get_clock().now().to_msg()
        gridmap.header.frame_id = 'map'
        gridmap.info.resolution = 0.1  # Grid resolution in meters
        gridmap.info.length_x = 100.0  # Number of cells in the x direction
        gridmap.info.length_y = 100.0  # Number of cells in the y direction
        num_cells = 100*100

        layer_data = GridMap()
        
        # Populate the layer data with float values
        data = Float32MultiArray()

        # Set the layout
        layout = MultiArrayLayout()
        layout.dim = [
            MultiArrayDimension(label='column', size=100, stride=100),
            MultiArrayDimension(label='row', size=100, stride=100)
        ]
        data.layout = layout

        # Populate the layer data with float values
        for i in range(num_cells):
            float_value = float(i) / num_cells
            data.data.append(float_value)

        layer_data.data.append(data)

        # Add the layer data to the gridmap
        gridmap.data.append(layer_data)

        self.publisher_.publish(gridmap)
        self.get_logger().info('Published gridmap')


def main(args=None):
    rclpy.init(args=args)
    publisher = GridMapPublisher()
    publisher.publish_gridmap()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
