import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid

class MinimalPublisher(Node):
  def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(OccupancyGrid, 'gridmap_topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
 
  def timer_callback(self):
    gridmap = OccupancyGrid()
    gridmap.header.frame_id = 'map'
    gridmap.info.resolution = 0.1  # Grid resolution in meters
    gridmap.info.width = 50  # Number of cells in the x direction
    gridmap.info.height = 50  # Number of cells in the y direction

    num_cells = gridmap.info.width * gridmap.info.height
    print(type(num_cells))
    # data = [1] * num_cells
    # print(type(data))
    data = 2500

    # # Populate the data with float values
    # for i in range(num_cells):
    #     float_value = float(i) / num_cells
    #     scaled_value = int(float_value * 254) - 128
    #     data[i] = scaled_value

    gridmap.data = data
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
