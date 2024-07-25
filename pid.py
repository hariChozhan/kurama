import rospy
from geometry_msgs.msg import Twist
import math

def move_to_point(x, y):
    rospy.init_node('move_to_point', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()

    # Calculate distance and angle
    distance = math.sqrt(x**2 + y**2)
    angle = math.atan2(y, x)

    # Set velocities
    twist.linear.x = distance
    twist.angular.z = angle

    # Publish velocities
    pub.publish(twist)
    rate.sleep()

if __name__ == '__main__':
    try:
        # Example coordinates
        target_x = 1.0
        target_y = 1.0
        move_to_point(target_x, target_y)
    except rospy.ROSInterruptException:
        pass



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class MoveToPoint(Node):
    def __init__(self):
        super().__init__('move_to_point')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.linear_pid = PID(Kp=1.0, Ki=0.0, Kd=0.1)
        self.angular_pid = PID(Kp=1.0, Ki=0.0, Kd=0.1)

        self.target_x = 1.0
        self.target_y = 1.0

    def timer_callback(self):
        # Calculate distance and angle
        distance = math.sqrt(self.target_x**2 + self.target_y**2)
        angle = math.atan2(self.target_y, self.target_x)

        # Update PID controllers
        self.linear_pid.setpoint = distance
        self.angular_pid.setpoint = angle

        linear_velocity = self.linear_pid.compute(0)  # Assume current distance is 0 for simplicity
        angular_velocity = self.angular_pid.compute(0)  # Assume current angle is 0 for simplicity

        # Create Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        # Publish velocities
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    move_to_point = MoveToPoint()
    rclpy.spin(move_to_point)
    move_to_point.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
