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

        # Initialize target coordinates
        self.target_x = 1.0
        self.target_y = 1.0

        # Initialize current position (this should be updated with real sensor data in a real application)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_angle = 0.0

    def timer_callback(self):
        # Calculate distance and angle to target
        distance = math.sqrt((self.target_x - self.current_x)
                             ** 2 + (self.target_y - self.current_y) ** 2)
        angle_to_target = math.atan2(
            self.target_y - self.current_y, self.target_x - self.current_x)
        angle_error = angle_to_target - self.current_angle

        # Update PID setpoints
        self.linear_pid.setpoint = distance
        self.angular_pid.setpoint = angle_error

        # Compute PID outputs
        # In a real application, compute based on current distance
        linear_velocity = self.linear_pid.compute(0)
        # In a real application, compute based on current angle
        angular_velocity = self.angular_pid.compute(0)

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    move_to_point = MoveToPoint()
    rclpy.spin(move_to_point)
    move_to_point.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
