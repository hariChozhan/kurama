"""_summary_
    follows human at 0.2m distance
    can move in backward direction
    
    Returns:
        _type_: _description_
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult  # Helper module
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from nav2_msgs.msg import VoxelGrid
from scipy.spatial.transform import Rotation as rot
import math
import time


class stalkerbot(Node):

    def __init__(self):
        super().__init__('human_follower')
        self.current_pose_sub = self.create_subscription(
            Odometry,
            'wheel/odometry',
            self.current_robot_pose,
            10)
        self.current_pose_sub

        self.target_camera_pose_sub = self.create_subscription(
            PoseStamped,
            'pose_topic',
            self.target_camera_pose,
            10)
        self.target_camera_pose_sub

        self.target_rf_pose_sub = self.create_subscription(
            PoseStamped, '/rf_pose', self.target_rf_pose, 10)
        self.target_rf_pose_sub

        self.goal_poses = []
        self.goal_pose = PoseStamped()
        self.a = 0
        self.b = 0
        self.navigator = BasicNavigator()
        self.initial_pose = [0, 0, 0]  # initial pose of the robot
        self.current_pose = [0, 0, 0, 0, 0, 0, 0]  # current pose of the robot
        self.current_robot_orientation = [0, 0, 0]
        # previous pose of the robot
        self.previous_pose = [0, 0, 0, 0, 0, 0, 0]
        # current goal for the robot / current position of the human
        self.current_goal = [0, 0, 0]
        # previous goal for the robot / previous position of the human
        self.previous_goal = [0, 0, 0]

        self.human_goal_pose_pub = self.create_publisher(
            Marker, '/human_goal_pose_marker', 2)
        self.humanGoalPose = Marker()
        # self.bootup()

    # def bootup(self):
    #     print("*****************************IMU Calibration*****************************")
    #     self.goal_pose.header.frame_id = 'map'
    #     self.goal_pose.pose.position.x = 0.0
    #     self.goal_pose.pose.position.y = 0.0
    #     self.goal_pose.pose.position.z = 0.0
    #     self.goal_pose.pose.orientation.x = 0.0
    #     self.goal_pose.pose.orientation.y = 0.0
    #     self.goal_pose.pose.orientation.z = 0.0
    #     self.goal_pose.pose.orientation.w = 1.0
    #     self.navigator.goToPose(self.goal_pose)

    #     self.goal_pose.header.frame_id = 'map'
    #     self.goal_pose.pose.position.x = 0.0
    #     self.goal_pose.pose.position.y = 0.0
    #     self.goal_pose.pose.position.z = 0.0
    #     self.goal_pose.pose.orientation.x = 0.0
    #     self.goal_pose.pose.orientation.y = 0.0
    #     self.goal_pose.pose.orientation.z = 1.0
    #     self.goal_pose.pose.orientation.w = 1.0
    #     self.navigator.goToPose(self.goal_pose)

    #     self.goal_pose.header.frame_id = 'map'
    #     self.goal_pose.pose.position.x = 0.0
    #     self.goal_pose.pose.position.y = 0.0
    #     self.goal_pose.pose.position.z = 0.0
    #     self.goal_pose.pose.orientation.x = 0.0
    #     self.goal_pose.pose.orientation.y = 0.0
    #     self.goal_pose.pose.orientation.z = 1.5
    #     self.goal_pose.pose.orientation.w = 1.0
    #     self.navigator.goToPose(self.goal_pose)

    #     self.goal_pose.header.frame_id = 'map'
    #     self.goal_pose.pose.position.x = 0.0
    #     self.goal_pose.pose.position.y = 0.0
    #     self.goal_pose.pose.position.z = 0.0
    #     self.goal_pose.pose.orientation.x = 0.0
    #     self.goal_pose.pose.orientation.y = 0.0
    #     self.goal_pose.pose.orientation.z = 0.0
    #     self.goal_pose.pose.orientation.w = 1.0
    #     self.navigator.goToPose(self.goal_pose)
    #     print("*****************************IMU Calibration completed*****************************")

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z  # in radians

    def current_robot_pose(self, currentPose):
        # if currentPose.pose.pose.position.x-self.previous_pose[0] > 1:
        self.current_pose = [currentPose.pose.pose.position.x, currentPose.pose.pose.position.y, currentPose.pose.pose.position.z,
                             currentPose.pose.pose.orientation.x, currentPose.pose.pose.orientation.y, currentPose.pose.pose.orientation.z,
                             currentPose.pose.pose.orientation.w]
        # print("current robot pose: ", self.current_pose, self.previous_pose)
        self.current_robot_orientation = self.euler_from_quaternion(currentPose.pose.pose.orientation.x, currentPose.pose.pose.orientation.y,
                                                                    currentPose.pose.pose.orientation.z, currentPose.pose.pose.orientation.w)
        # print("yaw:", self.current_robot_orientation[2])
        # self.previous_pose = self.current_pose

    def target_camera_pose(self, targetCameraPose):
        print(targetCameraPose)

    def target_rf_pose(self, targetRfPose):
        self.current_goal = [targetRfPose.pose.position.x,
                             targetRfPose.pose.position.y, targetRfPose.pose.position.z]

        dist = math.sqrt(((self.current_goal[0] - self.current_pose[0])**2) +
                         ((self.current_goal[1] - self.current_pose[1])**2))
        print("distance between robot and current goal: ", dist)
        if dist > 2.0:
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.pose.position.x = self.current_goal[0]
            self.goal_pose.pose.position.y = self.current_goal[1]
            self.goal_pose.pose.position.z = self.current_goal[2]
            # self.goal_pose.pose.orientation.x = 0.0
            # self.goal_pose.pose.orientation.y = 0.0
            # self.goal_pose.pose.orientation.z = 0.0
            # self.goal_pose.pose.orientation.w = 1.0
            self.navigator.goToPose(self.goal_pose)

            # f = open('waypoints.txt', 'a')
            # f.writelines(("Current goal: ", str(self.current_goal[0]), " , ", str(
            # self.current_goal[1]), " , ", str(self.current_goal[2]), "\n", "\n"))

            # f.writelines(("Target pose: ", str(targetRfPose.pose.position.x), " , ", str(
            #     targetRfPose.pose.position.x), " , ", str(targetRfPose.pose.position.x), "\n", "\n"))

            for self.j in range(0, 10000):
                self.humanGoalPose.header.frame_id = 'map'
                # self.humanGoalPose.header.stamp = time.CLOCK_REALTIME
                self.humanGoalPose.ns = str(self.j)  # unique ID
                self.humanGoalPose.action = Marker().ADD
                self.humanGoalPose.type = Marker().ARROW
                # self.humanGoalPose.lifetime = self.marker_lifetime
                self.humanGoalPose.scale.x = 0.2
                self.humanGoalPose.scale.y = 0.2
                self.humanGoalPose.scale.z = 0.2
                self.humanGoalPose.pose.position.x = float(
                    self.current_goal[0])
                self.humanGoalPose.pose.position.y = float(
                    self.current_goal[1])
                self.humanGoalPose.pose.position.z = 0.0
                self.humanGoalPose.pose.orientation.x = 0.0
                self.humanGoalPose.pose.orientation.y = 0.0
                self.humanGoalPose.pose.orientation.z = 0.0
                self.humanGoalPose.pose.orientation.w = 1.0
                self.humanGoalPose.color.r = 1.0
                self.humanGoalPose.color.g = 0.0
                self.humanGoalPose.color.b = 0.0
                self.humanGoalPose.color.a = 1.0
                # print("\n", self.humanGoalPose)
                self.human_goal_pose_pub.publish(self.humanGoalPose)
                self.j = self.j+1

            i = 0
            while not self.navigator.isNavComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Distance remaining: ' + '{:.2f}'.format(
                        feedback.distance_remaining) + ' meters.')

                if feedback.distance_remaining < 2.5:
                    self.previous_pose = self.current_goal
                    self.navigator.cancelNav()
                    # if feedback.distance_remaining < 1.8:
                    # self.navigator.cancelNav()
                    # break
                    # break
            # result = self.navigator.getResult()
            # if result == NavigationResult.SUCCEEDED:
            #     print('Goal succeeded!')
            # elif result == NavigationResult.CANCELED:
            #     print('Goal was canceled!')
            # elif result == NavigationResult.FAILED:
            #     print('Goal failed!')
            # else:
            #     print('Goal has an invalid return status!')


def main():
    rclpy.init()
    human_follower = stalkerbot()
    navigator = BasicNavigator()
    # navigator.waitUntilNav2Active()
    try:
        rclpy.spin(human_follower)
    except KeyboardInterrupt:
        human_follower.destroy_node()
        rclpy.shutdown()
        navigator.lifecycleShutdown()
        # exit(0)


if __name__ == '__main__':
    main()
