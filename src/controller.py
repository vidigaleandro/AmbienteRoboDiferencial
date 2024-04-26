#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import tf
import math

class GoToGoalController:
    def __init__(self):
        rospy.init_node('go_to_goal_controller')

        # Constants for the control law
        self.k_rho = 0.2  # Gain for the distance
        self.k_alpha = 0.9  # Gain for the heading towards the goal
        self.k_beta = -0.5  # Gain for the orientation of the goal
        self.stopping_radius = 0.1  # Radius within which the robot should stop
        self.v_max = 2

        self.pub = rospy.Publisher('set_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.endpoint_sub = rospy.Subscriber('set_endpoint', Pose, self.endpoint_callback)

        self.current_pose = Pose()
        self.goal_pose = Pose()
        self.goal_reached = False  # Flag to track whether the goal has been reached
        self.rate = rospy.Rate(10)  # Hz

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def endpoint_callback(self, msg):
        self.goal_pose = msg
        self.goal_reached = False  # Reset the goal reached flag when a new goal is set
        self.control_loop()

    def control_loop(self):
        if self.goal_reached:
            return  # If goal is already reached, skip processing

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        rho = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        if rho < self.stopping_radius:
            if not self.goal_reached:  # Check if the goal reached log has been made
                self.pub.publish(Twist())  # Stop the robot
                rospy.loginfo("Reached goal")
                self.goal_reached = True  # Set the flag after logging
        else:
            theta = self.get_yaw_from_quaternion(self.current_pose.orientation)
            goal_theta = math.atan2(goal_y - current_y, goal_x - current_x)
            alpha = self.normalize_angle(goal_theta - theta)
            goal_orientation = self.get_yaw_from_quaternion(self.goal_pose.orientation)
            beta = self.normalize_angle(goal_orientation - goal_theta)

            linear_speed = min(self.k_rho * rho, self.v_max)

            if abs(alpha) > math.pi/2:
                linear_speed = -linear_speed
                alpha = self.normalize_angle(goal_theta - theta + math.pi)
                beta = self.normalize_angle(goal_orientation - goal_theta + math.pi)


            angular_speed = self.k_alpha * alpha + self.k_beta * beta

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.pub.publish(twist)

    def get_yaw_from_quaternion(self, quat):
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    try:
        controller = GoToGoalController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

