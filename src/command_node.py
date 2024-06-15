#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel, DeleteModel
import tf
import math

class CommandClass:
    def __init__(self):
        rospy.init_node('command_node', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_sub = rospy.Subscriber('set_vel', Twist, self.vel_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.endpoint_sub = rospy.Subscriber('set_endpoint', Pose, self.endpoint_callback)

        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.rate = rospy.Rate(10)

        self.endpoint_reached_logged = False
        self.endpoint_marker_spawned = False  # Flag to track if the endpoint marker has been spawned

        self.models = []  # Initialize the list to track marker names
        self.model_counter = 0  # Counter to ensure unique names

        self.endpoint = Pose()
        self.endpoint.position.x = 10
        self.endpoint.position.y = 10
        self.endpoint.orientation.w = 1  # Assume no rotation by default

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        rospy.wait_for_service('gazebo/delete_model')
        self.delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    def endpoint_callback(self, msg):
        self.endpoint = msg
        if not self.endpoint_marker_spawned:
            self.spawn_endpoint_marker()


    def spawn_path_marker(self, pose):
        model_name = 'path_marker_{}'.format(self.model_counter)
        model_xml = f'''
        <sdf version="1.6">
            <model name="{model_name}">
                <static>true</static>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <sphere><radius>0.05</radius></sphere>
                        </geometry>
                        <material>
                            <ambient>1 0 0 1</ambient>  <!-- Red color -->
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        '''
        try:
            self.spawn_model(model_name, model_xml, "", pose, "world")
            self.models.append(model_name)
            self.model_counter += 1
            # Remove the oldest model if the limit is exceeded
            if len(self.models) > 50:
                oldest_model = self.models.pop(0)
                self.delete_model(oldest_model)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn Model failed: {0}".format(e))

    def spawn_endpoint_marker(self):
        if not self.endpoint_marker_spawned:  # Check if the marker has already been spawned
            endpoint_name = 'endpoint_marker'
            model_xml = f'''
            <sdf version="1.6">
                <model name="{endpoint_name}">
                    <static>true</static>
                    <link name="link">
                        <visual name="visual">
                            <geometry>
                                <cylinder>
                                    <radius>0.5</radius>
                                    <length>0.01</length>
                                </cylinder>
                            </geometry>
                            <material>
                                <ambient>0 1 0 1</ambient>  <!-- Green color -->
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            '''
            pose = Pose()
            pose.position.x = self.endpoint.position.x
            pose.position.y = self.endpoint.position.y
            pose.position.z = 0  # Ground level

            try:
                self.spawn_model(endpoint_name, model_xml, "", pose, "world")
                rospy.loginfo(f"Spawned model {endpoint_name} at position ({pose.position.x}, {pose.position.y}).")
                self.endpoint_marker_spawned = True  # Set the flag after successful spawning
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to spawn model {endpoint_name}: {str(e)}")

    def vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def publish_velocity(self):
        while not rospy.is_shutdown():
            vel_msg = Twist()
            vel_msg.linear.x = self.linear_x
            vel_msg.angular.z = self.angular_z
            self.pub.publish(vel_msg)
            self.rate.sleep()

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.orientation = yaw
        distance_to_endpoint = self.calculate_distance(self.position_x, self.position_y, self.endpoint.position.x, self.endpoint.position.y)
        if distance_to_endpoint < 0.5:
            if not self.endpoint_reached_logged:
                rospy.loginfo("Endpoint reached.")
                self.endpoint_reached_logged = True  # Set the flag after logging

        # Throttle the rate of spawning to prevent clutter
        if self.model_counter % 30 == 0:
            self.spawn_path_marker(msg.pose.pose)
        self.model_counter += 1

    def euler_from_quaternion(self, quat):
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        return tf.transformations.euler_from_quaternion(quaternion)

    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

if __name__ == '__main__':
    try:
        command = CommandClass()
        command.publish_velocity()
    except rospy.ROSInterruptException:
        pass
