#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Pose, Quaternion
import tf
import math

def publish_endpoint():
    rospy.init_node('endpoint_publisher')

    # Publisher for the endpoint, publishing to 'set_endpoint' topic
    pub = rospy.Publisher('set_endpoint', Pose, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz

    # Define the fixed endpoint position and orientation
    endpoint = Pose()
    endpoint.position.x = 10
    endpoint.position.y = 10
    endpoint.position.z = 0.0  # Assuming 2D navigation

    # Orientation using Euler angles (yaw = π/4 radians here, converted to quaternion)
    yaw = math.pi  # set degree
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    endpoint.orientation = Quaternion(*quaternion)

    while not rospy.is_shutdown():
        # Publish the endpoint
        pub.publish(endpoint)
        #rospy.loginfo("Published endpoint: %s", endpoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_endpoint()
    except rospy.ROSInterruptException:
        pass
