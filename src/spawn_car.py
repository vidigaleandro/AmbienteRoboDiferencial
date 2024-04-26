#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import subprocess

def get_model_xml(xacro_path):
    try:
        # Execute the xacro command to convert Xacro to URDF XML string directly
        command = ['xacro', xacro_path]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = process.communicate()

        if process.returncode == 0:
            rospy.loginfo("Xacro to URDF conversion successful.")
            return output.decode('utf-8')
        else:
            rospy.logerr("Failed to convert Xacro to URDF: %s", error.decode('utf-8'))
            return None
    except Exception as e:
        rospy.logerr("Error converting Xacro to URDF: %s", str(e))
        return None

def spawn_model(model_name, model_xml, pose):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        response = spawn_model_service(model_name, model_xml, "", pose, "world")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

if __name__ == '__main__':
    rospy.init_node('spawn_car_model')

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('diff_robot')
    xacro_path = package_path + '/world/robot.urdf.teste.xacro'

    model_xml = get_model_xml(xacro_path)
    if model_xml:
        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = 0.1

        if spawn_model('your_car_model', model_xml, initial_pose):
            rospy.loginfo("Model spawned successfully.")
        else:
            rospy.logerr("Failed to spawn model.")
