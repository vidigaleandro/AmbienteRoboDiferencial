import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

class TangentBug:
    def __init__(self):
        rospy.init_node('go_to_goal_controller')

      
        self.pub = rospy.Publisher('set_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.endpoint_sub = rospy.Subscriber('set_endpoint', Pose, self.endpoint_callback)
        self.linear_speed = 1
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.current_pose = Pose()
        self.goal_pose = Pose()
        self.goal_reached = False  # Flag to track whether the goal has been reached
        self.rate = rospy.Rate(10)  # Hz

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges  # Armazena os dados de distância dos scans

        # Novo array para armazenar posições e distâncias onde a diferença supera o percentual
        self.significant_changes = []

        # Definindo o percentual de interesse
        threshold_percent = 0.2  # 20%

        for range_value in msg.ranges:
            if range_value == float('inf'):  # Se a distância for inf, substituir pelo range_max
                self.scan_ranges.append(msg.range_max)
            else:
                self.scan_ranges.append(range_value)

        # Iterar pelo array de distâncias, exceto o último elemento para evitar índice fora de alcance
        for i in range(len(self.scan_ranges) - 1):
            current_range = self.scan_ranges[i]
            next_range = self.scan_ranges[i + 1]

            # Calcula a diferença absoluta
            difference = abs(next_range - current_range)

            # Checa se a diferença é maior que o percentual da distância atual
            if difference > threshold_percent * min(current_range,next_range):
                if min(current_range,next_range) == current_range:
                    self.significant_changes.append((i, current_range))
                if min(current_range,next_range) == next_range:
                    self.significant_changes.append((i, next_range))
            
            

            

        # Opcional: imprimir ou logar as mudanças significativas
        print("Significant changes recorded at positions and ranges:")
        for change in self.significant_changes:
            print(f"Position {change[0]}, Range {change[1]}")

    def endpoint_callback(self, msg):
        self.goal_pose = msg
        self.goal_reached = False  # Reset the goal reached flag when a new goal is set
        self.control_loop()

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
    
    def free_walk(self, alpha):
        # Inicializando velocidades
        angular_speed = 0
        linear_speed = 1

        # Ajustando a velocidade angular com base em alpha
        if alpha > 0.1:
            linear_speed = 0
            angular_speed = -1
        elif alpha < -0.1:
            linear_speed = 0
            angular_speed = 1

        # Retorna as velocidades calculadas
        return linear_speed, angular_speed


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
                     

            linear_speed, angular_speed = self.free_walk(alpha)



            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.pub.publish(twist)


if __name__ == '__main__':
    try:
        controller = TangentBug()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass