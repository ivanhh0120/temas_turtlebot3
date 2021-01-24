#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import sqrt, pow


class RobotControl(object):
    def __init__(self, tolerancia=0.1, linear_velocity=0.5, angular_velocity=0.5):
        rospy.init_node('ctrl_tarea01')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.twist = Twist()
        self.set_Twist(linear_velocity, angular_velocity)
        self.pose = Pose()
        self.tolerancia = tolerancia

    def set_Twist(self, linear_velocity, angular_velocity):
        self.twist.linear.x = linear_velocity # m/s
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = angular_velocity # rad/s

    def distancia_euclideana(self, x, y):
        return sqrt(pow((x - self.pose.position.x), 2) + 
                    pow((y - self.pose.position.y), 2))                 

    def update_pose(self, data):
        self.pose = data.pose.pose
        self.pose.position.x = round(self.pose.position.x, 4)
        self.pose.position.y = round(self.pose.position.y, 4)
        self.pose.position.z = round(self.pose.position.z, 4)

    def switch_twist(self):
        self.set_Twist(self.twist.linear.x, -self.twist.angular.z)    

    def check_goal(self, timeout=False):
        result = False
        distance_goal = self.distancia_euclideana(0, 0)
        if distance_goal >= self.tolerancia: 
            if not timeout:
                self.switch_twist()
                rospy.loginfo("Cambiando sentido")
            else:
                rospy.loginfo("omitido, estoy en timeout")
            result = True    

        rospy.loginfo("distancia a (0, 0): {} m".format(distance_goal))
        self.velocity_publisher.publish(self.twist)

        return result

def main():
    robot_ctrl = RobotControl(tolerancia=0.3)
    rate = rospy.Rate(10)
    result = True   
    while not rospy.is_shutdown():
        result = robot_ctrl.check_goal(result)
        rospy.loginfo("Result: {}".format(result))
        rate.sleep()

if __name__ == "__main__":
    main()


