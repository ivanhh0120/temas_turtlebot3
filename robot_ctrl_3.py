#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from math import sqrt, pow, pi
from math import atan2


class RobotControl(object):
    def __init__(self, tolerancia=0.1, linear_velocity=0.5, angular_velocity=0.1, x_goal= 2.0 , y_goal= 2.0):
        rospy.init_node('ctrl_tarea02')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.scan_subscriber = rospy.Subscriber('/scan',LaserScan, self.scan_callback_2)
        self.twist = Twist()
        self.set_Twist(linear_velocity, angular_velocity)
        self.pose = Pose()
        self.tolerancia = tolerancia
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.last_rotation = 0.0
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0
        self.d = 0.0
        self.ctrl_c = False
        #self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)


    def scan_callback_2(self, msg):
        self.a = msg.ranges[90]
        self.b = msg.ranges[int(len(msg.ranges)/2)]
        self.c = msg.ranges[180]
        self.d = msg.ranges[1]
    
    def read_laser(self):
        while not self.ctrl_c:
            if self.b > 5:
                self.b = 5
            if self.a > 5:
                self.a = 5
            if self.c > 5:
                self.c = 5
            
            print "a = "+ str(self.a)+" b = "+str(self.b)+" c = "+str(self.c)+" d = "+str(self.d)
            #self.rate.sleep()
    
    def shutdownhook(self):

        self.ctrl_c = True

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
        self.pose.orientation = self.pose.orientation


    def sensorcallback(self, msg):
        rmin = msg.range_min
        rmax = msg.range_max
        s= msg.intensities 
        rospy.loginfo('rmin:{} rmax:{} i:{}'.format(rmin,rmax,s[160]))
    
    

    def switch_twist(self):
        self.set_Twist(self.twist.linear.x, -self.twist.angular.z)   
    
    def stop_twist(self):
        self.set_Twist(0.0,0.0) 

    def check_goal (self, timeout=False):
        result = False
        distance_goal = self.distancia_euclideana(self.x_goal, self.y_goal)
        if distance_goal >= self.tolerancia: 
            if not timeout:
                self.stop_twist()
                
            else:
                rospy.loginfo("omitido, estoy en timeout")
            result = True    

        rospy.loginfo("distancia a (0, 0): {} m".format(distance_goal))
        self.velocity_publisher.publish(self.twist)

        return result
    
    def check_goal_2 (self, linear_velocity, angular_velocity,timeout=False):
        distance_goal = self.distancia_euclideana(self.x_goal, self.y_goal)
        if distance_goal >= self.tolerancia: 
            #self.update_pose(self)
            x_start = self.pose.position.x
            y_start = self.pose.position.y
            alpha = atan2(self.y_goal - y_start, self.x_goal- x_start)
            theta = self.theta()
            if alpha < -pi/4 or alpha > pi/4:
                if self.y_goal < 0 and y_start < self.y_goal:
                    alpha = -2*pi + alpha
                elif self.y_goal >= 0 and y_start > self.y_goal:
                    alpha = 2*pi + alpha
            #if self.last_rotation > pi-0.1 and theta <= 0:
            #    theta = 2*pi + theta
            #elif self.last_rotation < -pi+0.1 and theta > 0:
            #    theta = -2*pi + theta
            self.twist.angular.z = angular_velocity * (alpha-theta)

            distance_goal = self.distancia_euclideana(self.x_goal, self.y_goal)
            self.twist.linear.x = min(linear_velocity * distance_goal, 0.1)

            if self.twist.angular.z > 0:
                self.twist.angular.z = min(self.twist.angular.z, 1.5)
            else:
                self.twist.angular.z = max(self.twist.angular.z, -1.5)

            print " d = "+str(self.d)
            if self.d <= 0.5:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.velocity_publisher.publish(self.twist)
                rospy.loginfo('se esta cumpliendo')
            else:
                pass

            self.last_rotation = theta
            
            self.velocity_publisher.publish(self.twist)
    
    def theta(self):
        oq = self.pose.orientation
        (roll, pitch, theta)=euler_from_quaternion([oq.x, oq.y, oq.z, oq.w])
        return theta


def main():
    robot_ctrl = RobotControl(tolerancia=0.3)
    rate = rospy.Rate(10)
    result = True   
    while not rospy.is_shutdown():
        #result = robot_ctrl.check_goal(result)
        robot_ctrl.check_goal_2(linear_velocity=0.5, angular_velocity=1.0)
        rospy.loginfo("Result: {}".format(result))
        rate.sleep()

if __name__ == "__main__":
    main()