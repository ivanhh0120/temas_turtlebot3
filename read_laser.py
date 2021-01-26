#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Robot():
    def __init__(self):
        self.robot_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.robot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0
        self.d = 0.0
        self.ctrl_c = False

        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def scan_callback(self, msg):
        self.a = msg.ranges[45]
        self.b = msg.ranges[-45]
        self.c = msg.ranges[3]
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
            self.rate.sleep()
    
    def shutdownhook(self):

        self.ctrl_c = True

    def main(self):
        self.out_of_maze = False
        self.vel = Twist()
        while not self.out_of_maze:
            print self.d
            if self.d > 1.0:
                self.vel.linear.x = 0.25
            else:
                self.vel.linear.x = 0.0
            
            self.robot_pub.publish(self.vel)
            self.rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('rosbot_test', anonymous= True)
    rosbot_object = Robot()

    try:
        rosbot_object.read_laser()

    except rospy.ROSInterruptException:
        pass