#!/usr/bin/env python

import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from math import atan2
x=0.0 #metros
y=0.0 #metros 
theta=0.0 #Radianes


def controller_calback(msg):
    global x 
    global y 
    global theta


    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    oq = msg.pose.pose.orientation
    (roll, pitch, theta)=euler_from_quaternion([oq.x, oq.y, oq.z, oq.w])


def main():
    rospy.init_node('nav_controller')

    sub = rospy.Subscriber('/odom',Odometry, controller_calback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1,)
    goal1=Point()
    goal1.x=1.5
    goal1.y=2.5
    goal = Point()
    goal.x = 5
    goal.y = 5

    speed= Twist()
    r = rospy.Rate(4)
    current_goal=goal1
    while not rospy.is_shutdown():
        dx = current_goal.x - x
        dy = current_goal.y - y
        alpha = atan2(dx,dy)
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        rospy.loginfo('dx: {}  dy: {} alpha: {}'.format(dx,dy,alpha))
        if abs(alpha-theta)> 0.1:
            speed.linear.x=0.0
            speed.angular.z=0.3
        else:
            if( x < current_goal.x and y < current_goal.y):

                speed.linear.x = 0.5
                speed.angular.z = 0.0
            elif current_goal ==goal:
                current_goal =goal
            else:
                speed.linear.x=0.0
                speed.angular.z=0.0



        pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        r.sleep()



if __name__ == '__main__':
    main()