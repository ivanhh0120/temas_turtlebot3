#!/usr/bin/env python


import rospy 
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan

def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w

    rospy.loginfo('x: {} y:{} o:({}, {}, {}, {})'.format(x,y,ox,oy,oz,ow))

def sensorcallback(msg):
    rmin = msg.range_min
    rmax = msg.range_max
    s= msg.intensities 
    rospy.loginfo('rmin:{} rmax:{} i:{}'.format(rmin,rmax,s[160]))

def main():
    rospy.init_node('loc_monitor')
    rospy.Subscriber('/odom',Odometry,callback)
    rospy.Subscriber('/scan',LaserScan, sensorcallback)
    rospy.spin()

if __name__ == '__main__':
    main()