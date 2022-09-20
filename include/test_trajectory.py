#!/usr/bin/env python
#coding=utf-8
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

def tf_callback(data):
    global path_pub, odom_pub, path
    pose = PoseStamped()

    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = data.pose.position.x
    pose.pose.position.y = data.pose.position.y
    pose.pose.position.z = data.pose.position.z
    pose.pose.orientation.x = data.pose.orientation.x
    pose.pose.orientation.y = data.pose.orientation.y
    pose.pose.orientation.z = data.pose.orientation.z
    pose.pose.orientation.w = data.pose.orientation.w

    path.poses.append(pose);
    path_pub.publish(path);


    odom = Odometry()

    odom.header.frame_id = "map"
    odom.child_frame_id = "odom_imu";
    odom.header.stamp = rospy.Time.now()
    odom.pose.pose.position.x = data.pose.position.x
    odom.pose.pose.position.y = data.pose.position.y
    odom.pose.pose.position.z = data.pose.position.z
    odom.pose.pose.orientation.x = data.pose.orientation.x
    odom.pose.pose.orientation.y = data.pose.orientation.y
    odom.pose.pose.orientation.z = data.pose.orientation.z
    odom.pose.pose.orientation.w = data.pose.orientation.w

    odom_pub.publish(odom);


def velodyne_node():    
    global path_pub, odom_pub, path

    rospy.init_node('test')
    rospy.Subscriber('current_pose',PoseStamped,tf_callback)
    path_pub = rospy.Publisher('trajectory',Path, queue_size=10)
    odom_pub = rospy.Publisher('odom',Odometry, queue_size=10)
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

if __name__ == '__main__':
    velodyne_node()
    rospy.spin()
