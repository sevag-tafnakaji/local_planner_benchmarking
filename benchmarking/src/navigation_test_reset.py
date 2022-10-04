#!/usr/bin/env python3

from math import sqrt
from copy import copy
import rospy
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

if __name__ == "__main__":
    rospy.init_node("navigation_tester_publisher", anonymous=True)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    reset_pose = PoseStamped()
    reset_pose.header.frame_id = "map"
    reset_pose.header.stamp = rospy.Time.now()
    reset_pose.pose.position.x = 1.157362
    reset_pose.pose.position.y = 7.082126
    reset_pose.pose.orientation.z = -0.7071068
    reset_pose.pose.orientation.w = 0.7071068
    rospy.sleep(3)
    print(reset_pose)
    goal_pub.publish(reset_pose)