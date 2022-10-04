#!/usr/bin/env python3

import rospy
from actionlib import GoalStatusArray
from geometry_msgs.msg import PoseStamped

ready = False

def update_status(status):
    global ready
    if status:
        ready = True

def get_goal_pose(world_name):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = rospy.Time.now()
    if world_name == "new_maze":
        goal_pose.pose.position.x = 7.60832
        goal_pose.pose.position.y = -3.26584
        goal_pose.pose.orientation.z = 0.3007058
        goal_pose.pose.orientation.w = 0.953717
    elif world_name == "complex_maze":
        goal_pose.pose.position.x = 16.1586
        goal_pose.pose.position.y = -18.0215
        goal_pose.pose.orientation.w = 1
    elif world_name == "tricky_maze":
        goal_pose.pose.position.x = 9.0338
        goal_pose.pose.position.y = -3.42789
        goal_pose.pose.orientation.z = 0.6087614
        goal_pose.pose.orientation.w = 0.7933533
    elif world_name == "playground":
        goal_pose.pose.position.x = 10.47422
        goal_pose.pose.position.y = -0.418156
        goal_pose.pose.orientation.z = -0.0556293
        goal_pose.pose.orientation.w = 0.9984515
    elif world_name == "office":
        goal_pose.pose.position.x = -22.510065
        goal_pose.pose.position.y = 14.06405
        goal_pose.pose.orientation.z = -1
        goal_pose.pose.orientation.w = 0
    elif world_name == "old_maze":
        goal_pose.pose.position.x = 0.0366682
        goal_pose.pose.position.y = 5.3570690
        goal_pose.pose.orientation.z = -0.9636836
        goal_pose.pose.orientation.w = 0.2670467
    elif world_name == "warehouse":
        goal_pose.pose.position.x = -3.515613
        goal_pose.pose.position.y = 6.208450
        goal_pose.pose.orientation.z = 0.6959281
        goal_pose.pose.orientation.w = 0.7181115
    return goal_pose


if __name__=="__main__":
    rospy.init_node("nav_goal_publisher", anonymous=True)
    # getting navigation status. 
    # If this topic publishes anything, then we can publish the navigation goal.
    # This is because if it publishes anything, it means that the 
    # navigation is ready to recieve goals. If we publish before that,
    # then the navigation stack won't recieve the goal.
    rospy.Subscriber("/move_base/status", GoalStatusArray, update_status)

    # publisher for navigation goal
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=4)
    
    # getting the name of the world that is launched.
    # This changes the navigation goal that is published
    world_name = rospy.get_param("/world_name_py")

    # delay if navigation stack is not ready
    while not ready: rospy.sleep(0.3)


    goal_pose = PoseStamped()
    rospy.loginfo("Sending navigation goal...")
    rospy.loginfo(world_name)
    goal_pose = get_goal_pose(world_name)
    # used for debugging
    # rospy.loginfo(goal_pose)
    rospy.sleep(3)
    rospy.logwarn("PUBLISHING NAVIGATION GOAL")
    goal_pub.publish(goal_pose)
    # helps kill the program faster when the launch file is stopped
    while not rospy.is_shutdown(): continue
    rospy.spin()