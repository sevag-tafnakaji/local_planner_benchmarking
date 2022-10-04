#!/usr/bin/python3

import rospy
from actionlib import GoalStatusArray
from mobile_manipulator.msg import travelInfo
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from geometry_msgs.msg import Twist
from copy import deepcopy

# unnecessary now, but keeping for just-in-case uses.
base_mass = 4.0
shoulder_mass = 7.778
upper_arm_mass = 12.93
forearm_mass = 3.87
wrist_1_mass = 1.96
wrist_2_mass = 1.96
wrist_3_mass = 0.202
bh282_mass = 0.36699

tot_mass = base_mass + shoulder_mass + upper_arm_mass + forearm_mass + wrist_1_mass + wrist_2_mass + wrist_3_mass + bh282_mass

old_vel = Twist()
data = travelInfo()
status = 0

def update_status(curr_status):
    global status
    for goal_status in curr_status.status_list:
        status = deepcopy(goal_status.status)


if __name__ == "__main__":
    rospy.init_node("ee_stability_metric")
    # using gazebo service to find how the gripper is affected
    # by getting its pose and twist (calculations can be done to find  
    # how wobbly the path is
    linkState = GetLinkStateRequest()
    linkState.link_name = "UR10_wrist_3_link"
    linkState.reference_frame = ""
    rospy.wait_for_service("/gazebo/get_link_state")
    get_link_state_srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    data_rate = rospy.Rate(2)
    # Helps keep track of when the navigation starts and ends
    rospy.Subscriber("/move_base/status", GoalStatusArray, update_status)

    # publishes the poses and twists
    info_publisher = rospy.Publisher("gripper_info_publisher", travelInfo, queue_size=5)
    while not rospy.is_shutdown():
        if status == 3:
            # once navigation is done, start publishing the data
            info_publisher.publish(data)
        elif status == 1:
            # calling the service
            data_rate.sleep()
            new_info = get_link_state_srv(linkState)
            # if the call was successful
            if new_info.success:
                #adding the data to the message
                current_time = rospy.get_time()
                data.times.append(current_time)
                data.poses.append(new_info.link_state.pose)
                data.vels.append(new_info.link_state.twist)
                info_publisher.publish(data)
    rospy.spin()
