#!/usr/bin/python3

"""
The purpose of this script is only to isolate the status from the 
move_base/status topic as that has more info than what we need.

The status is cast to an UInt8 type to publish to its own topic where:

1 - Driving
3 - Done

which is really what we mostly care about (as there are other values for status).

"""

import rospy
from actionlib import GoalStatusArray
from std_msgs.msg import UInt8

goal_ids = []
REACHED = UInt8(3)
NOT_REACHED = UInt8(0)
DRIVING = UInt8(1)

def get_status(goalStatusArr, status_pub):
    # extracting the status from the message type
    # goal_ids array was used in case multiple navigation
    # goals were sent in one run, which is not done in this case
    for goalStatus in goalStatusArr.status_list:
        if goalStatus.goal_id.id not in goal_ids: 
            if goalStatus.status == REACHED:

                status_pub.publish(REACHED)
                goal_ids.append(goalStatus.goal_id.id)
            elif goalStatus.status == DRIVING:
                rospy.loginfo("Robot now driving to given goal!")
                status_pub.publish(DRIVING)
        else:
            status_pub.publish(NOT_REACHED)

if __name__ == "__main__":
    rospy.init_node("navigation_status")
    status_pub = rospy.Publisher("/navigation_goal_status", UInt8, queue_size=5)
    rospy.Subscriber("move_base/status", GoalStatusArray, get_status, callback_args=status_pub)
    rospy.spin()