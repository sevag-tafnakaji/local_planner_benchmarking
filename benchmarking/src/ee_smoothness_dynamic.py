#!/usr/bin/env python3
import rospy
import path
import sys

folder = path.Path(__file__).abspath()
sys.path.append(folder.parent.parent)
import move_group_python
from mobile_manipulator.msg import JointStates
from actionlib import GoalStatusArray
from sensor_msgs.msg import JointState
from copy import deepcopy
from moveit_commander import MoveGroupCommander

test = JointState()
tests = JointStates()
tests.states.append(test)

def get_status(curr_status):
    global status
    for goal_status in curr_status.status_list:
        status = deepcopy(goal_status.status)

# def get_joint_states(joints):
#     global joint_states, indecies, states_publisher, status
#     # rospy.loginfo("HERE!!!!")
#     # rospy.loginfo(joints)
#     # Populate indecies list if it isn't already
#     wanted_joints = []
#     joint_names = []
#     ur10_joints = []
#     index = None
#     desired_state = JointState()
#     # print("Joint data gotten from subscriber:")
#     # print(joints)
#     if "UR10" in joints.name:
#         # Populate list with the indecies of the right joints
#         # Using the name of the joints and the prefix UR10
#         for aname in joints.name:
#             if "UR10" in aname:
#                 index = joints.name.index(aname)
#                 ur10_joints.append(joints.position[index])
#                 joint_names.append(aname)
#         # Get the time this measurement was taken at
#         curr_time = deepcopy(joints.header.stamp.to_sec())
#         joint_states.times.append(deepcopy(curr_time))
#         print("Joints:")
#         print(joint_names)
#         print("Anlges:")
#         print(ur10_joints)
#         # print(indecies)
#         # Take only the joint data of the arm (not MiR or BH)
#         # start, end = indecies[0], indecies[len(indecies)-1]
#         # print("All names:")
#         # print(joints.name)
#         # start, end = index, index+5
#         # ur10_joints = deepcopy(joints.position[start:end])
#         # joint_names = deepcopy(joints.name[start:end])
#         # print("Arm names:")
#         # print(joint_names)
#         if len(wanted_joints) != 0:
#             desired_state.header = deepcopy(joints.header)
#             desired_state.name = joint_names
#             desired_state.position = wanted_joints
#             desired_state.velocity, desired_state.effort = [], []
#             joint_states.states.append(desired_state)
#             # Get the time this measurement was taken at
#             curr_time = deepcopy(joints.header.stamp.to_sec())
#             joint_states.times.append(deepcopy(curr_time))
#             print("Collected Joint Data:")
#             print(joint_states.states)
#         states_publisher.publish(joint_states)

if __name__ == "__main__":
    rospy.init_node("ee_smoothness_measurements", anonymous=True)
    status = 0
    joint_states = JointStates()
    started = False
    indecies = []
    # Defining moveit planning group
    ur10_arm = move_group_python.MoveGroupPython(create_node=False)
    ur10_joints = ur10_arm.move_group.get_active_joints()
    joint_states.names=ur10_joints
    # Get needed data
    rospy.Subscriber("/move_base/status", GoalStatusArray, get_status)
    # rospy.Subscriber("/joint_states", JointState, get_joint_states, queue_size=5)
    # print("Status still "+str(0))
    while (status == 0):
        ur10_arm.move_group.set_named_target("Folded")
    # Publish the collected joint states
    states_publisher = rospy.Publisher("arm_joints_publisher", JointStates, queue_size=10)
    # print("status: "+str(status))

    start_time = rospy.get_time()
    wait_time = 3
    while not rospy.is_shutdown():
        diff_time = rospy.get_time() - start_time
        if not started and diff_time > wait_time:
            ur10_arm.move_group.go()
            started = True
        if status != 3:
            curr_time = rospy.get_time()
            joint_angles = ur10_arm.move_group.get_current_joint_values()
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.from_sec(curr_time)
            joint_state.name = ur10_joints
            joint_state.position = joint_angles 
            joint_states.times.append(curr_time)
            joint_states.states.append(joint_state)
            states_publisher.publish(joint_states)
        else:
            states_publisher.publish(joint_states)
    rospy.spin()