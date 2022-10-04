#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman


##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy

import gpg
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from geometry_msgs.msg import Pose,PoseStamped
import time
import tf2_geometry_msgs

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped or tf2_geometry_msgs.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPython(object):
    """
        Using python code to send joint/state plans for the robot to accomplish

        Can plan for multiple plan groups
    """

    def __init__(self, plan_group_name = "ur10_arm", create_node = True):
        """
            initializes the move_group planner for a specific part of DORA. These
            specific parts are called groups and they are predefined during the setup
            assistant. The current possible groups that you can plan for are:
            ur10_arm            bh282_gripper       mir100_mobile_base

            The arm (ur10_arm) is unique in the sense that it has an end effector. This
            means that we can use IK and a desired pose for the gripper to move the entire arm.

            @param: plan_group_name     the group that this object plans for.
        """
        super(MoveGroupPython, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        if create_node:
            rospy.init_node("move_group_python_"+plan_group_name)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()
        
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).
        ## This interface can be used to plan and execute motions:
        self.plan_group_name = plan_group_name
        self.move_group = moveit_commander.MoveGroupCommander(self.plan_group_name)

        self.eef_link = self.move_group.get_end_effector_link()
        self.planning_frame = self.move_group.get_planning_frame()
        if self.eef_link: print(f" ============ The end effector is: {self.eef_link} ============")
        print(f"============ The planning frame is: {self.planning_frame} ============")

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


    def go_to_default_pose(self):
        # creates a list of all zeros with the same length as amounts of joints in the move_group planner
        default_pose = [0 for x in self.move_group.get_current_joint_values()]
        # making the robot have the same angles as defined in default_pose
        self.move_group.go(default_pose, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

    def test_each_joint(self, neg = True):
        """
            tests each joint by moving it with 1/8th of a turn,
            waiting for a set amount of time and returning to 0.
            This is simply a function to see if the joint list is
            the same as the list of joints defined for the controller
            of the planning group.

            ex: ur10_arm list of controllers:
            
                UR10__base_to_shoulder_pan_joint
                UR10__shoulder_pan_to_upper_arm_joint
                UR10__upper_arm_to_fore_arm_joint
                UR10__fore_arm_to_wrist_1_joint
                UR10__wrist_1_to_wrist_2_joint
                UR10__wrist_2_to_wrist_3_joint
            
            matches their index from 0 to 5.

            @param neg      bool value to use negative angles for joint angles if true
        """
        # uses negative angles if neg is true, otherwise it is positive.
        # uses negative by default. This is useful for when positive angles are needed
        # for example controlling the gripper joints, pos angles are needed.
        ang = (- tau / 10) if neg is True else (tau / 10)
        # getting list of joints
        joint_goal = self.move_group.get_current_joint_values()
        # for loop for moving specific joints in joint list
        for i in range(len(joint_goal)):
            # moves each joint by ang, waits for 3 seconds then moves back to default "pose"
            print(f"moving joint nr.{i}")
            joint_goal[i] = ang
            self.move_group.go(joint_goal, wait=True)
            time.sleep(3)
            joint_goal[i] = 0
            self.move_group.go(joint_goal, wait=True)
        # stops all movement by controllers to make sure no weird movement happens.
        self.move_group.stop()

    def go_to_joint_state(self, joint_state = []):

        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        joint_goal = []
        given_joints = len(joint_state)>0
        if given_joints: 
            joint_goal = joint_state
        else:
            joint_goal = self.move_group.get_current_joint_values()
            # making 90 deg angle at elbow around MiR height with gripper flipped backwards
            joint_goal[0] = tau / 4
            joint_goal[1] = tau / 4
            joint_goal[2] = - tau / 4
            joint_goal[3] = tau / 2
            joint_goal[4] = - tau / 2
            joint_goal[5] = 0
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        print(joint_goal)
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal):

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        print(self.move_group.get_current_pose())
        # given pose is randomely chosen. Chosen to be within tolerance.
        print(pose_goal)
        # if you give a pose that is outside of tolerance the code will not work!

        self.move_group.set_pose_target(pose_goal)
        #self.move_group.set_goal_tolerance(0.01)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose()
        # return all_close(pose_goal, current_pose, 0.01)
        return plan
        
    def plan_cartesian_path(self,goalpose, scale=1):

        gp = Pose()
        gp.position.x = goalpose.pose.position.x
        gp.position.y = goalpose.pose.position.y
        gp.position.z = goalpose.pose.position.z

        gp.orientation.x = goalpose.pose.orientation.x
        gp.orientation.y = goalpose.pose.orientation.y
        gp.orientation.z = goalpose.pose.orientation.z
        gp.orientation.w = goalpose.pose.orientation.w
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []


        pose = self.move_group.get_current_pose().pose
        print(self.move_group.get_current_pose())
        #pose.position.x = 0.30195006454481393
        #pose.position.y = -0.10796936266786213
        #pose.position.z = 0.06598587271125143

        # pose.orientation.x = 0.47546512065967284
        # pose.orientation.y = 0.5809846403765418
        # pose.orientation.z = 0.4702847078679366
        # pose.orientation.w = 0.4639203166796994

        waypoints.append(copy.deepcopy(pose))
        #wpose = self.move_group.get_current_pose().pose
        #posestamped.position.z -= scale * 0.1  # First move up (z)
        #posestamped.position.y += scale * 0.2  # and sideways (y)
        #waypoints.append(copy.deepcopy(posestamped))

        #posestamped.position.x += scale * 0.1  # Second move forward/backwards in (x)
        #waypoints.append(copy.deepcopy(posestamped))

        #posestamped.position.y -= scale * 0.1  # Third move sideways (y)
        #waypoints.append(copy.deepcopy(posestamped))

        waypoints.append(copy.deepcopy(gp))

        print(f"waypoints: {waypoints}")

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):

        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):

        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print(f"Beginning control of robot using MoveIt!")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin initializing the moveit_commander ..."
        )
        arm = MoveGroupPython()
        gripper = MoveGroupPython("bh282_gripper")

        input(
           "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        arm.go_to_joint_state()
        ang = tau/5
        angles = [ang, ang, ang/2, ang, ang, ang/2, ang, ang/2]
        gripper.go_to_joint_state(angles)

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        arm.go_to_pose_goal()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = arm.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        arm.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        arm.execute_plan(cartesian_plan)

        print("============ robot Moveit motion demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

