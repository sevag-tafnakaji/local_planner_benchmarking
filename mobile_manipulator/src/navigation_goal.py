#!/usr/bin/env python3
from tf2_geometry_msgs import PoseStamped, PointStamped
from copy import deepcopy
import rospy
import math
import tf2_ros
from grasp_pose_subscriber_sim import get_quaternion_from_euler

def get_magn(pose):
  x = pose.position.x
  y = pose.position.y
  z = pose.position.z
  return (x**2 + y**2 + z**2)**0.5

def get_point_magn(point):
  x = point.point.x
  y = point.point.y
  z = point.point.z
  return (x**2 + y**2 + z**2)**0.5

def find_ang(vec1, vec2):
    diff = deepcopy(vec1)
    can_pose = deepcopy(vec2)
    magnitude_diff = get_magn(diff.pose)
    diff.pose.position.x = diff.pose.position.x / magnitude_diff
    diff.pose.position.y = diff.pose.position.y / magnitude_diff
    diff.pose.position.z = diff.pose.position.z / magnitude_diff
    magnitude_can = get_point_magn(can_pose)
    can_pose.point.x = can_pose.point.x / magnitude_can
    can_pose.point.y = can_pose.point.y / magnitude_can
    can_pose.point.z = can_pose.point.z / magnitude_can
    x1 = diff.pose.position.x
    y1 = diff.pose.position.y
    z1 = diff.pose.position.z

    x2 = can_pose.point.x
    y2 = can_pose.point.y
    z2 = can_pose.point.z
    
    dot = x1*x2 + y1*y2 + z1*z2
    yaw = math.acos(dot)
    return yaw

def give_offset(can_point, robot_pose_transform, offset=1.3):
    """
        This function will return a pose that is the closest point some offset away
        function defined using https://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point

        Basically making a circle with the radius the same as the offset, centered at the can's position.
        Then finding the point on this circle that is closest to the robot's own pose

        Parameters

        ------------------

        pose : The pose of the object we want to calculate the offset from i.e. the can's x,y,z as a PointStamped

        robot_pose : The transform to get the pose of the robot from the map frame

        offset : desired offset from pose, in m
    """
    goal_pose = deepcopy(can_point)
    # Defining the center point of the circle
    cx = can_point.point.x
    cy = can_point.point.y
    # Defining the vector from the robot to the centerpoint
    vx = robot_pose_transform.transform.translation.x - cx
    vy = robot_pose_transform.transform.translation.y - cy
    # turning to unit vector
    vmag = (vx**2 + vy**2)**0.5
    # going in the direction of the vector v by the offset amount
    # hence we get a point that is the offset away from the can
    goal_pose.point.x = cx + vx / vmag * offset
    goal_pose.point.y = cy + vy / vmag * offset

    return goal_pose

def go_to_can(can_point, args):
    """
        This function uses the can centerpoint and after some prep
        sets a navigation goal. This prep is getting an offset from the can 
        that is closest to the robot's current position so that the robot
        does not drive into the can/table.

        Parameters:

        ----------------

        can_point : Can centerpoint defined in point_cloud_subscriber.py, type: PointStamped

        args : triplet defined in the subscriber that includes the navigation goal publisher,
        The transform buffer to find the robot position, and the transformed can centerpoint
        publisher.
    """
    # Getting the needed arguments
    goal_pub = args[0]
    tfBuffer = args[1]
    can_tf_pub = args[2]
    # Initializing goal pose
    goal_pose = PoseStamped()
    # transforms the can point from frame "camera_link" to "map" using the static transform in the launch file 
    # can's centerpoint calculated in point_cloud_subscriber.py
    can_point_tf = tfBuffer.transform(can_point, "map",rospy.Duration(1), PointStamped)
    # Publishing this centerpoint to be used in other places (such as visualizing in Rviz)
    can_tf_pub.publish(can_point_tf)
    # This transform is used to get the current position of the robot as 
    # odom_topic is not giving correct positions.
    pose_of_robot = tfBuffer.lookup_transform("map", "MiR_footprint", rospy.Time.now(), rospy.Duration(1))
    diff = PoseStamped()
    diff.header.frame_id = "map"
    diff.header.stamp = rospy.Time.now()
    diff.pose.position.x = can_point_tf.point.x - pose_of_robot.transform.translation.x
    diff.pose.position.y = can_point_tf.point.y - pose_of_robot.transform.translation.y
    diff.pose.position.z = can_point_tf.point.z - pose_of_robot.transform.translation.z
    yaw = find_ang(diff, can_point_tf)
    goal_position = give_offset(can_point_tf, pose_of_robot, 1)
    # Since the centerpoint is a point and we need a pose,
    # we have to define a new stamped pose variable to publish
    goal_pose.header.frame_id = can_point_tf.header.frame_id
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = goal_position.point.x
    goal_pose.pose.position.y = goal_position.point.y
    goal_pose.pose.position.z = goal_position.point.z

    quat = get_quaternion_from_euler(0, 0, -(math.pi/2 + yaw))
    goal_pose.pose.orientation.w = quat[3]
    goal_pose.pose.orientation.x = quat[0]
    goal_pose.pose.orientation.y = quat[1]
    goal_pose.pose.orientation.z = quat[2]
    # After publishing, a delay of 15s is used to allow time 
    # for planning and execution of the plan
    goal_pub.publish(goal_pose)
    # rospy.sleep(15)
    rospy.signal_shutdown("navigation goal set. shutting down goal subscriber ...")

if __name__ == "__main__":
    rospy.init_node("navigation_publisher")
    # Defining the needed variables for the transform procedure to be done later
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # Delay of 2s to allow time for everything to initialize
    rospy.sleep(2)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=4)
    can_tf_pub = rospy.Publisher("/transformed_can_centerpoint", PointStamped, queue_size=4)
    rate = rospy.Rate(5)
    # waits untill the publisher has some connections
    while goal_pub.get_num_connections() == 0:
        rate.sleep()
    rospy.Subscriber("can_centerpoint",PointStamped, callback=go_to_can, callback_args=(goal_pub, tfBuffer, can_tf_pub))
    rospy.spin()
