#! /usr/bin/python3
'''
This function is a monitor for path planning. It will display the following measures:
- Continuously: Rospy time
- At the end: Current position
- At the end: Length of the paths (planned one and taken one)
- At the end: Area between the paths
- At the end: Travelled distance
'''

from copy import deepcopy
from math import *
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Float32
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import rospy
from actionlib import GoalStatusArray

from compare_paths import SEUIL, compare_paths


# Init the node and publisher
rospy.init_node('path_metrics')
path_pub = rospy.Publisher('/path', Path, queue_size=10)
dist_travelled_pub = rospy.Publisher("/distance_metric", Float32, queue_size=4)
global_area_pub = rospy.Publisher("/area_between_paths", Float32, queue_size=4)
rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
path = Path()
header = Header()
header.frame_id='odom'
model = GetModelStateRequest()
model.model_name='Mobile_Manipulator'
r = rospy.Rate(20)


# Global variables
global_planner_plan = []
counter = 0
local_counter = 0
old_x=0
old_y=0
started=False
status=0
data = {}


# Callback function
def count(msg):
    global counter
    global global_planner_plan
    counter += 1
    for element in msg.poses:
        global_planner_plan.append([element.pose.position.x, element.pose.position.y])

def check_status(curr_status):
    global status
    # status: 1 is driving, 3 is done
    for goal_status in curr_status.status_list:
        status = deepcopy(goal_status.status)

#Depending on wich local and global planner you are using, rospy will subscribe to the good one and let the other.
rospy.Subscriber("/move_base_node/GlobalPlanner/plan", Path, count)
rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, count)

# used to keep track of the navigation status (i.e. when it is driving or done)
rospy.Subscriber("/move_base/status", GoalStatusArray, check_status)




rospy.loginfo("Waiting for navigation to start:")

while status != 1: continue

rospy.loginfo("Monitoring")
starting_time = rospy.get_time()

# This loop collects the data on the real path that the local planner followed
while status == 1:
    result = get_model_srv(model)
    rot_q = result.pose.orientation
    (roll,pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    x = result.pose.position.x
    y = result.pose.position.y
    r.sleep()

    if not started:
        old_x=x
        old_y=y
        started=True

    old_x = x
    old_y = y
        
    header.stamp = rospy.Time.now()
    path.header = header
    pose = PoseStamped()
    pose.header.frame_id = header.frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose = result.pose
    path.poses.append(pose)

    data[counter] = pose
    counter += 1
    path_pub.publish(path)
        
path_pub.publish(path)
execution_time = rospy.get_time()-starting_time


rospy.loginfo("Monitoring done")

# Isolating the x, y coordinates for the real path
real_path = [[element.pose.position.x, element.pose.position.y] for element in path.poses]

# getting the area between the global path and real path as well as the total distance travelled by robot
global_area, global_traveled_distance = compare_paths(global_planner_plan, real_path, 100000)

# continuously publishing these infos when the navigation is done
while not rospy.is_shutdown(): 
    dist_travelled_pub.publish(global_traveled_distance)
    global_area_pub.publish(global_area)
    # the real path is published as the isolation of x,y coordinates is done locally in the other file as well
    path_pub.publish(path)

# printing the data for debugging
# rospy.loginfo(global_area)
# rospy.loginfo(global_traveled_distance)

rospy.spin()