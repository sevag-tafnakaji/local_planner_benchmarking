import roslaunch
import rospy
from sys import argv
import os
# custom error type. 
class ProgramFailed(Exception):
    """Raised when the program failes to exceute its intended function"""
    pass

# default terms if none are given when launching the script
# i.e. 
world_name = "new_maze"
dynamic = False
local_planner = "dwb"
num_trials = 10
is_moving = False

def start_roslaunch(world_name, dynamic, trial, local_planner, is_moving):
    """
        start the benchmarking launch file using the given parameters.

        This is built on the example gotten from the roslaunch api page.

        @param world_name: the name of the world the benchmarking is done in.
                           (Example: "new_maze", "playground")
        
        @param dynamic: if there will be dynamic obstacles in the world

        @param trial: the number for the trial (used when exporting the data)

        @param local_planner: the algorithm for the local planner.
                              (Example: "dwb", "teb")
    """
    #check the example for explenations
    path_to_launch = os.path.abspath("../launch")+"/{}_benchmark.launch".format(world_name)
    cli_args = [path_to_launch,f'dynamic:={dynamic}',f"trial_arg:={trial}", f"local_planner:={local_planner}", f'move_arm:={is_moving}']
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    # connecting to master
    rospy.init_node("benchmarking_navigation", anonymous=True)
    
    # wait until benchmarking is finished (or crashed)
    while not rospy.get_param("/benchmarking_done") == "true" and not rospy.is_shutdown(): 
        # if the benchmarking crashes, stop the launch file and raise an error
        if rospy.get_param("/benchmarking_done") == "failed":
            parent.shutdown()
            raise ProgramFailed("Benchmarking trial failed")
    parent.shutdown()
    
if __name__ == "__main__":
    # updating the terms if any are given.
    # to avoid index errors, this is done.
    num_terms = len(argv)
    if num_terms > 1:
        if num_terms == 2:
            world_name = argv[1]
        elif num_terms == 3:
            world_name = argv[1]
            local_planner = argv[2]
        elif num_terms == 4:
            world_name = argv[1]
            local_planner = argv[2]
            dynamic = argv[3]
        elif num_terms == 5:
            world_name = argv[1]
            local_planner = argv[2]
            dynamic = argv[3]
            is_moving = argv[4]
        elif num_terms == 6:
            world_name = argv[1]
            local_planner = argv[2]
            dynamic = argv[3]
            is_moving = argv[4]
            num_trials = argv[5]
    # These are terms that indicate if the benchmarking failed or succeeded
    search_term = "Data saved, shutting down"
    error_term = "Program failed to collect data. Please repeat trial"

    # In case a capital letter was used
    world_name = world_name.lower()
    local_planner = local_planner.lower()

    # starting the launch file for all trials
    for i in range(1, num_trials+1):
        try:
            start_roslaunch(world_name, dynamic, i, local_planner, is_moving)
        except ProgramFailed:
            # repeat trial if the benchmarking failed.
            # if the benchmarking fails again, then the entire script will stop
            start_roslaunch(world_name, dynamic, i, local_planner, is_moving)
        


    
