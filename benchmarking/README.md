In this directory you will find all the necessary code and launch files to run the benchmarkings.


# Launch Files

The `launch` directory contains the launch files for each world. In the `[world_name]_benchmark.launch` files you can change where obstacles would be added for when running the dynamic version of the benchmarking as well as the starting pose of the robot.

In `common_benchmarking.launch` you can control which robot is simulated, which navigation stack implementation is used, as well as which benchmarking criteria is used (bottom of the file).

# Data Collection

The following files are used for data collection:

1. `ee_stability_metric.py`: Collects the path data of the end-effector link of the arm. Continuously publishes the data during the benchmarking in the publisher `gripper_info_publisher` from the node `ee_stability_metric`.

2. `ee_smoothness_dynamic.py`: similar to the previous script. This one is used when the arm is to be moved during the benchmarking. This will provide the expected position of the end-effector that can be compared with the actual path travelled found by the previous script.

3. `local_path_metric.py`: Gets the planned global path and the actual travelled path, and publishes the travelled distance and area between the paths. 

4. `navigation_status.py`: Keeps track of the navgiation status. The status value depends on the datatype [GoalStatusArray](http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatusArray.html) where we care only for `PENDING`, `ACTIVE` (or travelling), and `SUCCEEDED` states (or 0, 1, and 3 respectively).

5. `navigation_test_publisher.py`: publishes the desired goal pose the robot would navigate to for the benchmarking. The goal changes based on the current world used.

6. `navigation_testing.py`: Collects all the data once the benchmarking trial is over, and saves them into excel files found in the `data` directory. Depending on the algorithm and other parameters they would be saved in different files/directories.


# Data Analysis

1. `compare_paths.py`: Contains functions that finds the local planner path length, and the area between the local planner path and the global planner path.

2. `analyze_ee_stability.py`: Will calculate and save the smoothness of the path of the end-effector metric as well as save a plot. You can change which worlds, algorithms, configurations of trials you want to analyze based on the arrays

3. `pose_from_joint_angle.py`: Does the same as the previous script except it is used for when the arm is moved during the benchmarking. The script uses the Forward Kinematics calculator from `UR_FK_calculator.py` to calculate the metric values and save them into excel files.

4. `analyze_path_smoothness.py`: Calculates the path smoothness of the local path for all different configurations of parameters.

5. `analyze_other.py`: Gathers the rest of the benchmarking criteria such as area between paths, travelled distance, total time taken, and final position accuracy.

The rest of the scripts are used for cleaning up plots. The plotting is done using the plotly library. If plots are not needed for your implementation, then you'd need to comment it out manually. The excel sheets are found in the `data` directory.

# Worlds

For the paper, only three of the worlds were used but there are many more available in this repository. The worlds used in the paper are `playground`, `warehouse`, and `office`. 

## Adding worlds:

To add worlds, you need to add the files into the `worlds` directory, create a corresponding launch file in the `launch` directory, and add goal poses in the `navigation_test_publisher.py` script.

# Changing the Robot

If you wish to run the benchmark using another robot, make sure to add the necessary files as a separate ros package. then you'd need to change the `common_benchmarking.launch` file to launch the right `robot_description` and start the right controllers. Keep in mind that you also need to add your own navigation stack implementation as the one found in this project corresponds to the MiR100 mobile base.