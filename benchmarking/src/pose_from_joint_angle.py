from cmath import nan
from navigation_testing import append_df_to_excel
from UR_FK_calculator import HTrans
import numpy as np
import pandas as pd
import plotly.express as px
import os
from navigation_testing import append_df_to_excel

def get_joint_time(state_list):
    time_index = state_list.index("stamp:")
    # state_list, time_index
    time = int(state_list[time_index+1].split(":")[1])+int(state_list[time_index+2].split(":")[1])*1e-9
    # time = int(state_list[time_index+1])+int(state_list[time_index+2])*1e-9
    return time

def get_joint_angles(state_list):
    angles_index = state_list.index("name:")+7
    angles_string = state_list[angles_index]
    angles_string = angles_string.split(":")[1]
    angles_string = angles_string[1:-1]
    angles_string = angles_string.split(",")
    angles = np.zeros(6)
    for i,angle in enumerate(angles_string):
        angles[i] = np.float64(angle)
    return angles

def get_joint_names(state):
    names = []
    name_index = state.index("name:")+1
    for i in range(name_index, name_index+6):
        names.append(state[i][7:])
    return names

def extract_data(raw_joints):
    states = raw_joints["joint states"]
    times = np.zeros((len(raw_joints)))
    angles = np.zeros((len(raw_joints), 6))
    names = []
    named = False
    for i,state in enumerate(states):
        state_filtered = state.replace(" ", "").split("\n")
        if not named:
            names = get_joint_names(state_filtered)
            named = True
        times[i] = get_joint_time(state_filtered)
        angles[i] = get_joint_angles(state_filtered)
    return times, names, angles

def extract_time(excel_file, column_name):
    """
        Gets the time from time columns in excel files.
    """
    raw_times = excel_file[column_name]
    times = np.zeros(len(raw_times))
    for i,time in enumerate(raw_times):
        times[i] = time
    return times

def extract_path(excel_file, column_name="real path"):
    """
        Extracts the 2D position from excel files.
        Used here for the footprint positions from paths.
    """
    raw_positions = excel_file[column_name].dropna()
    positions = np.zeros((len(raw_positions), 2))
    for i,position in enumerate(raw_positions):
        position = position[1:-1]
        position = position.split(", ")
        positions[i] = np.array([float(position[0]), float(position[1])])
    return positions

def extract_position(excel_file, column_name="pose"):
    """
        Exctracs the 3D position from excel files. 
        Used here for the end-effector pose
    """
    poses = excel_file[column_name].apply(str.split,("\n"))
    positions = np.zeros((len(poses), 3))
    for i,pose in enumerate(poses):
        positions[i] = np.array([pose[2], pose[4], pose[6]])
    return positions

def compare_times(ee_times, joint_times):
    indecies = []
    tol = 1e-2
    last_step = 0
    for i in range(0, len(ee_times)):
        for j in range(last_step, len(joint_times)):
            diff_times = abs(ee_times[i] - joint_times[j])
            # print(diff_times)
            if diff_times < tol:
                # print(joint_times[j], ee_times[i], diff_times, j)
                indecies.append((i,j))
                last_step = j
                break
            # elif diff_times > 0.5:
            #     last_step = j
            #     print(ee_times[i], joint_times[j], diff_times)
            #     break
        
    return indecies 

def get_theta(joint_df, names):
    th = np.zeros((joint_df.shape[0], 6))
    # print(th.shape)
    for i,name in enumerate(names):
        th[:,i] = joint_df[name]
    # print("")
    # print(th.shape)
    return th

def get_theta(joint_df):
    th = np.zeros((joint_df.shape[0], 6))
    counter = 0
    for (column_name, column_data) in joint_df.iteritems():
        if column_name == "Times":
            continue
        th[:, counter] = column_data.to_numpy()
        counter += 1

    return th

def process_data(ee_data_raw, joint_data_raw, path_data_raw):
    
    # Separate data from Joint raw data
    times_data, joint_names, joint_angles = extract_data(joint_data_raw)

    # Extract (x,y)-coordinates of travelled path
    travelled_path = extract_path(path_data_raw)

    # Cleaned up joint data put back into a df
    joint_data = {}
    joint_data["Times"] = times_data
    for i, name in enumerate(joint_names):
        joint_data[name] = joint_angles[:,i]
    data_df = pd.DataFrame(joint_data)

    # Get times measured from EE measurements and path measurements
    ee_times = extract_time(ee_data_raw, "time")
    path_times = extract_time(path_data_raw, "local_times")
    path_times = path_times[~np.isnan(path_times)]

    # Get path time measurements more close to ee time measurements (approx 10 times reduction)
    filtered_travelled_path = np.zeros((len(ee_times)-1,2))
    filtered_path_times = np.zeros(len(ee_times)-1)
    for i in range(0,len(ee_times)-1):
            # path had measurements every 0.05s, ee path had one every 0.5,
            # hence the index increments by 10.
            # Most of the time the last measurements were not the same and therefore
            # it is dropped from the new ee path array
            if i*10<len(travelled_path):
                filtered_path_times[i] = path_times[i*10] if path_times[i*10] != 0 else nan 
                filtered_travelled_path[i] = travelled_path[i*10]

    # If 0's added due to this filtering process, remove them
    filtered_path_times = filtered_path_times[filtered_path_times!=0]
    filtered_travelled_path = filtered_travelled_path[np.all(filtered_travelled_path != 0, axis=1)]

    filtered_travelled_path = filtered_travelled_path.tolist()

    # Clean up path data
    travelled_path_df = pd.DataFrame({"Times": filtered_path_times, "Travelled Path": filtered_travelled_path})

    # Make ee measurements and joint measurements have measurements taken at approx the same time
    # This filtering step uses the times to find the right indeces for the df's of respective data's
    indecies = compare_times(ee_times, times_data)
    indecies_joints = []
    indecies_ee = []
    for index in indecies:
        indecies_joints.append(index[1])
        indecies_ee.append(index[0])
    # Indecies found, taking the right data
    temp_joints_data = data_df.iloc[indecies_joints]
    temp_ee_data = ee_data_raw.iloc[indecies_ee]

    # Now doing the same step with the paths as they might not be exactly the same as EE
    # Since ee and joint data are approx the same, they can use the same indecies
    temp_ee_time = extract_time(temp_ee_data, "time")
    indecies = compare_times(temp_ee_time, filtered_path_times)
    indecies_joints = []
    indecies_ee = []
    indecies_path = []
    for index in indecies:
        indecies_joints.append(index[0])
        indecies_ee.append(index[0])
        indecies_path.append(index[1])

    # Cleaned up data. All have approx measurements at the same time and same number of total measurements
    final_ee_data = temp_ee_data.iloc[indecies_ee]
    final_joints_data = temp_joints_data.iloc[indecies_joints]
    final_travelled_path_data = travelled_path_df.iloc[indecies_path]

    return final_ee_data, final_joints_data, final_travelled_path_data

def get_expected_poses(joint_data, path_data, offsets):
    x_offset, y_offset, z_offset = offsets[0], offsets[1], offsets[2]
    # Create matrix of angles, rows are times, columns are specific joints
    th = np.zeros((joint_data.shape[0], 6))


    th = get_theta(joint_data)
    ee_FK_transforms = []
    ee_expected_poses_base_frame = []
    ee_expected_poses = np.zeros((joint_data.shape[0], 3))
    path_poses = path_data["Travelled Path"].to_numpy()
    
    
    for i in range(0,th.shape[0]):
        matrix = HTrans(th,i)
        ee_FK_transforms.append(matrix)
        temp_poses = matrix[0:3,3]
        ee_expected_poses_base_frame.append(np.asarray(temp_poses).reshape(-1))
    for i,pose in enumerate(ee_expected_poses_base_frame):
        
        ee_expected_poses[i] = np.array([pose[0]+x_offset+path_poses[i][0], pose[1]+y_offset+path_poses[i][1], pose[2]+z_offset])

    return ee_expected_poses

def calc_errors_with_avg(expected_poses, actual_poses):
    errors = np.zeros((len(expected_poses)+1, 3))

    error_df = pd.DataFrame({"x": [], "y": [], "z": [], "type": []})
    for i,pose in enumerate(expected_poses):
        error_x = np.abs(actual_poses[i][0] - pose[0])
        error_y = np.abs(actual_poses[i][1] - pose[1])
        error_z = np.abs(actual_poses[i][2] - pose[2])
        errors[i] = np.array([error_x, error_y, error_z])
    average_error = np.average(errors,axis=0)
    errors[len(errors)-1] = average_error
    types = ["trial" for i in range(0,len(errors)-1)]
    types.append("average")
    error_df = pd.DataFrame({"x": errors[:,0], "y": errors[:,1], "z": errors[:,2], "type": types})
    return error_df
    
def plot_error(error_data, fig_title):
    fig = px.scatter_3d(error_data, x="x", y="y", z="z", color="type", title=fig_title)
    # fig.show()
    image_path = os.path.abspath("benchmarking/images/"+alg.upper()+"/ee_path_moving_smoothness/"+alg.upper()+"_ee_path_moving_smoothness_"+world+".png")
    fig.write_image(image_path)


if __name__ == "__main__":
    algs = ["dwb", "teb"]
    worlds = ["office"]
    # Offset of base link of the arm from where the path measurement is taken from
    x_offset, y_offset, z_offset = 0.188, 0, 1.007298
    
    for alg in algs:
        file_path = os.path.abspath("benchmarking/data/"+alg.upper()+"/ee_path_moving_smoothness.xlsx")
        for world in worlds:
            integrated_errors = []
            avg_errors = np.zeros((10,3))
            fig_title = "end effector smoothness of "+alg.upper()+" in "+world+" when moving the arm"
            for i in range(1,10):
                joint_value_raw = pd.read_excel(os.path.abspath("benchmarking/data/"+alg.upper()+"/"+world+"/benchmarking_moved_joints_"+world+".xlsx"),i, index_col=0)
                ee_smoothness_raw = pd.read_excel(os.path.abspath("benchmarking/data/"+alg.upper()+"/"+world+"/benchmarking_moved_ee_path_"+world+".xlsx"), i, index_col=0)
                travelled_path_raw = pd.read_excel(os.path.abspath("benchmarking/data/"+alg.upper()+"/"+world+"/benchmarking_moved_paths_"+world+".xlsx"), i, index_col=0)

                ee_pose_data, joint_value_data, path_travelled_data = process_data(ee_smoothness_raw, joint_value_raw, travelled_path_raw)
                
                offsets = [x_offset, y_offset, z_offset]
                ee_expected_poses = get_expected_poses(joint_value_data, path_travelled_data, offsets)
                ee_actual_poses = extract_position(ee_pose_data)
                
                ee_errors = calc_errors_with_avg(ee_expected_poses, ee_actual_poses)
                np_errors = np.array([ee_errors["x"][:], ee_errors["y"][:], ee_errors["z"][:]])
                np_errors = np_errors.transpose()

                # print(np_errors[0:-1])
                integrated_error = np.trapz(np_errors[0:-1], axis=0)
                # avg_trial_error = np.array([ee_errors["x"].iloc[-1], ee_errors["y"].iloc[-1], ee_errors["z"].iloc[-1]]) 
                # avg_errors[i] = avg_trial_error
                integrated_errors.append(integrated_error)
            integrated_errors.append(np.average(integrated_errors[0:len(integrated_errors)-1],axis=0))
            # avg_errors[avg_errors.shape[0]-1] = np.average(avg_errors[0:avg_errors.shape[0]-1],axis=0)
            avg_errors = np.array(integrated_errors)
            types = ["Trial avg" for i in range(0,avg_errors.shape[0]-1)]
            types.append("Average")
            avg_error_df = pd.DataFrame({"x":avg_errors[:,0], "y":avg_errors[:,1], "z":avg_errors[:,2], "type": types})
            append_df_to_excel(file_path, avg_error_df, world)
            plot_error(avg_error_df, fig_title)

