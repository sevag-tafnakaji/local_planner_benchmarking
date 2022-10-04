import pandas as pd
import numpy as np
import plotly.express as px
from navigation_testing import append_df_to_excel

# ee_file = pd.read_excel("benchmarking/data/DWB/new_maze/benchmarking_ee_path_new_maze.xlsx",sheet_name=1, header=0)
# mb_file = pd.read_excel("benchmarking/data/DWB/new_maze/benchmarking_paths_new_maze.xlsx",sheet_name=1, header=0, dtype=list)


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

def extract_time(excel_file, column_name):
    """
        Gets the time from time columns in excel files.
    """
    raw_times = excel_file[column_name]
    times = np.zeros(len(raw_times))
    for i,time in enumerate(raw_times):
        times[i] = time
    return times

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

def calculate_error(world_name, alg, dynamic):
    """
        Takes in the necessary information to calculate the error between where the end-effector
        should be and where it actually is. The data is taken from an excel sheet.
    """
    errors = np.zeros((9,3))
    for j in range(1,10):
        ee_file = pd.read_excel("benchmarking/data/"+alg.upper()+"/"+world_name.lower()+"/benchmarking_ee_path_"+world_name.lower()+".xlsx",sheet_name=j, header=0)
        mb_file = pd.read_excel("benchmarking/data/"+alg.upper()+"/"+world_name.lower()+"/benchmarking_paths_"+world_name.lower()+".xlsx",sheet_name=j, header=0, dtype=list)
        if dynamic:
            ee_file = pd.read_excel("benchmarking/data/"+alg.upper()+"/"+world_name.lower()+"/benchmarking_ee_path_"+world_name.lower()+"_dynamic.xlsx",sheet_name=j, header=0)
            mb_file = pd.read_excel("benchmarking/data/"+alg.upper()+"/"+world_name.lower()+"/benchmarking_paths_"+world_name.lower()+"_dynamic.xlsx",sheet_name=j, header=0, dtype=list)

        # Getting the position data
        travelled_path = extract_path(mb_file)
        ee_path = extract_position(ee_file)
        # Getting the time data
        path_times = extract_time(mb_file, "local_times")
        ee_times = extract_time(ee_file, "time")
        l2 = len(ee_path)

        # defining two new arrays in order to have the same length
        filtered_travelled_path = np.zeros((l2-1,2))
        filtered_ee_path = np.delete(ee_path,-1,0)
        filtered_time = np.delete(ee_times,-1)
        for i in range(0,l2-1):
            # path had measurements every 0.05s, ee path had one every 0.5,
            # hence the index increments by 10.
            # Most of the time the last measurements were not the same and therefore
            # it is dropped from the new ee path array
            if i*10<len(travelled_path):
                filtered_travelled_path[i] = travelled_path[i*10]
        expected_z = 2.3
        # print(len(filtered_travelled_path), len(filtered_ee_path))
        error = np.zeros((len(filtered_ee_path), 3))
        for i, pos in enumerate(filtered_ee_path):
            
            error[i,0] = np.abs(pos[0] - filtered_travelled_path[i][0])
            error[i,1] = np.abs(pos[1] - filtered_travelled_path[i][1])
            error[i,2] = np.abs(pos[2] - expected_z)
            # print(filtered_travelled_path[i],pos, error[i])
        integrated_errors = np.trapz(error,filtered_time,axis=0)
        errors[j-1] = integrated_errors
    return errors

def plot_data_with_average(array, fig_title, alg, world):
    average_error = np.average(array,axis=0)
    average_data = {}
    average_data["x"] = [average_error[0]]
    average_data["y"] = [average_error[1]]
    average_data["z"] = [average_error[2]]
    average_data["datatype"] = ["average"]
    # print(average_data)
    data = {}
    data["x"] = array[:,0]
    data["y"] = array[:,1]
    data["z"] = array[:,2]
    data["datatype"] = ["trial" for i in range(0,len(array))]
    data = pd.DataFrame(data)
    average_data = pd.DataFrame(average_data)
    data = pd.concat([data, average_data], ignore_index=True, axis=0)
    # print(data)
    fig = px.scatter_3d(data, x="x", y="y", z="z", color="datatype", title=fig_title)
    # fig.show()
    image_path = "mobile_manipulator/src/benchmarking/images/"+alg.upper()+"/ee_path_smoothness/ee_arm_motion_path_smoothness_"+world+".png"

    fig.write_image(image_path)
    return data

if __name__ == "__main__":
    alg = "dwb"
    # worlds = ["new_maze", "complex_maze", "office", "old_maze", "playground", "warehouse"]
    world_name = "office"
    file_path = "benchmarking/data/"+alg.upper()+"/ee_arm_motion_path_smoothness.xlsx"
    errors = pd.DataFrame({})

    figure_title = "end effector smoothness of "+alg.upper()+" in "+world_name+" with arm motion along the path"

    calculated_errors = calculate_error(world_name,alg)
    errors = plot_data_with_average(calculated_errors, figure_title, alg, world_name)
    # print(errors[i])
    append_df_to_excel(file_path, errors, world_name)
