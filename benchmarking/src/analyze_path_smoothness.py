import pandas as pd
import numpy as np
import plotly.express as px
import os
from analyze_ee_stability import extract_path
from navigation_testing import append_df_to_excel

def make_unit_vectors(path_measurements):
    vectors = np.zeros((path_measurements.shape[0]-2,2))
    for i,pose in enumerate(path_measurements):
        if i==1 or i==path_measurements.shape[0]-1:
            continue
        vectors[i-1] = np.array([pose[0]-path_measurements[i-1][0], pose[1]-path_measurements[i-1][1]])
        if np.linalg.norm(vectors[i-1]) > 0:
            vectors[i-1] = vectors[i-1]/np.linalg.norm(vectors[i-1])
        # print(pose, vectors[i-1])
    return vectors 

def find_angles(unit_vectors):
    angles = np.zeros(unit_vectors.shape[0]-2)
    for i,vector in enumerate(unit_vectors):
        if i==1 or i == unit_vectors.shape[0]-1:
            continue
        angle = np.arccos(np.dot(vector, unit_vectors[i-1]))
        angles[i-1] = angle
    return angles

def plot_angles_from_path(path, fig_title, alg, world, trial_num, dynamic, moved):
    vectors = make_unit_vectors(path)
    alphas = find_angles(vectors)
    path_smoothness = np.sum(np.array([alpha for alpha in alphas]))
    # print(path_smoothness)
    data = {"x": [], "y": [], "angle": []}
    for j,pose in enumerate(path):
        data["x"].append(pose[0])
        data["y"].append(pose[1])
        if j < len(path)-4:
            data["angle"].append(alphas[j])
        else:
            data["angle"].append(0)
    fig = px.scatter_3d(data, x="x", y="y", z="angle", color="angle")
    # fig.show()
    image_path = os.path.abspath(__file__+"../../..")+"/images/"+alg.upper()+"/path_smoothness/path_smoothness_"+world+"_trial"+str(trial_num)+".png"
    if dynamic:
        image_path = os.path.abspath(__file__+"../../..")+"/images/"+alg.upper()+"/path_smoothness/dynamic_path_smoothness_"+world+"_trial"+str(trial_num)+".png"
    if moved:
        image_path = os.path.abspath(__file__+"../../..")+"/images/"+alg.upper()+"/path_smoothness/path_moved_smoothness_"+world+"_trial"+str(trial_num)+".png"
    camera = dict(eye=dict(x=1.0, y=-1.5, z=1.2))
    side = 900
    fig.update_layout(scene_camera=camera, 
        scene=dict(xaxis=dict(range=[min(data["x"])*1.2,max(data["x"])*1.2]), 
                   yaxis=dict(range=[min(data["y"])*1.2,max(data["y"])*1.2]), 
                   zaxis=dict(range=[min(data["angle"])*1.2,max(data["angle"])*1.2])),
        width=side, height=side, coloraxis=dict(colorbar=dict(orientation="h")),
        coloraxis_colorbar_y = 0.90, font=dict(size=17))
    fig.write_image(image_path)
    return path_smoothness

if __name__ == "__main__":
    algs = ["dwb", "teb"]
    # world_name = "complex_maze"
    dynamics = [False, True]
    moving = False
    # path_smoothnesses = {"world": [], "smoothness": [], "datatype": []}
    # path_smoothnesses = pd.DataFrame(path_smoothnesses)
    for alg in algs:
        for dynamic in dynamics:
            worlds = ["playground", "office", "warehouse"]
            if alg == "dwb" and dynamic:
                worlds = ["playground", "office"]
            file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/path_smoothness.xlsx"
            if dynamic:
                file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/path_smoothness_dynamic.xlsx"
            if moving:
                file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/path_moving_smoothness.xlsx"
            path_smoothnesses = [pd.DataFrame() for i in range(0,len(worlds))]
            for n,world in enumerate(worlds):
                print("Currently working on "+world)
                world_data = {"smoothness": [], "datatype": []}
                average_data = {"smoothness": [], "datatype": ["average"]} 
                for j in range(0,10):
                    print("Trial Nr."+str(j+1))
                    data_file = pd.read_excel(os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/"+world.lower()+"/benchmarking_paths_"+world.lower()+".xlsx", sheet_name=j, header=0)
                    if dynamic:
                        data_file = pd.read_excel(os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/"+world.lower()+"/benchmarking_paths_"+world.lower()+"_dynamic.xlsx", sheet_name=j, header=0)
                    if moving:
                        data_file = pd.read_excel(os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/"+world.lower()+"/benchmarking_moved_paths_"+world.lower()+".xlsx", sheet_name=j, header=0)
                    travelled_path = extract_path(data_file)
                    figure_title = "angles along path, "+alg.upper()+" in "+world+": Trial "+str(j+1)
                    if dynamic:
                        figure_title = "angles along path, "+alg.upper()+" in dynamic "+world+": Trial "+str(j+1)
                    if moving:
                        figure_title = "angles along path, "+alg.upper()+" in "+world+" while moving the arm: Trial "+str(j+1)
                    smoothness = plot_angles_from_path(travelled_path, figure_title,alg,world,j+1, dynamic,moving)
                    world_data["smoothness"].append(smoothness)
                    world_data["datatype"].append("trial")
                average_smooth = np.average(world_data["smoothness"])
                average_data["smoothness"] = average_smooth
                world_data = pd.DataFrame(world_data)
                average_data = pd.DataFrame(average_data)
                path_smoothnesses[n] = pd.concat([world_data, average_data], ignore_index=True, axis=0)
                
                # append_df_to_excel(file_path, path_smoothnesses[n],world)