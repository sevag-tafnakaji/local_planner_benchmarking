#!/usr/bin/env python3

import pandas as pd
import numpy as np
import plotly.express as px
import plotly.io as pio
from analyze_path_smoothness import extract_path

if __name__ == "__main__":
    world = "playground"
    v2 = True
    dynamic = False
    can_center = [-0.9111032, 0.7030391]
    can_radius = 0.06
    t = np.linspace(0,2*np.pi, 50)
    x = can_center[0] + can_radius*np.cos(t)
    y = can_center[1] + can_radius*np.sin(t)
    for i in range(0,10):
        planned_path = "benchmarking/data/DWB/"+world+"/benchmarking_paths_"+world+".xlsx"
        real_path_dwb = "benchmarking/data/DWB/"+world+"/benchmarking_paths_"+world+".xlsx"
        real_path_teb = "benchmarking/data/TEB/"+world+"/benchmarking_paths_"+world+".xlsx"
        if v2:
            real_path_dwb = "benchmarking/data/DWB/"+world+"/benchmarking_paths_"+world+"_dynamic_v2.xlsx"
            real_path_teb = "benchmarking/data/TEB/"+world+"/benchmarking_paths_"+world+"_dynamic_v2.xlsx"
        planned_paths_raw = pd.read_excel(planned_path, sheet_name=i, header=0)
        actual_paths_raw_dwb = pd.read_excel(real_path_dwb, sheet_name=i, header=0)
        actual_paths_raw_teb = pd.read_excel(real_path_teb, sheet_name=i, header=0)
        global_paths = extract_path(planned_paths_raw, "global path")
        actual_paths_dwb = extract_path(actual_paths_raw_dwb, "real path")
        actual_paths_teb = extract_path(actual_paths_raw_teb, "real path")
        paths = {"x": [], "y": [], "path": []}
        for path in global_paths:
            paths["x"].append(path[0])
            paths["y"].append(path[1])
            paths["path"].append("planned")
        for path in actual_paths_dwb:
            paths["x"].append(path[0])
            paths["y"].append(path[1])
            paths["path"].append("DWB")
        for path in actual_paths_teb:
            paths["x"].append(path[0])
            paths["y"].append(path[1])
            paths["path"].append("TEB")
        if v2:
            for j,point in enumerate(t):
                paths["x"].append(x[j])
                paths["y"].append(y[j])
                paths["path"].append("obstacle")
        
        trial_num = i+1
        fig = px.line(paths, x="x", y="y", color="path", title="planned vs. travelled path, trial"+str(trial_num))
        
        # print(i)
        image_path = "mobile_manipulator/src/benchmarking/images/simple paths/path_trial_"+str(trial_num)+".png"
        # print(image_path)
        # fig.write_image(image_path)
        pio.write_image(fig, image_path, format="png")
        # fig.show()