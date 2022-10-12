import pandas as pd
import numpy as np
import os
from navigation_testing import append_df_to_excel

def get_data(alg, world, dynamic, sheet, if_moving):
    file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/"+world.lower()+"/benchmarking_other_"+world.lower()+".xlsx"
    if dynamic:
        file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/"+world.lower()+"/benchmarking_other_"+world.lower()+"_dynamic.xlsx"
    if if_moving:
        file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/"+world.lower()+"/benchmarking_moved_other_"+world.lower()+".xlsx"
    file_data = pd.read_excel(file_path,sheet_name=sheet,header=0)
    time = file_data["Time taken"][0]
    acc = file_data["distance offset"][0]
    dist = file_data["distance traveled"][0]
    area = file_data["area between paths"][0]
    return time, acc, dist, area


if __name__ == "__main__":
    alg = "teb"
    worlds = ["office"]
    dynamic = False
    moving = False
    file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/other_metrics.xlsx"
    if dynamic:
        file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/other_metrics_dynamic.xlsx"
    if moving:
        file_path = os.path.abspath(__file__+"../../..")+"/data/"+alg.upper()+"/other_moving_metrics.xlsx"
    for world in worlds:
        print("Currently on world "+world)
        times = []
        accuracies = []
        dists = []
        areas = []
        datatypes = ["trial" for i in range(0,10)]
        collected_data = pd.DataFrame()
        for i in range(0,10):
            print("Trial No. "+str(i))
            time, accuracy, distance, area = get_data(alg,world,dynamic,i, moving)
            times.append(time)
            accuracies.append(accuracy)
            dists.append(distance)
            areas.append(area)
            # print(file_data["Time taken"][0])
        times.append(np.average(times))
        accuracies.append(np.average(accuracies))
        dists.append(np.average(dists))
        areas.append(np.average(areas))
        datatypes.append("average")

        collected_data["time taken"] = times
        collected_data["distance offset"] = accuracies
        collected_data["distance traveled"] = dists
        collected_data["area between paths"] = areas
        collected_data["data type"] = datatypes
        append_df_to_excel(file_path, collected_data,world)
