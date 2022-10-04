#!/usr/bin/env python3

import pandas as pd
import plotly.express as px

if __name__ == "__main__":
	algs = ["dwb", "teb"]
	dynamics = [False, True]
	moved = [False, True]
	worlds = []
	errors = {"x": [], "y":[], "z":[], "planner": [], "world":[]}
	for is_moved in moved:
		for dynamic in dynamics:
			for alg in algs:
				alg = alg.upper()
				# print(alg, dynamic, is_moved)
				if alg == "DWB" and not dynamic:
					worlds = ["new_maze", "office", "old_maze", "playground", "warehouse"]
				elif alg == "DWB" and dynamic:
					worlds = ["playground"]
				elif alg == "TEB" and dynamic:
					worlds = ["playground", "warehouse"]
				elif alg == "TEB" and not dynamic:
					worlds = ["new_maze", "office", "playground", "warehouse"]
				if is_moved:
					worlds = ["office"]
				for world in worlds:
					path = "benchmarking/data/"+alg+"/ee_path_smoothness.xlsx"
					world_name = world
					if dynamic:
						path = "benchmarking/data/"+alg+"/ee_path_smoothness_dynamic.xlsx"
						world_name = "dynamic "+world
					if is_moved:
						path = "benchmarking/data/"+alg+"/ee_path_moving_smoothness.xlsx"
						world_name = "arm moved, "+world
					errors_raw = pd.read_excel(path, sheet_name=world, header=0)
					average_error = errors_raw.iloc[-1]
					errors["x"].append(average_error["x"])
					errors["y"].append(average_error["y"])
					errors["z"].append(average_error["z"])
					errors["planner"].append(alg)
					errors["world"].append(world_name)
	fig = px.scatter_3d(errors, x="x", y="y", z="z", color="world", symbol="planner") 
	image_path = "mobile_manipulator/src/benchmarking/images/ee_path_smoothness.png"
	fig.write_image(image_path)
	# fig.show()
