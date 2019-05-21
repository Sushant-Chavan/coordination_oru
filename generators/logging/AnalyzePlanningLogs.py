#!/usr/bin/env python
# coding: utf-8

import numpy as np
import os
import sys
import argparse
import pandas as pd
import re
import matplotlib.pyplot as plt
from ctypes import *

# A class to pass the array of poses to the shared libarary for comparison
class PathPose(Structure):
    _fields_=[("x", c_double),
              ("y", c_double),
              ("theta", c_double)]

class PlanData():
    def __init__(self, df):
        self.start_time = None
        self.map_resolution = None
        self.start = None
        self.goal = None
        self.is_holonomic = None
        self.planning_time = None
        self.simplification_time = None
        self.from_recall = None
        self.total_planning_time = None
        self.path = None

        self.fill_data(df)

    def fill_data(self, df):
        self.start_time = df["Planning Start Time"].values[0]
        self.map_resolution = df["Map Resolution"].values[0]

        self.start = np.zeros(3)
        self.start[0] = df["Start X"].values[0]
        self.start[1] = df["Start Y"].values[0]
        self.start[2] = df["Start Theta"].values[0]

        self.goal = np.zeros(3)
        self.goal[0] = df["Goal X"].values[0]
        self.goal[1] = df["Goal Y"].values[0]
        self.goal[2] = df["Goal Theta"].values[0]

        self.is_holonomic = (df["Holonomic"].values[0] != 0)
        self.planning_time = df["Planning Time"].values[0]
        self.simplification_time = df["Path simplification time"].values[0]
        self.from_recall = (df["From recall"].values[0] != 0)
        self.total_planning_time = df["Total planning time"].values[0]
        self.path = self.load_path(df["Path"].values[0])

    def load_path(self, path_data):
        path_data = path_data.strip()
        values = [i for i in path_data.split(';')][:-1]
        values = np.array([float(i) for i in values])
        nPoses = int(values.size / 3)
        return values.reshape((nPoses, 3))

# A class to extract relevant lines from a complete log file of a test run and generate a CSV logg file
class AnalyzePlanning:
    def __init__(self, csv_abs_path):
        self.csv_path = csv_abs_path

        self.test_df = None
        self.maps = None
        self.planner = None
        self.plan_stats = None
        self.nTests = None
        self.cdll = None

        self.load_csv()
        self.load_planning_stats()
        self.load_native_library()

    def load_native_library(self):
        self.cdll = cdll.LoadLibrary('libcomparePaths.so')
        self.cdll.comparePaths.arguments = [POINTER(PathPose), c_int, POINTER(PathPose), c_int, c_bool, c_double, c_double]
        self.cdll.comparePaths.restype = c_bool

    def compare_paths(self, plan1, plan2, similarity_threshold):
        pose_array1, pose_array2 = [], []
        for i in range(plan1.path.shape[0]):
            pose = PathPose()
            pose.x = plan1.path[i][0]
            pose.y = plan1.path[i][1]
            pose.theta = plan1.path[i][2]
            pose_array1.append(pose)

        for i in range(plan2.path.shape[0]):
            pose = PathPose()
            pose.x = plan2.path[i][0]
            pose.y = plan2.path[i][1]
            pose.theta = plan2.path[i][2]
            pose_array2.append(pose)

        return self.cdll.comparePaths((PathPose * len(pose_array1))(*pose_array1), len(pose_array1),
                                     (PathPose * len(pose_array2))(*pose_array2), len(pose_array2),
                                     bool(plan1.is_holonomic), c_double(4.0), c_double(similarity_threshold))

    def load_csv(self):
        df = pd.read_csv(self.csv_path, index_col=None)
        col = "Test Start Time"
        test_start_times = df[col].values.tolist()
        test_ids = sorted(set(test_start_times))

        self.test_df = {}
        self.maps = {}
        self.planner = {}
        test_num = 0
        for id in test_ids:
            self.test_df[test_num] = df.loc[df[col] == id]
            self.maps[test_num] = self.test_df[test_num]["Test Name"].values[0]
            self.planner[test_num] = self.test_df[test_num]["Planner Type"].values[0]
            test_num += 1
        self.nTests = len(self.test_df.keys())
        print("CSV contains data for", self.nTests, "tests")

    def load_planning_stats(self):
        self.plan_stats = {}
        for test_id in range(self.nTests):
            plan_stats = {}
            df = self.test_df[test_id]
            plan_num = 0
            for start_time in df["Planning Start Time"].values:
                plan_df = df.loc[df["Planning Start Time"] == start_time]
                plan_stats[plan_num] = PlanData(plan_df)
                plan_num += 1
            self.plan_stats[test_id] = plan_stats

    def get_predictable_path_ratio(self, similarity_threshold=0.25):
        print("Generating path predictability ratios. This may take some time...")
        nPlan_per_iteration = len(self.plan_stats[0])
        matches = np.zeros(nPlan_per_iteration)
        for plan_id in range(nPlan_per_iteration):
            num_of_matches = np.zeros(self.nTests)
            for test_id in range(self.nTests):
                for first_pass_test_id in range(self.nTests):
                    if test_id != first_pass_test_id:
                        res = self.compare_paths(self.plan_stats[test_id][plan_id],
                                              self.plan_stats[first_pass_test_id][plan_id],
                                              similarity_threshold)
                        if res:
                            num_of_matches[test_id] += 1
            max_match_test_id = np.argmax(num_of_matches)
            matches[plan_id] = num_of_matches[max_match_test_id]
        print("Path predictability ratios generation complete!")
        return matches, (self.nTests - 1)  # -1 because we dont test against self test_id

    def plot_path_predictability_stats(self):
        similarity_threshold = 0.25
        success, nTests = self.get_predictable_path_ratio(similarity_threshold)
        failure = (np.ones_like(success) * nTests) - success
        x = np.arange(1, success.size+1, 1)

        f = plt.figure(figsize=(10, 10))
        ax = f.add_subplot(111)
        ax.bar(x, success, color='g', label='Number of similar paths')
        ax.bar(x, failure, bottom=success, color='r', label="Number of non-similar paths")
        ax.plot(x, np.ones(len(x))*np.mean(success), color='b', linestyle="-", label="Average number of similar paths", linewidth=2.5)
        ax.set_xlabel("Plan ID")
        ax.set_ylabel("Number of similar/non-similar paths")
        ax.set_title("Number of predictable paths with similarity threshold = {}".format(similarity_threshold))
        ax.set_xticks(x)
        ax.set_yticks(np.arange(0, nTests+1, 1))
        ax.legend()
        ax.grid()
        plt.savefig("predictability.svg", format='svg')

    def plot_planning_times(self):
        # One extra plot for plotting the average plot of all iterations
        nRows = int((self.nTests + 1) / 2) + ((self.nTests + 1) % 2)
        f = plt.figure(figsize=(15, 7.5 * nRows))

        avg_itr_plan_times = []
        avg_itr_sim_times = []
        avg_itr_total_times = []
        for test_id in range(self.nTests):
            ax = f.add_subplot(nRows, 2, test_id+1)
            plan_stats = self.plan_stats[test_id]
            plan_times = []
            simplification_times = []
            total_times = []
            for plan_id in plan_stats.keys():
                plan_times.append(plan_stats[plan_id].planning_time)
                simplification_times.append(plan_stats[plan_id].simplification_time)
                total_times.append(plan_stats[plan_id].total_planning_time)

            avg_plan_times = np.ones(len(plan_times)) * np.mean(plan_times)
            avg_simplification_times = np.ones(len(plan_times)) * np.mean(simplification_times)
            avg_total_times = np.ones(len(plan_times)) * np.mean(total_times)
            avg_itr_plan_times.append(avg_plan_times[0])
            avg_itr_sim_times.append(avg_simplification_times[0])
            avg_itr_total_times.append(avg_total_times[0])
            x = np.arange(1, len(plan_times)+1, 1)

            ax.plot(x, np.log10(plan_times), label="Planning times", color="r")
            ax.plot(x, np.log10(simplification_times), label="Simplification times", color="b")
            ax.plot(x, np.log10(total_times), label="Total times", color="g")
            ax.plot(x, np.log10(avg_plan_times), label="Average Planning time", color="r", linestyle="--")
            ax.plot(x, np.log10(avg_simplification_times), label="Average Simplification time", color="b", linestyle="--")
            ax.plot(x, np.log10(avg_total_times), label="Average Total time", color="g", linestyle="--")
            ax.set_xlabel("Planning instance")
            ax.set_ylabel("Planning time (in seconds), $log_{10}$ scale")
            ax.set_title("Test number " + str(test_id+1))
            ax.legend()
            ax.grid()

        ax = f.add_subplot(nRows, 2, self.nTests+1)
        avg_plan_time = np.ones(len(avg_itr_plan_times)) * np.mean(avg_itr_plan_times)
        avg_sim_time = np.ones(len(avg_itr_sim_times)) * np.mean(avg_itr_sim_times)
        avg_total_time = np.ones(len(avg_itr_sim_times)) * np.mean(avg_itr_total_times)
        x = np.arange(1, len(avg_itr_plan_times)+1, 1)
        ax.plot(x, np.log10(avg_itr_plan_times), label="Planning times", color="r")
        ax.plot(x, np.log10(avg_itr_sim_times), label="Simplification times", color="b")
        ax.plot(x, np.log10(avg_itr_total_times), label="Total times", color="g")
        ax.plot(x, np.log10(avg_plan_time), label="Average Planning time", color="r", linestyle="--")
        ax.plot(x, np.log10(avg_sim_time), label="Average Simplification time", color="b", linestyle="--")
        ax.plot(x, np.log10(avg_total_time), label="Average Total time", color="g", linestyle="--")
        ax.set_xlabel("Test Number")
        ax.set_ylabel("Mean Planning time (in seconds), $log_{10}$ scale")
        ax.set_title("Mean planning time per test instance")
        ax.legend()
        ax.grid()

        plt.savefig("planning_times.svg", format='svg')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_filename", type=str, help=" CSV logfile (Ex. BRSU_Floor0_lightning.csv)")
    args = parser.parse_args()

    log_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/experienceLogs/")
    csv_abs_path = os.path.abspath(os.path.join(log_dir, args.csv_filename))

    if not os.path.isfile(csv_abs_path):
        print("CSV log file does not exist! \nPath specified was:\n", csv_abs_path)
        return

    ap = AnalyzePlanning(csv_abs_path)
    # ap.plot_planning_times()
    ap.plot_path_predictability_stats()

if __name__ == "__main__":
    main()
