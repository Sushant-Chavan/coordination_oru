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
        self.from_recall = (df["From recall"].values[0] == 1)
        self.total_planning_time = df["Total planning time"].values[0]
        self.path = self.load_path(df["Path"].values[0])

    def load_path(self, path_data):
        path_data = path_data.strip()
        values = [i for i in path_data.split(';')][:-1]
        values = np.array([float(i) for i in values])
        nPoses = int(values.size / 3)
        return values.reshape((nPoses, 3))

class RobotMissionData:
    def __init__(self, df):
        self.plans = None
        self.average_path_planning_time = None
        self.average_path_simplification_time = None
        self.total_path_planning_time = None
        self.total_path_simplification_time = None
        self.total_planning_time = None
        self.is_holonomic = None
        self.complete_path = None
        self.nPlans_from_recall = None

        self.fill_data(df)

    def fill_data(self, df):
        self.plans = []
        self.average_path_planning_time = 0
        self.average_path_simplification_time = 0
        self.total_path_planning_time = 0
        self.total_path_simplification_time = 0
        self.total_planning_time = 0
        self.nPlans_from_recall = 0

        plan_num = 0
        for start_time in df["Planning Start Time"].values:
            plan_df = df.loc[df["Planning Start Time"] == start_time]
            plan_data = PlanData(plan_df)
            self.plans.append(plan_data)

            self.total_path_planning_time += plan_data.planning_time
            self.total_path_simplification_time += plan_data.simplification_time
            self.total_planning_time += plan_data.total_planning_time
            if plan_data.from_recall:
                self.nPlans_from_recall += 1

            if self.complete_path is None:
                self.complete_path = plan_data.path
            else:
                self.complete_path = np.vstack((self.complete_path, plan_data.path))

            plan_num += 1

        nPlans = len(self.plans)
        self.average_path_planning_time = self.total_path_planning_time / nPlans
        self.average_path_simplification_time = self.total_path_simplification_time / nPlans
        self.is_holonomic = self.plans[0].is_holonomic

class FleetMissionData:
    def __init__(self, df):
        self.robot_missions = None
        self.total_planning_time = None
        self.total_path_planning_time = None
        self.total_path_simplification_time = None
        self.nPlans_from_recall = None
        self.nPlans_from_scratch = None
        self.map = None
        self.planner = None
        self.nRobots = None

        self.fill_data(df)

    def fill_data(self, df):
        self.load_robot_missions(df)

        self.map = df["Test Name"].values[0]
        self.planner = df["Planner Type"].values[0]

        self.total_planning_time = 0
        self.total_path_planning_time = 0
        self.total_path_simplification_time = 0
        self.nPlans_from_recall = 0

        for robot_id in range(self.nRobots):
            self.total_planning_time += self.robot_missions[robot_id].total_planning_time
            self.total_path_planning_time += self.robot_missions[robot_id].total_path_planning_time
            self.total_path_simplification_time += self.robot_missions[robot_id].total_path_simplification_time
            self.nPlans_from_recall += self.robot_missions[robot_id].nPlans_from_recall

        # self.total_path_planning_time /= self.nRobots
        # self.total_path_simplification_time /= self.nRobots
        self.nPlans_from_scratch = (self.nRobots * 3) - self.nPlans_from_recall

    def load_robot_missions(self, df):
        nPlans = df.values.shape[0]

        # Check if number of plans in df is a multiple of 3
        assert (nPlans % 3 == 0), "Expected number of plans in a fleet mission is a multiple of 3"
        self.nRobots = int(nPlans / 3)

        self.robot_missions = []
        for i in range(0, nPlans, 3):
            mission_df = df.iloc[i:i+3]
            self.robot_missions.append(RobotMissionData(mission_df))

        assert self.nRobots == len(self.robot_missions), "Number of missions loaded not equal to number of robots in fleet!!"

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
        self.fleet_missions = None
        self.nSimilar_paths = None

        self.load_csv()
        self.load_native_library()

    def load_native_library(self):
        self.cdll = cdll.LoadLibrary('libcomparePaths.so')
        self.cdll.comparePaths.arguments = [POINTER(PathPose), c_int, POINTER(PathPose), c_int, c_bool, c_double, c_double]
        self.cdll.comparePaths.restype = c_bool

    def compare_paths(self, path1, path2, is_holonomic, similarity_threshold):
        pose_array1, pose_array2 = [], []
        for i in range(path1.shape[0]):
            pose = PathPose()
            pose.x = path1[i][0]
            pose.y = path1[i][1]
            pose.theta = path1[i][2]
            pose_array1.append(pose)

        for i in range(path2.shape[0]):
            pose = PathPose()
            pose.x = path2[i][0]
            pose.y = path2[i][1]
            pose.theta = path2[i][2]
            pose_array2.append(pose)

        return self.cdll.comparePaths((PathPose * len(pose_array1))(*pose_array1), len(pose_array1),
                                     (PathPose * len(pose_array2))(*pose_array2), len(pose_array2),
                                     bool(is_holonomic), c_double(4.0), c_double(similarity_threshold))

    def load_csv(self):
        df = pd.read_csv(self.csv_path, index_col=None)
        self.load_fleet_missions(df)

    def load_fleet_missions(self, df):
        col = "Test Start Time"
        test_start_times = df[col].values.tolist()
        test_start_times = sorted(set(test_start_times))

        self.fleet_missions = []
        for fleet_id, start_time in enumerate(test_start_times):
            fleet_df = df.loc[df[col] == start_time]
            self.fleet_missions.append(FleetMissionData(fleet_df))
        print("Loaded", len(self.fleet_missions), "fleet missions")

    def custom_line_plot(self, ax, x, y, label=None, color=None, linestyle=None,
                    linewidth=None, xlabel=None, ylabel=None, title=None,
                    xticks=None, yticks=None, useLog10Scale=False, avg_line_col=None):
        if useLog10Scale:
            y = np.log10(y)
            ylabel = ylabel + " ($log_{10}$ scale)"
        ax.plot(x, y, label=label, color=color, linestyle=linestyle, linewidth=linewidth)

        if avg_line_col is not None:
            y_mean = np.ones_like(y) * np.mean(y)
            ax.plot(x, y_mean, label="Mean " + label, color=avg_line_col)

        if xticks is not None:
            ax.set_xticks(xticks)
        if yticks is not None:
            ax.set_yticks(yticks)

        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend()
        ax.grid()

    def custom_bar_plot(self, ax, x, height, label=None, color=None, barwidth=0.8,
                        bottom=0, xlabel=None, ylabel=None, title=None,
                        xticks=None, yticks=None, avg_line_col=None):
        ax.bar(x, height, label=label, color=color, width=barwidth, bottom=bottom)

        if avg_line_col is not None:
            mean_height = np.ones_like(height) * np.mean(height)
            ax.plot(x, mean_height, label="Mean " + label, color=avg_line_col, linewidth=3.0)

        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)

        if xticks is not None:
            ax.set_xticks(xticks)
        if yticks is not None:
            ax.set_yticks(yticks)

        ax.set_title(title)
        ax.legend()
        ax.grid()

    def plot_fleet_planning_times(self, fleets=None):
        if fleets is None:
            fleets = self.fleet_missions

        fig = plt.figure(figsize=(15, 15))

        nFleets = len(fleets)
        fleet_ids = np.arange(1, nFleets + 1, 1)
        path_planning_times = []
        path_simpl_times = []
        total_plan_times = []
        nPlans_from_recall = []
        nPlans_from_scratch = []

        for f in fleets:
            path_planning_times.append(f.total_path_planning_time)
            path_simpl_times.append(f.total_path_simplification_time)
            total_plan_times.append(f.total_planning_time)
            nPlans_from_recall.append(f.nPlans_from_recall)
            nPlans_from_scratch.append(f.nPlans_from_scratch)

        ax = fig.add_subplot(221)
        self.custom_line_plot(ax, fleet_ids, path_planning_times, label="Path planning time",
                         color='r', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=fleet_ids, useLog10Scale=False, avg_line_col='b')

        ax = fig.add_subplot(222)
        self.custom_line_plot(ax, fleet_ids, path_simpl_times, label="Path simplification time",
                         color='r', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=fleet_ids, useLog10Scale=False, avg_line_col='b')

        ax = fig.add_subplot(223)
        self.custom_line_plot(ax, fleet_ids, total_plan_times, label="Total planning time",
                         color='r', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=fleet_ids, useLog10Scale=False, avg_line_col='b')

        ax = fig.add_subplot(224)
        self.custom_bar_plot(ax, fleet_ids, nPlans_from_recall, label="Number of plans from recall",
                         color='g', xlabel="Fleet ID", ylabel="Count",
                         xticks=fleet_ids, avg_line_col='b')
        self.custom_bar_plot(ax, fleet_ids, nPlans_from_scratch, label="Number of plans from scratch",
                         bottom=nPlans_from_recall, color='r', xlabel="Fleet ID", ylabel="Count",
                         xticks=fleet_ids)

        fig.suptitle("Planning time stats \nPlanner:{} Map:{}".format(fleets[0].planner, fleets[0].map))

        plt.savefig("fleet_planning_times.svg", format='svg')

    def determine_num_similar_paths(self, fleets=None, similarity_threshold=0.25):
        print("Checking similarity of paths. This may take some time...")
        if fleets is None:
            fleets = self.fleet_missions

        max_num_matches = (len(fleets) -1) # -1 because we dont test a path against itself

        similarity_count = np.zeros(fleets[0].nRobots)
        for robot_id in range(fleets[0].nRobots):
            num_of_matches = np.zeros(len(fleets))
            for fleet_id_1 in range(len(fleets)):
                for fleet_id_2 in range(len(fleets)):
                    if fleet_id_1 != fleet_id_2:
                        path1 = fleets[fleet_id_1].robot_missions[robot_id].complete_path
                        path2 = fleets[fleet_id_2].robot_missions[robot_id].complete_path
                        is_holonomic = fleets[fleet_id_2].robot_missions[robot_id].is_holonomic
                        if self.compare_paths(path1, path2, is_holonomic, similarity_threshold):
                            num_of_matches[fleet_id_1] += 1
                            if num_of_matches[fleet_id_1] >= max_num_matches:
                                break
                if num_of_matches[fleet_id_1] >= max_num_matches:
                    break
            similarity_count[robot_id] = np.max(num_of_matches)
            print("\tNumber of similar paths for Robot", robot_id+1, "=", similarity_count[robot_id])
        print("Path similarity tests complete!")
        return similarity_count, max_num_matches

    def plot_path_predictability_stats(self, similarity_threshold = 0.3):
        similarities, nTests = self.determine_num_similar_paths(similarity_threshold=similarity_threshold)
        dissimilarities = (np.ones_like(similarities) * nTests) - similarities
        robot_ids = np.arange(1, similarities.size+1, 1)

        f = plt.figure(figsize=(10, 10))
        ax = f.add_subplot(111)
        self.custom_bar_plot(ax, robot_ids, similarities, label='Number of similar paths',
                         color='g', xlabel="Robot ID", ylabel="Number of similar/non-similar paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+1, 1), avg_line_col='b',
                         title="Number of predictable paths with similarity threshold = {}".format(similarity_threshold))
        self.custom_bar_plot(ax, robot_ids, dissimilarities, label="Number of non-similar paths",
                         bottom=similarities, color='r', xlabel="Robot ID", ylabel="Number of similar/non-similar paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+1, 1),
                         title="Number of predictable paths with similarity threshold = {}".format(similarity_threshold))

        plt.savefig("fleet_path_predictability.svg", format='svg')

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
    ap.plot_path_predictability_stats()
    ap.plot_fleet_planning_times()

if __name__ == "__main__":
    main()
