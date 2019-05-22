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
    def __init__(self, planning_df, execution_df):
        self.plans = None
        self.average_path_planning_time = None
        self.average_path_simplification_time = None
        self.total_path_planning_time = None
        self.total_path_simplification_time = None
        self.total_planning_time = None
        self.is_holonomic = None
        self.complete_path = None
        self.nPlans_from_recall = None
        self.nSuccessful_mission_executions = None
        self.mission_execution_durations = None
        self.total_execution_time = None

        self.fill_data(planning_df, execution_df)

    def mission_execution_successful(self):
        return self.nSuccessful_mission_executions == 3

    def fill_data(self, planning_df, execution_df):
        self.plans = []
        self.average_path_planning_time = 0
        self.average_path_simplification_time = 0
        self.total_path_planning_time = 0
        self.total_path_simplification_time = 0
        self.total_planning_time = 0
        self.nPlans_from_recall = 0

        plan_num = 0
        for start_time in planning_df["Planning Start Time"].values:
            plan_df = planning_df.loc[planning_df["Planning Start Time"] == start_time]
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

        # load execution infromation
        self.nSuccessful_mission_executions = execution_df["Successful Misions"]
        self.mission_execution_durations = np.zeros(3)
        for i in range(3):
            col = "Mission"+ str(i+1) + " Duration"
            value = execution_df[col]
            if value != "x":
                self.mission_execution_durations[i] = value
        self.total_execution_time = np.sum(self.mission_execution_durations)

class FleetMissionData:
    def __init__(self, planning_df, execution_df):
        self.robot_missions = None
        self.total_planning_time = None
        self.total_path_planning_time = None
        self.total_path_simplification_time = None
        self.nPlans_from_recall = None
        self.nPlans_from_scratch = None
        self.map = None
        self.planner = None
        self.nRobots = None
        self.nReplans = None
        self.total_path_execution_time = None

        self.fill_data(planning_df, execution_df)

    def fill_data(self, planning_df, execution_df):
        self.load_robot_missions(planning_df, execution_df)

        self.map = planning_df["Test Name"].values[0]
        self.planner = planning_df["Planner Type"].values[0]
        self.nReplans = execution_df["Num of replans"].values[0]

        self.total_planning_time = 0
        self.total_path_planning_time = 0
        self.total_path_simplification_time = 0
        self.nPlans_from_recall = 0
        self.total_path_execution_time = 0

        for robot_id in range(self.nRobots):
            self.total_planning_time += self.robot_missions[robot_id].total_planning_time
            self.total_path_planning_time += self.robot_missions[robot_id].total_path_planning_time
            self.total_path_simplification_time += self.robot_missions[robot_id].total_path_simplification_time
            self.nPlans_from_recall += self.robot_missions[robot_id].nPlans_from_recall
            self.total_path_execution_time += self.robot_missions[robot_id].total_execution_time

        # self.total_path_planning_time /= self.nRobots
        # self.total_path_simplification_time /= self.nRobots
        self.nPlans_from_scratch = (self.nRobots * 3) - self.nPlans_from_recall

    def load_robot_missions(self, planning_df, execution_df):
        nPlans = planning_df.values.shape[0]

        # Check if number of plans in planning_df is a multiple of 3
        assert (nPlans % 3 == 0), "Expected number of plans in a fleet mission is a multiple of 3"
        self.nRobots = int(nPlans / 3)

        self.robot_missions = []
        for i in range(0, self.nRobots, 1):
            start = i * 3
            end = start + 3
            mission_planning_df = planning_df.iloc[start:end]
            mission_execution_df = execution_df.iloc[i]
            self.robot_missions.append(RobotMissionData(mission_planning_df, mission_execution_df))

        assert self.nRobots == len(self.robot_missions), "Number of missions loaded not equal to number of robots in fleet!!"

    def mission_execution_successful(self):
        for m in self.robot_missions:
            if not m.mission_execution_successful():
                return False
        return True

    def get_percentage_of_mission_success(self):
        nSuccess = 0
        max_success = self.nRobots

        for robot_id in range(self.nRobots):
            if self.robot_missions[robot_id].mission_execution_successful():
                nSuccess += 1

        return nSuccess * 100.0 /max_success, max_success

# A class to extract relevant lines from a complete log file of a test run and generate a CSV logg file
class LogAnalyzer:
    def __init__(self, planning_csv_abs_path, execution_csv_abs_path):
        self.planning_csv_path = planning_csv_abs_path
        self.execution_csv_path = execution_csv_abs_path

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
        planning_df = pd.read_csv(self.planning_csv_path, index_col=None)
        execution_df = pd.read_csv(self.execution_csv_path, index_col=None)
        self.load_fleet_missions(planning_df, execution_df)

    def load_fleet_missions(self, planning_df, execution_df):
        col = "Test Start Time"
        test_start_times = planning_df[col].values.tolist()
        test_start_times = sorted(set(test_start_times))

        self.fleet_missions = []
        for fleet_id, start_time in enumerate(test_start_times):
            fleet_planning_df = planning_df.loc[planning_df[col] == start_time]
            fleet_execution_df = execution_df.loc[execution_df[col] == start_time]
            self.fleet_missions.append(FleetMissionData(fleet_planning_df, fleet_execution_df))
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

        if isinstance(bottom, int) and bottom == 0:
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)

            if xticks is not None:
                ax.set_xticks(xticks)
            if yticks is not None:
                ax.set_yticks(yticks)

            ax.set_title(title)
            ax.grid()
        ax.legend()

    def plot_fleet_planning_times(self, fleets=None):
        if fleets is None:
            fleets = self.fleet_missions

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

        fig = plt.figure(figsize=(15, 15))
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

    def plot_execution_stats(self, fleets=None):
        if fleets is None:
            fleets = self.fleet_missions

        num_replans = np.zeros(len(fleets))
        nsuccessful_robots = np.zeros_like(num_replans)
        execution_times = np.zeros_like(num_replans)
        success_markers = ['X'] * len(fleets)

        for i, f in enumerate(fleets):
            num_replans[i] = f.nReplans
            execution_times[i] = f.total_path_execution_time
            percent, max_robots = f.get_percentage_of_mission_success()
            nsuccessful_robots[i] = percent * max_robots / 100.0
            if np.allclose(percent, 100.0):
                success_markers[i] = "^"

        fleet_ids = np.arange(1, len(fleets)+1, 1)
        nunsuccessful_robots = np.ones_like(nsuccessful_robots)*fleets[0].nRobots - nsuccessful_robots

        fig = plt.figure(figsize=(15, 15))
        ax = fig.add_subplot(221)
        self.custom_line_plot(ax, fleet_ids, execution_times, label="Path execution time",
                         color='r', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=fleet_ids, useLog10Scale=False, avg_line_col='b',
                         title="Path execution times")
        for i, m in enumerate(success_markers):
            ax.scatter(fleet_ids[i], execution_times[i], marker=m, c='g', s=100)

        ax = fig.add_subplot(222)
        self.custom_bar_plot(ax, fleet_ids, num_replans, label="Number of replannings triggered",
                         color='g', xlabel="Fleet ID", ylabel="Count",
                         xticks=fleet_ids, avg_line_col='b',
                         title="Replanning stats")

        ax = fig.add_subplot(223)
        self.custom_bar_plot(ax, fleet_ids, nsuccessful_robots, label="Number of successful robot missions",
                         color='g', xlabel="Fleet ID", ylabel="Count",
                         xticks=fleet_ids, avg_line_col='b',
                         title="Robot mission success stats")
        self.custom_bar_plot(ax, fleet_ids, nunsuccessful_robots, label="Number of unsuccessful robot missions",
                         bottom=nsuccessful_robots, color='r', xlabel="Fleet ID", ylabel="Count",
                         xticks=fleet_ids)

        fig.suptitle("Plan Execution stats \nPlanner:{} Map:{}".format(fleets[0].planner, fleets[0].map))

        plt.savefig("fleet_execution_stats.svg", format='svg')

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
    parser.add_argument("map", type=str, help=" Name of the map (Ex. BRSU_Floor0)")
    parser.add_argument("planner", type=int, help="ID of the planner (SIMPLE:0, LIGHTNING:1, THUNDER:2)")
    args = parser.parse_args()

    planner_names = ["simple", "lightning", "thunder"]
    planning_csv_filename = args.map + "_" + planner_names[args.planner] + "_planning.csv"
    execution_csv_filename = args.map + "_" + planner_names[args.planner] + "_execution.csv"
    log_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/experienceLogs/")
    planning_csv_filename = os.path.abspath(os.path.join(log_dir, planning_csv_filename))
    execution_csv_filename = os.path.abspath(os.path.join(log_dir, execution_csv_filename))

    if not os.path.isfile(planning_csv_filename):
        print("Planning CSV log file does not exist! \nPath specified was:\n", planning_csv_filename)
        return

    if not os.path.isfile(execution_csv_filename):
        print("Execution CSV log file does not exist! \nPath specified was:\n", execution_csv_filename)
        return

    la = LogAnalyzer(planning_csv_filename, execution_csv_filename)
    la.plot_path_predictability_stats()
    la.plot_fleet_planning_times()
    la.plot_execution_stats()

if __name__ == "__main__":
    main()
