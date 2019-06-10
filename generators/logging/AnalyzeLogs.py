#!/usr/bin/env python
# coding: utf-8

import numpy as np
import os
import errno
import sys
import argparse
import pandas as pd
import re
import matplotlib.pyplot as plt
from matplotlib.transforms import Bbox
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
        self.path_length = None
        self.optimal_path_length = None

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
        self.path_length = df["Path Length"].values[0]

    def load_path(self, path_data):
        path_data = path_data.strip()
        values = [i for i in path_data.split(';')][:-1]
        values = np.array([float(i) for i in values])
        nPoses = int(values.size / 3)
        return values.reshape((nPoses, 3))

    def set_optimal_path_length(self, length):
        self.optimal_path_length = length

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
        self.complete_path_length = None
        self.complete_optimal_path_length = None
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
        self.complete_path_length = 0
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
            self.complete_path_length += plan_data.path_length
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

    def set_optimal_path_lengths(self, lengths):
        assert lengths.size == len(self.plans), "Number of optimal path costs not equal to number of robot mission plans"

        for i, p in enumerate(self.plans):
            p.set_optimal_path_length(lengths[i])
        self.complete_optimal_path_length = np.sum(lengths)

class FleetMissionData:
    def __init__(self, planning_df, execution_df, nExperiences):
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
        self.nExperiences = nExperiences

        self.fill_data(planning_df, execution_df)
        self.load_optimal_path_lengths()

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

    def load_optimal_path_lengths(self):
        directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/testingData/Optimality/")
        filename = self.map + "-" + str(self.nRobots) + "Problems.txt"
        filepath = os.path.join(directory, filename)
        assert os.path.isfile(filepath), "Optimal path cost file ({}) not found!".format(filename)

        optimal_costs = np.loadtxt(filepath, delimiter='\t')
        assert optimal_costs.size == (3 * self.nRobots), "Number of optimal path costs not equal to total number of robot plans"

        for i in range(self.nRobots):
            start = 3 * i
            self.robot_missions[i].set_optimal_path_lengths(optimal_costs[start:start+3])

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

    def get_mission_execution_stats(self):
        stats = np.zeros(4)
        for m in self.robot_missions:
            nSuccess = m.nSuccessful_mission_executions
            stats[nSuccess] += 1
        return stats

    def get_holonomic(self, robot_id=0):
        return self.robot_missions[robot_id].is_holonomic

# A class to extract relevant lines from a complete log file of a test run and generate a CSV logg file
class LogAnalyzer:
    def __init__(self, planning_csv_abs_path, execution_csv_abs_path, nExperiences):
        self.planning_csv_path = planning_csv_abs_path
        self.execution_csv_path = execution_csv_abs_path

        self.test_df = None
        self.plan_stats = None
        self.nTests = None
        self.cdll = None
        self.fleet_missions = None
        self.nSimilar_paths = None
        self.save_path = None
        self.nExperiences = nExperiences

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
            self.fleet_missions.append(FleetMissionData(fleet_planning_df, fleet_execution_df, self.nExperiences))
        print("Loaded", len(self.fleet_missions), "fleet missions")

    def get_map_name(self, fleets):
        return fleets[0].map

    def get_planner_name(self, fleets):
        return fleets[0].planner

    def get_nRobots(self, fleets):
        return fleets[0].nRobots

    def get_holonomic(self, fleets):
        return fleets[0].get_holonomic()

    def reject_outliers(self, data, m=2):
        return data[abs(data - np.mean(data)) < m * np.std(data)]

    def clean_mean(self, data, outlier_threshold=2):
        clean_data = self.reject_outliers(np.array(data), outlier_threshold)
        if clean_data.size > 0:
            return np.mean(clean_data)
        else:
            return np.mean(data)

    def custom_line_plot(self, ax, x, y, label=None, color=None, linestyle=None,
                    linewidth=None, xlabel=None, ylabel=None, title=None,
                    xticks=None, yticks=None, useLog10Scale=False, avg_line_col=None, avg_text_color='black'):
        if useLog10Scale:
            y = np.log10(y)
            ylabel = ylabel + " ($log_{10}$ scale)"
        ax.plot(x, y, label=label, color=color, linestyle=linestyle, linewidth=linewidth)

        if avg_line_col is not None:
            y_mean = np.ones_like(y) * self.clean_mean(y)
            ax.plot(x, y_mean, label="Mean " + label, color=avg_line_col)
            ax.text(x[0], y_mean[0], str(np.round(y_mean[0], decimals=3)), color=avg_text_color, fontweight='bold', horizontalalignment='left', verticalalignment='bottom')

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
                        xticks=None, yticks=None, avg_line_col=None,
                        avg_text_color='black', value_color=None):
        ax.bar(x, height, label=label, color=color, width=barwidth, bottom=bottom)

        if value_color is not None:
            for i, h in enumerate(height):
                ax.text(x[i], h, str(h), color=value_color, fontweight='bold', horizontalalignment='center')
        if avg_line_col is not None:
            mean_height = np.ones_like(height) * self.clean_mean(height)
            ax.plot(x, mean_height, label="Mean " + label, color=avg_line_col, linewidth=3.0)
            ax.text(x[0], mean_height[0], str(np.round(mean_height[0], decimals=3)), color=avg_text_color, fontweight='bold', horizontalalignment='left', verticalalignment='bottom')

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

    def get_figure_title(self, prefix, fleets, assisted_sampling):
        sampling_name = "UsingHotspots" if assisted_sampling else "Uniform"
        kinematics = "Holonomic" if self.get_holonomic(fleets) else "ReedsSheep"
        title = prefix + "\nPlanner:{}, Map:{}, Num of Robots:{}, Kinematics:{}, Sampling Strategy:{}".format(
                    self.get_planner_name(fleets), self.get_map_name(fleets), self.get_nRobots(fleets),
                    kinematics, sampling_name)
        return title

    def subplot_extent(self, fig, ax, pad=0.0):
        """Get the full extent of an axes, including axes labels, tick labels, and
        titles."""
        # For text objects, we need to draw the figure first, otherwise the extents
        # are undefined.
        ax.figure.canvas.draw()
        items = ax.get_xticklabels() + ax.get_yticklabels() 
        items += [ax, ax.title, ax.xaxis.label, ax.yaxis.label]
        items += [ax, ax.title]
        bbox = Bbox.union([item.get_window_extent() for item in items])

        return bbox.expanded(1.0 + pad, 1.0 + pad).transformed(fig.dpi_scale_trans.inverted())

    def plot_fleet_planning_times(self, assisted_sampling, fleets=None):
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

        fig.suptitle(self.get_figure_title("Planning time stats", fleets, assisted_sampling))

        plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "planning_times.svg")
        plt.savefig(plot_name, format='svg')

    def plot_execution_stats(self, assisted_sampling, fleets=None):
        if fleets is None:
            fleets = self.fleet_missions

        num_replans = np.zeros(len(fleets))
        execution_times = np.zeros_like(num_replans)
        success_markers = ['X'] * len(fleets)
        success_status = np.zeros((len(fleets), 4))
        fleet_ids = np.arange(1, len(fleets)+1, 1)

        for i, f in enumerate(fleets):
            num_replans[i] = f.nReplans
            execution_times[i] = f.total_path_execution_time
            percent, max_robots = f.get_percentage_of_mission_success()
            if np.allclose(percent, 100.0):
                success_markers[i] = "^"
            exec_status = f.get_mission_execution_stats()
            for num_success in range(4):
                success_status[i, num_success] = exec_status[num_success]

        fig = plt.figure(figsize=(15, 15))
        ax = fig.add_subplot(221)
        self.custom_line_plot(ax, fleet_ids, execution_times, label="Path execution time",
                         color='r', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=fleet_ids, useLog10Scale=False, avg_line_col='b',
                         title="Path execution times")
        for i, m in enumerate(success_markers):
            ax.scatter(fleet_ids[i], execution_times[i], marker=m, c='g', s=100)

        ax = fig.add_subplot(222)
        self.custom_bar_plot(ax, fleet_ids, num_replans, label="",
                         color='g', xlabel="Fleet ID", ylabel="Number of replannings triggered",
                         xticks=fleet_ids, avg_line_col='b',
                         title="Replanning stats")

        all_success = success_status[:, 3]
        one_failure = success_status[:, 2]
        two_failure = success_status[:, 1]
        all_failure = success_status[:, 0]

        ax = fig.add_subplot(223)
        self.custom_bar_plot(ax, fleet_ids, all_success, label="All missions successful",
                         color=(0, 1, 0), xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids, avg_line_col='b',
                         title="Robot mission execution status per fleet")
        self.custom_bar_plot(ax, fleet_ids, one_failure, label="Failed after delivering Mobidik",
                         bottom=all_success, color='yellow', xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids)
        self.custom_bar_plot(ax, fleet_ids, two_failure, label="Failed while transporting Mobidik",
                         bottom=one_failure+all_success, color='orange', xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids)
        self.custom_bar_plot(ax, fleet_ids, all_failure, label="Failed before reaching Mobidik",
                         bottom=two_failure+one_failure+all_success, color=(1, 0, 0), xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids)

        ax = fig.add_subplot(224)
        nSuccess = np.arange(0, 4, 1)
        nSuccessful_robots = np.sum(success_status, axis=0)

        self.custom_bar_plot(ax, nSuccess, nSuccessful_robots, label=None,
                         color='g', xlabel="Number of successful missions", ylabel="Number of robots",
                         xticks=nSuccess, title="Robot mission execution status over all fleets",
                         value_color='black')

        fig.suptitle(self.get_figure_title("Plan Execution stats", fleets, assisted_sampling))

        plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "execution_stats.svg")
        plt.savefig(plot_name, format='svg')

    def determine_num_similar_paths(self, fleets, similarity_threshold=0.25):
        print("Checking similarity of paths. This may take some time...")
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

    def plot_path_predictability_stats(self, assisted_sampling, fleets=None, similarity_threshold = 0.3):
        if fleets is None:
            fleets = self.fleet_missions
        similarities, nTests = self.determine_num_similar_paths(fleets, similarity_threshold=similarity_threshold)
        dissimilarities = (np.ones_like(similarities) * nTests) - similarities
        robot_ids = np.arange(1, similarities.size+1, 1)

        fig = plt.figure(figsize=(15, 15))
        ax1 = fig.add_subplot(221)
        self.custom_bar_plot(ax1, robot_ids, similarities, label='Number of similar paths',
                         color='g', xlabel="Robot ID", ylabel="Number of similar/non-similar paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+1, 1), avg_line_col='b',
                         title="Number of predictable paths with similarity threshold = {}".format(similarity_threshold))
        self.custom_bar_plot(ax1, robot_ids, dissimilarities, label="Number of non-similar paths",
                         bottom=similarities, color='r', xlabel="Robot ID", ylabel="Number of similar/non-similar paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+1, 1),
                         title="Number of predictable paths with similarity threshold = {}".format(similarity_threshold))

        # Get the path lengths of all the robot missions in all fleet trials
        path_lengths = np.zeros((len(fleets), fleets[0].nRobots))
        optimal_path_lengths = np.zeros_like(path_lengths)
        for i, f in enumerate(fleets):
            for j, m in enumerate(f.robot_missions):
                path_lengths[i, j] = m.complete_path_length
                optimal_path_lengths[i, j] = m.complete_optimal_path_length

        ax2 = fig.add_subplot(222)
        fleet_ids = np.arange(1, path_lengths.shape[0] + 1, 1)
        for id in range(path_lengths.shape[1]):
            robot_path_lengths = path_lengths[:, id]
            # TODO: Although the units seems to be correct in meters, how do we describe the rotation since that is also included in the path length?
            self.custom_line_plot(ax2, fleet_ids, robot_path_lengths, label="Robot {}".format(id+1),
                                  xlabel="Fleet ID", ylabel="Length measure",
                                  xticks=fleet_ids, useLog10Scale=False,
                                  title="Path Lengths")

        ax3 = fig.add_subplot(223)
        for id in range(path_lengths.shape[1]):
            robot_optimality_ratio = np.clip(optimal_path_lengths[:, id] / path_lengths[:, id], 0.0, 1.0)
            self.custom_line_plot(ax3, fleet_ids, robot_optimality_ratio, label="Robot {}".format(id+1),
                                  xlabel="Fleet ID", ylabel="Optimality ratio",
                                  xticks=fleet_ids, useLog10Scale=False,
                                  title="Path Optimality")

        fig.suptitle(self.get_figure_title("Plan stats", fleets, assisted_sampling))

        plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "path_predictability.svg")
        plt.savefig(plot_name, format='svg')
        # fig.savefig(os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "PathLengths.svg"), bbox_inches=self.subplot_extent(fig, ax2))

    def get_directory_to_save_plots(self, fleets, assisted_sampling):
        if self.save_path is None:
            sampling_name = "UsingHotspots" if assisted_sampling else "Uniform"
            kinematics = "Holonomic" if self.get_holonomic(fleets) else "ReedsSheep"
            self.save_path = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/executionData/")
            self.save_path = os.path.join(self.save_path, self.get_map_name(fleets))
            self.save_path = os.path.join(self.save_path, self.get_planner_name(fleets))
            self.save_path = os.path.join(self.save_path, str(self.get_nRobots(fleets))+"_Robots")
            self.save_path = os.path.join(self.save_path, kinematics)
            self.save_path = os.path.join(self.save_path, sampling_name)
            self.save_path = os.path.join(self.save_path, str(self.nExperiences)+"_TrainingExperiences/Plots")

        # Make the directory if it does not exist
        try:
            os.makedirs(self.save_path)
        except OSError as exc:
            if exc.errno ==errno.EEXIST and os.path.isdir(self.save_path):
                pass
            else:
                raise "Could not create directory to save plots at {}".format(self.save_path)

        return self.save_path

def get_log_dir(args):
    sampling_name = "Uniform" if args.no_hotspots else "UsingHotspots"
    kinematics = "ReedsSheep" if args.constrained else "Holonomic"
    planner_names = ["SIMPLE(RRT-Connect)", "Lightning", "Thunder", "EGraphs", "SIMPLE(RRT-Star)"]
    directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/executionData/")
    directory = os.path.join(directory, args.map)
    directory = os.path.join(directory, planner_names[args.planner])
    directory = os.path.join(directory, str(args.nRobots)+"_Robots")
    directory = os.path.join(directory, kinematics)
    directory = os.path.join(directory, sampling_name)
    directory = os.path.join(directory, str(args.nExperiences)+"_TrainingExperiences/Logs")

    return directory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", type=str, help=" Name of the map (Ex. BRSU_Floor0)")
    parser.add_argument("planner", type=int, help="ID of the planner (SIMPLE_RRT-Connect:0, LIGHTNING:1, THUNDER:2, SIMPLE_RRT-Star:3)")
    parser.add_argument("--nRobots", type=int, help="Number of robots to be used in the testing. Default: 3", default=3)
    parser.add_argument("--constrained", type=bool, help="Indicate if the robots are ReedsSheep like vehicles. Default: False (holonomic)", default=False)
    parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience databases are generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    parser.add_argument("--nExperiences", type=int, help="Number of training problems used to build the experience DB. Default: 100", default=100)
    args = parser.parse_args()

    planner_names = ["rrt_connect", "lightning", "thunder", "rrt_star"]
    planning_csv_filename = "Planning.csv"
    execution_csv_filename = "Execution.csv"
    log_dir = get_log_dir(args)
    planning_csv_filename = os.path.abspath(os.path.join(log_dir, planning_csv_filename))
    execution_csv_filename = os.path.abspath(os.path.join(log_dir, execution_csv_filename))

    if not os.path.isfile(planning_csv_filename):
        print("Planning CSV log file does not exist! \nPath specified was:\n", planning_csv_filename)
        return

    if not os.path.isfile(execution_csv_filename):
        print("Execution CSV log file does not exist! \nPath specified was:\n", execution_csv_filename)
        return

    assisted_sampling = not args.no_hotspots

    la = LogAnalyzer(planning_csv_filename, execution_csv_filename, args.nExperiences)
    la.plot_path_predictability_stats(assisted_sampling, similarity_threshold=0.4)
    la.plot_fleet_planning_times(assisted_sampling)
    la.plot_execution_stats(assisted_sampling)

if __name__ == "__main__":
    main()
