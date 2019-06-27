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
import seaborn as sns
sns.set(style="darkgrid")

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

    def load_optimal_path_lengths(self, planner_name="ARA-Star"):
        directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/testingData/Optimality/")
        directory = os.path.join(directory, planner_name)
        directory = os.path.join(directory, self.map + "-" + str(self.nRobots) + "Problems")
        files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
        costs = []
        paths = []
        # Recontruct file names to avoid sorting problems for filenames greater than 9
        for i in range(len(files)):
            files[i] = "Path" + str(i+1) + ".txt"
            filepath = os.path.join(directory, files[i])
            loaded_data = np.loadtxt(filepath, delimiter='\t')
            costs.append(loaded_data[0, 0])
            paths.append(loaded_data[1:, :])

        optimal_costs = np.array(costs)
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

    def get_highest_robot_mission_execution_time(self):
        return max([rm.total_execution_time for rm in self.robot_missions])

class DWT:
    def __init__(self):
        self.load_native_library()

    def load_native_library(self):
        self.cdll = cdll.LoadLibrary('libcomparePaths.so')
        self.cdll.comparePaths.arguments = [POINTER(PathPose), c_int, POINTER(PathPose), c_int, c_bool, c_double, c_double]
        self.cdll.comparePaths.restype = c_bool

    def compare_paths(self, path1, path2, is_holonomic, similarity_threshold):
        # Use DWT to compare two paths
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

    def determine_num_similar_paths(self, fleets, similarity_threshold=0.25):
        print("Checking similarity of paths. This may take some time...")
        max_num_matches = (len(fleets) -1) # -1 because we dont test a path against itself

        similarity_count = np.zeros(fleets[0].nRobots)
        for robot_id in range(fleets[0].nRobots):
            max_matches = 0
            similarity_matrix = np.array([[False] * len(fleets)] * len(fleets))

            for fleet_id_1 in range(len(fleets)):

                for fleet_id_2 in range(fleet_id_1+1, len(fleets), 1):
                    if fleet_id_1 != fleet_id_2:
                        path1 = fleets[fleet_id_1].robot_missions[robot_id].complete_path
                        path2 = fleets[fleet_id_2].robot_missions[robot_id].complete_path
                        is_holonomic = fleets[fleet_id_2].robot_missions[robot_id].is_holonomic
                        if self.compare_paths(path1, path2, is_holonomic, similarity_threshold):
                            # Update the symmetric elements of the matrix
                            similarity_matrix[fleet_id_1, fleet_id_2] = True
                            similarity_matrix[fleet_id_2, fleet_id_1] = True

                num_matches = similarity_matrix[fleet_id_1].tolist().count(True)
                if num_matches > max_matches:
                    max_matches = num_matches

                if max_matches >= max_num_matches:
                    # Early stop since max matches found
                    break

            similarity_count[robot_id] = max_matches
            print("\tNumber of similar paths for Robot", robot_id+1, "=", similarity_count[robot_id])

        print("Path similarity tests complete!")
        return similarity_count, max_num_matches

class PlotUtils:
    def reject_outliers(self, data, m=2, force=False):
        if not force and data.size <= 20:
            return data

        return data[abs(data - np.mean(data)) < m * np.std(data)]

    def clean_mean(self, data, outlier_threshold=2, force_outlier_removal=False):
        clean_data = self.reject_outliers(np.array(data), outlier_threshold, force_outlier_removal)
        if clean_data.size > 0:
            return np.mean(clean_data)
        else:
            return np.mean(data)

    def custom_line_plot(self, ax, x, y, label=None, color=None, linestyle=None,
                    linewidth=None, xlabel=None, ylabel=None, title=None,
                    xticks=None, yticks=None, useLog10Scale=False, avg_line_col=None,
                    avg_text_color='black', avg_line_style='--'):
        if useLog10Scale:
            y = np.log10(y)
            ylabel = ylabel + " ($log_{10}$ scale)"
        ax.plot(x, y, label=label, color=color, linestyle=linestyle, linewidth=linewidth)

        if avg_line_col is not None:
            y_mean = np.ones_like(y) * self.clean_mean(y)
            ax.plot(x, y_mean, label="Mean " + label, color=avg_line_col, linestyle=avg_line_style)
            ax.text(x[0], y_mean[0], str(np.round(y_mean[0], decimals=3)), color=avg_text_color, fontweight='bold', horizontalalignment='left', verticalalignment='bottom')

        if xticks is not None:
            ax.set_xticks(xticks)
        if yticks is not None:
            ax.set_yticks(yticks)

        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True)
        ax.legend()

    def custom_bar_plot(self, ax, x, height, label=None, color=None, barwidth=0.8,
                        bottom=0, xlabel=None, ylabel=None, title=None,
                        xticks=None, yticks=None, avg_line_col=None,
                        avg_text_color='black', avg_line_style='--', value_color=None, value=None):
        ax.bar(x, height, label=label, color=color, width=barwidth, bottom=bottom)

        if value_color is not None:
            for i, h in enumerate(height):
                y_pos = h
                if not isinstance(bottom, int):
                    y_pos += bottom[i]
                if value is None and h > 0:
                    value = str(h)

                if value is not None:
                    ax.text(x[i], y_pos, value[i], color=value_color, fontweight='bold', horizontalalignment='center', verticalalignment='top')
        if avg_line_col is not None:
            mean_height = np.ones_like(height) * self.clean_mean(height)
            ax.plot(x, mean_height, label="Mean " + label, color=avg_line_col, linestyle=avg_line_style, linewidth=3.0)
            ax.text(x[0], mean_height[0], str(np.round(mean_height[0], decimals=3)), color=avg_text_color, fontweight='bold', horizontalalignment='left', verticalalignment='bottom')

        if isinstance(bottom, int) and bottom == 0:
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)

            if xticks is not None:
                ax.set_xticks(xticks)
            if yticks is not None:
                ax.set_yticks(yticks)

            ax.set_title(title)
            ax.grid(True)
        ax.legend()

    def autopct_func(self, pct):
        return ('%.1f%%' % pct) if pct >= 1 else ''


    def custom_pie_plot(self, ax, values, labels=None, title=None, startangle=90):
        ax.pie(values, autopct=self.autopct_func, shadow=True, startangle=startangle)

        ax.set_title(title)
        ax.legend(labels)

    def custom_box_plot(self, ax, x_vals, data, color=None,
                        xlabel=None, ylabel=None, title=None, horizontal=False):
        x = []
        y = []
        for i in range(data.shape[1]):
            x.extend([x_vals[i]] * data.shape[0])
            y.extend(data[:, i])
            
        if horizontal:
            sns.boxplot(ax=ax, x=y, y=x, orient='h')
        else:
            sns.boxplot(ax=ax, x=x, y=y, orient='v')
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)

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
