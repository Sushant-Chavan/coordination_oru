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
from Utils import *

class LogAnalyzer:
    def __init__(self, planning_csv_abs_path, execution_csv_abs_path, nExperiences):
        self.planning_csv_path = planning_csv_abs_path
        self.execution_csv_path = execution_csv_abs_path

        self.nTests = None
        self.fleet_missions = None
        self.save_path = None
        self.nExperiences = nExperiences

        self.dwt = DWT()
        self.plot_utils = PlotUtils()

        self.load_csv()

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

    def get_figure_title(self, prefix, fleets, assisted_sampling):
        sampling_name = "UsingHotspots" if assisted_sampling else "Uniform"
        kinematics = "Holonomic" if self.get_holonomic(fleets) else "ReedsSheep"
        title = prefix + "\nPlanner:{}, Map:{}, Num of Robots:{}, Kinematics:{}, Sampling Strategy:{}".format(
                    self.get_planner_name(fleets), self.get_map_name(fleets), self.get_nRobots(fleets),
                    kinematics, sampling_name)
        return title

    def plot_fleet_planning_times(self, assisted_sampling, fleets=None):
        if fleets is None:
            fleets = self.fleet_missions

        nFleets = len(fleets)
        fleet_ids = np.arange(1, nFleets + 1, 1)
        xticks = np.arange(0, len(fleets)+1, 5)
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

        fig = plt.figure(figsize=(15, 7.5))

        # Plot planning times
        ax = fig.add_subplot(121)
        self.plot_utils.custom_line_plot(ax, fleet_ids, path_planning_times, label="Path planning",
                         color='r', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=xticks, useLog10Scale=False, avg_line_col='r', title="Planning time")

        self.plot_utils.custom_line_plot(ax, fleet_ids, path_simpl_times, label="Path simplification",
                         color='b', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=xticks, useLog10Scale=False, avg_line_col='b', title="Planning time")

        self.plot_utils.custom_line_plot(ax, fleet_ids, total_plan_times, label="Total planning time",
                         color='g', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=xticks, useLog10Scale=False, avg_line_col='g', title="Planning time")

        # Plot Recall statistics
        ax = fig.add_subplot(122)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, nPlans_from_recall, label="Recall",
                         color='g', xlabel="Fleet ID", ylabel="Number of plans",
                         xticks=xticks, title="Plans generated from recall/scratch")
        self.plot_utils.custom_bar_plot(ax, fleet_ids, nPlans_from_scratch, label="Scratch",
                         bottom=nPlans_from_recall, color='r', xlabel="Fleet ID", ylabel="Number of plans",
                         xticks=xticks, title="Plans generated from recall/scratch")

        fig.suptitle(self.get_figure_title("Planning time stats", fleets, assisted_sampling))

        plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "planning_times.svg")
        plt.savefig(plot_name, format='svg')

    def plot_execution_stats(self, assisted_sampling, fleets=None):
        if fleets is None:
            fleets = self.fleet_missions

        num_replans = np.zeros(len(fleets))
        max_execution_times = np.zeros_like(num_replans)
        success_markers = ['X'] * len(fleets)
        success_status = np.zeros((len(fleets), 4))
        fleet_ids = np.arange(1, len(fleets)+1, 1)
        xticks = np.arange(0, len(fleets)+1, 5)

        for i, f in enumerate(fleets):
            num_replans[i] = f.nReplans
            max_execution_times[i] = f.get_highest_robot_mission_execution_time()
            percent, max_robots = f.get_percentage_of_mission_success()
            if np.allclose(percent, 100.0):
                success_markers[i] = "^"
            exec_status = f.get_mission_execution_stats()
            for num_success in range(4):
                success_status[i, num_success] = exec_status[num_success]

        fig = plt.figure(figsize=(15, 15))
        ax = fig.add_subplot(221)
        self.plot_utils.custom_line_plot(ax, fleet_ids, max_execution_times, label="Execution time",
                         color='r', xlabel="Fleet ID", ylabel="Time in seconds",
                         xticks=xticks, useLog10Scale=False, avg_line_col='b',
                         title="Complete fleet mission execution time")
        for i, m in enumerate(success_markers):
            ax.scatter(fleet_ids[i], max_execution_times[i], marker=m, c='g', s=100)

        ax = fig.add_subplot(222)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, num_replans, label="",
                         color='g', xlabel="Fleet ID", ylabel="Number of replannings triggered",
                         xticks=xticks, avg_line_col='b',
                         title="Replanning stats")

        all_success = success_status[:, 3]
        one_failure = success_status[:, 2]
        two_failure = success_status[:, 1]
        all_failure = success_status[:, 0]

        colors=[(0, 0.75, 0), 'yellow', 'orange', (0.75, 0, 0)]

        ax = fig.add_subplot(223)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, all_success, label="Successful execution",
                         color=colors[0], xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=xticks, title="Robot mission execution status per fleet")
        self.plot_utils.custom_bar_plot(ax, fleet_ids, one_failure, label="Failed after delivering Mobidik",
                         bottom=all_success, color=colors[1], xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=xticks)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, two_failure, label="Failed while transporting Mobidik",
                         bottom=one_failure+all_success, color=colors[2], xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=xticks)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, all_failure, label="Failed before reaching Mobidik",
                         bottom=two_failure+one_failure+all_success, color=colors[3], xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=xticks)

        ax = fig.add_subplot(224)
        nSuccess = np.arange(0, 4, 1)
        nSuccessful_robots = np.sum(success_status, axis=0)

        labels=["Failed before reaching Mobidik", "Failed while transporting Mobidik",
                "Failed while returning to charging station", "Successful execution"]
        self.plot_utils.custom_pie_plot(ax, nSuccessful_robots[::-1], labels=labels[::-1], title="Robot mission execution status over all fleets",
                                        colors=colors)

        fig.suptitle(self.get_figure_title("Plan Execution stats", fleets, assisted_sampling))

        plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "execution_stats.svg")
        plt.savefig(plot_name, format='svg')

    def plot_predictability_subplot(self, ax, similarity_threshold, fleets, assisted_sampling):
        similarities, nTests = self.dwt.determine_num_similar_paths(fleets, assisted_sampling, similarity_threshold=similarity_threshold)
        dissimilarities = (np.ones_like(similarities) * nTests) - similarities
        robot_ids = np.arange(1, similarities.size+1, 1)

        self.plot_utils.custom_bar_plot(ax, robot_ids, similarities, label='Similar paths',
                         color='g', xlabel="Robot ID", ylabel="Number of paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+2, 2),
                         title="Similarity between paths with similarity threshold = {}".format(similarity_threshold))
        self.plot_utils.custom_bar_plot(ax, robot_ids, dissimilarities, label="Non-similar paths",
                         bottom=similarities, color='r', xlabel="Robot ID", ylabel="Number paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+2, 2))

    def plot_path_predictability_stats(self, assisted_sampling, fleets=None, similarity_threshold = 0.3):
        if fleets is None:
            fleets = self.fleet_missions
        thresholds = [similarity_threshold, np.round(similarity_threshold-0.1, 2), np.round(similarity_threshold+0.1, 2)]
        fig = plt.figure(figsize=(15, 15))

        ax1 = fig.add_subplot(221)
        self.plot_predictability_subplot(ax1, thresholds[0], fleets, assisted_sampling)

        # Get the path lengths of all the robot missions in all fleet trials
        path_lengths = np.zeros((len(fleets), fleets[0].nRobots))
        optimal_path_lengths = np.zeros_like(path_lengths)
        for i, f in enumerate(fleets):
            for j, m in enumerate(f.robot_missions):
                path_lengths[i, j] = m.complete_path_length
                optimal_path_lengths[i, j] = m.complete_optimal_path_length

        ax2 = fig.add_subplot(222)
        fleet_ids = np.arange(1, path_lengths.shape[0] + 1, 1)
        robot_suboptimality = None
        for id in range(path_lengths.shape[1]):
            if robot_suboptimality is None:
                robot_suboptimality = np.clip(path_lengths[:, id] / optimal_path_lengths[:, id], 1.0, 100.0)
            else:
                robot_suboptimality = np.vstack((robot_suboptimality, np.clip(path_lengths[:, id] / optimal_path_lengths[:, id], 1.0, 100.0)))

        robot_suboptimality = robot_suboptimality.T
        robot_ids = np.arange(1, robot_suboptimality.shape[1]+1)
        # self.plot_utils.custom_box_plot(ax2, robot_ids, robot_suboptimality,
        #                                 xlabel="Robot ID", ylabel="Suboptimality ratio", title="Path Suboptimality")

        colors = ['r', 'g', 'b', 'k', 'orange', 'yellow', 'magenta', 'purple', 'cyan', 'pink']
        ticks = np.arange(0, path_lengths.shape[0] + 5, 5) if path_lengths.shape[0] > 20 else fleet_ids
        for id in range(path_lengths.shape[1]):
            c = colors[id] if id < len(colors) else plt.cm.jet(id/path_lengths.shape[1])
            robot_path_lengths = path_lengths[:, id]
            # TODO: Although the units seems to be correct in meters, how do we describe the rotation since that is also included in the path length?
            self.plot_utils.custom_line_plot(ax2, fleet_ids, robot_path_lengths, label="Robot {}".format(id+1),
                                  color=c, xlabel="Iteration number", ylabel="Length measure",
                                  xticks=ticks, useLog10Scale=False,
                                  title="Path Lengths")
            mean = self.plot_utils.clean_mean(robot_path_lengths, outlier_threshold=2, force_outlier_removal=True)
            var = np.var(robot_path_lengths)
            ax2.axhline(y=mean + 2, linestyle=':', color=c)
            ax2.axhline(y=mean - 2, linestyle=':', color=c)

        ax3 = fig.add_subplot(223)
        self.plot_predictability_subplot(ax3, thresholds[1], fleets)

        ax4 = fig.add_subplot(224)
        self.plot_predictability_subplot(ax4, thresholds[2], fleets)

        fig.suptitle(self.get_figure_title("Plan stats", fleets, assisted_sampling))

        plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "path_predictability.svg")
        plt.savefig(plot_name, format='svg')
        # fig.savefig(os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "PathLengths.svg"), bbox_inches=self.plot_utils.subplot_extent(fig, ax2))

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", type=str, help=" Name of the map (Ex. BRSU_Floor0)")
    parser.add_argument("planner", type=int, help="ID of the planner (SIMPLE_RRT-Connect:0, LIGHTNING:1, THUNDER:2, SIMPLE_RRT-Star:3)")
    parser.add_argument("--nRobots", type=int, help="Number of robots to be used in the testing. Default: 3", default=3)
    parser.add_argument("--constrained", type=bool, help="Indicate if the robots are ReedsSheep like vehicles. Default: False (holonomic)", default=False)
    parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience databases are generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    parser.add_argument("--nExperiences", type=int, help="Number of training problems used to build the experience DB. Default: 100", default=100)
    args = parser.parse_args()

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
    la.plot_fleet_planning_times(assisted_sampling)
    la.plot_execution_stats(assisted_sampling)
    la.plot_path_predictability_stats(assisted_sampling, similarity_threshold=0.3)

if __name__ == "__main__":
    main()
