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

class DataLoader:
    def __init__(self):
        self.fleet_missions = None

    def get_log_files(self, map_name, planner, nRobots, holonomic, use_hotspots, nExperiences):
        sampling_name = "UsingHotspots" if use_hotspots else "Uniform"
        kinematics = "Holonomic" if holonomic else "ReedsSheep"
        planner_names = ["SIMPLE(RRT-Connect)", "Lightning", "Thunder", "EGraphs", "SIMPLE(RRT-Star)"]
        common_directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/executionData/")
        directory = os.path.join(common_directory, map_name)
        directory = os.path.join(directory, planner_names[planner])
        directory = os.path.join(directory, str(nRobots)+"_Robots")
        directory = os.path.join(directory, kinematics)
        directory = os.path.join(directory, sampling_name)
        directory = os.path.join(directory, str(nExperiences)+"_TrainingExperiences/Logs")

        planning_csv_filename = "Planning.csv"
        execution_csv_filename = "Execution.csv"
        planning_csv_filename = os.path.abspath(os.path.join(directory, planning_csv_filename))
        execution_csv_filename = os.path.abspath(os.path.join(directory, execution_csv_filename))

        assert os.path.isfile(planning_csv_filename), "Planning CSV log file does not exist! \nPath specified was:\n{}".format(planning_csv_filename)
        assert os.path.isfile(execution_csv_filename), "Execution CSV log file does not exist! \nPath specified was:\n{}".format(execution_csv_filename)
        
        relative_dir = os.path.relpath(directory, common_directory)

        return planning_csv_filename, execution_csv_filename, relative_dir

    def load_data(self, map_name="BRSU_Floor0", planner=1, nRobots=5, holonomic=True, use_hotspots=True, nExperiences=100):
        planning_data_file, execution_data_file, rel_dir = self.get_log_files(map_name, planner, nRobots, holonomic, use_hotspots, nExperiences)
        planning_df = pd.read_csv(planning_data_file, index_col=None)
        execution_df = pd.read_csv(execution_data_file, index_col=None)

        fleet_data = self.load_fleet_missions(planning_df, execution_df, nExperiences)
        print("\tLoaded", len(fleet_data), "fleet missions from", rel_dir)
        self.add_loaded_data_to_dict(map_name, planner, nRobots, holonomic, use_hotspots, nExperiences, fleet_data)

    def load_fleet_missions(self, planning_df, execution_df, nExperiences):
        col = "Test Start Time"
        test_start_times = planning_df[col].values.tolist()
        test_start_times = sorted(set(test_start_times))

        fleet_missions = []
        for fleet_id, start_time in enumerate(test_start_times):
            fleet_planning_df = planning_df.loc[planning_df[col] == start_time]
            fleet_execution_df = execution_df.loc[execution_df[col] == start_time]
            fleet_missions.append(FleetMissionData(fleet_planning_df, fleet_execution_df, nExperiences))
        return fleet_missions

    def add_loaded_data_to_dict(self, map_name, planner, nRobots, 
                                holonomic, use_hotspots, nExperiences, fleet_data):
        if self.fleet_missions is None:
            self.fleet_missions = {map_name:{planner:{nRobots:{holonomic:{use_hotspots:{nExperiences:fleet_data}}}}}}
            return
        
        if map_name not in self.fleet_missions.keys():
            self.fleet_missions[map_name] = {planner:{nRobots:{holonomic:{use_hotspots:{nExperiences:fleet_data}}}}}
            return

        if planner not in self.fleet_missions[map_name].keys():
            self.fleet_missions[map_name][planner] = {nRobots:{holonomic:{use_hotspots:{nExperiences:fleet_data}}}}
            return

        if nRobots not in self.fleet_missions[map_name][planner].keys():
            self.fleet_missions[map_name][planner][nRobots] = {holonomic:{use_hotspots:{nExperiences:fleet_data}}}
            return

        if holonomic not in self.fleet_missions[map_name][planner][nRobots].keys():
            self.fleet_missions[map_name][planner][nRobots][holonomic] = {use_hotspots:{nExperiences:fleet_data}}
            return

        if use_hotspots not in self.fleet_missions[map_name][planner][nRobots][holonomic].keys():
            self.fleet_missions[map_name][planner][nRobots][holonomic][use_hotspots] = {nExperiences:fleet_data}
            return

        self.fleet_missions[map_name][planner][nRobots][holonomic][use_hotspots][nExperiences] = fleet_data

    def get_fleet_data(self, map_name="BRSU_Floor0", planner=1, nRobots=5,
                       holonomic=True, use_hotspots=True, nExperiences=100):
        data = self.fleet_missions[map_name][planner][nRobots][holonomic][use_hotspots][nExperiences]

        assert data is not None, "Could not retreive fleet data. Possibly data was not loaded!"

        return data


class MultiLogAnalyzer:
    def __init__(self):
        self.data_loader = DataLoader()
        self.dwt = DWT()
        self.plot_utils = PlotUtils()

    def load_all_fleets(self, maps, planners, nRobots, holonomic, use_hotspots, nExperiences):
        print("Loading all fleets...")
        for m in maps:
            for p in planners:
                for r in nRobots:
                    for h in holonomic:
                        for hp in use_hotspots:
                            for e in nExperiences:
                                self.data_loader.load_data(m, p, r, h, hp, e)
        print("All fleets successfully loaded.")

    def get_variables(self, maps, planners, nRobots, holonomic, use_hotspots, nExperiences):
        # Detect variables
        is_param_variable = []
        is_param_variable.append(len(maps) > 1)
        is_param_variable.append(len(planners) > 1)
        is_param_variable.append(len(nRobots) > 1)
        is_param_variable.append(len(holonomic) > 1)
        is_param_variable.append(len(use_hotspots) > 1)
        is_param_variable.append(len(nExperiences) > 1)

        return is_param_variable

    def get_figure_title(self, prefix, maps, planners, nRobots, holonomic, use_hotspots, nExperiences, is_param_variable):
        title = prefix + "\n"
        if not is_param_variable[0]:
            title += "Map:{}, ".format(maps[0])
        if not is_param_variable[1]:
            title += "Planner:{}, ".format(planners[0])
        if not is_param_variable[2]:
            title += "Num of robots:{}, ".format(nRobots[0])
        if not is_param_variable[3]:
            kinematics = "Holonomic" if holonomic[0] else "ReedsSheep"
            title += "Kinematics:{}, ".format(kinematics)
        if not is_param_variable[4]:
            sampling_name = "UsingHotspots" if use_hotspots[0] else "Uniform"
            title += "Sampling:{}, ".format(sampling_name)
        if not is_param_variable[5]:
            title += "Experience Count:{}, ".format(nExperiences[0])
        return title

    def get_variable_name(self, prefix, params, is_param_variable):
        planner_names = ["SIMPLE(RRT-Connect)", "Lightning", "Thunder", "EGraphs", "SIMPLE(RRT-Star)"]
        variable_name = prefix
        if is_param_variable[0]:
            if len(variable_name) > 0:
                variable_name += "-"
            variable_name += params[0]
        if is_param_variable[1]:
            if len(variable_name) > 0:
                variable_name += "-"
            variable_name += planner_names[params[1]]
        if is_param_variable[2]:
            if len(variable_name) > 0:
                variable_name += "-"
            variable_name += str(params[2]) + "Robots"
        if is_param_variable[3]:
            kinematics = "Holonomic" if params[3] else "ReedsSheep"
            if len(variable_name) > 0:
                variable_name += "-"
            variable_name += kinematics
        if is_param_variable[4]:
            sampling_name = "UsingHotspots" if params[4] else "Uniform"
            if len(variable_name) > 0:
                variable_name += "-"
            variable_name += sampling_name
        if is_param_variable[5]:
            if len(variable_name) > 0:
                variable_name += "-"
            variable_name += str(params[5]) + "Exp"
        return variable_name

    def get_unique_params(self, params):
        params = np.array(params)
        return list(set(params[:, 0])), list(set(params[:, 1])), list(set(params[:, 2])),\
               list(set(params[:, 3])), list(set(params[:, 4])), list(set(params[:, 5]))


    def plot_planning_times(self, params):
        fleets_list = []
        maps, planners, nRobots, holonomic, use_hotspots, nExperiences = self.get_unique_params(params)
        is_param_variable = self.get_variables(maps, planners, nRobots, holonomic, use_hotspots, nExperiences)

        fig = plt.figure(figsize=(15, 7.5))
        ax1 = fig.add_subplot(121)
        ax2 = fig.add_subplot(122)

        for p in params:
            fleets_list.append(self.data_loader.get_fleet_data(p[0], p[1], p[2], p[3], p[4], p[5]))

        fleet_ids = []
        total_plan_times = []
        nPlans_from_recall = []
        nPlans_from_scratch = []
        variable_names = []
        color_ids = []
        for i in range(len(fleets_list)):
            variable_names.append(self.get_variable_name("", params[i], is_param_variable))
            fleets = fleets_list[i]
            color_ids.append(i/len(fleets_list))
            fleet_ids.append(np.arange(1, len(fleets) + 1, 1))
            plan_times = []
            recall = []
            scratch = []

            for f in fleets:
                plan_times.append(f.total_planning_time)
                recall.append(f.nPlans_from_recall)
                scratch.append(f.nPlans_from_scratch)

            total_plan_times.append(plan_times)
            nPlans_from_recall.append(sum(recall))
            nPlans_from_scratch.append(sum(scratch))

        # Plot planning times
        for i in range(len(total_plan_times)):
            self.plot_utils.custom_line_plot(ax1, fleet_ids[i], total_plan_times[i], label=variable_names[i],
                            color=plt.cm.summer(color_ids[i]), xlabel="Fleet ID", ylabel="Time in seconds",
                            useLog10Scale=False, avg_line_col=plt.cm.summer(color_ids[i]), title="Total planning time")

        # Plot recall stats
        recall_percent = np.round(np.array(nPlans_from_recall) / (np.array(nPlans_from_recall) + np.array(nPlans_from_scratch)) * 100.0, 1)
        scratch_percent = 100-recall_percent
        recall_percent = [str(np.round(p, 1))+"%" if p > 0 else None for p in recall_percent.tolist()]
        scratch_percent = [str(np.round(p, 1))+"%" if p > 0 else None for p in scratch_percent.tolist()]
        self.plot_utils.custom_bar_plot(ax2, variable_names, nPlans_from_recall, label="Number of plans from recall",
                        color='g', ylabel="Count", value_color='k', value=recall_percent)
        self.plot_utils.custom_bar_plot(ax2, variable_names, nPlans_from_scratch, label="Number of plans from scratch",
                        bottom=nPlans_from_recall, color='r', ylabel="Count", value_color='k', value=scratch_percent)

        fig.suptitle(self.get_figure_title("Planning time stats", maps, planners, nRobots, holonomic, 
                                            use_hotspots, nExperiences, is_param_variable))

        # plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "planning_times.svg")
        plt.savefig("/home/suvich15/Desktop/PlanningTimes.svg", format='svg')

    def plot_execution_stats(self, assisted_sampling, fleets=None):
        if fleets is None:
            fleets = self.fleet_missions

        num_replans = np.zeros(len(fleets))
        max_execution_times = np.zeros_like(num_replans)
        success_markers = ['X'] * len(fleets)
        success_status = np.zeros((len(fleets), 4))
        fleet_ids = np.arange(1, len(fleets)+1, 1)

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
                         xticks=fleet_ids, useLog10Scale=False, avg_line_col='b',
                         title="Complete fleet mission execution time")
        for i, m in enumerate(success_markers):
            ax.scatter(fleet_ids[i], max_execution_times[i], marker=m, c='g', s=100)

        ax = fig.add_subplot(222)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, num_replans, label="",
                         color='g', xlabel="Fleet ID", ylabel="Number of replannings triggered",
                         xticks=fleet_ids, avg_line_col='b',
                         title="Replanning stats")

        all_success = success_status[:, 3]
        one_failure = success_status[:, 2]
        two_failure = success_status[:, 1]
        all_failure = success_status[:, 0]

        ax = fig.add_subplot(223)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, all_success, label="All missions successful",
                         color=(0, 1, 0), xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids, avg_line_col='b',
                         title="Robot mission execution status per fleet")
        self.plot_utils.custom_bar_plot(ax, fleet_ids, one_failure, label="Failed after delivering Mobidik",
                         bottom=all_success, color='yellow', xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, two_failure, label="Failed while transporting Mobidik",
                         bottom=one_failure+all_success, color='orange', xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids)
        self.plot_utils.custom_bar_plot(ax, fleet_ids, all_failure, label="Failed before reaching Mobidik",
                         bottom=two_failure+one_failure+all_success, color=(1, 0, 0), xlabel="Fleet ID", ylabel="Number of robots",
                         xticks=fleet_ids)

        ax = fig.add_subplot(224)
        nSuccess = np.arange(0, 4, 1)
        nSuccessful_robots = np.sum(success_status, axis=0)

        labels=["Failed before reaching Mobidik", "Failed while transporting Mobidik",
                "Failed while returning to charging station", "All missions successful"]
        self.plot_utils.custom_pie_plot(ax, nSuccessful_robots, labels=labels, title="Robot mission execution status over all fleets")

        fig.suptitle(self.get_figure_title("Plan Execution stats", fleets, assisted_sampling))

        plot_name = os.path.join(self.get_directory_to_save_plots(fleets, assisted_sampling), "execution_stats.svg")
        plt.savefig(plot_name, format='svg')

    def plot_predictability_subplot(self, ax, similarity_threshold, fleets):
        similarities, nTests = self.dwt.determine_num_similar_paths(fleets, similarity_threshold=similarity_threshold)
        dissimilarities = (np.ones_like(similarities) * nTests) - similarities
        robot_ids = np.arange(1, similarities.size+1, 1)

        self.plot_utils.custom_bar_plot(ax, robot_ids, similarities, label='Number of similar paths',
                         color='g', xlabel="Robot ID", ylabel="Number of similar/non-similar paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+1, 1), avg_line_col='b',
                         title="Number of predictable paths with similarity threshold = {}".format(similarity_threshold))
        self.plot_utils.custom_bar_plot(ax, robot_ids, dissimilarities, label="Number of non-similar paths",
                         bottom=similarities, color='r', xlabel="Robot ID", ylabel="Number of similar/non-similar paths",
                         xticks=robot_ids, yticks=np.arange(0, nTests+1, 1))

    def plot_path_predictability_stats(self, assisted_sampling, fleets=None, similarity_threshold = 0.3):
        if fleets is None:
            fleets = self.fleet_missions
        thresholds = [similarity_threshold, np.round(similarity_threshold-0.2, 2), np.round(similarity_threshold+0.2, 2)]
        fig = plt.figure(figsize=(15, 15))

        ax1 = fig.add_subplot(221)
        self.plot_predictability_subplot(ax1, thresholds[0], fleets)

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
        self.plot_utils.custom_box_plot(ax2, robot_ids, robot_suboptimality,
                                        xlabel="Robot ID", ylabel="Suboptimality ratio", title="Path Suboptimality")

        ax3 = fig.add_subplot(223)
        self.plot_predictability_subplot(ax3, thresholds[1], fleets)

        ax4 = fig.add_subplot(224)
        self.plot_predictability_subplot(ax4, thresholds[2], fleets)

        # ax3 = fig.add_subplot(223)
        # for id in range(path_lengths.shape[1]):
        #     robot_path_lengths = path_lengths[:, id]
        #     # TODO: Although the units seems to be correct in meters, how do we describe the rotation since that is also included in the path length?
        #     self.plot_utils.custom_line_plot(ax3, fleet_ids, robot_path_lengths, label="Robot {}".format(id+1),
        #                           xlabel="Fleet ID", ylabel="Length measure",
        #                           xticks=fleet_ids, useLog10Scale=False,
        #                           title="Path Lengths")

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
    # parser = argparse.ArgumentParser()
    # parser.add_argument("map", type=str, help=" Name of the map (Ex. BRSU_Floor0)")
    # parser.add_argument("planner", type=int, help="ID of the planner (SIMPLE_RRT-Connect:0, LIGHTNING:1, THUNDER:2, SIMPLE_RRT-Star:3)")
    # parser.add_argument("--nRobots", type=int, help="Number of robots to be used in the testing. Default: 3", default=3)
    # parser.add_argument("--constrained", type=bool, help="Indicate if the robots are ReedsSheep like vehicles. Default: False (holonomic)", default=False)
    # parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience databases are generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    # parser.add_argument("--nExperiences", type=int, help="Number of training problems used to build the experience DB. Default: 100", default=100)
    # args = parser.parse_args()

    # assisted_sampling = not args.no_hotspots

    mla = MultiLogAnalyzer()
    mla.load_all_fleets(["BRSU_Floor0"], [1, 2], [5], [True], [True], [100])
    mla.load_all_fleets(["BRSU_Floor0"], [3], [5], [True], [True], [25])
    params = [["BRSU_Floor0", 1, 5, True, True, 100],
              ["BRSU_Floor0", 2, 5, True, True, 100],
              ["BRSU_Floor0", 3, 5, True, True, 25]]
    # params = [["BRSU_Floor0", 1, 5, True, True, 100],
    #           ["BRSU_Floor0", 1, 10, True, True, 100]]
    mla.plot_planning_times(params)

    # la = LogAnalyzer(planning_csv_filename, execution_csv_filename, args.nExperiences)
    # la.plot_path_predictability_stats(assisted_sampling, similarity_threshold=0.8)
    # la.plot_fleet_planning_times(assisted_sampling)
    # la.plot_execution_stats(assisted_sampling)

if __name__ == "__main__":
    main()
