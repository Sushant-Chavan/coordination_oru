#!/usr/bin/env python
# coding: utf-8

import numpy as np
import os
import sys
import argparse
import pandas as pd
import re

# A class to extract relevant lines from a complete log file of a test run and generate a CSV logg file
class logParser:
    def __init__(self, tags_filpath):
        self.tags_filpath = tags_filpath
        self.tags = None
        self.logs = None

        # In addtion to the tags, also extract the start of test tags
        self.test_start_regex = re.compile("Test (.*) started at")

        self.get_tags(self.tags_filpath)

    def get_tags(self, tags_filpath):
        assert(tags_filpath is not None and (len(tags_filpath) != 0))
        with open(tags_filpath) as f:
            self.tags = f.readlines()
            # Remove whitespace characters like `\n` at the end of each line
            self.tags = [x.strip() for x in self.tags]

    def validate_log(self, lines):
        for l in lines:
            assert ("tempMaps" not in l), "Found planning details of replanning stage. Please clean log before proceeding!!. Hint: Search for \"tempMaps\" in logs"


    def extract_tagged_lines(self, log_filepath):
        assert(self.tags is not None)
        with open(log_filepath) as f:
            lines = f.readlines()

        self.validate_log(lines)
        self.logs = []
        for l in lines:
            for t in self.tags:
                if (t in l) or re.match(self.test_start_regex, l):
                    self.logs.append(l)
                    break
        return self.logs

    def dump_log_summary(self, filename):
        assert(self.logs is not None)

        with open(filename, 'w') as f:
            f.writelines(self.logs)

    def _extract_test_start_times(self):
        ln_to_testN_map = {}
        for i, l in enumerate(self.logs):
            if re.match(self.test_start_regex, l):
                test_name = (l.split("\"")[1]).strip()
                test_time = (l.split("at")[-1]).strip()
                ln_to_testN_map[i] = (test_name, test_time)

        planning_ln = []
        for i, l in enumerate(self.logs):
            if "Start time: " in l:
                planning_ln.append(i)

        keys = sorted(ln_to_testN_map.keys())
        test_mapping = np.zeros(len(planning_ln))
        for i, l_num in enumerate(planning_ln):
            for j, k in enumerate(keys):
                if k < l_num:
                    test_mapping[i] = k
                else:
                    break

        test_names = [ln_to_testN_map[k][0] for k in test_mapping]
        test_times = [ln_to_testN_map[k][1] for k in test_mapping]

        return test_names, test_times

    def _extract_start_times(self):
        times = []
        for l in self.logs:
            if "Start time: " in l:
                times.append((l.split(": ")[1]).strip())
        return times

    def _extract_map_filenames(self):
        map_names = []
        for l in self.logs:
            if "Map Filename: " in l:
                map_names.append((l.split(": ")[1]).strip())
        return map_names

    def _extract_map_resolutions(self):
        map_res = []
        for l in self.logs:
            if "Map Resolution: " in l:
                map_res.append(float((l.split(": ")[1]).strip()))
        return map_res

    def _get_pose_from_string(self, string):
        pose = string[1:-1]
        elements = pose.split(',')
        elements = [float(e) for e in elements]
        return elements

    def _extract_poses(self, tag="Start Pose: "):
        poses = []
        for l in self.logs:
            if tag in l:
                p = (l.split(": ")[1]).strip()
                poses.append(self._get_pose_from_string(p))
        return np.array(poses)

    def _extract_planner_types(self):
        planners = []
        for l in self.logs:
            if "Planner Type: " in l:
                planners.append((l.split(": ")[1]).strip())
        return planners

    def _extract_path_length(self):
        lengths = []
        for l in self.logs:
            if "Length of computed path" in l:
                lengths.append((l.split("= ")[1]).strip())
        return lengths

    def _extract_robot_kinematics(self):
        kinemetics = []
        for l in self.logs:
            if "Is Holonomic Robot: " in l:
                kinemetics.append(1 if (l.split(": ")[1]).strip() == "True" else 0)
        return kinemetics

    def _extract_planning_times(self):
        times = []
        for l in self.logs:
            if "Possible solution found in " in l or\
                "Solution found in " in l:
                times.append(float(l.strip().split()[-2]))
        return times

    def _extract_path_simplification_times(self):
        times = []
        for l in self.logs:
            if "SimpleSetup: Path simplification took" in l:
                times.append(float(l.strip().split()[5]))
        return times

    def _extract_from_recall_stats(self):
        recall = []
        for l in self.logs:
            if "and was generated from planner" in l:
                last_element = l.strip().split()[-1]
                if ((last_element == "LightningRetrieveRepair") or 
                    (last_element == "Thunder_Retrieve_Repair")):
                    recall.append(1)
                else:
                    recall.append(0)
        return recall

    def _extract_total_planning_times(self):
        times = []
        for l in self.logs:
            if "Planning took " in l:
                times.append(float(l.strip().split()[-2]))
        return times

    def _extract_path(self):
        paths = []
        for l in self.logs:
            if l[0] == "(":
                clean_path = l.replace("(", "")
                clean_path = clean_path.replace(")", ";")
                clean_path = clean_path.replace(",", ";")
                paths.append(clean_path)
        return paths

    def generate_planning_csv(self, csv_filepath):
        assert(self.logs is not None)

        nPlans = sum('Planning took' in s for s in self.logs)
        assert (nPlans % 3 == 0), "Expected number of plans to be a multiple of 3!!"

        indices = np.arange(1, nPlans+1, 1)
        columns = ["Test Name", "Test Start Time",  "Planning Start Time", "Map Filename", "Map Resolution", "Start X", "Start Y", "Start Theta",
                   "Goal X", "Goal Y", "Goal Theta", "Planner Type", "Holonomic", 
                   "Planning Time", "Path simplification time", "From recall", "Total planning time", "Path Length", "Path"]
        df = pd.DataFrame(index=indices, columns=columns)
        df = df.fillna("-")

        df["Test Name"], df["Test Start Time"] = self._extract_test_start_times()

        df["Planning Start Time"] = self._extract_start_times()
        df["Map Filename"] = self._extract_map_filenames()
        df["Map Resolution"] = self._extract_map_resolutions()

        start_poses = self._extract_poses(tag='Start Pose: ')
        df["Start X"] = start_poses[:, 0]
        df["Start Y"] = start_poses[:, 1]
        df["Start Theta"] = start_poses[:, 2]

        goal_poses = self._extract_poses(tag='Goal Pose: ')
        df["Goal X"] = goal_poses[:, 0]
        df["Goal Y"] = goal_poses[:, 1]
        df["Goal Theta"] = goal_poses[:, 2]

        df["Planner Type"] = self._extract_planner_types()
        df["Holonomic"] = self._extract_robot_kinematics()
        df["Planning Time"] = self._extract_planning_times()
        df["Path simplification time"] = self._extract_path_simplification_times()
        recall_stats = self._extract_from_recall_stats()
        if len(recall_stats) > 0:
            df["From recall"] = recall_stats
        df["Total planning time"] = self._extract_total_planning_times()
        df["Path Length"] = self._extract_path_length()
        df["Path"] = self._extract_path()

        df.to_csv(csv_filepath)
        print("Planning logs CSV generated/extended at", csv_filepath)

    def _find_nRobots(self):
        max_robot_ID = 1
        for l in self.logs:
            if "Start Mission 0" in l:
                tag = l.split()[0]
                id = int(tag.split("-")[1][:-1])
                if id > max_robot_ID:
                    max_robot_ID = id
        return max_robot_ID

    def _extract_test_name_and_time(self):
        ln_to_testN_dic = {}
        for i, l in enumerate(self.logs):
            if re.match(self.test_start_regex, l):
                test_name = (l.split("\"")[1]).strip()
                test_time = (l.split("at")[-1]).strip()
                ln_to_testN_dic[i] = (test_name, test_time)
        keys = sorted(ln_to_testN_dic.keys())

        return ln_to_testN_dic, keys

    def get_test_num_from_line_num(self, ln_to_test_dic, line_num):
        test_start_line_num = sorted(ln_to_test_dic.keys())

        test_num = None
        for n, k in enumerate(test_start_line_num):
            if k < line_num:
                test_num = n
            else:
                break

        return test_num


    def generate_execution_csv(self, csv_filepath):
        assert(self.logs is not None)

        ln_to_test_dic, keys = self._extract_test_name_and_time()
        nTests = len(keys)

        nRobots = self._find_nRobots()
        nRows = nRobots * nTests
        indices = np.arange(1, nRows+1, 1)
        columns = ["Test Name", "Test Start Time", "Num of replans", "Robot ID", "Successful Misions",
                   "Mission1 Duration", "Mission2 Duration", "Mission3 Duration",
                   "Mission1 Start Time", "Mission1 End Time",
                   "Mission2 Start Time", "Mission2 End Time",
                   "Mission3 Start Time", "Mission3 End Time",]
        df = pd.DataFrame(index=indices, columns=columns)
        df = df.fillna("x")

        test_names = []
        test_start_times = []
        for k in keys:
            test_names.extend([ln_to_test_dic[k][0]] * nRobots)
            test_start_times.extend([ln_to_test_dic[k][1]] * nRobots)
        
        df["Test Name"] = test_names
        df["Test Start Time"] = test_start_times
        df["Robot ID"] = np.arange(1, nRobots+1, 1).tolist() * nTests

        num_replans = np.zeros(nTests)
        for i, line in enumerate(self.logs):
            if "[ROBOT-" in line:
                test_num = self.get_test_num_from_line_num(ln_to_test_dic, i)
                # print(line, test_num)
                if "Start Mission" in line:
                    segments = line.split()
                    mission_id = int(segments[3]) + 1
                    robot_num = int((segments[0].split("-"))[1][:-1]) - 1
                    time = segments[-2] + " " + segments[-1]
                    row_id = (test_num * nRobots) + robot_num
                    col_id = "Mission" + str(mission_id) + " Start Time"
                    df.iloc[row_id, df.columns.get_loc(col_id)] = time
                elif "completed at" in line:
                    segments = line.split()
                    mission_id = int(segments[2]) + 1
                    robot_num = int((segments[0].split("-"))[1][:-1]) - 1
                    time = segments[-2] + " " + segments[-1]
                    row_id = (test_num * nRobots) + robot_num
                    col_id = "Mission" + str(mission_id) + " End Time"
                    df.iloc[row_id, df.columns.get_loc(col_id)] = time
                elif "Time to complete mission" in line:
                    segments = line.split()
                    mission_id = int(segments[5]) + 1
                    robot_num = int((segments[0].split("-"))[1][:-1]) - 1
                    time = float(segments[-1][:-1])
                    row_id = (test_num * nRobots) + robot_num
                    col_id = "Mission" + str(mission_id) + " Duration"
                    df.iloc[row_id, df.columns.get_loc(col_id)] = time
            elif "Replanning Triggered" in line:
                test_num = self.get_test_num_from_line_num(ln_to_test_dic, i)
                num_replans[test_num] += 1

        replan_list = np.zeros(df.values.shape[0])
        for test_id in range(nTests):
            start = test_id * nRobots
            end = start + nRobots
            replan_list[start:end] = num_replans[test_id]

        df["Num of replans"] = replan_list

        col_id = "Successful Misions"
        for row_id in range(df.values.shape[0]):
            nSuccess = 0
            for i in range(1, 4):
                if df.iloc[row_id, df.columns.get_loc("Mission" + str(i) + " Duration")] != 'x':
                    nSuccess += 1
            df.iloc[row_id, df.columns.get_loc(col_id)] = nSuccess
        df.to_csv(csv_filepath)
        print("Execution logs CSV generated/extended at", csv_filepath)

def get_log_filename(args):
    sampling_name = "Uniform" if args.no_hotspots else "UsingHotspots"
    kinematics = "ReedsSheep" if args.constrained else "Holonomic"
    planner_names = ["SIMPLE(RRT-Connect)", "Lightning", "Thunder", "SIMPLE(RRT-Star)"]
    directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/executionData/")
    directory = os.path.join(directory, args.map)
    directory = os.path.join(directory, planner_names[args.planner])
    directory = os.path.join(directory, str(args.nRobots)+"_Robots")
    directory = os.path.join(directory, kinematics)
    directory = os.path.join(directory, sampling_name)
    directory = os.path.join(directory, str(args.nExperiences)+"_TrainingExperiences/Logs")
    print(directory)

    return directory + "/CompleteLog.log"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", type=str, help="Filename of the map image that should be used for experience generation (ex. map1.png)")
    parser.add_argument("planner", type=int, help="ID of the planner (SIMPLE_RRT-Connect:0, LIGHTNING:1, THUNDER:2, SIMPLE_RRT-Star:3)")
    parser.add_argument("--nRobots", type=int, help="Number of robots to be used in the testing. Default: 3", default=3)
    parser.add_argument("--constrained", type=bool, help="Indicate if the robots are ReedsSheep like vehicles. Default: False (holonomic)", default=False)
    parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience databases are generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    parser.add_argument("--nExperiences", type=int, help="Number of training problems used to build the experience DB. Default: 100", default=100)
    parser.add_argument("--tags_filename", type=str, help="Filename of file containing tags used to filter the log", default="default_tags.txt")
    args = parser.parse_args()

    planner_names = ["rrt_connect", "lightning", "thunder", "rrt_star"]

    root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
    map_name = os.path.splitext(args.map)[0]

    tags_filepath = root_dir + "/generators/logging/tags/" + args.tags_filename
    planner_name = planner_names[args.planner]
    log_filepath = get_log_filename(args)

    if not os.path.isfile(tags_filepath):
        print("Log tags file does not exist! \nPath specified was:\n", tags_filepath)
        return

    if not os.path.isfile(log_filepath):
        print("Log file does not exist! \nPath specified was:\n", log_filepath)
        return

    summary_log_filename = os.path.splitext(log_filepath)[0] + "_summary.log"
    csv_planning_log_filename = os.path.dirname(log_filepath) + "/Planning.csv"
    csv_execution_log_filename = os.path.dirname(log_filepath) + "/Execution.csv"

    lp = logParser(tags_filepath)
    lp.extract_tagged_lines(log_filepath)
    # lp.dump_log_summary(summary_log_filename)
    lp.generate_planning_csv(csv_planning_log_filename)
    lp.generate_execution_csv(csv_execution_log_filename)

if __name__ == "__main__":
    main()
