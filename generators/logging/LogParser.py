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

    def extract_tagged_lines(self, log_filepath):
        assert(self.tags is not None)
        with open(log_filepath) as f:
            lines = f.readlines()
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
        indices = np.arange(1, nPlans+1, 1)
        columns = ["Test Name", "Test Start Time",  "Planning Start Time", "Map Filename", "Map Resolution", "Start X", "Start Y", "Start Theta",
                   "Goal X", "Goal Y", "Goal Theta", "Planner Type", "Holonomic", 
                   "Planning Time", "Path simplification time", "From recall", "Total planning time", "Path"]
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
        df["Path"] = self._extract_path()

        df.to_csv(csv_filepath)
        print("Planning logs CSV generated/extended at", csv_filepath)

    def generate_execution_csv(self):
        pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_filename", type=str, help="Filename of the map image that should be used for experience generation (ex. map1.png)")
    parser.add_argument("planner", type=int, help="ID of the planner (SIMPLE:0, LIGHTNING:1, THUNDER:2)")
    parser.add_argument("--tags_filename", type=str, help="Filename of file containing tags used to filter the log", default="default_tags.txt")
    args = parser.parse_args()

    planner_names = ["simple", "lightning", "thunder"]

    root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
    map_name = os.path.splitext(args.map_filename)[0]

    tags_filepath = root_dir + "/generators/logging/tags/" + args.tags_filename
    planner_name = planner_names[args.planner]
    log_filepath = root_dir + "/generated/experienceLogs/" + map_name + "_" + planner_name + ".log"

    if not os.path.isfile(tags_filepath):
        print("Log tags file does not exist! \nPath specified was:\n", tags_filepath)
        return

    if not os.path.isfile(log_filepath):
        print("Log file does not exist! \nPath specified was:\n", log_filepath)
        return

    summary_log_filename = os.path.splitext(log_filepath)[0] + "_summary.log"
    csv_log_filename = os.path.splitext(log_filepath)[0] + ".csv"

    lp = logParser(tags_filepath)
    lp.extract_tagged_lines(log_filepath)
    # lp.dump_log_summary(summary_log_filename)
    lp.generate_planning_csv(csv_log_filename)

if __name__ == "__main__":
    main()
