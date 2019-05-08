#!/usr/bin/env python
# coding: utf-8

import numpy as np
import os
import sys
import argparse
import pandas as pd

# A class to extract relevant lines from a complete log file of a 
class logParser:
    def __init__(self, tags_filpath):
        self.tags_filpath = tags_filpath
        self.tags = None
        self.logs = None

        self.get_tags(self.tags_filpath)
        print(self.tags)

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
                if t in l:
                    self.logs.append(l)
                    break
        return self.logs

    def dump_log_summary(self, filename):
        assert(self.logs is not None)

        with open(filename, 'w') as f:
            f.writelines(self.logs)

    def generate_planning_csv(self):
        pass
    
    def generate_execution_csv(self):
        pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_filename", type=str, help="Filename of the map image that should be used for experience generation (ex. map1.png)")
    parser.add_argument("--tags_filename", type=str, help="Radius of the robot (in pixels) to be used for collision detection", default="default_tags.txt")
    parser.add_argument("--thunder", help="Set this if used with logs of Thunder planners", action="store_true", default=False)
    args = parser.parse_args()

    root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
    map_name = os.path.splitext(args.map_filename)[0]

    tags_filepath = root_dir + "/generators/logging/tags/" + args.tags_filename
    planner_name = "thunder" if args.thunder else "lightning"
    log_filepath = root_dir + "/generated/experienceLogs/" + map_name + "_" + planner_name + ".log"
    summary_log_filename = os.path.splitext(log_filepath)[0] + "_summary.log"

    lp = logParser(tags_filepath)
    lp.extract_tagged_lines(log_filepath)
    lp.dump_log_summary(summary_log_filename)

if __name__ == "__main__":
    main()
