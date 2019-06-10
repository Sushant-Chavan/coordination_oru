#!/usr/bin/env python
# coding: utf-8

from ctypes import *
import numpy as np
import os
import errno
import sys
import pandas as pd
import yaml
import argparse
import shutil

# A class to receive the array of poses from the shared libarary
class PathPose(Structure):
    _fields_=[("x", c_double),
              ("y", c_double),
              ("theta", c_double)]

class OMPL_Wrapper():
    def __init__(self, map_filepath,
                 robot_footprint, robot_radius, turning_radius,
                 dist_between_points, planner_type, logfile,
                 is_holonomic_robot, training_data_file_name,
                 experienceDBPath, libname = 'libomplMotionPlanner.so'):
        self.map_filepath = map_filepath.encode(encoding='utf-8')
        self.robot_radius = c_double(robot_radius)
        self.turning_radius = c_double(turning_radius)
        self.dist_between_points = c_double(dist_between_points)
        self.planner_type = c_int(planner_type)
        self.logfile = logfile.encode(encoding='utf-8')
        self.is_holonomic_robot = c_bool(is_holonomic_robot)
        self.experienceDBPath = experienceDBPath.encode(encoding='utf-8')

        self.generate_collision_centers(robot_footprint)
        self.load_shared_library(libname)
        self.load_training_dataset(training_data_file_name)

        yaml_file_path = os.path.splitext(map_filepath)[0] + ".yaml"
        self.map_resolution = c_double(self.get_YAML_data(yaml_file_path)['resolution'])

    def get_YAML_data(self, filepath):
        data = None
        with open(filepath, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return data

    def load_training_dataset(self, training_data_filename):
        df = pd.read_csv(training_data_filename, header=None, sep='\t', usecols=[1,2,3])
        data = df.values
        nProblems = int(data.shape[0]/2)
        self.start_training_poses = data[:nProblems, :]
        self.goal_training_poses = data[nProblems:, :]

    def generate_collision_centers(self, robot_footprint):
        xCoords = []
        yCoords = []
        for i in range(robot_footprint.shape[0]):
            start = robot_footprint[i - 1]
            end = robot_footprint[i]
            xLen = np.abs(start[0] - end[0])
            yLen = np.abs(start[1] - end[1])

            numSamples = 4
            xCoords.extend(np.linspace(start[0], end[0], numSamples).tolist())
            yCoords.extend(np.linspace(start[1], end[1], numSamples).tolist())

        self.n_collsion_centers = len(xCoords)
        DoublesArray = c_double * self.n_collsion_centers
        self.collision_centers_x = (c_double * len(xCoords))(*xCoords)
        self.collision_centers_y = (c_double * len(yCoords))(*yCoords)

    def load_shared_library(self, lib_name):
        self.cdll = cdll.LoadLibrary(lib_name)

        # Function signature:
        # plan_multiple_circles(const char *mapFilename, double mapResolution,
        #                       double robotRadius, double *xCoords, double *yCoords,
        #                       int numCoords, double startX, double startY,
        #                       double startTheta, double goalX, double goalY,
        #                       double goalTheta, PathPose **path, int *pathLength,
        #                       double distanceBetweenPathPoints, double turningRadius,
        #                       PLANNER_TYPE plannerType, MODE mode, bool isHolonomicRobot,
        #                       const char* experienceDBPath, const char* logfile)
        self.cdll.plan_multiple_circles.arguments = [c_char_p, c_double,
                                       c_double, POINTER(c_double), POINTER(c_double), 
                                       c_int, c_double, c_double, 
                                       c_double, c_double, c_double, 
                                       c_double, POINTER(POINTER(PathPose)), POINTER(c_int), 
                                       c_double, c_double, 
                                       c_int, c_int, c_bool,
                                       c_char_p, c_char_p]
        self.cdll.plan_multiple_circles.restype = c_bool

    def invoke(self, start, goal):
        # Preparing to receive array of structs from the library
        path = POINTER(PathPose)()
        path_length = c_int(0)
        mode = 2 # Corresponds to the experience generation mode

        self.cdll.plan_multiple_circles(self.map_filepath, self.map_resolution, self.robot_radius,
                                        self.collision_centers_x, self.collision_centers_y, self.n_collsion_centers,
                                        c_double(start[0]), c_double(start[1]), c_double(start[2]),
                                        c_double(goal[0]), c_double(goal[1]), c_double(goal[2]),
                                        byref(path), byref(path_length), self.dist_between_points,
                                        self.turning_radius, self.planner_type, mode, self.is_holonomic_robot,
                                        self.experienceDBPath, self.logfile)

    def start_training(self):
        print("\n============ Starting Training ============")
        n_training_problems = self.start_training_poses.shape[0]
        for p_idx in range(n_training_problems):
            print("\n----------- Problem", p_idx+1, "/", n_training_problems, "-----------")
            self.invoke(self.start_training_poses[p_idx], self.goal_training_poses[p_idx])
        print("\n============ Training Complete ============")

def get_footprint(args):
    xmin = args.footprint[0]
    xmax = args.footprint[1]
    ymin = args.footprint[2]
    ymax = args.footprint[3]

    footprint = np.array([[xmin, ymax],
                          [xmax, ymax],
                          [xmax, ymin],
                          [xmin, ymin]])
    return footprint

def get_database_filepath(args, map_name):
    sampling_name = "Uniform" if args.no_hotspots else "UsingHotspots"
    kinematics = "ReedsSheep" if args.non_holonomic else "Holonomic"
    planner_names = ["SIMPLE(RRT-Connect)", "Lightning.db", "Thunder.db", "EGraphs", "SIMPLE(RRT-Star)"]
    directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/experienceDBs/")
    directory = os.path.join(directory, map_name)
    directory = os.path.join(directory, str(args.count)+"_TrainingExperiences")
    directory = os.path.join(directory, sampling_name)
    directory = os.path.join(directory, kinematics)
    if args.planner_type == 3:
        directory = os.path.join(directory, planner_names[args.planner_type])
        if os.path.isdir(directory):
            print("Clearing already existing EGraph paths")
            # shutil.rmtree(directory)

    # Make the directory if it does not exist
    try:
        os.makedirs(directory)
    except OSError as exc:
        if exc.errno ==errno.EEXIST and os.path.isdir(directory):
            pass
        else:
            raise "Could not create directory {}".format(directory)

    if args.planner_type == 3:
        path = directory
    else:
        path = os.path.join(directory, planner_names[args.planner_type])

    if os.path.isfile(path):
        print("A database already exists. Deleting it and creating a new one.")
        os.remove(path)

    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_filename", type=str, help="Filename of the map image that should be used for experience generation (ex. map1.png)")
    parser.add_argument("--robot_radius", type=float, help="Radius of the robot (in pixels) to be used for collision detection", default=0.1)
    parser.add_argument("--turning_radius", type=float, help="Turning radius of the vehicle in case of ReedsSheep type car", default=4.0)
    parser.add_argument("--dist_between_points", type=float, help="Max distance between two poses in the generated path", default=0.3)
    parser.add_argument("--planner_type", type=int, help="Type of planner to be used for experience generation (LIGHTNING:1, THUNDER:2), Default: Lightning", default=1)
    parser.add_argument("--non_holonomic", type=bool, help="Flag to specify if the robot is non_holonomic. Default: False", default=False)
    parser.add_argument("--count", type=int, help="Number of problems present in the training dataset. Default: 10", default=10)
    parser.add_argument("--footprint", nargs="*", type=float, help="Robot footprint as a list of xmin xmax ymin ymax", default=[-0.25, 0.25, -0.25, 0.25])
    parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience database is being generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    args = parser.parse_args()

    root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
    map_filepath = os.path.abspath(root_dir + "/maps/" + args.map_filename)
    footprint = get_footprint(args)

    map_name = os.path.splitext(args.map_filename)[0]
    training_dataset = os.path.abspath(root_dir + "/generated/trainingData/" + map_name + "-" + str(args.count) + "Problems.txt")

    if not os.path.isfile(training_dataset):
        print("Could not find training dataset file at",training_dataset)
        return

    database_path = get_database_filepath(args, map_name)

    ompl_wrapper = OMPL_Wrapper(map_filepath,
                    footprint, args.robot_radius, args.turning_radius,
                    args.dist_between_points, args.planner_type, "",
                    not args.non_holonomic, training_dataset, database_path)

    ompl_wrapper.start_training()

if __name__ == "__main__":
    main()
