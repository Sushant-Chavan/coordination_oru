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
                 is_holonomic_robot, testing_data_file_name,
                 experienceDBPath, libname = 'libomplMotionPlanner.so'):
        self.map_filepath = map_filepath.encode(encoding='utf-8')
        self.robot_radius = c_double(robot_radius)
        self.turning_radius = c_double(turning_radius)
        self.dist_between_points = c_double(dist_between_points)
        self.planner_type = c_int(planner_type)
        self.logfile = logfile.encode(encoding='utf-8')
        self.is_holonomic_robot = c_bool(is_holonomic_robot)
        self.experienceDBPath = experienceDBPath.encode(encoding='utf-8')
        self.testing_data_file_name = testing_data_file_name

        self.generate_collision_centers(robot_footprint)
        self.load_shared_library(libname)
        self.load_testing_dataset(self.testing_data_file_name)

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

    def load_testing_dataset(self, testing_data_filename):
        df = pd.read_csv(testing_data_filename, header=None, sep='\t', usecols=[1,2,3])
        data = df.values
        n_missions = int(data.shape[0]/3)

        source_offset = n_missions
        dest_offset = (2 * n_missions)

        self.start_testing_poses = []
        self.goal_testing_poses = []
        for mission in range(n_missions):
            start = data[mission]
            source = data[mission + source_offset]
            dest = data[mission + dest_offset]
            self.start_testing_poses.append(start)
            self.goal_testing_poses.append(source)
            self.start_testing_poses.append(source)
            self.goal_testing_poses.append(dest)
            self.start_testing_poses.append(dest)
            self.goal_testing_poses.append(start)
        self.start_testing_poses = np.array(self.start_testing_poses)
        self.goal_testing_poses = np.array(self.goal_testing_poses)

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
        # extern "C" bool plan_multiple_circles(
        #     const char *mapFilename, double mapResolution, double robotRadius,
        #     double *xCoords, double *yCoords, int numCoords, double startX,
        #     double startY, double startTheta, double goalX, double goalY,
        #     double goalTheta, PathPose **path, int *pathLength, double* pathCost,
        #     double distanceBetweenPathPoints, double turningRadius,
        #     PLANNER_TYPE plannerType, MODE mode, bool isHolonomicRobot,
        #     const char *experienceDBPath, const char *logfile)
        self.cdll.plan_multiple_circles.arguments = [c_char_p, c_double,
                                       c_double, POINTER(c_double), POINTER(c_double), 
                                       c_int, c_double, c_double, 
                                       c_double, c_double, c_double, 
                                       c_double, POINTER(POINTER(PathPose)), POINTER(c_int), 
                                       POINTER(c_double), c_double, c_double, 
                                       c_int, c_int, c_bool,
                                       c_char_p, c_char_p]
        self.cdll.plan_multiple_circles.restype = c_bool

    def invoke(self, start, goal):
        # Preparing to receive array of structs from the library
        path = POINTER(PathPose)()
        path_length = c_int(0)
        path_cost = c_double(0)
        mode = 3 # Corresponds to the optimal path generation mode

        self.cdll.plan_multiple_circles(self.map_filepath, self.map_resolution, self.robot_radius,
                                        self.collision_centers_x, self.collision_centers_y, self.n_collsion_centers,
                                        c_double(start[0]), c_double(start[1]), c_double(start[2]),
                                        c_double(goal[0]), c_double(goal[1]), c_double(goal[2]),
                                        byref(path), byref(path_length), byref(path_cost), self.dist_between_points,
                                        self.turning_radius, self.planner_type, mode, self.is_holonomic_robot,
                                        self.experienceDBPath, self.logfile)

        return path_cost.value

    def find_optimal_solutions(self):
        print("\n============ Finding optimal solutions ============")
        n_testing_problems = self.start_testing_poses.shape[0]
        path_costs = []
        for p_idx in range(n_testing_problems):
            print("\n----------- Problem", p_idx+1, "/", n_testing_problems, "-----------")
            path_costs.append(self.invoke(self.start_testing_poses[p_idx], self.goal_testing_poses[p_idx]))
        print("\n============ Found all optimal solutions ============")
        print("\nSaving optimality data")
        self.save_optimal_costs(np.array(path_costs))

    def save_optimal_costs(self, path_costs):
        save_path = os.path.join(os.path.dirname(self.testing_data_file_name), "Optimality")
        # Make the directory if it does not exist
        try:
            os.makedirs(save_path)
        except OSError as exc:
            if exc.errno ==errno.EEXIST and os.path.isdir(save_path):
                pass
            else:
                raise "Could not create directory {}".format(save_path)
        
        save_path = os.path.join(save_path, os.path.basename(self.testing_data_file_name))
        np.savetxt(save_path, path_costs, delimiter='\t')
        print("Optimal costs saved to file:", save_path)


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
    directory = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../generated/tempExpDB/")
    directory = os.path.join(directory, map_name)
    directory = os.path.join(directory, str(args.count)+"_optimalityExperiences")
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
    parser.add_argument("--count", type=int, help="Number of problems present in the testing dataset. Default: 10", default=10)
    parser.add_argument("--footprint", nargs="*", type=float, help="Robot footprint as a list of xmin xmax ymin ymax", default=[-0.25, 0.25, -0.25, 0.25])
    parser.add_argument("--no_hotspots", type=bool, help="Indicate if the experience database is being generated using uniform sampling of the map. Default: False (hotspots used)", default=False)
    args = parser.parse_args()

    root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
    map_filepath = os.path.abspath(root_dir + "/maps/" + args.map_filename)
    footprint = get_footprint(args)

    map_name = os.path.splitext(args.map_filename)[0]
    testing_dataset = os.path.abspath(root_dir + "/generated/testingData/" + map_name + "-" + str(args.count) + "Problems.txt")

    if not os.path.isfile(testing_dataset):
        print("Could not find testing dataset file at",testing_dataset)
        return

    database_path = get_database_filepath(args, map_name)

    ompl_wrapper = OMPL_Wrapper(map_filepath,
                    footprint, args.robot_radius, args.turning_radius,
                    args.dist_between_points, args.planner_type, "",
                    not args.non_holonomic, testing_dataset, database_path)

    ompl_wrapper.find_optimal_solutions()

if __name__ == "__main__":
    main()
