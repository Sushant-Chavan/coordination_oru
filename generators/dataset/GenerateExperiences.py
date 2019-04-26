#!/usr/bin/env python
# coding: utf-8

from ctypes import *
import numpy as np
import os
import sys
lib = 'libomplMotionPlanner.so'
dll = cdll.LoadLibrary(lib)

# Defining a class to receive the array of poses from the shared libarary
class PathPose(Structure):
    _fields_=[("x", c_double),
              ("y", c_double),
              ("theta", c_double)]

# # Declaring the argument types
# dll.testFunc.arguments = [c_char_p, POINTER(c_double), c_int, POINTER(POINTER(PathPose)), POINTER(c_int), c_int]

# # Passing an array to the library
# nDoubles = 5
# DoublesAray = c_double * nDoubles
# dArr = DoublesAray(0.0, 0.1, 0.2, 0.3, 0.4)

# # Preparing to receive array of structs from the library
# mem = POINTER(PathPose)()
# size = c_int(0)

# name = "Sushant"

# # Calling the function
# dll.testFunc(name.encode(encoding='utf-8'), dArr, nDoubles, byref(mem), byref(size), 1)

# print("Path length: ", size.value)
# print("PathPoses:")
# for i in range(size.value):
#     pose = mem[i]
#     print(pose.x, pose.y, pose.theta)

class OMPL_Wrapper():
    def __init__(self, map_filename, map_resolution,
                 robot_footprint, robot_radius, turning_radius,
                 dist_between_points, planner_type, experience_db_name,
                 is_holonomic_robot, libname = 'libomplMotionPlanner.so'):
        self.map_filename = map_filename.encode(encoding='utf-8')
        self.map_resolution = c_double(map_resolution)
        self.robot_radius = c_double(robot_radius)
        self.turning_radius = c_double(turning_radius)
        self.dist_between_points = c_double(dist_between_points)
        self.planner_type = c_int(planner_type)
        self.experience_db_name = experience_db_name.encode(encoding='utf-8')
        self.is_holonomic_robot = c_bool(is_holonomic_robot)

        self.generate_collision_centers(robot_footprint)
        self.load_shared_library(libname)

    def generate_collision_centers(self, robot_footprint):
        xCoords = []
        yCoords = []
        for i in range(robot_footprint.shape[0]):
            start = robot_footprint[i - 1]
            end = robot_footprint[i]
            xLen = np.abs(start[0] - end[0])
            yLen = np.abs(start[1] - end[1])

            numSamples = int(max(xLen, yLen) * 5)
            xCoords.extend(np.linspace(start[0], end[0], numSamples).tolist())
            yCoords.extend(np.linspace(start[1], end[1], numSamples).tolist())

        self.n_collsion_centers = len(xCoords)
        DoublesArray = c_double * self.n_collsion_centers
        self.collision_centers_x = (c_double * len(xCoords))(*xCoords)
        self.collision_centers_y = (c_double * len(yCoords))(*yCoords)

    def load_shared_library(self, lib_name):
        self.cdll = cdll.LoadLibrary(lib_name)

        # Function signature:
        # bool plan_multiple_circles(const char *mapFilename, double mapResolution,
        #                       double robotRadius, double *xCoords, double *yCoords,
        #                       int numCoords, double startX, double startY,
        #                       double startTheta, double goalX, double goalY,
        #                       double goalTheta, PathPose **path, int *pathLength,
        #                       double distanceBetweenPathPoints, double turningRadius,
        #                       PLANNER_TYPE plannerType, const char *experienceDBName,
        #                       bool isReplan, bool isHolonomicRobot)
        self.cdll.plan_multiple_circles.arguments = [c_char_p, c_double,
                                       c_double, POINTER(c_double), POINTER(c_double), 
                                       c_int, c_double, c_double, 
                                       c_double, c_double, c_double, 
                                       c_double, POINTER(POINTER(PathPose)), POINTER(c_int), 
                                       c_double, c_double, 
                                       c_int,  c_char_p, 
                                       c_bool, c_bool]
        self.cdll.plan_multiple_circles.restype = c_bool

    def invoke(self, start, goal):
        # Preparing to receive array of structs from the library
        path = POINTER(PathPose)()
        path_length = c_int(0)

        self.cdll.plan_multiple_circles(self.map_filename, self.map_resolution, self.robot_radius,
                                        self.collision_centers_x, self.collision_centers_y, self.n_collsion_centers,
                                        c_double(start[0]), c_double(start[1]), c_double(start[2]),
                                        c_double(goal[0]), c_double(goal[1]), c_double(goal[2]),
                                        byref(path), byref(path_length), self.dist_between_points,
                                        self.turning_radius, self.planner_type, self.experience_db_name,
                                        False, self.is_holonomic_robot)


root_dir = os.path.abspath(os.path.split(os.path.abspath(sys.argv[0]))[0]  + "/../../")
map_filename = "map1.png"
robot_radius = 10

map_filename = os.path.abspath(root_dir + "/maps/" + map_filename)
map_resolution = 1.0
footprint = np.array([[-1.0,0.5],
                      [1.0,0.5],
                      [1.0,-0.5],
                      [-1.0,-0.5]])
robot_radius = 1.0
turning_radius = 1.0
dist_between_points = 0.5
planner_type = 1
experience_db_name = "map1"
is_holonomic_robot = True

ompl_wrapper = OMPL_Wrapper(map_filename, map_resolution,
                 footprint, robot_radius, turning_radius,
                 dist_between_points, planner_type, experience_db_name,
                 is_holonomic_robot)

ompl_wrapper.invoke([722.0, 432.0, -0.06940318744781715], [564.0, 565.0, -0.3667634555408852])
