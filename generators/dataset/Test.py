#!/usr/bin/env python
# coding: utf-8

from ctypes import *

lib = 'libomplMotionPlanner.so'
dll = cdll.LoadLibrary(lib)

# Defining a class to receive the array of poses from the shared libarary
class PathPose(Structure):
    _fields_=[("x", c_double),
              ("y", c_double),
              ("theta", c_double)]

# Declaring the argument types
dll.testFunc.arguments = [c_char_p, POINTER(c_double), c_int, POINTER(POINTER(PathPose)), POINTER(c_int), c_int]

# Passing an array to the library
nDoubles = 5
DoublesAray = c_double * nDoubles
dArr = DoublesAray(0.0, 0.1, 0.2, 0.3, 0.4)

# Preparing to receive array of structs from the library
mem = POINTER(PathPose)()
size = c_int(0)

name = "Sushant"

# Calling the function
dll.testFunc(name.encode(encoding='utf-8'), dArr, nDoubles, byref(mem), byref(size), 1)

print("Path length: ", size.value)
print("PathPoses:")
for i in range(size.value):
    pose = mem[i]
    print(pose.x, pose.y, pose.theta)

# Function signature:
# bool plan_multiple_circles(const char *mapFilename, double mapResolution,
#                       double robotRadius, double *xCoords, double *yCoords,
#                       int numCoords, double startX, double startY,
#                       double startTheta, double goalX, double goalY,
#                       double goalTheta, PathPose **path, int *pathLength,
#                       double distanceBetweenPathPoints, double turningRadius,
#                       PLANNER_TYPE plannerType, const char *experienceDBName,
#                       bool isReplan, bool isHolonomicRobot)

dll.plan_multiple_circles.arguments = [c_char_p, c_double,
                                       c_double, POINTER(c_double), POINTER(c_double), 
                                       c_int, c_double, c_double, 
                                       c_double, c_double, c_double, 
                                       c_double, POINTER(POINTER(PathPose)), POINTER(c_int), 
                                       c_double, c_double, 
                                       c_int,  c_char_p, 
                                       c_bool, c_bool]
dll.plan_multiple_circles.restype = c_bool
