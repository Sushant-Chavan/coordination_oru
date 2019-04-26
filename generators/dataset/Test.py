#!/usr/bin/env python
# coding: utf-8

from ctypes import *

lib = 'libomplMotionPlanner.so'
dll = cdll.LoadLibrary(lib)

dll.helloWorld.arguments=[c_char_p]

dll.helloWorld(b" Sushant")
