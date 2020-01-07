#!/usr/bin/env python

import rospy
import rospkg
import rosservice
import yaml
import random
import math
import socket 
import json

from pi_trees_lib.pi_trees_lib import CallbackTask, TaskStatus, print_tree
from pi_trees_lib.pi_trees_lib import Sequence, UntilSuccess, loop  #, Limit

from piiq_msgs.srv import GetCameraData, GetCameraDataRequest, DetectComponents, DetectComponentsRequest
from piiq_msgs.srv import ExcuteMove, ExcuteMoveRequest, LoadEnvironment
from piiq_msgs.msg import CModel_robot_output as outputMsg
from piiq_msgs.srv import RobotiqCCommand
from piiq_coordinator_node import PIIQCoordinatorNode

import time

target_pose_home_rad = [-1.569, -1.668, -1.932, -0.327, 1.571, 1.568]
target_pose_place_rad = [-1.05, -1.87, -1.74, -1.52, 0.92, 0.63]
GRIPPER_TYPE_0 = 255  # cheng
GRIPPER_TYPE_1 = 130  # xiao jia
GRIPPER_TYPE_2 = 100  # da jia
GRIPPER_TYPE_DELTA = 90


try:
    PIIQCoordinatorNode()
except KeyboardInterrupt:
    print "Shutting down piiq_groceries_coordinator node."
