#!/usr/bin/env python

import rospy
import rospkg
import rosservice
import yaml
import random
import math
import serial
import threading
import cv2 
import numpy as np


from pi_trees_lib.pi_trees_lib import CallbackTask, TaskStatus, print_tree
from pi_trees_lib.pi_trees_lib import Sequence, UntilSuccess, loop  #Limit

from piiq_msgs.srv import GetCameraData, GetCameraDataRequest, DetectComponents, DetectComponentsRequest
from piiq_msgs.srv import ExcuteMove, ExcuteMoveRequest, LoadEnvironment
from piiq_msgs.msg import CModel_robot_output as outputMsg
from piiq_msgs.srv import RobotiqCCommand

from Test40 import SerialPort as spclose
from Test2300 import SerialPort as spopen
from compiler.ast import flatten

import time

import socket 
import json

#target_pose_home_rad = [-1.569, -1.668, -1.932, -0.327, 1.571, 1.568]
#target_pose_home_rad = [-1.45,-1.658,-1.6,-1.3,1.53,1.9]
target_pose_home_rad = [-84.98*math.pi/180.0, -98.19*math.pi/180.0, -90.98*math.pi/180.0, -77.08*math.pi/180.0, 89.34*math.pi/180.0, 19.10*math.pi/180.0]
#target_pose_home_rad = [-83.80*math.pi/180.0, -96.49*math.pi/180.0, -100.85*math.pi/180.0, -65.67*math.pi/180.0, 86.74*math.pi/180.0, 17.08*math.pi/180.0]
target_pose_place_rad = [-73.82*math.pi/180.0, -108.46*math.pi/180.0, -93.49*math.pi/180.0, -64.91*math.pi/180.0, 86.68*math.pi/180.0, 27.01*math.pi/180.0]

GRIPPER_TYPE_0 = 255  # cheng
GRIPPER_TYPE_1 = 130  # xiao jia
GRIPPER_TYPE_2 = 100  # da jia

rospack = rospkg.RosPack()
rospack.list() 
cameraFilepath = rospack.get_path("piiq_components")+"/config/axle_base/test_camera_calib_bas.yml";
print cameraFilepath

fs=cv2.FileStorage(cameraFilepath, cv2.FileStorage_READ )

camera_matrix = np.zeros([3,3])
distcoeffs = np.zeros([1,5])
camera_matrix[0,0]=fs.getNode('fx').real()

camera_matrix[1,1]=fs.getNode('fy').real()
camera_matrix[0,2]=fs.getNode('cx').real()
camera_matrix[1,2]=fs.getNode('cy').real()
camera_matrix[2,2] = 1;
distcoeffs[0,2]=fs.getNode('dist_px').real()
distcoeffs[0,3]=fs.getNode('dist_py').real()
distcoeffs[0,0]=fs.getNode('dist_k1').real()
distcoeffs[0,1]=fs.getNode('dist_k2').real()
distcoeffs[0,4]=fs.getNode('dist_k3').real()
fs.release()
print camera_matrix
print distcoeffs

class PIIQCoordinatorNode:
    def __init__(self):
        rospy.init_node('piiq_components_coordinator', anonymous=False)
        self.init_self_data()

        SEQ_BEHAVE = Sequence("piiq_components_demo")

        RUN_INIT_ROBOT = CallbackTask("initialize_the_robot", self.init_robot)

        SUC_INIT_ROBOT = UntilSuccess("init_until_success", max_attempts=50)
        SUC_INIT_ROBOT.add_child(RUN_INIT_ROBOT)

        SEQ_PICKING = Sequence("picking_action_sequence")
        RUN_MOVE_HOME = CallbackTask("move_home_pose", self.move_back_home)
        RUN_SENSE = CallbackTask("take_picture_and_detect", self.sense_and_detect)

        RUN_HOME_SENSE = CallbackTask("move_home_and_detect", self.home_and_detect)
        SUC_HOME_SENSE = UntilSuccess("home_and_detect_until_suc", max_attempts=150)
        SUC_HOME_SENSE.add_child(RUN_HOME_SENSE)

        RUN_MOVE_PICK = CallbackTask("move_picking_pose", self.move_to_pick)
        RUN_SUCTION = CallbackTask("grip_and_check", self.grasp_and_check)
        RUN_MOVE_PLACE = CallbackTask("move_placing_pose", self.move_to_place)

        # SEQ_PICKING.add_child(RUN_MOVE_HOME)
        # SEQ_PICKING.add_child(RUN_SENSE)

        SEQ_PICKING.add_child(SUC_HOME_SENSE)
        SEQ_PICKING.add_child(RUN_MOVE_PICK)
        SEQ_PICKING.add_child(RUN_SUCTION)
        SEQ_PICKING.add_child(RUN_MOVE_PLACE)
        SEQ_PICKING.add_child(RUN_MOVE_HOME)

        LOO_PICKING = loop(SEQ_PICKING, iterations=150)

        SEQ_BEHAVE.add_child(SUC_INIT_ROBOT)
        SEQ_BEHAVE.add_child(LOO_PICKING)

        print "Behavior Tree Structure"
        print_tree(SEQ_BEHAVE, use_symbols=True)

        # Run the tree
        # while True:
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            status = SEQ_BEHAVE.run()
            # print TaskStatus.SUCCESS, status
            if status == TaskStatus.SUCCESS:
                rospy.loginfo("Finished running piiq_components_coordinator.")
                break
            r.sleep()

    def init_self_data(self):
        self.camera_request = GetCameraDataRequest()
        self.camera_request.num_to_average = 1
        self.camera_request.dump_image = False
        self.camera_request.dump_pointcloud = False
        self.camera_request.dump_prefix = ""
        self.camera_request.visualize = False
        self.camera_request.rectified = False

        self.camera_response = None  # GetCameraDataResponse()

        self.motion_request = ExcuteMoveRequest()
        self.motion_request.type = 'moveit-joint'
        self.motion_request.velocity = 0.2  # 0.2  # 0.5  # 0.2
        self.motion_request.acceleration = 0.14  # 0.14  # 0.5  # 0.5 0.14
        self.motion_request.manipulator = 'manipulator'  # 'manipulator'
        self.motion_request.frame = 'world'
        self.motion_request.refine = False
        self.motion_request.target_joint = target_pose_home_rad
        # [-1.24, -1.68, -1.43,-1.69, 1.29, 2.70]
        self.close = spclose('/dev/000', 115200)
        self.open = spopen('/dev/000', 115200)
        self.open.send_data()
        #self.close.port=serial.Serial('/dev/ttyUSB0', 115200,timeout=0.05)
        #if not self.port.isOpen():
            #self.port.open()

        self.initialized = False

    def check_services(self):
        service_list = rosservice.get_service_list()
        if '/piiq/get_camera_data' in service_list and \
                '/piiq/excute_move' in service_list and \
                '/piiq/components_vision' in service_list and \
                '/piiq/load_environment' in service_list :
            return True
        return False


    def init_robot(self):
        if self.check_services():
            if self.initialized:
                return True
            else:
                # get services call ready
                self.excute_move = rospy.ServiceProxy(
                    '/piiq/excute_move', ExcuteMove)
                self.get_camera_data = rospy.ServiceProxy(
                    '/piiq/get_camera_data', GetCameraData)
                self.detect_components = rospy.ServiceProxy(
                    "/piiq/components_vision", DetectComponents)          
                self.load_environment = rospy.ServiceProxy(
                    "/piiq/load_environment", LoadEnvironment)
#                self.gripper = rospy.ServiceProxy(
#                    "/piiq/robotiq_c_control", RobotiqCCommand)

                response = self.excute_move(self.motion_request)
                if response.status.code < 0:
                    return False

                # load envirodetect_compnment from file
                rospy.sleep(5)
                self.load_environment(rospkg.RosPack().get_path(
                    'piiq_components') + '/config/scene_objects.yaml')

                self.initialized = True
                return True

        rospy.sleep(1)
        rospy.logwarn('necessary services not available yet!')
        return False

    def move_back_home(self):
        rospy.loginfo("moving back home position...")
        self.motion_request.type = "moveit-joint"
        target_pose_home = target_pose_home_rad
        target_pose_home[5] = target_pose_home[5] + 0.02*random.uniform(-1, 1)  # 0.1*random.uniform(-1, 1) 
        self.motion_request.target_joint = target_pose_home  # [-1.57, -1.67, -1.93, -0.33, 1.57, 1.57]   

        response = self.excute_move(self.motion_request)

        if response.status.code > 0:
            return True
        else:
            rospy.logerr(response.status.message)
            return False

    def sense_and_detect(self):
        self.camera_response = self.get_camera_data(self.camera_request)
        if self.camera_response.camera_data.status.code < 0:
            return False

        self.detect_comp = DetectComponentsRequest()
        self.detect_comp.camera_data = self.camera_response.camera_data
        self.detect_comp.camera_mat  = flatten(camera_matrix.tolist())
        self.detect_comp.distcoe  = flatten(distcoeffs.tolist())
        self.detect_comp.visualize = True
        print(self.detect_comp.camera_mat)
        print(self.detect_comp.distcoe)
        print("-------------")

        response = self.detect_components(self.detect_comp)  # self.camera_response.camera_data

        T = response.gripper_pose
        if abs(T[0] - 0.0) < 0.00001 and abs(T[1] - 0.0) < 0.00001 and abs(T[2] - 0.0) < 0.00001:
            return False
        
        if response.gripper_type == 0:
            self.gripper_size = GRIPPER_TYPE_0
            self.close.send_data()
        elif response.gripper_type == 1:
            self.gripper_size = GRIPPER_TYPE_1
            self.open.send_data()
        elif response.gripper_type == 2:
            self.gripper_size = GRIPPER_TYPE_2
            self.open.send_data()
        # fill-in self.motion_request
        self.motion_request.type = 'moveit-pose'
        print "*************"
        print response.gripper_pose
        print "*************"
        self.motion_request.target_pose = list(response.gripper_pose)
        self.motion_request.target_pose[2] -= 0.00
        print "XXXXXXX"
        print self.motion_request.target_pose
        print "XXXXXXX"
        self.motion_request.refine = True  # target_pose[2] += 0.05
        return True

    def home_and_detect(self):
        if not self.move_back_home():
            # rospy.sleep(1)
            return False

        if not self.sense_and_detect():
            # rospy.sleep(1)
            return False

        return True

    def move_to_pick(self):

        print self.motion_request.target_pose
        print self.motion_request.target_joint

        response = self.excute_move(self.motion_request)

        if response.status.code > 0:
            return True
        else:
            rospy.logerr(response.status.message)
            return False

    def grasp_and_check(self):

        self.motion_request.type = 'movel'
        self.motion_request.target_pose[2] += 0.15

        #self.close('/dev/ttyUSB1', 115200)

        if self.gripper_size == GRIPPER_TYPE_0:
            self.open.send_data()
        elif self.gripper_size == GRIPPER_TYPE_1:
            self.close.send_data()
        elif self.gripper_size == GRIPPER_TYPE_2:
            self.close.send_data()

        # lift-up
        #self.motion_request.target_pose[2] += 0.1
        response = self.excute_move(self.motion_request)

        if response.status.code > 0:
            return True
        else:
            rospy.logerr(response.status.message)
            return False

    def move_to_place(self):
        self.motion_request.type = "moveit-joint"

        self.motion_request.target_joint = target_pose_place_rad
        # [-49.85*math.pi/180.0, -116.39*math.pi/180.0,
        # -73.11*math.pi/180.0, -81.11*math.pi/180.0, 89.30*math.pi/180.0, 175.37*math.pi/180.0]

        response = self.excute_move(self.motion_request)

        if response.status.code < 0:
            rospy.logerr(response.status.message)
            return False

        #self.open('/dev/ttyUSB1', 115200)
        if self.gripper_size == GRIPPER_TYPE_0:
            self.close.send_data()
        elif self.gripper_size == GRIPPER_TYPE_1:
            self.open.send_data()
        elif self.gripper_size == GRIPPER_TYPE_2:
            self.open.send_data()
        

        return True


if __name__ == '__main__':
    try:
        PIIQCoordinatorNode()
       
    except KeyboardInterrupt:
        print "Shutting down piiq_groceries_coordinator node."
