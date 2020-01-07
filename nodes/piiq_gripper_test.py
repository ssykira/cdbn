#!/usr/bin/env python

# <read me>
# roscore
# rosrun ipr_gripper_robotiq_c robotiq_c_control.py
# rosrun piiq_components piiq_gripper_test.py

import rospy
import rospkg
import rosservice
import yaml
import random
import math

from pi_trees_lib.pi_trees_lib import CallbackTask, TaskStatus, print_tree
from pi_trees_lib.pi_trees_lib import Sequence, UntilSuccess, loop

from piiq_msgs.srv import GetCameraData, GetCameraDataRequest, DetectComponents
from piiq_msgs.srv import ExcuteMove, ExcuteMoveRequest, LoadEnvironment
from piiq_msgs.msg import CModel_robot_output as outputMsg
from piiq_msgs.srv import RobotiqCCommand

import time


class PIIQGripperServiceNode:
    def __init__(self):
        rospy.init_node('piiq_gripper_test', anonymous=False)
        self.init_gripper_status = outputMsg()
        self.gripper = None
        self.init_self_data()
        self.initialized = False

        SEQ_BEHAVE = Sequence("piiq_components_demo")

        RUN_INIT_GRIPPER = CallbackTask("init robotiq gripper", self.init_gripper)
        SUC_INIT_ROBOT = UntilSuccess("init_until_success", max_attempts=50)
        SUC_INIT_ROBOT.add_child(RUN_INIT_GRIPPER)

        SEQ_GRIP = Sequence("grip and release test")
        RUN_GRASP = CallbackTask("grip test", self.gripper_grasp)
        RUN_RLEASE = CallbackTask("release test", self.gripper_release)
        SEQ_GRIP.add_child(RUN_GRASP)
        SEQ_GRIP.add_child(RUN_RLEASE)
        LOOP_RUN = loop(SEQ_GRIP, iterations=10)

        SEQ_BEHAVE.add_child(SUC_INIT_ROBOT)
        SEQ_BEHAVE.add_child(LOOP_RUN)

        print "Behavior Tree Structure"
        print_tree(SEQ_BEHAVE, use_symbols=True)

        # Run the tree
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            status = SEQ_BEHAVE.run()
            # print TaskStatus.SUCCESS, status
            if status == TaskStatus.SUCCESS:
                rospy.loginfo("Finished running piiq_components_coordinator.")
                break
            r.sleep()

    def init_gripper(self):
        if self.check_services():
            if self.initialized:
                return True
            else:
                # get services call ready
                self.gripper = rospy.ServiceProxy("/piiq/robotiq_c_control", RobotiqCCommand)

                self.init_gripper_status.rACT = 0  # 0 ,reset, 1, active
                response = self.gripper(self.init_gripper_status)
                rospy.sleep(2)

                self.init_gripper_status.rACT = 1  # 0 ,reset, 1, active
                response = self.gripper(self.init_gripper_status)
                rospy.sleep(5)
                #
                self.init_gripper_status.rACT = 1  # active
                self.init_gripper_status.rPR = 125  # close
                self.init_gripper_status.rSP = 255
                self.init_gripper_status.rFR = 150
                response = self.gripper(self.init_gripper_status)
                #
                # self.gripper_grasp()

                if not response.success:
                    rospy.logwarn("init gripper failed!")
                    return False
                rospy.loginfo("init gripper success")
                self.initialized = True
                return True
        rospy.sleep(1)
        rospy.logwarn('necessary services not available yet!')
        return False

    def gripper_grasp(self):
        self.init_gripper_status.rPR = 0
        response = self.gripper(self.init_gripper_status)
        if not response.success:
            return False
        return True

    def gripper_release(self):
        self.init_gripper_status.rPR = 150
        response = self.gripper(self.init_gripper_status)
        if not response.success:
            return False
        return True

    def init_self_data(self):
        # default: activate/open
        self.init_gripper_status = outputMsg()
        self.init_gripper_status.rACT = 1  # gripper activation, if 0,reset
        self.init_gripper_status.rGTO = 1  # go to position request
        self.init_gripper_status.rATR = 0  # auto releasee
        self.init_gripper_status.rPR = 100  # position request
        self.init_gripper_status.rSP = 20  # speed request
        self.init_gripper_status.rFR = 30  # force request

    def check_services(self):
        service_list = rosservice.get_service_list()
        if '/piiq/robotiq_c_control' in service_list:
            return True
        return False


if __name__ == '__main__':
    try:
        PIIQGripperServiceNode()
    except KeyboardInterrupt:
        print "Shutting down piiq_groceries_coordinator node."
