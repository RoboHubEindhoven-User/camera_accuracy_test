#!/usr/bin/env python
import sys
import rospy
import std_msgs
import yaml
from std_msgs.msg import String
import math
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class SendMove (object):
    def __init__(self):
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=10, latch=True)
        self.addr = '/home/suii/catkin_ws/src/suii_manipulation/yaml/poses.yaml'

    def buildMove(self, moveType, space, pose, radius=0):
        """This function builds a string for sending a move command to URScript
        
        Arguments:
            moveType {string} -- [description]
            space {string} -- [description]
            pose {string} -- [description]
        
        Keyword Arguments:
            radius {int} -- [description] (default: {0})
        
        Returns:
            [type] -- [description]
        """
        if moveType == "l":
            acceleration = 0.3
            speed = 0.3
        else:
            acceleration = 1.0  #Joint acceleration in rad/s^2
            speed = 1.0 #Joint speed in rad/s
        time = 0 #Time the move must take
        array = list(pose)
        sendable = "move%s(%s%s, a=%s, v=%s, t=%s, r=%s)"%(moveType, space, array, acceleration, speed, time, radius)
        return sendable

    def buildBlendMove(self, moveType, space, pose, secondPose, radius=0):
        if moveType == "l":
            acceleration = 0.3
            speed = 0.3
        else:
            acceleration = 1.0  #Joint acceleration in rad/s^2
            speed = 1.0 #Joint speed in rad/s
        time = 0 #Time the move must take
        array = list(pose)
        array2 = list(secondPose)
        sendable = "move%s(%s%s, a=%s, v=%s, t=%s, r=%s)\nmove%s(%s%s, a=%s, v=%s, t=%s, r=%s)"%(moveType, space, array, acceleration, speed, time, radius, moveType, space, array2, acceleration, speed, time, 0)
        return sendable

    def euler2Rot(self, roll, pitch, yaw):
        yawMatrix = np.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
        ])

        pitchMatrix = np.matrix([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        rollMatrix = np.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
        ])

        R = yawMatrix * pitchMatrix * rollMatrix

        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
        multi = 1 / (2 * math.sin(theta))

        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta
        rot = [rx, ry, rz]
        return rot

    def getPos(self, pos_name):
        file = open(self.addr, 'r')
        y = yaml.load(file)
        file.close()
        poses = y
        wanted_pose = poses.get(pos_name)
        return wanted_pose

    def sendMove(self, string):
        st = String()        
        st.data = string
        self.pub.publish(st)
        #print st.data

    def searchStringInYaml(self,string):
        with open(self.addr) as f:
            contents = f.read()
            if string in contents:
                return True
            else:
                return False
