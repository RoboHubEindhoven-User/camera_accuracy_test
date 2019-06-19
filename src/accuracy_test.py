#!/usr/bin/env python
#
# Created by Ruben Arts, Remco Kuijpers and Jeroen Bongers in name of RoboHub Eindhoven
#

import rospy
import cv2 as cv
from SendMove import SendMove
from UR import UR
from post_processing_test_measuring_precision import PostProcessingTest
import time
import tf2_ros
import csv
import os
from std_msgs.msg import String

rospy.init_node('accuracy_test')
stepl = 0.015
m = SendMove()
arm = UR()
vision = PostProcessingTest()
#ls = tf.TransformListener()
#tr = tf.Transformer()
adress = '/home/suii/catkin_ws/src/camera_accuracy_test/results/'
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

world_poses = []
camera_poses = []
ur3Base_poses = []
last_camera = []


def save_data(base_filename):
    """This function saves all the position/orientation lists to a file
    
    Arguments:
        base_filename {string} -- [Folder to save the text files]
    """
    world_pose_filename = base_filename + "_world_poses"
    camera_pose_filename = base_filename + "_camera_poses"
    ur3Base_pose_filename = base_filename + "_ur3Base_poses"
    base_filename = base_filename + "/"
    first_row = ['xyzrpyw']
    try_false = True
    while(try_false):
        if not os.path.exists(adress + base_filename):
            os.mkdir(adress + base_filename)
            print "Directory " + adress + base_filename +" is created "
            try_false = False
        else:
            print("Directory ", base_filename, " already exists.")
            base_filename = raw_input(" Give new filename :  ")

    with open( adress + base_filename + world_pose_filename, 'w') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(first_row)
        writer.writerows(world_poses)
    writeFile.close()
    with open( adress + base_filename + camera_pose_filename, 'w') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(first_row)
        writer.writerows(camera_poses)
    writeFile.close()
    with open( adress + base_filename + ur3Base_pose_filename, 'w') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(first_row)
        writer.writerows(ur3Base_poses)
    writeFile.close()

def filter(pose):
    """This function is used to filter the TF data for position and orientation
    
    Arguments:
        pose {transformStamped} -- [TFMessage to be filtered]
    
    Returns:
        {list} -- [filtered list containing position and orientation]
    """
    trans = pose.transform #filter for only translation and rotation
    position = [trans.translation.x, trans.translation.y, trans.translation.z, trans.rotation.x,
                trans.rotation.y, trans.rotation.z, trans.rotation.w]
    return position
    
def record_tf():
    """This function adds the TF data to lists (x,y,z,qx,qy,qz,qw)
    """
    msg = 'now'
    vision.postprocessing_test("object") #service to call vision
    rospy.sleep(0.5) 
    now = rospy.Time(0) # pick last tf

    #check TF's
    world = tfBuffer.lookup_transform("base_link", "object", now)
    camera = tfBuffer.lookup_transform("camera", "object", now)
    ur3Base = tfBuffer.lookup_transform("ur3/base", "object", now)

    #filter to save only important data
    if camera is not last_camera:
        world_poses.append(filter(world))
        camera_poses.append(filter(camera))
        ur3Base_poses.append(filter(ur3Base))

    last_camera = camera


def accuracy_test():
    """This function runs the arm sequence and runs image processing at 40 different positions.
    """
    m.sendMove(m.buildMove('j', '', m.getPos("test3")))
    arm.waitForArm()
    tool = arm.tool

    for i in range(0, 10, 1):
        arm.moveTool([tool[0], tool[1]+stepl*(i-5), tool[2], tool[3], tool[4], tool[5]])
        record_tf()

    for i in range(0, 10, 1):
        arm.moveTool([tool[0]+stepl*(i-5), tool[1], tool[2], tool[3], tool[4], tool[5]])
        record_tf()

    for i in range(0, 10, 1):
        arm.moveTool([tool[0]+stepl*(i-5), tool[1]+stepl*(i-5), tool[2], tool[3], tool[4], tool[5]])
        record_tf()

    for i in range(0, 10, 1):
        arm.moveTool([tool[0]+stepl*(i-5), tool[1]-stepl*(i-5), tool[2], tool[3], tool[4], tool[5]])
        record_tf()

while not rospy.is_shutdown():
    """[Commands to run the whole code]
    """
    foldername = raw_input('Give the foldername for saving: ')
    accuracy_test()
    save_data(base_filename=foldername)
    print("Done testing")
    rospy.spin()
