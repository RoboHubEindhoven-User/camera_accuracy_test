#!/usr/bin/env python
import rospy
import time
from SendMove import SendMove
from ur_driver.io_interface import set_analog_out, set_states

class Gripper():
    def __init__(self):
        self.s = SendMove()

        #Start gripper service-server
        set_states()

        #Set analog output to voltage mode
        self.s.sendMove("set_analog_outputdomain(0, 1)")

    def moveGripper(self, pos):
        """This function can open or close the gripper.
        
        Arguments:
            pos {String} -- ["Open" or "Close"]
        """
        if pos == "Open":
            set_analog_out(0, 0.4) #Open gripper
            time.sleep(1)
        elif pos == "Close":
            set_analog_out(0, 0.0) #Close gripper
            time.sleep(1)
        else:
            rospy.loginfo("Invalid input: %s", str(pos))