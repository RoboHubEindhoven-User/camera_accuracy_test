#!/usr/bin/env python
import rospy
import time
import tf
from geometry_msgs.msg import Pose, TwistStamped
from tf2_msgs.msg import TFMessage
from math import pi, sqrt, sin
from Gripper import Gripper
from SendMove import SendMove

class UR():
    def __init__(self):
        #Instances
        self.gripper = Gripper()
        self.m = SendMove()
        self.t = tf.TransformListener()
        self.armPose = ""

        #Subscribers
        rospy.Subscriber("/tf", TFMessage, self.getToolTF)
        rospy.Subscriber("tool_velocity", TwistStamped, self.getToolVelocity)

        #Open gripper
        self.gripper.moveGripper("Open")

        #Go to initial position
        self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('test3')))
        self.waitForArm()

    def onTF(self, link, state):
        """[This function is used to do pick and place based on TF positions]
        
        Arguments:
            link {String} -- [This is the TF name]
            state {String} -- ["Pick" or "Place"]
        """
        transform = self.t.lookupTransform("ur3/base", link, rospy.Time(0))
        rospy.loginfo_throttle(5, "Tranformation found for %s"%link)
        x = transform[0][0]
        y = transform[0][1]
        z = transform[0][2]
        qrx = transform[1][0]
        qry = transform[1][1]
        qrz = transform[1][2]
        qrw = transform[1][3]
        [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([qrx, qry, qrz, qrw])
        [rx,ry,rz] = self.m.euler2Rot(roll, pitch, yaw)
        blend_radius = 0.01

        #self.m.sendMove(self.m.buildMove('j', 'p', [x, y, z + 0.05, rx, ry, rz], blend_radius))        
        #self.waitForArmBlend([x, y, (z + 0.05), rx, ry, rz], blend_radius+0.02)
        #self.moveTool([x, y, z, rx, ry, rz])

        self.m.sendMove(self.m.buildBlendMove("l", "p", [x, y, z + 0.05, rx, ry, rz], [x, y, z, rx, ry, rz], blend_radius))
        self.waitForArmBlend([x, y, z + 0.05, rx, ry, rz], blend_radius)

        if state == "Pick":
            self.gripper.moveGripper("Close")
        elif state == "Place":
            self.gripper.moveGripper("Open")

        #self.moveTool([x, y, z + 0.05, rx, ry, rz])

        #----------Code for object following----------
        # pose = [x, y, z + 0.05 * sin(2.3*rospy.get_time()) + 0.05, rx, ry, rz]
        # array = list(pose)
        # a = 1.0
        # v = 1.0
        # t = 0.05
        # st = "servoj(get_inverse_kin(p%s), a=%s, v=%s, t=%s)"%(array, a, v, t)
        # self.m.sendMove(st)
        # time.sleep(t)

    def moveTool(self, pose):
        """[This function is used to send the tool to a position in linear tool space]
        
        Arguments:
            pose {List} -- [x,y,z,rx,ry,rz]
        """
        if self.isPoseReachable(pose):
            self.m.sendMove(self.m.buildMove('l', 'p', pose))
            self.waitForArm()

    def goHolder(self, holderPlace, picking):
        """[This function is used to do pick and place task to/from a holder place on Suii]
        
        Arguments:
            holderPlace {String} -- ["HOLDER_1", "HOLDER_2", "HOLDER_3"]
            picking {String} -- ["Pick" or "Place"]
        """
        rospy.loginfo("Going to holderplace %s", holderPlace)
        if holderPlace == "HOLDER_1":
            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalPreHolder1')))
            self.waitForArm()
            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalHolder1')))
            self.waitForArm()
            
            if picking == "Pick":
                self.gripper.moveGripper("Close")
            elif picking == "Place":
                self.gripper.moveGripper("Open")

            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalPreHolder1')))
            self.waitForArm()

        elif holderPlace == "HOLDER_2":
            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalPreHolder2')))
            self.waitForArm()
            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalHolder2')))
            self.waitForArm()
            
            if picking == "Pick":
                self.gripper.moveGripper("Close")
            elif picking == "Place":
                self.gripper.moveGripper("Open")

            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalPreHolder2')))
            self.waitForArm()
           
        elif holderPlace == "HOLDER_3":
            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalPreHolder3')))
            self.waitForArm()
            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalHolder4')))
            self.waitForArm()

            if picking == "Pick":
                self.gripper.moveGripper("Close")
            elif picking == "Place":
                self.gripper.moveGripper("Open")

            self.m.sendMove(self.m.buildMove('j', '', self.m.getPos('finalPreHolder2')))
            self.waitForArm()

    def waitForArm(self):
        """[This function waits till the arm reached it's position, by waiting till tool velocity is zero.]
        """
        time.sleep(0.3) #0.3 value for real ur3 otherwise arm has no time to get speed
        while (abs(self.xVel) > 0.0001 or abs(self.yVel) > 0.0001 or abs(self.zVel) > 0.0001) and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "Robot is moving to position")
        rospy.loginfo("Robot reached position")

    def waitForArmBlend(self, pose, radius):
        while abs(sqrt((self.tool[0]-pose[0])**2 + (self.tool[1]-pose[1])**2 + (self.tool[2]-pose[2])**2)) > radius:
            rospy.loginfo_throttle(1, "Moving to blend radius")
        rospy.loginfo("Robot reached blend radius")

    def isPoseReachable(self, pose):
        """[This function checks if the goal position is within reach of UR3]
        
        Arguments:
            pose {list} -- [x,y,z,rx,ry,rz]
        
        Returns:
            [bool] -- [True if is within reach, else false]
        """
        maxReach = 0.538
        minReach = 0.064
        x = pose[0] 
        y = pose[1]
        z = pose[2]
        distance = sqrt(x**2 + y**2 + z**2)
        if distance < maxReach and distance > minReach:
            rospy.loginfo("Positon in reach. Distance = %s", distance)
            return True
        else:
            rospy.logwarn("Out of reach, command not sended. Distance = %s", distance)

    def getToolVelocity(self, msg):
        self.xVel = msg.twist.linear.x
        self.yVel = msg.twist.linear.y
        self.zVel = msg.twist.linear.z
        self.rxVel = msg.twist.angular.x
        self.ryVel = msg.twist.angular.y
        self.rzVel = msg.twist.angular.z

    def getToolTF(self, msg):
        # if len(msg.transforms) == 0:
        #     return
        if msg.transforms[-1].child_frame_id == "ur3/tool0_controller":
            x = msg.transforms[-1].transform.translation.x
            y = msg.transforms[-1].transform.translation.y
            z = msg.transforms[-1].transform.translation.z
            qx = msg.transforms[-1].transform.rotation.x
            qy = msg.transforms[-1].transform.rotation.y
            qz = msg.transforms[-1].transform.rotation.z
            qw = msg.transforms[-1].transform.rotation.w
            [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
            [rx, ry, rz] = self.m.euler2Rot(roll, pitch, yaw)
            self.tool = [x,y,z,rx,ry,rz]
