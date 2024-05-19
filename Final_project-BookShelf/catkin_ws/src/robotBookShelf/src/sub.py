#!/usr/bin/env python3

import rospy
from std_msgs.msg import String,Float64, _Float64MultiArray
from geometry_msgs.msg import Twist
from platform import python_branch
import traj_Gen as t
import sympy as sym
import numpy as np
from pub import MyPublisher


class MyListener:
    """
    A Listener/subscriber class to read messages from the odom topic.

    Attributes:
        _anonymous (bool): flag to create an anonymous node
        _robot_name (str): Name of the robot

    """
    def __init__(self, anonymous=False):
        """
        Initialize object attributes.

        Args:
            anonymous: Flag to check whether or not we need to create 
            anonymous nodes.
            anonymous=True will give a unique id to the node
            anonymous=True allows you to create multiple instances of this node

        Returns:
            None
        """
        self.robot_name = "robotBookShelf"
        self.anonymous = anonymous
        rospy.init_node("vel_sub",anonymous=self.anonymous)
        self.gripper = rospy.Subscriber("/robotBookShelf/gripperSlider_Cont/command", Twist, self.moveRot_callback)
        self.gripperPinch = rospy.Subscriber("/robotBookShelf/gripFinger1_Cont/command", Float64, self.pinch_callback)
        self.gripperPinch2 = rospy.Subscriber("/robotBookShelf/gripFinger2_Cont/command", Float64, self.pinch_callback)
        self.jacobian = t.trans2Jack()
        self.theta = t.theta_param()
        
        # setup subscribers and callbacks
        rospy.spin()
        

        # while not rospy.is_shutdown():
        #     rospy.loginfo("Hello from main thread")

    def moveRot_callback(self, msg):
        """

        Args:
            msg: Messages received on the Topic
        """
        
        x_dot = msg.linear.x
        y_dot = msg.linear.y
        z_dot = msg.linear.z
        th_dot = msg.angular.z 
        
        rospy.loginfo("For {} : Pose is [{},{},{}] and Rotation is [{}]"
                      .format(self.robot_name, x_dot, y_dot, z_dot, th_dot))

    def pinch_callback(self, msg):
        """

        Args:
            msg: Messages received on the Topic
        """
        
        pinch_dot = msg
        rospy.loginfo("{} pincher is moving at: {} m/s"
                      .format(self.robot_name, pinch_dot))
        

if __name__ == '__main__':
    velInput = MyListener()
    angleOutput = MyPublisher()
    angleOutput.linear2Angle(velInput.gripper)
    