#!/usr/bin/env python3

import rospy
from std_msgs.msg import String,Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from platform import python_branch
import traj_Gen as t
import sympy as sym
import numpy as np
#from sub import MyListener

class MyPublisher:
    """
    A publisher class to build and publish messages to the cmd_vel topic.

    Attributes:
        _anonymous (bool): flag to create an anonymous node
        _publisher (Publisher): Publisher object
        _message (Twist): Twist object for geometry_msgs/Twist messages
        _rate (rospy.timer.Rate)

    """

    def __init__(self, anonymous=False, rate=1):
        """
        Initialize object attributes.

        Args:
            anonymous: Flag to check whether or not we need to create 
            anonymous nodes.
            anonymous=True will give a unique id to the node
            anonymous=True allows you to create multiple instances of this node

            rate: Rate at which messages are published on a topic.

        Returns:
            None
        """
        self.anonymous = anonymous
        rospy.init_node("angleSpeed_pub", anonymous=self.anonymous)
        self.angleSpeed_Pub = rospy.Publisher("cmd_omega", Float64MultiArray, queue_size=10)
        self.angleSpeed = Float64MultiArray()
        self.rate = rospy.Rate(rate)
        self.robot_name = 'robotBookShelf'
        self.jacobian = t.trans2Jack()
        self.theta = t.theta_param()
        #self.gripper = MyListener().gripper
        # if rospy.has_param('/robotBookShelf/gripperSlider_Cont/command'):
        #     self.gripper = rospy.get_param('/robotBookShelf/gripperSlider_Cont/command')
        
    def linear2Angle(self, msg):
        """
        Takes in Linear values to compute angular and linear speeds of every link in robot

        Args:
            msg (_Twist_): velocity kinematics of gripper
            
        Return: Column vector of angular or linear speeds of respective joints
        """
        
        x_dot = msg.linear.x
        y_dot = msg.linear.y
        z_dot = msg.linear.z
        
        XX_dot =  sym.Matrix([x_dot, y_dot, z_dot, 0, 0, 0])
        XX_dot = XX_dot.reshape(6,1)
        
        # Initial joints orientation given
    
        Q_dot= [] # store joint link rotation terms 
        
        # Initial: qJnt[0] and qJnt[2] are prismatic, the rest are rotary
        qJnt = sym.Matrix([0,0,0,np.pi/2,np.pi/2,0,0,0,0])
        qJnt = qJnt.reshape(9,1)
        
        # Update Jacobian matrix with new theta values
        J_new = self.jacobian.subs(zip(self.theta,[qJnt[0,0],qJnt[1,0],qJnt[2,0],qJnt[3,0],qJnt[4,0],qJnt[5,0],qJnt[6,0],qJnt[7,0],qJnt[8,0]]))
        
        # Solve Q_dot link velocities
        Q_dot = sym.Inverse(J_new)*XX_dot
        
        self.angleSpeed.data = Q_dot 
        
        self.angleSpeed_Pub.publish(self.angleSpeed)
        self.rate.sleep()
        
#if __name__ == '__main__':
    