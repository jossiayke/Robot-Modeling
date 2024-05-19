#!/usr/bin/python

# ========================================
# ENPM662 Fall 2022: Introduction to Robot Modeling
# Project 2: Bookshelf Robot Trajectory Generation
#
# Author: Yoseph Kebede
# ========================================
# Run as 'python traj_Gen.py'
# Press ESC for exit

import copy
import math as m
from sympy import pprint
from pyclbr import Function
from this import d
import matplotlib.pyplot as plt
import numpy as np
import sympy as sym
from mpl_toolkits import mplot3d


"""Global Definitions (link length, distance and angle between frames,
    ngle between common normal"""

# Change units to / 1000 (m)

# variable link length for ith frame 
a1 = .025
a2 = .145
a3 = 0   # Max val is 140
a4 = .250
a5 = .250
a6 = .200
a7 = .075
a8 = .605
a9 = .050

#A = [a, a, a3, -a3, 0, a3, 0]
A = [a1, a2, a3, a4, a5, a6, a7, a8, a9]
# converting to meter

# vertical distance between joints for ith frame
d1 = sym.symbols('d1')
d2 = 0  
d3 = sym.symbols('d3')
d4 = 0.170 
d5 = 0.150
d6 = 0.350
d7 = 0.075
d8 = 0
d9 = 0 #sym.symbols('d9') Extending wrist

D = [d1, d2, d3, d4, d5, d6, d7, d8, d9]

# Angle between Z axes for ith frame (Always constant)
alpha1 = 0
alpha2 = 0  
alpha3 = np.pi/2 
alpha4 = -np.pi/2 
alpha5 = 0
alpha6 = np.pi/2
alpha7 = -np.pi/2
alpha8 = 0
alpha9 = -np.pi/2

alpha = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7, alpha8, alpha9]


# Angles of rotation for each joint in ith frame
th1 = 0
th2 = sym.symbols('th2')
th3 = 0    # fixed angle, kept for reference
th4 = sym.symbols('th4')
th5 = sym.symbols('th5')
th6 = sym.symbols('th6')
th7 = 0
th8 = sym.symbols('th8')
th9 = 0

th = [th1,th2,th3,th4,th5,th6,th7,th8,th9]

def a_param():
    
    # variable link length for ith frame 
    a1 = .025
    a2 = .145
    a3 = 0   # Max val is 140
    a4 = .250
    a5 = .250
    a6 = .200
    a7 = .075
    a8 = .605
    a9 = .050

    #A = [a, a, a3, -a3, 0, a3, 0]
    A = [a1, a2, a3, a4, a5, a6, a7, a8, a9]
    # converting to meter
    
    return A

def d_param():
    
    # vertical distance between joints for ith frame
    d1 = sym.symbols('d1')
    d2 = 0  
    d3 = sym.symbols('d3')
    d4 = 0.170 
    d5 = 0.150
    d6 = 0.350
    d7 = 0.075
    d8 = 0
    d9 = 0 #sym.symbols('d9') Extending wrist

    D = [d1, d2, d3, d4, d5, d6, d7, d8, d9]
    
    return D

def alpha_param():
    
    # Angle between Z axes for ith frame (Always constant)
    alpha1 = 0
    alpha2 = 0  
    alpha3 = np.pi/2 
    alpha4 = -np.pi/2 
    alpha5 = 0
    alpha6 = np.pi/2
    alpha7 = -np.pi/2
    alpha8 = 0
    alpha9 = -np.pi/2

    alpha = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7, alpha8, alpha9]
    
    return alpha

def theta_param():
    
    # Angles of rotation for each joint in ith frame
    th1 = 0
    th2 = sym.symbols('th2')
    th3 = 0    # fixed angle, kept for reference
    th4 = sym.symbols('th4')
    th5 = sym.symbols('th5')
    th6 = sym.symbols('th6')
    th7 = 0
    th8 = sym.symbols('th8')
    th9 = 0

    th = [th1,th2,th3,th4,th5,th6,th7,th8,th9]
    
    return th
    
def comp_Mat_A(alpha,D,A,theta, i):
    """
    Computes the full transformation from base to end effector

    Args:
        theta (_float/sym_): angle between common normal
        alpha (_float/sym_): angle between axis of rotation
        D (_list_): holds distance d parameter between frames
        AA (_tuple_) : holds transformation matrices between frames
        
    return: Transformation matrix
    """
    
    A = sym.Matrix([[sym.cos(theta[i]), -sym.sin(theta[i])*sym.cos(alpha[i]),sym.sin(theta[i])*sym.sin(alpha[i]), A[i]*sym.cos(theta[i])],
                    [sym.sin(theta[i]), sym.cos(theta[i])*sym.cos(alpha[i]),-sym.cos(theta[i])*sym.sin(alpha[i]), A[i]*sym.sin(theta[i])],
                    [0, sym.sin(alpha[i]), sym.cos(alpha[i]), D[i]],
                    [0, 0, 0, 1]])
    
    
    return A    
    
def full_Trans(idx):
    """
    Computes the full transformation matrix from two given frames using the successive matrices between frames 

    Args:
        
        idx: number of recursive multiplication to get Transfromation
             between base frame and idx-1 th frame
    
    return: Computed full transformation matrix
    """
    
    tr_full = sym.Identity(4)
    
    for i in range(idx):
        AA = comp_Mat_A(alpha,D,A,th, i)
        tr_full = tr_full * AA 
    
    
    return tr_full


def comp_Jacob(transf_1st, transf_2nd, idx=1):
    """
    Computes Jacobian matrix from one frame to next

    Args:
        transf_1st (_matrix_): transformation upto first frame
        transf_2nd (_matrix_): transfromation upto end effector
        idx (_int_): to get Jacobian for base frame
        
    return: Jacoian matrix
    """
    
    # Find the Jacobian Matrix for the full transformation
    # J_n = [Z_n-1 X (O_eff - O_n-1)]
    #       [       Z_n-1         ]

    O_eff = sym.Matrix([transf_2nd[0,3], transf_2nd[1,3], transf_2nd[2,3]])
        
    Z_n_1 = sym.Matrix([transf_1st[0,2], transf_1st[1,2], transf_1st[2,2]])
    
    O_n_1 = sym.Matrix([transf_1st[0,3], transf_1st[1,3], transf_1st[2,3]])
    
    J_v = Z_n_1.cross(O_eff - O_n_1)
    
    J_w = Z_n_1
    
    J = sym.Matrix.vstack(J_v,J_w)

    return J

def trans2Jack():
    """
    Computes Full Jacobian of Robot
    
    """
    
    #A,D, alpha, th = initializer() 
       
    # Transformation from base frame to end effector symbolically
    # T = 0_T_1 * 1_T_2 * 2_T_3 * 3_T_4 * 4_T_5 * 5_T_6 * 6_T_7 * 7_T_8
    # * 8_T_9 * 9_T_10   
    trFull = full_Trans(len(th))
    
    """ 2. Find Jacobian Matrix
    
    theta_x is the theta angle for joint x
    """
    
    # print(trans_Full)   # Uncomment line to see output on terminal
    
    J = sym.matrices.zeros(6,1)
    
    # iterative process continues until Jacobian matrix for all joints is obtained
    i =0
    
    L = len(th) # total joints in apparatus
    
    while i < L:
        # if i == 2:
        #     i+=1
        #     continue
        
        trans_preF = full_Trans(i)
        
        J_i = comp_Jacob(trans_preF, trFull)
        #print(sym.shape(J),sym.shape(J_i))
        J = sym.Matrix.hstack(J,J_i)
        
        i+=1    
        
    J = J[:,1:]
    
    return J

def main():
    """
    Velocity Kinematics: Bookshelf robot
    1. Receive end effector velocity command
    1. Find transformation between base frame and end effector 
    
    return: Joint velocity command for each joint frame
    """
    
    """
    Assumptions: 
    1) the frame on the end effector is translated upto the base of robot
    2) theta_1, theta_3, theta_4, theta_6, theta_10 are locked 
       but since links are displaced translationally still considered as new frames
    """
    
    """ 1. Find transformation equation"""
    
    J = trans2Jack()
    sym.pprint(J)
    print(sym.shape(J))
    return
    trFull = full_Trans(len(th))
    
    # Print Symbolic Jacobian Matrix to terminal
    #sym.pprint(J)
    
    """ Equations of parameters of circle from base frame
    
    x = 67.9 # units are in cms
    y = 10 * sym.cos(phi)
    z = 10 * sym.sin(phi) + 82.5"""
    
    # Parametrized time to track pen path
    t= sym.symbols('t')
    
    # Angluar speed of pen when drawing circle
    phi_dot = 2 * np.pi / 5  # 360 deg / 5 secs 
    
    # Hand computed derivative of parameters
    x_dot = 0
    #y_dot = sym.simplify(-10 * phi_dot * sym.sin(phi))
    y_dot = -0.1*phi_dot*sym.cos(t*phi_dot)
    #z_dot = sym.simplify(10 * phi_dot * sym.cos(phi))
    z_dot = -0.1*phi_dot*sym.sin(t*phi_dot)
    
    # Let number of sample points used in linearization be 50
    N = 500  # might change for better accuracy
    del_t = float(200/N)

    # Initial joints orientation given
    
    Q_dot= [] # store joint link rotation terms 
    XX_dot = [] # store joint velocity terms
    
    # Storing transformed points using base frame
    pts = []
    
    """Create figure to plot points"""
    fig = plt.figure(figsize=(16,9))
    ax = plt.axes(projection = "3d")
    ax.set_title('Plotting Circle from Robot Base frame')
    
    # # Initial: Replace theta_4 = 90 and theta_6 = 180, rest theta values are zero
    qJnt = sym.Matrix([0,0,np.pi/2,0,np.pi,0])
    qJnt = qJnt.reshape(6,1)
    
    t_var = 0 # initial time
    
    while t_var < 200:
        
        # Solve ydot and zdot with updated time
        y_dotEv = y_dot.subs(t,t_var)
        z_dotEv = z_dot.subs(t,t_var)
               
        
        # Create velocity vector for pen trajectory
        XX_dot =  sym.Matrix([x_dot, y_dotEv, z_dotEv, 0, 0, 0])
        XX_dot = XX_dot.reshape(6,1)
        
        # Update Jacobian matrix with new theta values
        J_new = J.subs(zip([th1,th2,th4,th5,th6,th7],[qJnt[0,0],qJnt[1,0],qJnt[2,0],qJnt[3,0],qJnt[4,0],qJnt[5,0]]))
        
        # Solve Q_dot link velocities
        Q_dot = sym.Inverse(J_new)*XX_dot
        
        # Replace theta in T_full matrix with new link rotation values        
        trf_New = trFull.subs(zip([th1,th2,th4,th5,th6,th7],[qJnt[0,0],qJnt[1,0],qJnt[2,0],qJnt[3,0],qJnt[4,0],qJnt[5,0]]))
         
        # Integrate
        qJnt = qJnt + Q_dot * del_t
        
        # Update parametrized time
        t_var = t_var + del_t
        
        # Store newly transformed points
        pt = [trf_New[0,3], trf_New[1,3], trf_New[2,3]]
        pts.append(pt)
       
    # Plot points   
    pts = np.array(pts,dtype=np.float)
    ax.plot(pts[:,0], pts[:,1], pts[:,2], color="green")
    ax.set_xlabel('x-axis')
    ax.set_ylabel('y-axis')
    ax.set_zlabel('z-axis')
    ax.set_xlim(0, 0.70) 
    ax.set_ylim(-0.70, 0.70) 
    ax.set_zlim(0, 1) 
    
    plt.show()
    
    plt.savefig('results/circlePlot_test.jpg')
    plt.clf()
    
if __name__ == '__main__':
    main()
    
    
    
##################### Code Ends Here #####################