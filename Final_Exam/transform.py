#!/usr/bin/python

# ========================================
# ENPM662 Fall 2022: Introduction to Robot Modeling
# Final Exam : 3D - 3D Registration
#
# Author: Yoseph Kebede
# ========================================
# Run as 'python finExam.py'
# Press ESC for exit

import csv
import math as m
from sympy import pprint
import matplotlib.pyplot as plt
import numpy as np
import sympy as sym
from mpl_toolkits import mplot3d

def readTable(input):
    """
    Read input table from csv and save as array

    Args:
        input (_csv file_): csv file with data of set of points
        
    return: array containing points in data set
    """
    # referenced from: https://realpython.com/python-csv/ 
    X = []
    X1 = []
    
    with open(input, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        #last_line = csv_file.readlines()[-1]
        
        for row in csv_reader:
            if line_count > 1:
                x = float(row[0])
                y = float(row[1])
                z = float(row[2])
                
                #print(x, y, z)
                
                #saving homogenous variables by dividing z
                X.append([x/z, y/z, 1])     
                
                x1 = float(row[3])
                y1 = float(row[4])
                z1 = float(row[5])
                
                #saving homogenous variables by dividing z1
                X1.append([x1/z1, y1/z1, 1])
                
            line_count += 1
    
    # X = [x1, y1, 1]       X1 = [x1, y1, 1]
    #     [x2, y2, 1]            [x2, y2, 1]
    #     [., ., .]              [., ., .]
    #     [xn, xn, 1]             [xn, xn, 1]
    X = np.vstack(X)
    X1 = np.vstack(X1)
    
    return X, X1

def homography(pts1, pts2):
    """
    Finds homography between point sets 1 and point sets 2
    
    Args:
        pts1 (array) : set of points representing first point cloud set
        pts2 (array) : set of points representing second point cloud set
        
    Return: homography matrix 
    """  
    # Referenced code from ENPM673 class assignment
    
    # Create the homogrophy matrix and perform homography
    # A = [[pts1[:,0],pts1[:,1],1,0,0,0,-x1[0]*x11[0],-x1[1]*x11[0],-x11[0]],
	# 	 [0,0,0,x1[0],x1[1],1,-x1[0]*x11[1],-x1[1]*x11[1],-x11[1]],
	# 	 [x2[0],x2[1],1,0,0,0,-x2[0]*x12[0],-x2[1]*x12[0],-x12[0]],
	# 	 [0,0,0,x2[0],x2[1],1,-x2[0]*x12[1],-x2[1]*x12[1],-x12[1]],
	# 	 [x3[0],x3[1],1,0,0,0,-x3[0]*x21[0],-x3[1]*x21[0],-x21[0]],
	# 	 [0,0,0,x3[0],x3[1],1,-x3[0]*x21[1],-x3[1]*x21[1],-x21[1]],
	# 	 [x4[0],x4[1],1,0,0,0,-x4[0]*x22[0],-x4[1]*x22[0],-x22[0]],
	# 	 [0,0,0,x4[0],x4[1],1,-x4[0]*x22[1],-x4[1]*x22[1],-x22[1]]]
    
    A = []
    
    for i in range(len(pts1[:,0])):
        A.append([[pts1[i,0],pts1[i,1],1,0,0,0,-pts1[i,0]*pts2[i,0],-pts1[i,1]*pts2[i,0],-pts2[i,0]],
                 [0,0,0,pts1[i,0],pts1[i,1],1,-pts1[i,0]*pts2[i,1],-pts1[i,1]*pts2[i,1],-pts2[i,1]]])
        
    A = np.vstack(A)
    
    # SVD(A) = U E V.T
    # U = np.matmul(A,np.transpose(A))
    # V = np.matmul(np.transpose(A),A)
    
    U, EE, V =np.linalg.svd(A)
    
    eV0, eUt = np.linalg.eig(U)
    eV1,eVt = np.linalg.eig(V)
    
    # Creating identity matrix with the Eigenvalues

    eVt =np.column_stack((eVt))
    minEig = np.argmin(eV1,axis=0)
        
    # SVD can now be computed as the column of V.T corresponding to eigenvector
    # with smallest eigenvalue

    # Eigen vector with smalles eigenvalue 
    svdA= np.vstack(eVt[minEig])
    
    #svdA = np.vstack(svdA)
    
    # Normalize vector by 9th element in vector
    #svdA = svdA / svdA[8]
    
    # Create a 3X3 homography matrix from SVD(A)
    H = svdA.reshape(3,3)
    H = np.float32(H)
    
    return H
    

def main():
    """
    Estimating 3D rigit transform that optimally aligns corresponding
    point in a set.
    
    """
    
    # List of datasets to comput homography in 
    datasets = ['dataset1.csv', 'dataset2.csv', 'dataset3.csv']
    error = []
    for setOfPts in datasets:
    
        # Start by reading datasets
        set1, set2 = readTable(setOfPts)
        
        # compute homography between two points
        H = homography(set1, set2)
        
        # Finding mean error after aligning point clouds
        sum = 0
        N = len(set1[:,0])
        
        for i in range(len(set1)):
            x1 = np.vstack(set1[i]) 
            x2 = np.matmul(H,x1)
            sqrt_diff = ((set2[i][0] - x2[0])**2 + (set2[i][1] - x2[1])**2 +
                         (set2[i][2] - x2[2])**2)
            
            sum = sum + np.sqrt(sqrt_diff)
            
        err = sum / N
        
        print("For " + setOfPts + " mean error after point cloud alignation is: \n")
        print(err, "\n")
        
        error.append(err)
    
if __name__ == '__main__':
    main()
    
    
    
##################### Code Ends Here #####################