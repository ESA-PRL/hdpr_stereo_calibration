#!/usr/bin/env python

"""
This script transforms the cameras to the center of the rover.

The cameras are assumed to be in the following order:
cam0: BB2 left
cam1: BB2 right
cam2: BB3 left
cam3: BB3 center
cam4: BB3 right
cam5: pancam left
cam6: pancam right
"""

__author__ = "Martin Azkarate, Levin Gerdes, Karl Kangur, Marco Pagnamenta"

import yaml
import re
import numpy as np
import os.path
from decimal import *
from Tkinter import Tk
from tkFileDialog import askopenfilename
import transformations as TF
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1.5,1.5)
ax.set_ylim(-1.5,1.5)
ax.set_zlim(-0.5,1.5)

# precision for Decimal
getcontext().prec = 16

## Ask for calibration results
#print("Please choose the files containing the Kalibr calibration output\n")
#Tk().withdraw()
#calibResults = askopenfilename(title="Locate results-cam-kalibr.txt",
#                                   initialdir="~/Desktop")
#camChain = askopenfilename(title="Locate camchain-kalibr.yaml",
#                                   initialdir="~/Desktop")

# translations are given as [x,y,z] in cm, rotations as [yaw, pitch, roll]

# kalibrs output files
camChain = 'sampleFiles/camchain-kalibr.yaml'
calibResults = 'sampleFiles/results-cam-kalibr.txt'

f = open(camChain)
fCamChain = f.read()
f.close()
data = yaml.load(fCamChain)

# get rotation from transformation matrix
def getR(tf):
    return tf[:-1,:-1]
# get translation from transformation matrix
def getT(tf):
    return tf[:-1,3:]

# get quaternions from transformation
def getQfromTF(tf):
    return TF.quaternion_from_matrix(getR(tf))

def drawToGround(vec):
    x = vec[0][0]
    y = vec[1][0]
    z = vec[2][0]
    ax.plot3D([x,x],[y,y],[0,z], color='y')

def drawReference(pos, rot):
    x_axis = np.array([[.1],[0],[0]])
    y_axis = np.array([[0],[.1],[0]])
    z_axis = np.array([[0],[0],[.1]])
    x_axis = qv_mult(rot, x_axis)
    y_axis = qv_mult(rot, y_axis)
    z_axis = qv_mult(rot, z_axis)
    x_axis = np.add(x_axis, pos)
    y_axis = np.add(y_axis, pos)
    z_axis = np.add(z_axis, pos)
    myPlotLine(pos, x_axis, "r")
    myPlotLine(pos, y_axis, "g")
    myPlotLine(pos, z_axis, "b")

# camera transformations from yaml file
tf_BB2Right_BB2Left       = np.array(data['cam1']['T_cn_cnm1'])
tf_BB3Left_BB2Right       = np.array(data['cam2']['T_cn_cnm1'])
tf_BB3Center_BB3Left      = np.array(data['cam3']['T_cn_cnm1'])
tf_BB3Right_BB3Center     = np.array(data['cam4']['T_cn_cnm1'])
tf_PanCamLeft_BB3Right    = np.array(data['cam5']['T_cn_cnm1'])
tf_PanCamRight_PanCamLeft = np.array(data['cam6']['T_cn_cnm1'])

def arrayToColVec(a):
    return [[a[0]],[a[1]],[a[2]]]

def colVecToArray(v):
    return [v[0][0],v[1][0],v[2][0]]

# rotate vector v by quaternion q
#  0        0   
#  vr0 = z  v0  z*
#  vr1      v1  
#  vr2      v2 
def qv_mult(q,v):
    q = TF.unit_vector(q)
    q2 = list([0,v[0],v[1],v[2]])
    vr = TF.quaternion_multiply(
                TF.quaternion_multiply(q,q2),
                TF.quaternion_conjugate(q)
          )[1:]
    return vr

def getOppositeFacingQuaternion(q):
    euler = TF.euler_from_quaternion(q)
    return TF.quaternion_from_euler( -euler[0], -euler[1], -euler[2] )

def labelPos(pos, label):
    pos = colVecToArray(pos)
    ax.text3D(pos[0], pos[1], pos[2], label)

def myPlotLine(p1,p2,color="b"):
    p1 = np.asarray(colVecToArray(p1))
    p2 = np.asarray(colVecToArray(p2))
    ax.plot3D([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color)

drawReference(np.array([[0],[0],[0]]), TF.quaternion_from_euler(0,0,0))

# transformation from kalibr apriltag demo:
# left pancam to target
# x=0.364034, y=0.0139871, z=1.30524, yaw=0.256855, pitch=0.103205, roll=2.69673 # -165deg
#x=0.237599, y=0.00594341, z=1.26584, yaw=0.099315, pitch=0.0509062, roll=2.65734 # -175deg
# x=0.178021, y=0.00981665, z=1.25501, yaw=0.0235408, pitch=0.0114584, roll=2.64158 # -180deg
yaw = 0.099315 #175
pitch=0.0509062
roll=2.65734

yaw = 0.0235408 #180
pitch=0.0114584
roll=2.64158

#yaw = 0.256855 #165
#pitch=0.103205
#roll=2.69673


print(np.degrees([yaw,pitch,roll]))
q_PanCamLeft_Target = TF.quaternion_from_euler(yaw, pitch, roll, 'rzyx') #TODO
#q_PanCamLeft_Target = TF.quaternion_from_euler(pitch, roll, yaw, 'rxyz') #TODO
t_PanCamLeft_Target = np.array([[0.237599],#175
                                [0.00594341], 
                                [1.26855], ])
t_PanCamLeft_Target = np.array([[0.178021], #180
                                [0.00981665], 
                                [1.25501], ])
#t_PanCamLeft_Target = np.array([[0.364034], #165
#                                [0.0139871], 
#                                [1.30524], ])  145020999925178

                              
#t_PanCamLeft_Target = np.array([ [0],
#                                 [0],
#                                 [1] ])

q_PanCamLeft_Rotated = TF.quaternion_from_euler(np.radians(0),np.radians(60),np.radians(-180), 'rzxy')

pos_BB2Left     = arrayToColVec([0,0,0])
pos_BB2Right    = arrayToColVec([0,0,0])
pos_BB3Left     = arrayToColVec([0,0,0])
pos_BB3Center   = arrayToColVec([0,0,0])
pos_BB3Right    = arrayToColVec([0,0,0])
pos_PanCamLeft  = arrayToColVec([0,0,0])
pos_PanCamRight = arrayToColVec([0,0,0])
pos_Center      = arrayToColVec([0,0,0])
rot_Center      = TF.quaternion_from_euler(0,0,0)
pos_Target      = arrayToColVec([-0.403,-0.086,0.115])
rot_Target      = TF.quaternion_from_euler(np.radians(90),np.radians(-1.0),np.radians(-0.0), 'rzyx') # assumed z,y,x

t_Center_Target = np.subtract(pos_Center,pos_Target)
q_Center_Target = TF.quaternion_multiply(rot_Center, rot_Target)

def posToVec4(pos):
    return [pos[0],pos[1],pos[2],[1]]

import pyquaternion

tf_Center_Target = pyquaternion.Quaternion(rot_Target).unit.transformation_matrix

tf_Center_Target[0][3] = pos_Target[0][0]
tf_Center_Target[1][3] = pos_Target[1][0]
tf_Center_Target[2][3] = pos_Target[2][0]

tf_Center_Target = np.asmatrix(tf_Center_Target)

tf_PanCamLeft_Target = pyquaternion.Quaternion(q_PanCamLeft_Target).unit.transformation_matrix

tf_PanCamLeft_Target[0][3] = t_PanCamLeft_Target[0]
tf_PanCamLeft_Target[1][3] = t_PanCamLeft_Target[1]
tf_PanCamLeft_Target[2][3] = t_PanCamLeft_Target[2]

tf_PanCamLeft_Target = np.asmatrix(tf_PanCamLeft_Target)

tf_Target_PancamLeft=np.linalg.inv(tf_PanCamLeft_Target)
RI = np.linalg.inv(tf_PanCamLeft_Target[0:3,0:3])
t = -RI*t_PanCamLeft_Target

#tf_Center_PanCamLeft = tf_Target_PancamLeft*tf_Center_Target
tf_Center_PanCamLeft = tf_Center_Target*tf_Target_PancamLeft
#print(np.matrix(posToVec4(pos_Center)))
pos_PanCamLeft = tf_Center_PanCamLeft * np.matrix(posToVec4(pos_Center))
print(pos_Target)
print(pos_PanCamLeft)

# try to go from the calculated pancamleft position to the target using the values from apriltagdemo
#tf_ptu_rotation = pyquaternion.Quaternion(q_PanCamLeft_Rotated_Target).unit.transformation_matrix
#pos_Target_from_Inverted = tf_ptu_rotation * np.asarray(tf_PanCamLeft_Target) * np.asarray(pos_PanCamLeft)
#myPlotLine(pos_Target_from_Inverted, pos_PanCamLeft, 'r')
#a = np.asarray(getT(tf_PanCamLeft_Target))
#crosspr = np.cross([a[0][0],a[1][0],a[2][0]],[1,0,0])
#myPlotLine(pos_Center,pos_Center + arrayToColVec(crosspr))

#myPlotLine(pos_Center,pos_Target)crosspr = np.cross([a[0][0],a[1][0],a[2][0]],[1,0,0])
drawReference(pos_Target, TF.quaternion_from_matrix(tf_Center_Target))
myPlotLine(pos_Target,pos_PanCamLeft)
pos_PanCamLeft = np.asarray(pos_PanCamLeft)
pos_PanCamLeft = pos_PanCamLeft[0:3]
drawReference(pos_PanCamLeft, TF.quaternion_from_matrix(tf_Center_PanCamLeft))

# Go from the rotated left pancam to the PTU
t_PanCamLeft_PTU = np.array([[0.25],
                             [0.055], 
                             [0.01]])

## NOT EXACTLZ RIHT, CONTINUE HERE FOR NEUTRAL POSITION OF PTU, CHECK ALSO THE TRANSLATIONS UP HERE
#q_PanCamLeft_PTU = TF.quaternion_from_euler(np.radians(175),np.radians(60-20),np.radians(0), 'szyx')
tf_PanCamLeft_PTU = pyquaternion.Quaternion(q_PanCamLeft_Rotated).unit.transformation_matrix
#tf_PanCamLeft_Rotation = pyquaternion.Quaternion(q_PanCamLeft_Rotated).unit.transformation_matrix

tf_PanCamLeft_PTU[0][3] = t_PanCamLeft_PTU[0]
tf_PanCamLeft_PTU[1][3] = t_PanCamLeft_PTU[1]
tf_PanCamLeft_PTU[2][3] = t_PanCamLeft_PTU[2]

tf_PanCamLeft_PTU = np.asmatrix(tf_PanCamLeft_PTU)

tf_Center_PTU = tf_Center_PanCamLeft*tf_PanCamLeft_PTU
pos_PTU = tf_Center_PTU * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_PTU),pos_PanCamLeft)
myPlotLine(pos_Center,pos_Target)

drawReference(np.asarray(pos_PTU)[:-1],getQfromTF(tf_Center_PTU))

# Go from the PTU to the front facing left pancam
q_PTU_PanCamLeftKalibr = TF.quaternion_from_euler(np.radians(0),np.radians(0),np.radians(-20), 'rzyx')
tf_PTU_PanCamLeftKalibr = pyquaternion.Quaternion(q_PTU_PanCamLeftKalibr).unit.transformation_matrix
tf_PTU_PanCamLeftKalibr[0][3] = -t_PanCamLeft_PTU[0]
tf_PTU_PanCamLeftKalibr[1][3] = -t_PanCamLeft_PTU[1]
tf_PTU_PanCamLeftKalibr[2][3] = -t_PanCamLeft_PTU[2]

tf_Center_PanCamLeftKalibr = tf_Center_PTU*tf_PTU_PanCamLeftKalibr

pos_PanCamLeftKalibr = tf_Center_PanCamLeftKalibr * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_PanCamLeftKalibr),np.asarray(pos_PTU))
drawReference(np.asarray(pos_PanCamLeftKalibr)[:-1],getQfromTF(tf_Center_PanCamLeftKalibr))

# Go from the right pancam to the left one (kalibr tilt)
tf_PanCamLeftKalibr_PanCamRight = np.linalg.inv(tf_PanCamRight_PanCamLeft)
tf_Center_PanCamRight = tf_Center_PanCamLeftKalibr * tf_PanCamLeftKalibr_PanCamRight
pos_PanCamRight = tf_Center_PanCamRight * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_PanCamLeftKalibr),np.asarray(pos_PanCamRight))
drawReference(np.asarray(pos_PanCamRight)[:-1],getQfromTF(tf_Center_PanCamRight))

# Go from the left pancam to the right bb3
tf_Center_BB3Right = tf_Center_PanCamLeftKalibr * tf_PanCamLeft_BB3Right
pos_BB3Right = tf_Center_BB3Right * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_PanCamLeftKalibr),np.asarray(pos_BB3Right))
drawReference(np.asarray(pos_BB3Right)[:-1],getQfromTF(tf_Center_BB3Right))

# Go from the Right BB3 to the Center BB3
tf_Center_BB3Center = tf_Center_BB3Right * tf_BB3Right_BB3Center
pos_BB3Center = tf_Center_BB3Center * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_BB3Right),np.asarray(pos_BB3Center))
drawReference(np.asarray(pos_BB3Center)[:-1],getQfromTF(tf_Center_BB3Center))

# Go from the Center BB3 to the Left BB3
tf_Center_BB3Left = tf_Center_BB3Center * tf_BB3Center_BB3Left
pos_BB3Left = tf_Center_BB3Left * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_BB3Center),np.asarray(pos_BB3Left))
drawReference(np.asarray(pos_BB3Left)[:-1],getQfromTF(tf_Center_BB3Left))

# Go from the Left BB3 to the Right BB2
tf_Center_BB2Right = tf_Center_BB3Left * tf_BB3Left_BB2Right
pos_BB2Right = tf_Center_BB2Right * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_BB3Left),np.asarray(pos_BB2Right))
drawReference(np.asarray(pos_BB2Right)[:-1],getQfromTF(tf_Center_BB2Right))

# Go from the Left BB3 to the Right BB2
tf_Center_BB2Left = tf_Center_BB2Right * tf_BB2Right_BB2Left
pos_BB2Left = tf_Center_BB2Left * np.matrix(posToVec4(pos_Center))
myPlotLine(np.asarray(pos_BB2Right),np.asarray(pos_BB2Left))
drawReference(np.asarray(pos_BB2Left)[:-1],getQfromTF(tf_Center_BB2Left))

# Output positions and rotations
print("BB2 Left:\nposition\n" + repr(pos_BB2Left) + "\nrotation\n" + repr(TF.euler_from_matrix(tf_Center_BB2Left)))
print("BB3 Left:\nposition\n" + repr(pos_BB3Left) + "\nrotation\n" + repr(TF.euler_from_matrix(tf_Center_BB3Left)))
print("PTU:\nposition\n" + repr(pos_PTU) + "\nrotation\n" + repr(TF.euler_from_matrix(tf_Center_PTU)))

#myPlotLine(pos_Center,pos_Target)
#drawReference(pos_Target, TF.quaternion_from_matrix(tf_Center_Target))
#
#t_Target_PanCamLeft = -t_PanCamLeft_Target
#q_Target_PanCamLeft = getOppositeFacingQuaternion(q_PanCamLeft_Target)
#t_Center_PanCamLeft = np.subtract(t_Target_PanCamLeft, t_Center_Target)
#q_Center_PanCamLeft = TF.quaternion_multiply(q_Target_PanCamLeft, q_Center_Target)
#
#v_Target_PanCamLeft = qv_mult(rot_Target,t_Target_PanCamLeft)
#pos_PanCamLeft = np.add(pos_Target,v_Target_PanCamLeft)
#pos_PanCamLeft = np.asarray(pos_PanCamLeft)
#pos_PanCamLeft = pos_PanCamLeft[0:3]
#
#myPlotLine(pos_PanCamLeft, pos_Target)
#drawReference(pos_PanCamLeft, TF.quaternion_from_matrix(tf_Center_PanCamLeft))
#
#pos_PTU = np.asarray(pos_PTU)
#pos_PTU = pos_PTU[0:3]
#
#myPlotLine(pos_PTU, pos_PanCamLeft)
#drawReference(pos_PTU, TF.quaternion_from_matrix(tf_Center_PTU))
##q_Target_PanCamLeft = TF.quaternion_multiply(q_Target_PanCamLeft, rot_Target)
#
##pos_PanCamLeft = np.add(pos_Target, t_Target_PanCamLeft)
##rot_PanCamLeft = TF.quaternion_multiply(q_Target_PanCamLeft, rot_Target)
#
#
#
##pos_PanCamLeft = qv_mult(q_Target_PanCamLeft,t_Target_PanCamLeft)
##myPlotLine(pos_Target, pos_PanCamLeft)
##rot_PanCamLeft = TF.quaternion_multiply(q_Target_PanCamLeft, rot_Target)
##drawReference(pos_PanCamLeft, rot_PanCamLeft)
#
##drawReference(pos_PanCamLeft, q_Target_PanCamLeft)
##v2 = qv_mult(q_Target_PanCamLeft,t_PanCamLeft_Target)
##print(v2)
## rotate by target orientation
##v2 = qv_mult( TF.quaternion_from_euler(0,np.radians(-90),np.radians(180), "rzyx"), v2)
##v2 = [[v2[2]], [v2[1]], [v2[0]]]
##print(v2)
##pos_PanCamLeft = np.add(v2, pos_Target)
##print(pos_PanCamLeft)
##myPlotLine(pos_Center,pos_Target)
##myPlotLine(pos_Target,pos_PanCamLeft)
#
##q_90_up = getOppositeFacingQuaternion(TF.quaternion_from_euler(np.radians(0), np.radians(90), np.radians(0), 'rzyx'))
##v2 = qv_mult(q_90_up,t_Center_Target)
##pos_test2 = np.add(v2, pos_Target)
##myPlotLine(pos_Center,v2)
#
## find unrotated right pancam


# labels in figure
#labelPos(pos_PanCamLeft, "PanCam Left (Rotated)")
labelPos(pos_Target, "Target")
labelPos(pos_Center, "Center")
labelPos(pos_PTU, "PTU")
#drawToGround(pos_Target)
#drawToGround(pos_PanCamLeft)

# draw dots at interesting positions
#lst = [
#    colVecToArray(pos_BB2Left),
#    colVecToArray(pos_BB2Right),
#    colVecToArray(pos_BB3Left),
#    colVecToArray(pos_BB3Center),
#    colVecToArray(pos_BB3Right),
#    colVecToArray(pos_PanCamLeft),
#    colVecToArray(pos_PanCamRight),
#    colVecToArray(pos_Target),
#    colVecToArray(pos_Center)
#    ]
#zipit = zip(*lst)
#ax.scatter(zipit[0],zipit[1],zipit[2])

#plt.gca().invert_yaxis()
plt.xlabel('x')
plt.ylabel('y')
plt.show()
