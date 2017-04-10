#!/usr/bin/env python

"""This script takes the calibration results from ROS as input (via a user
provided file) and prints the converted and correctly formatted stereo camera
parameters to the terminal, ready for use in ROCK.

To use this script, save the stereo calibration output (terminal) to a file and
run this script. You will then be asked for that file and will be presented with
the results which can be copied directly to the stereo yml
(default: ~/dev/bundles/hdpr/config/orogen/stereo::Task.yml).

The transformation from the camera rotation matrix to Euler angles is done using
Christoph Gohlkes transformations.py."""

__author__ = "Levin Gerdes"

#import yaml
import re
import numpy as np
#import shutil # to back up stereo yml
import os.path
from decimal import *
from Tkinter import Tk
from tkFileDialog import askopenfilename
from transformations import euler_from_matrix

# precision for Decimal
getcontext().prec = 16

cams = ['exoter_bb2','hdpr_bb2','hdpr_bb3_left_right','panCam']
stereoFilePath = '/home/hdpr/dev/bundles/hdpr/config/orogen/stereo::Task.yml'

print("\nCALIBRATION CONVERSION.\nThis script does not write anything to disk.\n")

# Ask which camera parameters should be written.
# As long as we are not writing to the yml, this is only necessary in order to
# read and print the frame height, width, and some panCam specific settings.
question = "Which camera is being re-calibrated?\n"
i = 1
for cam in cams:
    question += str(i) + ") " + cam + "\n"
    i += 1
question += "0) Abort\n"

userIn = int(raw_input(question))
if (userIn == 0):
    quit()

# RegEx for most numbers being used in the yml and calibration output
reNumber = '-?\d+\.\d+'

# Ask for calibration results
print("Please choose the file with the ROS calibration results\n")
Tk().withdraw()
calibResultsFile = askopenfilename(title="Locate calibration results",
                                   initialdir="~/Desktop")

# Read calibration file
f = open(calibResultsFile, 'r')
calibFile = f.read()
f.close()

# TRANSLATION VALUES
# (self.T)
# look for "self.T ', [-0.8342,1.12325,7.234234,..."
reTrans = '(?<=self.T\ \',\ \[)(' + reNumber + '(,\ )?){3}'
selfT = re.search(reTrans, calibFile)
selfT = selfT.group(0)
#print("Translation Values: "+selfT)
# ... are given in meters, but ROCK needs them in millimeters
selfT = selfT.split(',')
tx = float(selfT[0])*1000
ty = float(selfT[1])*1000
tz = float(selfT[2])*1000
#print(tx,ty,tz)

# ROTATION VALUES
# look for "self.R ', [-0.8342,1.12325,7.234234,..."
reRot = '(?<=self.R\ \',\ \[)(' + reNumber + '(,\ )?){9}'
selfR = re.search(reRot, calibFile)
selfR = selfR.group(0)
# ... are given in matrix form, but ROCK needs them in Euler angles.
# turn into matrix representation
rotMatrix = eval('[' + selfR + ']')
rotMatrix = np.array(rotMatrix).reshape(3,3)
#print(repr(rotMatrix))
# Call provided Python function to transform them.
rotEuler = euler_from_matrix(rotMatrix, 'sxyz')
#print(repr(rotEuler))

# CAMERA MATRICES
# left
reCamMat = '(?<=left\]\n\ncamera\ matrix\n)(' + reNumber + '\ ' + reNumber + '\ ' + reNumber + '\n){2}'
camMat = re.search(reCamMat, calibFile).group(0).split()

fxLeft = camMat[0]
fyLeft = camMat[4]
cxLeft = camMat[2]
cyLeft = camMat[5]

# right
reCamMat = '(?<=right]\n\ncamera\ matrix\n)(' + reNumber + '\ ' + reNumber + '\ ' + reNumber + '\n){2}'
camMat = re.search(reCamMat, calibFile).group(0).split()

fxRight = camMat[0]
fyRight = camMat[4]
cxRight = camMat[2]
cyRight = camMat[5]

#print(fx,fy,cx,cy)

# DISTORTION VALUES
reDist = '('+ reNumber + '\ ){4}'
# left
reDistAll = '(?<=left\]\n)(.*\n){7}' + reDist + reNumber + '(?=\n\nrectification)'
distLeft  = re.search(reDistAll, calibFile).group(0)
distLeft  = re.search(reDist, distLeft).group(0).split()
# right
reDistAll = '(?<=right\]\n)(.*\n){7}' + reDist + reNumber + '(?=\n\nrectification)'
distRight = re.search(reDistAll, calibFile).group(0)
distRight = re.search(reDist, distRight).group(0).split()

# Ask user to provide path to stereo yml if default is non-existant
if not (os.path.isfile(stereoFilePath) and os.access(stereoFilePath, os.R_OK)):
    print("Could not read default stereo yml file: \"" + stereoFilePath + "\"\nUse dialog to locate the correct yml.\n")
    Tk().withdraw()
    stereoFilePath = askopenfilename(title="Locate stereo yml",
                                     initialdir="~/dev/bundles")

f = open(stereoFilePath, 'r')
stereoYML = f.read()
f.close()

# only grab till the next camera (or until the end of the file)
reCamSettings = '(?<=---\ name\:' + cams[userIn-1] + '\n)(.*\n)*?' + '(?=(---\ name)|\Z)'
# all settings of the selected camera (+ name of next cam in ascii art) 
camSettings = re.search(reCamSettings, stereoYML).group(0)

width = re.search('(?<=\ width\:\ )\d+',camSettings).group(0)
height = re.search('(?<=\ height\:\ )\d+',camSettings).group(0)

# Create backup of existing stereo yml
#shutil.copy2(stereoFilePath, stereoFilePath + '.BAK')

# PRINT RESULTS
res =  "\nstereoCameraCalibration:"
res += "\n  camLeft:"
res += "\n    fx: " + fxLeft
res += "\n    fy: " + fyLeft
res += "\n    cx: " + cxLeft
res += "\n    cy: " + cyLeft
res += "\n    d0: " + distLeft[0]
res += "\n    d1: " + distLeft[1]
res += "\n    d2: " + distLeft[2]
res += "\n    d3: " + distLeft[3]
res += "\n    width: " + width
res += "\n    height: " + height
if (cams[userIn-1] == "panCam"):
    res += "\n    ex: 0"
    res += "\n    ey: 0"
res += "\n  camRight:"
res += "\n    fx: " + fxRight
res += "\n    fy: " + fyRight
res += "\n    cx: " + cxRight
res += "\n    cy: " + cyRight
res += "\n    d0: " + distRight[0]
res += "\n    d1: " + distRight[1]
res += "\n    d2: " + distRight[2]
res += "\n    d3: " + distRight[3]
res += "\n    width: " + width
res += "\n    height: " + height
if (cams[userIn-1] == "panCam"):
    res += "\n    ex: 0.0"
    res += "\n    ey: 0.0"
res += "\n#Distance between left and right camera"
res += "\n  extrinsic:"
res += "\n    tx: " + str(1*Decimal(tx))
res += "\n    ty: " + str(1*Decimal(ty))
res += "\n    tz: " + str(1*Decimal(tz))
res += "\n    rx: " + str(1*Decimal(rotEuler[0]))
res += "\n    ry: " + str(1*Decimal(rotEuler[1]))
res += "\n    rz: " + str(1*Decimal(rotEuler[2]))

print(res)

#TODO? Write directly to yml
