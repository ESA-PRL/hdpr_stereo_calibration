# HDPR stereo camera calibration routines

## Overview

ROS launch files starting the stereo camera calibration routines for the cameras mounted on HDPR.

**Authors: Karl Kangur  
Contact: Martin Azkarate  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**

## Dependencies

* [camera_calibration](http://wiki.ros.org/camera_calibration)
* [pointgrey_camera_driver](http://wiki.ros.org/pointgrey_camera_driver)
* [image_proc](http://wiki.ros.org/image_proc)
* [camera1394](http://wiki.ros.org/camera1394)

## Basic Usage

Call the `.launch` file depending on which camera to calibrate, for example:

    roslaunch hdpr_stereo_calibration stereo_calibrate_bb2.launch

Run the `camera_calibration` routine, example:

    rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.02783 right:=/camera/right/image_color left:=/camera/left/image_color --no-service-check

