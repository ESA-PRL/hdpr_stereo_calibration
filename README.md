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

Call the `.launch` file depending on which camera to calibrate and run the `camera_calibration` routine, for example:

    roslaunch hdpr_stereo_calibration stereo_calibrate_bb2_HDPR.launch
    rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.02783 right:=/camera/right/image_color left:=/camera/left/image_color --no-service-check

or

    roslaunch hdpr_stereo_calibration stereo_calibrate_bb3.launch
    rosrun camera_calibration cameracalibrator.py --size 5x4 --square 0.1445 right:=/stereo/right/image_raw left:=/stereo/left/image_raw --no-service-check

The ROCK framework requires the extrinsic rotation values in Euler angles, but the calibration process outputs them in the form of a rotation matrix. To transform the values a script called `rot_ros2rock.py` is in the `src` folder.
