![Kalibr](https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/images/kalibr_small.png)

*Ubuntu 14.04+ROS indigo*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=kalibr_weekly/label=ubuntu-trusty)](https://jenkins.asl.ethz.ch/job/kalibr_weekly/label=ubuntu-trusty/) *Ubuntu 16.04+ROS kinetic*: [![Build Status](https://jenkins.asl.ethz.ch/buildStatus/icon?job=kalibr_weekly/label=ubuntu-trusty)](https://jenkins.asl.ethz.ch/job/kalibr_weekly/label=ubuntu-xenial/)

# Introduction
This is the PointOne FORK of the Kalibr toolbox. The offline imu to camera calibration tool was modified to use a 3D point cloud from the SFM solution as the calibration target instead of a 2D cailbration grid.

# Using the Modified IMU-Camera Calibratoin tool:
## Setup
Setup your environment per the Kalibr wiki referenced below. In the ~/kalibr_workspace/ directory created in the instructions you copy to src/Kalibr/ this Kalibr source Git repository. 

Setup your environment per Kalibr instructions via $source ~/kalibr_workspace/devel/setup.bash
This should include the following in your PYTHONPATH =/home/name/kalibr_workspace/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages

## Create a ROS BAG
Currently you still need to create a ROS bag from the camera images and the IMU file. The PointOne bagcreator is given a directory where it expects to find the camera and IMU data. You need to put the image files from each camera into separate subfolders called cam0/, cam1/, etc. Put the IMU CSV file in the top of directory. Then execute:
~/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_bagcreater_PointOneLog 
--folder /logs/calibration/kalibr/testfolder 
--output-bag /logs/calibration/kalibr/testfolder/rosbag.bag

The "topics" in the rosbag will be cam#/ and imu_filename/

## Other Files
You will also need:
 1. the SFM protobuf as a JSON file. This should contain the 3D structure, the camera calibration information, and the list of images with detections. The timestamps are in the names of the image files and must correspond to those put into the rosbag.
2. A camchain.yaml that includes the camera intrinsics for each camera topic in the rosbag. 
3. A imu.yaml that contains the noise characteristics of the IMU topic in the rosbag

## Running the PointOne IMU-CAM Calibrator:
In this example the files are all in the /logs/2018-07-25/ folder:

~/kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python:master$ ./kalibr_calibrate_imu_camera_PointOne  
 --cam /logs/2018-07-25/camchain-equi.yaml
 --imu /logs/2018-07-25/imu.yaml 
 --map /logs/2018-07-25/sfmMap.json
 --bag /logs/2018-07-25/rosbag.bag 

It will produce graphs and a PDF report.


# Original Kalibr Introduction

Kalibr is a toolbox that solves the following calibration problems:

1. **Multiple camera calibration**: 
    intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view
1. **Camera-IMU calibration**:
    spatial and temporal calibration of an IMU w.r.t a camera-system
1. **Rolling Shutter Camera calibration**:
    full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras


**Please find more information on the [wiki pages](https://github.com/ethz-asl/kalibr/wiki) of this repository.**

**For questions or comments, please open an issue on Github.**

## Tutorial: IMU-camera calibration
A video tutorial for the IMU-camera calibration can be found here:

[![alt text](https://user-images.githubusercontent.com/5337083/44033014-50208b8a-9f09-11e8-8e9a-d7d6d3c69d97.png)](https://m.youtube.com/watch?v=puNXsnrYWTY "imu cam calib")

(Credits: @indigomega)

## Authors
* Paul Furgale ([email](paul.furgale@mavt.ethz.ch))
* Jérôme Maye ([email](jerome.maye@mavt.ethz.ch))
* Jörn Rehder ([email](joern.rehder@mavt.ethz.ch))
* Thomas Schneider ([email](schneith@ethz.ch))
* Luc Oth

## References
The calibration approaches used in Kalibr are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)

## Acknowledgments
This work is supported in part by the European Union's Seventh Framework Programme (FP7/2007-2013) under grants #269916 (V-Charge), and #610603 (EUROPA2).

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
