# ros_dvs_bridge

Work done at CNRS-AIST JRL. This package enables the direct visual servoings of https://github.com/jrl-umi3218/DirectVisualServoing to run within a ROS wrapping. 

Authors: Guillaume Caron, ...

Dates: from January 2023 to ...

# Pre-requisities

Tested under `Ubuntu 20.04` and `ROS Noetic`

## ros packages:
- `spinnaker`: to use Flir camera (Flir Spinnaker SDK, version 3.0.0.118 tested: `https://flir.app.boxcn.net/v/SpinnakerSDK?pn=Spinnaker+SDK&vn=Spinnaker_SDK`)

- `flir_camera_driver`: to use Flir camera with ROS (`https://github.com/ros-drivers/flir_camera_driver`)

- `visp_bridge`: to convert ROS messages to ViSP data formats, mainly grayscale images (`sudo apt-get install ros-noetic-visp-bridge`)

- `Universal_Robots_ROS_Driver`: to use UR10 robot (`https://github.com/UniversalRobots/Universal_Robots_ROS_Driver`, checkout commit `ca2b11cdaf0233d59d1fe3e4c25a4a844331ec07` to work with the 3.7 old version of Polyscope still within the UR10 controller)

## How to run

Prior to using the UR10 robot, one must gets its calibration file:

`roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.3 target_filename:=$HOME/AIST_UR10_robot_calibration.yaml"`

### Example with launching several nodes from the same launch file with UR10 robot equipped with a Flir camera with Computar lens:

After creating a directory `ros_dvs_bridge` in your directory `$HOME/.ros`

Terminal 1: run `roscore`

Terminal 2: run the robot driver with `roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.3 \ kinematics_config:=$HOME/AIST_UR10_robot_calibration.yaml`, then run the External Control URCap by running the 'externalControl7' program from the teach pendent of the robot.

Terminal 3: 
- to run the camera node, the image resize x0.5 and the desired image capture (this doesn't need the robot driver to run): `roslaunch ros_dvs_bridge pvsCaptureAndSaveDesired_FL3-U3_resize0p5.launch`
- to run the camera node, the image resize x0.5 (for higher control rate) and visual servoing: `roslaunch ros_dvs_bridge pvsPhotometricVisualServoing_UR10_FL3-U3_resize0p5.launch`


### Example with launching each node separately with UR10 robot equipped with a Flir camera with Yakumo lens:

After creating a directory `ros_dvs_bridge` in your directory `$HOME/.ros`

Terminal 1: run `roscore`

Terminal 2: run the camera node with `roslaunch spinnaker_camera_driver camera.launch`

Terminal 3: run the robot driver with `roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.3 \ kinematics_config:=$HOME/AIST_UR10_robot_calibration.yaml`, then run the External Control URCap on the robot controller

Terminal 4:
- to run the desired image capture (this doesn't need the robot driver to run): `roslaunch ros_dvs_bridge pvsCaptureAndSaveDesired.launch`
- to run the visual servoing: `roslaunch ros_dvs_bridge pvsPhotometricVisualServoing_UR10.launch`


### Note: to stop the robot from another terminal

`rostopic pub -1 /twist_controller/command geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'`


