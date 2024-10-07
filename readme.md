# ros_dvs_bridge

Work done at CNRS-AIST JRL. This package enables the direct visual servoings of https://github.com/jrl-umi3218/DirectVisualServoing to run within a ROS wrapping. 

Authors: Guillaume Caron, Antoine Andre, Belinda Naamani, ...

Dates: from January 2023 to ...

# Pre-requisities

Tested under `Ubuntu 20.04` and `ROS Noetic`

## camera drivers
- `spinnaker`: to use Flir camera (Spinnaker SDK Download, version 3.1.0.79 tested: `https://www.flir.eu/products/spinnaker-sdk/?vertical=machine+vision&segment=iis`, create an account and click on Download (Login required))
- `insta360`: to use Insta360 ONE X2 (http://mis.u-picardie.fr/~g-caron/softs/20220909_insta360_SDK_linux.tar.gz and add udev rule `SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", ATTR{idProduct}=="0002", MODE="0666"`)
If unclear checkout [this] (https://thomasduvinage.github.io/wiki/documentation/Camera/Insta360/)

## ros packages:

- `flir_camera_driver`: to use Flir camera with ROS (https://github.com/ros-drivers/flir_camera_driver)

- `ros_insta360`: to use Insta360 camera with ROS

- `visp_bridge`: to convert ROS messages to ViSP data formats, mainly grayscale images:
```bash 
    sudo apt-get install ros-noetic-visp-bridge
```
 
- `Universal_Robots_ROS_Driver`: to use UR10 robot (https://github.com/UniversalRobots/Universal_Robots_ROS_Driver, checkout commit `ca2b11cdaf0233d59d1fe3e4c25a4a844331ec07` to work with the 3.7 old version of Polyscope still within the UR10 controller)

- `ros_controllers_cartesian`: to control a UR robot in Cartesian velocity; if not yet installed: 
 ```bash 
sudo apt install ros-noetic-ros-controllers-cartesian
```
- `evs`: ROS node to generate reference trajectory and actual trajectory (https://github.com/NathanCrombez/evs)

## external libraries
- `libPeR`: to use the PGM VS (https://github.com/PerceptionRobotique/libPeR). When running `catkin_make`, mind to pass the additional `cmake` parameters `-DUSE_PER=True` to make use of libPeR (otherwise PGM VS will not be available) and `-DPER_DIR=/path/to/libPeR/install/dir` to allow finding the libPeR library (in that order)

- `evo`: to evaluate and compare ideal and actual trajectory from the evs generated ROS bag (https://github.com/MichaelGrupp/evo)

# How to run on UR10 and UR5e
## Before running the VS

Prior to using the UR10 robot, one must get its calibration file:

```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.3 target_filename:=$HOME/AIST_UR10_robot_calibration.yaml 
```

Prior to using a UR5e robot, one must get its calibration file:

```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.1.4 target_filename:=$HOME/AIST_UR5_robot_calibration.yaml
```
Create a directory `ros_dvs_bridge` in your directory `$HOME/.ros`:

```
cd $HOME/.ros && mkdir ros_dvs_bridge
```
### PGMVS
The PGMVS allows a switch between perspective, hemispherical and full-spherical (equirectangular) camera. 
#### Extrinsic camera calibration
Depending on how the camera is mounted to the end effector of the UR robot, the following parameter must be adjusted: 
`<param name="camera2tool" type="string" value="$(find ros_dvs_bridge)/config/FL3-U3toTool.yml"/>`
The .yml `FL3-U3toTool` must be changed accordingly. The `config` file provides the extrinsic camera calibration for the FL3-U3-13E4C and Insta360 ONE X2 camera with a mounting directly to the tool and a mounting to  wrist 3 (checkout `img` to see the exact mounting). To check if the calibration is correct send slow velocity commands and check that the motion is indeed on the cameras axis. 

#### Intrinsic camera calibration, resizing and FoV
Depending on the resizing (to save processing time) the intrinsic calibration must be adjusted:
`<param name="cameraXml" type="string" value="$(find ros_dvs_bridge)/config/insta360_one_x2/calib_equi_184_92.xml"/>`
The path to and the .xml itself `insta360_one_x2/calib_equi_184_92.xml` must be changed accordingly. 
Examples:
 - The [FL3-U3-13E4C] (https://www.teledynevisionsolutions.com/products/flea3-usb3/?model=FL3-U3-13E4C-C&vertical=machine%20vision&segment=iis) has a resolution of 1280 × 1024 pixels. With a camera binning of 2 and a resize of 0.125, this leads to `calib_persp_80_64.xml`
- The Insta360 ONE X2 has a resolution of 2304 x 1152 for the equirectangular image leading to `calib_equi_184_92.xml` for a resizing of 0.08
Note that the resizing can be done in the camera.launch and in the `captureAndSaveDesired/VisualServoing.launch ` with the [image_proc] (http://wiki.ros.org/image_proc) package. 

To consider the different field of views of the cameras, the cameraType value in the `VisualServoing.launch` must be adjusted accordingly (hemispherical: 0, Persp: 1, Equirectangular: 4):
` <param name="cameraType" type="int" value="4"/>` 

#### Dynamic reconfigure
The [dynamic_reconfigure] (http://wiki.ros.org/dynamic_reconfigure) provides a means to update parameters (lambda_g, gain, scene depth) at runtime without having to restart the node. 
Get 
```bash
sudo apt-get install ros-noetic-rqt-common-plugins
```
and run 
```
rosrun rqt_reconfigure rqt_reconfigure
```
Change the default, min, max parameters in the `lambda_g.cfg` and run `catkin_make`. 

### PVS
Change the intrinsic calibration, gain factor (lambda) and scene depth directly in the launch file. 


## Example VS Task: 

Terminal 1: run `roscore`

Terminal 2: run the robot driver with 
```
roslaunch ros_dvs_bridge ur10_bringup.launch robot_ip:=192.168.1.3 kinematics_config:=$HOME/AIST_UR10_robot_calibration.yaml
```
or  
```
roslaunch ros_dvs_bridge ur5e_bringup.launch robot_ip:=192.168.1.4 kinematics_config:=$HOME/AIST_UR5_robot_calibration.yaml
```
and run the External Control URCap on the robot controller (Note: these launch files come from the [ur_robot_driver] (https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver/launch), but modified to start only the `twist_controller`, thus shipped with `ros_dvs_bridge`)


#### For Photometric VS (Flir camera with Computar lens)
Terminal 4:  
- to run the camera node, the image resize x0.5 and the desired image capture (this doesn't need the robot driver to run): 
```
roslaunch ros_dvs_bridge pvsCaptureAndSaveDesired_FL3-U3_resize0p5.launch
```
- to run the camera node, the image resize x0.5 (for higher control rate) and visual servoing: 
```
roslaunch ros_dvs_bridge pvsPhotometricVisualServoing_UR10_FL3-U3_resize0p5.launch
```

#### For PGM VS
Terminal 3: run the camera node. 
Examples:
- For the FL3-U3-13E4C (Mono8, binning: 2, resize: 0.125):
```
roslaunch ros_dvs_bridge spinnaker_resize.launch
```
- For Insta360
```
roslaunch insta360 bringup.launch 
```
Terminal 4 (default: Insta360 to tool, 0.08 resize): 
- Capture the desired image (this doesn't need the robot driver to run): 
```
roslaunch ros_dvs_bridge pgmvsCaptureAndSaveDesired.launch
```
- Run the visual servoing: 
```
roslaunch ros_dvs_bridge pgmvsPGMVisualServoing.launch
```

##### Evaluation PGMVS
- Use `rqt_image_view` to check the difference in desired and current image and image features 
- Use `rqt_plot` to check the residuals (`/costTopic`) and velocity (`/twist_controller_command`)
- Use `evs-evo` to evaluate the desired and actual trajectory path after the vs task (Note: convert the rosbag into tum file format to plot multiple vs tasks in one plot)

<!-- 
### Example with launching each node separately with UR10 robot equipped with a Flir camera with Yakumo lens:

After creating a directory `ros_dvs_bridge` in your directory `$HOME/.ros`

Terminal 1: run `roscore`

Terminal 2: run the camera node with `roslaunch spinnaker_camera_driver camera.launch`

Terminal 3: run the robot driver with `roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.3 \ kinematics_config:=$HOME/AIST_UR10_robot_calibration.yaml`, then run the External Control URCap on the robot controller

Terminal 4:
- to run the desired image capture (this doesn't need the robot driver to run): `roslaunch ros_dvs_bridge pvsCaptureAndSaveDesired.launch`
- to run the visual servoing: `roslaunch ros_dvs_bridge pvsPhotometricVisualServoing_UR10.launch` -->


### Note: to stop the robot from another terminal

```
rostopic pub -1 /twist_controller/command geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```