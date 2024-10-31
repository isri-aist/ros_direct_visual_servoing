# ros_dvs_bridge

Work done at CNRS-AIST JRL. This package enables the direct visual servoings of https://github.com/jrl-umi3218/DirectVisualServoing to run within a ROS wrapping. 

Authors: Guillaume Caron, Antoine Andre, Belinda Naamani, ...

Dates: from January 2023 to ...

# Pre-requisities

Tested under `Ubuntu 20.04` and `ROS Noetic`

## camera drivers

- `spinnaker`: to use Flir camera (Spinnaker SDK Download, version 3.1.0.79 tested: https://www.flir.eu/products/spinnaker-sdk/?vertical=machine+vision&segment=iis, create an account and click on Download (Login required))
- `insta360`: to use Insta360 ONE X2 (http://mis.u-picardie.fr/~g-caron/softs/20220909_insta360_SDK_linux.tar.gz and add udev rule `SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", ATTR{idProduct}=="0002", MODE="0666"`)

If unclear checkout [this](https://thomasduvinage.github.io/wiki/documentation/Camera/Insta360/)

## ros packages:

- `flir_camera_driver`: to use Flir camera with ROS (https://github.com/ros-drivers/flir_camera_driver)

- `ros_insta360`: to use Insta360 camera with ROS

- `visp_bridge`: to convert ROS messages to ViSP data formats, mainly grayscale images:
```bash 
    sudo apt-get install ros-noetic-visp-bridge
```
- `Universal_Robots_ROS_Driver`: to use UR10 robot (https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- `ros_controllers_cartesian`: to control a UR robot in Cartesian velocity; if not yet installed: 
 ```bash 
sudo apt install ros-noetic-ros-controllers-cartesian
```
- `evs`: ROS node to generate reference trajectory and actual trajectory (https://github.com/NathanCrombez/evs)

## external libraries

- `libPeR`: to use the PGM VS (https://github.com/PerceptionRobotique/libPeR). When running `catkin_make`, mind to pass the additional `cmake` parameters `-DUSE_PER=True` to make use of libPeR (otherwise PGM VS will not be available) and `-DPER_DIR=/path/to/libPeR/install/dir` to allow finding the libPeR library (in that order)

- `evo`: to evaluate and compare ideal and actual trajectory from the evs generated ROS bag (https://github.com/MichaelGrupp/evo)
- `differentiableImage`: to compute the initial lambda_g based on the differentiability of desired and starting image (https://github.com/GuicarMIS/differentiableImage/tree/toDual). If you don't want to use this, remove it in the cmake. 

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

- The [FL3-U3-13E4C](https://www.teledynevisionsolutions.com/products/flea3-usb3/?model=FL3-U3-13E4C-C&vertical=machine%20vision&segment=iis) has a resolution of 1280 × 1024 pixels. With a camera binning of 2 and a resize of 0.125, this leads to `calib_persp_80_64.xml`. Note that the resizing can be with the [image_proc](http://wiki.ros.org/image_proc) package
- The Insta360 ONE X2 has a resolution of 2304 x 1152 for the equirectangular image leading to `calib_equi_184_92.xml` for a resizing of 0.08. For using the `ros_insta360` package, apply the resizing in the package launch, to save computation time for calculating the equirectangular image.

To consider the different field of views of the cameras, the cameraType value in the `VisualServoing.launch` must be adjusted accordingly (hemispherical: 0, Persp: 1, Equirectangular: 4):

` <param name="cameraType" type="int" value="4"/>` 

#### Dynamic reconfigure

The [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) provides a means to update parameters (lambda_g, gain, scene depth) at runtime without having to restart the node. 

Get 
```bash
sudo apt-get install ros-noetic-rqt-common-plugins
```

and run 
```
rosrun rqt_reconfigure rqt_reconfigure
```
Change the default, min, max parameters in the `lambda_g.cfg` and run `catkin_make`. 

#### Differentiable Image

While captuing the desired image and at the beginning of the callback function, a picture will be stored in the `$HOME/.ros` directory. This images can be used to calculate the initial lambda_g via a rosservice, based on the difference between those two. Run the service with:
```
rosrun ros_dvs_bridge lambdaService
```
Include a mask for the omnidirectional VS case.

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
and run the External Control URCap on the robot controller (Note: these launch files come from the [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver/launch), but modified to start only the `twist_controller`, thus shipped with `ros_dvs_bridge`)


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

**FL3-U3-13E4C (Mono8, binning: 2, resize: 0.125)**

Terminal 3: 
- to run the camera node and the resizing 0.125:
```
roslaunch ros_dvs_bridge spinnaker_resize.launch
```

Terminal 4: 
- to capture the desired image:
```
roslaunch ros_dvs_bridge pgmvsCaptureAndSaveDesired_FL3-U3_resize0p125.launch
```
- to start the visual servoing:
```
roslaunch ros_dvs_bridge pgmvsPGMVisualServoing_FL3-U3_resize0p125.launch
```


**Insta360 (hemispherical and equirectangular, resize 0.08)** 

Terminal 3: 
- Resize by 0.08 and run:
```
roslaunch insta360 bringup.launch 
```
Terminal 4: 
- to capture the desired image:
```
roslaunch ros_dvs_bridge pgmvsCaptureAndSaveDesired_Insta360_fisheye_resize0p08.launch
``` 
or
```
roslaunch ros_dvs_bridge pgmvsCaptureAndSaveDesired_Insta360_equi_resize0p08.launch
```

- to start the visual servoing:
```
roslaunch ros_dvs_bridge pgmvsPGMVisualServoing_Insta360_fisheye_resize0p08.launch
```
or
```
roslaunch ros_dvs_bridge pgmvsPGMVisualServoing_Insta360_equi_resize0p08.launch
```

##### Evaluation PGMVS

- Use `rqt_image_view` to check the difference in desired and current image and image features
- Use `rqt_plot` to check the residuals (`/costTopic`) and velocity (`/twist_controller/command/angular` and `/twist_controller/command/linear`)
- To save the experiment data in a folder automatically after a vs task set `<param name="saveExprimentData" type="bool" value="true"/>` in the launch
- To change the lambda_g and gain automatically at convergence either use the dynamic_reconfigure manually or set `<param name="twoStepVS" type="bool" value="true"/>` and adjust the treshold for the velocity/residuals
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