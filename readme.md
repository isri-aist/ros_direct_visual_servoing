
Pre-requisities

Tested under Ubuntu 20.04 and ROS Noetic

ros packages:
- spinnaker: to use Flir camera (Flir Spinnaker SDK, version 3.0.0.118 tested: https://flir.app.boxcn.net/v/SpinnakerSDK?pn=Spinnaker+SDK&vn=Spinnaker_SDK)

- flir_camera_driver: to use Flir camera with ROS (https://github.com/ros-drivers/flir_camera_driver)

- visp_bridge: sudo apt-get install ros-noetic-visp-bridge

How to run:

After creating a directory `ros_dvs_bridge` in your directory `$HOME/.ros`

Terminal 1: run `roscore`

Terminal 2: run the camera node with `roslaunch spinnaker_camera_driver camera.launch`

Terminal 3: 
- to run the desired image capture: `roslaunch ros_dvs_bridge pvsCaptureAndSaveDesired.launch `
 
