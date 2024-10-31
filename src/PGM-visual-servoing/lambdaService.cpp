#include "ros/console.h"
#include <ros_dvs_bridge/diff_lambda.h>
#include <ros/ros.h>
#include <cstdlib>
#include <cstdio> 
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

#include <DifferentiableImage/DifferentiableImage.h>

DifferentiableImage diff_image;

bool callFunction(ros_dvs_bridge::diff_lambda::Request &req, ros_dvs_bridge::diff_lambda::Response &res) {
    res.lambda_g = diff_image.start(req.desired_image_path, req.init_image_path);//, "/home/ur10t/.ros/ros_dvs_bridge/mask_92x92.png");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lambdaService");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("diff_lambda", callFunction);

  ros::spin();

  return 0;
}
