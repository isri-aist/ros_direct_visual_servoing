#include "pvsPhotometricVisualServoing.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pvsPhotometricVisualServoing");
    pvsPhotometricVisualServoing prog;
    ros::spin();
    return 0;
}

