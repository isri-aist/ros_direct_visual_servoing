#include "pvsDesiredImageManager.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pvsCaptureAndSaveDesired");
    pvsDesiredImageManager prog;
    ros::spin();
    return 0;
}

