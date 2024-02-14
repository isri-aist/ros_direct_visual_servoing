#include "pgmvsDesiredImageManager.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pgmvsCaptureAndSaveDesired");
    pgmvsDesiredImageManager prog;
    ros::spin();
    return 0;
}

