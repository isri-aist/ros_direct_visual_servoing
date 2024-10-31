#include "mppvsDesiredImageManager.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mppvsCaptureAndSaveDesired");
    mppvsDesiredImageManager prog;
    ros::spin();
    return 0;
}

