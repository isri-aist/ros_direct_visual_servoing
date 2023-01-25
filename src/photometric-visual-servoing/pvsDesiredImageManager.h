#ifndef __pvsDesiredImageManager_H__
#define __pvsDesiredImageManager_H__

#include <image_transport/image_transport.h>

#include "geometry_msgs/Twist.h"

#include "visp_bridge/image.h"

using namespace std;

class pvsDesiredImageManager
{

public:

    pvsDesiredImageManager();
		~pvsDesiredImageManager();

    void imageCallback(const sensor_msgs::ImageConstPtr& image);    

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;

    vpImage<unsigned char> m_desired_image;

    int m_iter;

    std::ofstream m_logfile;
    string m_logs_path;
    string m_data_path;

		/*
    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;
    VisualServoTools m_visual_servo_tools;
		*/

};

#endif //__pvsDesiredImageManager_H__
