#ifndef __mppvsDesiredImageManager_H__
#define __mppvsDesiredImageManager_H__

#include <image_transport/image_transport.h>

#include "tf2_msgs/TFMessage.h" //evs

#include "geometry_msgs/Twist.h"

#include "visp_bridge/image.h"

using namespace std;

class mppvsDesiredImageManager
{

public:

    mppvsDesiredImageManager();
		~mppvsDesiredImageManager();

    void imageCallback(const sensor_msgs::ImageConstPtr& image);    
    void toolPoseCallback(const tf2_msgs::TFMessage& tf); //evs
		vpHomogeneousMatrix toVispHomogeneousMatrix(const tf2_msgs::TFMessage& trans);  //evs

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;

    ros::Subscriber m_camPose_sub; //evs
		vpHomogeneousMatrix m_bMt;
		std::mutex mutex_bMt;
    vpHomogeneousMatrix m_tMc;



    vpImage<unsigned char> m_desired_image;

    int m_iter;

    std::ofstream m_logfile;
    string m_logs_path;

    bool m_rosbagForEVS; //evs
    string m_desiredPoseTopicForEVS;

};

#endif //__mppvsDesiredImageManager_H__
