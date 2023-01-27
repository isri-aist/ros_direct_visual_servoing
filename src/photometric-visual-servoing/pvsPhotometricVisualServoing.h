#ifndef __pvsPhotometricVisualServoing_H__
#define __pvsPhotometricVisualServoing_H__

#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>

#include "tf2_msgs/TFMessage.h"

#include "visp_bridge/image.h"

#include <visp3/core/vpCameraParameters.h>
#include <visp3/visual_features/vpFeatureLuminance.h>
#include <visp3/vs/vpServo.h>

#define INDICATORS

using namespace std;

class pvsPhotometricVisualServoing
{

public:

    pvsPhotometricVisualServoing();
		~pvsPhotometricVisualServoing();

		void initVisualServoTask();

		void imageCallback(const sensor_msgs::ImageConstPtr& image); 

    void toolPoseCallback(const tf2_msgs::TFMessage& tf); 
		vpHomogeneousMatrix toVispHomogeneousMatrix(const tf2_msgs::TFMessage& trans);   

		void stopRobot();

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
		ros::Publisher m_velocity_pub;

		ros::Subscriber m_camPose_sub; 
		vpVelocityTwistMatrix m_bVc;

		geometry_msgs::Twist m_velocity;

    vpImage<unsigned char> m_desired_image;
		vpImage<unsigned char> m_current_image;
    vpImage<unsigned char> m_difference_image;

		double m_mu;
    double m_lambda;
		double m_px;
    double m_py;
    double m_u0;
    double m_v0;
		unsigned int m_height, m_width;
		double m_sceneDepth;
		vpCameraParameters m_cam;
		vpFeatureLuminance sId; // desired visual features for visual servoing
    vpFeatureLuminance sI;  // current visual features for visual servoing
    vpServo servo; // visual servoing task
		vpColVector m_v;
		double m_normError;

    int m_iter;
		bool vsInitialized;
		bool m_controlInBaseFrame;

    std::ofstream m_logfile;
#ifdef INDICATORS
		std::ofstream m_residuals;
		std::ofstream m_times;
#endif
    string m_logs_path;
    string m_data_path;

		/*
    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;
    VisualServoTools m_visual_servo_tools;
		*/

};

#endif //__pvsPhotometricVisualServoing_H__
