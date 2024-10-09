#ifndef __pgmvsPGMVisualServoing_H__
#define __pgmvsPGMVisualServoing_H__

#include <mutex>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
//#include "ros_dvs_bridge/VisualServoing.h"


#include "tf2_msgs/TFMessage.h"

#include <rosbag/bag.h>

#include "visp_bridge/image.h"

//Camera types that can be considered
#include <per/prPerspective.h>
#include <per/prPerspectiveXML.h>
#include <per/prOmni.h>
#include <per/prOmniXML.h>
#include <per/prEquirectangular.h>
#include <per/prEquirectangularXML.h>

//Acquisition model
#include <per/prRegularlySampledCPImage.h>

//Visual features
#include <per/prPhotometricnnGMS.h>
#include <per/prFeaturesSet.h>
#include <per/prSSDCmp.h>

//Generic Visual Servo Control tools 
#include <per/prCameraPoseEstim.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR
#define MAX_STAGNATION_ITER 10 
#define INDICATORS

using namespace std;

class pgmvsPGMVisualServoing
{

public:

    pgmvsPGMVisualServoing();
		~pgmvsPGMVisualServoing();

		void initVisualServoTask();

		void imageCallback(const sensor_msgs::ImageConstPtr& image); 

    void toolPoseCallback(const tf2_msgs::TFMessage& tf); 
		vpHomogeneousMatrix toVispHomogeneousMatrix(const tf2_msgs::TFMessage& trans);   
		int errorToImage(vpColVector &e, vpImage<unsigned char> &m_diff_image);

		void stopRobot();

		void updateParameters(double new_lambda_g, double new_lambda, double new_sceneDepth);
		void saveExperimentData();
		
		rosbag::Bag m_vsbag;

private:

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;
    image_transport::Publisher m_diff_pub;
	image_transport::Publisher m_featuresDiff_pub;
	image_transport::Publisher m_featuresImage_pub;
	image_transport::Publisher m_desiredFeaturesImage_pub;
		ros::Publisher m_velocity_pub;

		ros::Subscriber m_camPose_sub; 
		vpVelocityTwistMatrix m_bVt;
		std::mutex mutex_bVt;
		vpHomogeneousMatrix m_tMc;
		vpVelocityTwistMatrix m_tVc;
		
		vpHomogeneousMatrix m_bMc;
		vpHomogeneousMatrix m_bMt;
		std::mutex mutex_bMt;

		bool m_pub_cost;
		ros::Publisher m_cost_pub;
		std_msgs::Float32 m_cost; 

		bool m_pub_velocity;
		ros::Publisher m_velo_pub; 
		std_msgs::Float32 m_velo;

		geometry_msgs::Twist m_velocity;
		sensor_msgs::Image m_diff;
		sensor_msgs::Image m_featuresDiff;
		sensor_msgs::Image m_featuresImage;
		sensor_msgs::Image m_desiredFeaturesImage;

		vpImage<unsigned char> m_desired_image;
		vpImage<float> PGM_des_f;
		vpImage<unsigned char> PGM_des_u;
		vpPoseVector pp;
		
		vpImage<unsigned char> m_current_image;
		vpImage<float> PGM_cur_f;
		vpImage<unsigned char> PGM_cur_u;

		vpImage<unsigned char> m_difference_image;
		vpImage<unsigned char> m_difference_pgm;

		double DIFF_VELO;
		double RESIDUAL_THRESHOLD;  
		double lastVelocities[MAX_STAGNATION_ITER];
		int stagnationCount;              
		int iterCount;


		ros::Time t;

		double m_mu;
    double m_lambda;
		
		unsigned int m_height, m_width;
		double m_sceneDepth;
		
		prCameraModel *_camera;

		
		// desired visual features for visual servoing
		prRegularlySampledCPImage<unsigned char> *IP_des;
		prRegularlySampledCPImage<float> *GP;
    	prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_des;
		prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample_des;

    	// current visual features for visual servoing
		prPhotometricnnGMS<prCartesian2DPointVec> *GP_sample;
		prRegularlySampledCPImage<unsigned char> *IP_cur;
		prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage > fSet_cur;

    	prCameraPoseEstim<prFeaturesSet<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec>, prRegularlySampledCPImage >, 
                      prSSDCmp<prCartesian2DPointVec, prPhotometricnnGMS<prCartesian2DPointVec> > > servo; // visual servoing task
		
		std::mutex lambda_mutex;
		double m_lambda_g;
		bool updateSampler;
    	bool poseJacobianCompute;
    	bool robust; //to activate the M-Estimator
		
		bool m_dofs[6];
		vpColVector m_v, m_v6;
		double m_normError;

		unsigned int nbDOF, numDOF, indDOF;

    int m_iter;
		bool vsInitialized;
		bool m_controlInBaseFrame;
		bool m_pub_diffImage;
		bool m_pub_diffFeaturesImage;
		bool m_pub_featuresImage;
		bool m_pub_desiredFeaturesImage;

		bool m_saveExperimentData; //saves residuals, velocities, evs ros bag, logfile, times
		std::string bagFilePath;  
		std::string camera_type; 

		bool m_twoStepVS; 
		bool m_flagSecondStepVS; //flag for reducing lambda_g only once

		bool saveInitial; 
		
    std::ofstream m_logfile;
#ifdef INDICATORS
		std::ofstream m_residuals;
		std::ofstream m_times;
		std::ofstream m_velocities;
#endif
    string m_logs_path;
    string m_data_path;

		/*
    vpDisplayX m_display_current_image;
    vpDisplayX m_display_desired_image;
    vpDisplayX m_display_error;
    VisualServoTools m_visual_servo_tools;
		*/

    bool m_rosbagForEVS;
    string m_currentPoseTopicForEVS;

};

#endif //__pgmvsPGMVisualServoing_H__
