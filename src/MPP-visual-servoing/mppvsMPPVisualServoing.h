#ifndef __mppvsMPPVisualServoing_H__
#define __mppvsMPPVisualServoing_H__

#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32.h>

#include "tf2_msgs/TFMessage.h"

#include <rosbag/bag.h> //for evs

#include "visp_bridge/image.h"

// Camera types that can be considered
#include <per/prEquirectangular.h>
#include <per/prEquirectangularXML.h>
#include <per/prOmni.h>
#include <per/prOmniXML.h>
#include <per/prStereoModel.h>
#include <per/prStereoModelXML.h>

// Acquisition model
#include <per/prRegularlySampledCSImage.h>

// Visual features
#include <per/prFeaturesSet.h>
#include <per/prPhotometricGMS.h>
#include <per/prSSDCmp.h>

// Generic Visual Servo Control tools
#include <per/prPoseSphericalEstim.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

#define INDICATORS

using namespace std;

class mppvsMPPVisualServoing {

public:
  mppvsMPPVisualServoing();
  ~mppvsMPPVisualServoing();

  void initVisualServoTask();

  void imageCallback(const sensor_msgs::ImageConstPtr &image);

  void toolPoseCallback(const tf2_msgs::TFMessage &tf);
  vpHomogeneousMatrix toVispHomogeneousMatrix(const tf2_msgs::TFMessage &trans);
  int errorToImage(vpColVector &e, vpImage<unsigned char> &m_diff_image);

  void stopRobot();

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
  vpVelocityTwistMatrix m_bVc;
  std::mutex mutex_bVc; //evs
  vpHomogeneousMatrix m_tMc;
  vpVelocityTwistMatrix m_tVc;

  vpHomogeneousMatrix m_bMc; //evs
  vpHomogeneousMatrix m_bMt; //evs
  std::mutex mutex_bMt; //evs

  rosbag::Bag m_vsbag; //evs

  geometry_msgs::Twist m_velocity;
  sensor_msgs::Image m_diff;
  sensor_msgs::Image m_featuresDiff;
  sensor_msgs::Image m_featuresImage;
  sensor_msgs::Image m_desiredFeaturesImage;

  vpImage<unsigned char> m_desired_image, m_desired_image_small, Mask_small;
  vpImage<float> MPP_des_f;
  vpImage<unsigned char> MPP_des_u;
  vpPoseVector pp;

  vpImage<unsigned char> m_current_image, m_current_image_small;
  vpImage<float> MPP_cur_f;
  vpImage<unsigned char> MPP_cur_u;

  vpImage<unsigned char> m_difference_image;
  vpImage<unsigned char> m_difference_mpp;

  double m_mu;
  double m_lambda;
  double scaleFactor;
  int m_subdiv_level;
  bool truncGauss;
  int camtyp;

  unsigned int m_height, m_width;
  unsigned int resizedWidth, resizedHeight;
  double m_sceneDepth;

  prCameraModel *_camera;
  prStereoModel *m_stereo_cam;
  prEquirectangular m_ecam;

  //-------- LibPeR
  prRegularlySampledCSImage<unsigned char> *IS_req, *IS_cur;
  prRegularlySampledCSImage<float> *GS;
  prPhotometricGMS<prCartesian3DPointVec> *GS_sample_req, *GS_sample_cur,
      *GS_sample;
  prFeaturesSet<prCartesian3DPointVec, prPhotometricGMS<prCartesian3DPointVec>,
                prRegularlySampledCSImage>
      fSet_req, fSet_cur;
  prPoseSphericalEstim<
      prFeaturesSet<prCartesian3DPointVec,
                    prPhotometricGMS<prCartesian3DPointVec>,
                    prRegularlySampledCSImage>,
      prSSDCmp<prCartesian3DPointVec, prPhotometricGMS<prCartesian3DPointVec>>>
      servo;

  double m_lambda_g;
  bool updateSampler;
  bool poseJacobianCompute;
  bool robust; // to activate the M-Estimator

  bool m_dofs[6];
  vpColVector m_v, m_v6;
  vpPoseVector rNull;
  double m_normError;

  unsigned int nbDOF, numDOF, indDOF;

  int m_iter;
  bool vsInitialized;
  bool m_controlInBaseFrame;
  bool m_pub_diffImage;
  bool m_pub_diffFeaturesImage;
  bool m_pub_featuresImage;
  bool m_pub_desiredFeaturesImage;

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

    bool m_rosbagForEVS; //for evs
    string m_currentPoseTopicForEVS;
};

#endif //__pgmvsPGMVisualServoing_H__
