#include "mppvsMPPVisualServoing.h"

#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>
#include <visp/vpIoTools.h>

#include <geometry_msgs/PoseStamped.h> //for evs
#include <filesystem> //for evs


mppvsMPPVisualServoing::mppvsMPPVisualServoing()
    : m_it(m_nh), m_iter(-1), vsInitialized(false), m_height(0), m_width(0),
      m_sceneDepth(1.0), m_v(6), m_v6(6), m_normError(1e12),
      m_pub_diffImage(false), _camera(nullptr), updateSampler(true),
      poseJacobianCompute(true), robust(false), nbDOF(6) {
  string cameraTopic, robotTopic, cameraPoseTopic, diffTopic, featuresDiffTopic,
      camxml, cam2tool;
  string desiredFeaturesImageTopic, featuresImageTopic;
  double f, ku;

  m_nh.param("cameraTopic", cameraTopic, string(""));
  m_nh.param("robotTopic", robotTopic, string(""));
  m_nh.param("diffTopic", diffTopic, string(""));
  m_nh.param("featuresDiffTopic", featuresDiffTopic, string(""));
  m_nh.param("featuresImageTopic", featuresImageTopic, string(""));
  m_nh.param("desiredFeaturesImageTopic", desiredFeaturesImageTopic,
             string(""));
  m_nh.param("logs", m_logs_path, string(""));
  m_nh.param("lambda", m_lambda, double(1.0));

  m_nh.param("cameraType", camtyp, int(0)); // 0 for dual fisheye and 1 for equirectangular
  m_nh.param("cameraXml", camxml, string(""));
	m_nh.param("camera2tool", cam2tool, string(""));
  m_nh.param("lambda_g", m_lambda_g, double(1.0));
  m_nh.param("subdivLevel", m_subdiv_level, int(3));

  m_nh.param("sceneDepth", m_sceneDepth, double(1.0));
  m_nh.param("controlInBaseFrame", m_controlInBaseFrame, false);
  m_nh.param("cameraPoseTopic", cameraPoseTopic, string(""));
  m_nh.param("rosbagForEVS", m_rosbagForEVS, false); //for evs
  m_nh.param("currentPoseTopicForEVS", m_currentPoseTopicForEVS, string("")); //for evs

  m_nh.getParam("cameraTopic", cameraTopic);
  m_nh.getParam("robotTopic", robotTopic);
  m_nh.getParam("diffTopic", diffTopic);
  m_nh.getParam("featuresDiffTopic", featuresDiffTopic);
  m_nh.getParam("featuresImageTopic", featuresImageTopic);
  m_nh.getParam("desiredFeaturesImageTopic", desiredFeaturesImageTopic);
  m_nh.getParam("logs", m_logs_path);
  m_nh.getParam("lambda", m_lambda);
  m_nh.getParam("cameraType", camtyp);
  m_nh.getParam("cameraXml", camxml);
	m_nh.getParam("camera2tool", cam2tool);
  m_nh.getParam("lambda_g", m_lambda_g);
  m_nh.getParam("subdivLevel", m_subdiv_level);
  m_nh.getParam("sceneDepth", m_sceneDepth);
  m_nh.getParam("controlInBaseFrame", m_controlInBaseFrame);
  m_nh.getParam("rosbagForEVS", m_rosbagForEVS); //for evs
   m_nh.getParam("currentPoseTopicForEVS", m_currentPoseTopicForEVS);//for evs

  for (int i = 0; i < 6; i++)
    m_dofs[i] = true; //change DoF here
// m_dofs[0] = true;
// m_dofs[1] = true;
// m_dofs[2] = false;
// m_dofs[3] = false;
// m_dofs[4] = false;
// m_dofs[5] = false;

  stringstream str;
  str<<m_logs_path<<"logfile.txt";
  m_logfile.open(str.str().c_str());

	if(cam2tool.compare("") != 0)
	{
		vpPoseVector r_ct;
		if(!vpPoseVector::loadYAML(cam2tool, r_ct))
			m_logfile << "no camera to tool transform loaded" << std::endl;

		m_tMc.buildFrom(r_ct);
		m_tVc.buildFrom(m_tMc);
	}

  if(m_controlInBaseFrame || m_rosbagForEVS) 
  {
    m_nh.getParam("cameraPoseTopic", cameraPoseTopic);

    m_camPose_sub = m_nh.subscribe(cameraPoseTopic, 1, &mppvsMPPVisualServoing::toolPoseCallback, this);
  }

  string camera_type = "spherical";

  m_stereo_cam = new prStereoModel(2);

  prStereoModelXML fromFile(camxml);
  fromFile >> *m_stereo_cam;
  truncGauss = true;
  
#ifdef INDICATORS
  str.str("");
  str << m_logs_path << "residuals.txt";
  m_residuals.open(str.str().c_str());

  str.str("");
  str << m_logs_path << "times.txt";
  m_times.open(str.str().c_str());
#endif

  if (diffTopic.compare("") != 0) {
    m_diff_pub = m_it.advertise(diffTopic, 1);
    m_pub_diffImage = true;
  }

  if (featuresDiffTopic.compare("") != 0) {
    m_featuresDiff_pub = m_it.advertise(featuresDiffTopic, 1);
    m_pub_diffFeaturesImage = true;
  }

  if (featuresImageTopic.compare("") != 0) {
    m_featuresImage_pub = m_it.advertise(featuresImageTopic, 1);
    m_pub_featuresImage = true;
  }

  if (desiredFeaturesImageTopic.compare("") != 0) {
    m_desiredFeaturesImage_pub = m_it.advertise(desiredFeaturesImageTopic, 1);
    m_pub_desiredFeaturesImage = true;
  }

  initVisualServoTask();

  m_image_sub = m_it.subscribe(cameraTopic, 1,
                               &mppvsMPPVisualServoing::imageCallback, this);

  m_velocity_pub = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1);
  // m_velocity_pub = m_nh.advertise<ros_dvs_bridge::VisualServoing>(robotTopic, 1);

  if(m_rosbagForEVS) //for evs
  {
    stringstream ss_bag, dst_bag;       
    ss_bag<<m_logs_path<<"/desiredAndCurrentPoses.bag";
		dst_bag<<m_logs_path<<"/desiredAndCurrentPoses"<< ((unsigned)ros::Time::now().toSec()) << camera_type<<".bag";
		std::filesystem::copy(ss_bag.str().c_str(), dst_bag.str().c_str());
		m_vsbag.open(dst_bag.str().c_str(),rosbag::bagmode::Append);
  }
}

mppvsMPPVisualServoing::~mppvsMPPVisualServoing() 
{ 
  stopRobot(); 
  	if(m_rosbagForEVS) //close rosbag for evs
	{
		m_vsbag.close();
	}
}

void mppvsMPPVisualServoing::stopRobot() {
  // ros_dvs_bridge::VisualServoing msg;
	// msg.cmd.linear.x = 0;
	// msg.cmd.linear.y = 0;
	// msg.cmd.linear.z = 0;
	// msg.cmd.angular.x = 0;
	// msg.cmd.angular.y = 0;
	// msg.cmd.angular.z = 0;
	// msg.error = m_normError; 
	// m_velocity_pub.publish(msg);

  m_velocity.linear.x = 0;
  m_velocity.linear.y = 0;
  m_velocity.linear.z = 0;
  m_velocity.angular.x = 0;
  m_velocity.angular.y = 0;
  m_velocity.angular.z = 0;
  m_velocity_pub.publish(m_velocity);
}

void mppvsMPPVisualServoing::initVisualServoTask() {
  // load desired image
  std::string filename_read_image;
  stringstream ss_desired_image;

  ss_desired_image << m_logs_path << "/Id"
                   << ".png";
  filename_read_image = vpIoTools::path(ss_desired_image.str().c_str());

  try {
    vpImageIo::read(this->m_desired_image, filename_read_image);
    m_logfile << "desired_image read from" << filename_read_image << endl;

    m_height = m_desired_image.getHeight();
    m_width = m_desired_image.getWidth();

    IS_req = new prRegularlySampledCSImage<unsigned char>(m_subdiv_level);
    IS_req->setInterpType(prInterpType::IMAGEPLANE_BILINEAR);

    scaleFactor = sqrt((double)m_height * (double)m_width /
                       (double)(0.5 * (20.0 * pow(4, m_subdiv_level)) + 2.0));
    resizedWidth = round((double)m_width / scaleFactor);
    resizedHeight = round((double)m_height / scaleFactor);

    m_desired_image_small = vpImage<unsigned char>(resizedHeight, resizedWidth);
    m_current_image_small = vpImage<unsigned char>(resizedHeight, resizedWidth);
    Mask_small = vpImage<unsigned char>(resizedHeight, resizedWidth, 255);
    MPP_cur_f = vpImage<float>(resizedHeight, resizedWidth, 0);
    MPP_des_f = vpImage<float>(resizedHeight, resizedWidth, 0);

    vpImageTools::resize(m_desired_image, m_desired_image_small,
                         vpImageTools::INTERPOLATION_AREA);
    m_ecam.init(resizedWidth * 0.5 / M_PI, resizedHeight * 0.5 / (M_PI * 0.5),
                resizedWidth * 0.5, resizedHeight * 0.5);

    switch (camtyp) {
    case 0: // dual fisheye
      for (int camNum = 0; camNum < 2; camNum++) {
        double u0 =
            ((prOmni *)(m_stereo_cam->sen[camNum]))->getu0() / scaleFactor;
        double v0 =
            ((prOmni *)(m_stereo_cam->sen[camNum]))->getv0() / scaleFactor;
        double au =
            ((prOmni *)(m_stereo_cam->sen[camNum]))->getau() / scaleFactor;
        double av =
            ((prOmni *)(m_stereo_cam->sen[camNum]))->getav() / scaleFactor;
        double xi = ((prOmni *)(m_stereo_cam->sen[camNum]))->getXi();

        ((prOmni *)(m_stereo_cam->sen[camNum]))->setPrincipalPoint(u0, v0);
        ((prOmni *)(m_stereo_cam->sen[camNum]))->setPixelRatio(au, av);

        std::cout << ((prOmni *)(m_stereo_cam->sen[camNum]))->au << ", "
                  << ((prOmni *)(m_stereo_cam->sen[camNum]))->av << ", "
                  << ((prOmni *)(m_stereo_cam->sen[camNum]))->u0 << ", "
                  << ((prOmni *)(m_stereo_cam->sen[camNum]))->v0 << std::endl;

        IS_req->buildFromTwinOmni(m_desired_image_small, *m_stereo_cam,
                                  &Mask_small);
        }
      break;

    case 1:
      IS_req->buildFromEquiRect(m_desired_image_small, m_ecam, &Mask_small);
      break;

    // case 2: 
    //   IS_req->buildFromFisheyeEqui(m_desired_image_small, m_ecam, &Mask_small);
    //   break;
    
    // case 2: //fisheye
    //   for (int camNum = 0; camNum < 1; camNum++) {
    //     double u0 =
    //         ((prOmni *)(m_stereo_cam->sen[camNum]))->getu0() / scaleFactor;
    //     double v0 =
    //         ((prOmni *)(m_stereo_cam->sen[camNum]))->getv0() / scaleFactor;
    //     double au =
    //         ((prOmni *)(m_stereo_cam->sen[camNum]))->getau() / scaleFactor;
    //     double av =
    //         ((prOmni *)(m_stereo_cam->sen[camNum]))->getav() / scaleFactor;
    //     double xi = ((prOmni *)(m_stereo_cam->sen[camNum]))->getXi();

    //     ((prOmni *)(m_stereo_cam->sen[camNum]))->setPrincipalPoint(u0, v0);
    //     ((prOmni *)(m_stereo_cam->sen[camNum]))->setPixelRatio(au, av);

    //     std::cout << ((prOmni *)(m_stereo_cam->sen[camNum]))->au << ", "
    //               << ((prOmni *)(m_stereo_cam->sen[camNum]))->av << ", "
    //               << ((prOmni *)(m_stereo_cam->sen[camNum]))->u0 << ", "
    //               << ((prOmni *)(m_stereo_cam->sen[camNum]))->v0 << std::endl;

    //     IS_req->buildFromOmni(m_desired_image_small, *m_stereo_cam,
    //                               &Mask_small);
    //   }
    //   break;

    // case 3: //perspective 

    //   break;


    default:
      break;
    }

    IS_req->toAbsZN();

    // double mean;
    // if(IS_req->isBitmapfSet == false)
    //   {
    //       if(IS_req->bitmapf != NULL)
    //       {
    //           delete [] IS_req->bitmapf;
    //           IS_req->bitmapf = NULL;
    //       }
    //       IS_req->bitmapf = new float[IS_req->nbSamples];
    //       IS_req->isBitmapfSet = true;
    //   }
    //   mean = 0.0;
    // //mean intensity computation
    // for(long ns = 0 ; ns < IS_req->nbSamples ; ns++, IS_req->bitmapf++)
    //     mean += *IS_req->bitmapf;
    // mean /= (double)IS_req->nbSamples;
    
    // //centering intensities
    // for(long ns = 0 ; ns < IS_req->nbSamples ; ns++, IS_req->bitmapf++)
    //     *IS_req->bitmapf = (double)(*IS_req->bitmapf) - mean;
    
    // //normalization of intensities by their standard deviation
    // double var = 0.0;
    // for(long ns = 0 ; ns < IS_req->nbSamples ; ns++, IS_req->bitmapf++)
    //     var += (mean - (double)*IS_req->bitmapf)*(mean - (double)*IS_req->bitmapf);
    // var /= (double)IS_req->nbSamples;
    // double sigma = sqrt(var);
    // for(long ns = 0 ; ns < IS_req->nbSamples ; ns++, IS_req->bitmapf++)
    //         *IS_req->bitmapf = (double)(*IS_req->bitmapf) / sigma;
    

    IS_cur = new prRegularlySampledCSImage<unsigned char>(m_subdiv_level);
    IS_cur->setInterpType(prInterpType::IMAGEPLANE_BILINEAR);

    GS = new prRegularlySampledCSImage<float>(m_subdiv_level);
    GS_sample_req = new prPhotometricGMS<prCartesian3DPointVec>(
        m_lambda_g, truncGauss == 1);
    GS_sample_cur = new prPhotometricGMS<prCartesian3DPointVec>(
        m_lambda_g, truncGauss == 1);
    GS_sample = new prPhotometricGMS<prCartesian3DPointVec>(m_lambda_g,
                                                            truncGauss == 1);

    fSet_req.buildFrom(*IS_req, *GS, *GS_sample_req, poseJacobianCompute);

    servo.setdof(m_dofs[0], m_dofs[1], m_dofs[2], m_dofs[3], m_dofs[4],
                 m_dofs[5]);
    servo.buildFrom(fSet_req);
    servo.initControl(m_lambda, true); // gain, cst_Z

    m_current_image_small = m_desired_image_small;

    switch (camtyp) {
    case 0:
      IS_cur->buildFromTwinOmni(m_current_image_small, *m_stereo_cam,
                                &Mask_small);
      break;

    case 1:
      IS_cur->buildFromEquiRect(m_current_image_small, m_ecam, &Mask_small);
      break;
    
    // case 2: 
    //   IS_req->buildFromFisheyeEqui(m_desired_image_small, m_ecam, &Mask_small);
    //   break;
    // case 2: 
    //   IS_cur->buildFromOmni(m_current_image_small, *m_stereo_cam,
    //                             &Mask_small);
    // break;

    default:
      break;
    }
    IS_cur->toAbsZN();
    
    // if(IS_cur->isBitmapfSet == false)
    // {
    //     if(IS_cur->bitmapf != NULL)
    //     {
    //         delete [] IS_cur->bitmapf;
    //         IS_cur->bitmapf = NULL;
    //     }
    //     IS_cur->bitmapf = new float[IS_cur->nbSamples];
    //     IS_cur->isBitmapfSet = true;
    // }
    // mean = 0.0;
    // //mean intensity computation
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //     mean += *IS_cur->bitmapf;
    // mean /= (double)IS_cur->nbSamples;
    
    // //centering intensities
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //     *IS_cur->bitmapf = (double)(*IS_cur->bitmapf) - mean;
    
    // //normalization of intensities by their standard deviation
    // var = 0.0;
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //     var += (mean - (double)*IS_cur->bitmapf)*(mean - (double)*IS_cur->bitmapf);
    // var /= (double)IS_cur->nbSamples;
    // sigma = sqrt(var);
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //         *IS_cur->bitmapf = (double)(*IS_cur->bitmapf) / sigma;

    fSet_cur.buildFrom(*IS_cur, *GS, *GS_sample_cur, poseJacobianCompute);

    m_logfile << "des built" << endl;
    if (m_pub_diffFeaturesImage || m_pub_desiredFeaturesImage) {
      MPP_des_f.resize(resizedHeight, resizedWidth, true);
      MPP_des_u.resize(resizedHeight, resizedWidth, true);
      rNull.set(0, 0, 0, 0, 0, 0);
      GS->toEquiRect(MPP_des_f, rNull, m_ecam);
      vpImageConvert::convert(MPP_des_f, MPP_des_u);

      m_logfile << "convert des " << MPP_des_u.getWidth() << " "
                << MPP_des_u.getHeight() << endl;
    }

    if (m_pub_desiredFeaturesImage) {
      m_desiredFeaturesImage = visp_bridge::toSensorMsgsImage(MPP_des_u);
      m_desiredFeaturesImage_pub.publish(m_desiredFeaturesImage);
    }

    m_logfile << "cur built " << m_height << " " << m_width << endl;
    if (m_pub_diffFeaturesImage) {
      MPP_cur_f.resize(resizedHeight, resizedWidth, true);
      MPP_cur_u.resize(resizedHeight, resizedWidth, true);

      GS->toEquiRect(MPP_cur_f, rNull, m_ecam);
      vpImageConvert::convert(MPP_cur_f, MPP_cur_u);

      m_logfile << "convert cur " << MPP_cur_u.getWidth() << " "
                << MPP_cur_u.getHeight() << endl;

      m_difference_mpp.resize(resizedHeight, resizedWidth, true);
    }

    if (m_pub_diffImage)
      m_difference_image.resize(m_height, m_width, true);

    vsInitialized = true;
  } catch (const vpException &e) {
    m_logfile << "Catch an exception: " << e << std::endl;
    vsInitialized = false;
  }
}

void mppvsMPPVisualServoing::imageCallback(
    const sensor_msgs::ImageConstPtr &image) {
  if (vsInitialized) {
    m_iter++;

#ifdef INDICATORS
    double duration = vpTime::measureTimeMs();
#endif

    m_current_image = visp_bridge::toVispImage(*image);

    if ((m_current_image.getHeight() == m_height) &&
        (m_current_image.getWidth() == m_width)) {

      vpImageTools::resize(m_current_image, m_current_image_small,
                           vpImageTools::INTERPOLATION_AREA);

      switch (camtyp) {
      case 0:
        IS_cur->buildFromTwinOmni(m_current_image_small, *m_stereo_cam,
                                  &Mask_small);
        break;
      case 1:
        IS_cur->buildFromEquiRect(m_current_image_small, m_ecam, &Mask_small);
        break;
    // case 2: 
    //   IS_req->buildFromFisheyeEqui(m_desired_image_small, m_ecam, &Mask_small);
    //   break;
    //   case 2: 
    //     IS_cur->buildFromOmni(m_current_image_small, *m_stereo_cam,
    //                             &Mask_small);
    // break;

      default:
        break;
      }
      IS_cur->toAbsZN();

    // double mean;
    
    // mean = 0.0;
    // //mean intensity computation
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //     mean += *IS_cur->bitmapf;
    // mean /= (double)IS_cur->nbSamples;
    
    // //centering intensities
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //     *IS_cur->bitmapf = (double)(*IS_cur->bitmapf) - mean;
    
    // //normalization of intensities by their standard deviation
    // double var = 0.0;
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //     var += (mean - (double)*IS_cur->bitmapf)*(mean - (double)*IS_cur->bitmapf);
    // var /= (double)IS_cur->nbSamples;
    // double sigma = sqrt(var);
    // for(long ns = 0 ; ns < IS_cur->nbSamples ; ns++, IS_cur->bitmapf++)
    //         *IS_cur->bitmapf = (double)(*IS_cur->bitmapf) / sigma;


      fSet_cur.buildFrom(*IS_cur, *GS, *GS_sample_cur, poseJacobianCompute);

      // Compute control vector
      m_normError = 0.5 * servo.control(fSet_cur, m_v, robust);

      // Compute current visual feature
      m_logfile << "cur updated" << endl;

      // update the DOFs
      indDOF = 0;
      for (numDOF = 0; numDOF < nbDOF; numDOF++)
        if (m_dofs[numDOF]) {
          m_v6[numDOF] = -m_v[indDOF];
          indDOF++;
        } else
          m_v6[numDOF] = 0;

      m_v6 = m_tVc * m_v6;

      if (m_controlInBaseFrame) 
      
        mutex_bVc.lock(); //for evs
        m_v6 = m_bVc * m_v6;
        mutex_bVc.unlock(); //for evs
      
      if(m_rosbagForEVS) //for evs
			{
				vpHomogeneousMatrix bMc;
				mutex_bMt.lock();
				bMc = m_bMt * m_tMc;
				mutex_bMt.unlock();

				geometry_msgs::PoseStamped currentRobotPoseStamped;
				//mutex_bMc.lock();
				vpTranslationVector t = bMc.getTranslationVector();
				vpQuaternionVector q = vpQuaternionVector(bMc.getRotationMatrix());
				//mutex_bMc.unlock();

				currentRobotPoseStamped.header.stamp = ros::Time::now();
				currentRobotPoseStamped.pose.position.x = t[0];
				currentRobotPoseStamped.pose.position.y = t[1];
				currentRobotPoseStamped.pose.position.z = t[2];
				currentRobotPoseStamped.pose.orientation.x = q.x();
				currentRobotPoseStamped.pose.orientation.y = q.y();
				currentRobotPoseStamped.pose.orientation.z = q.z();
				currentRobotPoseStamped.pose.orientation.w = q.w();
				
				m_vsbag.write(m_currentPoseTopicForEVS, ros::Time::now(), currentRobotPoseStamped);
			}

			// ros_dvs_bridge::VisualServoing msg;
			// msg.cmd.linear.x = m_v6[0];
			// msg.cmd.linear.y = m_v6[1];
			// msg.cmd.linear.z = m_v6[2];
			// msg.cmd.angular.x = m_v6[3];
			// msg.cmd.angular.y = m_v6[4];
			// msg.cmd.angular.z = m_v6[5];
			// msg.error = m_normError; 
			// m_velocity_pub.publish(msg);

      m_velocity.linear.x = m_v6[0];
      m_velocity.linear.y = m_v6[1];
      m_velocity.linear.z = m_v6[2];
      m_velocity.angular.x = m_v6[3];
      m_velocity.angular.y = m_v6[4];
      m_velocity.angular.z = m_v6[5];
      m_velocity_pub.publish(m_velocity);

      // if (m_pub_featuresImage) {
      //   fSet_cur.sampler.toEquiRect(MPP_cur_f, rNull, m_ecam);
      //   m_logfile << "curr feature image " << MPP_cur_f.getWidth() << " "
      //             << MPP_cur_u.getWidth() << " " << MPP_des_u.getWidth()
      //             << endl;

      //   for (int i = 0; i < MPP_cur_f.getHeight(); i++) {
      //     for (int j = 0; j < MPP_cur_f.getWidth(); j++) {
      //       std::cout << MPP_cur_f[i][j] << ", ";
      //     }
      //     std::cout << "\n";
      //   }
      //   vpImageConvert::convert(MPP_cur_f, MPP_cur_u);

      //   m_featuresImage = visp_bridge::toSensorMsgsImage(MPP_cur_u);
      //   m_featuresImage_pub.publish(m_featuresImage);
      // }

      if (m_pub_diffImage) {
        vpImageTools::imageDifference(m_current_image, m_desired_image,
                                      m_difference_image);
        m_logfile << "m_difference_image computed" << endl;
        m_diff = visp_bridge::toSensorMsgsImage(m_difference_image);
        m_diff_pub.publish(m_diff);
      }

      // if (m_pub_diffFeaturesImage) {
      //   if (!m_pub_featuresImage) {
      //     fSet_cur.sampler.toEquiRectF(MPP_cur_f, rNull, m_ecam);

      //     // for (int i = 0; i < fSet_cur.sampler.nbSamples; i++) {
      //     //   std::cout << fSet_cur.sampler.bitmapf[i] << ", ";
      //     // }
      //     // GS->toEquiRect(MPP_cur_f, rNull, m_ecam);
      //     m_logfile << MPP_cur_f.getWidth() << " " << MPP_cur_u.getWidth()
      //               << " " << MPP_des_u.getWidth() << endl;
      //     vpImageConvert::convert(MPP_cur_f, MPP_cur_u);
      //   }
      //   vpImageTools::imageDifference(MPP_cur_u, MPP_des_u,
      //   m_difference_mpp);

      //   m_logfile << "m_difference_mpp updated" << endl;

      //   m_featuresDiff = visp_bridge::toSensorMsgsImage(m_difference_mpp);
      //   m_featuresDiff_pub.publish(m_featuresDiff);
      // }

#ifdef INDICATORS
      m_times << vpTime::measureTimeMs() - duration << " ms" << std::endl;

      m_residuals << m_normError << std::endl; // |e|
#endif
      m_logfile << " |e| = " << m_normError << std::endl;
      m_logfile << " |v| = " << sqrt(m_v6.sumSquare()) << std::endl;
      m_logfile << " v = " << m_v6.t() << std::endl;
      std::cout << "v = " << m_v6.t() << std::endl;
    } else {
      m_logfile << "current image size " << m_current_image.getWidth() << "x"
                << m_current_image.getHeight()
                << " pixels different from the desired " << m_width << "x"
                << m_height << " pixels" << endl;
      
      // ros_dvs_bridge::VisualServoing msg;
			// msg.cmd.linear.x = 0;
			// msg.cmd.linear.y = 0;
			// msg.cmd.linear.z = 0;
			// msg.cmd.angular.x = 0;
			// msg.cmd.angular.y = 0;
			// msg.cmd.angular.z = 0;
			// msg.error = m_normError; 
			// m_velocity_pub.publish(msg);

      m_velocity.linear.x = 0;
      m_velocity.linear.y = 0;
      m_velocity.linear.z = 0;
      m_velocity.angular.x = 0;
      m_velocity.angular.y = 0;
      m_velocity.angular.z = 0;
      m_velocity_pub.publish(m_velocity);
    }
  } else
    m_logfile << "VS not initialized, cannot control motion." << std::endl;
}

void mppvsMPPVisualServoing::toolPoseCallback(const tf2_msgs::TFMessage &tf) //stuff for evs added
{
	if(tf.transforms[0].child_frame_id.compare("tool0_controller") == 0)
    {
      mutex_bMt.lock();
      m_bMt = /*visp_bridge::*/toVispHomogeneousMatrix(tf);
      mutex_bMt.unlock();
      
      vpHomogeneousMatrix bMt = m_bMt;
      bMt[0][3] = bMt[1][3] = bMt[2][3] = 0;

      //m_logfile << bMc << std::endl;
      mutex_bVc.lock();
      m_bVc.buildFrom(bMt);
      mutex_bVc.unlock();
    }
  // vpHomogeneousMatrix bMc = /*visp_bridge::*/ toVispHomogeneousMatrix(tf);
  // bMc[0][3] = bMc[1][3] = bMc[2][3] = 0;

  // // m_logfile << bMc << std::endl;

  // m_bVc.buildFrom(bMc);
}

vpHomogeneousMatrix mppvsMPPVisualServoing::toVispHomogeneousMatrix(const tf2_msgs::TFMessage &trans) {
  vpHomogeneousMatrix mat;
  vpTranslationVector vec(trans.transforms[0].transform.translation.x,
                          trans.transforms[0].transform.translation.y,
                          trans.transforms[0].transform.translation.z);
  vpRotationMatrix rmat;

  double a = trans.transforms[0].transform.rotation.w; // x
  double b = trans.transforms[0].transform.rotation.x; // y
  double c = trans.transforms[0].transform.rotation.y; // z
  double d = trans.transforms[0].transform.rotation.z; // w
  rmat[0][0] = a * a + b * b - c * c - d * d;
  rmat[0][1] = 2 * b * c - 2 * a * d;
  rmat[0][2] = 2 * a * c + 2 * b * d;

  rmat[1][0] = 2 * a * d + 2 * b * c;
  rmat[1][1] = a * a - b * b + c * c - d * d;
  rmat[1][2] = 2 * c * d - 2 * a * b;

  rmat[2][0] = 2 * b * d - 2 * a * c;
  rmat[2][1] = 2 * a * b + 2 * c * d;
  rmat[2][2] = a * a - b * b - c * c + d * d;

  mat.buildFrom(vec, rmat);

  return mat;
}

int mppvsMPPVisualServoing::errorToImage(vpColVector &e,
                                         vpImage<unsigned char> &m_diff_image) {
  if ((m_diff_image.getHeight() != m_height) ||
      (m_diff_image.getWidth() != m_width))
    return -1;

  double *pt_e = e.data;
  unsigned char *pt_Idiff;
  unsigned int bord = 10; // protected but always 10 in vpFeatureLuminance
  // for(unsigned int i = 0 ; i < e.size() ; i++, pt_Idiff++, pt_e++)
  for (unsigned int i = bord; i < m_height - bord; i++) 
  {
    pt_Idiff = &(m_diff_image[i][bord]);
    for (unsigned int j = bord; j < m_width - bord; j++, pt_Idiff++, pt_e++)
      *pt_Idiff = (*pt_e + 255) * 0.5;
  }

  return 0;
}
