#include "pgmvsPGMVisualServoing.h"
#include "ros_dvs_bridge/diff_lambda.h" //service header

#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <filesystem>

pgmvsPGMVisualServoing::pgmvsPGMVisualServoing()
    : m_it(m_nh), m_iter(-1), vsInitialized(false), m_height(0), m_width(0), m_sceneDepth(1.0), 
	  m_v(6), m_v6(6), m_normError(1e12), m_pub_diffImage(false),  m_pub_diffFeaturesImage(false),
	  m_pub_featuresImage(false), m_pub_desiredFeaturesImage(false), _camera(nullptr),
	  updateSampler(true), poseJacobianCompute(true), robust(false),
	  nbDOF(6), m_flagSecondStepVS(false), DIFF_VELO(1e-3), RESIDUAL_THRESHOLD(1e-4), saveInitial(false), m_normError_treshold(1e12), initialized_m_bMt(false)
{
    string cameraTopic, robotTopic, cameraPoseTopic, diffTopic, featuresDiffTopic, camxml, cam2tool, costTopic, velocityTopic;
	string desiredFeaturesImageTopic, featuresImageTopic;
	double f, ku, initial_lambda_g;
	int camtyp;	 

    m_nh.param("cameraTopic", cameraTopic, string(""));
	m_nh.param("robotTopic", robotTopic, string(""));
	m_nh.param("diffTopic", diffTopic, string(""));
	m_nh.param("featuresDiffTopic", featuresDiffTopic, string(""));
	m_nh.param("featuresImageTopic", featuresImageTopic, string(""));
	m_nh.param("desiredFeaturesImageTopic", desiredFeaturesImageTopic, string(""));
    m_nh.param("logs", m_logs_path, string(""));
    m_nh.param("lambda", m_lambda, double(1.0));
	m_nh.param("cameraType", camtyp, int(1)); //1: Perspective (same numbering as libPeR's CameraModelType)
    m_nh.param("cameraXml", camxml, string(""));
	m_nh.param("camera2tool", cam2tool, string(""));
	// m_nh.param("lambda_g", m_lambda_g, double(1.0));
    m_nh.param("sceneDepth", m_sceneDepth, double(1.0));
    m_nh.param("controlInBaseFrame", m_controlInBaseFrame, false);
    m_nh.param("cameraPoseTopic", cameraPoseTopic, string(""));
	m_nh.param("rosbagForEVS", m_rosbagForEVS, false);
    m_nh.param("currentPoseTopicForEVS", m_currentPoseTopicForEVS, string(""));
	m_nh.param("costTopic", costTopic, string(""));
	m_nh.param("velocityTopic", velocityTopic, string(""));
	m_nh.param("saveExprimentData", m_saveExperimentData, false);
	m_nh.param("twoStepVS", m_twoStepVS, bool(false));

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
	m_nh.getParam("sceneDepth", m_sceneDepth);
    m_nh.getParam("controlInBaseFrame", m_controlInBaseFrame);
	m_nh.getParam("rosbagForEVS", m_rosbagForEVS);
    m_nh.getParam("currentPoseTopicForEVS", m_currentPoseTopicForEVS);
	m_nh.getParam("costTopic", costTopic);
	m_nh.getParam("velocityTopic", velocityTopic);
	m_nh.getParam("saveExprimentData", m_saveExperimentData);
	m_nh.getParam("twoStepVS", m_twoStepVS);

	for(int i = 0 ; i<6; i++)
		m_dofs[i] = true;

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
		
		m_camPose_sub = m_nh.subscribe(cameraPoseTopic, 1, &pgmvsPGMVisualServoing::toolPoseCallback, this);
	}

	if(costTopic.compare("")!=0){
		m_cost_pub = m_nh.advertise<std_msgs::Float32>(costTopic, 1);
		m_pub_cost = true;
	}

	if (velocityTopic.compare("") != 0) {
    	m_velo_pub = m_nh.advertise<std_msgs::Float32>(velocityTopic, 1);
    	m_pub_velocity = true;
	}


	switch(camtyp)
	{
		case Omni:
		{
			camera_type = "fisheye";
	    	_camera = new prOmni();

    		// Load the camera parameters from the XML file
    		prOmniXML fromFile(camxml.c_str());

			fromFile >> (*((prOmni *)_camera));
			m_logfile<<"prOmni: "<<_camera->getau()<<endl;

			break;
		}
		case Equirectangular:
		{
			camera_type = "equirect";
			_camera = new prEquirectangular();

			// Load the camera parameters from the XML file
			prEquirectangularXML fromFile(camxml.c_str());

			fromFile >> (*((prEquirectangular*)_camera));

			m_logfile<<"prEquirectangular: "<<_camera->getau()<<endl;

			break;
		}
		case Persp:
		{
			camera_type = "perspective";
			//including P for perspective camera, which is the default
			_camera = new prPerspective();

			// Load the camera parameters from the XML file
			prPerspectiveXML fromFile(camxml.c_str());

			fromFile >> (*((prPerspective*)_camera));
			m_logfile<<"prPerspective: "<<_camera->getau()<<endl;
			break;
		}
		default:
			m_logfile<<"camera model not considered"<<endl;
	}

#ifdef INDICATORS
	str.str("");
	str<<m_logs_path<<"residuals.txt";
    m_residuals.open(str.str().c_str());

	str.str("");
	str<<m_logs_path<<"times.txt";
    m_times.open(str.str().c_str());

	str.str("");
	str<<m_logs_path<<"velocities.txt";
    m_velocities.open(str.str().c_str());
#endif

	if(diffTopic.compare("") != 0)
	{
		m_diff_pub = m_it.advertise(diffTopic, 1);
		m_pub_diffImage = true;
	}

	if(featuresDiffTopic.compare("") != 0)
	{
		m_featuresDiff_pub = m_it.advertise(featuresDiffTopic, 1);
		m_pub_diffFeaturesImage = true;
	}

	if(featuresImageTopic.compare("") != 0)
	{
		m_featuresImage_pub = m_it.advertise(featuresImageTopic, 1);
		m_pub_featuresImage = true;
	}

	if(desiredFeaturesImageTopic.compare("") != 0)
	{
		m_desiredFeaturesImage_pub = m_it.advertise(desiredFeaturesImageTopic, 1);
		m_pub_desiredFeaturesImage = true;
	}

	initVisualServoTask();

    m_image_sub = m_it.subscribe(cameraTopic, 1, &pgmvsPGMVisualServoing::imageCallback, this);

	m_velocity_pub = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1);

	if(m_rosbagForEVS)
	{
		stringstream ss_bag, dst_bag;       
        ss_bag<<m_logs_path<<"/desiredAndCurrentPoses.bag";
		dst_bag<<m_logs_path<<"/desiredAndCurrentPoses" <<((unsigned)ros::Time::now().toSec()) <<camera_type <<".bag";
		std::filesystem::copy(ss_bag.str().c_str(), dst_bag.str().c_str());
		bagFilePath = dst_bag.str().c_str();
		m_vsbag.open(dst_bag.str().c_str(),rosbag::bagmode::Append);
	}
}

pgmvsPGMVisualServoing::~pgmvsPGMVisualServoing()
{
	// std::cout << "Im in the destuctor just before" << std::endl;

	m_image_sub.shutdown();
	if(m_rosbagForEVS && m_vsbag.isOpen())
	{
		// std:cout << "Im in the destuctor just before closing the rosbag" << std::endl;
		m_vsbag.close();
		std::cout << "rosbag closed" << std::endl;
	}
	// std::cout << "Im in the destuctor just before closing the log file" << std::endl;
	m_logfile.close();
	// std::cout << "logfile closed" << std::endl;
  	stopRobot();

	m_nh.shutdown();
}

void
pgmvsPGMVisualServoing::stopRobot()
{
	if (m_saveExperimentData) {
        saveExperimentData();
		std::cout << "Experiment Data saved" << std::endl;
    }

	m_velocity.linear.x = 0;
	m_velocity.linear.y = 0;
	m_velocity.linear.z = 0;
	m_velocity.angular.x = 0;
	m_velocity.angular.y = 0;
  	m_velocity.angular.z = 0;
	m_velocity_pub.publish(m_velocity);
}

void pgmvsPGMVisualServoing::initVisualServoTask()
{ 
	//load desired image
	std::string filename_read_image;
  stringstream ss_desired_image;
  
  ss_desired_image<<m_logs_path<<"/Id_des"<<".png";
  filename_read_image = vpIoTools::path(ss_desired_image.str().c_str());

	try 
	{
  		vpImageIo::read(this->m_desired_image, filename_read_image);
  		m_logfile<<"desired_image read from"<<filename_read_image<<endl;

		m_height = m_desired_image.getHeight();
		m_width = m_desired_image.getWidth();

		m_logfile<< m_height << " x " << m_width <<endl;

		// 2. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
		servo.setdof(m_dofs[0], m_dofs[1], m_dofs[2], m_dofs[3], m_dofs[4], m_dofs[5]);
		
		servo.setSensor(_camera);

    	// desired visual feature built from the image
		//prepare the desired image 
    	IP_des = new prRegularlySampledCPImage<unsigned char>(m_height, m_width); //the regularly sample planar image to be set from the acquired/loaded perspective image
    	IP_des->setInterpType(INTERPTYPE);
    	IP_des->buildFrom(m_desired_image, _camera); 

		// IP_des->toAbsZN(); 		//adding normalization of intensities

    	GP = new prRegularlySampledCPImage<float>(m_height, m_width); //contient tous les pr2DCartesianPointVec (ou prFeaturePoint) u_g et fera GS_sample.buildFrom(IP_des, u_g);

    	GP_sample_des = new prPhotometricnnGMS<prCartesian2DPointVec>(m_lambda_g);

    	fSet_des.buildFrom(*IP_des, *GP, *GP_sample_des, false, true); // Goulot !

		m_logfile << "des built" << endl;
		if(m_pub_diffFeaturesImage || m_pub_desiredFeaturesImage)
		{
			PGM_des_f.resize(m_height, m_width, true);
			PGM_des_u.resize(m_height, m_width, true);
			fSet_des.sampler.toImage(PGM_des_f, pp, _camera);
			vpImageConvert::convert(PGM_des_f, PGM_des_u);

			m_logfile << "convert des " << PGM_des_u.getWidth() << " " << PGM_des_u.getHeight() << endl;
		}

		if(m_pub_desiredFeaturesImage)
		{
			m_desiredFeaturesImage = visp_bridge::toSensorMsgsImage(PGM_des_u);
			m_desiredFeaturesImage_pub.publish(m_desiredFeaturesImage);
		}

		servo.buildFrom(fSet_des);

		servo.initControl(m_lambda, m_sceneDepth);

   		// current visual feature built from the image
		m_current_image = m_desired_image;

		GP_sample = new prPhotometricnnGMS<prCartesian2DPointVec>(m_lambda_g);

		// Current features set setting from the current image
    	IP_cur = new prRegularlySampledCPImage<unsigned char>(m_height, m_width);
    	IP_cur->setInterpType(INTERPTYPE);
    	IP_cur->buildFrom(m_current_image, _camera); 

		// IP_cur->toAbsZN(); 		//adding normalization of intensities

    	fSet_cur.buildFrom(*IP_cur, *GP, *GP_sample, poseJacobianCompute, updateSampler); // Goulot !

		m_logfile << "cur built " << m_height << " " << m_width << endl;
		if(m_pub_diffFeaturesImage || m_pub_featuresImage)
		{
			PGM_cur_f.resize(m_height, m_width, true);
			PGM_cur_u.resize(m_height, m_width, true);
			fSet_cur.sampler.toImage(PGM_cur_f, pp, _camera);
			vpImageConvert::convert(PGM_cur_f, PGM_cur_u);

			m_logfile << "convert cur " << PGM_cur_u.getWidth() << " " << PGM_cur_u.getHeight() << endl;
		}

    	if(m_pub_diffFeaturesImage)
		{
			m_difference_pgm.resize(m_height, m_width, true);
		}

		if(m_pub_diffImage)
		{
			m_difference_image.resize(m_height, m_width, true);
		}

		vsInitialized = true;
	} catch (const vpException &e) {
    m_logfile << "Catch an exception: " << e << std::endl;
    vsInitialized = false;
  }

  t = ros::Time::now();
  std::cout << "Visual Servoing initialized: " << vsInitialized << std::endl;

}

void pgmvsPGMVisualServoing::imageCallback(const sensor_msgs::ImageConstPtr &image) {
	lastVelocities[MAX_STAGNATION_ITER] = {0};  
	stagnationCount = 0;                 
	iterCount = 0; 

	if(image->header.stamp <= t)
	{
		m_logfile << "late message " << image->header.stamp << " vs " << t << endl;
		return;
	}		

	if(vsInitialized)
	{
    	m_iter++;

#ifdef INDICATORS
    	double duration = vpTime::measureTimeMs();
#endif
		m_current_image = visp_bridge::toVispImage(*image);

		if(!saveInitial){
			// m_normError_treshold = m_normError * 0.2; //setting threshold to 20% of initial residual
			// std::cout << "normError_treshold: " << m_normError_treshold << std::endl;
			std::string filename_write_image;
			stringstream ss_initial_image;

			ss_initial_image<<m_logs_path<<"/Id_init.png";
			filename_write_image = vpIoTools::path(ss_initial_image.str().c_str());

			vpImageIo::write(this->m_current_image, filename_write_image);
			m_logfile<<"initial_image written to"<<filename_write_image<<endl;

		client = m_nh.serviceClient<ros_dvs_bridge::diff_lambda>("diff_lambda");
		ros_dvs_bridge::diff_lambda srv;
		srv.request.desired_image_path = m_logs_path + "/Id_des.png";
		srv.request.init_image_path = m_logs_path + "/Id_init.png";

		if(client.call(srv)){
			double initial_lambda_g = srv.response.lambda_g;
			if (initial_lambda_g >= 5 && camera_type == "perspective" || initial_lambda_g >= 10){
				m_logfile << "calculated initial lambda_g too high: " << initial_lambda_g << endl;
				initial_lambda_g = 2.00;
			} 
			std::cout << "------------------------------lambda_g: " << initial_lambda_g << std::endl;
			m_logfile << "calculated initial lambda_g: " << initial_lambda_g << endl;
 			updateParameters(initial_lambda_g, m_lambda, m_sceneDepth);
		}
		else{
			ROS_ERROR("Failed to call service diff_lambda");
		}

			saveInitial = true;	

		}



			//update the DOFs
			indDOF = 0;
			for (numDOF = 0 ; numDOF < nbDOF ; numDOF++)
				if (m_dofs[numDOF])
				{
					m_v6[numDOF] = -m_v[indDOF];
					indDOF++;
				}
				else
					m_v6[numDOF] = 0;
			
			if((m_current_image.getHeight() == m_height) && (m_current_image.getWidth() == m_width)){
			IP_cur->buildFrom(m_current_image, _camera); 

			// IP_cur->toAbsZN(); //adding normalization of intensities

			fSet_cur.updateMeasurement(*IP_cur, *GP, *GP_sample, poseJacobianCompute, updateSampler);  
			m_logfile << "cur updated" << endl;

			//Compute control vector
			m_normError = 0.5*servo.control(fSet_cur, m_v, robust);
			// std::cout << "m_normerror: " << m_normError << std::endl; 

			//---------------test extrinsic camera_calib here-------------------
			// m_v6 = 0; 
			// m_v6[4] = 0.05; //sending slow velocity commands (0 for transl. x in camera frame)
			//------------------------------------------------------------------

			m_v6 = m_tVc * m_v6;

			if(m_controlInBaseFrame)
			{
				mutex_bVt.lock();
				m_v6 = m_bVt * m_v6;
				mutex_bVt.unlock();
			}

			if(m_twoStepVS){
				lastVelocities[iterCount % MAX_STAGNATION_ITER] = sqrt(m_v6.sumSquare()); 
				iterCount++;

				bool stagnant = true; 
				for(int i = 1; i < MAX_STAGNATION_ITER; ++i) {
					if(fabs(lastVelocities[i] - lastVelocities[i-1]) > DIFF_VELO || lastVelocities[i] == 0.0) {
						stagnant = false;
						// std::cout << "stagnant: " << stagnant << std::endl;
						break;
					}
				}


				if ( stagnant  && sqrt(m_v6.sumSquare()) < RESIDUAL_THRESHOLD && !m_flagSecondStepVS) { //&& sqrt(m_v6.sumSquare()) < RESIDUAL_THRESHOLD
					m_logfile << "Residuals stabilized, starting second step of vs task" << std::endl;
					std::cout << "--------------------Initializing second step of vs--------------------" << std::endl; 
					// stopRobot();  

					double new_lambda_g = 1; //updating parameters for 2. step
					// if (new_lambda_g <= 0.3){ //otherwise robot crazy 
					// 	new_lambda_g = 0.5;
					// }
					double new_lambda = 0.5;
					updateParameters(new_lambda_g, new_lambda, m_sceneDepth);
					m_flagSecondStepVS = true; //set flag true 
					return;
				}
			}



			if(m_rosbagForEVS && initialized_m_bMt) 
			{
				if(!initialized_m_bMt){
				std::cout<<"not initialized"<<std::endl;
				}
				
				geometry_msgs::PoseStamped currentRobotPoseStamped;
				mutex_bMt.lock();
				vpTranslationVector t = m_bMt.getTranslationVector();
				vpQuaternionVector q = vpQuaternionVector(m_bMt.getRotationMatrix());
				mutex_bMt.unlock();
				

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
		
			m_velocity.linear.x = m_v6[0];
			m_velocity.linear.y = m_v6[1];
			m_velocity.linear.z = m_v6[2];
			m_velocity.angular.x = m_v6[3];
			m_velocity.angular.y = m_v6[4];
			m_velocity.angular.z = m_v6[5];
			m_velocity_pub.publish(m_velocity);

			t = ros::Time::now();
			
			if(m_pub_featuresImage)
			{
				fSet_cur.sampler.toImage(PGM_cur_f, pp, _camera);
				m_logfile << PGM_cur_f.getWidth() << " " << PGM_cur_u.getHeight() << endl;
				vpImageConvert::convert(PGM_cur_f, PGM_cur_u);

				m_featuresImage = visp_bridge::toSensorMsgsImage(PGM_cur_u);
				m_featuresImage_pub.publish(m_featuresImage);
			}

			if(m_pub_diffImage)
			{
				vpImageTools::imageDifference(m_current_image, m_desired_image, m_difference_image) ;
				m_logfile << "m_difference_image computed" << endl;
				m_diff = visp_bridge::toSensorMsgsImage(m_difference_image);
				m_diff_pub.publish(m_diff);
			}

			if(m_pub_diffFeaturesImage)
			{				
				if(!m_pub_featuresImage)
				{
					fSet_cur.sampler.toImage(PGM_cur_f, pp, _camera);
					m_logfile << PGM_cur_f.getWidth() << " " << PGM_cur_u.getWidth() << " " << PGM_des_u.getWidth() << endl;
					vpImageConvert::convert(PGM_cur_f, PGM_cur_u);
				}
				vpImageTools::imageDifference(PGM_cur_u,PGM_des_u,m_difference_pgm) ;
				
				m_logfile << "m_difference_pgm updated" << endl;

				m_featuresDiff = visp_bridge::toSensorMsgsImage(m_difference_pgm);
				m_featuresDiff_pub.publish(m_featuresDiff);
			}
			
			if(m_pub_cost){
				m_cost.data = m_normError;
				m_cost_pub.publish(m_cost);
			}

			if(m_pub_velocity){
				m_velo.data = sqrt(m_v6.sumSquare());
				m_velo_pub.publish(m_velo);
			}

#ifdef INDICATORS
			m_times << vpTime::measureTimeMs() - duration << " ms" << std::endl;

			m_residuals << m_normError << std::endl; // |e|

			m_velocities << m_v6.t() << std::endl; 
#endif 
			m_logfile << " |e| = " << m_normError << std::endl;
			m_logfile << " |v| = " << sqrt(m_v6.sumSquare()) << std::endl;
			m_logfile << " v = " << m_v6.t() << std::endl;
			// ROS_INFO("Speedb updated"); 
			//std::cout << " v = " << m_v6.t() << std::endl;
			// std::cout << " |v| = " << sqrt(m_v6.sumSquare()) << std::endl;
   		 }
		else
		{
			m_logfile<<"current image size " << m_current_image.getWidth() << "x" << m_current_image.getHeight() <<  " pixels different from the desired " << m_width << "x" << m_height<<" pixels" << endl;

			m_velocity.linear.x = 0;
			m_velocity.linear.y = 0;
			m_velocity.linear.z = 0;
			m_velocity.angular.x = 0;
			m_velocity.angular.y = 0;
      		m_velocity.angular.z = 0;
			m_velocity_pub.publish(m_velocity);
		}
	}
	else
		m_logfile<<"VS not initialized, cannot control motion." << std::endl;
}

void pgmvsPGMVisualServoing::toolPoseCallback(const tf2_msgs::TFMessage& tf)
{
	if(tf.transforms[0].child_frame_id.compare("tool0_controller") == 0)
    {
		mutex_bMt.lock();
		m_bMt = /*visp_bridge::*/toVispHomogeneousMatrix(tf);
		initialized_m_bMt = true; 
		mutex_bMt.unlock();
		
		vpHomogeneousMatrix bMt = m_bMt;
		bMt[0][3] = bMt[1][3] = bMt[2][3] = 0;

		//m_logfile << bMc << std::endl;
		mutex_bVt.lock();
  		m_bVt.buildFrom(bMt);
		mutex_bVt.unlock();

	}
}

void pgmvsPGMVisualServoing::updateParameters(double new_lambda_g, double new_lambda, double new_sceneDepth) {

    if (new_lambda_g != m_lambda_g) {
		const std::lock_guard<std::mutex> lock(lambda_mutex);
        std::cout << "Updating lambda_g from " << m_lambda_g << " to " << new_lambda_g << std::endl;
        m_lambda_g = new_lambda_g;
        // vsInitialized = false;
        initVisualServoTask();
    }

    if (new_lambda != m_lambda) {
        std::cout << "Updating lambda from " << m_lambda << " to " << new_lambda << std::endl;
        m_lambda = new_lambda;
        servo.initControl(m_lambda, m_sceneDepth); 
    }

    if (new_sceneDepth != m_sceneDepth) {
        std::cout << "Updating sceneDepth from " << m_sceneDepth << " to " << new_sceneDepth << std::endl;
        m_sceneDepth = new_sceneDepth;
        servo.initControl(m_lambda, m_sceneDepth); 
    }
}

void 
pgmvsPGMVisualServoing::saveExperimentData(){
	std::string user_folder_name;
    std::cout << "Enter the name for the folder: ";
    std::getline(std::cin, user_folder_name);

	std::string exp;
	std::stringstream exp_path;
    exp_path << m_logs_path << "/exp" << user_folder_name << camera_type;
	std::filesystem::create_directories(exp_path.str());

	std::filesystem::copy(m_logs_path + "/velocities.txt", exp_path.str() + "/velocities.txt");
	std::filesystem::copy(m_logs_path + "/residuals.txt", exp_path.str() + "/residuals.txt");
	std::filesystem::copy(m_logs_path + "/logfile.txt", exp_path.str() + "/logfile.txt");
	std::filesystem::copy(m_logs_path + "/times.txt", exp_path.str() + "/times.txt");
	std::filesystem::copy(m_logs_path + "/Id_des.png", exp_path.str() + "/Id_des.png");
	std::filesystem::copy(m_logs_path + "/Id_init.png", exp_path.str() + "/Id_init.png");

	std::filesystem::copy(bagFilePath, exp_path.str());
}


vpHomogeneousMatrix
pgmvsPGMVisualServoing::toVispHomogeneousMatrix(const tf2_msgs::TFMessage& trans)
{
	vpHomogeneousMatrix mat;
	vpTranslationVector vec(trans.transforms[0].transform.translation.x,trans.transforms[0].transform.translation.y,trans.transforms[0].transform.translation.z);
	vpRotationMatrix rmat;

	double a = trans.transforms[0].transform.rotation.w; //x
	double b = trans.transforms[0].transform.rotation.x; //y
	double c = trans.transforms[0].transform.rotation.y; //z
	double d = trans.transforms[0].transform.rotation.z; //w
	rmat[0][0] = a*a+b*b-c*c-d*d;
	rmat[0][1] = 2*b*c-2*a*d;
	rmat[0][2] = 2*a*c+2*b*d;

	rmat[1][0] = 2*a*d+2*b*c;
	rmat[1][1] = a*a-b*b+c*c-d*d;
	rmat[1][2] = 2*c*d-2*a*b;

	rmat[2][0] = 2*b*d-2*a*c;
	rmat[2][1] = 2*a*b+2*c*d;
	rmat[2][2] = a*a-b*b-c*c+d*d;

	mat.buildFrom(vec,rmat);

	return mat;
}

int pgmvsPGMVisualServoing::errorToImage(vpColVector &e, vpImage<unsigned char> &m_diff_image)
{
	if((m_diff_image.getHeight() != m_height) || (m_diff_image.getWidth() != m_width))
		return -1;

	double *pt_e = e.data;	
	unsigned char *pt_Idiff;
	unsigned int bord = 10; //protected but always 10 in vpFeatureLuminance
	//for(unsigned int i = 0 ; i < e.size() ; i++, pt_Idiff++, pt_e++)
	for (unsigned int i = bord; i < m_height - bord; i++)
	{
		pt_Idiff = &(m_diff_image[i][bord]);
    for (unsigned int j = bord; j < m_width - bord; j++, pt_Idiff++, pt_e++)
			*pt_Idiff = (*pt_e + 255) * 0.5;
	}
	
	return 0;
}
