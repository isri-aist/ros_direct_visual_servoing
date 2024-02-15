#include "pgmvsPGMVisualServoing.h"

#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

pgmvsPGMVisualServoing::pgmvsPGMVisualServoing()
    : m_it(m_nh), m_iter(-1), vsInitialized(false), m_height(0), m_width(0), m_sceneDepth(1.0), 
	  m_v(6), m_v6(6), m_normError(1e12), m_pub_diffImage(false),  m_pub_diffFeaturesImage(false),
	  m_pub_featuresImage(false), m_pub_desiredFeaturesImage(false), _camera(nullptr),
	  updateSampler(true), poseJacobianCompute(true), robust(false),
	  nbDOF(6)
{
    string cameraTopic, robotTopic, cameraPoseTopic, diffTopic, featuresDiffTopic, camxml;
	string desiredFeaturesImageTopic, featuresImageTopic;
	double f, ku;
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
	m_nh.param("lambda_g", m_lambda_g, double(1.0));

    m_nh.param("sceneDepth", m_sceneDepth, double(1.0));
    m_nh.param("controlInBaseFrame", m_controlInBaseFrame, false);
    m_nh.param("cameraPoseTopic", cameraPoseTopic, string(""));

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
	m_nh.getParam("lambda_g", m_lambda_g);
	m_nh.getParam("sceneDepth", m_sceneDepth);
    m_nh.getParam("controlInBaseFrame", m_controlInBaseFrame);

	for(int i = 0 ; i<6; i++)
		m_dofs[i] = true;
		
	if(m_controlInBaseFrame)
	{
		m_nh.getParam("cameraPoseTopic", cameraPoseTopic);
		
		m_camPose_sub = m_nh.subscribe(cameraPoseTopic, 1, &pgmvsPGMVisualServoing::toolPoseCallback, this);
	}

    stringstream str;
    str<<m_logs_path<<"logfile.txt";
    m_logfile.open(str.str().c_str());

	switch(camtyp)
	{
		case Omni:
		{
	    	_camera = new prOmni();

    		// Load the camera parameters from the XML file
    		prOmniXML fromFile(camxml.c_str());

			fromFile >> (*((prOmni *)_camera));
			m_logfile<<"prOmni: "<<_camera->getau()<<endl;

			break;
		}
		case Equirectangular:
		{
			_camera = new prEquirectangular();

			// Load the camera parameters from the XML file
			prEquirectangularXML fromFile(camxml.c_str());

			fromFile >> (*((prEquirectangular*)_camera));

			m_logfile<<"prEquirectangular: "<<_camera->getau()<<endl;

			break;
		}
		case Persp:
		{
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
}

pgmvsPGMVisualServoing::~pgmvsPGMVisualServoing()
{
  stopRobot();
}

void
pgmvsPGMVisualServoing::stopRobot()
{
	m_velocity.linear.x = 0;
	m_velocity.linear.y = 0;
	m_velocity.linear.z = 0;
	m_velocity.angular.x = 0;
	m_velocity.angular.y = 0;
  	m_velocity.angular.z = 0;
	m_velocity_pub.publish(m_velocity);
}

void
pgmvsPGMVisualServoing::initVisualServoTask()
{
	//load desired image
	std::string filename_read_image;
  stringstream ss_desired_image;
  
  ss_desired_image<<m_logs_path<<"/Id"<<".png";
  filename_read_image = vpIoTools::path(ss_desired_image.str().c_str());

	try 
	{
  		vpImageIo::read(this->m_desired_image, filename_read_image);
  		m_logfile<<"desired_image read from"<<filename_read_image<<endl;

		m_height = m_desired_image.getHeight();
		m_width = m_desired_image.getWidth();

		// 2. VS objects initialization, considering the pose control of a perspective camera from the feature set of photometric non-normalized Gaussian mixture 2D samples compared thanks to the SSD
		servo.setdof(m_dofs[0], m_dofs[1], m_dofs[2], m_dofs[3], m_dofs[4], m_dofs[5]);
		
		servo.setSensor(_camera);

    	// desired visual feature built from the image
		//prepare the desired image 
    	IP_des = new prRegularlySampledCPImage<unsigned char>(m_height, m_width); //the regularly sample planar image to be set from the acquired/loaded perspective image
    	IP_des->setInterpType(INTERPTYPE);
    	IP_des->buildFrom(m_desired_image, _camera); 

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

}

void pgmvsPGMVisualServoing::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
	if(vsInitialized)
	{
    	m_iter++;

#ifdef INDICATORS
    	double duration = vpTime::measureTimeMs();
#endif

   		m_current_image = visp_bridge::toVispImage(*image);

		if((m_current_image.getHeight() == m_height) && (m_current_image.getWidth() == m_width))
		{
			// Compute current visual feature
			IP_cur->buildFrom(m_current_image, _camera); 
			fSet_cur.updateMeasurement(*IP_cur, *GP, *GP_sample, poseJacobianCompute, updateSampler);  
			m_logfile << "cur updated" << endl;

			//Compute control vector
			m_normError = 0.5*servo.control(fSet_cur, m_v, robust);

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

			if(m_controlInBaseFrame)
				m_v6 = m_bVc * m_v6;

			m_velocity.linear.x = m_v6[0];
			m_velocity.linear.y = m_v6[1];
			m_velocity.linear.z = m_v6[2];
			m_velocity.angular.x = m_v6[3];
			m_velocity.angular.y = m_v6[4];
			m_velocity.angular.z = m_v6[5];
			m_velocity_pub.publish(m_velocity);
			
			if(m_pub_featuresImage)
			{
				fSet_cur.sampler.toImage(PGM_cur_f, pp, _camera);
				m_logfile << PGM_cur_f.getWidth() << " " << PGM_cur_u.getWidth() << " " << PGM_des_u.getWidth() << endl;
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
			
#ifdef INDICATORS
			m_times << vpTime::measureTimeMs() - duration << " ms" << std::endl;

			m_residuals << m_normError << std::endl; // |e|
#endif 
			m_logfile << " |e| = " << m_normError << std::endl;
			m_logfile << " |v| = " << sqrt(m_v6.sumSquare()) << std::endl;
			m_logfile << " v = " << m_v6.t() << std::endl;
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
	vpHomogeneousMatrix bMc = /*visp_bridge::*/toVispHomogeneousMatrix(tf);
	bMc[0][3] = bMc[1][3] = bMc[2][3] = 0;

	//m_logfile << bMc << std::endl;

  m_bVc.buildFrom(bMc);
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

