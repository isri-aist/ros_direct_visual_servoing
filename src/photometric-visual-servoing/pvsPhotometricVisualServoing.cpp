#include "pvsPhotometricVisualServoing.h"

#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>


pvsPhotometricVisualServoing::pvsPhotometricVisualServoing()
    : m_it(m_nh), m_iter(-1), vsInitialized(false), m_height(0), m_width(0), m_sceneDepth(1.0), m_v(6), m_normError(1e12)
{
    string cameraTopic, robotTopic, cameraPoseTopic;
		double f, ku;

    m_nh.param("CameraTopic", cameraTopic, string(""));
		m_nh.param("robotTopic", robotTopic, string(""));
    m_nh.param("logs", m_logs_path, string(""));
    m_nh.param("lambda", m_lambda, double(1.0));
    m_nh.param("focal", f, double(1.0));
    m_nh.param("pixelSize", ku, double(1.0));
    m_nh.param("u0", m_u0, double(1.0));
    m_nh.param("v0", m_v0, double(1.0));
    m_nh.param("sceneDepth", m_sceneDepth, double(1.0));
    m_nh.param("controlInBaseFrame", m_controlInBaseFrame, false);
    m_nh.param("cameraPoseTopic", cameraPoseTopic, string(""));

    m_nh.getParam("cameraTopic", cameraTopic);
    m_nh.getParam("robotTopic", robotTopic);
    m_nh.getParam("logs", m_logs_path);
    m_nh.getParam("lambda", m_lambda);
    m_nh.getParam("focal", f);
    m_nh.getParam("pixelSize", ku);
    m_nh.getParam("u0", m_u0);
    m_nh.getParam("v0", m_v0);
		m_nh.getParam("sceneDepth", m_sceneDepth);
    m_nh.getParam("controlInBaseFrame", m_controlInBaseFrame);
		if(m_controlInBaseFrame)
		{
    	m_nh.getParam("cameraPoseTopic", cameraPoseTopic);
			
			m_camPose_sub = m_nh.subscribe(cameraPoseTopic, 1, &pvsPhotometricVisualServoing::toolPoseCallback, this);
		}

		if(ku > 0)
			m_px = m_py = f/ku;
		else
			m_px = m_py = f;

		m_cam.initPersProjWithoutDistortion(m_px, m_py, m_u0, m_v0);
    
    stringstream str;
    str<<m_logs_path<<"logfile.txt";
    m_logfile.open(str.str().c_str());

#ifdef INDICATORS
		str.str("");
		str<<m_logs_path<<"residuals.txt";
    m_residuals.open(str.str().c_str());

		str.str("");
		str<<m_logs_path<<"times.txt";
    m_times.open(str.str().c_str());
#endif

		initVisualServoTask();

    m_image_sub = m_it.subscribe(cameraTopic, 1, &pvsPhotometricVisualServoing::imageCallback, this);

		m_velocity_pub = m_nh.advertise<geometry_msgs::Twist>(robotTopic, 1);
}

pvsPhotometricVisualServoing::~pvsPhotometricVisualServoing()
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
pvsPhotometricVisualServoing::initVisualServoTask()
{
	//load desired image
	std::string filename_read_image;
  stringstream ss_desired_image;
  
  ss_desired_image<<m_logs_path<<"/Id"<<".png";
  filename_read_image = vpIoTools::path(ss_desired_image.str().c_str());

	try {
  	vpImageIo::read(this->m_desired_image, filename_read_image);
  	m_logfile<<"desired_image read from"<<filename_read_image<<endl;

		m_height = m_desired_image.getHeight();
		m_width = m_desired_image.getWidth();

    // desired visual feature built from the image
    sId.init(m_height, m_width, m_sceneDepth);
    sId.setCameraParameters(m_cam);
    sId.buildFrom(m_desired_image);

   	// current visual feature built from the image
    // (actually, this is the image...)
		m_current_image = m_desired_image;
    sI.init(m_height, m_width, m_sceneDepth);
    sI.setCameraParameters(m_cam);
    sI.buildFrom(m_current_image);

    // Create visual-servoing task
    // define the task
    // - we want an eye-in-hand control law
    // - robot or freeFlyingCamera is controlled in the camera frame
    servo.setServo(vpServo::EYEINHAND_CAMERA);
	  // add current and desired visual features
	  servo.addFeature(sI, sId);
    // set the gain
    servo.setLambda(m_lambda); 
    // compute interaction matrix at the current position
    servo.setInteractionMatrixType(vpServo::CURRENT);

		vsInitialized = true;
	} catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    vsInitialized = false;
  }

}

void pvsPhotometricVisualServoing::imageCallback(const sensor_msgs::ImageConstPtr &image)
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
      sI.buildFrom(m_current_image);

      m_v = servo.computeControlLaw(); // camera velocity send to the robot

			if(m_controlInBaseFrame)
				m_v = m_bVc * m_v;

			m_velocity.linear.x = m_v[0];
			m_velocity.linear.y = m_v[1];
			m_velocity.linear.z = m_v[2];
			m_velocity.angular.x = m_v[3];
			m_velocity.angular.y = m_v[4];
      m_velocity.angular.z = m_v[5];
    	m_velocity_pub.publish(m_velocity);

      m_normError = servo.getError().sumSquare();

#ifdef INDICATORS
			m_times << vpTime::measureTimeMs() - duration << " ms" << std::endl;

			m_residuals << m_normError << std::endl; // |e|
#endif 
			m_logfile << " |e| = " << m_normError << std::endl;
      m_logfile << " |v| = " << sqrt(m_v.sumSquare()) << std::endl;
			m_logfile << " v = " << m_v.t() << std::endl;
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

void pvsPhotometricVisualServoing::toolPoseCallback(const tf2_msgs::TFMessage& tf)
{
	vpHomogeneousMatrix bMc = /*visp_bridge::*/toVispHomogeneousMatrix(tf);
	bMc[0][3] = bMc[1][3] = bMc[2][3] = 0;

	//m_logfile << bMc << std::endl;

  m_bVc.buildFrom(bMc);
}

vpHomogeneousMatrix
pvsPhotometricVisualServoing::toVispHomogeneousMatrix(const tf2_msgs::TFMessage& trans)
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
