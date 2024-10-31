#include "mppvsDesiredImageManager.h"

#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>

#include <rosbag/bag.h> //for evs 
#include <geometry_msgs/PoseStamped.h>
#include <filesystem>


mppvsDesiredImageManager::mppvsDesiredImageManager()
    : m_it(m_nh), m_iter(-1), m_rosbagForEVS(true) //evs
{

    string cameraTopic, cameraPoseTopic, cam2tool; //evs
    m_nh.param("CameraTopic", cameraTopic, string(""));
    m_nh.param("logs", m_logs_path, string(""));
    m_nh.param("rosbagForEVS", m_rosbagForEVS, false); //evs
    m_nh.param("cameraPoseTopic", cameraPoseTopic, string(""));
    m_nh.param("desiredPoseTopicForEVS", m_desiredPoseTopicForEVS, string(""));
    m_nh.param("camera2tool", cam2tool, string(""));

    m_nh.getParam("logs", m_logs_path);
    m_nh.getParam("cameraTopic", cameraTopic);
    m_nh.getParam("rosbagForEVS", m_rosbagForEVS); //evs
    m_nh.getParam("desiredPoseTopicForEVS", m_desiredPoseTopicForEVS);
	m_nh.getParam("camera2tool", cam2tool);
    
    stringstream str;
    str<<m_logs_path<<"logfile.txt";
    m_logfile.open(str.str().c_str());

    	if(cam2tool.compare("") != 0) //evs
	    {
            vpPoseVector r_ct;
            if(!vpPoseVector::loadYAML(cam2tool, r_ct))
                m_logfile << "no camera to tool transform loaded" << std::endl;

            m_tMc.buildFrom(r_ct);
	    }



    m_image_sub = m_it.subscribe(cameraTopic, 1, &mppvsDesiredImageManager::imageCallback, this);

        if(m_rosbagForEVS) //evs for evo
    {
        m_nh.getParam("cameraPoseTopic", cameraPoseTopic);
        
        m_camPose_sub = m_nh.subscribe(cameraPoseTopic, 1, &mppvsDesiredImageManager::toolPoseCallback, this);
    }
}

mppvsDesiredImageManager::~mppvsDesiredImageManager()
{

}

void mppvsDesiredImageManager::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    m_iter++;

    m_desired_image = visp_bridge::toVispImage(*image);
    
    if (m_iter == 20)
    {
        std::string filename_write_image;
        stringstream ss_desired_image;
        
        ss_desired_image<<m_logs_path<<"/Id"<<".png";
        filename_write_image = vpIoTools::path(ss_desired_image.str().c_str());

        vpImageIo::write(this->m_desired_image, filename_write_image);
        m_logfile<<"desired_image written to"<<filename_write_image<<endl;
    

            if(m_rosbagForEVS) //evs
        {
            rosbag::Bag vsbag;
            geometry_msgs::PoseStamped desiredRobotPoseStamped;
            vpHomogeneousMatrix bMc;
		    mutex_bMt.lock();
            bMc = m_bMt * m_tMc;
            mutex_bMt.unlock();
            vpTranslationVector t = bMc.getTranslationVector();
            vpQuaternionVector q = vpQuaternionVector(bMc.getRotationMatrix());
		    

            desiredRobotPoseStamped.header.stamp = ros::Time::now();
            desiredRobotPoseStamped.pose.position.x = t[0];
            desiredRobotPoseStamped.pose.position.y = t[1];
            desiredRobotPoseStamped.pose.position.z = t[2];
            desiredRobotPoseStamped.pose.orientation.x = q.x();
            desiredRobotPoseStamped.pose.orientation.y = q.y();
            desiredRobotPoseStamped.pose.orientation.z = q.z();
            desiredRobotPoseStamped.pose.orientation.w = q.w();
            
            stringstream ss_bag;       
            ss_bag<<m_logs_path<<"/desiredAndCurrentPoses.bag";
            vsbag.open(ss_bag.str().c_str(), rosbag::bagmode::Write);
            vsbag.write(m_desiredPoseTopicForEVS, ros::Time::now(), desiredRobotPoseStamped);
            vsbag.close();
        }
    }
}

void mppvsDesiredImageManager::toolPoseCallback(const tf2_msgs::TFMessage& tf) //evs
{
	if(tf.transforms[0].child_frame_id.compare("tool0_controller") == 0)
    {
        mutex_bMt.lock();
		m_bMt = /*visp_bridge::*/toVispHomogeneousMatrix(tf);
        mutex_bMt.unlock();	
	}
}

vpHomogeneousMatrix
mppvsDesiredImageManager::toVispHomogeneousMatrix(const tf2_msgs::TFMessage& trans)
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

