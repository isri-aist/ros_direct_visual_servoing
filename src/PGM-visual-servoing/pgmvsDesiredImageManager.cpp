#include "pgmvsDesiredImageManager.h"

#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>


pgmvsDesiredImageManager::pgmvsDesiredImageManager()
    : m_it(m_nh), m_iter(-1)
{

    string cameraTopic;
    m_nh.param("CameraTopic", cameraTopic, string(""));
    m_nh.param("logs", m_logs_path, string(""));

    m_nh.getParam("logs", m_logs_path);
    m_nh.getParam("cameraTopic", cameraTopic);
    
    stringstream str;
    str<<m_logs_path<<"logfile.txt";
    m_logfile.open(str.str().c_str());

    m_image_sub = m_it.subscribe(cameraTopic, 1, &pgmvsDesiredImageManager::imageCallback, this);
}

pgmvsDesiredImageManager::~pgmvsDesiredImageManager()
{

}

void pgmvsDesiredImageManager::imageCallback(const sensor_msgs::ImageConstPtr &image)
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
    }
}

