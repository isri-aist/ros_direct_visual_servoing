#include "pgmvsPGMVisualServoing.h"
#include "ros/init.h"
#include <memory>
#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <ros_dvs_bridge/lambda_gConfig.h>
#include <signal.h>

pgmvsPGMVisualServoing *prog = nullptr;

// std::unique_ptr<pgmvsPGMVisualServoing> prog;

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.

  std::cout << "Shutting down..." << std::endl;
  
	// prog->stopRobot();
  if(prog != nullptr)
	  delete prog;

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
  exit(sig);
}

void callbackServer(ros_dvs_bridge::lambda_gConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f  %f  %f", 
            config.lambda_g,
            config.lambda,
            config.sceneDepth);
  prog->updateParameters(config.lambda_g, config.lambda, config.sceneDepth);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pgmvsPGMVisualServoing", ros::init_options::NoSigintHandler);
		prog = new pgmvsPGMVisualServoing();

    dynamic_reconfigure::Server<ros_dvs_bridge::lambda_gConfig> server;
    dynamic_reconfigure::Server<ros_dvs_bridge::lambda_gConfig>::CallbackType f;
    f = boost::bind(&callbackServer, _1, _2);
    server.setCallback(f);
		signal(SIGINT, mySigintHandler);

    // ros::Rate loop_rate(100);
    while(ros::ok()){
      ros::spinOnce();
    }
    return 0;
}

