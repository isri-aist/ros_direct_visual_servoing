#include "pvsPhotometricVisualServoing.h"
#include <ros/ros.h>

#include <signal.h>

pvsPhotometricVisualServoing *prog;

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
	prog->stopRobot();
	delete prog;

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pvsPhotometricVisualServoing");
		prog = new pvsPhotometricVisualServoing();
		signal(SIGINT, mySigintHandler);
    ros::spin();
    return 0;
}

