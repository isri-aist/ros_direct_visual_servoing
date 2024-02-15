#include "pgmvsPGMVisualServoing.h"
#include <ros/ros.h>

#include <signal.h>

pgmvsPGMVisualServoing *prog;

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
    ros::init(argc, argv, "pgmvsPGMVisualServoing");
		prog = new pgmvsPGMVisualServoing();
		signal(SIGINT, mySigintHandler);
    ros::Rate loop_rate(100);
    ros::spin();
    return 0;
}

