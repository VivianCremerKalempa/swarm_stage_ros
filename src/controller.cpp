#include <ros/ros.h>
//#include "turtlesim/Pose.h"
#include "MainController.cpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Controller");
  ros::NodeHandle n;
  MainController ctrl;
  int robotsNumber;
  
  robotsNumber = atoi(argv[1]); 
  
  ctrl.setNode(n);
  ctrl.mainControl(robotsNumber);
 
  ros::spin();
  return 0;
}
