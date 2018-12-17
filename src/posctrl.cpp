#include "ros/ros.h"
#include <sstream>
#include <tf/transform_listener.h>
#include "string.h"
#include "turtlesim/Pose.h"
#include "PositionController.cpp"

int main(int argc, char** argv)
{
   std::string nodeName;
   std::stringstream ss;
   ss << argv[1]; 
   nodeName = ss.str();

   ros::init(argc, argv, nodeName);
   ros::NodeHandle n;
   PositionController posctrl;
 
   posctrl.setNode(n);
   posctrl.setTopics(nodeName);
   posctrl.mainControl();
 
   ros::spin();
   return 0;
}




 	
