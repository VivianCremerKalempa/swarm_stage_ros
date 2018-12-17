#include <ros/ros.h>
#include "turtlesim/Pose.h"
	
class MainController{
private:
	ros::NodeHandle n_;
  	
	ros::Publisher * pub_;

        turtlesim::Pose * pos_;

public:
	void setNode(ros::NodeHandle n){
		n_ = n;
	}

        //Controls all actions in this class
	void mainControl(int n){
           char name[50];
           int i;
           //A vector for storing publishers for robots
           pub_ = (ros::Publisher *)malloc(n * sizeof(ros::Publisher));
              
           //Computes the positions  
           setPositions(n);
                      
           //Each robot on each side of the square receives its coordinates to form one side of the cross
           for (i=0; i<(n/4); i++){
              snprintf(name, 50, "%s%i", "/robot_", i);
              pub_[i] = n_.advertise<turtlesim::Pose>(name, 1000);
              ros::Duration(10.0).sleep();
              pub_[i].publish(pos_[i]);
           }
           for (i=(n/4)-1; i>=0; i--){
              snprintf(name, 50, "%s%i", "/robot_", i+(n/4)*3);
              pub_[i+(n/4)*3] = n_.advertise<turtlesim::Pose>(name, 1000);
              ros::Duration(10.0).sleep();
              pub_[i+(n/4)*3].publish(pos_[i+(n/4)*3]);
           }
           for (i=(n/4)-1; i>=0; i--){
              snprintf(name, 50, "%s%i", "/robot_", i+(n/4));
              pub_[i+(n/4)] = n_.advertise<turtlesim::Pose>(name, 1000);
              ros::Duration(10.0).sleep();
              pub_[i+(n/4)].publish(pos_[i+(n/4)]);
           }
           for (i=(n/4)-1; i>=0; i--){
              snprintf(name, 50, "%s%i", "/robot_", i+(n/4)*2);
              pub_[i+(n/4)*2] = n_.advertise<turtlesim::Pose>(name, 1000);
              ros::Duration(10.0).sleep();
              pub_[i+(n/4)*2].publish(pos_[i+(n/4)*2]);
           }
        }

        void setPositions(int n){
           //A vector to store robot positions
           pos_ = (turtlesim::Pose *)malloc(n * sizeof(turtlesim::Pose));
           float x, y;
           int i;
           
           //Calculate each part of the cross separately, spacing 0.5 between robots
           y = -0.5;
           x = (n/4) * 0.5 + 0.25;
           for (i=0; i<(n/4); i++){
              pos_[i].x = x;
              pos_[i].y = y;
              y = y - 0.5; 
           }
           y = 0;
           x = (n/4) * 0.5;
           for (i=0; i<(n/4); i++){
              x = x + 0.5;
              pos_[i+(n/4)*3].x = x;
              pos_[i+(n/4)*3].y = y;
           }

           y = 0;
           x = 0;
           for (i=0; i<(n/4); i++){
              x = x + 0.5;
              pos_[i+(n/4)].x = x;
              pos_[i+(n/4)].y = y;
           }

           y = (n/4) * 0.5;
           x = (n/4) * 0.5 + 0.25;
           for (i=0; i<(n/4); i++){
              pos_[i+(n/4)*2].x = x;
              pos_[i+(n/4)*2].y = y;
              y = y - 0.5; 
           }
        }
};


