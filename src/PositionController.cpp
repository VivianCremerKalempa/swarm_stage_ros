#include "ros/ros.h"
#include <sstream>
#include <tf/transform_listener.h>
#include "string.h"
#include "FuzzyController.cpp"
#include "turtlesim/Pose.h"

//Robot control class
class PositionController{
private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	turtlesim::Pose feedback_;

	//Used to tf
	tf::TransformListener listener_;
	tf::StampedTransform transform_;

	FuzzyController fuzzy_;
	
	//Variables to control the robot
	ros::Publisher pub_;
	geometry_msgs::Twist cmd_vel_msg;

        //Variables for topics
        std::string topicPub_;
        std::string topicSub_;
        std::string topicTrans_;
   
public:

	//Controls all actions in this class
	void mainControl(){
	   pub_ = n_.advertise<geometry_msgs::Twist>(topicPub_,1000);
           sub_ = n_.subscribe(topicSub_, 15000,&PositionController::subCallback,this);
	}

	void setNode(ros::NodeHandle n){
           n_ = n;
        }

	void setTopics(std::string nodeName){
	   topicSub_ = '/' + nodeName;           
           topicPub_ = '/' + nodeName + "/cmd_vel"; 
	   topicTrans_ = nodeName + "/base_link";    
	}
  
        //Leads the robot to the position it receives
	void subCallback(const turtlesim::Pose::ConstPtr& msg){
	   geometry_msgs::Twist cmd_vel_msg;
	   float theta,  orientationError;
	   float linearDistance=6;

	   while ((abs(linearDistance) > 0.3)){
              robotTransformed();
              linearDistance = sqrt(pow((feedback_.x - msg->x),2) + pow((feedback_.y - msg->y),2));
              theta = atan2(msg->y - feedback_.y, msg->x - feedback_.x);
	      if (theta < 0){
	         //Works with angles always in the range 0 -> 2pi
		 theta = 3.14 + (3.14 - abs(theta)); 
              }

	      orientationError = theta - feedback_.theta;   
	      if (abs(orientationError) > 3.14){
	         orientationError = 6.28 - abs(orientationError);
		 if (feedback_.theta < theta)
		    orientationError *= -1;
              }
     
              float * a = fuzzy_.fuzzyPositionController(orientationError, linearDistance);
		      
	      cmd_vel_msg.linear.x = a[1];
              cmd_vel_msg.angular.z = a[0];
              pub_.publish(cmd_vel_msg);
	   }
	 
	}

	//Get the position of the robot on the map
	void robotTransformed(void){
           while(ros::ok()){
	      try{
	         listener_.lookupTransform("map", topicTrans_, ros::Time(0), transform_);
		 feedback_.x = transform_.getOrigin().x();
		 feedback_.y = transform_.getOrigin().y();
		 feedback_.theta = tf::getYaw(transform_.getRotation());
                 break;
              }
	      catch (tf::TransformException ex){
	         ROS_ERROR("%s",ex.what());
		 ros::Duration(1.0).sleep();
              }
	   }
	}	

};

