#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "Eigen/Dense"
using Eigen::MatrixXd;

std::string colors(int value, std::string random_colors, bool leader);
std::string sensors(int sonar, std::string laser);
bool xaxis(int y, int originY);
bool yaxis(int x, int originX);
bool circle(int x, int y, int originX, int originY, int radius);
bool origin(int x, int y, int originX, int originY);
int draw(int px, int py, int originX, int originY, int circleRadius);
float image(char path[300], std::string formation, int robots, int side, std::string scenario);
void robot(int colors_number, std::string random_colors, bool leader, int sonar, std::string laser, int number, float x, float y,
           std::string position_controller, std::string publish_tf, std::string reference,FILE *file, FILE *pcfile, FILE *tffile);
std::string robot_types(std::string reference, bool leader);

//-------------------------------------------------------------------- 
int main(int argc, char **argv)
//--------------------------------------------------------------------
{
   ros::init(argc, argv, "swarm_stage");

   ros::NodeHandle n;

   std::string s, formation, color, completed, sensor, leaderSensor, leader, leaderLaser, othersLaser, random_colors, scenario, position_controller, run;
   std::string publish_tf, reference;
   int robots,i,half,t, number, side, count, leaderSonar, othersSonar, newSide;
   float x, x1, y, y1, formationHalf, size;
   bool leaderFlag;

   FILE *file;
   FILE *pcfile;
   FILE *tffile;
   int result;

   // obtain current folder
   system("rospack find swarm_stage_ros > current.fld");
   file = fopen("current.fld", "r");
   if (file == NULL)
     ROS_ERROR("Error in determine the current folder\n");
   char path[300], filepath[300];
   fgets(path, 300, file);
   path[strlen(path)-1]='/';
   ROS_INFO("%s",path);
   strcpy(filepath,path);
   strcat(filepath,"launch/custom.world");

   file = fopen(filepath, "w+");
   if (file == NULL)
     ROS_ERROR("Error in file creation\n");

   char pcpath[300]; 
   strcpy(pcpath,path);
   strcat(pcpath,"launch/controller.launch");

   pcfile = fopen(pcpath, "w+");
   if (pcfile == NULL)
     ROS_ERROR("Error in pcfile creation\n");
   fprintf(pcfile,"<?xml version=\"1.0\"?>");
   fprintf(pcfile,"\n<launch>");
   fprintf(pcfile,"\n  <!-- ROSCORE -->");
   fprintf(pcfile,"\n	<master auto=\"start\"/>");
   fprintf(pcfile,"\n  <!--  Stage Simulator -->");
   fprintf(pcfile,"\n  	<node pkg=\"stage_ros\" type=\"stageros\" name=\"stageros\" args=\"$(find swarm_stage_ros)/launch/custom.world\" respawn=\"true\" output=\"log\"/> "); 
          
   if (n.getParam("/swarm_stage/swarm/publish_tf", publish_tf)){
      ROS_INFO("Got param '/swarm_stage/swarm/publish_tf': %s", publish_tf.c_str()); 
   } 
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/publish_tf'");
      return 0;
   }

   if (strcmp(publish_tf.c_str(), "yes")==0){
      char tfpath[300];   
      strcpy(tfpath,path);
      strcat(tfpath,"launch/tf.launch");

      tffile = fopen(tfpath, "w+");
      if (tffile == NULL)
        ROS_ERROR("Error in tffile creation\n");
      fprintf(tffile,"<?xml version=\"1.0\"?>");
      fprintf(tffile,"\n<launch>");
      fprintf(tffile,"\n  <param name=\"pub_map_odom_transform\" value=\"true\"/> ");
      fprintf(tffile,"\n  <param name=\"map_frame\" value=\"map\"/> ");
      fprintf(tffile,"\n  <param name=\"base_frame\" value=\"base_frame\"/> ");
      fprintf(tffile,"\n  <param name=\"odom_frame\" value=\"odom\"/>"); 
   }

   if (n.getParam("/swarm_stage/swarm/formation", formation))
      ROS_INFO("Got param '/swarm_stage/swarm/formation': %s", formation.c_str());  
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/formation'");
      return 0;
   }
   if (n.getParam("/swarm_stage/swarm/robots", robots))
      ROS_INFO("Got param '/swarm_stage/swarm/robots': %i", robots); 
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/robots'");
      return 0;
   }
   if (n.getParam("/swarm_stage/swarm/side", side))
      ROS_INFO("Got param '/swarm_stage/swarm/side': %i", side); 
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/side'");
      return 0;
   }
   fprintf(file,"include \"%slaunch/include/map.inc\"\n",path);
   fprintf(file,"include \"%slaunch/include/robots.inc\"\n",path);
   fprintf(file,"include \"%slaunch/include/sensors.inc\"\n",path);

   if (n.getParam("/swarm_stage/swarm/scenario", scenario))
      ROS_INFO("Got param '/swarm_stage/swarm/scenario': %s", scenario.c_str()); 
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/scenario'");
      return 0;
   }
   size = image(path, formation, robots, side, scenario);
   fprintf(file,"floorplan (name \"cave\" size [%.3f %.3f 0.800] pose [0 0 0 0] bitmap \"%slaunch/map/image.png\")",size, size, path);
   fprintf(file,"\n");

   if (n.getParam("/swarm_stage/swarm/other_robots_sonar_sensor", othersSonar))
      ROS_INFO("Got param '/swarm_stage/swarm/other_robots_sonar_sensor': %i", othersSonar);  
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/other_robots_sonar_sensor'");
      return 0;
   }
   if (n.getParam("/swarm_stage/swarm/other_robots_laser_sensor", othersLaser))
      ROS_INFO("Got param '/swarm_stage/swarm/other_robots_laser_sensor': %s", othersLaser.c_str());  
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/other_robots_laser_sensor'");
      return 0;
   }
   if (n.getParam("/swarm_stage/swarm/random_colors", random_colors))
      ROS_INFO("Got param '/swarm_stage/swarm/random_colors': %s", random_colors.c_str()); 
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/random_colors'");
      return 0;
   }
   if (n.getParam("/swarm_stage/swarm/position_controller", position_controller))
      ROS_INFO("Got param '/swarm_stage/swarm/position_controller': %s", position_controller.c_str());  
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/position_controller'");
      return 0;
   }
   if (n.getParam("/swarm_stage/swarm/reference", reference))
      ROS_INFO("Got param '/swarm_stage/swarm/reference': %s", reference.c_str());  
   else{
      ROS_ERROR("Failed to get param '/swarm_stage/swarm/reference'");
      return 0;
   }
   if (othersSonar != 0 && (strcmp(othersLaser.c_str(), "yes")==0)){
      ROS_ERROR("Choose only one sensor type for the others robots.");
      return 0;
   }
   if (strcmp(formation.c_str(), "no")==0){
      count = -1;
      x=-0.5;y=0;
      for(i=0;i<robots;i++){
            count ++;
            if ((i%15)==0){
               y = y + 0.5;
               x = -0.5;
            }
            else x = x - 0.5;
            robot(i%10,random_colors,false,othersSonar,othersLaser,i,x,y,position_controller,publish_tf,reference,file,pcfile,tffile);
         }
      
      }
   else{
 
         if (n.getParam("/swarm_stage/swarm/completed", completed))
            ROS_INFO("Got param '/swarm_stage/swarm/completed': %s", completed.c_str()); 
         else{
            ROS_ERROR("Failed to get param '/swarm_stage/swarm/completed'");
            return 0;
         }
         if (n.getParam("/swarm_stage/swarm/leader", leader))
            ROS_INFO("Got param '/swarm_stage/swarm/leader': %s", leader.c_str()); 
         else{
            ROS_ERROR("Failed to get param '/swarm_stage/swarm/leader'");
            return 0;
         }
         if (n.getParam("/swarm_stage/swarm/leader_sonar_sensor", leaderSonar))
            ROS_INFO("Got param '/swarm_stage/swarm/leader_sonar_sensor': %i", leaderSonar);  
         else{
            ROS_ERROR("Failed to get param '/swarm_stage/swarm/leader_sonar_sensor'");
            return 0;
         }
         if (n.getParam("/swarm_stage/swarm/leader_laser_sensor", leaderLaser))
            ROS_INFO("Got param '/swarm_stage/swarm/leader_laser_sensor': %s", leaderLaser.c_str());  
         else{
            ROS_ERROR("Failed to get param '/swarm_stage/swarm/leader_laser_sensor'");
            return 0;
         }
         if (leaderSonar != 0 && (strcmp(leaderLaser.c_str(), "yes")==0)){
            ROS_ERROR("Choose only one sensor type for the leader.");
            return 0;
         }

         if (strcmp(formation.c_str(), "wedge")==0){
            count = 0;
            if (strcmp(leader.c_str(), "no")==0){
               robot(0,random_colors,false,othersSonar,othersLaser, count,0,0,position_controller,publish_tf,reference,file,pcfile,tffile);
            }
            else{
               robot(0,random_colors,true,leaderSonar,leaderLaser,count,0,0,position_controller,publish_tf,reference,file,pcfile,tffile);
            }
            x=0;y=0;
            for(i=1;i<side;i++){
               x = x - 0.3;
               y = y + 0.3;
               color = colors(i%10, random_colors, false);
               count++;
               robot(i%10,random_colors,false,othersSonar,othersLaser,count,x,y,position_controller,publish_tf,reference,file,pcfile,tffile);
               count++; 
               robot(i%10,random_colors,false,othersSonar,othersLaser,count,x,y*-1,position_controller,publish_tf,reference,file,pcfile,tffile);
            }
            if (strcmp(completed.c_str(), "yes")==0){
               //Complete wedge formation
               newSide = side - 2;
               x1=0;
               while(newSide > 0){
                  x1 = x1 - 0.6;
                  x = x1;
                  y = 0;
                  count++;
                  robot(count%10,random_colors,false,othersSonar,othersLaser,count,x,y,position_controller,publish_tf,reference,file,pcfile, tffile);
               
                  for(i=1;i<newSide;i++){
                     x = x - 0.3;
                     y = y + 0.3;
                     count ++;
                     robot(count%10,random_colors,false,othersSonar,othersLaser,count,x,y,position_controller,publish_tf,reference,file,pcfile, tffile);
                     count ++; 
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y*-1, position_controller,publish_tf,reference, file, pcfile, tffile);
                  }
                  newSide = newSide - 2;

               }

            }
         }
         else{
            leaderFlag = false;
            if (strcmp(formation.c_str(), "square")==0){
               if (strcmp(completed.c_str(), "no")==0){
                  x=0;y=0.5;
                  if (strcmp(leader.c_str(), "yes")==0)
                     count = 0;
                  else
                     count = -1;

                  if (side == 1){
                     x = x - 0.5;
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     if (strcmp(leader.c_str(), "yes")==0){
                        count ++;
                        robot(0, random_colors, true, leaderSonar, leaderLaser, 0, x+0.5, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     }
                  }

                  for(i=1;i<=(side-1);i++){
                     x = x - 0.5;
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                  }
                  x = x - 0.5;
                  for(i=1;i<=(side-1);i++){
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     y = y +  0.5; 
                  }        
                  for(i=1;i<=(side-1);i++){
                     count ++; 
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     x = x + 0.5;
                  }
                  formationHalf = side /  2.0;

                  for(i=1;i<=(side-1);i++){
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     
                     if (strcmp(leader.c_str(), "yes")==0){
                        if (i>formationHalf and leaderFlag == false){
                           leaderFlag = true;
                           robot(0, random_colors, true, leaderSonar, leaderLaser, 0, x+0.5, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                        }
                        else{
                           if (i==formationHalf and leaderFlag == false){
                              leaderFlag = true;
                              robot(0, random_colors, true, leaderSonar, leaderLaser, 0, x+0.5, y-0.25, position_controller,publish_tf,reference, file, pcfile, tffile);
                           }
                        }
                     }
                     y = y -  0.5; 
                  }   
               }
               else{
                  x=0;y=0.5;
                  if (strcmp(leader.c_str(), "yes")==0)
                     count = 0;
                  else
                     count = -1;
                  formationHalf = side /  2.0;

                  for(t=1;t<=side;t++){  
                     if (strcmp(leader.c_str(), "yes")==0){
                        if (t>formationHalf and leaderFlag == false){
                           leaderFlag = true;
                           robot(0, random_colors, true, leaderSonar, leaderLaser, 0, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                        }
                        else{
                           if (t==formationHalf and leaderFlag == false){
                              leaderFlag = true;
                              robot(0, random_colors, true, leaderSonar, leaderLaser, 0, x, y+0.25, position_controller,publish_tf,reference, file, pcfile, tffile);
                           }
                        }
                     }
                     
                     for(i=0;i<side;i++){
                        x = x - 0.5;
                        count ++;
                        robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     }
                     x = 0;
                     y = y + 0.5;
                  }
               }
            }
            else{
               if (strcmp(formation.c_str(), "diamond")==0){
                  if (strcmp(leader.c_str(), "no")==0){
                     robot(0, random_colors, false, othersSonar, othersLaser, 0, 0, 0, position_controller,publish_tf,reference, file, pcfile, tffile);
                  }
                  else{
                     robot(0, random_colors, true, leaderSonar, leaderLaser, 0, 0, 0, position_controller,publish_tf,reference, file, pcfile, tffile);
                  }
                  x=0;y=0;
                  count = 0;
                  for(i=1;i<side;i++){
                     x = x - 0.3;
                     y = y + 0.3;
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     count ++; 
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y*-1, position_controller,publish_tf,reference, file, pcfile, tffile);
                  }
                  for(i=1;i<(side-1);i++){
                     x = x - 0.3;
                     y = y - 0.3;
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y*-1, position_controller,publish_tf,reference, file, pcfile, tffile);
                  }
                  if (side>1){
                     count ++;
                     robot(count%10, random_colors, false, othersSonar, othersLaser, count, x-0.3, y-0.3, position_controller,publish_tf,reference, file, pcfile, tffile);
                  }
                  if (strcmp(completed.c_str(), "yes")==0){
                     newSide = side - 2;
                     x1=0;
                     while(newSide > 0){
                        x1 = x1 - 0.6;
                        x = x1;
                        y = 0;
                        count++;
                        robot(count%10, random_colors, false, othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                     
                        for(i=1;i<newSide;i++){
                           x = x - 0.3;
                           y = y + 0.3;
                           count ++;
                           robot(count%10,random_colors,false,othersSonar, othersLaser, count, x, y, position_controller,publish_tf,reference, file, pcfile, tffile);
                           count ++; 
                           robot(count%10,random_colors,false,othersSonar,othersLaser,count,x,y*-1,position_controller,publish_tf,reference, file, pcfile, tffile);
                        }
                        for(i=1;i<(newSide-1);i++){
                           x = x - 0.3;
                           y = y - 0.3;
                           count ++;
                           robot(count%10,random_colors,false,othersSonar,othersLaser,count,x,y,position_controller,publish_tf,reference, file, pcfile, tffile);
                           count ++;
                           robot(count%10,random_colors,false,othersSonar,othersLaser,count,x,y*-1, position_controller,publish_tf,reference, file, pcfile, tffile);
                        }
                        count ++;
                        robot(count%10,random_colors,false,othersSonar,othersLaser,count,x-0.3,y-0.3,position_controller,publish_tf,reference, file, pcfile, tffile);

                        newSide = newSide - 2;

                     }
                  }
               }
            }
         } 
      } 

  fclose(file);

  if (strcmp(position_controller.c_str(), "yes")==0)
     fprintf(pcfile,"\n        <node name=\"controller\" pkg=\"swarm_stage_ros\" type=\"controller\" args=\"%i\" output=\"screen\"/>",count+1);

  if (strcmp(publish_tf.c_str(), "yes")==0){
     fprintf(pcfile,"\n        <include file=\"$(find swarm_stage_ros)/launch/tf.launch\"/>");
     fprintf(tffile,"\n</launch>");
     fclose(tffile);
  } 
  fprintf(pcfile,"\n</launch>");
  fclose(pcfile);

  // execute stage_ros if parameter run is yes
  if (n.getParam("/swarm_stage/run", run) && (strcmp(run.c_str(), "yes")==0)){
  	system(" killall stageros >/dev/null 2>&1");  //close all stage_ros
  	if (ros::ok())
      	   system("roslaunch swarm_stage_ros controller.launch");
  	else
   	   ROS_ERROR("ROSCORE not running");
  }
  return 0;
}

//--------------------------------------------------------------------
std::string colors(int value, std::string random_colors, bool leader){
//--------------------------------------------------------------------
   std::string color;
   
   if (strcmp(random_colors.c_str(), "yes")==0){ 
      if (leader)
         color = "red";
      else{
         switch(value)
         {
            case 0: color="black"; break;
            case 1: color="blue"; break;
            case 2: color="grey"; break;
            case 3: color="pink"; break;
            case 4: color="brown"; break;
            case 5: color="violet"; break;
            case 6: color="yellow"; break;
            case 7: color="orange"; break;
            case 8: color="purple"; break;
            case 9: color="green"; break;
            case 10: color="magenta"; break;        
         }
      }
   }
   else{
      if (leader)
         color = "red";
      else
         color = "blue";
   }
   

   return color;
}
//--------------------------------------------------------------------
std::string sensors(int sonar, std::string laser){
//--------------------------------------------------------------------
   std::string sensor;
   if (sonar == 1)  
      sensor = " sonar_sensor(pose [ 0.1 -0.15 0 0 ]) ";
   else {
      if (sonar == 3)
         sensor = " sonar_array(pose [ 0.1 -0.15 0 0 ]) ";
      else
         if (strcmp(laser.c_str(), "yes")==0)
            sensor = " laser(pose [ 0.1 -0.15 0 0 ]) ";
   }
   return sensor;
}
//--------------------------------------------------------------------
bool xaxis(int y, int originY)
//--------------------------------------------------------------------
{
    return y == originY;
}
//-------------------------------------------------------------------- 
bool yaxis(int x, int originX)
//--------------------------------------------------------------------
{
    return x == originX;
}
//--------------------------------------------------------------------
bool circle(int x, int y, int originX, int originY, int radius)
//--------------------------------------------------------------------
{
    return sqrt((x-originX)*(x-originX)+(y-originY)*(y-originY)) <= radius;
}
//-------------------------------------------------------------------- 
bool origin(int x, int y, int originX, int originY)
//--------------------------------------------------------------------
{
    return xaxis(y, originY) && yaxis(x, originX);
}
//-------------------------------------------------------------------- 
int draw(int px, int py, int originX, int originY, int circleRadius)
//--------------------------------------------------------------------
{
    if (origin(px, py, originX, originY) || circle(px, py, originX, originY, circleRadius))
        return 0;
     
    return 1;
}
//----------------------------------------------------------------------------------------------
float image(char path[300], std::string formation, int robots, int side, std::string scenario){
//----------------------------------------------------------------------------------------------
   float biggerX, biggerY;
   FILE *imageFile;
   char text[300];
   int x,y,pixel,height,width,obstacles,circleRadius,originX=0,originY,lastOriginX,rooms,lastY;
 
   if (strcmp(formation.c_str(), "no")==0){
      if(robots > 15)
         biggerX = -7.5;
      else
         biggerX = robots*-0.5;
      biggerY = (robots/15)*0.5;
      if (robots%15>0)
         biggerY = biggerY + 0.5;
   }
   if (strcmp(formation.c_str(), "square")==0){
      biggerX = side*-0.5;
      biggerY = side*0.5;
   }
   if (strcmp(formation.c_str(), "wedge")==0){
      biggerX = (side-1)*-0.3;
      biggerY = (side-1)*0.3;
   }
   if (strcmp(formation.c_str(), "diamond")==0){
      biggerX = (2*(side-1))*-0.3;
      biggerY = (side-1)*0.3;
   }
    
   if (abs(biggerX) < abs(biggerY))
      biggerX = biggerY;
   else
      biggerY = biggerX;
   
   if (biggerX<0) biggerX = biggerX * -1;
   if (biggerY<0) biggerY = biggerY * -1;
   width  = 2*biggerX*31.25;
   height = 2*biggerY*31.25; 
   
   MatrixXd m(height,width);

   for(int i=0;i<width;i++)
	for(int y=0;y<height;y++)
		if ((i==0)||(i==(width-1))||(y==0)||(y==(height-1))) m(y,i)=0;
		else	m(y,i) = 1;
   
   if (strcmp(scenario.c_str(), "forest")==0){
      obstacles = width/200 + 1;
      circleRadius = width/50;
      ROS_INFO("Obstacles: %i, CircleRadius: %i",obstacles, circleRadius);
      //draw obstacles
      for(int i=1; i<=obstacles; i++){
         originY = (height/(obstacles+1))*i;
         
         originX = (width/2)+(rand()%((width/2)-(width/20)));

         if (originX >=(width/2) && originX <= ((width/2) + (width/20)))
            originX = originX + (width/20);

         for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++)
               if (m(x,y) == 1) m(x,y)=draw(x, y, originX, originY, circleRadius);
         }
      }
   
      //square or no formation
      if (strcmp(formation.c_str(), "square")==0||strcmp(formation.c_str(), "no")==0){
         for(int i=1; i<=2; i++){
            originX = width/4;
            originY = (height/4)*(i+1.5);
            for (y = 0; y < height; y++){          
               for (x = 0; x < width; x++)
                  if (m(x,y) == 1) m(x,y)=draw(x, y, originX, originY, circleRadius);
            }
         }
      }
      else{
         //wedge or diamond
         for(int i=1; i<=2; i++){
            originX = (width/8)*3;
            if (i==1)
               originY = (height/8);
            else 
               originY = (height/8)*7;
            for (y = 0; y < height; y++){          
               for (x = 0; x < width; x++)
                  if (m(x,y) == 1) m(x,y)=draw(x, y, originX, originY, circleRadius);
            }
         }
      }
   }
   //end draw obstacles
   if (strcmp(scenario.c_str(), "hospital")==0){
      rooms = width/(width*0.25);
   
      y = 0;
      x = (width/2)+(width/10);
      //draw the rooms of the right side
      for(int i=1; i<=rooms; i++){
         lastY = y;
         y = y + height/rooms;
         for (int y1=lastY; y1<(y-((y-lastY)/4)); y1++)
            m(x,y1)= 0;
         if (i!=rooms)
            for (int x1 = x; x1<width; x1++)
               m(x1,y) = 0;
      }	
      //draw the rooms of the left side
      if (strcmp(formation.c_str(), "diamond")==0){
         y = 0;
         x = (width/2)-(width/10);;
         for(int i=1; i<=2; i++){
            if (i == 1){
               lastY = y;
               y = y + height/rooms;
            }
            else{
               lastY = (y * 3);
               y = (y * 4) -1;
            }
            for (int y1=lastY; y1<(y-((y-lastY)/4)); y1++)
               m(x,y1)= 0;
            if (i == 2) 
               y = lastY;
            for (int x1 = 0; x1<x; x1++)
               m(x1,y) = 0;
         } 
      }
      if (strcmp(formation.c_str(), "square")==0||strcmp(formation.c_str(), "no")==0){
         y = height/rooms;
         x = (width/2)-(width/10);;
         lastY = (y * 3);
         y = (y * 4);
         for (int y1=lastY; y1<(y-((y-lastY)/4)); y1++)
            m(x,y1)= 0;
         y = lastY;
         for (int x1 = 0; x1<x; x1++)
            m(x1,y) = 0; 
      }
      
   }
   	
   char imagepath[300]; 
   strcpy(imagepath,path);
   strcat(imagepath,"launch/map/image.ppm");

   imageFile = fopen(imagepath, "wb");

   if(imageFile==NULL){
      perror("ERROR: Cannot open output file");
      exit(EXIT_FAILURE);
   }

   fprintf(imageFile,"P3\n");                  // filetype
   fprintf(imageFile,"%d %d\n",width,height);  // dimensions
   fprintf(imageFile,"255\n");                 // Max pixel

   for(int i=0;i<width;i++)
        for(int y=0;y<height;y++)
               if (m(y,i)==1)
			fprintf(imageFile,"255\n255\n255\n");
                else    
			fprintf(imageFile,"0\n0\n0\n");


   fclose(imageFile);

   snprintf(text, 300, "%s%s%s%s%s", "convert ",imagepath," ",path,"launch/map/image.png");
   system(text);
   
   float result; 

   if (biggerX < 3 && strcmp(formation.c_str(), "square")!=0 && strcmp(formation.c_str(), "no")!=0){
      result = 8;
   }
   else{
      if (biggerX < 4 && (strcmp(formation.c_str(), "square")==0 || strcmp(formation.c_str(), "no")==0))
         result = 8;
      else
         result = (biggerX/6)*16;
   }

   return result;
}

//------------------------------------------------------------------------------------------------------------------------------------
void robot(int colors_number, std::string random_colors, bool leader, int sonar, std::string laser, int number, float x, float y,
           std::string position_controller, std::string publish_tf, std::string reference,FILE *file, FILE *pcfile, FILE *tffile){
//------------------------------------------------------------------------------------------------------------------------------------
   std::string color = colors(colors_number, random_colors, leader);
   std::string sensor = sensors(sonar, laser);
   std::string type = robot_types(reference,leader);
   
   fprintf(file,"%s (name \"r%i\" pose [%.2f %.2f 0 0 ] %s color \"%s\")\n",type.c_str(),number,x,y,sensor.c_str(),color.c_str());
   
   if (strcmp(position_controller.c_str(), "yes")==0)
      fprintf(pcfile,"\n        <node name=\"robot%i\" pkg=\"swarm_stage_ros\" type=\"posctrl\" args=\"robot_%i\" output=\"screen\"/>",number, number);
   if (strcmp(publish_tf.c_str(), "yes")==0)
      fprintf(tffile,"\n  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"robot_%i\" args=\"%.2f %.2f 0 0 0 0 /map robot_%i/odom 100\"/>",number, x, y, number);
   
}

//-----------------------------------------------------------
std::string robot_types(std::string reference, bool leader){
//-----------------------------------------------------------
   std::string type;
   if (strcmp(reference.c_str(), "relative")==0){
      if (leader == true)
         type = "turtlebot";
      else
         type = "roomba";
   }
   else{
      if (leader == true)
         type = "turtlebot_gps";
      else
         type = "roomba_gps";
   }
   return type;
}
