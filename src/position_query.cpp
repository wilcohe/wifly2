#include <ros/ros.h>
#include <nav_msgs/LoadMap.h>
#include <geometry_msgs/Pose.h>
#include <wifly2/intensity.h>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <random>
#include <math.h>

nav_msgs::OccupancyGrid base_map; 
nav_msgs::OccupancyGrid wifi_map; 

void map_callback(const nav_msgs::OccupancyGrid map){

  ROS_INFO("Received Map"); 
  ROS_INFO("Size: %ld", map.data.size()); 
  ROS_INFO("Width: %d", map.info.width); 
  ROS_INFO("Height: %d", map.info.height); 
  ROS_INFO("Res: %f", map.info.resolution); 

  base_map = map; 

}

bool occupiedCallback(wifly2::intensity::Request  &req,
                      wifly2::intensity::Response &res){

  int xcell = (int)std::round(req.pos.x/base_map.info.resolution); 
  int ycell = (int)std::round(req.pos.y/base_map.info.resolution); 

  ROS_INFO("Nearest cell is (%d, %d).", xcell, ycell); 
  if(xcell < 0 || ycell < 0){
    res.intensity = 1; 
    res.x = -1; 
    res.y = -1; 
    return true; 
  }

  res.occupied = base_map.data[ycell*base_map.info.width + xcell] > 0 : true ? false; 
  res.intensity = wifi_map.data[ycell*base_map.info.width + xcell];
  res.x = xcell; 
  res.y = ycell; 

  return true;  

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_query");
  ros::NodeHandle nh("~");
  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_callback); 
  ros::Subscriber wifi_sub = nh.subscribe("/wifi_map", 1, wifi_callback); 

  
  ros::Rate rate(1.0); 

  // ROS_INFO("%ld", sizeof(base_map.data)); 

  ros::ServiceServer occupied = nh.advertiseService("/intensity", occupiedCallback); 

  ROS_INFO("Intensity server initialized."); 

  ros::spin();

  

}
