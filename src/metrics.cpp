#include <fstream>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>

std::string turtle_name;
ros::Publisher metrics_pub;

/// has the true value of the map to compare to
bool has_map;

void mapCallback(const nav_msgs::OccupancyGrid& msg){
  int explored = 0;
  int unoccupied = 0;
  for (size_t i=0; i< msg.data.size(); i++)
  {
    if (msg.data[i] != -1) explored++;
    if (msg.data[i] > 0.5) unoccupied++;
  } 

  std::stringstream ss;
  ss << explored;

  std_msgs::String message;
  message.data = ss.str();
  metrics_pub.publish(message);

  std::string filename = turtle_name + "metrics.txt";
  std::ofstream metrics (filename.c_str(), std::ios_base::app);
  if (metrics.is_open())
  {
    metrics << ss.str() + ", ";
    metrics.close();
  }
  else ROS_ERROR("could not open metrics file for publishing");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map progress metrics");

  if (argc < 2) {
    ROS_ERROR("need turtle name as argument"); 
    return -1;
  };

  turtle_name = argv[1];
  if (argv[2]) 
  {
    has_map = true;
  }

  std::ofstream ofs;
  std::string filename = turtle_name + "metrics.txt";
  ofs.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
  ofs.close();

  ros::NodeHandle n;
  metrics_pub = n.advertise<std_msgs::String>("metrics", 1000);

  ros::Subscriber sub = n.subscribe(
      "map_merger/global_map", 
      10, 
      &mapCallback);

  ros::spin();
  return 0;
}
