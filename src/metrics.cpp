#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
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

  ros::NodeHandle n;
  metrics_pub = n.advertise<std_msgs::String>("metrics", 1000);
  
  ros::Subscriber sub = n.subscribe(
      "map_merger/global_map", 
      10, 
      &mapCallback);
  
  ros::spin();
  return 0;

}
/**
 *
 *
 *
 *     **
 *      * save info on the map
 *      *
 *     void map_info()
 *     {       
 *       fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
 *       fs_csv << "#time,exploration_travel_path_global,exploration_travel_path_average,global_map_progress,local_map_progress,number_of_completed_auctions, number_of_uncompleted_auctions, frontier_selection_strategy, costmap_size, unreachable_frontiers" << std::endl;
 *       fs_csv.close();
 * 
 *       while(ros::ok() && exploration_finished != true)
 *       {
 *         costmap_mutex.lock();
 * 
 *         ros::Duration time = ros::Time::now() - time_start;
 *         double exploration_time = time.toSec();  
 * 
 *         map_progress.global_freespace = global_costmap_size();
 *         map_progress.local_freespace = local_costmap_size();
 *         map_progress.time = exploration_time;               
 *         map_progress_during_exploration.push_back(map_progress);
 * 
 * 
 *         double exploration_travel_path_global = (double)exploration->exploration_travel_path_global * 0.02;
 *         double exploration_travel_path_average = 0;
 *         if(counter != 0)
 *         {
 *           exploration_travel_path_average = (exploration->exploration_travel_path_global) / counter;
 *         }
 * 
 *         //                ROS_INFO("global map size: %f   at time: %f", map_progress.global_freespace, map_progress.time);
 *         //                ROS_INFO("local map size : %f   at time: %f", map_progress.local_freespace, map_progress.time);
 *         //                ROS_INFO("travel path glo: %d   at time: %f", exploration_travel_path_global, map_progress.time);
 *         //                ROS_INFO("travel path ave: %d   at time: %f", exploration_travel_path_average, map_progress.time);
 * 
 *         fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
 * 
 *         fs_csv << map_progress.time << "," << exploration_travel_path_global << "," << exploration_travel_path_average << "," << map_progress.global_freespace << "," << map_progress.local_freespace << "," << global_iterattions <<  "," << exploration->number_of_completed_auctions << "," << exploration->number_of_uncompleted_auctions << "," << frontier_selection << "," <<  costmap_width << "," << exploration->unreachable_frontiers.size() <<  std::endl;
 *         //                fs_csv << "travel_path_global   = " << exploration_travel_path_global << std::endl;
 *         //                fs_csv << "travel_path_average  = " << exploration_travel_path_average << std::endl;             
 *         //                fs_csv << "map_progress_global  = " << map_progress.global_freespace << std::endl;
 *         //                fs_csv << "map_progress_average = " << map_progress.local_freespace << std::endl;
 *         //                fs_csv << "time                 = " << map_progress.time << std::endl;
 *         //                fs_csv << "                       " << std::endl;
 * 
 *         fs_csv.close();
 * 
 *         costmap_mutex.unlock();    
 * 
 *         // call map_merger to log data
 *         map_merger::LogMaps log;
 *         log.request.log = 12;    /// request local and global map progress
 *         ROS_DEBUG("Calling map_merger service logOutput");
 *         if(!mm_log_client.call(log))
 *           ROS_WARN("Could not call map_merger service to store log.");
 *         ROS_DEBUG("Finished service call.");
 * 
 *         save_progress();
 * 
 *         ros::Duration(10.0).sleep();
 *       }
 *     }
 * 
 *     **
 *      * size of the global costmap
 *      *
 *     int global_costmap_size()
 *     {
 *       occupancy_grid_global = costmap2d_global->getCostmap()->getCharMap();
 *       int num_map_cells_ = costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
 *       int free = 0;
 * 
 *       for (unsigned int i = 0; i < num_map_cells_; i++) 
 *       {
 *         if ((int) occupancy_grid_global[i] == costmap_2d::FREE_SPACE) 
 *         {
 *           free++;
 *         }
 *       }   
 *       return free;
 *     }
 * 
 *     int local_costmap_size()
 *     {   
 *       {
 *         occupancy_grid_local = costmap2d_local_size->getCostmap()->getCharMap();
 *         int num_map_cells_ = costmap2d_local_size->getCostmap()->getSizeInCellsX() * costmap2d_local_size->getCostmap()->getSizeInCellsY();
 *         int free = 0;
 * 
 *         for (unsigned int i = 0; i < num_map_cells_; i++) 
 *         {
 *           if ((int) occupancy_grid_local[i] == costmap_2d::FREE_SPACE) 
 *           {
 *             free++;
 *           }
 *         }   
 *         return free;
 *       }
 *     }
 */
