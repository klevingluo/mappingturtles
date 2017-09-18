#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string turtle_name;
tf::Transform init_pos;
bool saved;

void groundTruthCallback(const nav_msgs::OdometryPtr& msg){
  static tf::TransformBroadcaster br;

  if (!saved) {
    init_pos.setOrigin(
        tf::Vector3(- msg->pose.pose.position.x, 
                    - msg->pose.pose.position.y,
                    msg->pose.pose.position.z));

    tf::Quaternion q;
    q.setRPY(0, 
             0, 
             msg->pose.pose.orientation.w);

    init_pos.setRotation(q);

    saved = true;
  }

  br.sendTransform(
      tf::StampedTransform(init_pos, 
                           ros::Time::now(), 
                           "world", 
                           turtle_name + "/map"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_ground_truth_publisher");

  if (argc != 2) {
    ROS_ERROR("need turtle name as argument"); 
    return -1;
  };

  turtle_name = argv[1];
  
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(
      "base_pose_ground_truth", 
      10, 
      &groundTruthCallback);
  
  ros::spin();
  return 0;

}
