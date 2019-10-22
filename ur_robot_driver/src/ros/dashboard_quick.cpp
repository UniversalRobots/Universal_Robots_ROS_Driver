#include <ros/ros.h>
#include <ur_robot_driver/ros/dashboard_client.h>

std::string ROBOT_IP = "192.168.56.101";

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "dashboard_quick");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  ur_driver::DashboardClientROS client(priv_nh, ROBOT_IP);

  ros::spin();
  return 0;
}
