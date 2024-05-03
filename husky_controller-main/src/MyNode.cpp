#include <ros/ros.h>
#include "husky_controller/MyController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "husky_controller_node");
  ros::NodeHandle nodeHandle("~");

  husky_controller::MyController myController(nodeHandle);

  ros::spin();
  return 0;
}
