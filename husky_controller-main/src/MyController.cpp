#include "husky_controller/MyController.hpp"

namespace husky_controller
{
  MyController::MyController(ros::NodeHandle &nodeHandle) : m_nodeHandle(&nodeHandle)
  {
    std::string scanTopicName;
    int scanTopicQueueSize;
    if (!m_nodeHandle->getParam("scan_name", scanTopicName))
    {
      ROS_ERROR("Error getting paramenter scan/name\n");
      return;
    }
    if (!m_nodeHandle->getParam("scan_queue_size", scanTopicQueueSize))
    {
      ROS_ERROR("Error getting paramenter scan/queue_size\n");
      return;
    }
    m_clusterCenterSubscriber = m_nodeHandle->subscribe(scanTopicName, scanTopicQueueSize, &MyController::controlCallback, this);

    std::string commandVelocityTopicName;
    int commandVelocityTopicQueueSize;
    if (!m_nodeHandle->getParam("command_velocity_name", commandVelocityTopicName))
    {
      ROS_ERROR("Error getting paramenter command_velocity/name\n");
      return;
    }
    if (!m_nodeHandle->getParam("command_velocity_queue_size", commandVelocityTopicQueueSize))
    {
      ROS_ERROR("Error getting paramenter command_velocity/queue_size\n");
      return;
    }
    m_commandVelocityPublisher = m_nodeHandle->advertise<geometry_msgs::Twist>(commandVelocityTopicName, commandVelocityTopicQueueSize);

    if (!m_nodeHandle->getParam("target_distance", targetDistance))
    {
      ROS_ERROR("Error getting paramenter target_distance\n");
      return;
    }
    if (!m_nodeHandle->getParam("kd", kD))
    {
      ROS_ERROR("Error getting paramenter kD\n");
      return;
    }
    if (!m_nodeHandle->getParam("kw", kW))
    {
      ROS_ERROR("Error getting paramenter kW\n");
      return;
    }
  }

  void MyController::controlCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
  {
    // calculate the distance to the target
    float omega_cmd = kW * (msg->point.y - 0);
    float dist = sqrt(pow(msg->point.x, 2) + pow(msg->point.y, 2));
    float speed_cmd = kD * (dist - targetDistance);

    // create a command velocity message using a proportional gain an publish it
    geometry_msgs::Twist commandVelocityMessage;
    // FIXME: REPAIRE
    commandVelocityMessage.linear.x = speed_cmd;
    commandVelocityMessage.angular.z = omega_cmd;
    m_commandVelocityPublisher.publish(commandVelocityMessage);

    return;
  }

} /* namespace */
