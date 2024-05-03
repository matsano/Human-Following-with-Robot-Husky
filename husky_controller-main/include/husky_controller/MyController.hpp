#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"

namespace husky_controller
{

    /*!
     * Class containing the Husky Controller
     */
    class MyController
    {
    public:
        MyController(ros::NodeHandle &nodeHandle);

    private:
        ros::NodeHandle *m_nodeHandle;

        ros::Subscriber m_clusterCenterSubscriber;

        void controlCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

        ros::Publisher m_commandVelocityPublisher;

        float targetDistance;
        float kD;
        float kW;
    };

} /* namespace */
