#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

#include "Filt_Frame.hpp"
using namespace std;

class RplidarDistNode
{
    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _ns;

        ros::Subscriber _ros_rplidar_subscriber;
        ros::Publisher _pole_distance_publisher;

        std::string _subscribe_rplidar_topic;
        std::string _pole_distance_topic;

        rp_frame rpfilter;


        void SubscriberInit();
        void PublisherInit();
        void Rplidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan);

    public:
        RplidarDistNode(ros::NodeHandle* nh,ros::NodeHandle* ns);
        ~RplidarDistNode();
};
