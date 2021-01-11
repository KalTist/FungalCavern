#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "RplidarDist.hpp"

RplidarDistNode::RplidarDistNode(ros::NodeHandle* nh,ros::NodeHandle* ns):_nh(*nh),_ns(*ns)
{
    _ns.param<std::string>("subscribe_rplidar_topic",_subscribe_rplidar_topic,"/scan_filtered");
    _ns.param<std::string>("pole_distance_topic",_pole_distance_topic,"pole_distance");
    SubscriberInit();
    PublisherInit();
    this->rpfilter = rp_frame();
}

RplidarDistNode::~RplidarDistNode()
{ 
    _nh.~NodeHandle();
    _ns.~NodeHandle();
}

void RplidarDistNode::SubscriberInit()
{
    _ros_rplidar_subscriber = _nh.subscribe(_subscribe_rplidar_topic,1,&RplidarDistNode::Rplidar_Callback,this);
}

void RplidarDistNode::PublisherInit()
{
    _pole_distance_publisher = _nh.advertise<sensor_msgs::LaserScan>(_pole_distance_topic, 5, true);
}

void RplidarDistNode::Rplidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    vector<float> vec(begin(scan->ranges),end(scan->ranges));
    dt_point pole;
    this->rpfilter.get_distance(vec);
    this->rpfilter.set_angle(scan->angle_min, scan->angle_max, scan->angle_increment);
    pole = this->rpfilter.get_pole();
    if(pole.distance == 0){ pole.angle = 0; }
    // set angle = 0 for print, originally (angle_min + angle_max) / 2 (approximately 0)
    if(pole.type == 0){
        std::cout<<"discarded:";
    }else if(pole.type == 1){
        std::cout<<"reliable: ";
    }else{
        std::cout<<"dubious:  ";
    }
    std::cout<<pole.distance<<",   \t"<<pole.angle<<std::endl;
}