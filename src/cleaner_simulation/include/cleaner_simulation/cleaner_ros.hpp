#pragma once

// STL
#include <iostream>
#include <mutex>
#include <string>



// ROS
#include <ros/ros.h>
#include <thread>
class Cleaner_Node{
    public:
        Cleaner_Node(ros::NodeHandle& nh);
        ~Cleaner_Node();
    private:
        ros::NodeHandle nh_;
};