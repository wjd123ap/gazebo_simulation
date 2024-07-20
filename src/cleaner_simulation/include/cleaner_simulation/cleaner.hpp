#pragma once

// STL
#include <iostream>
#include <mutex>
#include <string>



// ROS
#include <ros/ros.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
class Cleaner{
    public:
        Cleaner();
        ~Cleaner();

    private:

};