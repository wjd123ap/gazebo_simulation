#include <ros/ros.h>
#include "cleaner_simulation/cleaner_ros.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensorNode");
  ros::NodeHandle nh("~");
 
  Cleaner_Node cleanerNode(nh);
  // Spin
  ros::AsyncSpinner spinner(2);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}