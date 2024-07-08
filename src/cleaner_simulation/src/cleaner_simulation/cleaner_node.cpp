#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "cleaner_simulation/cleaner.hpp"

using namespace std;
bool target_on=false;
Eigen::Vector2d target_pos;
double wheel_radius, desired_velocity, pos_threshold;
double left_wheel_angvel, right_wheel_angvel;


void odometry_callback(const nav_msgs::OdometryConstPtr &odometry_msg){
    if (target_on){
    Eigen::Vector3d chassis_position(odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y,odometry_msg->pose.pose.position.z);
    Eigen::Vector3d chassis_velocity(odometry_msg->twist.twist.linear.x,odometry_msg->twist.twist.linear.y,odometry_msg->twist.twist.linear.z);
     
    Eigen::Quaterniond orientation(odometry_msg->pose.pose.orientation.w,odometry_msg->pose.pose.orientation.x,
    odometry_msg->pose.pose.orientation.y,odometry_msg->pose.pose.orientation.z);
    Eigen::Matrix3d rotation_matrix= orientation.toRotationMatrix();
    double theta = atan2(rotation_matrix(1,0),rotation_matrix(0,0));
    Eigen::Vector2d chassis_position2d(odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y);
    Eigen::Vector2d chassis_velocity2d(odometry_msg->twist.twist.linear.x,odometry_msg->twist.twist.linear.y);
    Eigen::Vector2d target_offset = target_pos-chassis_position2d;
    target_offset.normalize();
    double desired_theta = atan2(target_offset(1),target_offset(0));

    if ((target_pos-chassis_position2d).norm()<pos_threshold)
    {

      target_on=false;
    }
    
    }
    // cout<<"theta:"<<theta<<endl;
}
void wheel_state_callback(const sensor_msgs::JointStateConstPtr &jointstate_msg){
  left_wheel_angvel = (jointstate_msg->velocity)[0];
  right_wheel_angvel = (jointstate_msg->velocity)[1];
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "cleanerNode");
  ros::NodeHandle nh("~");
  double init_target_x,init_target_y;
  nh.param("/parameters/desired_velocity", desired_velocity, 0.2);

  nh.param("/parameters/wheel_radius", wheel_radius, 0.0375);
  nh.param("/parameters/init_target_x", init_target_x, 0.0);
  nh.param("/parameters/init_target_y", init_target_y, 0.0);
  nh.param("/parameters/pos_threshold",pos_threshold,0.02);
  target_on=true;
  target_pos(0)=init_target_x;
  target_pos(1)=init_target_y;
  ros::Subscriber sub_odometry = nh.subscribe("/robot_cleaner/chassis_pose",2000,odometry_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_wheelstate = nh.subscribe("/robot_cleaner/wheel_state",2000,wheel_state_callback, ros::TransportHints().tcpNoDelay());
  // Spin
  ros::AsyncSpinner spinner(2);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}