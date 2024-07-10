#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "cleaner_simulation/cleaner.hpp"
#include <math.h>
#include "cleaner_simulation/WheelVel.h"
using namespace std;
bool target_on=false;
Eigen::Vector2d target_pos;
ros::Publisher pub_wheelvel;
double wheel_radius, desired_velocity, pos_threshold,theta_p_gain,theta_d_gain;
double left_wheel_angvel, right_wheel_angvel;
double normalize_angle(double angle) {
    // Normalize angle to be within -PI and PI
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}
double calculate_clockwise_difference(double theta, double desired_theta) {
    double difference = desired_theta - theta;
    difference = fmod(difference + 2 * M_PI, 2 * M_PI);
    return normalize_angle(difference);
}

void odometry_callback(const nav_msgs::OdometryConstPtr &odometry_msg){
    if (target_on){
    Eigen::Vector3d chassis_position(odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y,odometry_msg->pose.pose.position.z);
    Eigen::Vector3d chassis_velocity(odometry_msg->twist.twist.linear.x,odometry_msg->twist.twist.linear.y,odometry_msg->twist.twist.linear.z);
    Eigen::Vector3d chassis_angular(odometry_msg->twist.twist.angular.x,odometry_msg->twist.twist.angular.y,odometry_msg->twist.twist.angular.z);
    Eigen::Quaterniond orientation(odometry_msg->pose.pose.orientation.w,odometry_msg->pose.pose.orientation.x,
    odometry_msg->pose.pose.orientation.y,odometry_msg->pose.pose.orientation.z);
    Eigen::Matrix3d rotation_matrix= orientation.toRotationMatrix();
    double theta = atan2(rotation_matrix(1,0),rotation_matrix(0,0));
    Eigen::Vector2d chassis_position2d(odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y);
    Eigen::Vector2d chassis_velocity2d(odometry_msg->twist.twist.linear.x,odometry_msg->twist.twist.linear.y);
    Eigen::Vector2d target_offset = target_pos-chassis_position2d;
    Eigen::Vector2d target_offset_normalized =target_offset.normalized();
    double desired_theta = atan2(target_offset(1),target_offset(0));
    double error_theta= calculate_clockwise_difference(theta,desired_theta);
    double desired_theta_dot = (target_offset(0)*chassis_velocity(1) - target_offset(1)*chassis_velocity(0))/(target_offset(0)*target_offset(0)+target_offset(1)*target_offset(1)); 
    double error_theta_dot = desired_theta_dot-chassis_angular(2);
    double command_angvel = theta_p_gain*error_theta+theta_d_gain*error_theta_dot;
    double desired_left_angvel = (desired_velocity/wheel_radius) - command_angvel;
    double desired_right_angvel = (desired_velocity/wheel_radius) + command_angvel;
    // cout<<"desired_left_angvel:"<<desired_left_angvel<<endl;
    // cout<<"desired_right_angvel:"<<desired_right_angvel<<endl;
    if ((target_pos-chassis_position2d).norm()<pos_threshold)
    {
      cleaner_simulation::WheelVel wheelvel_msg;
      wheelvel_msg.left=0;
      wheelvel_msg.right=0;

      target_on=false;
      pub_wheelvel.publish(wheelvel_msg);
    }else{
      cleaner_simulation::WheelVel wheelvel_msg;
      wheelvel_msg.left=desired_left_angvel;
      wheelvel_msg.right=desired_right_angvel;

      pub_wheelvel.publish(wheelvel_msg);
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
  nh.param("/parameters/theta_p_gain", theta_p_gain, 1.0);
  nh.param("/parameters/theta_d_gain",theta_d_gain,0.02);
  target_on=true;
  target_pos(0)=init_target_x;
  target_pos(1)=init_target_y;
  ros::Subscriber sub_odometry = nh.subscribe("/robot_cleaner/chassis_pose",2000,odometry_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_wheelstate = nh.subscribe("/robot_cleaner/wheel_state",2000,wheel_state_callback, ros::TransportHints().tcpNoDelay());
  pub_wheelvel = nh.advertise< cleaner_simulation::WheelVel>("/wheelvel_control", 1000);
  // Spin
  ros::AsyncSpinner spinner(2);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}