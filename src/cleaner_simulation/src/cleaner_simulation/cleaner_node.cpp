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
bool target_on = false;
enum CONTROL_TYPE{MOVE,ROTATION};
CONTROL_TYPE control_type; 

Eigen::Vector2d target_pos;
ros::Publisher pub_wheelvel;
double wheel_radius,chassis_radius,chassis_mass;
double desired_velocity, pos_threshold,theta_threshold,move_p_gain,move_i_gain,move_d_gain,rotation_p_gain,rotation_d_gain;
double left_wheel_angvel, right_wheel_angvel,left_wheel_torque,right_wheel_torque,previous_error_theta,integral_error_theta,constant_wheel_vel;
double free_energy;
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
    if ((target_pos-chassis_position2d).norm()<pos_threshold)
    {
      cleaner_simulation::WheelVel wheelvel_msg;
      wheelvel_msg.left=0;
      wheelvel_msg.right=0;

      target_on=false;
      integral_error_theta=0;
      pub_wheelvel.publish(wheelvel_msg);
      return;
    }

    Eigen::Vector2d target_offset = target_pos-chassis_position2d;
    Eigen::Vector2d target_offset_normalized =target_offset.normalized();
    double desired_theta = atan2(target_offset(1),target_offset(0));
    double error_theta= calculate_clockwise_difference(theta,desired_theta);
    double desired_left_angvel,desired_right_angvel;

    // cout<<"current_theta:"<<theta<<", desired_theta:"<<desired_theta<<endl;
    if ((abs(error_theta)>M_PI/36)&(control_type==MOVE)){
      control_type=ROTATION;
      previous_error_theta=error_theta;

    }

    if (control_type==ROTATION){

      if (abs(error_theta)>theta_threshold){
      // cout<<"error_theta2:"<<error_theta<<endl;
        double error_theta_dot = (error_theta-previous_error_theta);
        double command_angvel = rotation_p_gain*error_theta+rotation_d_gain*error_theta_dot;
        desired_left_angvel = - command_angvel;
        desired_right_angvel = + command_angvel;
        previous_error_theta = error_theta;
      }
      else{
        control_type=MOVE;
        previous_error_theta = 0;
        desired_left_angvel = constant_wheel_vel;
        desired_right_angvel = constant_wheel_vel;
      }
    }
    else{
        // desired_left_angvel = 0;
        // desired_right_angvel = 0;
        double desired_theta_dot = (target_offset(0)*chassis_velocity(1) - target_offset(1)*chassis_velocity(0))/(target_offset(0)*target_offset(0)+target_offset(1)*target_offset(1)); 
        double error_theta_dot = desired_theta_dot - chassis_angular(2);
        integral_error_theta += error_theta;
        std::cout<<"integral_error_theta:"<<integral_error_theta<<std::endl;
        std::cout<<"error_theta:"<<error_theta<<std::endl;
        double command_angvel = move_p_gain * error_theta + move_i_gain * integral_error_theta + move_d_gain * error_theta_dot;
        desired_left_angvel = constant_wheel_vel - command_angvel;
        desired_right_angvel = constant_wheel_vel + command_angvel;
    }

    // cout<<"desired_left_angvel:"<<desired_left_angvel<<endl;
    // cout<<"desired_right_angvel:"<<desired_right_angvel<<endl;

    cleaner_simulation::WheelVel wheelvel_msg;
    wheelvel_msg.left = desired_left_angvel;
    wheelvel_msg.right = desired_right_angvel;

    pub_wheelvel.publish(wheelvel_msg);
    if (control_type == MOVE){
      double residual_velocity = chassis_velocity2d.norm() - ((left_wheel_angvel + right_wheel_angvel) * wheel_radius / 2);
      double residual_angvel = chassis_angular(2)- ((right_wheel_angvel - left_wheel_angvel ) * wheel_radius / (2*chassis_radius));
      double residual_accel = (left_wheel_torque + right_wheel_torque)/(2*wheel_radius*chassis_mass);
      free_energy = residual_velocity * residual_velocity + residual_angvel * residual_angvel + residual_accel * residual_accel;
    }
    }

}
void wheel_state_callback(const sensor_msgs::JointStateConstPtr &jointstate_msg){
  left_wheel_angvel = (jointstate_msg->velocity)[0];
  right_wheel_angvel = (jointstate_msg->velocity)[1];
  left_wheel_torque = (jointstate_msg->effort)[0];
  right_wheel_torque = (jointstate_msg->effort)[1];
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "cleanerNode");
  ros::NodeHandle nh("~");
  double init_target_x,init_target_y;
  nh.param("/parameters/desired_velocity", desired_velocity, 0.2);
  
  nh.param("/parameters/wheel_radius", wheel_radius, 0.0375);
  nh.param("/parameters/chassis_radius", chassis_radius, 0.114);
  nh.param("/parameters/chassis_mass", chassis_mass, 4.0);
  nh.param("/parameters/init_target_x", init_target_x, 0.0);
  nh.param("/parameters/init_target_y", init_target_y, 0.0);
  nh.param("/parameters/theta_threshold",theta_threshold,0.02);
  nh.param("/parameters/pos_threshold",pos_threshold,0.02);
  nh.param("/parameters/rotation_p_gain", rotation_p_gain, 1.0);
  nh.param("/parameters/rotation_d_gain",rotation_d_gain,0.02);
  nh.param("/parameters/move_p_gain", move_p_gain, 1.0);
  nh.param("/parameters/move_i_gain", move_i_gain, 1.0);
  nh.param("/parameters/move_d_gain",move_d_gain,0.02);
  target_on=true;
  control_type=MOVE;
  target_pos(0)=init_target_x;
  target_pos(1)=init_target_y;
  ros::Subscriber sub_odometry = nh.subscribe("/robot_cleaner/chassis_pose",2000,odometry_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_wheelstate = nh.subscribe("/robot_cleaner/wheel_state",2000,wheel_state_callback, ros::TransportHints().tcpNoDelay());
  pub_wheelvel = nh.advertise< cleaner_simulation::WheelVel>("/wheelvel_control", 1000);
  constant_wheel_vel=desired_velocity/wheel_radius;

  // Spin
  ros::AsyncSpinner spinner(2);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}