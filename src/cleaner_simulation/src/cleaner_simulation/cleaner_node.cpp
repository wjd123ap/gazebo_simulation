#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <queue>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "cleaner_simulation/Fep.h"
#include <math.h>
#include "cleaner_simulation/WheelVel.h"
#include "cleaner_simulation/Odometry.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>

using namespace std;
#define square(x) (x*x)
#define ANGLE_CONSTANT 180.0/M_PI;
bool target_on = false;

enum CONTROL_TYPE{ROTATION,MOVE,MOVE_BACK};

CONTROL_TYPE control_type; 

Eigen::Vector2d target_pos;

ros::Publisher pub_wheelvel;
ros::Publisher pub_FE;
double wheel_radius,chassis_radius,chassis_mass,chassis_inertia;

double desired_velocity, pos_threshold,theta_threshold,move_p_gain,move_i_gain,move_d_gain,rotation_p_gain,rotation_d_gain;
double previous_error_theta,integral_error_theta,constant_wheelVel;
double free_energy, weight1, weight2, weight3,weight4, FE_threshold, MOVE_BACK_threshold;
double tmp_FE_sum=0;
double left_wheelVel, right_wheelVel,left_wheelTorque,right_wheelTorque;
double prev_angvel=0;
bool moveback_stop;
int buffer_size;
queue<double> free_energy_buf;
deque<double> left_wheelVel_buf(2), right_wheelVel_buf(2),left_wheelTorque_buf(2),right_wheelTorque_buf(2);
deque<double> chassis_accel_buf(2), chassis_angvel_buf(3);

double normalize_angle(double angle) {
    // Normalize angle to be within -PI and PI
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    else if (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

double calculate_clockwise_difference(double theta, double desired_theta) {
    double difference = desired_theta - theta;
    difference = fmod(difference + 2 * M_PI, 2 * M_PI);
    return normalize_angle(difference);
}

void stop(){
  cleaner_simulation::WheelVel wheelvel_msg;
  wheelvel_msg.left=0;
  wheelvel_msg.right=0;
  target_on=false;

  integral_error_theta=0;
  pub_wheelvel.publish(wheelvel_msg);
}

void change_target_pos(Eigen::Vector2d& current_pos,double d,double theta) {
  
  target_pos(0) = current_pos(0)-d*cos(theta);
  target_pos(1) = current_pos(1)-d*sin(theta);
}




void odometry_callback(const cleaner_simulation::OdometryConstPtr &odometry_msg){
    if (target_on){
      
      double average_FE=0;
      Eigen::Vector3d chassis_position(odometry_msg->odometry.pose.pose.position.x,odometry_msg->odometry.pose.pose.position.y,
      odometry_msg->odometry.pose.pose.position.z);

      Eigen::Vector3d chassis_velocity(odometry_msg->odometry.twist.twist.linear.x,odometry_msg->odometry.twist.twist.linear.y,
      odometry_msg->odometry.twist.twist.linear.z);
      Eigen::Vector3d chassis_angular(odometry_msg->odometry.twist.twist.angular.x,odometry_msg->odometry.twist.twist.angular.y,
      odometry_msg->odometry.twist.twist.angular.z);
      Eigen::Quaterniond orientation(odometry_msg->odometry.pose.pose.orientation.w,odometry_msg->odometry.pose.pose.orientation.x,
      odometry_msg->odometry.pose.pose.orientation.y,odometry_msg->odometry.pose.pose.orientation.z);
      Eigen::Matrix3d rotation_matrix= orientation.toRotationMatrix();
      
      double theta = atan2(rotation_matrix(1,0),rotation_matrix(0,0));
      Eigen::Vector2d chassis_position2d(odometry_msg->odometry.pose.pose.position.x,odometry_msg->odometry.pose.pose.position.y);
      Eigen::Vector2d chassis_velocity2d(odometry_msg->odometry.twist.twist.linear.x,odometry_msg->odometry.twist.twist.linear.y);
      
      double chassis_accel = odometry_msg -> accel.x;
      chassis_accel_buf.pop_front();
      chassis_accel_buf.push_back(chassis_accel);
      chassis_angvel_buf.pop_front();
      chassis_angvel_buf.push_back(chassis_angular(2));
      left_wheelVel_buf.pop_front();
      left_wheelVel_buf.push_back(left_wheelVel);
      right_wheelVel_buf.pop_front();
      right_wheelVel_buf.push_back(right_wheelVel);
      left_wheelTorque_buf.pop_front();
  
      left_wheelTorque_buf.push_back(left_wheelTorque);
      right_wheelTorque_buf.pop_front();
      right_wheelTorque_buf.push_back(right_wheelTorque);


      // cout<<"tilt_angle:"<<tilt_angle*180/M_PI<<endl;
      if ((target_pos-chassis_position2d).norm()<pos_threshold)
      {
        stop();
        return;
      }
      Eigen::Vector2d target_offset = target_pos-chassis_position2d;
      Eigen::Vector2d target_offset_normalized =target_offset.normalized();
      Eigen::Vector3d tilt_vec=rotation_matrix.col(2);
      double tilt_angle = acos(tilt_vec(2)/tilt_vec.norm());
      double angaccel = (chassis_angular(2) - prev_angvel)/0.005;
      prev_angvel=chassis_angular(2);
      if (control_type == MOVE_BACK){
        theta=fmod(theta+M_PI,2 * M_PI);
      }
      double desired_theta = atan2(target_offset(1),target_offset(0));
      double error_theta= calculate_clockwise_difference(theta,desired_theta);
      double desired_left_angvel,desired_right_angvel;



      // cout<<"error_theta:"<<error_theta<<endl;
      if ((abs(error_theta)>M_PI/36)&&(control_type==MOVE)){
        control_type=ROTATION;
        previous_error_theta=error_theta;
      }

      if (control_type != ROTATION){
        double residual_velocity = chassis_velocity2d.norm() - ((left_wheelVel + right_wheelVel) * wheel_radius / 2);
        double residual_angvel = chassis_angular(2)- ((right_wheelVel - left_wheelVel ) * wheel_radius / (2*chassis_radius));
        double residual_accel = chassis_accel-((left_wheelTorque + right_wheelTorque)/(wheel_radius*chassis_mass));
        double residual_tilt = tilt_angle*ANGLE_CONSTANT;

        free_energy = weight1 * square(residual_velocity) + weight2 * square(residual_angvel) + weight3 * square(residual_accel) + weight4 * square(residual_tilt);
        // cout<<"residual_velocity:"<<residual_velocity<<", residual_angvel:"<<residual_angvel<<", residual_accel:"<<residual_accel<<endl;
        free_energy_buf.push(free_energy);  
        tmp_FE_sum = tmp_FE_sum + free_energy;
        if (free_energy_buf.size()>buffer_size){
          tmp_FE_sum = tmp_FE_sum - free_energy_buf.front();
          free_energy_buf.pop();
          average_FE = tmp_FE_sum / buffer_size;
        }
        // cout<<"control_type:"<<control_type<<endl;
        // cout<<"free_energy:"<<free_energy<<endl;
        // cout<<"average_FE:"<<average_FE<<endl;
        if ((average_FE>FE_threshold)&&(control_type != MOVE_BACK)){
          change_target_pos(chassis_position2d,1.0,theta);
          previous_error_theta=error_theta;
          integral_error_theta=0;

          moveback_stop=false;
          control_type=MOVE_BACK;
        }
        // cout<<"average_FE:"<<average_FE<<endl;
        if ((control_type == MOVE_BACK)&&(average_FE<MOVE_BACK_threshold)&&(!moveback_stop)){
          change_target_pos(chassis_position2d,-0.05,theta);
          moveback_stop=true;
          return;
        }

        std_msgs::Float64 FE_msgs;
        FE_msgs.data = free_energy;
        pub_FE.publish(FE_msgs);
      }

      double error_theta_dot,command_angvel;

      switch (control_type) {
          case ROTATION:
              if (abs(error_theta)>theta_threshold){

                error_theta_dot = (error_theta-previous_error_theta);
                command_angvel = rotation_p_gain*error_theta+rotation_d_gain*error_theta_dot;
                desired_left_angvel = - command_angvel;
                desired_right_angvel = + command_angvel;
                previous_error_theta = error_theta;
                
              }
              else{
                control_type=MOVE;
                desired_left_angvel = constant_wheelVel;
                desired_right_angvel = constant_wheelVel;
              }
              break;
          case MOVE:
              // double desired_theta_dot = (target_offset(0)*chassis_velocity(1) - target_offset(1)*chassis_velocity(0))/(target_offset(0)*target_offset(0)+target_offset(1)*target_offset(1)); 
              // double error_theta_dot = desired_theta_dot - chassis_angular(2);
              error_theta_dot = (error_theta-previous_error_theta);
              integral_error_theta += error_theta;
              previous_error_theta = error_theta;

              command_angvel = move_p_gain * error_theta + move_i_gain * integral_error_theta + move_d_gain * error_theta_dot;
              desired_left_angvel = constant_wheelVel - command_angvel;
              desired_right_angvel = constant_wheelVel + command_angvel;
              break;
          case MOVE_BACK:
              error_theta_dot = (error_theta-previous_error_theta);
              integral_error_theta += error_theta;
              previous_error_theta = error_theta;
              command_angvel = move_p_gain * error_theta + move_i_gain * integral_error_theta + move_d_gain * error_theta_dot;
              desired_left_angvel = -(constant_wheelVel + command_angvel);
              desired_right_angvel = -(constant_wheelVel - command_angvel);
              break;
          default:
              break;
      }
      
      cleaner_simulation::WheelVel wheelvel_msg;
      wheelvel_msg.left = desired_left_angvel;
      wheelvel_msg.right = desired_right_angvel;
      pub_wheelvel.publish(wheelvel_msg);
    }

}

void wheel_state_callback(const sensor_msgs::JointStateConstPtr &jointstate_msg){
  left_wheelVel = (jointstate_msg->velocity)[0];
  right_wheelVel = (jointstate_msg->velocity)[1];
  left_wheelTorque = (jointstate_msg->effort)[0];
  right_wheelTorque = (jointstate_msg->effort)[1];
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
  nh.param("/parameters/weight1", weight1, 1.0);
  nh.param("/parameters/weight2", weight2, 1.0);
  nh.param("/parameters/weight3", weight3, 1.0);
  nh.param("/parameters/weight4", weight4, 1.0);
  nh.param("/parameters/buffer_size",buffer_size,10);
  nh.param("/parameters/FE_threshold",FE_threshold,1.5);
  nh.param("/parameters/MOVE_BACK_threshold",MOVE_BACK_threshold,0.5);
  nh.param("/parameters/chassis_inertia",chassis_inertia,0.046);
  target_on=true;
  control_type=MOVE;
  target_pos(0)=init_target_x;
  target_pos(1)=init_target_y;
  ros::Subscriber sub_odometry = nh.subscribe("/robot_cleaner/chassis_pose",2000,odometry_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_wheelstate = nh.subscribe("/robot_cleaner/wheel_state",2000,wheel_state_callback, ros::TransportHints().tcpNoDelay());
  pub_wheelvel = nh.advertise< cleaner_simulation::WheelVel>("/wheelvel_control", 1000);
  pub_FE = nh.advertise<std_msgs::Float64>("/free_energy",1000);

  constant_wheelVel=desired_velocity/wheel_radius;

  // Spin
  ros::AsyncSpinner spinner(3);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}