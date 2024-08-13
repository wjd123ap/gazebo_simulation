#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>
#include "cleaner_simulation/TorqueTest.h"
#include "cleaner_simulation/WheelVel.h"
#include "cleaner_simulation/Odometry.h"
#include <cmath>
#include <algorithm>
#include <sensor_msgs/JointState.h>
#include <random>
#include <Eigen/Core>
#include <Eigen/Dense>
// #include
using namespace std;
const double stall_torque=(77.5/32.5);
const double alpha=0.5;
const  double odometry_timer_interval = (1.0)/200;
std::mt19937 gen1(123456); // 난수 엔진 초기화
std::mt19937 gen2(12145); // 난수 엔진 초기화


std::normal_distribution<double> velocity_white_noise(0.0, 0.01);
std::normal_distribution<double> orientation_white_noise(0.0, 0.003);


double last_pos_x_n=0.0,last_pos_y_n=0.0;
double last_vel_x_n=velocity_white_noise(gen1);
double last_vel_y_n=velocity_white_noise(gen1);
double last_orientation_x_n = orientation_white_noise(gen2);
double last_orientation_y_n = orientation_white_noise(gen2);
double last_orientation_z_n = orientation_white_noise(gen2);
double last_orientation_w_n = orientation_white_noise(gen2);

namespace gazebo
{
  class ModelControlPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        ROS_INFO("ModelPlugin for %s loaded successfully!", _model->GetName().c_str());

        if (!ros::isInitialized())
        {      odometry.twist.twist.angular.x = this->chassis->RelativeAngularVel().X();
      odometry.twist.twist.angular.y = this->chassis->RelativeAngularVel().Y();
      odometry.twist.twist.angular.z = this->chassis->RelativeAngularVel().Z();
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "robot_cleaner", ros::init_options::NoSigintHandler);
        }

        physics::WorldPtr world = _model->GetWorld();

        double motor_hz;
        physics::PhysicsEnginePtr physics = world->Physics();

        if (_sdf->HasElement("wheel_p_gain")){
          this->wheel_p=_sdf->Get<double>("wheel_p_gain");
        }
        if (_sdf->HasElement("wheel_i_gain")){
          this->wheel_i=_sdf->Get<double>("wheel_i_gain");
        }
        if (_sdf->HasElement("wheel_d_gain")){
          this->wheel_d=_sdf->Get<double>("wheel_d_gain");
        }
        if (_sdf->HasElement("motor_updateRateHZ")){
          motor_hz=_sdf->Get<double>("motor_updateRateHZ");
        }
        this->model = _model;


        this->stepsize = (1.0)/motor_hz;
        cout<<"this->stepsize:"<<this->stepsize<<endl;
        this->rosNode.reset(new ros::NodeHandle("robot_cleaner"));
        this->left_wheel = _model->GetJoint("wheel_rear_left_spin");
        this->right_wheel = _model->GetJoint("wheel_rear_right_spin");
        // this->front_left_wheel = _model->GetJoint("wheel_front_left_spin");
        // this->front_right_wheel = _model->GetJoint("wheel_front_right_spin");
        // this->front_left_wheel->SetProvideFeedback(true);
        // this->front_right_wheel->SetProvideFeedback(true); 
        this->left_wheel->SetProvideFeedback(true);
        this->right_wheel->SetProvideFeedback(true); 
        this->left_rear_wheel_link= _model->GetLink("wheel_rear_left");
        this->right_rear_wheel_link= _model->GetLink("wheel_rear_right");
        // this->left_front_wheel_link= _model->GetLink("wheel_front_left");
        // this->right_front_wheel_link= _model->GetLink("wheel_front_right");
        this->last_updatetime=0;
        this->left_torque = 0;
        this->right_torque = 0;
        this->desired_left_angvel = 0;
        this->desired_right_angvel = 0;
        this->chassis = _model->GetLink("chassis");
        this->last_left_wheel_angle = 0;
        this->last_right_wheel_angle = 0;
        left_error_integral=0;
        right_error_integral=0;

        this->left_velocityPID.Init(this->wheel_p, this->wheel_i, this->wheel_d, 1.0, -1.0, stall_torque, -stall_torque);
        this->right_velocityPID.Init(this->wheel_p, this->wheel_i, this->wheel_d, 1.0, -1.0, stall_torque, -stall_torque);
        std::cout << " Joint Name: " << this->right_wheel->GetName() << ", Type: " << this->right_wheel->GetType() << std::endl;
        this->torque_Subscriber = this->rosNode->subscribe("/robot_control", 10, &ModelControlPlugin::OnTorqueMsg, this);
        this->wheelvel_Subscriber = this->rosNode->subscribe("/wheelvel_control", 10, &ModelControlPlugin::OnWheelVelMsg, this);
        this->imu_Subscriber = this->rosNode->subscribe("/robot_cleaner/imu", 10, &ModelControlPlugin::OnImuMsg, this);       
        this->pose_Publisher = this->rosNode->advertise<cleaner_simulation::Odometry>("chassis_pose", 1000);
        
        this->wheelstate_Publisher = this->rosNode->advertise<sensor_msgs::JointState>("wheel_state", 1000);
        
        motor_timer = rosNode->createTimer(ros::Duration(this->stepsize), &ModelControlPlugin::OnWheelState, this);


        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelControlPlugin::OnUpdateState, this));
        ROS_INFO("RobotControlPlugin loaded");

    }


    void OnTorqueMsg(const cleaner_simulation::TorqueTestConstPtr &msg)
    {
        this->left_torque=msg->left_torque;
        this->right_torque=msg->right_torque;

    }

    void OnWheelVelMsg(const cleaner_simulation::WheelVelConstPtr &msg)
    {
        this->desired_left_angvel=msg->left;
        this->desired_right_angvel=msg->right;
    }

    void OnImuMsg(const sensor_msgs::ImuConstPtr &msg)
    {
      
      this->chassis_angvel[0] = msg->angular_velocity.x;
      this->chassis_angvel[1] = msg->angular_velocity.y;
      this->chassis_angvel[2] = msg->angular_velocity.z;
      cleaner_simulation::Odometry odometry_msg;
      nav_msgs::Odometry odometry;
      odometry.header.frame_id = "world";
      odometry.header.stamp = msg->header.stamp;
      double cur_vel_x_n=alpha*velocity_white_noise(gen1)+(1-alpha)*last_vel_x_n;
      double cur_vel_y_n=alpha*velocity_white_noise(gen1)+(1-alpha)*last_vel_y_n;
      double cur_orientation_x_n=alpha*orientation_white_noise(gen2)+(1-alpha)*last_orientation_x_n;
      double cur_orientation_y_n=alpha*orientation_white_noise(gen2)+(1-alpha)*last_orientation_y_n;
      double cur_orientation_z_n=alpha*orientation_white_noise(gen2)+(1-alpha)*last_orientation_z_n;
      double cur_orientation_w_n=alpha*orientation_white_noise(gen2)+(1-alpha)*last_orientation_w_n;
      double cur_pos_x_n= odometry_timer_interval * cur_vel_x_n+last_pos_x_n;
      double cur_pos_y_n= odometry_timer_interval * cur_vel_y_n+last_pos_y_n;
      // gzmsg << "cur_vel_x_n: " << cur_vel_x_n <<std::endl;
      // gzmsg << "cur_vel_y_n: " << cur_vel_y_n <<std::endl;
      odometry.pose.pose.position.x = this->chassis->WorldPose().Pos().X()+cur_pos_x_n;
      odometry.pose.pose.position.y = this->chassis->WorldPose().Pos().Y()+cur_pos_y_n;
      odometry.pose.pose.position.z = this->chassis->WorldPose().Pos().Z();
      Eigen::Quaterniond q(
      this->chassis->WorldPose().Rot().W()+cur_orientation_w_n,
      this->chassis->WorldPose().Rot().X()+cur_orientation_x_n,
       this->chassis->WorldPose().Rot().Y()+cur_orientation_y_n,
       this->chassis->WorldPose().Rot().Z()+cur_orientation_z_n);
      q.normalize();
      odometry.pose.pose.orientation.w = q.w();
      odometry.pose.pose.orientation.x = q.x();
      odometry.pose.pose.orientation.y = q.y();
      odometry.pose.pose.orientation.z = q.z();
      odometry.twist.twist.angular.x = this->chassis->RelativeAngularVel().X();
      odometry.twist.twist.angular.y = this->chassis->RelativeAngularVel().Y();
      odometry.twist.twist.angular.z = this->chassis->RelativeAngularVel().Z();
      odometry.twist.twist.linear.x = this->chassis->WorldLinearVel().X()+cur_vel_x_n;
      odometry.twist.twist.linear.y = this->chassis->WorldLinearVel().Y()+cur_vel_y_n;
      odometry.twist.twist.linear.z = this->chassis->WorldLinearVel().Z();
      odometry_msg.odometry=odometry;
      odometry_msg.accel.x= this->chassis-> RelativeLinearAccel().X();
      odometry_msg.accel.y= this->chassis-> RelativeLinearAccel().Y();
      odometry_msg.accel.z= this->chassis-> RelativeLinearAccel().Z();
      pose_Publisher.publish(odometry_msg);
      cur_pos_x_n=last_pos_x_n;
      cur_pos_y_n=last_pos_y_n;
      cur_vel_x_n=last_vel_x_n;
      cur_vel_y_n=last_vel_y_n;
      cur_orientation_x_n=last_orientation_x_n;
      cur_orientation_y_n=last_orientation_y_n;
      cur_orientation_z_n=last_orientation_z_n;
      cur_orientation_w_n=last_orientation_w_n;
    }


    
    void OnWheelState(const ros::TimerEvent&)
    {

        sensor_msgs::JointState wheelMsgs;
        // 조인트 상태 읽기
        wheelMsgs.header.stamp = ros::Time::now();
        double steptime=wheelMsgs.header.stamp.toSec()-this->last_updatetime;
        this->last_updatetime=wheelMsgs.header.stamp.toSec();
        double left_angle = this->left_wheel->Position(0); // 조인트 각도 (라디안)
        double normalized_left_angle = std::fmod(left_angle, 2 * M_PI);
        if (normalized_left_angle < 0) {
            normalized_left_angle += 2 * M_PI;  // 음수 각도를 양수로 조정
        }


        double right_angle = this->right_wheel->Position(0); // 조인트 각도 (라디안)
        double normalized_right_angle = std::fmod(right_angle, 2 * M_PI);
        if (normalized_right_angle < 0) {
            normalized_right_angle += 2 * M_PI;  // 음수 각도를 양수로 조정
        }
        // double left_angvel = this->left_wheel->GetVelocity(0);
        // double right_angvel = this->right_wheel->GetVelocity(0);
        double left_angvel = (left_angle-this->last_left_wheel_angle)/steptime; // 조인트 각속도 (라디안/초)
        double right_angvel = (right_angle-this->last_right_wheel_angle)/steptime; // 조인트 각속도 (라디안/초)
        this->last_left_wheel_angle=left_angle;
        this->last_right_wheel_angle=right_angle;
        wheelMsgs.header.frame_id = "chassis";
        wheelMsgs.name.push_back("left_wheel");
        wheelMsgs.name.push_back("right_wheel");
        // wheelpos (radian)
        wheelMsgs.position.push_back(left_angle);
        wheelMsgs.position.push_back(right_angle);
        // wheel_velocity
        wheelMsgs.velocity.push_back(left_angvel);
        wheelMsgs.velocity.push_back(right_angvel);

        // gzmsg << "desired_left_angvel: " << this->desired_left_angvel<< ",desired_right_angvel: " << this->desired_right_angvel<<std::endl;
        double left_error = left_angvel-this->desired_left_angvel;
        double right_error = right_angvel-this->desired_right_angvel;
        // gzmsg << "left_error: " <<left_error  << ", right_error: " << right_error<<std::endl;

        this->left_torque = this -> left_velocityPID.Update(left_error, steptime);
        this->right_torque = this -> right_velocityPID.Update(right_error, steptime);
        double p_error,i_error,d_error;
        this->left_velocityPID.GetErrors(p_error,i_error,d_error);
        // gzmsg << "i_error: " <<i_error  << ", p_error: " << p_error<<std::endl;
        double left_torque_sign = std::copysign(1.0, this->left_torque); 
        double right_torque_sign = std::copysign(1.0, this->right_torque); 
        double left_angvel_sign = std::copysign(1.0, left_angvel); 
        double right_angvel_sign = std::copysign(1.0, right_angvel); 
        if (left_torque_sign*left_angvel_sign>0){
          double left_torque=max((-1/32.5)*((abs(left_angvel*60)/(2*M_PI))-77.5),0.0);
          // gzmsg << "left_torque1: " << this->left_torque <<std::endl;
          this->left_torque=min(left_torque,abs(this->left_torque));
          this->left_torque=(this->left_torque)*left_torque_sign;
        }
        else{
          this->left_torque=left_torque_sign*min(stall_torque,abs(this->left_torque));
        }       
          // gzmsg << "left_torque: " << this->left_torque  << ", left_angvel: " << left_angvel<<std::endl;

        if (right_torque_sign*right_angvel_sign>0){
          double right_torque=max((-1/32.5)*((abs(right_angvel*60)/(2*M_PI))-77.5),0.0);
          // gzmsg << "right_torque1: " << this->right_torque <<std::endl;

          this->right_torque=min(right_torque,abs(this->right_torque));
          this->right_torque=(this->right_torque)*right_torque_sign;
        }
        else{
          this->right_torque=right_torque_sign*min(stall_torque,abs(this->right_torque));
        } 
        // gzmsg << "left_torque: " << this->left_torque    <<", desried_angvel:"<<desired_left_angvel <<", left_angvel: " << left_angvel<<std::endl;
        // gzmsg << "right_torque: " << this->right_torque <<", desried_angvel:"<<desired_right_angvel <<  ", right_angvel: " << right_angvel<<std::endl;
        // ignition::math::Vector3d left_link_torque(0, 0, this->left_torque);
        // ignition::math::Vector3d right_link_torque(0, 0, this->right_torque);
        // this->left_rear_wheel_link->AddRelativeTorque(left_link_torque);
        // this->right_rear_wheel_link->AddRelativeTorque(right_link_torque);
        // this->left_wheel->SetForce(0, this->left_torque);
        // this->right_wheel->SetForce(0, this->right_torque);
        wheelMsgs.effort.push_back(this->left_torque);
        wheelMsgs.effort.push_back(this->right_torque);
        wheelstate_Publisher.publish(wheelMsgs);

    }
    void OnUpdateState(){
      ignition::math::Vector3d left_link_torque(0, 0, this->left_torque);
      ignition::math::Vector3d right_link_torque(0, 0, this->right_torque);
      this->left_rear_wheel_link->AddRelativeTorque(left_link_torque);
      this->right_rear_wheel_link->AddRelativeTorque(right_link_torque);
    }


  private:
    std::unique_ptr<ros::NodeHandle> rosNode;

    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    physics::JointPtr front_left_wheel;
    physics::JointPtr front_right_wheel;
    physics::JointPtr left_wheel;
    physics::JointPtr right_wheel;
    physics::LinkPtr chassis;
    physics::LinkPtr left_front_wheel_link;
    physics::LinkPtr right_front_wheel_link;
    physics::LinkPtr left_rear_wheel_link;
    physics::LinkPtr right_rear_wheel_link;
    std::string right_wheel_jointName;
    std::string left_wheel_jointName;
    ros::Subscriber torque_Subscriber;
    ros::Subscriber wheelvel_Subscriber;
    ros::Subscriber imu_Subscriber;
    ros::Publisher pose_Publisher;
    ros::Publisher wheelstate_Publisher;
    ros::Timer odometry_timer;
    ros::Timer motor_timer;
    common::PID left_velocityPID;
    common::PID right_velocityPID;
    double left_torque;
    double right_torque;
    double chassis_angvel[3];
    double last_updatetime;
    double last_left_wheel_angle;
    double last_right_wheel_angle;
    double left_error_integral;
    double right_error_integral;
    double desired_left_angvel;
    double desired_right_angvel;
    double wheel_p;
    double wheel_i;
    double wheel_d;
    double stepsize;

  };

  GZ_REGISTER_MODEL_PLUGIN(ModelControlPlugin)
}
