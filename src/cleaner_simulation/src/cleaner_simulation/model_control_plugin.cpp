#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "cleaner_simulation/TorqueTest.h"
#include "cleaner_simulation/WheelVel.h"
#include <cmath>
#include <algorithm>
#include <sensor_msgs/JointState.h>
using namespace std;
double stall_torque=(77.5/32.5);
namespace gazebo
{
  class ModelControlPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        ROS_INFO("ModelPlugin for %s loaded successfully!", _model->GetName().c_str());
      //   // 모델의 링크 출력
      //   auto links = _model->GetLinks();
      //   std::cout << "Model has " << links.size() << " links:" << std::endl;
      //   for (auto &link : links) {
      //       std::cout << " - Link Name: " << link->GetName() << std::endl;
      //   }
      // // 모델의 조인트 출력
      //   auto joints = _model->GetJoints();
      //   std::cout << "Model has " << joints.size() << " joints:" << std::endl;
      //   for (auto &joint : joints) {
      //   std::cout << " - Joint Name: " << joint->GetName() << ", Type: " << joint->GetType() << std::endl;
      //   }
        if (!ros::isInitialized())
        {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "robot_cleaner", ros::init_options::NoSigintHandler);
        }

        physics::WorldPtr world = _model->GetWorld();


        physics::PhysicsEnginePtr physics = world->Physics();

        stepsize = physics->GetMaxStepSize();
        if (_sdf->HasElement("wheel_p_gain")){
          this->wheel_p=_sdf->Get<double>("wheel_p_gain");
        }
        if (_sdf->HasElement("wheel_i_gain")){
          this->wheel_i=_sdf->Get<double>("wheel_i_gain");
        }
        if (_sdf->HasElement("wheel_d_gain")){
          this->wheel_d=_sdf->Get<double>("wheel_d_gain");
        }
        this->model = _model;
        double odometry_timer_interval = (1.0)/200;

        this->rosNode.reset(new ros::NodeHandle("robot_cleaner"));
        this->left_wheel = _model->GetJoint("wheel_rear_left_spin");
        this->right_wheel = _model->GetJoint("wheel_rear_right_spin");
        this->left_wheel->SetProvideFeedback(true);
        this->right_wheel->SetProvideFeedback(true); 
        this->left_rear_wheel_link= _model->GetLink("wheel_rear_left");
        this->right_rear_wheel_link= _model->GetLink("wheel_rear_right");
        this->left_front_wheel_link= _model->GetLink("front_wheel_left");
        this->right_front_wheel_link= _model->GetLink("front_wheel_right");

        this->left_torque = 0;
        this->right_torque = 0;
        this->desired_left_angvel = 0;
        this->desired_right_angvel = 0;
        this->chassis = _model->GetLink("chassis");


        this->left_velocityPID.Init(this->wheel_p, this->wheel_i, this->wheel_d, 1.0, -1.0, stall_torque, -stall_torque);
        this->right_velocityPID.Init(this->wheel_p, this->wheel_i, this->wheel_d, 1.0, -1.0, stall_torque, -stall_torque);
        std::cout << " Joint Name: " << this->right_wheel->GetName() << ", Type: " << this->right_wheel->GetType() << std::endl;
        this->torque_Subscriber = this->rosNode->subscribe("/robot_control", 10, &ModelControlPlugin::OnTorqueMsg, this);
        this->wheelvel_Subscriber = this->rosNode->subscribe("/wheelvel_control", 10, &ModelControlPlugin::OnWheelVelMsg, this);
        this->imu_Subscriber = this->rosNode->subscribe("/robot_cleaner/imu", 10, &ModelControlPlugin::OnImuMsg, this);       
        this->pose_Publisher = this->rosNode->advertise<nav_msgs::Odometry>("chassis_pose", 1000);
        
        this->wheelstate_Publisher = this->rosNode->advertise<sensor_msgs::JointState>("wheel_state", 1000);
        odometry_timer = rosNode->createTimer(ros::Duration(odometry_timer_interval), &ModelControlPlugin::Odometry_update, this);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelControlPlugin::OnWheelState, this));
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
    }

    void Odometry_update(const ros::TimerEvent&)
    {
      nav_msgs::Odometry odometry;
      odometry.header.frame_id = "world";
      odometry.header.stamp = ros::Time::now();
      odometry.pose.pose.position.x = this->chassis->WorldPose().Pos().X();
      odometry.pose.pose.position.y = this->chassis->WorldPose().Pos().Y();
      odometry.pose.pose.position.z = this->chassis->WorldPose().Pos().Z();
      odometry.pose.pose.orientation.w = this->chassis->WorldPose().Rot().W();
      odometry.pose.pose.orientation.x = this->chassis->WorldPose().Rot().X();
      odometry.pose.pose.orientation.y = this->chassis->WorldPose().Rot().Y();
      odometry.pose.pose.orientation.z = this->chassis->WorldPose().Rot().Z();
      odometry.twist.twist.angular.x = this->chassis_angvel[0];
      odometry.twist.twist.angular.y = this->chassis_angvel[1];
      odometry.twist.twist.angular.z = this->chassis_angvel[2];
      odometry.twist.twist.linear.x = this->chassis->WorldLinearVel().X();
      odometry.twist.twist.linear.y = this->chassis->WorldLinearVel().Y();
      odometry.twist.twist.linear.z = this->chassis->WorldLinearVel().Z();
      pose_Publisher.publish(odometry);
    }
    
    void OnWheelState()
    {

        sensor_msgs::JointState wheelMsgs;
        // 조인트 상태 읽기
        double left_angle = this->left_wheel->Position(0); // 조인트 각도 (라디안)
        double normalized_left_angle = std::fmod(left_angle, 2 * M_PI);
        if (normalized_left_angle < 0) {
            normalized_left_angle += 2 * M_PI;  // 음수 각도를 양수로 조정
        }
        double left_angvel = this->left_wheel->GetVelocity(0); // 조인트 각속도 (라디안/초)

        double right_angle = this->right_wheel->Position(0); // 조인트 각도 (라디안)
        double normalized_right_angle = std::fmod(right_angle, 2 * M_PI);
        if (normalized_right_angle < 0) {
            normalized_right_angle += 2 * M_PI;  // 음수 각도를 양수로 조정
        }
        double right_angvel = this->right_wheel->GetVelocity(0); // 조인트 각속도 (라디안/초)
        double right_measure_torque =this->right_wheel->GetForce(0);

        wheelMsgs.header.stamp = ros::Time::now();
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
        double left_error = left_angvel - this->desired_left_angvel;
        double right_error = right_angvel - this->desired_right_angvel;
        // gzmsg << "left_error: " <<left_error  << ", right_error: " << right_error<<std::endl;

        this->left_torque = this -> left_velocityPID.Update(left_error, this->stepsize);
        this->right_torque = this -> right_velocityPID.Update(right_error, this->stepsize);
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
        // gzmsg << "right_torque: " << this->right_torque    << ", right_angvel: " << right_angvel<<std::endl;
        // ignition::math::Vector3d left_link_torque(0, 0, this->left_torque);
        // ignition::math::Vector3d right_link_torque(0, 0, this->right_torque);
        // this->left_wheel_link->AddRelativeTorque(left_link_torque);
        // this->right_wheel_link->AddRelativeTorque(right_link_torque);        
        this->left_wheel->SetForce(0, this->left_torque);
        this->right_wheel->SetForce(0, this->right_torque);
        wheelMsgs.effort.push_back(this->left_torque);
        wheelMsgs.effort.push_back(this->right_torque);

        wheelstate_Publisher.publish(wheelMsgs);
        // gzmsg << "right_measure_torque: " << right_measure_torque<< ",desired_right_torque: " << this->right_torque<<std::endl;
        // 로그에 상태 출력
        // gzmsg << "left_wheel_angle: " << normalized_left_angle  << ", left_angvel: " << left_angvel<<std::endl;
        // gzmsg << "right_wheel_angle: " << normalized_right_angle  << ", right_angvel: " << right_angvel<<std::endl;
        ignition::math::Vector3d left_wheel_jointforce = this->left_wheel->LinkForce(0);
        ignition::math::Vector3d right_wheel_jointforce = this->right_wheel->LinkForce(0);        
        // auto force1 = this->left_rear_wheel_link->WorldForce();
        // auto force2 = this->right_rear_wheel_link->WorldForce();
        // auto force3 = this->chassis->WorldForce();
        // gazebo::physics::Collision_V collisions = this->right_rear_wheel_link->GetCollisions();
        // std::cout << "collisions.size(): " << collisions[0]->GetModel()->GetName() << std::endl;
        // std::cout << "collisions.size(): " << collisions.size() << std::endl;
        std::cout << "left_wheel_jointforce: " << left_wheel_jointforce << std::endl;
        std::cout << "right_wheel_jointforce: " << right_wheel_jointforce << std::endl;
  
    }
  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
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
    common::PID left_velocityPID;
    common::PID right_velocityPID;
    double left_torque;
    double right_torque;
    double chassis_angvel[3];

    double desired_left_angvel;
    double desired_right_angvel;
    double wheel_p;
    double wheel_i;
    double wheel_d;
    double stepsize;
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelControlPlugin)
}
