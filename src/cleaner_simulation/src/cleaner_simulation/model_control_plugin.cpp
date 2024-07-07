#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include "cleaner_simulation/TorqueTest.h"
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
        // 모델의 링크 출력
        auto links = _model->GetLinks();
        std::cout << "Model has " << links.size() << " links:" << std::endl;
        for (auto &link : links) {
            std::cout << " - Link Name: " << link->GetName() << std::endl;
        }
      // 모델의 조인트 출력
        auto joints = _model->GetJoints();
        std::cout << "Model has " << joints.size() << " joints:" << std::endl;
        for (auto &joint : joints) {
        std::cout << " - Joint Name: " << joint->GetName() << ", Type: " << joint->GetType() << std::endl;
        }
        if (!ros::isInitialized())
        {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "robot_cleaner", ros::init_options::NoSigintHandler);
        }
            // 모든 조인트 불러오기
        this->model = _model;
        double odometry_timer_interval = (1.0)/200;

        this->rosNode.reset(new ros::NodeHandle("robot_cleaner"));
        this->left_wheel = _model->GetJoint("wheel_rear_left_spin");
        this->right_wheel = _model->GetJoint("wheel_rear_right_spin");



        this->left_torque = 0;
        this->right_torque = 0;
        this->chassis = _model->GetLink("chassis");
        this->desired_velocity=3;
        // PID 파라미터 설정
        double p = 1.0, i = 0.05, d = 0.005;
        this->left_velocityPID.Init(p, i, d, 1.0, -1.0, 10, -10);
        this->right_velocityPID.Init(p, i, d, 1.0, -1.0, 10, -10);
        std::cout << " Joint Name: " << this->right_wheel->GetName() << ", Type: " << this->right_wheel->GetType() << std::endl;
        this->torque_Subscriber = this->rosNode->subscribe("/robot_control", 10, &ModelControlPlugin::OnTorqueMsg, this);
        this->pose_Publisher = this->rosNode->advertise<geometry_msgs::PoseStamped>("chassis_pose", 1000);
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


    void Odometry_update(const ros::TimerEvent&)
    {
      geometry_msgs::PoseStamped poseMsg;
      poseMsg.header.frame_id = "world";
      poseMsg.header.stamp = ros::Time::now();
      poseMsg.pose.position.x = this->chassis->WorldPose().Pos().X();
      poseMsg.pose.position.y = this->chassis->WorldPose().Pos().Y();
      poseMsg.pose.position.z = this->chassis->WorldPose().Pos().Z();
      poseMsg.pose.orientation.w = this->chassis->WorldPose().Rot().W();
      poseMsg.pose.orientation.x = this->chassis->WorldPose().Rot().X();
      poseMsg.pose.orientation.y = this->chassis->WorldPose().Rot().Y();
      poseMsg.pose.orientation.z = this->chassis->WorldPose().Rot().Z();
      pose_Publisher.publish(poseMsg);
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
        double left_error = left_angvel-this->desired_velocity;
        double right_error = right_angvel-this->desired_velocity;
        gzmsg << "left_error: " <<left_error  << ", right_error: " << right_error<<std::endl;

        this->left_torque = this->left_velocityPID.Update(left_error, 0.001);
        this->right_torque = this->right_velocityPID.Update(right_error, 0.001);
        // double left_torque_sign = std::copysign(1.0, this->left_torque); 
        // double right_torque_sign = std::copysign(1.0, this->right_torque); 
        // double left_angvel_sign = std::copysign(1.0, left_angvel); 
        // double right_angvel_sign = std::copysign(1.0, right_angvel); 
        // if (left_torque_sign*left_angvel_sign>0){
        //   double left_torque=max((-1/32.5)*((abs(left_angvel*60)/(2*M_PI))-77.5),0.0);
          // gzmsg << "left_torque1: " << this->left_torque <<std::endl;
        //   this->left_torque=min(left_torque,abs(this->left_torque));
        //   this->left_torque=(this->left_torque)*left_torque_sign;
        // }
        // else{
        //   this->left_torque=left_torque_sign*min(stall_torque,abs(this->left_torque));
        // }       
          gzmsg << "left_torque: " << this->left_torque  << ", left_angvel: " << left_angvel<<std::endl;

        // if (right_torque_sign*right_angvel_sign>0){
        //   double right_torque=max((-1/32.5)*((abs(right_angvel*60)/(2*M_PI))-77.5),0.0);
          // gzmsg << "right_torque1: " << this->right_torque <<std::endl;

        //   this->right_torque=min(right_torque,abs(this->right_torque));
        //   this->right_torque=(this->right_torque)*right_torque_sign;
        // }
        // else{
          // this->right_torque=right_torque_sign*min(stall_torque,abs(this->right_torque));
        // }        
        gzmsg << "right_torque: " << this->right_torque    << ", right_angvel: " << right_angvel<<std::endl;
        // this->left_wheel->SetVelocity(0,this->desired_velocity);
        // this->right_wheel->SetVelocity(0,this->desired_velocity);
        this->left_wheel->SetForce(0, this->left_torque);
        this->right_wheel->SetForce(0, this->right_torque);
        // wheeltorque
        wheelMsgs.effort.push_back(this->left_torque);
        wheelMsgs.effort.push_back(this->right_torque);

        wheelstate_Publisher.publish(wheelMsgs);

        // // // 로그에 상태 출력
        // gzmsg << "left_wheel_angle: " << normalized_left_angle  << ", left_angvel: " << left_angvel<<std::endl;
        // gzmsg << "right_wheel_angle: " << normalized_right_angle  << ", right_angvel: " << right_angvel<<std::endl;

    }
  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    physics::JointPtr left_wheel;
    physics::JointPtr right_wheel;
    physics::LinkPtr chassis;
    std::string right_wheel_jointName;
    std::string left_wheel_jointName;
    ros::Subscriber torque_Subscriber;
    ros::Publisher pose_Publisher;
    ros::Publisher wheelstate_Publisher;
    ros::Timer odometry_timer;
    common::PID left_velocityPID;
    common::PID right_velocityPID;
    double left_torque;
    double right_torque;
    double desired_velocity;
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelControlPlugin)
}
