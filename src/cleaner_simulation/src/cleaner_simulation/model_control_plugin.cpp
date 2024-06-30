#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
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

        this->right_wheel = _model->GetJoint("wheel_rear_right_spin");


        this->left_wheel = _model->GetJoint("wheel_rear_left_spin");
        this->left_torque = 0;
        this->right_torque = 0;
        this->chassis = _model->GetLink("chassis");
        

        std::cout << " Joint Name: " << this->right_wheel->GetName() << ", Type: " << this->right_wheel->GetType() << std::endl;
        this->torque_Subscriber = this->rosNode->subscribe("/robot_control", 10, &ModelControlPlugin::OnRosMsg, this);
        this->pose_Publisher = this->rosNode->advertise<geometry_msgs::PoseStamped>("chassis_pose", 1000);
        this->wheelstate_Publisher = this->rosNode->advertise<sensor_msgs::JointState>("wheel_state", 1000);
        odometry_timer = rosNode->createTimer(ros::Duration(odometry_timer_interval), &ModelControlPlugin::Odometry_update, this);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelControlPlugin::OnWheelState, this));
        ROS_INFO("RobotControlPlugin loaded");


    }

    void OnRosMsg(const cleaner_simulation::TorqueTestConstPtr &msg)
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
 

        double left_torque_sign = std::copysign(1.0, this->left_torque); 
        double right_torque_sign = std::copysign(1.0, this->right_torque); 
        if (abs(this->left_torque)>0.0){
          double left_torque=max((-1/32.5)*((abs(left_angvel*60)/(2*M_PI))-77.5),0.0);

          this->left_torque=min(left_torque,abs(this->left_torque));

          this->left_torque=this->left_torque*left_torque_sign;
        }
        if (abs(this->right_torque)>0.0){
          double right_torque=max((-1/32.5)*((abs(right_angvel*60)/(2*M_PI))-77.5),0.0);
          this->right_torque=min(right_torque,abs(this->right_torque));
          this->right_torque=this->right_torque*right_torque_sign;
        }
        // gzmsg << "left_torque: " << this->left_torque  << ", left_angvel: " << left_angvel<<std::endl;
        // gzmsg << "right_torque: " << this->right_torque    << ", right_angvel: " << right_angvel<<std::endl;

        this->left_wheel->SetForce(0, this->left_torque);
        this->right_wheel->SetForce(0, this->right_torque);
        // wheeltorque
        wheelMsgs.effort.push_back(this->left_torque);
        wheelMsgs.effort.push_back(this->right_torque);

        wheelstate_Publisher.publish(wheelMsgs);

        // // 로그에 상태 출력
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
    double left_torque;
    double right_torque;
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelControlPlugin)
}
