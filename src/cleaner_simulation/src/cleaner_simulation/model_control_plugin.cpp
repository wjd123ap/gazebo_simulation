#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cleaner_simulation/TorqueTest.h"
#include <cmath> 
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
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
        }
            // 모든 조인트 불러오기
        this->model = _model;
        double timer_interval = 0.05;
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        this->right_wheel = _model->GetJoint("wheel_rear_right_spin");


        this->left_wheel = _model->GetJoint("wheel_rear_left_spin");


        // this->left_wheel = this->model->GetJoint(this->left_wheel_jointName);
        // this->right_wheel = this->model->GetJoint(this->right_wheel_jointName);
        std::cout << " Joint Name: " << this->right_wheel->GetName() << ", Type: " << this->right_wheel->GetType() << std::endl;
        this->Sub = this->rosNode->subscribe("/robot_control", 10, &ModelControlPlugin::OnRosMsg, this);
        timer = rosNode->createTimer(ros::Duration(timer_interval), &ModelControlPlugin::OnTimer, this);
        ROS_INFO("RobotControlPlugin loaded");


    }

    void OnRosMsg(const cleaner_simulation::TorqueTestConstPtr &msg)
    {
      
        // ROS_INFO("MOVE");
        this->left_wheel->SetForce(0, msg->left_torque);
        this->right_wheel->SetForce(0, msg->right_torque);
        

    }


    void OnTimer(const ros::TimerEvent&)
    {
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
        // 로그에 상태 출력
        gzmsg << "left_wheel_angle: " << normalized_left_angle  << ", left_angvel: " << left_angvel<<std::endl;
        gzmsg << "right_wheel_angle: " << normalized_right_angle  << ", right_angvel: " << right_angvel<<std::endl;

    }

  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    physics::JointPtr left_wheel;
    physics::JointPtr right_wheel;
    std::string right_wheel_jointName;
    std::string left_wheel_jointName;
    ros::Subscriber Sub;
    ros::Timer timer;
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelControlPlugin)
}
