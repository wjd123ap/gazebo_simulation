#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cleaner_simulation/TorqueTest.h"
namespace gazebo
{
  class WorldControlPlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      ros::Subscriber sub = this->rosNode->subscribe("/world_control", 10, &WorldControlPlugin::OnRosMsg, this);
      ROS_INFO("WorldControlPlugin loaded");
      // 모델 이름 출력
      auto models = this->world->Models();
      ROS_INFO("Models in world:");
      for (const auto& model : models)
      {
        ROS_INFO("%s", model->GetName().c_str());
      }
    }

    void OnRosMsg(const std_msgs::StringConstPtr &msg)
    {
      if (msg->data == "pause")
        this->world->SetPaused(true);
      else if (msg->data == "unpause")
        this->world->SetPaused(false);
    }

  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    physics::WorldPtr world;
  };

  GZ_REGISTER_WORLD_PLUGIN(WorldControlPlugin)
}