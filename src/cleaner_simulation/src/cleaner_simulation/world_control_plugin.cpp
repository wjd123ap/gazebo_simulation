#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>

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
        ros::init(argc, argv, "gazebo_world", ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("gazebo_world"));

      ros::Subscriber sub = this->rosNode->subscribe("/world_control", 10, &WorldControlPlugin::OnRosMsg, this);
      ROS_INFO("WorldControlPlugin loaded");
      // 모델 이름 출력
      auto models = this->world->Models();
      ROS_INFO("Models in world:");
      for (const auto& model : models)
      {
        ROS_INFO("%s", model->GetName().c_str());
      }
      transport::NodePtr node(new transport::Node());
      node->Init(_world->Name());
      transport::PublisherPtr physicsPub =
        node->Advertise<msgs::Physics>("~/physics");
      msgs::Physics physicsMsg;
      physicsMsg.set_solver_type("quick");
      physicsMsg.set_iters(400);
      physicsMsg.set_sor(0.1);
      physicsMsg.set_erp(0.01);
      physicsMsg.set_real_time_factor(1.0);
      physicsMsg.set_max_step_size(0.0002);
      physicsMsg.set_real_time_update_rate(5000);
      physicsMsg.set_contact_surface_layer(0.001);

      physicsPub->Publish(physicsMsg);


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