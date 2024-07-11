#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include "cleaner_simulation/Contacts.h"
namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {

  public: 
    ContactPlugin();
    virtual ~ContactPlugin();
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    void ContactUpdate(const ros::TimerEvent&);
  private:
    std::unique_ptr<ros::NodeHandle> rosNode;
    std::string topicName;
    virtual void OnUpdate();
    sensors::ContactSensorPtr parentSensor;
    event::ConnectionPtr updateConnection;
    ros::Publisher contact_Publisher;
    ros::Timer contact_timer;
    double updateRateHZ;
  };
}
#endif