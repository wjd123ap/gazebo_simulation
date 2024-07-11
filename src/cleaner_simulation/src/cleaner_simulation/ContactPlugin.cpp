#include "cleaner_simulation/ContactPlugin.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)


ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

ContactPlugin::~ContactPlugin()
{
}

void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }
  if (_sdf->GetParent()->HasAttribute("name"))
  {
    // 'name' 속성의 값을 읽음
    this->topicName = _sdf->GetParent()->GetAttribute("name")->GetAsString();
  }
  if (_sdf->HasElement("updateRateHZ")){
          this->updateRateHZ=_sdf->Get<double>("updateRateHZ");
  }
  double contact_timer_interval = (1.0)/(this->updateRateHZ);

  this->rosNode.reset(new ros::NodeHandle("contact"));
  this->contact_Publisher = this->rosNode->advertise<cleaner_simulation::Contacts>(this->topicName, 1000);
  this->contact_timer = rosNode->createTimer(ros::Duration(contact_timer_interval), &ContactPlugin::ContactUpdate, this);
  // Connect to the sensor update event.
  // this->updateConnection = this->parentSensor->ConnectUpdated(
  //     std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

void ContactPlugin::ContactUpdate(const ros::TimerEvent&){
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  cleaner_simulation::Contacts contacts_msg;
  contacts_msg.num=contacts.contact_size();
  
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    cleaner_simulation::Contact contact_msg;
    contact_msg.collision1=contacts.contact(i).collision1();
    contact_msg.collision2=contacts.contact(i).collision2();
    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      
      geometry_msgs::Point contact_pos;
      contact_pos.x = contacts.contact(i).position(j).x();
      contact_pos.y = contacts.contact(i).position(j).y();
      contact_pos.z = contacts.contact(i).position(j).z();
      contact_msg.position.push_back(contact_pos);
      geometry_msgs::Point contact_normal;
      contact_normal.x = contacts.contact(i).normal(j).x();
      contact_normal.y = contacts.contact(i).normal(j).y();
      contact_normal.z = contacts.contact(i).normal(j).z();
      contact_msg.normal.push_back(contact_normal);
      geometry_msgs::Point contact_force1;
      contact_force1.x = contacts.contact(i).wrench(j).body_1_wrench().force().x();
      contact_force1.y = contacts.contact(i).wrench(j).body_1_wrench().force().y();
      contact_force1.z = contacts.contact(i).wrench(j).body_1_wrench().force().z();
      contact_msg.force1.push_back(contact_force1);
      geometry_msgs::Point contact_force2;
      contact_force2.x = contacts.contact(i).wrench(j).body_2_wrench().force().x();
      contact_force2.y = contacts.contact(i).wrench(j).body_2_wrench().force().y();
      contact_force2.z = contacts.contact(i).wrench(j).body_2_wrench().force().z();
      contact_msg.force2.push_back(contact_force2);
      contact_msg.depth.push_back(contacts.contact(i).depth(j));
    }
    contacts_msg.contact.push_back(contact_msg);
  }
  this->contact_Publisher.publish(contacts_msg);
}

void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    gzmsg << "Collision between[" << contacts.contact(i).collision1()//parent
              << "] and [" << contacts.contact(i).collision2() << "]\n";//child

    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      gzmsg << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << std::endl;;
      gzmsg << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << std::endl;

       gzmsg << "force:" << contacts.contact(i).wrench(j).body_1_wrench().force().x() << " "
        <<contacts.contact(i).wrench(j).body_1_wrench().force().y() << " " 
         <<contacts.contact(i).wrench(j).body_1_wrench().force().z() << std::endl;
      // std::cout << "   Depth:" << contacts.contact(i).depth(j) << std::endl;
    }
  }
}