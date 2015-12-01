/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Pressure sensor plugin
 * Author: Steve Peters
 */
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/Base.hh>
#include "PressurePlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(PressurePlugin)

/////////////////////////////////////////////////
PressurePlugin::PressurePlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
PressurePlugin::~PressurePlugin()
{
}

/////////////////////////////////////////////////
void PressurePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "PressurePlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&PressurePlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // Get world name.
  this->worldName = this->parentSensor->WorldName();

  // Get name of parent sensor.
  this->parentSensorName = this->parentSensor->Name();

  // Get collision names of parent sensor and physics pointers.
  physics::WorldPtr world = physics::get_world(this->worldName);
  unsigned int collisionCount = this->parentSensor->GetCollisionCount();
  for (unsigned int i = 0; i < collisionCount; ++i)
  {
    std::string collisionScopedName = this->parentSensor->GetCollisionName(i);
    // Strip off ::collision_name to get link name
    std::string linkName = collisionScopedName.substr(0,
                           collisionScopedName.rfind("::"));
    // Get unscoped name of collision
    std::string collisionName =
      collisionScopedName.substr(collisionScopedName.rfind("::") + 2);
    // Get physics pointers
    physics::EntityPtr entity = world->GetEntity(linkName);
    if (entity && entity->HasType(physics::Base::LINK))
    {
      physics::LinkPtr link =
        boost::dynamic_pointer_cast<physics::Link>(entity);
      if (link)
      {
        physics::CollisionPtr collision = link->GetCollision(collisionName);
        if (collision)
        {
          physics::ShapePtr shape = collision->GetShape();
          if (shape->HasType(physics::Base::BOX_SHAPE))
          {
            physics::BoxShapePtr box =
              boost::dynamic_pointer_cast<physics::BoxShape>(shape);
            if (box)
            {
              math::Vector3 size = box->GetSize();
              std::vector<double> sizeVector;
              sizeVector.push_back(size.x);
              sizeVector.push_back(size.y);
              sizeVector.push_back(size.z);
              std::sort(sizeVector.begin(), sizeVector.end());
              double area = sizeVector[1] * sizeVector[2];
              if (area > 0.0)
                this->collisionNamesToArea[collisionScopedName] = area;
            }
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void PressurePlugin::Init()
{
  this->node.reset(new transport::Node());
  this->node->Init(this->worldName);

  if (!this->parentSensorName.empty())
  {
    // Create publisher for tactile messages
    std::string topicName = "~/" + this->parentSensorName + "/tactile";
    boost::replace_all(topicName, "::", "/");
    this->tactilePub = this->node->Advertise<msgs::Tactile>(topicName);
  }
}

/////////////////////////////////////////////////
void PressurePlugin::OnUpdate()
{
  msgs::Tactile tactileMsg;

  // For each collision attached to this sensor
  boost::unordered_map<std::string, double>::iterator iter;
  for (iter = this->collisionNamesToArea.begin();
       iter != this->collisionNamesToArea.end(); ++iter)
  {
    double normalForceSum = 0, normalForce;
    // Get the contacts sorted by collision element.
    std::map<std::string, gazebo::physics::Contact> contacts;
    std::map<std::string, gazebo::physics::Contact>::iterator iter2;
    contacts = this->parentSensor->Contacts(iter->first);
    for (iter2 = contacts.begin(); iter2 != contacts.end(); ++iter2)
    {
      for (int i = 0; i < iter2->second.count; ++i)
      {
        // TODO: determine whether body1Force or body2Force should be used.
        normalForce = iter2->second.normals[i].x *
                      iter2->second.wrench[i].body1Force.x +
                      iter2->second.normals[i].y *
                      iter2->second.wrench[i].body1Force.y +
                      iter2->second.normals[i].z *
                      iter2->second.wrench[i].body1Force.z;
        normalForceSum += normalForce;
      }
    }
    if (normalForceSum > 0)
    {
      tactileMsg.add_collision_name(iter->first);
      tactileMsg.add_collision_id(0);
      tactileMsg.add_pressure(normalForceSum / iter->second);
    }
  }

  msgs::Contacts contacts = this->parentSensor->Contacts();
  int nc = contacts.contact_size();
  if (nc > 0)
  {
    common::Time currentContactTime;
    currentContactTime = msgs::Convert(contacts.contact(nc-1).time());
    msgs::Set(tactileMsg.mutable_time(), currentContactTime);

    if (this->tactilePub && tactileMsg.pressure_size() > 0)
      this->tactilePub->Publish(tactileMsg);
  }
}
