/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Contact sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
*/

#include <float.h>
#include <sstream>

#include "msgs/msgs.h"

#include "common/Exception.hh"
#include "physics/Physics.hh"
#include "physics/World.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Collision.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/ContactSensor.hh"

#include "transport/Node.hh"

#include "math/Vector3.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("contact", ContactSensor)

//////////////////////////////////////////////////
ContactSensor::ContactSensor()
    : Sensor()
{
  this->node = transport::NodePtr(new transport::Node());
  this->mutex = new boost::mutex();
}

//////////////////////////////////////////////////
ContactSensor::~ContactSensor()
{
  this->collisions.clear();
  delete this->mutex;
}

//////////////////////////////////////////////////
void ContactSensor::Load(sdf::ElementPtr &_sdf)
{
  Sensor::Load(_sdf);

  if (this->sdf->GetElement("topic"))
  {
    std::string worldName = this->sdf->GetWorldName();
    this->node->Init(worldName);
    this->contactsPub = this->node->Advertise<msgs::Contacts>(
        this->sdf->GetElement("topic")->GetValueString());
  }
}

//////////////////////////////////////////////////
void ContactSensor::Load()
{
  Sensor::Load();

  physics::CollisionPtr collision;
  std::string collisionName;
  std::string collisionFullyScopedName;
  std::string linkName = this->sdf->GetLinkName();
  std::string modelName = this->sdf->GetModelName();
  std::string worldName = this->sdf->GetWorldName();

  physics::WorldPtr world = gazebo::physics::get_world(worldName);
  this->model = world->GetModelByName(modelName);

  sdf::ElementPtr collisionElem = 
    this->sdf->GetElement("contact")->GetElement("collision");

  // Get all the collision elements
  while (collisionElem)
  {
    // get collision name
    collisionName = collisionElem->GetValueString("name");
    collisionFullyScopedName = worldName + "::" + modelName + "::" + 
      linkName + "::" + collisionName;
    collision = this->model->GetChildCollision(collisionFullyScopedName);

    if (!collision)
    {
      gzerr << "Unable to find collision element[" 
            << collisionFullyScopedName  << "]\n";
    }
    else
    {
      this->collisions.push_back(collision);
      this->connections.push_back(collision->ConnectContact(
            boost::bind(&ContactSensor::OnContact, this, _1, _2)));
    }
    collisionElem = collisionElem->GetNextElement();
  }
}

//////////////////////////////////////////////////
void ContactSensor::Init()
{
  Sensor::Init();

  std::vector<physics::CollisionPtr>::iterator iter;
  for (iter = this->collisions.begin(); iter != this->collisions.end(); iter++)
  {
    (*iter)->SetContactsEnabled(true);
  }
}

//////////////////////////////////////////////////
void ContactSensor::UpdateImpl(bool /*_force*/)
{
  this->mutex->lock();
  if (this->contactsPub && this->contactsPub->HasConnections() && 
      this->contacts.size() > 0)
  {
    msgs::Contacts msg;

    Contact_M::iterator iter;
    std::map<std::string, physics::Contact>::iterator iter2;
    for (iter = this->contacts.begin(); iter != this->contacts.end(); iter++)
    {
      // Only transmit one contact
      for (iter2 = iter->second.begin(); iter2 != iter->second.end(); iter2++)
      {
        msgs::Contact *contactMsg = msg.add_contact();
        contactMsg->set_collision1(
            iter2->second.collision1->GetCompleteScopedName());
        contactMsg->set_collision2(
            iter2->second.collision2->GetCompleteScopedName());

        for (int i=0; i < iter2->second.count; i++)
        {
          msgs::Vector3d *posMsg = contactMsg->add_position();
          msgs::Vector3d *normalMsg = contactMsg->add_normal();
          posMsg->set_x(iter2->second.positions[i].x);
          posMsg->set_y(iter2->second.positions[i].y);
          posMsg->set_z(iter2->second.positions[i].z);

          normalMsg->set_x(iter2->second.normals[i].x);
          normalMsg->set_y(iter2->second.normals[i].y);
          normalMsg->set_z(iter2->second.normals[i].z);

          contactMsg->add_depth(iter2->second.depths[i]);
        }
      }
    }

    this->contactsPub->Publish(msg);
  }

  this->mutex->unlock();
}

//////////////////////////////////////////////////
void ContactSensor::Fini()
{
  Sensor::Fini();
}

//////////////////////////////////////////////////
unsigned int ContactSensor::GetCollisionCount() const
{
  return this->collisions.size();
}

//////////////////////////////////////////////////
/// Get a collision name
std::string ContactSensor::GetCollisionName(unsigned int _index) const
{
  std::string result;

  if (_index < this->collisions.size())
    result = this->collisions[_index]->GetName();

  return result;
}

//////////////////////////////////////////////////
unsigned int ContactSensor::GetCollisionContactCount(
    const std::string &_collisionName) const
{
  Contact_M::const_iterator iter = this->contacts.find(_collisionName);

  if (iter != this->contacts.end())
    return iter->second.size();
  else
    gzerr << "Contact Sensor[" << this->GetName() << "] has no collision[" 
          << _collisionName << "]\n";

  return 0;
}

//////////////////////////////////////////////////
physics::Contact ContactSensor::GetCollisionContact(
    const std::string &_collisionName, unsigned int _index) const
{
  Contact_M::const_iterator iter = this->contacts.find(_collisionName);

  if (iter != this->contacts.end())
  {
    if (_index < iter->second.size())
    {
      std::map<std::string, physics::Contact>::const_iterator iter2;
      iter2 = iter->second.begin();
      std::advance(iter2,_index);
      return iter2->second;
    }
    else
    {
      gzerr << "Invalid index[" << _index 
            << "] retreiving contact for collision[" << _collisionName 
            << "] in contact sensor[" << this->GetName() << "]\n";
    }
  }
  else
  {
    gzerr << "Contact Sensor[" << this->GetName() << "] has no collision[" 
          << _collisionName << "]\n";
  }

  return physics::Contact();
}

//////////////////////////////////////////////////
std::map<std::string, gazebo::physics::Contact> ContactSensor::GetContacts(
    const std::string &_collisionName)
{

  Contact_M::const_iterator iter = this->contacts.find(_collisionName);

  if (iter != this->contacts.end())
    return iter->second;
  else
    gzerr << "Contact Sensor[" << this->GetName() << "] has no collision[" 
          << _collisionName << "]\n";

  return std::map<std::string, gazebo::physics::Contact>();
}

//////////////////////////////////////////////////
void ContactSensor::OnContact(const std::string &_collisionName, 
                              const physics::Contact &_contact)
{
  this->mutex->lock();
  this->contacts[_collisionName][_contact.collision2->GetName()] = _contact;
  this->mutex->unlock();
}
