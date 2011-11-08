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

#include "common/Exception.hh"
#include "physics/Physics.hh"
#include "physics/World.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Collision.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/ContactSensor.hh"

#include "math/Vector3.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("contact", ContactSensor)

//////////////////////////////////////////////////
ContactSensor::ContactSensor()
    : Sensor()
{
}

//////////////////////////////////////////////////
ContactSensor::~ContactSensor()
{
  this->collisions.clear();
}

//////////////////////////////////////////////////
void ContactSensor::Load(sdf::ElementPtr &_sdf)
{
  Sensor::Load(_sdf);
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
    collisionElem = this->sdf->GetElement("contact")->GetNextElement(
        "collision", collisionElem);
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
  this->contacts.clear();
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
      return iter->second[_index];
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
std::vector<gazebo::physics::Contact> ContactSensor::GetContacts(
    const std::string &_collisionName)
{
  Contact_M::const_iterator iter = this->contacts.find(_collisionName);

  if (iter != this->contacts.end())
    return iter->second;
  else
    gzerr << "Contact Sensor[" << this->GetName() << "] has no collision[" 
          << _collisionName << "]\n";

  return std::vector<gazebo::physics::Contact>();
}

//////////////////////////////////////////////////
void ContactSensor::OnContact(const std::string &_collisionName, 
                              const physics::Contact &_contact)
{
  this->contacts[_collisionName].push_back(_contact);
}
