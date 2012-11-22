/*
 * Copyright 2011 Nate Koenig
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

#include <sstream>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/physics/Physics.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ContactSensor.hh"

using namespace gazebo;
using namespace sensors;

transport::SubscriberPtr ContactSensor::contactSub = NULL;

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
void ContactSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  // Create a publisher for the contact information.
  if (this->sdf->HasElement("contact") &&
      this->sdf->GetElement("contact")->HasElement("topic") &&
      this->sdf->GetElement("contact")->GetValueString("topic")
      != "__default_topic__")
  {
    // This will create a topic based on the name specified in SDF.
    this->contactsPub = this->node->Advertise<msgs::Contacts>(
      this->sdf->GetElement("contact")->GetValueString("topic"));
  }
  else
  {
    // This will create a topic based on the name of the parent and the
    // name of the sensor.
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName() + "/imu";
    boost::replace_all(topicName, "::", "/");

    this->contactsPub = this->node->Advertise<msgs::Contacts>(topicName);
  }
}

//////////////////////////////////////////////////
void ContactSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  if (this->contactSub == NULL)
  {
    this->contactSub = this->node->Subscribe("~/physics/contacts",
        &ContactSensor::OnContacts, this);
  }

  /*physics::CollisionPtr collision;
  std::string collisionName;
  std::string collisionScopedName;

  sdf::ElementPtr collisionElem =
    this->sdf->GetElement("contact")->GetElement("collision");

  // Get all the collision elements
  while (collisionElem)
  {
    // get collision name
    collisionName = collisionElem->GetValueString();
    collisionScopedName =
      this->world->GetEntity(this->parentName)->GetScopedName();
    collisionScopedName += "::" + collisionName;
    collision = boost::shared_dynamic_cast<physics::Collision>(
        this->world->GetEntity(collisionScopedName));

    if (!collision)
    {
      gzerr << "Unable to find collision element["
            << collisionScopedName  << "]\n";
    }
    else
    {
      this->collisions.push_back(collision);
      this->connections.push_back(collision->ConnectContact(
            boost::bind(&ContactSensor::OnContact, this, _1, _2)));
    }
    collisionElem = collisionElem->GetNextElement("collision");
  }*/
}

//////////////////////////////////////////////////
void ContactSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void ContactSensor::UpdateImpl(bool /*_force*/)
{
/*  boost::mutex::scoped_lock lock(this->mutex);

  if (this->contactsPub && this->contactsPub->HasConnections() &&
      this->contacts.size() > 0)
  {
    this->contactsMsg.clear_contact();

    Contact_M::iterator iter;
    std::map<std::string, physics::Contact>::iterator iter2;

    // Iterate over all the contacts.
    for (iter = this->contacts.begin(); iter != this->contacts.end(); ++iter)
    {
      for (iter2 = iter->second.begin(); iter2 != iter->second.end(); ++iter2)
      {
        msgs::Contact *contactMsg = this->contactsMsg.add_contact();
        contactMsg->set_collision1(
            iter2->second.collision1->GetScopedName());
        contactMsg->set_collision2(
            iter2->second.collision2->GetScopedName());

        for (int i = 0; i < iter2->second.count; i++)
        {
          msgs::Set(contactMsg->add_position(), iter2->second.positions[i]);
          msgs::Set(contactMsg->add_normal(), iter2->second.normals[i]);
          contactMsg->add_depth(iter2->second.depths[i]);
        }
        msgs::Set(contactMsg->mutable_time(), iter2->second.time);
      }
    }

    this->contacts.clear();
    this->lastMeasurementTime = this->world->GetSimTime();
    this->contactsPub->Publish(this->contactsMsg);
  }
  */
}

//////////////////////////////////////////////////
void ContactSensor::Fini()
{
  Sensor::Fini();
  this->connections.clear();
}

//////////////////////////////////////////////////
unsigned int ContactSensor::GetCollisionCount() const
{
  return this->collisions.size();
}

//////////////////////////////////////////////////
std::string ContactSensor::GetCollisionName(unsigned int _index) const
{
  std::string result;

  if (_index < this->collisions.size())
    result = this->collisions[_index]->GetScopedName();

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
      std::advance(iter2, _index);
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
  boost::mutex::scoped_lock lock(this->mutex);
  this->contacts[_collisionName][_contact.collision2->GetName()] = _contact;
}

//////////////////////////////////////////////////
void ContactSensor::OnContacts(ConstContactsPtr &_msg)
{
  std::cout << "Got contact message\n";
}
