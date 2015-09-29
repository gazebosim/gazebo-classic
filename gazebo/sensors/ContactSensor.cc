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
/* Desc: Contact sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
*/

#include <boost/algorithm/string.hpp>
#include <sstream>

#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ContactSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("contact", ContactSensor)

//////////////////////////////////////////////////
ContactSensor::ContactSensor()
: Sensor(sensors::OTHER)
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
      this->sdf->GetElement("contact")->Get<std::string>("topic")
      != "__default_topic__")
  {
    // This will create a topic based on the name specified in SDF.
    this->contactsPub = this->node->Advertise<msgs::Contacts>(
      this->sdf->GetElement("contact")->Get<std::string>("topic"), 100);
  }
  else
  {
    // This will create a topic based on the name of the parent and the
    // name of the sensor.
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName();
    boost::replace_all(topicName, "::", "/");

    this->contactsPub = this->node->Advertise<msgs::Contacts>(topicName, 100);
  }
}

//////////////////////////////////////////////////
void ContactSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  std::string collisionName;
  std::string collisionScopedName;

  sdf::ElementPtr collisionElem =
    this->sdf->GetElement("contact")->GetElement("collision");

  std::string entityName =
      this->world->GetEntity(this->parentName)->GetScopedName();
  std::string filterName = entityName + "::" + this->GetName();

  // Get all the collision elements
  while (collisionElem)
  {
    // get collision name
    collisionName = collisionElem->Get<std::string>();
    collisionScopedName = entityName;
    collisionScopedName += "::" + collisionName;

    this->collisions.push_back(collisionScopedName);

    collisionElem = collisionElem->GetNextElement("collision");
  }

  if (!this->collisions.empty())
  {
    // request the contact manager to publish messages to a custom topic for
    // this sensor
    physics::ContactManager *mgr =
        this->world->GetPhysicsEngine()->GetContactManager();
    std::string topic = mgr->CreateFilter(filterName, this->collisions);
    if (!this->contactSub)
    {
      this->contactSub = this->node->Subscribe(topic,
          &ContactSensor::OnContacts, this);
    }
  }
}

//////////////////////////////////////////////////
void ContactSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool ContactSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Don't do anything if there is no new data to process.
  if (this->incomingContacts.empty())
    return false;

  std::vector<std::string>::iterator collIter;
  std::string collision1;

  // Clear the outgoing contact message.
  this->contactsMsg.clear_contact();

  // Iterate over all the contact messages
  for (ContactMsgs_L::iterator iter = this->incomingContacts.begin();
      iter != this->incomingContacts.end(); ++iter)
  {
    // Iterate over all the contacts in the message
    for (int i = 0; i < (*iter)->contact_size(); ++i)
    {
      collision1 = (*iter)->contact(i).collision1();

      // Try to find the first collision's name
      collIter = std::find(this->collisions.begin(),
          this->collisions.end(), collision1);

      // If unable to find the first collision's name, try the second
      if (collIter == this->collisions.end())
      {
        collision1 = (*iter)->contact(i).collision2();
        collIter = std::find(this->collisions.begin(),
            this->collisions.end(), collision1);
      }

      // If this sensor is monitoring one of the collision's in the
      // contact, then add the contact to our outgoing message.
      if (collIter != this->collisions.end())
      {
        int count = (*iter)->contact(i).position_size();

        // Check to see if the contact arrays all have the same size.
        if (count != (*iter)->contact(i).normal_size() ||
            count != (*iter)->contact(i).wrench_size() ||
            count != (*iter)->contact(i).depth_size())
        {
          gzerr << "Contact message has invalid array sizes\n";
          continue;
        }

        // Copy the contact message.
        msgs::Contact *contactMsg = this->contactsMsg.add_contact();
        contactMsg->CopyFrom((*iter)->contact(i));
      }
    }
  }

  // Clear the incoming contact list.
  this->incomingContacts.clear();

  this->lastMeasurementTime = this->world->GetSimTime();

  // Generate a outgoing message only if someone is listening.
  if (this->contactsPub && this->contactsPub->HasConnections())
  {
    msgs::Set(this->contactsMsg.mutable_time(), this->lastMeasurementTime);
    this->contactsPub->Publish(this->contactsMsg);
  }

  return true;
}

//////////////////////////////////////////////////
void ContactSensor::Fini()
{
  if (this->world && this->world->GetRunning())
  {
    std::string entityName =
        this->world->GetEntity(this->parentName)->GetScopedName();
    std::string filterName = entityName + "::" + this->GetName();

    physics::ContactManager *mgr =
        this->world->GetPhysicsEngine()->GetContactManager();
    mgr->RemoveFilter(filterName);
  }

  this->contactSub.reset();
  this->contactsPub.reset();
  Sensor::Fini();
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
    result = this->collisions[_index];

  return result;
}

//////////////////////////////////////////////////
unsigned int ContactSensor::GetCollisionContactCount(
    const std::string &_collisionName) const
{
  boost::mutex::scoped_lock lock(this->mutex);
  unsigned int result = 0;

  for (int i = 0; i < this->contactsMsg.contact_size(); ++i)
  {
    if (this->contactsMsg.contact(i).collision1() == _collisionName ||
        this->contactsMsg.contact(i).collision2() == _collisionName)
    {
      result += this->contactsMsg.contact(i).position_size();
    }
  }

  return result;
}

//////////////////////////////////////////////////
msgs::Contacts ContactSensor::GetContacts() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->contactsMsg;
}

//////////////////////////////////////////////////
std::map<std::string, gazebo::physics::Contact> ContactSensor::GetContacts(
    const std::string &_collisionName)
{
  boost::mutex::scoped_lock lock(this->mutex);

  std::map<std::string, gazebo::physics::Contact> result;

  std::string collision2;

  for (int i = 0; i < this->contactsMsg.contact_size(); ++i)
  {
    collision2.clear();

    if (this->contactsMsg.contact(i).collision1() == _collisionName)
      collision2 = this->contactsMsg.contact(i).collision2();
    else if (this->contactsMsg.contact(i).collision2() == _collisionName)
      collision2 =  this->contactsMsg.contact(i).collision1();

    if (collision2.empty())
      continue;

    result[collision2] = this->contactsMsg.contact(i);
  }

  return result;
}

//////////////////////////////////////////////////
void ContactSensor::OnContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Only store information if the sensor is active
  if (this->IsActive())
  {
    // Store the contacts message for processing in UpdateImpl
    this->incomingContacts.push_back(_msg);

    // Prevent the incomingContacts list to grow indefinitely.
    if (this->incomingContacts.size() > 100)
      this->incomingContacts.pop_front();
  }
}

//////////////////////////////////////////////////
bool ContactSensor::IsActive()
{
  return this->active ||
         (this->contactsPub && this->contactsPub->HasConnections());
}
