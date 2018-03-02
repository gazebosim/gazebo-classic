/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

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
#include "gazebo/sensors/ContactSensorPrivate.hh"
#include "gazebo/sensors/ContactSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("contact", ContactSensor)

// Deprecated. In gazebo10 replace this with a member variable.
std::map<std::string, std::string> gFilterTopics;

//////////////////////////////////////////////////
ContactSensor::ContactSensor()
: Sensor(sensors::OTHER),
  dataPtr(new ContactSensorPrivate)
{
}

//////////////////////////////////////////////////
ContactSensor::~ContactSensor()
{
  this->dataPtr->collisions.clear();
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
    this->dataPtr->contactsPub = this->node->Advertise<msgs::Contacts>(
      this->sdf->GetElement("contact")->Get<std::string>("topic"),
      100);
  }
  else
  {
    // This will create a topic based on the name of the parent and the
    // name of the sensor.
    std::string topicName = "~/";
    topicName += this->ParentName() + "/" + this->Name();
    boost::replace_all(topicName, "::", "/");

    this->dataPtr->contactsPub =
      this->node->Advertise<msgs::Contacts>(topicName, 100);
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
      this->world->GetEntity(this->ParentName())->GetScopedName();
  this->dataPtr->filterName = entityName + "::" + this->Name();

  // Get all the collision elements
  while (collisionElem)
  {
    // get collision name
    collisionName = collisionElem->Get<std::string>();
    collisionScopedName = entityName;
    collisionScopedName += "::" + collisionName;

    this->dataPtr->collisions.push_back(collisionScopedName);

    collisionElem = collisionElem->GetNextElement("collision");
  }

  if (!this->dataPtr->collisions.empty())
  {
    // request the contact manager to publish messages to a custom topic for
    // this sensor
    physics::ContactManager *mgr =
        this->world->GetPhysicsEngine()->GetContactManager();
    gFilterTopics[this->dataPtr->filterName] =
      mgr->CreateFilter(this->dataPtr->filterName, this->dataPtr->collisions);
    // Disable transport in case there are no collisions
    // Create it if it's active, but we only added the collisions now
    this->SetActive(this->IsActive() && !this->dataPtr->collisions.empty());
  }
}

//////////////////////////////////////////////////
void ContactSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
// Warning: The point of this function is to delay subscription until
// the sensor is active. It relies on the ContactManager::CreateFilter
// function establishing a publisher. In Gazebo9 the ContactManager logic
// has changed. Be careful when merging this class forward.
void ContactSensor::SetActiveContactSensor(const bool _value)
{
  // Need collisions for filter
  if (_value && !this->dataPtr->collisions.empty() &&
      !this->dataPtr->contactSub)
  {
    // Subscribe to contact info
    this->dataPtr->contactSub = this->node->Subscribe(
        gFilterTopics[this->dataPtr->filterName],
        &ContactSensor::OnContacts, this);
  }
  else if (!_value && this->dataPtr->contactSub)
  {
    this->dataPtr->contactSub.reset();
  }
}

//////////////////////////////////////////////////
bool ContactSensor::UpdateImpl(const bool /*_force*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->contactSub && (
        this->active || (this->dataPtr->contactsPub &&
        this->dataPtr->contactsPub->HasConnections())))
  {
    this->SetActiveContactSensor(true);
  }

  // Don't do anything if there is no new data to process.
  if (this->dataPtr->incomingContacts.empty())
    return false;

  std::vector<std::string>::iterator collIter;
  std::string collision1;

  // Clear the outgoing contact message.
  this->dataPtr->contactsMsg.clear_contact();

  // Iterate over all the contact messages
  for (auto iter = this->dataPtr->incomingContacts.begin();
       iter != this->dataPtr->incomingContacts.end(); ++iter)
  {
    // Iterate over all the contacts in the message
    for (int i = 0; i < (*iter)->contact_size(); ++i)
    {
      collision1 = (*iter)->contact(i).collision1();

      // Try to find the first collision's name
      collIter = std::find(this->dataPtr->collisions.begin(),
          this->dataPtr->collisions.end(), collision1);

      // If unable to find the first collision's name, try the second
      if (collIter == this->dataPtr->collisions.end())
      {
        collision1 = (*iter)->contact(i).collision2();
        collIter = std::find(this->dataPtr->collisions.begin(),
            this->dataPtr->collisions.end(), collision1);
      }

      // If this sensor is monitoring one of the collision's in the
      // contact, then add the contact to our outgoing message.
      if (collIter != this->dataPtr->collisions.end())
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
        msgs::Contact *contactMsg = this->dataPtr->contactsMsg.add_contact();
        contactMsg->CopyFrom((*iter)->contact(i));
      }
    }
  }

  // Clear the incoming contact list.
  this->dataPtr->incomingContacts.clear();

  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->dataPtr->contactsMsg.mutable_time(),
            this->lastMeasurementTime);

  // Generate a outgoing message only if someone is listening.
  if (this->dataPtr->contactsPub &&
      this->dataPtr->contactsPub->HasConnections())
  {
    this->dataPtr->contactsPub->Publish(this->dataPtr->contactsMsg);
  }

  return true;
}

//////////////////////////////////////////////////
void ContactSensor::Fini()
{
  if (this->world && this->world->GetRunning())
  {
    physics::ContactManager *mgr =
        this->world->GetPhysicsEngine()->GetContactManager();
    mgr->RemoveFilter(this->dataPtr->filterName);
  }

  this->dataPtr->contactSub.reset();
  this->dataPtr->contactsPub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
unsigned int ContactSensor::GetCollisionCount() const
{
  return this->dataPtr->collisions.size();
}

//////////////////////////////////////////////////
std::string ContactSensor::GetCollisionName(unsigned int _index) const
{
  std::string result;

  if (_index < this->dataPtr->collisions.size())
    result = this->dataPtr->collisions[_index];

  return result;
}

//////////////////////////////////////////////////
unsigned int ContactSensor::GetCollisionContactCount(
    const std::string &_collisionName) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  unsigned int result = 0;

  for (int i = 0; i < this->dataPtr->contactsMsg.contact_size(); ++i)
  {
    if (this->dataPtr->contactsMsg.contact(i).collision1() == _collisionName ||
        this->dataPtr->contactsMsg.contact(i).collision2() == _collisionName)
    {
      result += this->dataPtr->contactsMsg.contact(i).position_size();
    }
  }

  return result;
}

//////////////////////////////////////////////////
msgs::Contacts ContactSensor::GetContacts() const
{
  return this->Contacts();
}

//////////////////////////////////////////////////
msgs::Contacts ContactSensor::Contacts() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->contactsMsg;
}

//////////////////////////////////////////////////
std::map<std::string, gazebo::physics::Contact> ContactSensor::GetContacts(
    const std::string &_collisionName)
{
  return this->Contacts(_collisionName);
}

//////////////////////////////////////////////////
std::map<std::string, gazebo::physics::Contact> ContactSensor::Contacts(
    const std::string &_collisionName) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  std::map<std::string, gazebo::physics::Contact> result;

  std::string collision2;

  for (int i = 0; i < this->dataPtr->contactsMsg.contact_size(); ++i)
  {
    collision2.clear();

    if (this->dataPtr->contactsMsg.contact(i).collision1() == _collisionName)
    {
      collision2 = this->dataPtr->contactsMsg.contact(i).collision2();
    }
    else if (this->dataPtr->contactsMsg.contact(i).collision2() ==
        _collisionName)
    {
      collision2 =  this->dataPtr->contactsMsg.contact(i).collision1();
    }

    if (collision2.empty())
      continue;

    result[collision2] = this->dataPtr->contactsMsg.contact(i);
  }

  return result;
}

//////////////////////////////////////////////////
void ContactSensor::OnContacts(ConstContactsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->IsActive())
  {
    gzwarn << "Contact sensor [" << this->Name() << "] is subscribed to "
           << "contacts even though it isn't active. This should not "
           << "happen." << std::endl;
    return;
  }

  // Store the contacts message for processing in UpdateImpl
  this->dataPtr->incomingContacts.push_back(_msg);

  // Prevent the incomingContacts list to grow indefinitely.
  if (this->dataPtr->incomingContacts.size() > 100)
    this->dataPtr->incomingContacts.pop_front();
}

//////////////////////////////////////////////////
bool ContactSensor::IsActive() const
{
  return this->active ||
    (this->dataPtr->contactsPub &&
     this->dataPtr->contactsPub->HasConnections());
}
