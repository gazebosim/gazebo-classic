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
#include <boost/algorithm/string.hpp>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/common/Time.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/ContactManager.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
ContactManager::ContactManager()
{
  this->contactIndex = 0;
  this->customMutex = new boost::recursive_mutex();
  this->neverDropContacts = false;
}

/////////////////////////////////////////////////
ContactManager::~ContactManager()
{
  this->Clear();

  this->contactPub.reset();
  if (this->node)
    this->node->Fini();
  this->node.reset();

  boost::unordered_map<std::string, ContactPublisher *>::iterator iter;
  for (iter = this->customContactPublishers.begin();
      iter != this->customContactPublishers.end(); ++iter)
  {
    if (iter->second)
    {
      iter->second->collisions.clear();
      iter->second->collisionNames.clear();
      iter->second->publisher.reset();
      delete iter->second;
      iter->second = NULL;
    }
  }
  this->customContactPublishers.clear();
  delete this->customMutex;
  this->customMutex = NULL;

  this->world.reset();
}

/////////////////////////////////////////////////
void ContactManager::Init(WorldPtr _world)
{
  this->world = _world;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->Name());

  this->contactPub =
    this->node->Advertise<msgs::Contacts>("~/physics/contacts", 50);
}

/////////////////////////////////////////////////
void ContactManager::SetNeverDropContacts(const bool _neverDrop)
{
  this->neverDropContacts = _neverDrop;
}

/////////////////////////////////////////////////
bool ContactManager::NeverDropContacts() const
{
  return this->neverDropContacts;
}

/////////////////////////////////////////////////
bool ContactManager::SubscribersConnected(Collision *_collision1,
                                          Collision *_collision2) const
{
  if (this->contactPub->HasConnections()) return true;

  boost::recursive_mutex::scoped_lock lock(*this->customMutex);
  boost::unordered_map<std::string, ContactPublisher *>::const_iterator iter;
  for (iter = this->customContactPublishers.begin();
       iter != this->customContactPublishers.end(); ++iter)
  {
    // A model can simply be loaded later, so check the collisionNames as well.
    if (!iter->second->collisionNames.empty())
    {
      std::vector<std::string>::const_iterator it;
      for (it = iter->second->collisionNames.begin();
           it != iter->second->collisionNames.end(); ++it)
      {
        BasePtr b = this->world->BaseByName(*it);
        if (b)
        {
          return true;
        }
        // We could do the same transformation which is done in
        // GetCustomPublishers() here (insert collisions which now have been
        // loaded), but this would remove the const qualifier of this function.
        // It would however speed up repeated calls of this function without
        // a call of NewContact() or GetCustomPublishers() in between.
      }
    }

    // only reason _collision1 or _collision1 cannot be const parameters
    // is that compiler can't find const pointers in unordered set
    if (iter->second->collisions.find(_collision1) !=
        iter->second->collisions.end() ||
        iter->second->collisions.find(_collision2) !=
        iter->second->collisions.end())
    {
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
void ContactManager::GetCustomPublishers(Collision *_collision1,
                     Collision *_collision2, const bool _getOnlyConnected,
                     std::vector<ContactPublisher*> &_publishers)
{
  boost::recursive_mutex::scoped_lock lock(*this->customMutex);
  boost::unordered_map<std::string, ContactPublisher *>::iterator iter;
  for (iter = this->customContactPublishers.begin();
       iter != this->customContactPublishers.end(); ++iter)
  {
    // A model can simply be loaded later, so convert ones that are not yet
    // found
    if (!iter->second->collisionNames.empty())
    {
      std::vector<std::string>::iterator it;
      for (it = iter->second->collisionNames.begin();
          it != iter->second->collisionNames.end();)
      {
        Collision *col = boost::dynamic_pointer_cast<Collision>(
            this->world->BaseByName(*it)).get();
        if (!col)
        {
          ++it;
          continue;
        }
        it = iter->second->collisionNames.erase(it);
        iter->second->collisions.insert(col);
      }
    }

    // only reason _collision1 or _collision1 cannot be const parameters
    // is that compiler can't find const pointers in unordered set
    if (iter->second->collisions.find(_collision1) !=
        iter->second->collisions.end() ||
        iter->second->collisions.find(_collision2) !=
        iter->second->collisions.end())
    {
      GZ_ASSERT(iter->second->publisher != NULL,
                "ContactPublisher must have a valid publisher");
      if (!_getOnlyConnected || iter->second->publisher->HasConnections())
      {
        _publishers.push_back(iter->second);
      }
    }
  }
}

/////////////////////////////////////////////////
Contact *ContactManager::NewContact(Collision *_collision1,
                                    Collision *_collision2,
                                    const common::Time &_time)
{
  Contact *result = NULL;

  if (!_collision1 || !_collision2)
    return result;

  // If no one is listening to the default topic, or there are no
  // custom contact publishers then don't create any contact information.
  // This is a signal to the Physics engine that it can skip the extra
  // processing necessary to get back contact information.

  std::vector<ContactPublisher *> publishers;
  bool getOnlyConnected = false;
  // TODO check: getOnlyConnected set to false to keep same behaviour as before.
  // But should we not only add publishers which are connected, as is done
  // for this->contactPub->HasConnections() condition?
  this->GetCustomPublishers(_collision1, _collision2,
                            getOnlyConnected, publishers);

  if (this->NeverDropContacts() ||
      this->contactPub->HasConnections() ||
      !publishers.empty())
  {
    // Get or create a contact feedback object.
    if (this->contactIndex < this->contacts.size())
      result = this->contacts[this->contactIndex++];
    else
    {
      result = new Contact();
      this->contacts.push_back(result);
      this->contactIndex = this->contacts.size();
    }
    for (unsigned int i = 0; i < publishers.size(); ++i)
    {
      publishers[i]->contacts.push_back(result);
    }
  }

  if (!result)
    return result;

  result->count = 0;
  result->collision1 = _collision1;
  result->collision2 = _collision2;
  result->time = _time;
  result->world = this->world;

  return result;
}

/////////////////////////////////////////////////
unsigned int ContactManager::GetContactCount() const
{
  return this->contactIndex;
}

/////////////////////////////////////////////////
Contact *ContactManager::GetContact(unsigned int _index) const
{
  if (_index < this->contactIndex)
    return this->contacts[_index];
  else
    return NULL;
}

/////////////////////////////////////////////////
const std::vector<Contact*> &ContactManager::GetContacts() const
{
  return this->contacts;
}

/////////////////////////////////////////////////
void ContactManager::ResetCount()
{
  this->contactIndex = 0;
}

/////////////////////////////////////////////////
void ContactManager::Clear()
{
  // Delete all the contacts.
  for (unsigned int i = 0; i < this->contacts.size(); ++i)
    delete this->contacts[i];

  this->contacts.clear();

  boost::unordered_map<std::string, ContactPublisher *>::iterator iter;
  for (iter = this->customContactPublishers.begin();
      iter != this->customContactPublishers.end(); ++iter)
    iter->second->contacts.clear();

  // Reset the contact count to zero.
  this->contactIndex = 0;
}

/////////////////////////////////////////////////
void ContactManager::PublishContacts()
{
//  if (this->contacts.size() == 0)
//    return;

  if (!this->contactPub)
  {
    gzerr << "ContactManager has not been initialized. "
          << "Unable to publish contacts.\n";
    return;
  }

  // publish to default topic, ~/physics/contacts
  if (!transport::getMinimalComms())
  {
    msgs::Contacts msg;
    for (unsigned int i = 0; i < this->contactIndex; ++i)
    {
      if (this->contacts[i]->count == 0)
        continue;

      msgs::Contact *contactMsg = msg.add_contact();
      this->contacts[i]->FillMsg(*contactMsg);
    }

    msgs::Set(msg.mutable_time(), this->world->SimTime());
    this->contactPub->Publish(msg);
  }

  // publish to other custom topics
  boost::recursive_mutex::scoped_lock lock(*this->customMutex);
  boost::unordered_map<std::string, ContactPublisher *>::iterator iter;
  for (iter = this->customContactPublishers.begin();
      iter != this->customContactPublishers.end(); ++iter)
  {
    ContactPublisher *contactPublisher = iter->second;
    msgs::Contacts msg2;
    for (unsigned int j = 0;
        j < contactPublisher->contacts.size(); ++j)
    {
      if (contactPublisher->contacts[j]->count == 0)
        continue;

      msgs::Contact *contactMsg = msg2.add_contact();
      contactPublisher->contacts[j]->FillMsg(*contactMsg);
    }
    msgs::Set(msg2.mutable_time(), this->world->SimTime());
    contactPublisher->publisher->Publish(msg2);
    contactPublisher->contacts.clear();
  }
}

/////////////////////////////////////////////////
std::string ContactManager::CreateFilter(const std::string &_name,
    const std::string &_collision)
{
  std::vector<std::string> collisions;
  collisions.push_back(_collision);
  return this->CreateFilter(_name, collisions);
}

/////////////////////////////////////////////////
std::string ContactManager::CreateFilter(const std::string &_name,
    const std::map<std::string, physics::CollisionPtr> &_collisions)
{
  std::string name = _name;
  boost::replace_all(name, "::", "/");

  if (this->customContactPublishers.find(name) !=
    this->customContactPublishers.end())
  {
    gzerr << "Filter with the same name already exists! Aborting" << std::endl;
    return "";
  }

  // Contact sensors make use of this filter
  std::string topic = "~/" + name + "/contacts";

  ContactPublisher *contactPublisher = new ContactPublisher;
  contactPublisher->publisher = this->node->Advertise<msgs::Contacts>(topic);

  std::map<std::string, physics::CollisionPtr>::const_iterator iter;
  for (iter = _collisions.begin(); iter != _collisions.end(); ++iter)
  {
    Collision *col = iter->second.get();
    if (col)
      contactPublisher->collisions.insert(col);
  }

  {
    boost::recursive_mutex::scoped_lock lock(*this->customMutex);
    this->customContactPublishers[name] = contactPublisher;
  }

  return topic;
}

/////////////////////////////////////////////////
std::string ContactManager::CreateFilter(const std::string &_name,
    const std::vector<std::string> &_collisions)
{
  if (_collisions.empty())
    return "";

  std::map<std::string, physics::CollisionPtr> collisionMap;

  // some collisions may not be loaded yet, so store their names in
  // collisionNames and try to find them later.
  std::vector<std::string> collisionNames;
  for (unsigned int i = 0; i < _collisions.size(); ++i)
  {
    CollisionPtr colPtr = boost::dynamic_pointer_cast<Collision>(
       this->world->BaseByName(_collisions[i]));
    if (colPtr)
    {
      collisionMap[_collisions[i]] = colPtr;
    }
    else
    {
      collisionNames.push_back(_collisions[i]);
    }
  }

  std::string topic  = this->CreateFilter(_name, collisionMap);

  // The filter should be created in the last call.
  std::string name = _name;
  boost::replace_all(name, "::", "/");

  {
    boost::recursive_mutex::scoped_lock lock(*this->customMutex);
    GZ_ASSERT(this->customContactPublishers.count(name) > 0,
        "Failed to create a custom filter");

    // Let it know about collisions not yet found.
    this->customContactPublishers[name]->collisionNames = collisionNames;
  }

  return topic;
}

/////////////////////////////////////////////////
void ContactManager::RemoveFilter(const std::string &_name)
{
  std::string name = _name;
  boost::replace_all(name, "::", "/");

  boost::recursive_mutex::scoped_lock lock(*this->customMutex);
  boost::unordered_map<std::string, ContactPublisher *>::iterator iter
      = this->customContactPublishers.find(name);
  if (iter != customContactPublishers.end())
  {
    ContactPublisher *contactPublisher = iter->second;
    contactPublisher->contacts.clear();
    contactPublisher->collisionNames.clear();
    contactPublisher->collisions.clear();
    contactPublisher->publisher->Fini();
    contactPublisher->publisher.reset();
    this->customContactPublishers.erase(iter);
  }
}

/////////////////////////////////////////////////
unsigned int ContactManager::GetFilterCount()
{
  boost::recursive_mutex::scoped_lock lock(*this->customMutex);
  return this->customContactPublishers.size();
}

/////////////////////////////////////////////////
bool ContactManager::HasFilter(const std::string &_name)
{
  std::string name = _name;
  boost::replace_all(name, "::", "/");

  boost::recursive_mutex::scoped_lock lock(*this->customMutex);
  return this->customContactPublishers.find(name) !=
      this->customContactPublishers.end();
}
