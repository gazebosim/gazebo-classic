/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
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

#include <functional>
#include <set>
#include <string>
#include <ignition/math/Rand.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/msgs/any.pb.h"
#include "gazebo/msgs/empty.pb.h"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/param.pb.h"
#include "gazebo/msgs/param_v.pb.h"
#include "gazebo/util/IntrospectionManager.hh"
#include "gazebo/util/IntrospectionManagerPrivate.hh"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
IntrospectionManager::IntrospectionManager()
  : dataPtr(new IntrospectionManagerPrivate)
{
  // Create a unique manager ID based on a combination of letters. We don't
  // expect to have a massive number of introspection managers running
  // concurrently, so no need to use UUIDs.
  this->dataPtr->managerId = this->CreateRandomId(6);

  this->dataPtr->prefix = "/introspection/" + this->dataPtr->managerId + "/";

  // Advertise the service for creating a new filter.
  std::string service = this->dataPtr->prefix + "filter_new";
  if (!this->dataPtr->node.Advertise(service,
      &IntrospectionManager::NewFilter, this))
  {
    gzerr << "Error advertising service [" << service << "]" << std::endl;
  }

  // Advertise the service for updating an existing filter.
  service = this->dataPtr->prefix + "filter_update";
  if (!this->dataPtr->node.Advertise(service,
      &IntrospectionManager::UpdateFilter, this))
  {
    gzerr << "Error advertising service [" << service << "]" << std::endl;
  }

  // Advertise the service for removing an existing filter.
  service = this->dataPtr->prefix + "filter_remove";
  if (!this->dataPtr->node.Advertise(service,
      &IntrospectionManager::RemoveFilter, this))
  {
    gzerr << "Error advertising service [" << service << "]" << std::endl;
  }

  // Advertise the service for listing all items registered in this manager.
  service = this->dataPtr->prefix + "items";
  if (!this->dataPtr->node.Advertise(service,
      &IntrospectionManager::Items, this))
  {
    gzerr << "Error advertising service [" << service << "]" << std::endl;
  }

  // Advertise the topic for notifying changes in the registered items.
  std::string topic = "/introspection/" + this->dataPtr->managerId +
      "/items_update";
  if (!this->dataPtr->node.Advertise<gazebo::msgs::Param_V>(topic))
  {
    gzerr << "Error advertising topic [" << topic << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
IntrospectionManager::~IntrospectionManager()
{
}

//////////////////////////////////////////////////
std::string IntrospectionManager::Id() const
{
  return this->dataPtr->managerId;
}

//////////////////////////////////////////////////
bool IntrospectionManager::Register(const std::string &_item,
    const std::function <gazebo::msgs::Any ()> &_cb)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Sanity check: Make sure that nobody has registered the same item before.
  if (this->dataPtr->allItems.find(_item) != this->dataPtr->allItems.end())
  {
    gzwarn << "Item [" << _item << "] already registered" << std::endl;
    return false;
  }

  this->dataPtr->allItems[_item] = _cb;

  this->dataPtr->itemsUpdated = true;

  return true;
}

//////////////////////////////////////////////////
bool IntrospectionManager::Unregister(const std::string &_item)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Sanity check: Make sure that the item has been previously registered.
  if (this->dataPtr->allItems.find(_item) == this->dataPtr->allItems.end())
  {
    gzwarn << "Item [" << _item << "] is not registered" << std::endl;
    return false;
  }

  // Remove the item from the list of all items.
  this->dataPtr->allItems.erase(_item);

  this->dataPtr->itemsUpdated = true;

  return true;
}

//////////////////////////////////////////////////
void IntrospectionManager::Clear()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->allItems.clear();
  this->dataPtr->itemsUpdated = true;
}

//////////////////////////////////////////////////
std::set<std::string> IntrospectionManager::Items() const
{
  std::set<std::string> items;

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    for (auto item : this->dataPtr->allItems)
      items.emplace(item.first);
  }

  return items;
}

//////////////////////////////////////////////////
void IntrospectionManager::Update()
{
  std::map<std::string, IntrospectionFilter> filtersCopy;
  std::map<std::string, std::function <gazebo::msgs::Any ()>> allItemsCopy;
  std::map<std::string, ObservedItem> observedItemsCopy;

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    // Make a copy of these members for avoiding locking a mutex while calling a
    // user callback (we could create a deadlock).
    // More creative solutions are welcome.
    filtersCopy = this->dataPtr->filters;
    allItemsCopy = this->dataPtr->allItems;
    observedItemsCopy = this->dataPtr->observedItems;
  }

  for (auto &observedItem : observedItemsCopy)
  {
    auto &item = observedItem.first;
    auto itemIter = allItemsCopy.find(item);

    // Sanity check: Make sure that we can update the item.
    if (itemIter == allItemsCopy.end())
      continue;

    try
    {
      // Update the values of the items under observation.
      gazebo::msgs::Any value = itemIter->second();
      auto &lastValue = observedItem.second.lastValue;
      lastValue.CopyFrom(value);
    }
    catch(...)
    {
      gzerr << "Exception caught calling user callback" << std::endl;
      continue;
    }
  }

  // Prepare the next message to be sent in each filter.
  for (auto &filter : filtersCopy)
  {
    // First of all, clear the old message.
    auto &nextMsg = filter.second.msg;
    nextMsg.Clear();

    // Insert the last value of each item under observation for this filter.
    for (auto const &item : filter.second.items)
    {
      // Sanity check: Make sure that someone registered this item.
      if (allItemsCopy.find(item) == allItemsCopy.end())
        continue;

      // Sanity check: Make sure that the value was updated.
      // (e.g.: an exception was not raised).
      auto &lastValue = observedItemsCopy[item].lastValue;
      if (lastValue.type() == gazebo::msgs::Any::NONE)
        continue;

      auto nextParam = nextMsg.add_param();
      nextParam->set_name(item);
      nextParam->mutable_value()->CopyFrom(lastValue);
    }

    // Sanity check: Make sure that we have at least one item updated.
    if (nextMsg.param_size() == 0)
      continue;

    // Publish the update for this filter.
    std::string topicName = this->dataPtr->prefix + "filter/" + filter.first;
    if (!this->dataPtr->node.Publish(topicName, nextMsg))
    {
      gzerr << "Error publishing update for topic [" << topicName << "]"
            << std::endl;
    }
  }

  this->NotifyUpdates();
}

//////////////////////////////////////////////////
void IntrospectionManager::NotifyUpdates()
{
  bool itemsUpdatedCopy;

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    itemsUpdatedCopy = this->dataPtr->itemsUpdated;
    this->dataPtr->itemsUpdated = false;
  }

  if (itemsUpdatedCopy)
  {
    gazebo::msgs::Empty req;
    bool result;
    gazebo::msgs::Param_V currentItems;
    // Prepare the list of items to be sent.
    this->Items(req, currentItems, result);

    std::string topicName = "/introspection/" + this->dataPtr->managerId +
      "/items_update";
    this->dataPtr->node.Publish(topicName, currentItems);
  }
}

//////////////////////////////////////////////////
bool IntrospectionManager::NewFilter(const std::set<std::string> &_newItems,
    std::string &_filterId)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Create a unique filter ID based on a combination of letters. We don't
  // expect to have a massive number of introspection managers running
  // concurrently, so no need to use UUIDs.
  do
  {
    _filterId = this->CreateRandomId(6);
  }
  while (this->dataPtr->filters.find(_filterId) !=
         this->dataPtr->filters.end());

  std::string topicName = this->dataPtr->prefix + "filter/" + _filterId;

  // Advertise the new topic.
  if (!this->dataPtr->node.Advertise<gazebo::msgs::Param_V>(topicName))
  {
    gzerr << "Error advertising topic [" << topicName << "]." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    return false;
  }

  // Add the items to the new filter.
  this->dataPtr->filters[_filterId].items = _newItems;

  // Register the new filter in the list of observed items.
  for (auto const &item : _newItems)
    this->dataPtr->observedItems[item].filters.emplace(_filterId);

  return true;
}

//////////////////////////////////////////////////
bool IntrospectionManager::UpdateFilter(const std::string &_filterId,
    const std::set<std::string> &_newItems)
{
  // Sanity check: Make sure that we have at least one item to be observed.
  if (_newItems.empty())
  {
    gzwarn << "Filter update request with empty list of items." << std::endl;
    gzwarn << "Ignoring request." << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Sanity check: Make sure that filter ID exists.
  if (this->dataPtr->filters.find(_filterId) == this->dataPtr->filters.end())
  {
    gzwarn << "Unknown ID [" << _filterId << "] in filter update" << std::endl;
    gzwarn << "Ignoring request." << std::endl;
    return false;
  }

  // Save the old list of items.
  auto oldItems = this->dataPtr->filters.at(_filterId).items;

  // Update the list of items for this filter.
  this->dataPtr->filters[_filterId].items = _newItems;

  // The next block is needed for updating the 'observedItems' data structure
  // that contains references to the filters.
  {
    // We need to ensure that the items that were part of the old filter but are
    // not in the new filter are removed.
    for (auto const &oldItem : oldItems)
    {
      if (_newItems.find(oldItem) == _newItems.end())
      {
        auto &filters = this->dataPtr->observedItems[oldItem].filters;
        filters.erase(_filterId);

        // If there is nobody interested in this item, remove it.
        if (filters.empty())
          this->dataPtr->observedItems.erase(oldItem);
      }
    }

    // We need to add the new items that were not part of the old filter.
    for (auto const &newItem : _newItems)
    {
      if (oldItems.find(newItem) == oldItems.end())
        this->dataPtr->observedItems[newItem].filters.emplace(_filterId);
    }
  }

  return true;
}

//////////////////////////////////////////////////
bool IntrospectionManager::RemoveFilter(const std::string &_filterId)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Sanity check: Make sure that filter ID exists.
  if (this->dataPtr->filters.find(_filterId) == this->dataPtr->filters.end())
  {
    gzwarn << "Unknown ID [" << _filterId << "] in filter removal" << std::endl;
    gzwarn << "Ignoring request." << std::endl;
    return false;
  }

  // Unadvertise topic.
  std::string topicName = this->dataPtr->prefix + "filter/" + _filterId;
  if (!this->dataPtr->node.Unadvertise(topicName))
  {
    gzerr << "Error unadvertising topic [" << topicName << "]" << std::endl;
    gzerr << "Ignoring request." << std::endl;
    return false;
  }

  // Save the old list of items.
  auto oldItems = this->dataPtr->filters.at(_filterId).items;

  // Let's remove the filter.
  this->dataPtr->filters.erase(_filterId);

  // Remove any reference to this filter inside observedItems.
  for (auto const &oldItem : oldItems)
  {
    auto &filters = this->dataPtr->observedItems[oldItem].filters;
    filters.erase(_filterId);

    // If there is nobody interested in this item, remove it.
    if (filters.empty())
      this->dataPtr->observedItems.erase(oldItem);
  }

  return true;
}

//////////////////////////////////////////////////
void IntrospectionManager::NewFilter(const gazebo::msgs::Param_V &_req,
    gazebo::msgs::GzString &_rep, bool &_result)
{
  _rep.set_data("");
  _result = false;

  // Sanity check: Make sure that the message contains at least one parameter.
  if (_req.param_size() == 0)
  {
    gzwarn << "Empty parameter message received." << std::endl;
    gzwarn << "Ignoring request." << std::endl;
    return;
  }

  std::set<std::string> requestedItems;

  // Store the new filter.
  for (auto i = 0; i < _req.param_size(); ++i)
  {
    auto param = _req.param(i);
    if (!this->ValidateParameter(param, {"item"}))
    {
      gzwarn << "Invalid parameter[" << param.name() << "] "
        << "Ignoring request." << std::endl;
      return;
    }

    auto item = param.value().string_value();
    requestedItems.emplace(item);
  }

  std::string topicName;
  if (!this->NewFilter(requestedItems, topicName))
  {
    gzwarn << "Ignoring request." << std::endl;
    return;
  }

  // Answer with the custom topic created for the client.
  _rep.set_data(topicName);
  _result = true;
}

//////////////////////////////////////////////////
void IntrospectionManager::UpdateFilter(const gazebo::msgs::Param_V &_req,
    gazebo::msgs::Empty &/*_rep*/, bool &_result)
{
  _result = false;

  // Sanity check: Make sure that the message contains at least one parameter.
  if (_req.param_size() == 0)
  {
    gzwarn << "Empty parameter message received." << std::endl;
    gzwarn << "Ignoring request." << std::endl;
    return;
  }

  std::set<std::string> newItems;
  std::string filterId;

  for (auto i = 0; i < _req.param_size(); ++i)
  {
    auto param = _req.param(i);
    if (!this->ValidateParameter(param, {"item", "filter_id"}))
    {
      gzwarn << "Ignoring request." << std::endl;
      return;
    }

    if (param.name() == "item")
    {
      // Update the new list of items.
      newItems.emplace(param.value().string_value());
    }
    else if (param.name() == "filter_id")
    {
      // Sanity check: Make sure that we didn't receive a 'filter_id' before.
      if (!filterId.empty())
      {
        gzwarn << "Received more than one param with 'filter_id'." << std::endl;
        gzwarn << "Ignoring request." << std::endl;
        return;
      }

      // Save filter ID to be updated.
      filterId = param.value().string_value();
    }
    else
    {
      gzwarn << "Unexpected param name [" << param.name() << "]." << std::endl;
      gzwarn << "Ignoring request." << std::endl;
      return;
    }
  }

  // Sanity check: Make sure that we received the filter_id.
  if (filterId.empty())
  {
    gzwarn << "Parameter without a 'filter_id' value." << std::endl;
    gzwarn << "Ignoring request." << std::endl;
    return;
  }

  if (!this->UpdateFilter(filterId, newItems))
    return;

  _result = true;
}

//////////////////////////////////////////////////
void IntrospectionManager::RemoveFilter(const gazebo::msgs::Param_V &_req,
    gazebo::msgs::Empty &/*_rep*/, bool &_result)
{
  _result = false;

  // Sanity check: Make sure that the message contains exactly one parameter.
  if (_req.param_size() != 1)
  {
    gzwarn << "Expecting message with exactly 1 parameter but "
          << _req.param_size() << " were received." << std::endl;
    gzwarn << "Ignoring request." << std::endl;
    return;
  }

  auto param = _req.param(0);
  if (!this->ValidateParameter(param, {"filter_id"}))
  {
    gzwarn << "Ignoring request." << std::endl;
    return;
  }

  std::string filterId = param.value().string_value();
  if (!this->RemoveFilter(filterId))
    return;

  _result = true;
}

//////////////////////////////////////////////////
void IntrospectionManager::Items(const gazebo::msgs::Empty &/*_req*/,
    gazebo::msgs::Param_V &_rep, bool &_result)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    for (auto const &item : this->dataPtr->allItems)
    {
      auto nextParam = _rep.add_param();
      nextParam->set_name("item");
      nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
      nextParam->mutable_value()->set_string_value(item.first);
    }
  }
  _result = true;
}

//////////////////////////////////////////////////
std::string IntrospectionManager::CreateRandomId(
    const unsigned int &_size) const
{
  std::string result;
  std::string alphabet;
  for (char c = 'a'; c <= 'z'; ++c)
    alphabet += c;

  for (auto i = 0u; i < _size; ++i)
  {
    auto index = ignition::math::Rand::IntUniform(0, alphabet.size() - 1);
    result += alphabet.at(index);
  }
  return result;
}

//////////////////////////////////////////////////
bool IntrospectionManager::ValidateParameter(const gazebo::msgs::Param &_msg,
    const std::set<std::string> &_allowedValues) const
{
  if (_allowedValues.find(_msg.name()) == _allowedValues.end())
  {
    gzwarn << "Unexpected parameter name. Expected names: [";
    for (auto const &v : _allowedValues)
      gzerr << "'" << v << "' ";
    gzwarn << "]" << std::endl << "Received name: " << _msg.name() << std::endl;
    return false;
  }

  if (!_msg.has_value())
  {
    gzwarn << "Parameter without a value." << std::endl;
    return false;
  }

  auto value = _msg.value();
  if (value.type() != gazebo::msgs::Any::STRING)
  {
    gzwarn << "Expected a parameter with STRING value. Instead, I received ["
          << value.type() << "]." << std::endl;
    return false;
  }

  if (!value.has_string_value())
  {
    gzwarn << "Received a parameter without the 'string_value' field."
          << std::endl;
    return false;
  }

  return true;
}
