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

#include <algorithm>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <ignition/math/Rand.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/msgs/any.pb.h"
#include "gazebo/util/IntrospectionManagerPrivate.hh"
#include "gazebo/util/IntrospectionManager.hh"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
IntrospectionManager::IntrospectionManager()
  : dataPtr(new IntrospectionManagerPrivate)
{
  // Create a unique manager ID based on a combination of letters. We don't
  // expect to have a massive number of introspection managers running
  // concurrently, so no need to use UUIDs.
  std::string alphabet;
  for (char c = 'a'; c <= 'z'; ++c)
    alphabet += c;

  const int kManagerIdSize = 6;
  for (auto i = 0; i < kManagerIdSize; ++i)
  {
    auto index = ignition::math::Rand::IntUniform(0, alphabet.size() - 1);
    this->dataPtr->managerId += alphabet.at(index);
  }

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

  // Advertise the service for listing the items of an existing filter.
  service = this->dataPtr->prefix + "filter_info";
  if (!this->dataPtr->node.Advertise(service,
      &IntrospectionManager::Filter, this))
  {
    gzerr << "Error advertising service [" << service << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
IntrospectionManager::~IntrospectionManager()
{
}

//////////////////////////////////////////////////
bool IntrospectionManager::Register(const std::string &_item,
    const std::string &_type,
    const std::function <bool (gazebo::msgs::Any &_msg)> &_cb)
{
  // Sanity check: Make sure that nobody has registered the same item before.
  if (this->dataPtr->allItems.find(_item) != this->dataPtr->allItems.end())
  {
    gzwarn << "Item [" << _item << "] already registered" << std::endl;
    return false;
  }

  this->dataPtr->allItems[_item].type = _type;
  this->dataPtr->allItems[_item].cb = _cb;
  return true;
}

//////////////////////////////////////////////////
bool IntrospectionManager::Unregister(const std::string &_item)
{
  // Sanity check: Make sure that the item has been previously registered.
  if (this->dataPtr->allItems.find(_item) == this->dataPtr->allItems.end())
  {
    gzwarn << "Item [" << _item << "] is not registered" << std::endl;
    return false;
  }

  // Remove the item from the list of all items.
  this->dataPtr->allItems.erase(_item);

  return true;
}

//////////////////////////////////////////////////
std::set<std::string> IntrospectionManager::RegisteredItems() const
{
  std::set<std::string> items;
  for (auto item : this->dataPtr->allItems)
    items.emplace(item.first);

  return items;
}

//////////////////////////////////////////////////
void IntrospectionManager::NewFilter(const gazebo::msgs::Param_V &_req,
    gazebo::msgs::GzString &_rep, bool &_result)
{
  _rep.set_data("");

  // Create a unique filter ID based on a combination of letters. We don't
  // expect to have a massive number of introspection managers running
  // concurrently, so no need to use UUIDs.
  std::string alphabet;
  for (char c = 'a'; c <= 'z'; ++c)
    alphabet += c;

  const int kManagerIdSize = 6;
  std::string filterId;
  for (auto i = 0; i < kManagerIdSize; ++i)
  {
    auto index = ignition::math::Rand::IntUniform(0, alphabet.size() - 1);
    filterId += alphabet.at(index);
  }

  // Sanity check: Make sure that the message contains at least one parameter.
  if (_req.param_size() == 0)
  {
    std::cerr << "Empty parameter message received." << std::endl;
    gzerr << "Empty parameter message received." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  std::set<std::string> requestedItems;

  // Store the new filter.
  for (auto i = 0; i < _req.param_size(); ++i)
  {
    auto param = _req.param(i);
    if (param.name() != "item")
    {
      std::cerr << "Unexpected parameter name in filter. I expect 'item' but "
            << "instead [" << param.name() << "] was received." << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    if (!param.has_value())
    {
      std::cerr << "Parameter without a value field in filter." << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    auto value = param.value();
    if (value.type() != gazebo::msgs::Any::STRING)
    {
      std::cerr << "Expected a parameter with STRING value. Instead, I received ["
            << value.type() << "]." << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    if (!value.has_string_value())
    {
      std::cerr << "Received a parameter without the 'string_value' field."
            << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    auto item = value.string_value();
    requestedItems.emplace(item);
  }

  // Advertise the new topic.
  std::string topicName = "/introspection/filter/" + filterId;
  if (!this->dataPtr->node.Advertise<gazebo::msgs::Param_V>(topicName))
  {
    std::cerr << "Error advertising topic [" << topicName << "]." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  // Add the items to the new filter.
  this->dataPtr->filters[filterId].items = requestedItems;

  // Register the new filter in the list of observed items.
  for (auto const item : requestedItems)
    this->dataPtr->observedItems[item].filters.emplace(filterId);

  // Answer with the custom topic created for the client.
  _rep.set_data(topicName);
  _result = true;
}

//////////////////////////////////////////////////
void IntrospectionManager::UpdateFilter(const gazebo::msgs::Param_V &_req,
    gazebo::msgs::GzString &_rep, bool &_result)
{
  _rep.set_data("");

  // Sanity check: Make sure that the message contains at least one parameter.
  if (_req.param_size() == 0)
  {
    gzerr << "Empty parameter message received." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  std::set<std::string> newItems;
  std::string filterId;

  for (auto i = 0; i < _req.param_size(); ++i)
  {
    auto param = _req.param(i);
    if ((param.name() != "item") || (param.name() != "filter_id"))
    {
      gzerr << "Unexpected parameter name in filter. I expect 'item' or "
            << "'filter_id' but instead received [" << param.name()
            << "]." << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    if (!param.has_value())
    {
      gzerr << "Parameter without a value field in filter." << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    auto value = param.value();
    if (value.type() != gazebo::msgs::Any::STRING)
    {
      gzerr << "Expected a parameter with STRING value. Instead, I received ["
            << value.type() << "]." << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    if (!value.has_string_value())
    {
      gzerr << "Received a parameter without the 'string_value' field."
            << std::endl;
      gzerr << "Ignoring request." << std::endl;
      _result = false;
      return;
    }

    if (param.name() == "item")
    {
      // Update the new list of items.
      newItems.emplace(value.string_value());
    }
    else if (param.name() == "filter_id")
    {
      // Sanity check: Make sure that we didn't receive a 'filter_id' before.
      if (!filterId.empty())
      {
        gzerr << "Received more that one parameter with 'filter_id' field."
              << std::endl;
        gzerr << "Ignoring request." << std::endl;
        _result = false;
        return;
      }

      // Save filter ID to be updated.
      filterId = value.string_value();
    }
  }

  // Sanity check: Make sure that we received the filter_id.
  if (filterId.empty())
  {
    gzerr << "Parameter without a 'filter_id' value." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  // Sanity check: Make sure that filter ID exists.
  if (this->dataPtr->filters.find(filterId) == this->dataPtr->filters.end())
  {
    gzerr << "Unknown filter ID [" << filterId << "] in filter update "
          << "request." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  // Sanity check: Make sure that we have at least one item to be observed.
  if (newItems.empty())
  {
    gzerr << "Filter update request without any items." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  // Save the old list of items.
  auto oldItems = this->dataPtr->filters.at(filterId).items;

  // Update the list of items for this filter.
  this->dataPtr->filters[filterId].items = newItems;

  // The next block is needed for updating the 'observedItems' data structure
  // that contains references to the filters.

  // We need to ensure that the items that were part of the old filter but are
  // not in the new filter are removed.
  for (auto const &oldItem : oldItems)
  {
    if (std::find(newItems.begin(), newItems.end(), oldItem) == newItems.end())
    {
      auto &filters = this->dataPtr->observedItems[oldItem].filters;
      filters.erase(filterId);

      // If there is nobody interested in this item, remove it.
      if (filters.empty())
        this->dataPtr->observedItems.erase(oldItem);
    }
  }

  // We need to add the new items that were not part of the old filter.
  for (auto const &newItem : newItems)
  {
    if (std::find(oldItems.begin(), oldItems.end(), newItem) == oldItems.end())
    {
      this->dataPtr->observedItems[newItem].filters.emplace(filterId);
    }
  }

  _result = true;
}

//////////////////////////////////////////////////
void IntrospectionManager::RemoveFilter(const gazebo::msgs::Param_V &_req,
    gazebo::msgs::GzString &_rep, bool &_result)
{
  _rep.set_data("");

  // Sanity check: Make sure that the message contains at least one parameter.
  if (_req.param_size() != 1)
  {
    gzerr << "Expecting message with exactly 1 parameter but "
          << _req.param_size() << " were received." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  auto param = _req.param(0);
  if (param.name() != "filter_id")
  {
    gzerr << "Unexpected parameter name in filter. I expect 'filter_id' but "
          << "instead received [" << param.name() << "]." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  if (!param.has_value())
  {
    gzerr << "Parameter without a value field in filter." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  auto value = param.value();
  if (value.type() != gazebo::msgs::Any::STRING)
  {
    gzerr << "Expected a parameter with STRING value. Instead, I received ["
          << value.type() << "]." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  if (!value.has_string_value())
  {
    gzerr << "Received a parameter without the 'string_value' field."
          << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  std::string filterId = value.string_value();

  // Sanity check: Make sure that filter ID exists.
  if (this->dataPtr->filters.find(filterId) == this->dataPtr->filters.end())
  {
    gzerr << "Unknown filter ID [" << filterId << "] in filter remove "
          << "request." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  // Unadvertise topic.
  std::string topicName = this->dataPtr->prefix + filterId;
  if (!this->dataPtr->node.Unadvertise(topicName))
  {
    std::cerr << "Error unadvertising topic [" << topicName << "]" << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  // Save the old list of items.
  auto oldItems = this->dataPtr->filters.at(filterId).items;

  // Let's remove the filter.
  this->dataPtr->filters.erase(filterId);

  // Remove any reference to this filter inside observedItems.
  for (auto const &oldItem : oldItems)
  {
    auto &filters = this->dataPtr->observedItems[oldItem].filters;
    filters.erase(filterId);

    // If there is nobody interested in this item, remove it.
    if (filters.empty())
      this->dataPtr->observedItems.erase(oldItem);
  }

  _result = true;
}

//////////////////////////////////////////////////
void IntrospectionManager::Filter(const gazebo::msgs::Param_V &_req,
    gazebo::msgs::Param_V &_rep, bool &_result)
{
  // Sanity check: Make sure that the message contains at least one parameter.
  if (_req.param_size() != 1)
  {
    gzerr << "Expecting message with exactly 1 parameter but "
          << _req.param_size() << " were received." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  auto param = _req.param(0);
  if (param.name() != "filter_id")
  {
    gzerr << "Unexpected parameter name in filter. I expect 'filter_id' but "
          << "instead received [" << param.name() << "]." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  if (!param.has_value())
  {
    gzerr << "Parameter without a value field in filter." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  auto value = param.value();
  if (value.type() != gazebo::msgs::Any::STRING)
  {
    gzerr << "Expected a parameter with STRING value. Instead, I received ["
          << value.type() << "]." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  if (!value.has_string_value())
  {
    gzerr << "Received a parameter without the 'string_value' field."
          << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  std::string filterId = value.string_value();

  // Sanity check: Make sure that filter ID exists.
  if (this->dataPtr->filters.find(filterId) == this->dataPtr->filters.end())
  {
    gzerr << "Unknown filter ID [" << filterId << "] in filter remove "
          << "request." << std::endl;
    gzerr << "Ignoring request." << std::endl;
    _result = false;
    return;
  }

  auto &filter = this->dataPtr->filters.at(filterId);
  for (auto const &item : filter.items)
  {
    auto newParam = _rep.add_param();
    newParam->set_name("item");
    newParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
    newParam->mutable_value()->set_string_value(item);
  }

  _result = true;
}

//////////////////////////////////////////////////
void IntrospectionManager::Update()
{
  for (auto &observedItem : this->dataPtr->observedItems)
  {
    auto &item = observedItem.first;
    auto &lastValue = observedItem.second.lastValue;

    // Sanity check: Make sure that we can update the item.
    if (this->dataPtr->allItems.find(item) == this->dataPtr->allItems.end())
      continue;

    // Update the values of the items under observation.
    auto &callback = this->dataPtr->allItems[item].cb;
    gazebo::msgs::Any value;
    if (!callback(value))
    {
      std::cerr << "Something went wrong updating the value for item [" << item
            << "]." << std::endl;
      continue;
    }
    lastValue.CopyFrom(value);

    // We should have at least one filter, otherwise this entry should have been
    // previously recycled.
    //GZ_ASSERT(!filters.empty(), "No filters on item [" + item + "]");
  }

  // Prepare the next message to be sent in each filter.
  for (auto &filter : this->dataPtr->filters)
  {
    // First of all, clear the old message.
    auto &nextMsg = filter.second.msg;
    nextMsg.Clear();

    // Insert the last value of each item under observation for this filter.
    for (auto const &item : filter.second.items)
    {
      // Sanity check: Make sure that someone registered this item.
      if (this->dataPtr->allItems.find(item) == this->dataPtr->allItems.end())
        continue;

      auto nextParam = nextMsg.add_param();
      nextParam->set_name(item);
      nextParam->mutable_value()->CopyFrom(this->dataPtr->observedItems[item].lastValue);
    }

    // Sanity check: Make sure that we have at least one item updated.
    if (nextMsg.param_size() == 0)
      continue;

    // Publish the update for this filter.
    std::string topicName = "/introspection/filter/" + filter.first;
    if (!this->dataPtr->node.Publish(topicName, nextMsg))
    {
      std::cerr << "Error publishing update for topic [" << topicName << "]"
            << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void IntrospectionManager::Show()
{
  std::cout << "***" << std::endl;
  std::cout << "Registered items" << std::endl;
  for (auto const &item : this->dataPtr->allItems)
    std::cout << "  " << item.first << " [" << item.second.type << "]" << std::endl;

  std::cout << std::endl << "Filters" << std::endl;
  for (auto const &filter : this->dataPtr->filters)
  {
    std::cout << "Id: " << filter.first << std::endl;
    std::cout << "Items: ";
    for (auto const &item : filter.second.items)
      std::cout << "[" << item << "] ";
    std::cout << std::endl;
  }

  //std::cout << std::endl << "Observed items";
}
