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
#include <string>
#include "gazebo/common/Console.hh"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/util/IntrospectionManagerPrivate.hh"
#include "gazebo/util/IntrospectionManager.hh"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
IntrospectionFilter::IntrospectionFilter()
  : dataPtr(new IntrospectionFilterPrivate)
{
}

//////////////////////////////////////////////////
IntrospectionFilter &IntrospectionFilter::operator=(
    const IntrospectionFilter &_other)
{
  this->dataPtr->items = _other.Items();
  this->dataPtr->msg = _other.Msg();
  return *this;
}

//////////////////////////////////////////////////
std::vector<std::string> IntrospectionFilter::Items() const
{
  return this->dataPtr->items;
}

//////////////////////////////////////////////////
//std::string IntrospectionFilter::Topic() const
//{
//  return this->dataPtr->topic;
//}

//////////////////////////////////////////////////
std::vector<std::string> &IntrospectionFilter::MutableItems()
{
  return this->dataPtr->items;
}

//////////////////////////////////////////////////
const msgs::Param_V &IntrospectionFilter::Msg() const
{
  return this->dataPtr->msg;
}

//////////////////////////////////////////////////
msgs::Param_V &IntrospectionFilter::MutableMsg()
{
  return this->dataPtr->msg;
}

//////////////////////////////////////////////////
IntrospectionManager::IntrospectionManager()
  : dataPtr(new IntrospectionManagerPrivate)
{
}

//////////////////////////////////////////////////
void IntrospectionManager::SetFilter(const std::string &_topic,
    const std::vector<std::string> _items)
{
  std::vector<std::string> oldItems;

  if (this->dataPtr->filters.find(_topic) == this->dataPtr->filters.end())
  {
    // Advertise the topic.
  }
  else
    oldItems = this->dataPtr->filters.at(_topic).Items();

  this->dataPtr->filters[_topic].MutableItems() = _items;

  // Remove the filter ID from the old items.
  for (auto const &oldItem : oldItems)
  {
    if (std::find(_items.begin(), _items.end(), oldItem) == _items.end())
    {
      auto &filters = this->dataPtr->observedItems[oldItem].filters;
      filters.erase(std::remove(
          filters.begin(), filters.end(), _topic), filters.end());

      // If there is nobody interested in this item, remove it.
      if (filters.empty())
        this->dataPtr->observedItems.erase(oldItem);
    }
  }

  // Add the filter ID to the new items.
  for (auto const &newItem : _items)
  {
    if (std::find(oldItems.begin(), oldItems.end(), newItem) == oldItems.end())
    {
      this->dataPtr->observedItems[newItem].filters.push_back(_topic);
    }
  }
}

//////////////////////////////////////////////////
void IntrospectionManager::RemoveFilter(const std::string &_topic)
{
  if (this->dataPtr->filters.find(_topic) == this->dataPtr->filters.end())
    return;

  std::vector<std::string> oldItems = this->dataPtr->filters.at(_topic).Items();

  // Let's remove the filter.
  this->dataPtr->filters.erase(_topic);

  // Remove any reference to this filter inside observedItems.
  for (auto const &oldItem : oldItems)
  {
    auto &filters = this->dataPtr->observedItems[oldItem].filters;
    filters.erase(std::remove(
        filters.begin(), filters.end(), _topic), filters.end());

    // If there is nobody interested in this item, remove it.
    if (filters.empty())
      this->dataPtr->observedItems.erase(oldItem);
  }

  // Unadvertise topic.
}

//////////////////////////////////////////////////
bool IntrospectionManager::Filter(const std::string &_topic,
    IntrospectionFilter &_filter) const
{
  if (this->dataPtr->filters.find(_topic) == this->dataPtr->filters.end())
    return false;

  _filter = this->dataPtr->filters.at(_topic);
  return true;
}

//////////////////////////////////////////////////
bool IntrospectionManager::Register(const std::string &_item,
    const std::string &_type,
    const std::function <bool (gazebo::msgs::GzString &_msg)> &/*_cb*/)
{
  // Sanity check: Make sure that nobody has register the same item before.
  if (this->dataPtr->allItems.find(_item) != this->dataPtr->allItems.end())
  {
    gzwarn << "Item [" << _item << "] already registered" << std::endl;
    return false;
  }

  this->dataPtr->allItems[_item] = _type;
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
