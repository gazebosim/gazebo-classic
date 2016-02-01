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
}

//////////////////////////////////////////////////
IntrospectionManager::~IntrospectionManager()
{
}

//////////////////////////////////////////////////
void IntrospectionManager::SetFilter(const std::string &_filterId,
    const std::vector<std::string> &_items)
{
  std::vector<std::string> oldItems;

  if (this->dataPtr->filters.find(_filterId) == this->dataPtr->filters.end())
  {
    // Advertise the topic.
  }
  else
    oldItems = this->dataPtr->filters.at(_filterId).items;

  this->dataPtr->filters[_filterId].items = _items;

  // This call might be an update through an existing filter. We have to make
  // sure that the items that were part of the old filter but are not in the
  // new filter are removed.
  for (auto const &oldItem : oldItems)
  {
    if (std::find(_items.begin(), _items.end(), oldItem) == _items.end())
    {
      auto &filters = this->dataPtr->observedItems[oldItem].filters;
      filters.erase(std::remove(
          filters.begin(), filters.end(), _filterId), filters.end());

      // If there is nobody interested in this item, remove it.
      if (filters.empty())
        this->dataPtr->observedItems.erase(oldItem);
    }
  }

  // We have to add the new items that were not part of the old filter.
  for (auto const &newItem : _items)
  {
    if (std::find(oldItems.begin(), oldItems.end(), newItem) == oldItems.end())
    {
      this->dataPtr->observedItems[newItem].filters.push_back(_filterId);
    }
  }
}

//////////////////////////////////////////////////
bool IntrospectionManager::RemoveFilter(const std::string &_filterId)
{
  if (this->dataPtr->filters.find(_filterId) == this->dataPtr->filters.end())
  {
    gzerr << "Unable to remove filter [" << _filterId << "]" << std::endl;
    return false;
  }

  auto oldItems = this->dataPtr->filters.at(_filterId).items;

  // Let's remove the filter.
  this->dataPtr->filters.erase(_filterId);

  // Remove any reference to this filter inside observedItems.
  for (auto const &oldItem : oldItems)
  {
    auto &filters = this->dataPtr->observedItems[oldItem].filters;
    filters.erase(std::remove(
        filters.begin(), filters.end(), _filterId), filters.end());

    // If there is nobody interested in this item, remove it.
    if (filters.empty())
      this->dataPtr->observedItems.erase(oldItem);
  }

  // Unadvertise topic.

  return true;
}

//////////////////////////////////////////////////
bool IntrospectionManager::Filter(const std::string &_filterId,
    std::vector<std::string> &_items) const
{
  if (this->dataPtr->filters.find(_filterId) == this->dataPtr->filters.end())
    return false;

  _items = this->dataPtr->filters.at(_filterId).items;
  return true;
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
void IntrospectionManager::Update()
{
  for (auto &observedItem : this->dataPtr->observedItems)
  {
    auto &item = observedItem.first;
    auto &lastValue = observedItem.second.lastValue;

    // Update the values of the items under observation.
    auto &callback = this->dataPtr->allItems[item].cb;
    gazebo::msgs::Any value;
    if (!callback(value))
    {
      gzerr << "Something went wrong updating the value for item [" << item
            << "]" << std::endl;
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
      auto nextParam = filter.second.msg.add_param();
      nextParam->CopyFrom(this->dataPtr->observedItems[item].lastValue);
    }
  }
}

std::vector<std::string> IntrospectionManager::RegisteredItems() const
{
  std::vector<std::string> items;
  for (auto item : this->dataPtr->allItems)
    items.push_back(item.first);

  return items;
}

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
