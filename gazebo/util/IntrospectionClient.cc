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

#include <set>
#include <string>
#include "gazebo/common/Console.hh"
#include "gazebo/msgs/any.pb.h"
#include "gazebo/msgs/empty.pb.h"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/msgs/param.pb.h"
#include "gazebo/msgs/param_v.pb.h"
#include "gazebo/util/IntrospectionClient.hh"
#include "gazebo/util/IntrospectionClientPrivate.hh"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
IntrospectionClient::IntrospectionClient()
  : dataPtr(new IntrospectionClientPrivate)
{
}

//////////////////////////////////////////////////
IntrospectionClient::~IntrospectionClient()
{
  this->RemoveAllFilters();
}

//////////////////////////////////////////////////
std::set<std::string> IntrospectionClient::WaitForManagers(
    const std::chrono::milliseconds _timeOut) const
{
  std::set<std::string> result;
  result = this->Managers();

  auto dur = std::chrono::milliseconds(500);
  std::chrono::milliseconds slept(0);

  while (result.empty() && (_timeOut == std::chrono::milliseconds::zero() ||
                            slept < _timeOut))
  {
    std::this_thread::sleep_for(dur);
    slept += dur;
    result = this->Managers();
  }

  return result;
}

//////////////////////////////////////////////////
std::set<std::string> IntrospectionClient::Managers() const
{
  std::vector<std::string> availableServices;
  this->dataPtr->node.ServiceList(availableServices);

  // We are looking for services that follow this convention:
  // /introspection/<manager_id>/<service_name>
  std::set<std::string> managers;
  const std::string kStartDelim = "/introspection/";
  auto from = kStartDelim.size();
  for (auto const &service : availableServices)
  {
    if (service.find(kStartDelim) == 0)
    {
      // Find the next "/".
      auto to = service.find("/", from);
      if (to != std::string::npos)
      {
        auto id = service.substr(from, to - from);
        managers.emplace(id);
      }
    }
  }
  return managers;
}

//////////////////////////////////////////////////
bool IntrospectionClient::NewFilter(const std::string &_managerId,
    const std::set<std::string> &_newItems,
    std::string &_filterId, std::string &_newTopic) const
{
  gazebo::msgs::Param_V req;
  gazebo::msgs::GzString rep;
  bool result;

  // Add to the message the list of items to include in the filter.
  for (auto const &itemName : _newItems)
  {
    auto nextParam = req.add_param();
    nextParam->set_name("item");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
    nextParam->mutable_value()->set_string_value(itemName);
  }

  // Request the service.
  auto service = "/introspection/" + _managerId + "/filter_new";
  if (!this->dataPtr->node.Request(service, req,
          this->dataPtr->kTimeout, rep, result))
  {
    gzerr << "Unable to create remote introspection filter" << std::endl;
    return false;
  }

  if (!result)
    return false;

  _filterId = rep.data();
  _newTopic = "/introspection/" + _managerId + "/filter/" + _filterId;

  // Save the new filter ID.
  this->dataPtr->filters[_filterId] = _managerId;
  return true;
}

//////////////////////////////////////////////////
bool IntrospectionClient::UpdateFilter(const std::string &_managerId,
    const std::string &_filterId, const std::set<std::string> &_newItems) const
{
  gazebo::msgs::Param_V req;
  gazebo::msgs::Empty rep;
  bool result;

  // Add the filter_id to the message.
  auto nextParam = req.add_param();
  nextParam->set_name("filter_id");
  nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
  nextParam->mutable_value()->set_string_value(_filterId);

  // Add to the message the list of items to include in the filter.
  for (auto const &itemName : _newItems)
  {
    nextParam = req.add_param();
    nextParam->set_name("item");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
    nextParam->mutable_value()->set_string_value(itemName);
  }

  // Request the service.
  auto service = "/introspection/" + _managerId + "/filter_update";
  if (!this->dataPtr->node.Request(service, req,
          this->dataPtr->kTimeout, rep, result))
  {
    gzerr << "Unable to update a remote introspection filter" << std::endl;
    return false;
  }

  return result;
}

//////////////////////////////////////////////////
bool IntrospectionClient::RemoveAllFilters() const
{
  bool result = true;

  // Remove all the filters from the manager.
  for (auto const &filter : this->dataPtr->filters)
    result = result && this->RemoveFilter(filter.second, filter.first);

  return result;
}

//////////////////////////////////////////////////
bool IntrospectionClient::RemoveFilter(const std::string &_managerId,
    const std::string &_filterId) const
{
  gazebo::msgs::Param_V req;
  gazebo::msgs::Empty rep;
  bool result;

  // Add the filter_id to the message.
  auto nextParam = req.add_param();
  nextParam->set_name("filter_id");
  nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
  nextParam->mutable_value()->set_string_value(_filterId);

  // Request the service.
  auto service = "/introspection/" + _managerId + "/filter_remove";
  if (!this->dataPtr->node.Request(service, req,
          this->dataPtr->kTimeout, rep, result))
  {
    gzerr << "Unable to remove a remote introspection filter" << std::endl;
    return false;
  }

  if (!result)
    return false;

  // Remove this filter from our internal list.
  this->dataPtr->filters.erase(_filterId);
  return true;
}

//////////////////////////////////////////////////
bool IntrospectionClient::Items(const std::string &_managerId,
    std::set<std::string> &_items) const
{
  gazebo::msgs::Empty req;
  gazebo::msgs::Param_V rep;
  bool result;

  // Request the service.
  auto service = "/introspection/" + _managerId + "/items";
  if (!this->dataPtr->node.Request(service, req,
          this->dataPtr->kTimeout, rep, result))
  {
    gzerr << "Unable to get list of items from a remote manager" << std::endl;
    return false;
  }

  for (auto i = 0; i < rep.param_size(); ++i)
  {
    auto param = rep.param(i);
    _items.emplace(param.value().string_value());
  }
  return true;
}

//////////////////////////////////////////////////
bool IntrospectionClient::IsRegistered(const std::string &_managerId,
    const std::string &_item) const
{
  std::set<std::string> items;

  if (this->Items(_managerId, items))
  {
    if (items.find(_item) != items.end())
      return true;
  }

  return false;
}

//////////////////////////////////////////////////
bool IntrospectionClient::IsRegistered(const std::string &_managerId,
    const std::set<std::string> &_items) const
{
  std::set<std::string> items;

  if (this->Items(_managerId, items))
  {
    for (auto const &item : _items)
    {
      if (items.find(item) == items.end())
        return false;
    }
    return true;
  }

  return false;
}
