/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <chrono>
#include <map>
#include <set>
#include <thread>

#include <ignition/transport.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/util/IntrospectionClient.hh"

// #include "gazebo/gui/Futures.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/IntrospectionCurveHandler.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the IntrospectionCurveHandler class.
    class IntrospectionCurveHandlerPrivate
    {
      /// \def CurveVariableMapIt
      /// \brief Curve variable map iterator
      public: using CurveVariableMapIt =
          std::map<std::string, CurveVariableSet>::iterator;

      /// \brief Mutex to protect the introspection updates.
      public: std::recursive_mutex mutex;

      /// \brief A map of variable names to plot curves.
      public: std::map<std::string, CurveVariableSet> curves;

      /// \brief Introspection Client
      public: util::IntrospectionClient introspectClient;

      /// \brief Introspection manager Id
      public: std::string managerId;

      /// \brief Ign transport node.
      public: ignition::transport::Node ignNode;

      /// \brief Introspection thread.
      public: std::unique_ptr<std::thread> introspectThread;

      /// \brief Introspection filter.
      public: std::set<std::string> introspectFilter;

      /// \brief Number of subscribers to an introspection filter.
      public: std::map<std::string, int> introspectFilterCount;

      /// \brief Introspection filter ID.
      public: std::string introspectFilterId;

      /// \brief Introspection filter topic.
      public: std::string introspectFilterTopic;

      /// \brief The sim time variable string registered in the
      /// introspection manager.
      public: std::string simTimeVar;

      /// \brief Flag to indicate that the introspection client is initialized.
      public: bool initialized = false;
    };
  }
}

/////////////////////////////////////////////////
IntrospectionCurveHandler::IntrospectionCurveHandler()
  : dataPtr(new IntrospectionCurveHandlerPrivate())
{
  // set up introspection client in another thread as it blocks on
  // discovery
  this->dataPtr->introspectThread.reset(
      new std::thread(&IntrospectionCurveHandler::SetupIntrospection, this));
}

/////////////////////////////////////////////////
IntrospectionCurveHandler::~IntrospectionCurveHandler()
{
  this->dataPtr->initialized = false;
  this->dataPtr->introspectThread->join();
}

/////////////////////////////////////////////////
void IntrospectionCurveHandler::AddCurve(const std::string &_name,
    PlotCurveWeakPtr _curve)
{
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
    if (!this->dataPtr->initialized)
    {
      gzerr << "Introspection client has not been initialized yet" << std::endl;
      return;
    }
  }

  auto c = _curve.lock();
  if (!c)
    return;

  this->dataPtr->mutex.lock();
  auto it = this->dataPtr->curves.find(_name);
  if (it == this->dataPtr->curves.end())
  {
    // unlock immediately to prevent deadlock in introspection client
    // callback in AddItemToFilter
    this->dataPtr->mutex.unlock();

    auto addItemToFilterCallback = [this, _curve, _name](const bool _result)
    {
      if (!_result)
        return;

      // create entry in map
      CurveVariableSet curveSet;
      curveSet.insert(_curve);
      this->dataPtr->curves[_name] = curveSet;
    };

    this->AddItemToFilter(_name, addItemToFilterCallback);
  }
  else
  {
    auto cIt = it->second.find(_curve);
    if (cIt == it->second.end())
    {
      it->second.insert(_curve);
    }
    this->dataPtr->mutex.unlock();
  }
}

/////////////////////////////////////////////////
void IntrospectionCurveHandler::RemoveCurve(PlotCurveWeakPtr _curve)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  if (!this->dataPtr->initialized)
  {
    gzerr << "Introspection client has not been initialized yet" << std::endl;
    return;
  }

  auto c = _curve.lock();
  if (!c)
    return;

  // find and remove the curve
  for (auto it = this->dataPtr->curves.begin();
      it != this->dataPtr->curves.end(); ++it)
  {
    auto cIt = it->second.find(_curve);
    if (cIt != it->second.end())
    {
      it->second.erase(cIt);
      if (it->second.empty())
      {
        // remove item from introspection filter
        this->RemoveItemFromFilter(it->first);
        this->dataPtr->curves.erase(it);
      }
      return;
    }
  }
}

/////////////////////////////////////////////////
unsigned int IntrospectionCurveHandler::CurveCount() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  unsigned int count = 0;
  for (const auto &it : this->dataPtr->curves)
    count += it.second.size();
  return count;
}

/////////////////////////////////////////////////
bool IntrospectionCurveHandler::Initialized() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->initialized;
}

/////////////////////////////////////////////////
void IntrospectionCurveHandler::SetupIntrospection()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  // TODO re-enable once Futures is integrated
  // Make sure that the managers have been retreived.
  // if (Futures::introspectionClientFuture.valid())
  //   Futures::introspectionClientFuture.get();

  gazebo::util::IntrospectionClient client;
  client.WaitForManagers(std::chrono::seconds(2));

  // Get the managers
  std::set<std::string> managerIds = this->dataPtr->introspectClient.Managers();
  if (managerIds.empty())
  {
    gzerr << "No introspection managers detected." << std::endl;
    return;
  }

  // get the first manager
  this->dataPtr->managerId = *managerIds.begin();

  if (this->dataPtr->managerId.empty())
  {
    gzerr << "Introspection manager ID is empty" << std::endl;
    return;
  }

  this->dataPtr->simTimeVar= "data://world/" + gui::get_world()
      + "?p=time/sim_time";

  if (!this->dataPtr->introspectClient.IsRegistered(
      this->dataPtr->managerId, this->dataPtr->simTimeVar))
  {
    gzerr << "The sim_time item is not registered on the manager.\n";
    return;
  }

  // Let's create a filter for sim_time.
  this->dataPtr->introspectFilter = {this->dataPtr->simTimeVar};
  this->dataPtr->introspectFilterCount[this->dataPtr->simTimeVar] = 1;
  if (!this->dataPtr->introspectClient.NewFilter(
      this->dataPtr->managerId, this->dataPtr->introspectFilter,
      this->dataPtr->introspectFilterId, this->dataPtr->introspectFilterTopic))
  {
    gzerr << "Unable to create introspection filter" << std::endl;
    return;
  }

  // Subscribe to custom introspection topic for receiving updates.
  if (!this->dataPtr->ignNode.Subscribe(this->dataPtr->introspectFilterTopic,
      &IntrospectionCurveHandler::OnIntrospection, this))
  {
    gzerr << "Error subscribing to introspection manager" << std::endl;
    return;
  }

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void IntrospectionCurveHandler::OnIntrospection(
    const gazebo::msgs::Param_V &_msg)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  // stores a list of curves iterators and their new values
  std::vector<
      std::pair<IntrospectionCurveHandlerPrivate::CurveVariableMapIt, double> >
      curvesUpdates;

  // collect data for updates
  double simTime = 0;
  bool hasSimTime = false;
  for (auto i = 0; i < _msg.param_size(); ++i)
  {
    auto param = _msg.param(i);
    if (param.name().empty() || !param.has_value())
      continue;

    std::string paramName = param.name();
    auto paramValue = param.value();

    // x axis is hardcoded to sim time for now
    if (!hasSimTime && paramName == this->dataPtr->simTimeVar)
    {
      if (paramValue.has_time_value())
      {
        common::Time t = msgs::Convert(paramValue.time_value());
        simTime = t.Double();
        hasSimTime = true;
      }
    }

    // see if there is a curve with variable name that matches param name or
    // a substring of the param name
    for (auto cIt = this->dataPtr->curves.begin();
        cIt != this->dataPtr->curves.end(); ++cIt)
    {
      if (cIt->first.find(paramName) != 0)
        continue;

      // get the data
      double data = 0;
      bool validData = true;
      std::string curveVarName = cIt->first;

      switch (paramValue.type())
      {
        case gazebo::msgs::Any::DOUBLE:
        {
          if (paramValue.has_double_value())
          {
            data = paramValue.double_value();
          }
          break;
        }
        case gazebo::msgs::Any::INT32:
        {
          if (paramValue.has_int_value())
          {
            data = paramValue.int_value();
          }
          break;
        }
        case gazebo::msgs::Any::BOOLEAN:
        {
          if (paramValue.has_bool_value())
          {
            data = static_cast<int>(paramValue.bool_value());
          }
          break;
        }
        case gazebo::msgs::Any::TIME:
        {
          if (paramValue.has_time_value())
          {
            common::Time t = msgs::Convert(paramValue.time_value());
            data = t.Double();
          }
          break;
        }
        case gazebo::msgs::Any::POSE3D:
        {
          if (paramValue.has_pose3d_value())
          {
            ignition::math::Pose3d p =
                msgs::ConvertIgn(paramValue.pose3d_value());

            double d = 0;
            // use uri to parse and get specific attribute
            common::URI uri(curveVarName);
            common::URIQuery query = uri.Query();
            std::string queryStr = query.Str();

            // example position query string:
            //   p=pose3d/world_pose/vector3d/position/double/x
            // example rotation query string:
            //   p=pose3d/world_pose/quaterniond/orientation/double/roll
            if (queryStr.find("position") != std::string::npos)
            {
              validData = this->Vector3dFromQuery(queryStr, p.Pos(), d);
            }
            else if (queryStr.find("orientation") != std::string::npos)
            {
              validData = this->QuaterniondFromQuery(queryStr, p.Rot(), d);
            }
            else
              validData = false;

            data = d;
          }
          else
            validData = false;
          break;
        }
        case gazebo::msgs::Any::VECTOR3D:
        {
          if (paramValue.has_vector3d_value())
          {
            ignition::math::Vector3d vec =
                msgs::ConvertIgn(paramValue.vector3d_value());

            double d = 0;
            // use uri to parse and get specific attribute
            common::URI uri(curveVarName);
            common::URIQuery query = uri.Query();
            std::string queryStr = query.Str();
            validData = this->Vector3dFromQuery(queryStr, vec, d);

            data = d;
          }
          else
            validData = false;
          break;
        }
        case gazebo::msgs::Any::QUATERNIOND:
        {
          if (paramValue.has_quaternion_value())
          {
            ignition::math::Quaterniond quat =
                msgs::ConvertIgn(paramValue.quaternion_value());

            double d = 0;
            // use uri to parse and get specific attribute
            common::URI uri(curveVarName);
            common::URIQuery query = uri.Query();
            std::string queryStr = query.Str();
            validData = this->QuaterniondFromQuery(queryStr, quat, d);

            data = d;
          }
          else
            validData = false;

          break;
        }
        default:
        {
          validData = false;
          break;
        }
      }
      if (!validData)
        continue;

      // push to tmp list and update later
      curvesUpdates.push_back(std::make_pair(cIt, data));
    }
  }

  // update curves!
  for (auto &curveUpdate : curvesUpdates)
  {
    for (auto cIt : curveUpdate.first->second)
    {
      auto curve = cIt.lock();
      if (!curve)
        continue;
      curve->AddPoint(ignition::math::Vector2d(simTime, curveUpdate.second));
    }
  }
}

/////////////////////////////////////////////////
void IntrospectionCurveHandler::AddItemToFilter(const std::string &_name,
    const std::function<void(const bool _result)> &_cb)
{
  // callback from a async items service request
  auto itemsCallback = [this, _name, _cb](const std::set<std::string> &_items,
      const bool _itemsResult)
  {
    if (!_itemsResult)
      return;

    std::lock_guard<std::recursive_mutex> itemLock(this->dataPtr->mutex);

    common::URI itemURI(_name);
    common::URIPath itemPath = itemURI.Path();
    common::URIQuery itemQuery = itemURI.Query();

    for (auto item : _items)
    {
      common::URI uri(item);
      common::URIPath path = uri.Path();
      common::URIQuery query = uri.Query();

      // check if the entity matches
      if (itemPath == path)
      {
        // A registered variable can have the query
        //  "?p=pose3d/world_pose"
        // and if the variable we are looking for has the query
        //  "?p=pose3d/world_pose/vector3d/position/double/x"
        // we need to add "scheme://path?pose3d/world_pose" to filter instead of
        //  "scheme://path?p=pose3d/world_pose/vector3d/position/double/x"

        // check substring
        if (itemQuery.Str().find(query.Str()) == 0)
        {
          // add item to introspection filter
          if (this->dataPtr->introspectFilter.find(item) ==
              this->dataPtr->introspectFilter.end())
          {
            auto filterCopy = this->dataPtr->introspectFilter;
            filterCopy.insert(item);

            // callback to update the filter and curve map if the
            // async service request is successful
            auto filterUpdateCallback = [this, item, _cb](const bool _result)
            {
              if (!_result)
                return;

              std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
              this->dataPtr->introspectFilter.insert(item);
              this->dataPtr->introspectFilterCount[item] = 1;
              if (_cb)
                _cb(_result);
            };

            // Update the filter. We're interested on "item1" and "item2".
            if (!this->dataPtr->introspectClient.UpdateFilter(
                this->dataPtr->managerId, this->dataPtr->introspectFilterId,
                filterCopy, filterUpdateCallback))
            {
              gzerr << "Error updating introspection filter" << std::endl;
              return;
            }
          }
          else
          {
            // filter already exists, increment counter.
            int &count = this->dataPtr->introspectFilterCount[item];
            count++;
            if (_cb)
              _cb(true);
          }

          break;
        }
      }
    }
  };

  common::URI itemURI(_name);
  if (!itemURI.Valid())
    return;

  this->dataPtr->introspectClient.Items(
      this->dataPtr->managerId, itemsCallback);
}

/////////////////////////////////////////////////
void IntrospectionCurveHandler::RemoveItemFromFilter(const std::string &_name,
    const std::function<void(const bool _result)> &_cb)
{
  // callback from a async items service request
  auto itemsCallback = [this, _name, _cb](const std::set<std::string> &_items,
      const bool _itemsResult)
  {
    if (!_itemsResult)
      return;

    std::lock_guard<std::recursive_mutex> itemLock(this->dataPtr->mutex);

    common::URI itemURI(_name);
    common::URIPath itemPath = itemURI.Path();
    common::URIQuery itemQuery = itemURI.Query();

    for (auto item : _items)
    {
      common::URI uri(item);
      common::URIPath path = uri.Path();
      common::URIQuery query = uri.Query();

      // check if the entity matches
      if (itemPath == path)
      {
        // A registered variable can have the query
        //  "?p=pose3d/world_pose"
        // and if the variable we are looking for has the query
        //  "?p=pose3d/world_pose/vector3d/position/double/x"
        // we need to remove "scheme://path?pose3d/world_pose" from the filter
        // instead of
        //  "scheme://path?p=pose3d/world_pose/vector3d/position/double/x"

        // check substring starts at index 0
        if (itemQuery.Str().find(query.Str()) == 0)
        {
          // check if it is in the filter list
          auto itemIt = this->dataPtr->introspectFilter.find(item);
          if (itemIt == this->dataPtr->introspectFilter.end())
            continue;

          // update filter only if no one else is subscribed to it
          int &count = this->dataPtr->introspectFilterCount[item];
          count--;
          if (count <= 0)
          {
            auto filterCopy = this->dataPtr->introspectFilter;
            filterCopy.erase(item);

            // callback for updating the filter and curve map if the
            // async service request is successful
            auto filterUpdateCallback = [this, item, _cb](const bool _result)
            {
              if (!_result)
                return;

              std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
              this->dataPtr->introspectFilter.erase(item);
              this->dataPtr->introspectFilterCount.erase(item);
            };

            // update filter
            if (!this->dataPtr->introspectClient.UpdateFilter(
                this->dataPtr->managerId, this->dataPtr->introspectFilterId,
                filterCopy, filterUpdateCallback))
            {
              gzerr << "Error updating introspection filter" << std::endl;
              return;
            }
          }

          if (_cb)
            _cb(true);
          break;
        }
      }
    }
  };

  common::URI itemURI(_name);

  if (!itemURI.Valid())
    return;

  this->dataPtr->introspectClient.Items(
      this->dataPtr->managerId, itemsCallback);
}

/////////////////////////////////////////////////
bool IntrospectionCurveHandler::Vector3dFromQuery(const std::string &_query,
    const ignition::math::Vector3d &_vec, double &_value) const
{
  std::string elem = _query.substr(_query.size()-1);
  if (elem == "x")
  {
    _value = _vec.X();
  }
  else if (elem == "y")
  {
    _value = _vec.Y();
  }
  else if (elem == "z")
  {
    _value = _vec.Z();
  }
  else
    return false;

  return true;
}

/////////////////////////////////////////////////
bool IntrospectionCurveHandler::QuaterniondFromQuery(const std::string &_query,
    const ignition::math::Quaterniond &_quat, double &_value) const
{
  ignition::math::Vector3d euler = _quat.Euler();
  if (_query.find("roll") != std::string::npos)
  {
    _value = euler.X();
  }
  else if (_query.find("pitch") != std::string::npos)
  {
    _value = euler.Y();
  }
  else if (_query.find("yaw") != std::string::npos)
  {
    _value = euler.Z();
  }
  else
    return false;

  return true;
}
