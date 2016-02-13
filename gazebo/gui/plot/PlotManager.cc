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

#include <map>
#include <mutex>
#include <chrono>
#include <thread>

#include <ignition/transport.hh>

#include "gazebo/util/IntrospectionClient.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/PlotWindow.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/PlotManager.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the PlotManager class
    class PlotManagerPrivate
    {
      /// \def CurveVariableSet
      /// \brief A set of unique plot curve pointers
      public: using CurveVariableSet = std::set<PlotCurveWeakPtr,
          std::owner_less<PlotCurveWeakPtr> >;

      /// \def CurveVariableSetIt
      /// \brief Curve variable map iterator
      public: using CurveVariableMapIt =
          std::map<std::string, CurveVariableSet>::iterator;

      /// \brief Node for communications.
      public: transport::NodePtr node;

      /// \brief Subscriber to the world control topic
      public: transport::SubscriberPtr worldControlSub;

      /// TODO remove me later
      /// \brief Subscriber to the world stats topic
      public: transport::SubscriberPtr worldStatsSub;

      /// \brief A map to variable topic names to plot curve.
      public: std::map<std::string, CurveVariableSet> curves;

      /// \brief A list of plot windows.
      public: std::vector<PlotWindow *> windows;

      /// \brief Mutex to protect the
      public: std::mutex mutex;

      /// \brief Introspection Client
      public: util::IntrospectionClient introspectClient;

      /// \brief Introspection manager Id
      public: std::string managerId;

      /// \brief Ign transport node.
      public: ignition::transport::Node ignNode;

      /// \brief Introspection thread.
      public: std::unique_ptr<std::thread> introspectThread;
    };
  }
}

/////////////////////////////////////////////////
PlotManager::PlotManager()
  : dataPtr(new PlotManagerPrivate())
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();


  this->dataPtr->worldControlSub =
      this->dataPtr->node->Subscribe("~/world_control",
      &PlotManager::OnWorldControl, this);

  // TODO remove me later. Testing!
  this->dataPtr->worldStatsSub =
      this->dataPtr->node->Subscribe("~/world_stats",
      &PlotManager::OnWorldStats, this);

  // set up introspection client in an other thread as it blocks on
  // discovery
  this->dataPtr->introspectThread.reset(
      new std::thread(&PlotManager::SetupIntrospection, this));
}

/////////////////////////////////////////////////
PlotManager::~PlotManager()
{
  this->dataPtr->introspectThread->join();
}

/////////////////////////////////////////////////
void PlotManager::SetupIntrospection()
{
  // Wait for the managers to come online
  std::set<std::string> managerIds =
      this->dataPtr->introspectClient.WaitForManagers(std::chrono::seconds(2));
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

  if (!this->dataPtr->introspectClient.IsRegistered(
      this->dataPtr->managerId, "sim_time"))
  {
    gzerr << "The sim_time item is not registered on the manager.\n";
    return;
  }

  std::string filterId;
  std::string topic;

  // Let's create a filter for sim_time.
  std::set<std::string> items = {"sim_time"};
  if (!this->dataPtr->introspectClient.NewFilter(
      this->dataPtr->managerId, items, filterId, topic))
  {
    gzerr << "Unable to create introspection filter" << std::endl;
    return;
  }

  // Subscribe to custom introspection topic for receiving updates.
  if (!this->dataPtr->ignNode.Subscribe(
      topic, &PlotManager::OnIntrospection, this))
  {
    gzerr << "Error subscribing to introspection manager" << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void PlotManager::OnWorldControl(ConstWorldControlPtr &_data)
{
  if (_data->has_reset())
  {
    if ((_data->reset().has_all() && _data->reset().all())  ||
        (_data->reset().has_time_only() && _data->reset().time_only()))
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
      for (auto &w : this->dataPtr->windows)
        w->Restart();
    }
  }
}

/////////////////////////////////////////////////
void PlotManager::AddCurve(const std::string &_name, PlotCurveWeakPtr _curve)
{
  if (_curve.expired())
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->curves.find(_name);
  if (it == this->dataPtr->curves.end())
  {
    PlotManagerPrivate::CurveVariableSet curveSet;
    curveSet.insert(_curve);
    this->dataPtr->curves[_name] = curveSet;
  }
  else
  {
    auto cIt = it->second.find(_curve);
    if (cIt == it->second.end())
    {
      it->second.insert(_curve);
    }
  }
}

/////////////////////////////////////////////////
void PlotManager::RemoveCurve(PlotCurveWeakPtr _curve)
{
  if (_curve.expired())
    return;

  // find and remove the curve
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (auto it = this->dataPtr->curves.begin();
      it != this->dataPtr->curves.end(); ++it)
  {
    auto cIt = it->second.find(_curve);
    if (cIt != it->second.end())
    {
      it->second.erase(cIt);
      return;
    }
  }
}

/////////////////////////////////////////////////
void PlotManager::AddWindow(PlotWindow *_window)
{
  this->dataPtr->windows.push_back(_window);
}

/////////////////////////////////////////////////
void PlotManager::RemoveWindow(PlotWindow *_window)
{
  for (auto it = this->dataPtr->windows.begin();
      it != this->dataPtr->windows.end(); ++it)
  {
    if ((*it) == _window)
    {
      this->dataPtr->windows.erase(it);
      return;
    }
  }
}

/////////////////////////////////////////////////
void PlotManager::OnWorldStats(ConstWorldStatisticsPtr &/*_data*/)
{
  // TODO only for testing purposes
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  static double testTime = 0;
  testTime += 0.1;
  auto it = this->dataPtr->curves.find("Dog");
  if (it != this->dataPtr->curves.end())
  {
    for (auto cIt : it->second)
    {
      auto curve = cIt.lock();
      if (!curve)
        continue;
      curve->AddPoint(ignition::math::Vector2d(testTime, 2));
    }
  }

  it = this->dataPtr->curves.find("Cat");
  if (it != this->dataPtr->curves.end())
  {
    for (auto cIt : it->second)
    {
      auto curve = cIt.lock();
      if (!curve)
        continue;
      curve->AddPoint(ignition::math::Vector2d(testTime, 10));
    }
  }

  it = this->dataPtr->curves.find("Turtle");
  if (it != this->dataPtr->curves.end())
  {
    for (auto cIt : it->second)
    {
      auto curve = cIt.lock();
      if (!curve)
        continue;
      curve->AddPoint(ignition::math::Vector2d(testTime, 6));
    }
  }
}

/////////////////////////////////////////////////
void PlotManager::OnIntrospection(const gazebo::msgs::Param_V &_msg)
{
  // stores a list of curves iterators and their new values
  std::vector<std::pair<PlotManagerPrivate::CurveVariableMapIt, double> >
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
    if (!hasSimTime && param.name() == "sim_time")
    {
      simTime = paramValue.double_value();
      hasSimTime = true;
    }

    // see if there is a curve with variable name that matches param name
    auto it = this->dataPtr->curves.find(param.name());
    if (it == this->dataPtr->curves.end())
      continue;

    // get the data
    double data = 0;
    bool validData = true;
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
      default:
      {
        validData = false;
        break;
      }
    }
    if (!validData)
      continue;
    // push to tmp list and update later
    curvesUpdates.push_back(std::make_pair(it, data));
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
