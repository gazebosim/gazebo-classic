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

#include "gazebo/transport/TransportIface.hh"

#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/PlotManager.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the PlotManager class
    struct PlotManagerPrivate
    {
      /// \def CurveVariableSet
      /// \brief A set of unique plot curve pointers
      using CurveVariableSet = std::set<PlotCurveWeakPtr,
          std::owner_less<PlotCurveWeakPtr> >;

      /// \brief Node for communications.
      public: transport::NodePtr node;

      /// \brief Subscriber to the world control topic
      public: transport::SubscriberPtr worldControlSub;

      /// TODO remove me later
      /// \brief Subscriber to the world stats topic
      public: transport::SubscriberPtr worldStatsSub;

      /// \brief A map to variable topic names to plot curve.
      public: std::map<std::string, CurveVariableSet> curves;

      /// \brief Mutex to protect the
      public: std::mutex mutex;
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
}

/////////////////////////////////////////////////
PlotManager::~PlotManager()
{
}

/////////////////////////////////////////////////
void PlotManager::OnWorldControl(ConstWorldControlPtr &_data)
{
  if (_data->has_reset())
  {
    if ((_data->reset().has_all() && _data->reset().all())  ||
        (_data->reset().has_time_only() && _data->reset().time_only()))
    {
      // reset plots
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
}
