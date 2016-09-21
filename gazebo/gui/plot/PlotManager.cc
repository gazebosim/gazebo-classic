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

#include <memory>
#include <mutex>


#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/gui/plot/IntrospectionCurveHandler.hh"
#include "gazebo/gui/plot/TopicCurveHandler.hh"
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
      /// \brief Mutex to protect plot manager updates.
      public: std::mutex mutex;

      /// \brief Node for communications.
      public: transport::NodePtr node;

      /// \brief Subscriber to the world control topic
      public: transport::SubscriberPtr worldControlSub;

      /// \brief Handler for updating introspection curves
      public: IntrospectionCurveHandler introspectionCurve;

      /// \brief Handler for updating topic curves
      public: TopicCurveHandler topicCurve;

      /// \brief A list of plot windows.
      public: std::vector<PlotWindow *> windows;
    };
  }
}

/////////////////////////////////////////////////
PlotManager::PlotManager()
  : dataPtr(new PlotManagerPrivate())
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  // check for reset events and restart plots when time is reset
  this->dataPtr->worldControlSub =
      this->dataPtr->node->Subscribe("~/world_control",
      &PlotManager::OnWorldControl, this);
}

/////////////////////////////////////////////////
PlotManager::~PlotManager()
{
  this->dataPtr->windows.clear();
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
void PlotManager::AddIntrospectionCurve(const std::string &_uri,
    PlotCurveWeakPtr _curve)
{
  this->dataPtr->introspectionCurve.AddCurve(_uri, _curve);
}

/////////////////////////////////////////////////
void PlotManager::RemoveIntrospectionCurve(PlotCurveWeakPtr _curve)
{
  this->dataPtr->introspectionCurve.RemoveCurve(_curve);
}

/////////////////////////////////////////////////
void PlotManager::AddTopicCurve(const std::string &_topic,
    PlotCurveWeakPtr _curve)
{
  this->dataPtr->topicCurve.AddCurve(_topic, _curve);
}

/////////////////////////////////////////////////
void PlotManager::RemoveTopicCurve(PlotCurveWeakPtr _curve)
{
  this->dataPtr->topicCurve.RemoveCurve(_curve);
}

/////////////////////////////////////////////////
void PlotManager::AddWindow(PlotWindow *_window)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->windows.push_back(_window);
}

/////////////////////////////////////////////////
void PlotManager::RemoveWindow(PlotWindow *_window)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
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
std::string PlotManager::HumanReadableName(const std::string &_uri) const
{
  std::string label;

  // expected name format:
  //   scheme: data://
  //   path:   world/world_name/model/model_name/link/link_name
  //   query:  ?p=param_type/param_name
  // convert to friendly name:
  //   name:   model_name/link_name?param_name
  common::URI uri(_uri);
  if (!uri.Valid())
    return _uri;

  common::URIPath path = uri.Path();
  common::URIQuery query = uri.Query();
  std::vector<std::string> pathTokens = common::split(path.Str(), "/");
  std::vector<std::string> queryTokens = common::split(query.Str(), "=/");

  // min path token size 2: [world, world_name]
  // min query token size 3: [p, param_type, param_name]
  if (pathTokens.size() < 2 || queryTokens.size() < 3)
    return label;

  // path: start from model name and ignore world and entity type str for now
  std::string pathStr;
  for (unsigned int i = 3; i < pathTokens.size(); i+=2)
  {
    if (!pathStr.empty())
      pathStr += "/";
    pathStr += pathTokens[i];
  }

  // query: take only first param name
  std::string queryStr;
  for (unsigned int i = 2; i < queryTokens.size(); i+=2)
  {
    if (!queryStr.empty())
      queryStr +="/";
    queryStr += queryTokens[i];
  }
  label = pathStr + "?" + queryStr;

  return label;
}
