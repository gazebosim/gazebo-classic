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

//#include "gazebo/transport/Node.hh"
//#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/TransportIface.hh"

//#include "gazebo/gui/plot/VariablePillContainer.hh"
//#include "gazebo/gui/plot/PlotCanvas.hh"
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
      /// \brief Node for communications.
      public: transport::NodePtr node;

      /// \brief Subscriber to the world control topic
      public: transport::SubscriberPtr worldControlSub;
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
//      this->ResetPlots();
    }
  }
}
