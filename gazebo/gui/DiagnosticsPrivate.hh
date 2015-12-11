/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_DIAGNOSTICS_PRIVATE_HH_
#define _GAZEBO_DIAGNOSTICS_PRIVATE_HH_

#include <list>
#include <map>
#include <mutex>
#include <vector>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class IncrementalPlot;

    /// \brief public data for the EntityMaker class
    class EntityMakerpublic
    {
      /// \def PointMap
      public: using PointMap = std::map<QString, std::list<QPointF>>;

      /// \brief Node for communications.
      public: transport::NodePtr node;

      /// \brief Subscribes to diagnostic info.
      public: transport::SubscriberPtr sub;

      /// \brief The list of diagnostic labels.
      public: QListWidget *labelList;

      /// \brief The currently selected label.
      public: PointMap selectedLabels;

      /// \brief True when plotting is paused.
      public: bool paused;

      /// \brief Mutex to protect the point map
      public: std::mutex mutex;

      /// \brief Plotting widget
      public: std::vector<IncrementalPlot *> plots;

      /// \brief Layout to hold all the plots.
      public: QVBoxLayout *plotLayout;
    };
  }
}
#endif

