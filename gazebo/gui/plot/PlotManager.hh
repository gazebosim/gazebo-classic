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

#ifndef _GAZEBO_GUI_PLOT_PLOTMANAGER_HH_
#define _GAZEBO_GUI_PLOT_PLOTMANAGER_HH_

#include <memory>
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotManagerPrivate;

    class PlotWindow;

    /// \brief A class that connects simulation data with the plotting tool
    class GZ_GUI_VISIBLE PlotManager : public SingletonT<PlotManager>
    {
      /// \brief Constructor.
      public: PlotManager();

      /// \brief Destructor.
      public: virtual ~PlotManager();

      /// \brief Callback when a world control message is received. It is used
      /// to detect simulation resets.
      /// \param[in] _data Message data containing world control commands
      public: void OnWorldControl(ConstWorldControlPtr &_data);

      /// \brief Add a curve to the manager. Data received from the named topic
      /// will be added to the curve
      /// \param[in] _name Name of topic
      /// \param[in] _curve Curve that will be populated with data.
      public: void AddCurve(const std::string &_name,
          PlotCurveWeakPtr _curve);

      /// \brief Remove a curve from the manager
      /// \param[in] _curve Curve to remove.
      public: void RemoveCurve(PlotCurveWeakPtr _curve);

      /// \brief Add a plot window to the manager. The manager will listen to
      /// world events, e.g. Reset, and update the window's plots accordingly
      /// \param[in] _window Plot window to add
      public: void AddWindow(PlotWindow *_window);

      /// \brief Remove a plot window from the manager.
      /// \param[in] _window Plot window to remove.
      public: void RemoveWindow(PlotWindow *_window);

      /// TODO remove me
      /// \brief Callback when a world control message is received. It is used
      /// to detect simulation resets.
      /// \param[in] _data Message data containing world stats msgs
      public: void OnWorldStats(ConstWorldStatisticsPtr &_data);

      /// \brief Set up introspection client
      public: void SetupIntrospection();

      /// \brief Function called each time a topic update is received.
      /// \param[in] _msg Introspection message filled with param data
      public: void OnIntrospection(const gazebo::msgs::Param_V &_msg);

      /// \brief This is a singleton class.
      private: friend class SingletonT<PlotManager>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotManagerPrivate> dataPtr;
    };
  }
}
#endif
