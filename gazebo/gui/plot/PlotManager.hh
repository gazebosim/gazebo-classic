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
#ifndef GAZEBO_GUI_PLOT_PLOTMANAGER_HH_
#define GAZEBO_GUI_PLOT_PLOTMANAGER_HH_

#include <memory>
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/util/system.hh"

/// \brief Explicit instantiation for typed SingletonT.
GZ_SINGLETON_DECLARE(GZ_GUI_VISIBLE, gazebo, gui, PlotManager)

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

      /// \brief Callback when a world control message is received over
      /// gazebo_transport using boost asio. It is used
      /// to detect simulation resets.
      /// \param[in] _data Message data containing world control commands
      public: void OnWorldControl(ConstWorldControlPtr &_data);

      /// \brief Add an introspection curve to the manager. Data received from
      /// the introspection client will be added to the curve
      /// \param[in] _name Name of variable
      /// \param[in] _curve Curve that will be populated with data.
      public: void AddIntrospectionCurve(const std::string &_name,
          PlotCurveWeakPtr _curve);

      /// \brief Remove an introspection curve from the manager
      /// \param[in] _curve Curve to remove.
      public: void RemoveIntrospectionCurve(PlotCurveWeakPtr _curve);

      /// \brief Add a topic curve to the manager. Data received from
      /// the topic subscriber will be added to the curve
      /// \param[in] _topic Name of topic
      /// \param[in] _curve Curve that will be populated with data.
      public: void AddTopicCurve(const std::string &_topic,
          PlotCurveWeakPtr _curve);

      /// \brief Remove a topic curve from the manager
      /// \param[in] _curve Curve to remove.
      public: void RemoveTopicCurve(PlotCurveWeakPtr _curve);

      /// \brief Add a plot window to the manager. The manager will listen to
      /// world events, e.g. Reset, and update the window's plots accordingly
      /// \param[in] _window Plot window to add
      public: void AddWindow(PlotWindow *_window);

      /// \brief Remove a plot window from the manager.
      /// \param[in] _window Plot window to remove.
      public: void RemoveWindow(PlotWindow *_window);

      /// \brief Get Human-readable name from uri-formatted variable name
      /// \param[in] _uri URI representing the variable
      /// \return Human readable name
      public: std::string HumanReadableName(const std::string &_uri) const;

      /// \brief Returns a pointer to the unique (static) instance
      public: static PlotManager* Instance();

      /// \brief Callback when a world control message is received over
      /// gz-transport using ZeroMQ. It is used to detect simulation resets.
      /// \param[in] _data Message data containing world control commands
      private: void OnControl(const msgs::WorldControl &_data);

      /// \brief This is a singleton class.
      private: friend class SingletonT<PlotManager>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotManagerPrivate> dataPtr;
    };
  }
}
#endif
