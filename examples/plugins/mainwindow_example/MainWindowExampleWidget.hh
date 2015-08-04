/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _MAIN_WINDOW_EXAMPLE_WIDGET_HH_
#define _MAIN_WINDOW_EXAMPLE_WIDGET_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE MainWindowExampleWidget : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: MainWindowExampleWidget();

    /// \brief Destructor
    public: virtual ~MainWindowExampleWidget();

    /// \brief Callback function to pause the world.
    public: void PauseWorld();

    /// \brief A signal used to set the sim time line edit.
    /// \param[in] _string String representation of sim time.
    signals: void SetSimTime(QString _string);

    /// \brief Callback that received world statistics messages.
    /// \param[in] _msg World statistics message that is received.
    protected: void OnStats(ConstWorldStatisticsPtr &_msg);

    /// \brief Helper function to format time string.
    /// \param[in] _msg Time message.
    /// \return Time formatted as a string.
    private: std::string FormatTime(const msgs::Time &_msg) const;

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;

    /// \brief Subscriber to world statistics messages.
    private: transport::SubscriberPtr statsSub;

    /// \brief The event connections for this plugin.
    private: std::vector<event::ConnectionPtr> connections;
  };
}

#endif
