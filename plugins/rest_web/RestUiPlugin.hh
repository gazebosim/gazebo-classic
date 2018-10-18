/*
 * Copyright 2015 Open Source Robotics Foundation
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

#ifndef _REST_UI_PLUGIN_HH_
#define _REST_UI_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/gui/qt.h>
#include <gazebo/util/system.hh>

#include "RestUiWidget.hh"

namespace gazebo
{
  /// \class RestUiPlugin RestUiPlugin.hh RestUiPlugin.hh
  /// \brief REST user interface plugin
  class GAZEBO_VISIBLE RestUiPlugin : public SystemPlugin
  {
    /// \brief Constructor
    public: RestUiPlugin();

    /// \brief Destructor
    public: virtual ~RestUiPlugin() = default;

    /// \brief Called when plugin is loaded.
    /// \param[in] _argc Arguments count
    /// \param[in] _argv Argument vector
    public: virtual void Load(int _argc, char **_argv);

    /// \brief Plugin initialization.
    private: virtual void Init();

    /// \brief Called by Gazebo after the main window has been setup.
    private: void OnMainWindowReady();

    /// \brief Called from the GUI thread before rendering.
    private: void Update();

    /// \brief Callbacks (to connect to the main window ready event).
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief The Title, used for the menu item.
    private: std::string menuTitle;

    /// \brief The login title.
    private: std::string loginTitle;

    /// \brief The url description.
    private: std::string urlLabel;

    /// \brief The default url (if fixed).
    private: std::string defaultUrl;

    /// \brief The widget.
    private: RestUiWidget *widget;
  };
}

#endif

