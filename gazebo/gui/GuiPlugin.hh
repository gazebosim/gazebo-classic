/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GZ_GUI_PLUGIN_HH_
#define _GZ_GUI_PLUGIN_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  /// \brief A plugin loaded within the gzclient on startup.
  class GAZEBO_VISIBLE GUIPlugin : public QWidget, public PluginT<GUIPlugin>
  {
    public: GUIPlugin() : QWidget(NULL)
            {this->type = GUI_PLUGIN;}

    /// \brief Load function
    ///
    /// Called when a plugin is first created.
    /// This function should not be blocking. This function is only called
    /// when a GUI plugin is loaded from an SDF file. This function is not
    /// called when a GUI plugin is loaded via a gui.ini file.
    /// \param[in] _sdf Pointer the the SDF element of the plugin. This is
    /// the plugin SDF, <plugin ...>, and its children.
    public: virtual void Load(sdf::ElementPtr /*_sdf*/) {}
  };

/// \brief Plugin registration function for gui plugin. Part of the
/// shared object interface. This function is called when loading the shared
/// library to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_GUI_PLUGIN(classname) \
  extern "C" GAZEBO_VISIBLE gazebo::GUIPlugin *RegisterPlugin(); \
  GAZEBO_VISIBLE \
  gazebo::GUIPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }
}

#endif
