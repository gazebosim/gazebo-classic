/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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


#ifndef GAZEBO_PLUGIN_PLUGININFO_HH_
#define GAZEBO_PLUGIN_PLUGININFO_HH_

#include <functional>
#include <string>

namespace gazebo
{
namespace plugin
{

/// \brief Struct with plugin info
struct PluginInfo
{
    /// \brief the name of the plugin
    std::string name;

    /// \brief the name of the type of plugin this implements
    std::string interface;
    
    // Weirdness note: If the macro sets name or interface, they
    // are just empty strings. C-style string is required to be
    // set when a dynamic library is loaded
    const char *char_name;
    const char *char_interface;

    /// \brief a method that instantiates a new instance of a plugin
    std::function<void*()> factory;
};

}
}

#endif
