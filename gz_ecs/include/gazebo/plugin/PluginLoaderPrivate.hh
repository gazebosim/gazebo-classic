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


#ifndef GAZEBO_PLUGIN_PLUGINLOADERPRIVATE_HH_
#define GAZEBO_PLUGIN_PLUGINLOADERPRIVATE_HH_

#include <vector>
#include <string>
#include "gazebo/plugin/PluginInfo.hh"

namespace gazebo
{
namespace plugin
{

/// \brief Private class for PIMPL on PluginLoader
class PluginLoaderPrivate
{
  public:
    /// \brief Directories that should be searched for plugins
    std::vector<std::string> searchPaths;

    /// \brief A list of known plugins
    std::vector<PluginInfo> plugins;

    /// \brief format the name to start with "::"
    std::string NormalizeName(std::string _name);

    /// \brief format the path to use "/" as a separator with "/" at the end
    std::string NormalizePath(std::string _path);

    /// \brief generates paths to try searching for the named library
    std::vector<std::string> GenerateSearchNames(std::string _libName);

    /// \brief attempt to load a library at the given path
    void* LoadLibrary(std::string _full_path);

    /// \brief get plugin info for a library that has only one plugin
    PluginInfo GetSinglePlugin(void *_dlHandle);

    /// \brief return true if string starts with another string
    bool StartsWith(std::string _s1, std::string _s2);

    /// \brief return true if string ends with another string
    bool EndsWith(std::string _s1, std::string _s2);
};

}
}

#endif
