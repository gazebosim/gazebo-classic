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


#ifndef GAZEBO_PLUGIN_PLUGINLOADER_HH_
#define GAZEBO_PLUGIN_PLUGINLOADER_HH_

#include <memory>
#include <vector>

namespace gazebo
{
namespace plugin
{

/// \brief Forward declaration
class PluginLoaderPrivate;

/// \brief Class for loading gazebo plugins
class PluginLoader
{
  public:
    /// \brief Constructor
    PluginLoader();

    /// \brief Destructor
    ~PluginLoader();

    /// \brief Adds a path to search for plugins
    void AddSearchPath(std::string _path);

    /// \brief Returns paths that are being searched for plugins
    std::vector<std::string> SearchPaths();

    /// \brief Load a library by name
    bool LoadLibrary(std::string _libName);

    /// \brief Returns a list of interfaces that the loader has plugins for
    std::vector<std::string> InterfacesImplemented();

    /// \brief Returns a list of plugin names that implement the interface
    std::vector<std::string> PluginsImplementing(std::string _interface);

    /// \brief Instantiates a plugin of the name and interface
    template <typename T>
    std::unique_ptr<T> Instantiate(std::string _name, std::string _interface)
    {
      std::unique_ptr<T> ptr;
      ptr.reset(static_cast<T*>(
          this->Instantiate(_name, _interface)));
      return ptr;
    }

  private:

    void* Instantiate(std::string name, std::string interface);

    std::shared_ptr<PluginLoaderPrivate> impl;

};

}
}

#endif
