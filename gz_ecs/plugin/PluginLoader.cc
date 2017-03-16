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

#include <algorithm>

#include "gazebo/plugin/PluginInfo.hh"
#include "gazebo/plugin/PluginLoader.hh"
#include "gazebo/plugin/PluginLoaderPrivate.hh"


namespace gazebo
{
namespace plugin
{

PluginLoader::PluginLoader()
{
  this->impl.reset(new PluginLoaderPrivate());
}

PluginLoader::~PluginLoader()
{
}

void PluginLoader::AddSearchPath(std::string _path)
{
  std::string path = this->impl->NormalizePath(_path);
  auto begin = this->impl->searchPaths.cbegin();
  auto end = this->impl->searchPaths.cend();
  if (end == std::find(begin, end, path))
  {
    this->impl->searchPaths.push_back(path);
  }
}

std::vector<std::string> PluginLoader::SearchPaths()
{
  return this->impl->searchPaths;
}

bool PluginLoader::LoadLibrary(std::string _libName)
{
  bool loadedLibrary = false;
  std::vector<std::string> searchNames = this->impl->GenerateSearchNames(_libName);
  
  for (auto iter = searchNames.cbegin(); iter != searchNames.cend(); ++iter)
  {
    // Attempt to load the library at this path
    void *dlHandle = this->impl->LoadLibrary(*iter);
    if (nullptr != dlHandle)
    {
      // Found a shared library, does it have the symbols we're looking for?
      PluginInfo plugin = this->impl->GetSinglePlugin(dlHandle);
      if (plugin.name.size())
      {
        plugin.name = this->impl->NormalizeName(plugin.name);
        plugin.interface = this->impl->NormalizeName(plugin.interface);
        this->impl->plugins.push_back(plugin);
        loadedLibrary = true;
      }
      break;
    }
  }
  return loadedLibrary;
}

std::vector<std::string> PluginLoader::InterfacesImplemented()
{
  std::vector<std::string> interfaces;
  auto begin = this->impl->plugins.cbegin();
  auto end = this->impl->plugins.cend();
  for (auto pluginIter = begin; pluginIter != end; ++pluginIter)
  {
    if (interfaces.cend() == std::find(
          interfaces.cbegin(), interfaces.cend(), pluginIter->interface))
    {
      interfaces.push_back(pluginIter->interface);
    }
  }
  return interfaces;
}

std::vector<std::string> PluginLoader::PluginsImplementing(std::string _interface)
{
  std::string interface = this->impl->NormalizeName(_interface);
  std::vector<std::string> plugins;
  auto begin = this->impl->plugins.cbegin();
  auto end = this->impl->plugins.cend();
  for (auto pluginIter = begin; pluginIter != end; ++pluginIter)
  {
    if (pluginIter->interface == interface)
    {
      plugins.push_back(pluginIter->name);
    }
  }
  return plugins;
}

void* PluginLoader::Instantiate(std::string _name, std::string _interface)
{
  void *plugin = nullptr;
  std::string name = this->impl->NormalizeName(_name);
  std::string interface = this->impl->NormalizeName(_interface);
  std::vector<std::string> plugins;
  auto begin = this->impl->plugins.cbegin();
  auto end = this->impl->plugins.cend();
  for (auto pluginIter = begin; pluginIter != end; ++pluginIter)
  {
    if (pluginIter->interface == interface && pluginIter->name == name)
    {
      // Creates plugin on heap
      plugin = pluginIter->factory();
      break;
    }
  }
  return plugin;
}

}
}

