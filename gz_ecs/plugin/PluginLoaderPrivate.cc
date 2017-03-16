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
#include <dlfcn.h>
#include <functional>

#include "gazebo/plugin/PluginLoaderPrivate.hh"


namespace gazebo
{
namespace plugin
{

std::string PluginLoaderPrivate::NormalizeName(std::string _name)
{
  if (!this->StartsWith(_name, "::"))
  {
    _name = std::string("::") + _name;
  }
  return _name;
}

std::string PluginLoaderPrivate::NormalizePath(std::string _path)
{
  // Use '/' because it works on Linux, OSX, and Windows
  std::replace(_path.begin(), _path.end(), '\\', '/');
  //Make last character '/'
  if (!this->EndsWith(_path, "/"))
  {
    _path += '/';
  }
  return _path;
}

bool PluginLoaderPrivate::StartsWith(std::string _s1, std::string _s2)
{
  bool result = false;
  if (_s1.size() >= _s2.size())
  {
    if (0 == _s1.compare(0, _s2.size(), _s2))
    {
      result = true;
    }
  }
  return result;
}

bool PluginLoaderPrivate::EndsWith(std::string _s1, std::string _s2)
{
  bool result = false;
  if (_s1.size() >= _s2.size())
  {
    if (0 == _s1.compare(_s1.size() - _s2.size(), _s2.size(), _s2))
    {
      result = true;
    }
  }
  return result;
}


std::vector<std::string> PluginLoaderPrivate::GenerateSearchNames(std::string _libName)
{
  // figure out all possible prefixes or extensions the library name could have
  bool hasLib = this->StartsWith(_libName, "lib");
  bool hasDotSo = this->EndsWith(_libName, ".so");
  bool hasDotDll = this->EndsWith(_libName, ".dll");
  bool hasDotDylib = this->EndsWith(_libName, ".dylib");

  // Try removing non cross platform parts of names
  std::vector<std::string> initNames;
  initNames.push_back(_libName);
  if (hasLib && hasDotSo)
    initNames.push_back(_libName.substr(3, _libName.size() - 6));
  if (hasDotDll)
    initNames.push_back(_libName.substr(0, _libName.size() - 3));
  if (hasLib && hasDotDylib)
    initNames.push_back(_libName.substr(3, _libName.size() - 9));

  // Create possible basenames on different platforms
  std::vector<std::string> basenames;
  for (auto iter = initNames.cbegin(); iter != initNames.cend(); ++iter)
  {
    basenames.push_back(*iter);
    basenames.push_back("lib" + *iter + ".so");
    basenames.push_back(*iter + ".so");
    basenames.push_back(*iter + ".dll");
    basenames.push_back("lib" + *iter + ".dylib");
    basenames.push_back(*iter + ".dylib");
  }

  std::vector<std::string> searchNames;
  // Concatenate these possible basenames with the search paths
  auto pathsBegin = this->searchPaths.cbegin();
  auto pathsEnd = this->searchPaths.cend();
  auto namesBegin = basenames.cbegin();
  auto namesEnd = basenames.cend();
  for (auto pathsIter = pathsBegin; pathsIter != pathsEnd; ++pathsIter)
  {
    for (auto namesIter = namesBegin; namesIter != namesEnd; ++namesIter)
    {
      searchNames.push_back(*pathsIter + *namesIter);
    }
  }
  return searchNames;
}

void* PluginLoaderPrivate::LoadLibrary(std::string _full_path)
{
  // Somehow this works on windows builds?
  return dlopen(_full_path.c_str(), RTLD_LAZY|RTLD_GLOBAL);
}

PluginInfo PluginLoaderPrivate::GetSinglePlugin(void *_dlHandle)
{
  PluginInfo plugin;
  if (nullptr != _dlHandle)
  {
    std::string sizeSymbol = "GZSinglePluginInfoSizeV1";
    std::string infoSymbol = "GZSinglePluginInfoV1";
    void *sizePtr = dlsym(_dlHandle, sizeSymbol.c_str());
    void *infoPtr = dlsym(_dlHandle, infoSymbol.c_str());
    if (nullptr != sizePtr && nullptr != infoPtr)
    {
      // It has the right symbols, check size to be extra careful
      std::size_t infoSize = *(static_cast<std::size_t*>(sizePtr));
      if (sizeof(PluginInfo) == infoSize)
      {
        PluginInfo (*GetSinglePluginInfoV1)() = reinterpret_cast<PluginInfo(*)()>(infoPtr);
        plugin = GetSinglePluginInfoV1();
      }
    }
  }
  return plugin;
}

}
}
