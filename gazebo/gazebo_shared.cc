/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sdf/sdf.hh>

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo_shared.hh"

/////////////////////////////////////////////////
void gazebo_shared::printVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

/////////////////////////////////////////////////
void gazebo_shared::addPlugin(const std::string &_filename,
    std::vector<gazebo::SystemPluginPtr> &_plugins)
{
  if (_filename.empty())
    return;

  gazebo::SystemPluginPtr plugin =
    gazebo::SystemPlugin::Create(_filename, _filename);

  if (plugin)
  {
    if (plugin->GetType() != gazebo::SYSTEM_PLUGIN)
    {
      gzerr << "System is attempting to load "
        << "a plugin, but detected an incorrect plugin type. "
        << "Plugin filename[" << _filename << "].\n";
      return;
    }
    _plugins.push_back(plugin);
  }
}

/////////////////////////////////////////////////
bool gazebo_shared::setup(const std::string &_prefix, int _argc, char **_argv,
    std::vector<gazebo::SystemPluginPtr> &_plugins)
{
  gazebo::common::load();

  // The SDF find file callback.
  sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

  // Initialize the informational logger. This will log warnings, and
  // errors.
  gzLogInit(_prefix, "default.log");

  // Load all the system plugins
  for (std::vector<gazebo::SystemPluginPtr>::iterator iter =
       _plugins.begin(); iter != _plugins.end(); ++iter)
  {
    (*iter)->Load(_argc, _argv);
  }

  if (!gazebo::transport::init())
  {
    gzerr << "Unable to initialize transport.\n";
    return false;
  }

  // Make sure the model database has started.
  gazebo::common::ModelDatabase::Instance()->Start();

  // Run transport loop. Starts a thread
  gazebo::transport::run();

  // Init all system plugins
  for (std::vector<gazebo::SystemPluginPtr>::iterator iter = _plugins.begin();
       iter != _plugins.end(); ++iter)
  {
    (*iter)->Init();
  }

  return true;
}
