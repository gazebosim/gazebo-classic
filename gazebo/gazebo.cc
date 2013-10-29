/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#include <vector>
#include <boost/thread/mutex.hpp>
#include <sdf/sdf.hh>

#include "gazebo/transport/transport.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/LogRecord.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/gazebo.hh"

boost::mutex fini_mutex;
std::vector<gazebo::SystemPluginPtr> g_plugins;

/////////////////////////////////////////////////
void gazebo::print_version()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

/////////////////////////////////////////////////
void gazebo::add_plugin(const std::string &_filename)
{
  if (_filename.empty())
    return;
  gazebo::SystemPluginPtr plugin =
    gazebo::SystemPlugin::Create(_filename, _filename);

  if (plugin)
  {
    if (plugin->GetType() != SYSTEM_PLUGIN)
    {
      gzerr << "System is attempting to load "
        << "a plugin, but detected an incorrect plugin type. "
        << "Plugin filename[" << _filename << "].\n";
      return;
    }
    g_plugins.push_back(plugin);
  }
}

/////////////////////////////////////////////////
bool gazebo::load(int _argc, char **_argv)
{
  gazebo::common::load();

  // The SDF find file callback.
  sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

  // Initialize the informational logger. This will log warnings, and
  // errors.
  if (!gazebo::common::Console::Instance()->IsInitialized())
    gazebo::common::Console::Instance()->Init("default.log");

  // Load all the plugins
  for (std::vector<gazebo::SystemPluginPtr>::iterator iter =
       g_plugins.begin(); iter != g_plugins.end(); ++iter)
  {
    (*iter)->Load(_argc, _argv);
  }

  // Start the transport system by connecting to the master.
  return gazebo::transport::init();
}

/////////////////////////////////////////////////
bool gazebo::init()
{
  for (std::vector<SystemPluginPtr>::iterator iter = g_plugins.begin();
       iter != g_plugins.end(); ++iter)
  {
    (*iter)->Init();
  }

  return true;
}

/////////////////////////////////////////////////
void gazebo::run()
{
  // Run transport loop. Starts a thread
  gazebo::transport::run();
}

/////////////////////////////////////////////////
void gazebo::stop()
{
  util::LogRecord::Instance()->Stop();
  gazebo::transport::stop();
}

/////////////////////////////////////////////////
void gazebo::fini()
{
  boost::mutex::scoped_lock lock(fini_mutex);
  util::LogRecord::Instance()->Fini();
  g_plugins.clear();
  gazebo::transport::fini();
}
