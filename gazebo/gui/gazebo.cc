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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <vector>
#include <sdf/sdf.hh>

#include "gazebo/transport/transport.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/LogRecord.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo.hh"

boost::mutex fini_mutex;
std::vector<gazebo::SystemPluginPtr> g_plugins;

/////////////////////////////////////////////////
struct g_vectorStringDup
{
  char *operator()(const std::string &_s)
  {
    return strdup(_s.c_str());
  }
};

/////////////////////////////////////////////////
// This function is used by both setupClient and setupServer
bool setup(const std::string &_prefix, int _argc, char **_argv)
{
  gazebo::common::load();

  // The SDF find file callback.
  sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

  // Initialize the informational logger. This will log warnings, and
  // errors.
  gzLogInit(_prefix, "default.log");

  // Load all the system plugins
  for (std::vector<gazebo::SystemPluginPtr>::iterator iter =
       g_plugins.begin(); iter != g_plugins.end(); ++iter)
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
  for (std::vector<gazebo::SystemPluginPtr>::iterator iter = g_plugins.begin();
       iter != g_plugins.end(); ++iter)
  {
    (*iter)->Init();
  }

  return true;
}

/////////////////////////////////////////////////
void gazebo::printVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

/////////////////////////////////////////////////
void gazebo::addPlugin(const std::string &_filename)
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
bool gazebo::setupClient(int _argc, char **_argv)
{
  if (!setup("client-", _argc, _argv))
  {
    gzerr << "Unable to setup Gazebo\n";
    return false;
  }

  common::Time waitTime(1, 0);
  int waitCount = 0;
  int maxWaitCount = 10;

  // Wait for namespaces.
  while (!gazebo::transport::waitForNamespaces(waitTime) &&
      (waitCount++) < maxWaitCount)
  {
    gzwarn << "Waited " << waitTime.Double() << "seconds for namespaces.\n";
  }

  if (waitCount >= maxWaitCount)
  {
    gzerr << "Waited " << (waitTime * waitCount).Double()
      << " seconds for namespaces. Giving up.\n";
  }

  return true;
}

/////////////////////////////////////////////////
bool gazebo::setupClient(const std::vector<std::string> &_args)
{
  std::vector<char *> pointers(_args.size());
  std::transform(_args.begin(), _args.end(), pointers.begin(),
                 g_vectorStringDup());
  pointers.push_back(0);
  bool result = gazebo::setupClient(_args.size(), &pointers[0]);

  // Deallocate memory for the command line arguments alloocated with strdup.
  for (size_t i = 0; i < pointers.size(); ++i)
    free(pointers.at(i));

  return result;
}

/////////////////////////////////////////////////
bool gazebo::shutdown()
{
  // Stop log recording
  util::LogRecord::Instance()->Stop();

  // Stop transport
  gazebo::transport::stop();

  // Make sure to shut everything down.
  boost::mutex::scoped_lock lock(fini_mutex);
  util::LogRecord::Instance()->Fini();
  g_plugins.clear();
  gazebo::transport::fini();

  // Cleanup model database.
  common::ModelDatabase::Instance()->Fini();

  return true;
}
