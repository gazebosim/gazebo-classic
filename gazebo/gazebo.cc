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
#include <boost/thread/mutex.hpp>
#include <sdf/sdf.hh>

#include "gazebo/Master.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/Diagnostics.hh"
#include "gazebo/util/LogRecord.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/gazebo_shared.hh"
#include "gazebo/gazebo.hh"

boost::mutex fini_mutex;
std::vector<gazebo::SystemPluginPtr> g_plugins;

gazebo::Master *g_master = NULL;

/////////////////////////////////////////////////
struct g_vectorStringDup
{
  char *operator()(const std::string &_s)
  {
    return strdup(_s.c_str());
  }
};

/////////////////////////////////////////////////
void gazebo::printVersion()
{
  gazebo_shared::printVersion();
}

/////////////////////////////////////////////////
void gazebo::addPlugin(const std::string &_filename)
{
  gazebo_shared::addPlugin(_filename, g_plugins);
}

/////////////////////////////////////////////////
bool gazebo::setupServer(int _argc, char **_argv)
{
  DIAG_TIMER_START("gazebo::setupServer");
  std::string host = "";
  unsigned int port = 0;

  gazebo::transport::get_master_uri(host, port);

  g_master = new gazebo::Master();
  DIAG_TIMER_LAP("gazebo::setupServer", "Create Master");
  g_master->Init(port);
  g_master->RunThread();
  DIAG_TIMER_LAP("gazebo::setupServer", "Create Master thread");

  if (!gazebo_shared::setup("server-", _argc, _argv, g_plugins))
  {
    gzerr << "Unable to setup Gazebo\n";
    return false;
  }
  DIAG_TIMER_LAP("gazebo::setupServer", "gazebo_shared::setup");

  if (!sensors::load())
  {
    gzerr << "Unable to load sensors\n";
    return false;
  }
  DIAG_TIMER_LAP("gazebo::setupServer", "sensors::load");

  if (!gazebo::physics::load())
  {
    gzerr << "Unable to initialize physics.\n";
    return false;
  }
  DIAG_TIMER_LAP("gazebo::setupServer", "physics::load");

  if (!sensors::init())
  {
    gzerr << "Unable to initialize sensors\n";
    return false;
  }
  DIAG_TIMER_LAP("gazebo::setupServer", "sensors::init");

  DIAG_TIMER_STOP("gazebo::setupServer");
  return true;
}

/////////////////////////////////////////////////
bool gazebo::setupServer(const std::vector<std::string> &_args)
{
  std::vector<char *> pointers(_args.size());
  std::transform(_args.begin(), _args.end(), pointers.begin(),
                 g_vectorStringDup());
  pointers.push_back(0);
  bool result = gazebo::setupServer(_args.size(), &pointers[0]);

  // Deallocate memory for the command line arguments allocated with strdup.
  for (size_t i = 0; i < pointers.size(); ++i)
    free(pointers.at(i));

  return result;
}

/////////////////////////////////////////////////
bool gazebo::setupClient(int _argc, char **_argv)
{
  if (!gazebo_shared::setup("client-", _argc, _argv, g_plugins))
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

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  bool result = gazebo::setupClient(_args.size(), &pointers[0]);
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

  // Deallocate memory for the command line arguments alloocated with strdup.
  for (size_t i = 0; i < pointers.size(); ++i)
    free(pointers.at(i));

  return result;
}

/////////////////////////////////////////////////
bool gazebo::shutdown()
{
  DIAG_TIMER_START("gazebo::shutdown");
  gazebo::physics::stop_worlds();
  DIAG_TIMER_LAP("gazebo::shutdown", "physics::stop_worlds");

  gazebo::sensors::stop();
  DIAG_TIMER_LAP("gazebo::shutdown", "sensors::stop");

  // Stop log recording
  util::LogRecord::Instance()->Stop();
  DIAG_TIMER_LAP("gazebo::shutdown", "LogRecord::Stop");

  // Stop transport
  gazebo::transport::stop();
  DIAG_TIMER_LAP("gazebo::shutdown", "transport::stop");

  // Make sure to shut everything down.
  boost::mutex::scoped_lock lock(fini_mutex);
  util::LogRecord::Instance()->Fini();
  DIAG_TIMER_LAP("gazebo::shutdown", "LogRecord::Fini");

  g_plugins.clear();
  DIAG_TIMER_LAP("gazebo::shutdown", "g_plugins.clear");

  gazebo::transport::fini();
  DIAG_TIMER_LAP("gazebo::shutdown", "transport::fini");

  gazebo::physics::fini();
  DIAG_TIMER_LAP("gazebo::shutdown", "physics::fini");

  gazebo::sensors::fini();
  DIAG_TIMER_LAP("gazebo::shutdown", "sensors::fini");

  delete g_master;
  g_master = NULL;
  DIAG_TIMER_LAP("gazebo::shutdown", "Destroy Master");

  // Cleanup model database.
  common::ModelDatabase::Instance()->Fini();

  DIAG_TIMER_STOP("gazebo::shutdown");
  return true;
}

/////////////////////////////////////////////////
gazebo::physics::WorldPtr gazebo::loadWorld(const std::string &_worldFile)
{
  gazebo::physics::WorldPtr world;

  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF);
  if (!sdf::init(sdf))
  {
    gzerr << "Unable to initialize sdf\n";
    return world;
  }

  // Find the file.
  std::string fullFile = gazebo::common::find_file(_worldFile);

  if (fullFile.empty())
  {
    gzerr << "Unable to find file[" << _worldFile << "]\n";
    return world;
  }

  if (!sdf::readFile(fullFile, sdf))
  {
    gzerr << "Unable to read sdf file[" << "empty.world" << "]\n";
    return world;
  }

  world = gazebo::physics::create_world();
  gazebo::physics::load_world(world, sdf->Root()->GetElement("world"));

  gazebo::physics::init_world(world);

  return world;
}

/////////////////////////////////////////////////
void gazebo::runWorld(gazebo::physics::WorldPtr _world,
                      unsigned int _iterations)
{
  if (!_world)
    gzerr << "World pointer is NULL\n";
  else
    _world->RunBlocking(_iterations);
}
