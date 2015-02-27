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
#include <stdio.h>
#include <signal.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/util/LogRecord.hh"
#include "gazebo/util/LogPlay.hh"
#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorsIface.hh"

#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Base.hh"

#include "gazebo/Master.hh"
#include "gazebo/Server.hh"

namespace po = boost::program_options;
using namespace gazebo;

bool Server::stop = true;

/////////////////////////////////////////////////
Server::Server()
{
  this->initialized = false;
  this->systemPluginsArgc = 0;
  this->systemPluginsArgv = NULL;
}

/////////////////////////////////////////////////
Server::~Server()
{
  fflush(stdout);
}

/////////////////////////////////////////////////
void Server::PrintUsage()
{
  std::cerr << "gzserver -- Run the Gazebo server.\n\n";
  std::cerr << "`gzserver` [options] <world_file>\n\n";
  std::cerr << "Gazebo server runs simulation and handles commandline "
    << "options, starts a Master, runs World update and sensor generation "
    << "loops.\n\n";
}

/////////////////////////////////////////////////
bool Server::ParseArgs(int _argc, char **_argv)
{
  // Save a copy of argc and argv for consumption by system plugins
  this->systemPluginsArgc = _argc;
  this->systemPluginsArgv = new char*[_argc];
  for (int i = 0; i < _argc; ++i)
  {
    int argvLen = strlen(_argv[i]) + 1;
    this->systemPluginsArgv[i] = new char[argvLen];
    snprintf(this->systemPluginsArgv[i], argvLen, "%s", _argv[i]);
  }

  po::options_description visibleDesc("Options");
  visibleDesc.add_options()
    ("version,v", "Output version information.")
    ("verbose", "Increase the messages written to the terminal.")
    ("help,h", "Produce this help message.")
    ("pause,u", "Start the server in a paused state.")
    ("physics,e", po::value<std::string>(),
     "Specify a physics engine (ode|bullet|dart|simbody).")
    ("play,p", po::value<std::string>(), "Play a log file.")
    ("record,r", "Record state data.")
    ("record_encoding", po::value<std::string>()->default_value("zlib"),
     "Compression encoding format for log data (zlib|bz2|txt).")
    ("record_path", po::value<std::string>()->default_value(""),
     "Absolute path in which to store state data")
    ("seed",  po::value<double>(), "Start with a given random number seed.")
    ("iters",  po::value<unsigned int>(), "Number of iterations to simulate.")
    ("minimal_comms", "Reduce the TCP/IP traffic output by gzserver")
    ("server-plugin,s", po::value<std::vector<std::string> >(),
     "Load a plugin.")
    ("profile,o", po::value<std::string>()->default_value(""),
     "Specify a preset profile name from the options in the world file.");

  po::options_description hiddenDesc("Hidden options");
  hiddenDesc.add_options()
    ("world_file", po::value<std::string>(), "SDF world to load.")
    ("pass_through", po::value<std::vector<std::string> >(),
     "not used, passed through to system plugins.");

  po::options_description desc("Options");
  desc.add(visibleDesc).add(hiddenDesc);

  po::positional_options_description positionalDesc;
  positionalDesc.add("world_file", 1).add("pass_through", -1);

  try
  {
    po::store(po::command_line_parser(_argc, _argv).options(desc).positional(
          positionalDesc).allow_unregistered().run(), this->vm);

    po::notify(this->vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    // NOTE: boost::diagnostic_information(_e) breaks lucid
    // std::cerr << boost::diagnostic_information(_e) << "\n";
    return false;
  }

  if (this->vm.count("version"))
  {
    std::cout << GAZEBO_VERSION_HEADER << std::endl;
    return false;
  }

  if (this->vm.count("help"))
  {
    this->PrintUsage();
    std::cerr << visibleDesc << "\n";
    return false;
  }

  if (this->vm.count("verbose"))
  {
    gazebo::printVersion();
    gazebo::common::Console::SetQuiet(false);
  }

  if (this->vm.count("minimal_comms"))
    gazebo::transport::setMinimalComms(true);
  else
    gazebo::transport::setMinimalComms(false);

  // Set the random number seed if present on the command line.
  if (this->vm.count("seed"))
  {
    try
    {
      math::Rand::SetSeed(this->vm["seed"].as<double>());
    }
    catch(boost::bad_any_cast &_e)
    {
      gzerr << "Unable to set random number seed. Must supply a number.\n";
    }
  }

  /// Load all the plugins specified on the command line
  if (this->vm.count("server-plugin"))
  {
    std::vector<std::string> pp =
      this->vm["server-plugin"].as<std::vector<std::string> >();

    for (std::vector<std::string>::iterator iter = pp.begin();
         iter != pp.end(); ++iter)
    {
      gazebo::addPlugin(*iter);
    }
  }

  // Set the parameter to record a log file
  if (this->vm.count("record"))
  {
    this->params["record"] = this->vm["record_path"].as<std::string>();
    this->params["record_encoding"] =
        this->vm["record_encoding"].as<std::string>();
  }

  if (this->vm.count("iters"))
  {
    try
    {
      this->params["iterations"] = boost::lexical_cast<std::string>(
          this->vm["iters"].as<unsigned int>());
    }
    catch(...)
    {
      this->params["iterations"] = "0";
      gzerr << "Unable to set iterations of [" <<
        this->vm["iters"].as<unsigned int>() << "]\n";
    }
  }

  if (this->vm.count("pause"))
    this->params["pause"] = "true";
  else
    this->params["pause"] = "false";

  if (!this->PreLoad())
  {
    gzerr << "Unable to load gazebo\n";
    return false;
  }

  // The following "if" block must be processed directly before
  // this->ProcessPrarams.
  //
  // Set the parameter to playback a log file. The log file contains the
  // world description, so don't try to reead the world file from the
  // command line.
  if (this->vm.count("play"))
  {
    // Load the log file
    util::LogPlay::Instance()->Open(this->vm["play"].as<std::string>());

    gzmsg << "\nLog playback:\n"
      << "  Log Version: "
      << util::LogPlay::Instance()->GetLogVersion() << "\n"
      << "  Gazebo Version: "
      << util::LogPlay::Instance()->GetGazeboVersion() << "\n"
      << "  Random Seed: "
      << util::LogPlay::Instance()->GetRandSeed() << "\n";

    // Get the SDF world description from the log file
    std::string sdfString;
    util::LogPlay::Instance()->Step(sdfString);

    // Load the server
    if (!this->LoadString(sdfString))
      return false;
  }
  else
  {
    // Get the world file name from the command line, or use "empty.world"
    // if no world file is specified.
    std::string configFilename = "worlds/empty.world";
    if (this->vm.count("world_file"))
      configFilename = this->vm["world_file"].as<std::string>();

    // Get the physics engine name specified from the command line, or use ""
    // if no physics engine is specified.
    std::string physics;
    if (this->vm.count("physics"))
      physics = this->vm["physics"].as<std::string>();

    // Load the server
    if (!this->LoadFile(configFilename, physics))
      return false;
    if (this->vm.count("profile"))
    {
      physics::get_world()->GetPresetManager()->CurrentProfile(
          this->vm["profile"].as<std::string>());
    }
  }

  this->ProcessParams();

  return true;
}

/////////////////////////////////////////////////
bool Server::GetInitialized() const
{
  return !this->stop && this->initialized;
}

/////////////////////////////////////////////////
bool Server::LoadFile(const std::string &_filename,
                      const std::string &_physics)
{
  // Quick test for a valid file
  FILE *test = fopen(common::find_file(_filename).c_str(), "r");
  if (!test)
  {
    gzerr << "Could not open file[" << _filename << "]\n";
    return false;
  }
  fclose(test);

  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF);
  if (!sdf::init(sdf))
  {
    gzerr << "Unable to initialize sdf\n";
    return false;
  }

  if (!sdf::readFile(common::find_file(_filename), sdf))
  {
    gzerr << "Unable to read sdf file[" << _filename << "]\n";
    return false;
  }

  return this->LoadImpl(sdf->root, _physics);
}

/////////////////////////////////////////////////
bool Server::LoadString(const std::string &_sdfString)
{
  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF);
  if (!sdf::init(sdf))
  {
    gzerr << "Unable to initialize sdf\n";
    return false;
  }

  if (!sdf::readString(_sdfString, sdf))
  {
    gzerr << "Unable to read SDF string[" << _sdfString << "]\n";
    return false;
  }

  return this->LoadImpl(sdf->root);
}

/////////////////////////////////////////////////
bool Server::PreLoad()
{
  // setup gazebo
  return gazebo::setupServer(this->systemPluginsArgc, this->systemPluginsArgv);
}

/////////////////////////////////////////////////
bool Server::LoadImpl(sdf::ElementPtr _elem,
                      const std::string &_physics)
{
  // If a physics engine is specified,
  if (_physics.length())
  {
    // Check if physics engine name is valid
    // This must be done after physics::load();
    if (!physics::PhysicsFactory::IsRegistered(_physics))
    {
      gzerr << "Unregistered physics engine [" << _physics
            << "], the default will be used instead.\n";
    }
    // Try inserting physics engine name if one is given
    else if (_elem->HasElement("world") &&
             _elem->GetElement("world")->HasElement("physics"))
    {
      _elem->GetElement("world")->GetElement("physics")
           ->GetAttribute("type")->Set(_physics);
    }
    else
    {
      gzerr << "Cannot set physics engine: <world> does not have <physics>\n";
    }
  }

  sdf::ElementPtr worldElem = _elem->GetElement("world");
  if (worldElem)
  {
    physics::WorldPtr world = physics::create_world();

    // Create the world
    try
    {
      physics::load_world(world, worldElem);
    }
    catch(common::Exception &e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("/gazebo");
  this->serverSub = this->node->Subscribe("/gazebo/server/control",
                                          &Server::OnControl, this);

  this->worldModPub =
    this->node->Advertise<msgs::WorldModify>("/gazebo/world/modify");

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

  physics::init_worlds();
  this->stop = false;

  return true;
}

/////////////////////////////////////////////////
void Server::SigInt(int)
{
  stop = true;

  // Signal to plugins/etc that a shutdown event has occured
  event::Events::sigInt();
}

/////////////////////////////////////////////////
void Server::Stop()
{
  this->stop = true;
}

/////////////////////////////////////////////////
void Server::Fini()
{
  this->Stop();
  gazebo::shutdown();
}

/////////////////////////////////////////////////
void Server::Run()
{
  // Now that we're about to run, install a signal handler to allow for
  // graceful shutdown on Ctrl-C.
  struct sigaction sigact;
  sigact.sa_handler = Server::SigInt;
  if (sigaction(SIGINT, &sigact, NULL))
    std::cerr << "sigaction(2) failed while setting up for SIGINT" << std::endl;

  if (this->stop)
    return;

  // Make sure the sensors are updated once before running the world.
  // This makes sure plugins get loaded properly.
  sensors::run_once(true);

  // Run the sensor threads
  sensors::run_threads();

  unsigned int iterations = 0;
  common::StrStr_M::iterator piter = this->params.find("iterations");
  if (piter != this->params.end())
  {
    try
    {
      iterations = boost::lexical_cast<unsigned int>(piter->second);
    }
    catch(...)
    {
      iterations = 0;
      gzerr << "Unable to cast iterations[" << piter->second << "] "
        << "to unsigned integer\n";
    }
  }

  // Run each world. Each world starts a new thread
  physics::run_worlds(iterations);

  this->initialized = true;

  // Update the sensors.
  while (!this->stop && physics::worlds_running())
  {
    this->ProcessControlMsgs();
    sensors::run_once();
    common::Time::MSleep(1);
  }

  // Shutdown gazebo
  gazebo::shutdown();
}

/////////////////////////////////////////////////
void Server::ProcessParams()
{
  common::StrStr_M::const_iterator iter;
  for (iter = this->params.begin(); iter != this->params.end(); ++iter)
  {
    if (iter->first == "pause")
    {
      bool p = false;
      try
      {
        p = boost::lexical_cast<bool>(iter->second);
      }
      catch(...)
      {
        // Unable to convert via lexical_cast, so try "true/false" string
        std::string str = iter->second;
        boost::to_lower(str);

        if (str == "true")
          p = true;
        else if (str == "false")
          p = false;
        else
          gzerr << "Invalid param value[" << iter->first << ":"
                << iter->second << "]\n";
      }

      physics::pause_worlds(p);
    }
    else if (iter->first == "record")
    {
      util::LogRecord::Instance()->Start(this->params["record_encoding"],
                                         iter->second);
    }
  }
}

/////////////////////////////////////////////////
void Server::SetParams(const common::StrStr_M &_params)
{
  common::StrStr_M::const_iterator iter;
  for (iter = _params.begin(); iter != _params.end(); ++iter)
    this->params[iter->first] = iter->second;
}

/////////////////////////////////////////////////
void Server::OnControl(ConstServerControlPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->receiveMutex);
  this->controlMsgs.push_back(*_msg);
}

/////////////////////////////////////////////////
void Server::ProcessControlMsgs()
{
  std::list<msgs::ServerControl>::iterator iter;
  for (iter = this->controlMsgs.begin();
       iter != this->controlMsgs.end(); ++iter)
  {
    if ((*iter).has_clone() && (*iter).clone())
    {
      bool success = true;
      std::string host;
      std::string port;
      physics::WorldPtr world;

      // Get the world's name to be cloned.
      std::string worldName = "";
      if ((*iter).has_save_world_name())
        worldName = (*iter).save_world_name();

      // Get the world pointer.
      try
      {
        world = physics::get_world(worldName);
      }
      catch(const common::Exception &)
      {
        gzwarn << "Unable to clone a server. Unknown world ["
               << (*iter).save_world_name() << "]" << std::endl;
        success = false;
      }

      // Check if the message contains a port for the new server.
      if ((*iter).has_new_port())
        port = boost::lexical_cast<std::string>((*iter).new_port());
      else
      {
        gzwarn << "Unable to clone a server. Port is missing" << std::endl;
        success = false;
      }

      if (success)
      {
        // world should not be NULL at this point.
        GZ_ASSERT(world, "NULL world pointer");

        // Save the world's state in a temporary file (clone.<PORT>.world).
        boost::filesystem::path tmpDir =
            boost::filesystem::temp_directory_path();
        boost::filesystem::path worldFilename = "clone." + port + ".world";
        boost::filesystem::path worldPath = tmpDir / worldFilename;
        world->Save(worldPath.string());

        // Get the hostname from the current server's master.
        unsigned int unused;
        transport::get_master_uri(host, unused);

        // Command to be executed for cloning the server. The new server will
        // load the world file /tmp/clone.<PORT>.world
        std::string cmd = "GAZEBO_MASTER_URI=http://" + host + ":" + port +
            " gzserver " + worldPath.string() + " &";

        // Spawn a new gzserver process and load the saved world.
        if (std::system(cmd.c_str()) == 0)
        {
          gzlog << "Cloning world [" << worldName << "]. "
                << "Connect to the server by typing:\n\tGAZEBO_MASTER_URI="
                << "http://" << host << ":" << port << " gzclient" << std::endl;
        }
        else
        {
          gzerr << "Unable to clone a simulation running the following command:"
                << std::endl << "\t[" << cmd << "]" << std::endl;
          success = false;
        }
      }

      // Notify the result.
      msgs::WorldModify worldMsg;
      worldMsg.set_world_name(worldName);
      worldMsg.set_cloned(success);
      if (success)
        worldMsg.set_cloned_uri("http://" + host + ":" + port);
      this->worldModPub->Publish(worldMsg);
    }
    else if ((*iter).has_save_world_name())
    {
      physics::WorldPtr world = physics::get_world((*iter).save_world_name());
      if ((*iter).has_save_filename())
        world->Save((*iter).save_filename());
      else
        gzerr << "No filename specified.\n";
    }
    else if ((*iter).has_new_world() && (*iter).new_world())
    {
      this->OpenWorld("worlds/empty.world");
    }
    else if ((*iter).has_open_filename())
    {
      this->OpenWorld((*iter).open_filename());
    }
    else if ((*iter).has_stop() && (*iter).stop())
    {
      this->Stop();
    }
  }
  this->controlMsgs.clear();
}

/////////////////////////////////////////////////
bool Server::OpenWorld(const std::string & /*_filename*/)
{
  gzerr << "Open World is not implemented\n";
  return false;
/*
  sdf::SDFPtr sdf(new sdf::SDF);
  if (!sdf::init(sdf))
  {
    gzerr << "Unable to initialize sdf\n";
    return false;
  }

  if (!sdf::readFile(_filename, sdf))
  {
    gzerr << "Unable to read sdf file[" << _filename << "]\n";
    return false;
  }

  msgs::WorldModify worldMsg;
  worldMsg.set_world_name("default");
  worldMsg.set_remove(true);
  this->worldModPub->Publish(worldMsg);

  physics::stop_worlds();

  physics::remove_worlds();

  sensors::remove_sensors();

  gazebo::transport::clear_buffers();

  sdf::ElementPtr worldElem = sdf->root->GetElement("world");

  physics::WorldPtr world = physics::create_world();

  physics::load_world(world, worldElem);

  physics::init_world(world);

  physics::run_world(world);

  worldMsg.set_world_name("default");
  worldMsg.set_remove(false);
  worldMsg.set_create(true);
  this->worldModPub->Publish(worldMsg);
  return true;
  */
}
