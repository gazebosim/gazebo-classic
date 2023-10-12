/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
  // snprintf is available since VS 2015
  #if defined(_MSC_VER) && (_MSC_VER < 1900)
    #define snprintf _snprintf
  #endif
#endif

#include <stdio.h>
#include <signal.h>
#include <mutex>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <sdf/parser.hh>
#include <sdf/sdf.hh>

#include <ignition/math/Rand.hh>
#include <ignition/math/SemanticVersion.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/URI.hh>
#ifdef _WIN32
  // DELETE is defined in winnt.h and causes a problem with
  // ignition::fuel_tools::REST::DELETE
  #undef DELETE
#endif
#include <ignition/fuel_tools/Interface.hh>

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
#include "gazebo/physics/PresetManager.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Base.hh"

#include "gazebo/rendering/RenderingIface.hh"

#include "gazebo/Master.hh"
#include "gazebo/Server.hh"

namespace po = boost::program_options;
using namespace gazebo;

namespace gazebo
{
  struct ServerPrivate
  {
    void InspectSDFElement(const sdf::ElementPtr _elem)
    {
      if (common::getEnv("GAZEBO11_BACKWARDS_COMPAT_WARNINGS_ERRORS"))
        return;

      // SDF 1.7 and above consider invalid sibling elements of ANY type with
      // repeated names while SDF 1.6 and lower only elements of the SAME type
      auto sdfVersion =
        ignition::math::SemanticVersion(_elem->OriginalVersion());
      bool result;
      if (sdfVersion >= ignition::math::SemanticVersion(1, 7))
        result = sdf::recursiveSiblingUniqueNames(_elem);
      else
        result = sdf::recursiveSameTypeUniqueNames(_elem);

      if (!result)
        gzerr << "SDF is not valid, see errors above. "
              << "This can lead to an unexpected behaviour." << "\n";
    }

    /// \brief Boolean used to stop the server.
    static bool stop;

    /// \brief Communication node.
    transport::NodePtr node;

    /// \brief Subscribe to server control messages.
    transport::SubscriberPtr serverSub;

    /// \brief Publisher for world modifications.
    transport::PublisherPtr worldModPub;

    /// \brief Mutex to protect controlMsgs.
    std::mutex receiveMutex;

    /// \brief List of received control messages.
    std::list<msgs::ServerControl> controlMsgs;

    /// \brief Command line params that are passed to various Gazebo objects.
    gazebo::common::StrStr_M params;

    /// \brief Boost program options variable map.
    boost::program_options::variables_map vm;

    /// \brief True when initialized.
    bool initialized;

    /// \brief Save argc for access by system plugins.
    int systemPluginsArgc;

    /// \brief Save argv for access by system plugins.
    char **systemPluginsArgv;

    /// \brief Set whether to lockstep physics and rendering
    bool lockstep = false;
  };
}

bool ServerPrivate::stop = true;

/////////////////////////////////////////////////
Server::Server()
  : dataPtr(new ServerPrivate())
{
  this->dataPtr->initialized = false;
  this->dataPtr->systemPluginsArgc = 0;
  this->dataPtr->systemPluginsArgv = NULL;
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
  this->dataPtr->systemPluginsArgc = _argc;
  this->dataPtr->systemPluginsArgv = new char*[_argc];
  for (int i = 0; i < _argc; ++i)
  {
    int argvLen = strlen(_argv[i]) + 1;
    this->dataPtr->systemPluginsArgv[i] = new char[argvLen];
    snprintf(this->dataPtr->systemPluginsArgv[i], argvLen, "%s", _argv[i]);
  }

  po::options_description visibleDesc("Options");
  visibleDesc.add_options()
    ("version,v", "Output version information.")
    ("verbose", "Increase the messages written to the terminal.")
    ("help,h", "Produce this help message.")
    ("pause,u", "Start the server in a paused state.")
    ("lockstep", "Lockstep simulation so sensor update rates are respected.")
    ("physics,e", po::value<std::string>(),
     "Specify a physics engine (ode|bullet|dart|simbody).")
    ("play,p", po::value<std::string>(), "Play a log file.")
    ("record,r", "Record state data.")
    ("record_encoding", po::value<std::string>()->default_value("zlib"),
     "Compression encoding format for log data (zlib|bz2|txt).")
    ("record_path", po::value<std::string>()->default_value(""),
     "Absolute path in which to store state data")
    ("record_period", po::value<double>()->default_value(-1),
     "Recording period (seconds).")
    ("record_filter", po::value<std::string>()->default_value(""),
     "Recording filter (supports wildcard and regular expression).")
    ("record_resources", "Recording with model meshes and materials.")
    ("seed",  po::value<double>(), "Start with a given random number seed.")
    ("initial_sim_time", po::value<double>(),
     "Initial simulation time (seconds). This time is also used after reset.")
    ("iters",  po::value<unsigned int>(), "Number of iterations to simulate.")
    ("minimal_comms", "Reduce the TCP/IP traffic output by gzserver")
    ("server-plugin,s", po::value<std::vector<std::string> >(),
     "Load a plugin.")
    ("profile,o", po::value<std::string>(),
     "Physics preset profile name from the options in the world file.");

  po::options_description hiddenDesc("Hidden options");
  hiddenDesc.add_options()
    // This is a bit of a hack. The server assumes the last item on the
    // command (if present) is a world file. A problem arises with:
    //     gazebo -g <some_gui_plugin.so>
    // Without this hidden option, the server would try to load
    // <some_gui_plugin.so> as a world file.
    ("gui-plugin,g", po::value<std::vector<std::string> >(),
     "Gui plugin ignored.")
    ("gui-client-plugin", po::value<std::vector<std::string> >(),
     "Gui plugin ignored.")
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
          positionalDesc).allow_unregistered().run(), this->dataPtr->vm);

    po::notify(this->dataPtr->vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    // NOTE: boost::diagnostic_information(_e) breaks lucid
    // std::cerr << boost::diagnostic_information(_e) << "\n";
    return false;
  }

  if (this->dataPtr->vm.count("version"))
  {
    std::cout << GAZEBO_VERSION_HEADER << std::endl;
    return false;
  }

  if (this->dataPtr->vm.count("help"))
  {
    this->PrintUsage();
    std::cerr << visibleDesc << "\n";
    return false;
  }

  if (this->dataPtr->vm.count("verbose"))
  {
    gazebo::printVersion();
    gazebo::common::Console::SetQuiet(false);
  }

  if (this->dataPtr->vm.count("minimal_comms"))
    gazebo::transport::setMinimalComms(true);
  else
    gazebo::transport::setMinimalComms(false);

  // Set the random number seed if present on the command line.
  if (this->dataPtr->vm.count("seed"))
  {
    try
    {
      ignition::math::Rand::Seed(this->dataPtr->vm["seed"].as<double>());
    }
    catch(boost::bad_any_cast &_e)
    {
      gzerr << "Unable to set random number seed. Must supply a number.\n";
    }
  }

  /// Load all the plugins specified on the command line
  if (this->dataPtr->vm.count("server-plugin"))
  {
    std::vector<std::string> pp =
      this->dataPtr->vm["server-plugin"].as<std::vector<std::string> >();

    for (std::vector<std::string>::iterator iter = pp.begin();
         iter != pp.end(); ++iter)
    {
      gazebo::addPlugin(*iter);
    }
  }

  // Set the parameter to record a log file
  if (this->dataPtr->vm.count("record"))
  {
    this->dataPtr->params["record"] =
      this->dataPtr->vm["record_path"].as<std::string>();
    this->dataPtr->params["record_encoding"] =
      this->dataPtr->vm["record_encoding"].as<std::string>();
    if (this->dataPtr->vm.count("record_resources"))
      this->dataPtr->params["record_resources"] = "true";
  }

  if (this->dataPtr->vm.count("iters"))
  {
    try
    {
      this->dataPtr->params["iterations"] = boost::lexical_cast<std::string>(
          this->dataPtr->vm["iters"].as<unsigned int>());
    }
    catch(...)
    {
      this->dataPtr->params["iterations"] = "0";
      gzerr << "Unable to set iterations of [" <<
        this->dataPtr->vm["iters"].as<unsigned int>() << "]\n";
    }
  }

  if (this->dataPtr->vm.count("lockstep"))
  {
    this->dataPtr->lockstep = true;
  }
  rendering::set_lockstep_enabled(this->dataPtr->lockstep);

  if (!this->PreLoad())
  {
    gzerr << "Unable to load gazebo\n";
    return false;
  }

  // The following "if" block must be processed directly before
  // this->dataPtr->ProcessPrarams.
  //
  // Set the parameter to playback a log file. The log file contains the
  // world description, so don't try to read the world file from the
  // command line.
  if (this->dataPtr->vm.count("play"))
  {
    // Load the log file
    util::LogPlay::Instance()->Open(
        this->dataPtr->vm["play"].as<std::string>());

    gzmsg << "\nLog playback:\n"
      << "  Log Version: "
      << util::LogPlay::Instance()->LogVersion() << "\n"
      << "  Gazebo Version: "
      << util::LogPlay::Instance()->GazeboVersion() << "\n"
      << "  Random Seed: "
      << util::LogPlay::Instance()->RandSeed() << "\n"
      << "  Log Start Time: "
      << util::LogPlay::Instance()->LogStartTime() << "\n"
      << "  Log End Time: "
      << util::LogPlay::Instance()->LogEndTime() << "\n";

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
    if (this->dataPtr->vm.count("world_file"))
      configFilename = this->dataPtr->vm["world_file"].as<std::string>();

    // Get the physics engine name specified from the command line, or use ""
    // if no physics engine is specified.
    std::string physics;
    if (this->dataPtr->vm.count("physics"))
      physics = this->dataPtr->vm["physics"].as<std::string>();

    // Load the server
    if (!this->LoadFile(configFilename, physics))
    {
      gzwarn << "Falling back on worlds/empty.world\n";
      if (!this->LoadFile("worlds/empty.world", physics))
      {
        gzerr << "worlds/empty.world could not be opened, "
              << "probably because it was not found. "
              << "Your GAZEBO_RESOURCE_PATH is probably improperly set. "
              << "Have you sourced <prefix>/share/gazebo/setup.bash?\n";
        return false;
      }
    }

    if (this->dataPtr->vm.count("profile"))
    {
      std::string profileName = this->dataPtr->vm["profile"].as<std::string>();
      if (physics::get_world()->PresetMgr()->HasProfile(profileName))
      {
        physics::get_world()->PresetMgr()->CurrentProfile(profileName);
        gzmsg << "Setting physics profile to [" << profileName << "]."
              << std::endl;
      }
      else
      {
        gzerr << "Specified profile [" << profileName << "] was not found."
              << std::endl;
      }
    }
  }

  if (this->dataPtr->vm.count("initial_sim_time"))
  {
    try
    {
      common::Time initialSimTime {
        this->dataPtr->vm["initial_sim_time"].as<double>()};
      physics::get_world()->SetSimTime(initialSimTime);
      physics::get_world()->SetInitialSimTime(initialSimTime);
      gzmsg << "Setting initial sim time to [" <<
        physics::get_world()->SimTime() << "]\n" << std::endl;
    }
    catch(...)
    {
      gzerr << "Unable to cast initial_sim_time[" <<
        this->dataPtr->vm["initial_sim_time"].as<double>() << "] "
        << "to double for setting initial sim time\n" << std::endl;
    }
  }

  this->ProcessParams();

  return true;
}

/////////////////////////////////////////////////
bool Server::GetInitialized() const
{
  return !this->dataPtr->stop && this->dataPtr->initialized;
}

/////////////////////////////////////////////////
bool Server::LoadFile(const std::string &_filename,
                      const std::string &_physics)
{
  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF);
  if (!sdf::init(sdf))
  {
    gzerr << "Unable to initialize sdf\n";
    return false;
  }

  std::string filename = _filename;
  auto filenameUri = ignition::common::URI(filename);

  if (filenameUri.Scheme() == "http" || filenameUri.Scheme() == "https")
  {
    std::string downloadedDir = common::FuelModelDatabase::Instance()->WorldPath(filename);
    // Find the first sdf file in the world path for now, the later intention is
    // to load an optional world config file first and if that does not exist,
    // continue to load the first sdf file found as done below
    for (ignition::common::DirIter file(downloadedDir);
         file != ignition::common::DirIter(); ++file)
    {
      std::string current(*file);
      if (ignition::common::isFile(current))
      {
        std::string fileName = ignition::common::basename(current);
        std::string::size_type fileExtensionIndex = fileName.rfind(".");
        std::string fileExtension = fileName.substr(fileExtensionIndex + 1);

        if (fileExtension == "sdf" || fileExtension == "world")
        {
          filename = current.c_str();
          if (!sdf::readFile(filename, sdf))
          {
            gzerr << "Unable to read SDF from URL[" << filename << "]\n";
            return false;
          }
          gzmsg << "Loading world file [" << filename << "]" << std::endl;
        }
      }
    }
  }
  else
  {
    auto foundFile = common::find_file(filename);

    // Quick test for a valid file
    FILE *test = fopen(foundFile.c_str(), "r");
    if (!test)
    {
      gzerr << "Could not open file[" << filename << "]\n";
      return false;
    }
    fclose(test);

    if (!sdf::readFile(foundFile, sdf))
    {
      gzerr << "Unable to read sdf file[" << filename << "]\n";
      return false;
    }

    gzmsg << "Loading world file [" << foundFile << "]" << std::endl;
  }
  
  return this->LoadImpl(sdf->Root(), _physics);
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

  return this->LoadImpl(sdf->Root());
}

/////////////////////////////////////////////////
bool Server::PreLoad()
{
  // setup gazebo
  return gazebo::setupServer(this->dataPtr->systemPluginsArgc,
                             this->dataPtr->systemPluginsArgv);
}

/////////////////////////////////////////////////
bool Server::LoadImpl(sdf::ElementPtr _elem,
                      const std::string &_physics)
{
  this->dataPtr->InspectSDFElement(_elem);

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

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init("/gazebo");
  this->dataPtr->serverSub =
    this->dataPtr->node->Subscribe("/gazebo/server/control",
                                   &Server::OnControl, this);

  this->dataPtr->worldModPub =
    this->dataPtr->node->Advertise<msgs::WorldModify>("/gazebo/world/modify");

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

  if (this->dataPtr->lockstep)
    physics::init_worlds(rendering::update_scene_poses);
  else
    physics::init_worlds(nullptr);

  this->dataPtr->stop = false;

  return true;
}

/////////////////////////////////////////////////
void Server::SigInt(int)
{
  event::Events::stop();
  ServerPrivate::stop = true;

  // Signal to plugins/etc that a shutdown event has occured
  event::Events::sigInt();
}

/////////////////////////////////////////////////
void Server::Stop()
{
  event::Events::stop();
  this->dataPtr->stop = true;
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
#ifdef _WIN32
  signal(SIGINT, Server::SigInt);
#else
  struct sigaction sigact;
  sigact.sa_flags = 0;
  sigact.sa_handler = Server::SigInt;
  if (sigemptyset(&sigact.sa_mask) != 0)
    std::cerr << "sigemptyset failed while setting up for SIGINT" << std::endl;
  if (sigaction(SIGINT, &sigact, NULL))
    std::cerr << "sigaction(2) failed while setting up for SIGINT" << std::endl;

  // The following was added in
  // https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2923,
  // but it is causing shutdown issues when gazebo is used with ros.
  // if (sigaction(SIGTERM, &sigact, NULL))
  // {
  //   std::cerr << "sigaction(15) failed while setting up for SIGTERM"
  //             << std::endl;
  // }
#endif

  if (this->dataPtr->stop)
    return;

  // Make sure the sensors are updated once before running the world.
  // This makes sure plugins get loaded properly.
  sensors::run_once(true);

  // Run the sensor threads
  sensors::run_threads();

  unsigned int iterations = 0;
  common::StrStr_M::iterator piter = this->dataPtr->params.find("iterations");
  if (piter != this->dataPtr->params.end())
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

  this->dataPtr->initialized = true;

  IGN_PROFILE_THREAD_NAME("gzserver");
  // Stay on this loop until Gazebo needs to be shut down
  // The server and sensor manager outlive worlds
  while (!this->dataPtr->stop)
  {
    IGN_PROFILE("Server::Run");
    IGN_PROFILE_BEGIN("ProcessControlMsgs");
    if (this->dataPtr->lockstep)
      rendering::wait_for_render_request("", 0.100);
    // bool ret = rendering::wait_for_render_request("", 0.100);
    // if (ret == false)
    //   gzerr << "time out reached!" << std::endl;

    this->ProcessControlMsgs();
    IGN_PROFILE_END();

    if (physics::worlds_running())
    {
      IGN_PROFILE_BEGIN("run_once");
      sensors::run_once();
      IGN_PROFILE_END();
    }
    else if (sensors::running())
    {
      IGN_PROFILE_BEGIN("stop");
      sensors::stop();
      IGN_PROFILE_END();
    }

    if (!this->dataPtr->lockstep)
      common::Time::MSleep(1);
  }

  // Shutdown gazebo
  gazebo::shutdown();
}

/////////////////////////////////////////////////
void Server::ProcessParams()
{
  bool p = this->dataPtr->vm.count("pause") > 0;
  physics::pause_worlds(p);
  common::StrStr_M::const_iterator iter;
  for (iter = this->dataPtr->params.begin();
       iter != this->dataPtr->params.end();
       ++iter)
  {
    if (iter->first == "record")
    {
      util::LogRecordParams params;

      params.encoding = this->dataPtr->params["record_encoding"];
      params.path = iter->second;
      params.period = this->dataPtr->vm["record_period"].as<double>();
      params.filter = this->dataPtr->vm["record_filter"].as<std::string>();
      params.recordResources =
          this->dataPtr->params.count("record_resources") > 0;
      util::LogRecord::Instance()->Start(params);
    }
  }
}

/////////////////////////////////////////////////
void Server::SetParams(const common::StrStr_M &_params)
{
  common::StrStr_M::const_iterator iter;
  for (iter = _params.begin(); iter != _params.end(); ++iter)
    this->dataPtr->params[iter->first] = iter->second;
}

/////////////////////////////////////////////////
void Server::OnControl(ConstServerControlPtr &_msg)
{
  std::unique_lock<std::mutex> lock(this->dataPtr->receiveMutex);
  this->dataPtr->controlMsgs.push_back(*_msg);
}

/////////////////////////////////////////////////
void Server::ProcessControlMsgs()
{
  std::list<msgs::ServerControl>::iterator iter;
  for (iter = this->dataPtr->controlMsgs.begin();
       iter != this->dataPtr->controlMsgs.end(); ++iter)
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
      this->dataPtr->worldModPub->Publish(worldMsg);
    }
    else if ((*iter).has_save_world_name())
    {
      // Get the world pointer.
      physics::WorldPtr world;
      try
      {
        world = physics::get_world((*iter).save_world_name());
      }
      catch(const common::Exception &)
      {
        gzerr << "Unable to save world. Unknown world ["
               << (*iter).save_world_name() << "]" << std::endl;
      }

      if (world && (*iter).has_save_filename())
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
  this->dataPtr->controlMsgs.clear();
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

  sdf::ElementPtr worldElem = sdf->Root()->GetElement("world");

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
