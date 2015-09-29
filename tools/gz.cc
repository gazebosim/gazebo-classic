/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include <signal.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include "gz_topic.hh"
#include "gz_log.hh"
#include "gz.hh"

using namespace gazebo;

boost::mutex Command::sigMutex;
boost::condition_variable Command::sigCondition;

std::map<std::string, Command *> g_commandMap;

/////////////////////////////////////////////////
Command::Command(const std::string &_name, const std::string &_brief)
  : name(_name), brief(_brief), visibleOptions("Options"), argc(0), argv(NULL)
{
  this->visibleOptions.add_options()
    ("help,h", "Print this help message");
}

/////////////////////////////////////////////////
Command::~Command()
{
  delete [] this->argv;
  this->argv = NULL;
}

/////////////////////////////////////////////////
void Command::Signal()
{
  boost::mutex::scoped_lock lock(sigMutex);
  sigCondition.notify_all();
}

/////////////////////////////////////////////////
void Command::ListOptions()
{
  std::vector<std::string> pieces;

  std::vector<boost::shared_ptr<po::option_description> >::const_iterator iter;
  for (iter = this->visibleOptions.options().begin();
      iter != this->visibleOptions.options().end(); ++iter)
  {
    pieces.clear();
    std::string formatName = (*iter)->format_name();
    boost::split(pieces, formatName, boost::is_any_of(" "));

    if (pieces.empty())
    {
      std::cerr << "Unable to process list options.\n";
      return;
    }

    // Output the short name option, or long name if there is no shortname
    std::cout << pieces[0] << std::endl;

    // Output the long name option if it exists.
    if (pieces.size() > 3)
      std::cout << pieces[2] << std::endl;
  }
}

/////////////////////////////////////////////////
void Command::Help()
{
  std::cerr << " gz " << this->name << " [options]\n\n";
  this->HelpDetailed();
  std::cerr << this->visibleOptions << "\n";
}

/////////////////////////////////////////////////
std::string Command::GetBrief() const
{
  return this->brief;
}

/////////////////////////////////////////////////
bool Command::TransportInit()
{
  // Some command require transport, and some do not. Only initialize
  // transport if required.
  if (!this->TransportRequired())
    return true;

  // Setup transport (communication)
  if (!transport::init("", 0, 1))
    return false;

  // Run transport (communication)
  transport::run();

  return true;
}

/////////////////////////////////////////////////
bool Command::TransportRequired()
{
  return true;
}

/////////////////////////////////////////////////
bool Command::TransportFini()
{
  transport::fini();
  return true;
}

/////////////////////////////////////////////////
bool Command::Run(int _argc, char **_argv)
{
  // save a copy of argc and argv for consumption by child commands
  this->argc = _argc;
  this->argv = new char*[_argc];
  for (int i = 0; i < _argc; ++i)
  {
    int argvLen = strlen(_argv[i]) + 1;
    this->argv[i] = new char[argvLen];
    snprintf(this->argv[i], argvLen, "%s", _argv[i]);
  }

  // The SDF find file callback.
  sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command")
    ("pass", po::value<std::vector<std::string> >(), "pass through");

  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions).add(this->visibleOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1).add("pass", -1);

  try
  {
    po::store(
        po::command_line_parser(_argc, _argv).options(allOptions).positional(
          positional).run(), this->vm);
    po::notify(this->vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n";
    return false;
  }

  if (this->vm.count("help"))
  {
    this->Help();
    return true;
  }

  if (!this->TransportInit())
  {
    std::cerr << "An instance of Gazebo is not running.\n";
    return false;
  }

  bool result = this->RunImpl();

  this->TransportFini();

  return result;
}

/////////////////////////////////////////////////
WorldCommand::WorldCommand()
  : Command("world", "Modify world properties")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("pause,p", po::value<bool>(), "Pause/unpause simulation. "
     "0=unpause, 1=pause.")
    ("step,s", "Step simulation one iteration.")
    ("multi-step,m", po::value<uint32_t>(),
     "Step simulation mulitple iteration.")
    ("reset-all,r", "Reset time and model poses")
    ("reset-time,t", "Reset time")
    ("reset-models,o", "Reset models");
}

/////////////////////////////////////////////////
void WorldCommand::HelpDetailed()
{
  std::cerr <<
    "\tChange properties of a Gazebo world on a running\n "
    "\tserver. If a name for the world, option -w, is not specified\n"
    "\tthe first world found on the Gazebo master will be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool WorldCommand::RunImpl()
{
  std::string worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::PublisherPtr pub =
    node->Advertise<msgs::WorldControl>("~/world_control");
  pub->WaitForConnection();

  msgs::WorldControl msg;
  bool good = false;

  if (this->vm.count("pause"))
  {
    msg.set_pause(this->vm["pause"].as<bool>());
    good = true;
  }

  if (this->vm.count("step"))
  {
    msg.set_step(true);
    good = true;
  }

  if (this->vm.count("multi-step"))
  {
    msg.set_multi_step(this->vm["multi-step"].as<uint32_t>());
    good = true;
  }

  if (this->vm.count("reset-all"))
  {
    msg.mutable_reset()->set_all(true);
    good = true;
  }

  if (this->vm.count("reset-time"))
  {
    msg.mutable_reset()->set_time_only(true);
    good = true;
  }

  if (this->vm.count("reset-models"))
  {
    msg.mutable_reset()->set_model_only(true);
    good = true;
  }

  if (good)
    pub->Publish(msg, true);
  else
    this->Help();

  return true;
}

/////////////////////////////////////////////////
PhysicsCommand::PhysicsCommand()
  : Command("physics", "Modify properties of the physics engine")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("gravity,g", po::value<std::string>(),
     "Gravity vector. Comma separated 3-tuple without whitespace, "
     "eg: -g 0,0,-9.8")
    ("step-size,s", po::value<double>(), "Maximum step size (seconds).")
    ("iters,i", po::value<double>(), "Number of iterations.")
    ("update-rate,u", po::value<double>(), "Target real-time update rate.");
}

/////////////////////////////////////////////////
void PhysicsCommand::HelpDetailed()
{
  std::cerr <<
    "\tChange properties of the physics engine on a specific\n"
    "\tworld. If a name for the world, option -w, is not specified,\n"
    "\tthe first world found on the Gazebo master will be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool PhysicsCommand::RunImpl()
{
  std::string worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::PublisherPtr pub =
    node->Advertise<msgs::Physics>("~/physics");
  pub->WaitForConnection();

  msgs::Physics msg;

  bool good = false;

  if (this->vm.count("step-size"))
  {
    msg.set_max_step_size(this->vm["step-size"].as<double>());
    good = true;
  }

  if (this->vm.count("iters"))
  {
    msg.set_iters(this->vm["iters"].as<double>());
    good = true;
  }

  if (this->vm.count("update-rate"))
  {
    msg.set_real_time_update_rate(this->vm["update-rate"].as<double>());
    good = true;
  }

  if (this->vm.count("gravity"))
  {
    std::vector<std::string> values;
    boost::split(values, this->vm["gravity"].as<std::string>(),
        boost::is_any_of(","));

    msg.mutable_gravity()->set_x(boost::lexical_cast<double>(values[0]));
    msg.mutable_gravity()->set_y(boost::lexical_cast<double>(values[1]));
    msg.mutable_gravity()->set_z(boost::lexical_cast<double>(values[2]));
    good = true;
  }

  if (good)
    pub->Publish(msg, true);
  else
    this->Help();

  return true;
}

/////////////////////////////////////////////////
ModelCommand::ModelCommand()
  : Command("model", "Modify properties of a model")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("model-name,m", po::value<std::string>(), "Model name.")
    ("world-name,w", po::value<std::string>(), "World name.")
    ("delete,d", "Delete a model.")
    ("spawn-file,f", po::value<std::string>(), "Spawn model from SDF file.")
    ("spawn-string,s", "Spawn model from SDF string, pass by a pipe.")
    ("info,i", "Output model state information to the terminal.")
    ("pose,p",
     "Output model pose as a space separated 6-tuple: x y z roll pitch yaw.")
    ("pose-x,x", po::value<double>(), "x value")
    ("pose-y,y", po::value<double>(), "y value")
    ("pose-z,z", po::value<double>(), "z value")
    ("pose-R,R", po::value<double>(), "roll in radians.")
    ("pose-P,P", po::value<double>(), "pitch in radians.")
    ("pose-Y,Y", po::value<double>(), "yaw in radians.");
}

/////////////////////////////////////////////////
void ModelCommand::HelpDetailed()
{
  std::cerr <<
    "\tChange properties of a model, delete a model, or\n"
    "\tspawn a new model. If a name for the world, option -w, is\n"
    "\tnot pecified, the first world found on the Gazebo master\n"
    "\twill be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool ModelCommand::RunImpl()
{
  std::string modelName, worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  if (this->vm.count("model-name"))
    modelName = this->vm["model-name"].as<std::string>();
  else
  {
    std::cerr << "A model name is required using the "
      << "(-m <model_name> command line argument)\n";
    std::cerr << "For more information: gz help model.\n";
    return false;
  }

  math::Pose pose;
  math::Vector3 rpy;

  if (this->vm.count("pose-x"))
    pose.pos.x = this->vm["pose-x"].as<double>();
  if (this->vm.count("pose-y"))
    pose.pos.y = this->vm["pose-y"].as<double>();
  if (this->vm.count("pose-z"))
    pose.pos.z = this->vm["pose-z"].as<double>();
  if (this->vm.count("pose-R"))
    rpy.x = this->vm["pose-R"].as<double>();
  if (this->vm.count("pose-P"))
    rpy.y = this->vm["pose-P"].as<double>();
  if (this->vm.count("pose-Y"))
    rpy.z = this->vm["pose-Y"].as<double>();
  pose.rot.SetFromEuler(rpy);

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  if (this->vm.count("delete"))
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete", modelName);
    transport::PublisherPtr pub = node->Advertise<msgs::Request>("~/request");
    pub->WaitForConnection();
    pub->Publish(*msg, true);
    delete msg;
  }
  else if (this->vm.count("spawn-file"))
  {
    std::string filename = this->vm["spawn-file"].as<std::string>();

    std::ifstream ifs(filename.c_str());
    if (!ifs)
    {
      std::cerr << "Error: Unable to open file[" << filename << "]\n";
      return false;
    }

    sdf::SDFPtr sdf(new sdf::SDF());
    if (!sdf::init(sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed" << std::endl;
      return false;
    }

    if (!sdf::readFile(filename, sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return false;
    }

    return this->ProcessSpawn(sdf, modelName, pose, node);
  }
  else if (this->vm.count("spawn-string"))
  {
    std::string input;
    std::string sdfString;

    // Read input from the command line.
    while (std::getline(std::cin, input))
    {
      sdfString += input;
    }

    sdf::SDFPtr sdf(new sdf::SDF());
    if (!sdf::init(sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed" << std::endl;
      return false;
    }

    if (!sdf::readString(sdfString, sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return false;
    }

    return this->ProcessSpawn(sdf, modelName, pose, node);
  }
  else if (this->vm.count("info") || this->vm.count("pose"))
  {
    boost::shared_ptr<msgs::Response> response = gazebo::transport::request(
        worldName, "entity_info", modelName);
    gazebo::msgs::Model modelMsg;

    if (response->has_serialized_data() &&
        !response->serialized_data().empty() &&
        modelMsg.ParseFromString(response->serialized_data()))
    {
      if (this->vm.count("info"))
        std::cout << modelMsg.DebugString() << std::endl;
      else if (this->vm.count("pose"))
        std::cout << gazebo::msgs::Convert(modelMsg.pose()) << std::endl;
    }
    else
    {
      std::string tmpWorldName = worldName.empty() ? "default" : worldName;
      std::cout << "Unable to get info on model[" << modelName << "] in "
        << "the world[" << tmpWorldName << "]\n";
    }
  }
  else
  {
    transport::PublisherPtr pub =
      node->Advertise<msgs::Model>("~/model/modify");
    pub->WaitForConnection();

    msgs::Model msg;
    msg.set_name(modelName);
    msgs::Set(msg.mutable_pose(), pose);
    pub->Publish(msg, true);
  }

  return true;
}

/////////////////////////////////////////////////
bool ModelCommand::ProcessSpawn(sdf::SDFPtr _sdf,
    const std::string &_name, const math::Pose &_pose, transport::NodePtr _node)
{
  sdf::ElementPtr modelElem = _sdf->root->GetElement("model");

  if (!modelElem)
  {
    gzerr << "Unable to find <model> element.\n";
    return false;
  }

  // Set the model name
  if (!_name.empty())
    modelElem->GetAttribute("name")->SetFromString(_name);

  transport::PublisherPtr pub = _node->Advertise<msgs::Factory>("~/factory");
  pub->WaitForConnection();

  msgs::Factory msg;
  msg.set_sdf(_sdf->ToString());
  msgs::Set(msg.mutable_pose(), _pose);
  pub->Publish(msg, true);

  return true;
}


/////////////////////////////////////////////////
JointCommand::JointCommand()
  : Command("joint", "Modify properties of a joint")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("model-name,m", po::value<std::string>(), "Model name.")
    ("joint-name,j", po::value<std::string>(), "Joint name.")
    ("delete,d", "Delete a model.")
    ("force,f", po::value<double>(), "Force to apply to a joint.")
    ("pos-t", po::value<double>(), "Target angle.")
    ("pos-p", po::value<double>(), "Position proportional gain.")
    ("pos-i", po::value<double>(), "Position integral gain.")
    ("pos-d", po::value<double>(), "Position differential gain.")
    ("vel-t", po::value<double>(), "Target speed.")
    ("vel-p", po::value<double>(), "Velocity proportional gain.")
    ("vel-i", po::value<double>(), "Velocity integral gain.")
    ("vel-d", po::value<double>(), "Velocity differential gain.");
}

/////////////////////////////////////////////////
void JointCommand::HelpDetailed()
{
  std::cerr <<
    "\tChange properties of a joint. If a name for the world, \n"
    "\toption -w, is not specified, the first world found on \n"
    "\tthe Gazebo master will be used.\n"
    "\tA model name and joint name are required.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool JointCommand::RunImpl()
{
  std::string modelName, worldName, jointName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  if (this->vm.count("model-name"))
    modelName = this->vm["model-name"].as<std::string>();
  else
  {
    std::cerr << "A model name is required using the "
      << "(-m <model_name> command line argument)\n";
    std::cerr << "For more information: gz help joint.\n";
    return false;
  }

  if (this->vm.count("joint-name"))
    jointName = this->vm["joint-name"].as<std::string>();
  else
  {
    std::cerr << "A joint name is required using the "
      << "(-j <joint_name> command line argument)\n";
    std::cerr << "For more information: gz help joint.\n";
    return false;
  }

  msgs::JointCmd msg;
  msg.set_name(modelName + "::" + jointName);

  if (this->vm.count("force"))
    msg.set_force(this->vm["force"].as<double>());

  if (this->vm.count("pos-t"))
  {
    msg.mutable_position()->set_target(this->vm["pos-t"].as<double>());

    if (this->vm.count("pos-p"))
      msg.mutable_position()->set_p_gain(this->vm["pos-p"].as<double>());

    if (this->vm.count("pos-i"))
      msg.mutable_position()->set_i_gain(this->vm["pos-i"].as<double>());

    if (this->vm.count("pos-d"))
      msg.mutable_position()->set_d_gain(this->vm["pos-d"].as<double>());
  }

  if (this->vm.count("vel-t"))
  {
    msg.mutable_velocity()->set_target(this->vm["vel-t"].as<double>());

    if (this->vm.count("vel-p"))
      msg.mutable_velocity()->set_p_gain(this->vm["vel-p"].as<double>());

    if (this->vm.count("vel-i"))
      msg.mutable_velocity()->set_i_gain(this->vm["vel-i"].as<double>());

    if (this->vm.count("vel-d"))
      msg.mutable_velocity()->set_d_gain(this->vm["vel-d"].as<double>());
  }

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::PublisherPtr pub =
    node->Advertise<msgs::JointCmd>(
        std::string("~/") + modelName + "/joint_cmd");

  pub->WaitForConnection();

  pub->Publish(msg, true);

  return true;
}

/////////////////////////////////////////////////
CameraCommand::CameraCommand()
  : Command("camera", "Control a camera")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("camera-name,c", po::value<std::string>(),
     "Camera name. Use gz camera -l to get a list of camera names.")
    ("list,l", "List all cameras")
    ("follow,f", po::value<std::string>(), "Model to follow.");
}

/////////////////////////////////////////////////
void CameraCommand::HelpDetailed()
{
  std::cerr <<
    "\tChange properties of a camera. If a name for the world, \n"
    "\toption -w, is not specified, the first world found on \n"
    "\tthe Gazebo master will be used.\n"
    "\tA camera name is required.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool CameraCommand::RunImpl()
{
  std::string cameraName, worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  if (this->vm.count("list"))
  {
    transport::ConnectionPtr connection = transport::connectToMaster();

    if (connection)
    {
      msgs::Packet packet;
      msgs::Request *request;
      msgs::GzString_V topics;
      std::string data;

      request = msgs::CreateRequest("get_topics");
      request->set_id(0);
      connection->EnqueueMsg(msgs::Package("request", *request), true);
      connection->Read(data);

      packet.ParseFromString(data);
      topics.ParseFromString(packet.serialized_data());

      for (int i = 0; i < topics.data_size(); ++i)
      {
        request = msgs::CreateRequest("topic_info", topics.data(i));
        connection->EnqueueMsg(msgs::Package("request", *request), true);

        int j = 0;
        do
        {
          connection->Read(data);
          packet.ParseFromString(data);
        } while (packet.type() != "topic_info_response" && ++j < 10);

        msgs::TopicInfo topicInfo;

        if (j <10)
          topicInfo.ParseFromString(packet.serialized_data());
        else
        {
          std::cerr << "Unable to get info for topic["
                    << topics.data(i) << "]\n";
        }

        if (topicInfo.msg_type() == "gazebo.msgs.CameraCmd")
        {
          std::vector<std::string> parts;
          boost::split(parts, topics.data(i), boost::is_any_of("/"));
          std::cout << parts[parts.size()-2] << std::endl;
        }
      }
    }
    else
    {
      std::cerr << "Unable to connect to a running instance of gazebo.\n";
    }

    return true;
  }


  if (this->vm.count("camera-name"))
    cameraName = this->vm["camera-name"].as<std::string>();
  else
  {
    std::cerr << "A camera name is required using the "
      << "(-c <camera_name> command line argument)\n";
    std::cerr << "For more information: gz help camera\n";
    return false;
  }

  msgs::CameraCmd msg;
  if (this->vm.count("follow"))
    msg.set_follow_model(this->vm["follow"].as<std::string>());

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::PublisherPtr pub =
    node->Advertise<msgs::CameraCmd>(
        std::string("~/") + cameraName + "/cmd");

  pub->WaitForConnection();

  pub->Publish(msg, true);

  return true;
}

/////////////////////////////////////////////////
StatsCommand::StatsCommand()
  : Command("stats", "Print statistics about a running gzserver instance.")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("duration,d", po::value<double>(), "Duration (seconds) to run.")
    ("plot,p", "Output comma-separated values, useful for processing and "
     "plotting.");
}

/////////////////////////////////////////////////
void StatsCommand::HelpDetailed()
{
  std::cerr <<
    "\tPrint gzserver statics to standard out. If a name for the world, \n"
    "\toption -w, is not specified, the first world found on \n"
    "\tthe Gazebo master will be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool StatsCommand::RunImpl()
{
  std::string worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::SubscriberPtr sub =
    node->Subscribe("~/world_stats", &StatsCommand::CB, this);

  boost::mutex::scoped_lock lock(this->sigMutex);
  if (this->vm.count("duration"))
    this->sigCondition.timed_wait(lock,
        boost::posix_time::seconds(this->vm["duration"].as<double>()));
  else
    this->sigCondition.wait(lock);

  return true;
}

/////////////////////////////////////////////////
void StatsCommand::CB(ConstWorldStatisticsPtr &_msg)
{
  GZ_ASSERT(_msg, "Invalid message received");

  double percent = 0;
  char paused;
  common::Time simTime  = msgs::Convert(_msg->sim_time());
  common::Time realTime = msgs::Convert(_msg->real_time());

  this->simTimes.push_back(msgs::Convert(_msg->sim_time()));
  if (this->simTimes.size() > 20)
    this->simTimes.pop_front();

  this->realTimes.push_back(msgs::Convert(_msg->real_time()));
  if (this->realTimes.size() > 20)
    this->realTimes.pop_front();

  common::Time simAvg, realAvg;
  std::list<common::Time>::iterator simIter, realIter;
  simIter = ++(this->simTimes.begin());
  realIter = ++(this->realTimes.begin());
  while (simIter != this->simTimes.end() && realIter != this->realTimes.end())
  {
    simAvg += ((*simIter) - this->simTimes.front());
    realAvg += ((*realIter) - this->realTimes.front());
    ++simIter;
    ++realIter;
  }

  // Prevent divide by zero
  if (realAvg <= 0)
    return;

  simAvg = simAvg / realAvg;

  if (simAvg > 0)
    percent = simAvg.Double();
  else
    percent = 0;


  if (_msg->paused())
    paused = 'T';
  else
    paused = 'F';

  if (this->vm.count("plot"))
  {
    static bool first = true;
    if (first)
    {
      std::cout << "# real-time factor (percent), simtime (sec), "
        << "realtime (sec), paused (T or F)\n";
      first = false;
    }
    printf("%4.2f, %16.6f, %16.6f, %c\n",
        percent, simTime.Double(), realTime.Double(), paused);
    fflush(stdout);
  }
  else
    printf("Factor[%4.2f] SimTime[%4.2f] RealTime[%4.2f] Paused[%c]\n",
        percent, simTime.Double(), realTime.Double(), paused);
}

/////////////////////////////////////////////////
SDFCommand::SDFCommand()
  : Command("sdf",
      "Converts between SDF versions, and provides info about SDF files")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("describe,d", "Print SDF format for given version(-v).")
    ("convert,c", po::value<std::string>(),
     "In place conversion of arg to the latest SDF version.")
    ("doc,o", "Print HTML SDF. Use -v to specify version.")
    ("check,k", po::value<std::string>(), "Validate arg.")
    ("version,v", po::value<std::string>(),
     "Version of SDF to use with other options.")
    ("print,p", po::value<std::string>(),
     "Print arg, useful for debugging and as a conversion tool.");
}

/////////////////////////////////////////////////
void SDFCommand::HelpDetailed()
{
  std::cerr <<
    "\tIntrospect, convert, and output SDF files.\n"
    "\tUse the -v option to specify the version of\n"
    "\tSDF for use with other options.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool SDFCommand::TransportRequired()
{
  return false;
}

/////////////////////////////////////////////////
bool SDFCommand::RunImpl()
{
  sdf::SDF::version = SDF_VERSION;

  try
  {
    // Initialize the informational logger. This will log warnings and errors.
    gzLogInit("gz-", "gzsdf.log");
  }
  catch(gazebo::common::Exception &_e)
  {
    _e.Print();
    std::cerr << "Error initializing log file" << std::endl;
  }

  sdf::SDFPtr sdf(new sdf::SDF());

  if (this->vm.count("version"))
  {
    try
    {
      sdf::SDF::version = boost::lexical_cast<std::string>(
          this->vm["version"].as<std::string>());
    }
    catch(...)
    {
      gzerr << "Invalid version number["
        <<this->vm["version"].as<std::string>() << "]\n";
      return false;
    }
  }

  if (!sdf::init(sdf))
  {
    std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
    return -1;
  }

  if (this->vm.count("check"))
  {
    boost::filesystem::path path = this->vm["check"].as<std::string>();

    if (!boost::filesystem::exists(path))
      std::cerr << "Error: File doesn't exist[" << path.string() << "]\n";

    if (!sdf::readFile(path.string(), sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return -1;
    }

    std::cout << "Check complete\n";
  }
  else if (this->vm.count("describe"))
  {
    sdf->PrintDescription();
  }
  else if (this->vm.count("doc"))
  {
    sdf->PrintDoc();
  }
  else if (this->vm.count("convert"))
  {
    boost::filesystem::path path = this->vm["convert"].as<std::string>();

    if (!boost::filesystem::exists(path))
      std::cerr << "Error: File doesn't exist[" << path.string() << "]\n";

    TiXmlDocument xmlDoc;
    if (xmlDoc.LoadFile(path.string()))
    {
      if (sdf::Converter::Convert(&xmlDoc, sdf::SDF::version, true))
      {
        // Create an XML printer to control formatting
        TiXmlPrinter printer;
        printer.SetIndent("  ");
        xmlDoc.Accept(&printer);

        // Output the XML
        std::ofstream stream(path.string().c_str(), std::ios_base::out);
        stream << printer.Str();
        stream.close();

        std::cout << "Success\n";
      }
    }
    else
    {
      std::cerr << "Unable to load file[" << path.string() << "]\n";
      return false;
    }
  }
  else if (this->vm.count("print"))
  {
    boost::filesystem::path path = this->vm["print"].as<std::string>();

    if (!boost::filesystem::exists(path))
      std::cerr << "Error: File doesn't exist[" << path.string() << "]\n";

    if (!sdf::readFile(path.string(), sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return false;
    }
    sdf->PrintValues();
  }
  else
  {
    this->Help();
  }

  return true;
}

/////////////////////////////////////////////////
HelpCommand::HelpCommand()
  : Command("help",
      "Outputs information about a command")
{
  // Options that are visible to the user through help.
  // this->visibleOptions.add_options()
  //  ("option,o", po::value<std::string>(), "Show the command options.");
}

/////////////////////////////////////////////////
void HelpCommand::HelpDetailed()
{
  std::cerr <<
    "\tOutput information about a gz command.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool HelpCommand::TransportRequired()
{
  return false;
}

/////////////////////////////////////////////////
bool HelpCommand::RunImpl()
{
  std::string option;
  if (vm.count("pass") && !vm["pass"].as<std::vector<std::string> >().empty())
    option = vm["pass"].as<std::vector<std::string> >()[0];

  this->Help(option);

  return true;
}

/////////////////////////////////////////////////
void HelpCommand::Help(const std::string &_command)
{
  std::cerr << "This tool modifies various aspects of a "
    << "running Gazebo simulation.\n\n";

  if (_command.empty() || g_commandMap.find(_command) == g_commandMap.end())
  {
    std::cerr << "  Usage:  gz <command>\n\n"
      << "List of commands:\n\n";

    std::cerr << "  " << std::left << std::setw(10) << std::setfill(' ')
      << "help";
    std::cerr << "Print this help text.\n";

    for (std::map<std::string, Command*>::iterator iter = g_commandMap.begin();
        iter != g_commandMap.end(); ++iter)
    {
      std::cerr << "  " << std::left << std::setw(10) << std::setfill(' ')
        << iter->first;
      std::cerr << iter->second->GetBrief() << "\n";
    }

    std::cerr << "\n\n";
    std::cerr << "Use \"gz help <command>\" to print help for a "
      "command.\n";
  }
  else if (g_commandMap.find(_command) != g_commandMap.end())
    g_commandMap[_command]->Help();
}

/////////////////////////////////////////////////
DebugCommand::DebugCommand()
  : Command("debug",
      "Returns completion list for a command. Used for bash completion.")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("option,o", po::value<std::string>(), "Show the command options.");
}

/////////////////////////////////////////////////
void DebugCommand::HelpDetailed()
{
  std::cerr <<
    "\tUsed primarily for bash completion, this tool\n"
    "\treturn the completion list for a given command.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool DebugCommand::TransportRequired()
{
  return false;
}

/////////////////////////////////////////////////
bool DebugCommand::RunImpl()
{
  if (this->vm.count("option") <= 0)
  {
    for (std::map<std::string, Command*>::iterator iter = g_commandMap.begin();
        iter != g_commandMap.end(); ++iter)
    {
      std::cout << iter->first << "\n";
    }
  }
  else
  {
    std::map<std::string, Command *>::iterator iter =
      g_commandMap.find(this->vm["option"].as<std::string>());
    if (iter != g_commandMap.end())
      iter->second->ListOptions();
  }

  return true;
}

//////////////////////////////////////////////////
void SignalHandler(int /*dummy*/)
{
  Command::Signal();
  return;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  gazebo::common::Console::SetQuiet(true);

  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command")
    ("pass", po::value<std::vector<std::string> >(), "pass through");

  // Both the hidden and visible options
  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1).add("pass", -1);

  po::variables_map vm;

  try
  {
    po::store(
        po::command_line_parser(argc, argv).options(allOptions).positional(
          positional).allow_unregistered().run(), vm);

    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n";
    return -1;
  }

  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  g_commandMap["camera"] = new CameraCommand();
  g_commandMap["help"] = new HelpCommand();
  g_commandMap["joint"] = new JointCommand();
  g_commandMap["model"] = new ModelCommand();
  g_commandMap["world"] = new WorldCommand();
  g_commandMap["physics"] = new PhysicsCommand();
  g_commandMap["stats"] = new StatsCommand();
  g_commandMap["topic"] = new TopicCommand();
  g_commandMap["log"] = new LogCommand();
  g_commandMap["sdf"] = new SDFCommand();
  g_commandMap["debug"] = new DebugCommand();

  // Get the command name
  std::string command =
    vm.count("command") ? vm["command"].as<std::string>() : "";

  std::map<std::string, Command *>::iterator iter =
    g_commandMap.find(command);

  int result = 0;

  if (iter != g_commandMap.end())
  {
    g_commandMap[command]->Run(argc, argv);
  }
  else
  {
    g_commandMap["help"]->Run(argc, argv);
    result = -1;
  }

  for (iter = g_commandMap.begin(); iter != g_commandMap.end(); ++iter)
    delete iter->second;

  return result;
}
