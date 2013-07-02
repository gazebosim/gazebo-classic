/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <boost/algorithm/string.hpp>

#include <gazebo/transport/transport.hh>
#include "gz.hh"

using namespace gazebo;

std::map<std::string, Command *> g_commandMap;

/////////////////////////////////////////////////
void Help(const std::string &_command)
{
  std::cout << "This tool modifies various aspects of a "
            << "running Gazebo simulation.\n\n";

  if (_command.empty() || g_commandMap.find(_command) == g_commandMap.end())
  {
    std::cout << "  Usage:  gzmodify <command>\n\n"
      << "List of commands:\n\n";

    std::cout << "  " << std::left << std::setw(10) << std::setfill(' ')
      << "help";
    std::cout << "Print this help text.\n";

    for (std::map<std::string, Command*>::iterator iter = g_commandMap.begin();
        iter != g_commandMap.end(); ++iter)
    {
      std::cout << "  " << std::left << std::setw(10) << std::setfill(' ')
        << iter->first;
      std::cout << iter->second->GetBrief() << "\n";
    }

    std::cout << "\n\n";
    std::cout << "Use \"gzmodify help <command>\" to print help for a "
      "command.\n";
  }
  else if (g_commandMap.find(_command) != g_commandMap.end())
    g_commandMap[_command]->Help();
}

/////////////////////////////////////////////////
Command::Command(const std::string &_name, const std::string &_brief)
  : name(_name), brief(_brief), visibleOptions("Options")
{
}

/////////////////////////////////////////////////
void Command::Help()
{
  std::cout << " gzmodify " << this->name << " [options]\n\n";
  this->HelpDetailed();
  std::cout << this->visibleOptions << "\n";
}

/////////////////////////////////////////////////
std::string Command::GetBrief() const
{
  return this->brief;
}

/////////////////////////////////////////////////
bool Command::TransportInit()
{
  if (!transport::init())
    return false;

  transport::run();

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
  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command");

  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions).add(this->visibleOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1);

  try
  {
    po::store(
        po::command_line_parser(_argc, _argv).options(allOptions).positional(
          positional).run(), this->vm);
    po::notify(this->vm);
  }
  catch(boost::exception &_e)
  {
    std::cout << "Invalid arguments\n";
    return false;
  }

  if (!this->TransportInit())
    return false;

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
    ("pause,p", po::value<bool>(), "Pause/unpause simulation.")
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
  std::cout << 
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
  if (this->vm.count("pause"))
    msg.set_pause(this->vm["pause"].as<bool>());
  if (this->vm.count("step"))
    msg.set_step(true);
  if (this->vm.count("multi-step"))
    msg.set_multi_step(this->vm["multi-step"].as<uint32_t>());
  if (this->vm.count("reset-all"))
    msg.mutable_reset()->set_all(true);
  if (this->vm.count("reset-time"))
    msg.mutable_reset()->set_time_only(true);
  if (this->vm.count("reset-models"))
    msg.mutable_reset()->set_model_only(true);

  pub->Publish(msg, true);

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
     "Gravity vector. Comma separated 3-tuple")
    ("dt,d", po::value<double>(), "Step size")
    ("iters,i", po::value<double>(), "Iterations")
    ("update-rate,u", po::value<double>(), "Update rate");
}

/////////////////////////////////////////////////
void PhysicsCommand::HelpDetailed()
{
  std::cout <<
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

  if (this->vm.count("dt"))
    msg.set_dt(this->vm["dt"].as<double>());

  if (this->vm.count("iters"))
    msg.set_iters(this->vm["iters"].as<double>());

  if (this->vm.count("update-rate"))
    msg.set_update_rate(this->vm["update-rate"].as<double>());

  if (this->vm.count("gravity"))
  {
    std::vector<std::string> values;
    boost::split(values, this->vm["gravity"].as<std::string>(),
        boost::is_any_of(","));

    msg.mutable_gravity()->set_x(boost::lexical_cast<double>(values[0]));
    msg.mutable_gravity()->set_y(boost::lexical_cast<double>(values[1]));
    msg.mutable_gravity()->set_z(boost::lexical_cast<double>(values[2]));
  }

  pub->Publish(msg, true);

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
    ("spawn-sdf,f", po::value<std::string>(), "Spawn model from SDF file.")
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
  std::cout <<
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
    std::cout << "A model name is required using the "
      << "(-m <model_name> command line argument)\n";
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
  else if (this->vm.count("spawn-sdf"))
  {
    std::string filename = this->vm["spawn-sdf"].as<std::string>();

    std::ifstream ifs(filename.c_str());
    if (!ifs)
    {
      std::cout << "Error: Unable to open file[" << filename << "]\n";
      return false;
    }

    boost::shared_ptr<sdf::SDF> sdf(new sdf::SDF());
    if (!sdf::init(sdf))
    {
      std::cout << "ERROR: SDF parsing the xml failed" << std::endl;
      return false;
    }

    if (!sdf::readFile(filename, sdf))
    {
      std::cout << "Error: SDF parsing the xml failed\n";
      return false;
    }

    sdf::ElementPtr modelElem = sdf->root->GetElement("model");

    if (!modelElem)
    {
      gzerr << "Unable to find <model> element.\n";
      return false;
    }

    // Get/Set the model name
    if (modelName.empty())
      modelName = modelElem->GetValueString("name");
    else
      modelElem->GetAttribute("name")->SetFromString(modelName);

    transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
    pub->WaitForConnection();

    msgs::Factory msg;
    msg.set_sdf(sdf->ToString());
    msgs::Set(msg.mutable_pose(), pose);
    pub->Publish(msg, true);
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
JointCommand::JointCommand()
  : Command("joint", "Modify properties of a joint")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("model-name,m", po::value<std::string>(), "Model name.")
    ("joint-name,j", po::value<std::string>(), "Model name.")
    ("delete,d", "Delete a model.")
    ("force,f", po::value<double>(), "Force to apply to a joint.")
    ("position-t", po::value<double>(), "Target angle.")
    ("position-p", po::value<double>(), "Position proportional gain.")
    ("position-i", po::value<double>(), "Position integral gain.")
    ("position-d", po::value<double>(), "Position differential gain.")
    ("velocity-t", po::value<double>(), "Target speed.")
    ("velocity-p", po::value<double>(), "Velocity proportional gain.")
    ("velocity-i", po::value<double>(), "Velocity integral gain.")
    ("velocity-d", po::value<double>(), "Velocity differential gain.");
}

/////////////////////////////////////////////////
void JointCommand::HelpDetailed()
{
  std::cout <<
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
    std::cout << "A model name is required using the "
      << "(-m <model_name> command line argument)\n";
    return false;
  }

  if (this->vm.count("joint-name"))
    jointName = this->vm["joint-name"].as<std::string>();
  else
  {
    std::cout << "A joint name is required using the "
      << "(-j <joint_name> command line argument)\n";
    return false;
  }

  msgs::JointCmd msg;
  msg.set_name(modelName + "::" + jointName);

  if (this->vm.count("force"))
    msg.set_force(this->vm["force"].as<double>());

  if (this->vm.count("position-t"))
  {
    msg.mutable_position()->set_target(
        this->vm["position-t"].as<double>());

    if (this->vm.count("position-p"))
      msg.mutable_position()->set_p_gain(
          this->vm["position-p"].as<double>());

    if (this->vm.count("position-i"))
      msg.mutable_position()->set_i_gain(
          this->vm["position-i"].as<double>());

    if (this->vm.count("position-d"))
      msg.mutable_position()->set_d_gain(
          this->vm["position-d"].as<double>());
  }

  if (this->vm.count("velocity-t"))
  {
    msg.mutable_velocity()->set_target(
        this->vm["velocity-t"].as<double>());

    if (this->vm.count("velocity-p"))
      msg.mutable_velocity()->set_p_gain(
          this->vm["velocity-p"].as<double>());

    if (this->vm.count("velocity-i"))
      msg.mutable_velocity()->set_i_gain(
          this->vm["velocity-i"].as<double>());

    if (this->vm.count("velocity-d"))
      msg.mutable_velocity()->set_d_gain(
          this->vm["velocity-d"].as<double>());
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
    ("camera-name,c", po::value<std::string>(), "Camera name.")
    ("follow,f", po::value<std::string>(), "Model to follow.");
}

/////////////////////////////////////////////////
void CameraCommand::HelpDetailed()
{
  std::cout <<
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

  if (this->vm.count("camera-name"))
    cameraName = this->vm["camera-name"].as<std::string>();
  else
  {
    std::cout << "A camera name is required using the "
      << "(-c <camera_name> command line argument)\n";
    return false;
  }

  msgs::CameraCmd msg;
  msg.set_name(cameraName);
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
int main(int argc, char **argv)
{
  gazebo::common::Console::Instance()->SetQuiet(true);

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
    std::cout << "Invalid arguments\n";
    return -1;
  }

  std::string command;

  // Get the command name
  command = vm.count("command") ? vm["command"].as<std::string>() : "";

  g_commandMap["camera"] = new CameraCommand();
  g_commandMap["joint"] = new JointCommand();
  g_commandMap["model"] = new ModelCommand();
  g_commandMap["world"] = new WorldCommand();
  g_commandMap["physics"] = new PhysicsCommand();

  // Output help when appropriate
  if (command.empty() || command == "help" || vm.count("help"))
  {
    std::string option;
    if (vm.count("pass") && !vm["pass"].as<std::vector<std::string> >().empty())
      option = vm["pass"].as<std::vector<std::string> >()[0];

    Help(option);
  }
  else
  {
    std::cout << "Run\n";
    g_commandMap[command]->Run(argc, argv);
  }

  return 0;
}
