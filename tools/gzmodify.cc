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
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <string>

#include <gazebo/transport/transport.hh>

using namespace gazebo;
namespace po = boost::program_options;

/////////////////////////////////////////////////
void Help()
{
  std::cerr << "This tool modifies various aspects of a "
            << "running Gazebo simulation.\n\n"
            << "  gzmodify <move> [options]\n"
            << "    move    : Move a model.\n"
            << "    physics : Modify physics.\n"
            << "\n\n";
}

/////////////////////////////////////////////////
bool TransportInit()
{
  if (!transport::init())
    return false;

  transport::run();

  return true;
}

/////////////////////////////////////////////////
bool TransportFini()
{
  transport::fini();
  return true;
}

/////////////////////////////////////////////////
void Server(int _argc, char **_argv)
{
  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command");

  // Options that are visible to the user through help.
  po::options_description visibleOptions("Options");
  visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("gravity,g", po::value<std::string>(),
     "Gravity vector. Comma separated 3-tuple")
    ("dt,d", po::value<double>(), "Step size")
    ("iters,i", po::value<double>(), "Iterations")
    ("update-rate,u", po::value<double>(), "Update rate");

  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions).add(visibleOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1);

  std::string worldName;

  po::variables_map _vm;
  try
  {
    po::store(
        po::command_line_parser(_argc, _argv).options(allOptions).positional(
          positional).run(), _vm);
    po::notify(_vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n";
    return;
  }

  if (_vm.count("world-name"))
    worldName = _vm["world-name"].as<std::string>();

  if (!TransportInit())
    return;

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::PublisherPtr pub =
    node->Advertise<msgs::Physics>("~/physics");
  pub->WaitForConnection();

  msgs::Physics msg;
}

/////////////////////////////////////////////////
void Physics(int _argc, char **_argv)
{
  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command");

  // Options that are visible to the user through help.
  po::options_description visibleOptions("Options");
  visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("gravity,g", po::value<std::string>(),
     "Gravity vector. Comma separated 3-tuple")
    ("dt,d", po::value<double>(), "Step size")
    ("iters,i", po::value<double>(), "Iterations")
    ("update-rate,u", po::value<double>(), "Update rate");

  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions).add(visibleOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1);

  std::string worldName;

  po::variables_map _vm;
  try
  {
    po::store(
        po::command_line_parser(_argc, _argv).options(allOptions).positional(
          positional).run(), _vm);
    po::notify(_vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n";
    return;
  }

  if (_vm.count("world-name"))
    worldName = _vm["world-name"].as<std::string>();

  if (!TransportInit())
    return;

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::PublisherPtr pub =
    node->Advertise<msgs::Physics>("~/physics");
  pub->WaitForConnection();

  msgs::Physics msg;
  
  if (_vm.count("dt"))
    msg.set_dt(_vm["dt"].as<double>());

  if (_vm.count("iters"))
    msg.set_iters(_vm["iters"].as<double>());

  if (_vm.count("update-rate"))
    msg.set_update_rate(_vm["update-rate"].as<double>());

  if (_vm.count("gravity"))
  {
    std::vector<std::string> values;
    boost::split(values, _vm["gravity"].as<std::string>(),
        boost::is_any_of(","));
    std::cout << "Gv[" << _vm["gravity"].as<std::string>() << "\n";
    std::cout << "Size[" << values.size() << "\n";
    msg.mutable_gravity()->set_x(boost::lexical_cast<double>(values[0]));
    msg.mutable_gravity()->set_y(boost::lexical_cast<double>(values[1]));
    msg.mutable_gravity()->set_z(boost::lexical_cast<double>(values[2]));
  }

  pub->Publish(msg, true);

  TransportFini();
}

/////////////////////////////////////////////////
void Move(int argc, char **argv)
{
  std::string modelName, worldName;

  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command");

  // Options that are visible to the user through help.
  po::options_description visibleOptions("Options");
  visibleOptions.add_options()
    ("model-name,m", po::value<std::string>(), "Model name.")
    ("world-name,w", po::value<std::string>(), "World name.")
    ("pose-x,x", po::value<double>(), "x value")
    ("pose-y,y", po::value<double>(), "y value")
    ("pose-z,z", po::value<double>(), "z value")
    ("pose-R,R", po::value<double>(), "roll in radians.")
    ("pose-P,P", po::value<double>(), "pitch in radians.")
    ("pose-Y,Y", po::value<double>(), "yaw in radians.");

  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions).add(visibleOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1);

  po::variables_map _vm;
  try
  {
    po::store(
        po::command_line_parser(argc, argv).options(allOptions).positional(
          positional).run(), _vm);
    po::notify(_vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n";
    return;
  }

  if (_vm.count("world-name"))
    worldName = _vm["world-name"].as<std::string>();

  if (_vm.count("model-name"))
    modelName = _vm["model-name"].as<std::string>();
  else
  {
    std::cerr << "A model name is required using the "
      << "(-m <mode_name> command line argument)\n";
    return;
  }


  math::Pose pose;
  math::Vector3 rpy;

  if (_vm.count("pose-x"))
    pose.pos.x = _vm["pose-x"].as<double>();
  if (_vm.count("pose-y"))
    pose.pos.y = _vm["pose-y"].as<double>();
  if (_vm.count("pose-z"))
    pose.pos.z = _vm["pose-z"].as<double>();
  if (_vm.count("pose-R"))
    rpy.x = _vm["pose-R"].as<double>();
  if (_vm.count("pose-P"))
    rpy.y = _vm["pose-P"].as<double>();
  if (_vm.count("pose-Y"))
    rpy.z = _vm["pose-Y"].as<double>();
  pose.rot.SetFromEuler(rpy);

  if (!TransportInit())
    return;

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  transport::PublisherPtr pub =
    node->Advertise<msgs::Model>("~/model/modify");
  pub->WaitForConnection();

  msgs::Model msg;
  msg.set_name(modelName);
  msgs::Set(msg.mutable_pose(), pose);
  pub->Publish(msg, true);

  TransportFini();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
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

  std::string command;

  // Get the command name
  command = vm.count("command") ? vm["command"].as<std::string>() : "";

  // Output help when appropriate
  if (command.empty() || command == "help" || vm.count("help"))
  {
    Help();
    // std::cerr << visibleOptions << "\n";
    return 0;
  }

  if (command == "move")
    Move(argc, argv);
  else if (command == "physics")
    Physics(argc, argv);
  else if (command == "server")
    Server(argc, argv);

  return 0;
}
