/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <fstream>
#include <string>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/CommonIface.hh>

using namespace gazebo;
namespace po = boost::program_options;

/////////////////////////////////////////////////
void help()
{
  std::cerr << "This tool for spawning or deleting models into or from a "
            << "running Gazebo simulation.\n\n"
            << "  gzfactory <spawn|delete> [options]\n"
            << "    spawn   : Spawn new model. Must specify a SDF model file.\n"
            << "    delete  : Delete existing model. Must specify model name.\n"
            << "\n\n";
}

/////////////////////////////////////////////////
void Spawn(po::variables_map &_vm)
{
  std::string filename, modelName;
  std::string worldName;

  if (!_vm.count("sdf"))
  {
    std::cerr << "Error: Missing filename.\n";
    return;
  }

  if (_vm.count("world-name"))
    worldName = _vm["world-name"].as<std::string>();

  if (_vm.count("model-name"))
    modelName = _vm["model-name"].as<std::string>();

  filename = _vm["sdf"].as<std::string>();

  std::ifstream ifs(filename.c_str());
  if (!ifs)
  {
    std::cerr << "Error: Unable to open file[" << filename << "]\n";
    return;
  }

  boost::shared_ptr<sdf::SDF> sdf(new sdf::SDF());
  if (!sdf::init(sdf))
  {
    std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
    return;
  }

  if (!sdf::readFile(filename, sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed\n";
    return;
  }

  sdf::ElementPtr modelElem = sdf->root->GetElement("model");

  if (!modelElem)
  {
    gzerr << "Unable to find <model> element.\n";
    return;
  }

  // Get/Set the model name
  if (modelName.empty())
    modelName = modelElem->Get<std::string>("name");
  else
    modelElem->GetAttribute("name")->SetFromString(modelName);

  math::Pose pose = modelElem->Get<math::Pose>("pose");
  math::Vector3 rpy = pose.rot.GetAsEuler();
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

  if (!transport::init())
    return;

  transport::run();

  transport::NodePtr node(new transport::Node());
  node->Init(worldName);

  std::cout << "Spawning " << modelName << " into "
            << node->GetTopicNamespace()  << " world.\n";

  transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
  pub->WaitForConnection();

  msgs::Factory msg;
  msg.set_sdf(sdf->ToString());
  msgs::Set(msg.mutable_pose(), pose);
  pub->Publish(msg, true);

  transport::fini();
}

/////////////////////////////////////////////////
void Delete(po::variables_map &vm)
{
  std::string modelName;
  // std::string worldName = "default";

  // if (vm.count("world-name"))
  //   worldName = vm["world-name"].as<std::string>();

  if (vm.count("model-name"))
    modelName = vm["model-name"].as<std::string>();
  else
  {
    std::cerr << "Error: No model name specified.\n";
    return;
  }

  msgs::Request *msg = msgs::CreateRequest("entity_delete", modelName);

  if (!transport::init())
    return;

  transport::run();

  transport::NodePtr node(new transport::Node());
  node->Init();

  transport::PublisherPtr pub = node->Advertise<msgs::Request>("~/request");
  pub->WaitForConnection();
  pub->Publish(*msg);
  delete msg;

  transport::fini();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  po::options_description v_desc("Allowed options");
  v_desc.add_options()
    ("help,h", "produce this help message")
    ("sdf,f", po::value<std::string>(), "SDF model file.")
    ("world-name,w", po::value<std::string>(), "Name of Gazebo world.")
    ("model-name,m", po::value<std::string>(), "Model name.")
    ("pose-x,x", po::value<double>(), "set model x position.")
    ("pose-y,y", po::value<double>(), "set model y position.")
    ("pose-z,z", po::value<double>(), "set model z positione.")
    ("pose-R,R", po::value<double>(), "set model roll orientation in radians.")
    ("pose-P,P", po::value<double>(), "set model pitch orientation in radians.")
    ("pose-Y,Y", po::value<double>(), "set model yaw orientation in radians.");

  po::options_description h_desc("Hidden options");
  h_desc.add_options()
    ("command", po::value<std::string>(), "<spawn|delete>");

  po::options_description desc("Allowed options");
  desc.add(v_desc).add(h_desc);

  po::positional_options_description p_desc;
  p_desc.add("command", 1);

  po::variables_map vm;
  try
  {
    po::store(po::command_line_parser(argc,
          argv).options(desc).positional(p_desc).run(), vm);
    po::notify(vm);
  } catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    return -1;
  }


  if (vm.count("help") || argc < 2)
  {
    help();
    std::cout << v_desc << "\n";
    return -1;
  }

  sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

  if (vm.count("command"))
  {
    std::string cmd = vm["command"].as<std::string>();
    if (cmd == "spawn")
      Spawn(vm);
    else if (cmd == "delete")
      Delete(vm);
    else
    {
      std::cerr << "Invalid command[" << cmd << "]. ";
      std::cerr << "Must use 'spawn' or 'delete'\n";
    }
  }
  else
    std::cerr << "Error: specify 'spawn' or 'delete'\n";

  return 0;
}
