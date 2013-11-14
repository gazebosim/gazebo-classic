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

#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "RosBag.hh"

using namespace gazebo;
namespace po = boost::program_options;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(RosBag)

/////////////////////////////////////////////
RosBag::~RosBag()
{
  rendering::fini();
}

/////////////////////////////////////////////
void RosBag::Load(int _argc, char **_argv)
{
  // Turn off sensors.
  gazebo::sensors::disable();

  po::options_description v_desc("Options");
  v_desc.add_options()
    ("bag", po::value<std::string>(), "Bag file to load");

  po::options_description desc("Options");
  desc.add(v_desc);

  po::variables_map vm;
  try
  {
    po::store(po::command_line_parser(_argc, _argv).options(
          desc).allow_unregistered().run(), vm);
    po::notify(vm);
  } catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    return;
  }

  std::string bagFile;

  if (vm.count("bag"))
    bagFile = vm["bag"].as<std::string>();
  else
    return;

  std::cout << "Bag file[" << bagFile << "]\n";

  this->bag.open(bagFile);

  std::string leftHandForceX = "/atlas/atlas_state";

  std::vector<std::string> topics;
  topics.push_back(leftHandForceX);

  this->view.addQuery(this->bag, rosbag::TopicQuery(topics));

  this->viewIter = this->view.begin();

  /*BOOST_FOREACH(rosbag::MessageInstance const m, this->view)
  {
    std::cout << m.getTime() << "\n";
  }*/
}

/////////////////////////////////////////////
void RosBag::Init()
{
  this->updateConn = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RosBag::Update, this, _1));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
}

/////////////////////////////////////////////
void RosBag::Update(const common::UpdateInfo &_info)
{

  rosbag::MessageInstance const m = *this->viewIter;

  std::cout << "SimTime[" << _info.simTime << "]\n";
  std::cout << "BagTime[" << m.getTime() << "]\n";
}
