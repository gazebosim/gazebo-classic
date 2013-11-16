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
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Vector3.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "AtlasState.h"
#include "RosBag.hh"

using namespace gazebo;
namespace po = boost::program_options;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(RosBag)

/////////////////////////////////////////////
RosBag::~RosBag()
{
  this->csvOut.close();
  rendering::fini();
}

/////////////////////////////////////////////
void RosBag::Load(int _argc, char **_argv)
{
  this->csvOut.open("/tmp/grasp.csv", std::ios::out);

  this->csvOut << "time, right force x, right force y, right force z"
    << ", right torque x, right torque y, right torque z"
    << ", left force x, left force y, left force z"
    << ", left torque x, left torque y, left torque z" << std::endl;

  this->filCoefA[0] = 1.0;
  this->filCoefA[1] = -0.924390491658207;

  this->filCoefB[0] = 0.037804754170897;
  this->filCoefB[1] = 0.037804754170897;

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


  ros::init(_argc, _argv,"gazebo", ros::init_options::NoSigintHandler);

  std::string bagFile;

  if (vm.count("bag"))
    bagFile = vm["bag"].as<std::string>();
  else
    return;

  // ros stuff
  this->bag.open(bagFile);

  std::string leftHandForceX = "/atlas/atlas_state";

  std::vector<std::string> topics;
  topics.push_back(leftHandForceX);

  this->view.addQuery(this->bag, rosbag::TopicQuery(topics));

  this->viewIter = this->view.begin();

  this->rosNode = new ros::NodeHandle("");
  this->rosNode->setParam("/use_sim_time", true);

  this->atlasStatePubUnLeft =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        "/atlas/atlas_state/left", 10);
  this->atlasStatePubUnRight =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        "/atlas/atlas_state/right", 10);

  this->atlasStatePubLeft =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        "/atlas/atlas_state_filtered/left", 10);

  this->timePub =
      this->rosNode->advertise<rosgraph_msgs::Clock>("/clock", 10);

  this->atlasStatePubRight =
      this->rosNode->advertise<geometry_msgs::WrenchStamped>(
        "/atlas/atlas_state_filtered/right", 10);
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
  bool updated = false;
  if (this->viewIter == this->view.end())
    return;

  ros::Time time = (*this->viewIter).getTime();

  while (time.toSec() < _info.simTime.Double() &&
      this->viewIter != this->view.end())
  {
    this->viewIter++;
    updated = true;
    if (this->viewIter != this->view.end())
      time = (*this->viewIter).getTime();
  }

  if (updated && this->viewIter != this->view.end())
  {
    rosbag::MessageInstance const m = *this->viewIter;
    atlas_msgs::AtlasState::ConstPtr stateMsg =
      m.instantiate<atlas_msgs::AtlasState>();

  std::cout << "SimTime[" << _info.simTime.sec << "." << _info.simTime.nsec
    << "] BagTime[" << time.sec << "." << time.nsec << "] MsgTime[" << stateMsg->header.stamp.sec << "." << stateMsg->header.stamp.nsec << "]\n";

    // Left hand filtering
    {
      // shift incoming buffer back
      this->leftWrenchIn[1] = this->leftWrenchIn[0];
      this->leftWrenchIn[0] = stateMsg->l_hand;

      // shift outgoing buffer back
      this->leftWrenchOut[1] = this->leftWrenchOut[0];

      // do filtering
      geometry_msgs::Wrench leftTmp;

      int j = 0;
      for (j=0; j < 2; ++j)
      {
        leftTmp.force.x += filCoefB[j] * leftWrenchIn[j].force.x;
        leftTmp.force.y += filCoefB[j] * leftWrenchIn[j].force.y;
        leftTmp.force.z += filCoefB[j] * leftWrenchIn[j].force.z;
        leftTmp.torque.x += filCoefB[j] * leftWrenchIn[j].torque.x;
        leftTmp.torque.y += filCoefB[j] * leftWrenchIn[j].torque.y;
        leftTmp.torque.z += filCoefB[j] * leftWrenchIn[j].torque.z;
      }
      j = 1;
      leftTmp.force.x -= filCoefA[j] * leftWrenchOut[j].force.x;
      leftTmp.force.y -= filCoefA[j] * leftWrenchOut[j].force.y;
      leftTmp.force.z -= filCoefA[j] * leftWrenchOut[j].force.z;
      leftTmp.torque.x -= filCoefA[j] * leftWrenchOut[j].torque.x;
      leftTmp.torque.y -= filCoefA[j] * leftWrenchOut[j].torque.y;
      leftTmp.torque.z -= filCoefA[j] * leftWrenchOut[j].torque.z;

      // Store filtered value
      leftWrenchOut[0] = leftTmp;
    }

    // Right hand filtering
    {
      // shift incoming buffer back
      this->rightWrenchIn[1] = this->rightWrenchIn[0];
      this->rightWrenchIn[0] = stateMsg->r_hand;

      // shift outgoing buffer back
      this->rightWrenchOut[1] = this->rightWrenchOut[0];

      // do filtering
      geometry_msgs::Wrench rightTmp;

      int j = 0;
      for (j = 0; j < 2; ++j)
      {
        rightTmp.force.x += filCoefB[j] * rightWrenchIn[j].force.x;
        rightTmp.force.y += filCoefB[j] * rightWrenchIn[j].force.y;
        rightTmp.force.z += filCoefB[j] * rightWrenchIn[j].force.z;
        rightTmp.torque.x += filCoefB[j] * rightWrenchIn[j].torque.x;
        rightTmp.torque.y += filCoefB[j] * rightWrenchIn[j].torque.y;
        rightTmp.torque.z += filCoefB[j] * rightWrenchIn[j].torque.z;
      }
      j = 1;
      rightTmp.force.x -= filCoefA[j] * rightWrenchOut[j].force.x;
      rightTmp.force.y -= filCoefA[j] * rightWrenchOut[j].force.y;
      rightTmp.force.z -= filCoefA[j] * rightWrenchOut[j].force.z;
      rightTmp.torque.x -= filCoefA[j] * rightWrenchOut[j].torque.x;
      rightTmp.torque.y -= filCoefA[j] * rightWrenchOut[j].torque.y;
      rightTmp.torque.z -= filCoefA[j] * rightWrenchOut[j].torque.z;

      // Store filtered value
      rightWrenchOut[0] = rightTmp;
    }

    geometry_msgs::WrenchStamped leftWrenchMsg, rightWrenchMsg;
    leftWrenchMsg.header = stateMsg->header;
    rightWrenchMsg.header = stateMsg->header;

    leftWrenchMsg.wrench = leftWrenchOut[0];
    rightWrenchMsg.wrench = rightWrenchOut[0];

    double secDouble = _info.simTime.Double();
    int sec = _info.simTime.sec;
    int min =  sec / 60;
    sec -= min*60;
    int ms = _info.simTime.nsec * 1e-6;

    this->csvOut << min << ":" << sec << "." << ms << ",";
    this->csvOut << rightWrenchOut[0].force.x << ","
      << rightWrenchOut[0].force.y << ","
      << rightWrenchOut[0].force.z << ","
      << rightWrenchOut[0].torque.x << ","
      << rightWrenchOut[0].torque.y << ","
      << rightWrenchOut[0].torque.z << ","
      << leftWrenchOut[0].force.x << ","
      << leftWrenchOut[0].force.y << ","
      << leftWrenchOut[0].force.z << ","
      << leftWrenchOut[0].torque.x << ","
      << leftWrenchOut[0].torque.y << ","
      << leftWrenchOut[0].torque.z << "\n";

    this->atlasStatePubLeft.publish(leftWrenchMsg);
    this->atlasStatePubRight.publish(rightWrenchMsg);

    /*leftWrenchMsg.wrench = stateMsg->l_hand;
    rightWrenchMsg.wrench = stateMsg->r_hand;
    this->atlasStatePubUnLeft.publish(leftWrenchMsg);
    this->atlasStatePubUnRight.publish(rightWrenchMsg);
    */


    rosgraph_msgs::Clock rosTime;
    rosTime.clock = m.getTime();
    this->timePub.publish(rosTime);
  }
}
