/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/Joystick.hh"
#include "gz_joy.hh"

using namespace gazebo;

/////////////////////////////////////////////////
JoyCommand::JoyCommand()
  : Command("joy", "Joystick")
{
  this->joy = new util::Joystick();
  this->joy->Init(0);

  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("run,r", "Run");
}

/////////////////////////////////////////////////
void JoyCommand::HelpDetailed()
{
  std::cerr <<
    "\tPrint topic information to standard out. If a name for the world, \n"
    "\toption -w, is not specified, the first world found on \n"
    "\tthe Gazebo master will be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool JoyCommand::RunImpl()
{
  std::string worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  this->node.reset(new transport::Node());
  this->node->Init(worldName);

  this->joyPub = this->node->Advertise<msgs::Joystick>("~/joystick");

  while (this->running)
  {
    msgs::Joysticks msg;
    if (this->joy->Poll(msg))
    {
      std::cout << msg.DebugString() << "\n";
      // this->joyPub->Publish(msg);
    }
  }

  return true;
}

/////////////////////////////////////////////////
bool JoyCommand::TransportRequired()
{
  return false;
}
