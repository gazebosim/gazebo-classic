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
/* Desc: Generic Gazebo Device Inteface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#include "gazebo/transport/transport.hh"
#include "GazeboInterface.hh"
#include "GazeboDriver.hh"

std::string GazeboInterface::worldName = "default";

GazeboInterface::GazeboInterface(player_devaddr_t _addr, GazeboDriver *_driver,
                                 ConfigFile * /*cf*/, int /*section*/)
{
  this->device_addr = _addr;
  this->driver = _driver;
}

GazeboInterface::~GazeboInterface()
{
}
