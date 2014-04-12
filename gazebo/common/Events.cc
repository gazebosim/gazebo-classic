/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "gazebo/common/Events.hh"

using namespace gazebo;
using namespace common;

ignition::common::EventT<void (bool)> Events::pause;
ignition::common::EventT<void ()> Events::step;
ignition::common::EventT<void ()> Events::stop;
ignition::common::EventT<void ()> Events::sigInt;
ignition::common::EventT<void (std::string)> Events::worldCreated;
ignition::common::EventT<void (std::string)> Events::entityCreated;
ignition::common::EventT<
void(std::string, std::string)> Events::setSelectedEntity;
ignition::common::EventT<void (std::string)> Events::addEntity;
ignition::common::EventT<void (std::string)> Events::deleteEntity;
ignition::common::EventT<
void(const gazebo::common::UpdateInfo &)> Events::worldUpdateBegin;
ignition::common::EventT<void ()> Events::worldUpdateEnd;
ignition::common::EventT<void ()> Events::preRender;
ignition::common::EventT<void ()> Events::render;
ignition::common::EventT<void ()> Events::postRender;
ignition::common::EventT<void (std::string)> Events::diagTimerStart;
ignition::common::EventT<void (std::string)> Events::diagTimerStop;

/////////////////////////////////////////////////
void Events::DisconnectWorldUpdateBegin(
    ignition::common::ConnectionPtr _subscriber)
{
  worldUpdateBegin.Disconnect(_subscriber);
}
