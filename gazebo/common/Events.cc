/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
using namespace event;

EventT<void (bool)> Events::pause;
EventT<void ()> Events::step;
EventT<void ()> Events::stop;
EventT<void ()> Events::sigInt;

EventT<void (std::string)> Events::worldCreated;
EventT<void (std::string)> Events::entityCreated;
EventT<void (std::string, std::string)> Events::setSelectedEntity;
EventT<void (std::string)> Events::addEntity;
EventT<void (std::string)> Events::deleteEntity;

EventT<void (const common::UpdateInfo &)> Events::worldUpdateBegin;
EventT<void (const common::UpdateInfo &)> Events::beforePhysicsUpdate;

EventT<void ()> Events::worldUpdateEnd;
EventT<void ()> Events::worldReset;
EventT<void ()> Events::timeReset;

EventT<void ()> Events::preRender;
EventT<void ()> Events::render;
EventT<void ()> Events::postRender;

EventT<void (std::string)> Events::diagTimerStart;
EventT<void (std::string)> Events::diagTimerStop;

EventT<void (std::string)> Events::removeSensor;

EventT<void (sdf::ElementPtr, const std::string &,
    const std::string &, const uint32_t)> Events::createSensor;

/////////////////////////////////////////////////
void Events::DisconnectWorldUpdateBegin(ConnectionPtr _subscriber)
{
  worldUpdateBegin.Disconnect(_subscriber->Id());
}
