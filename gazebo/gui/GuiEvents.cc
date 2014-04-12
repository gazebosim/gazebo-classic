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

#include "gazebo/gui/GuiEvents.hh"

using namespace gazebo;
using namespace gui;

ignition::common::EventT<void (std::string, std::string)> Events::createEntity;
ignition::common::EventT<void (bool)> Events::moveMode;
ignition::common::EventT<void (std::string)> Events::manipMode;
ignition::common::EventT<void (bool)> Events::fullScreen;
ignition::common::EventT<void ()> Events::fps;
ignition::common::EventT<void ()> Events::orbit;
ignition::common::EventT<void (std::string)> Events::keyPress;
ignition::common::EventT<void (const msgs::Model &)> Events::modelUpdate;
ignition::common::EventT<void (int)> Events::inputStepSize;
ignition::common::EventT<void (const std::string &)> Events::follow;
