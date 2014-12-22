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

event::EventT<void (std::string, std::string)> Events::createEntity;
event::EventT<void (bool)> Events::moveMode;
event::EventT<void (std::string)> Events::manipMode;
event::EventT<void (std::string, std::string, std::string, bool)>
    Events::alignMode;
event::EventT<void (bool)> Events::fullScreen;
event::EventT<void ()> Events::fps;
event::EventT<void ()> Events::orbit;
event::EventT<void (std::string)> Events::keyPress;
event::EventT<void (const msgs::Model &)> Events::modelUpdate;
event::EventT<void (const msgs::Light &)> Events::lightUpdate;
event::EventT<void (int)> Events::inputStepSize;
event::EventT<void (const std::string &)> Events::follow;
event::EventT<void (bool)> Events::leftPaneVisibility;
event::EventT<void ()> Events::mainWindowReady;
