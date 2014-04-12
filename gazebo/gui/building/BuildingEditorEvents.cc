/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include "gazebo/gui/building/BuildingEditorEvents.hh"

using namespace gazebo;
using namespace gui;

ignition::common::EventT<void (std::string)>
editor::Events::createBuildingEditorItem;
ignition::common::EventT<void (std::string, std::string)>
    editor::Events::saveBuildingModel;
ignition::common::EventT<void ()> editor::Events::finishBuildingModel;
ignition::common::EventT<void ()> editor::Events::discardBuildingModel;
ignition::common::EventT<void (int)> editor::Events::changeBuildingLevel;
ignition::common::EventT<void ()> editor::Events::addBuildingLevel;
ignition::common::EventT<void (int)> editor::Events::deleteBuildingLevel;
ignition::common::EventT<void (int, std::string)>
editor::Events::changeBuildingLevelName;
ignition::common::EventT<void (double)>
editor::Events::changeBuildingEditorZoom;

ignition::common::EventT<void ()> editor::Events::saveBuildingEditor;
ignition::common::EventT<void ()> editor::Events::discardBuildingEditor;
ignition::common::EventT<void ()> editor::Events::doneBuildingEditor;
ignition::common::EventT<void ()> editor::Events::exitBuildingEditor;
