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

event::EventT<void (std::string)> editor::Events::createBuildingEditorItem;
event::EventT<void (std::string, std::string)>
    editor::Events::saveBuildingModel;
event::EventT<void ()> editor::Events::finishBuildingModel;
event::EventT<void ()> editor::Events::discardBuildingModel;
event::EventT<void (int)> editor::Events::changeBuildingLevel;
event::EventT<void ()> editor::Events::addBuildingLevel;
event::EventT<void (int)> editor::Events::deleteBuildingLevel;
event::EventT<void (int, std::string)> editor::Events::changeBuildingLevelName;
event::EventT<void (double)> editor::Events::changeBuildingEditorZoom;

event::EventT<void ()> editor::Events::saveBuildingEditor;
event::EventT<void ()> editor::Events::discardBuildingEditor;
event::EventT<void ()> editor::Events::doneBuildingEditor;
event::EventT<void ()> editor::Events::exitBuildingEditor;
