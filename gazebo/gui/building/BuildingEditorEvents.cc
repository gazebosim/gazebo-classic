/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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

event::EventT<void (bool)> editor::Events::toggleEditMode;
event::EventT<void (std::string)> editor::Events::createBuildingEditorItem;
event::EventT<void (QColor)> editor::Events::colorSelected;
event::EventT<void (QString)> editor::Events::textureSelected;
event::EventT<void (std::string, std::string)>
    editor::Events::saveBuildingModel;
event::EventT<void ()> editor::Events::finishBuildingModel;
event::EventT<void ()> editor::Events::newBuildingModel;
event::EventT<void (int)> editor::Events::changeBuildingLevel;
event::EventT<void ()> editor::Events::addBuildingLevel;
event::EventT<void ()> editor::Events::deleteBuildingLevel;
event::EventT<void ()> editor::Events::showFloorplan;
event::EventT<void ()> editor::Events::triggerShowFloorplan;
event::EventT<void ()> editor::Events::showElements;
event::EventT<void ()> editor::Events::triggerShowElements;
event::EventT<void (int, std::string)> editor::Events::updateLevelWidget;
event::EventT<void (double)> editor::Events::changeBuildingEditorZoom;
event::EventT<bool (std::string)> editor::Events::saveAsBuildingEditor;
event::EventT<bool (std::string)> editor::Events::saveBuildingEditor;
event::EventT<void ()> editor::Events::newBuildingEditor;
event::EventT<void ()> editor::Events::exitBuildingEditor;
event::EventT<void (std::string)> editor::Events::buildingNameChanged;
