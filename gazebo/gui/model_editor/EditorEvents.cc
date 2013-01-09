/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/gui/model_editor/EditorEvents.hh"

using namespace gazebo;
using namespace gui;

event::EventT<void (std::string)> editor::Events::createEditorItem;
event::EventT<void (std::string, std::string)> editor::Events::saveModel;
event::EventT<void ()> editor::Events::finishModel;
event::EventT<void ()> editor::Events::discardModel;
event::EventT<void (int)> editor::Events::changeLevel;
event::EventT<void ()> editor::Events::addLevel;
event::EventT<void (int)> editor::Events::deleteLevel;
event::EventT<void (int, std::string)> editor::Events::changeLevelName;
event::EventT<void (double)> editor::Events::changeZoom;
