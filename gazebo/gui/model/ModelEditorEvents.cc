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

#include "gazebo/gui/model/ModelEditorEvents.hh"

using namespace gazebo;
using namespace gui;

event::EventT<void ()> model::Events::finishModel;
event::EventT<bool ()> model::Events::saveAsModelEditor;
event::EventT<bool ()> model::Events::saveModelEditor;
event::EventT<void ()> model::Events::newModelEditor;
event::EventT<void ()> model::Events::exitModelEditor;
event::EventT<void ()> model::Events::modelChanged;
event::EventT<void (std::string)> model::Events::modelNameChanged;
event::EventT<void (bool, bool, const math::Pose &, const std::string &)>
    model::Events::modelPropertiesChanged;

event::EventT<void (std::string)> model::Events::saveModel;
event::EventT<void ()> model::Events::newModel;

event::EventT<void (std::string)> model::Events::linkInserted;
event::EventT<void (std::string)> model::Events::linkRemoved;
event::EventT<void (std::string)> model::Events::requestLinkRemoval;
event::EventT<void (std::string, std::string, std::string, std::string,
    std::string)> model::Events::jointInserted;
event::EventT<void (std::string)> model::Events::jointRemoved;
event::EventT<void (std::string, std::string, std::string, std::string,
    std::string)> model::Events::jointChanged;
event::EventT<void (std::string)> model::Events::openLinkInspector;
event::EventT<void (std::string)> model::Events::openJointInspector;
event::EventT<void (std::string)> model::Events::openModelPluginInspector;
event::EventT<void (std::string, std::string)> model::Events::jointNameChanged;
event::EventT<void (std::string)> model::Events::showLinkContextMenu;
event::EventT<void (std::string)> model::Events::showJointContextMenu;
event::EventT<void (std::string, bool)> model::Events::setSelectedLink;
event::EventT<void (std::string, bool)> model::Events::setSelectedJoint;
event::EventT<void (std::string)> model::Events::modelPluginInserted;
