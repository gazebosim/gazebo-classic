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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/EntityMaker.hh"

using namespace gazebo;
using namespace gui;

bool EntityMaker::snapToGrid = false;

//////////////////////////////////////////////////
EntityMaker::EntityMaker()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
}

//////////////////////////////////////////////////
EntityMaker::~EntityMaker()
{
  this->node->Fini();
  this->node.reset();
}

/////////////////////////////////////////////////
void EntityMaker::Start()
{
}

/////////////////////////////////////////////////
void EntityMaker::Stop()
{
  gui::Events::moveMode(true);
}

//////////////////////////////////////////////////
void EntityMaker::SetSnapToGrid(bool _snap)
{
  snapToGrid = _snap;
}

//////////////////////////////////////////////////
void EntityMaker::OnMouseRelease(const common::MouseEvent &_event)
{
  // Place if not dragging, or if dragged for less than 50 pixels.
  // The 50 pixels is used to account for accidental mouse movement
  // when placing an object.
  if (_event.Button() == common::MouseEvent::LEFT &&
      (!_event.Dragging() || _event.PressPos().Distance(_event.Pos()) < 50))
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

//////////////////////////////////////////////////
void EntityMaker::OnMouseMove(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();

  ignition::math::Vector3d pos =
      (ModelManipulator::GetMousePositionOnPlane(camera, _event)).Ign();

  if (_event.Control() || this->snapToGrid)
  {
    pos = ModelManipulator::SnapPoint(math::Vector3(pos)).Ign();
  }
 pos.Z(this->EntityPosition().Z());

  this->SetEntityPosition(pos);
}

/////////////////////////////////////////////////
ignition::math::Vector3d EntityMaker::EntityPosition() const
{
  return ignition::math::Vector3d();
}

/////////////////////////////////////////////////
void EntityMaker::SetEntityPosition(const ignition::math::Vector3d &/*_pos*/)
{
}

