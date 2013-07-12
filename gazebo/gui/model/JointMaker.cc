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

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/model/JointMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointMaker::JointMaker()
{
}

/////////////////////////////////////////////////
JointMaker::~JointMaker()
{
  delete this->jointLine;
  this->hoverVis.reset();
}

/////////////////////////////////////////////////
bool JointMaker::OnMousePress(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  rendering::VisualPtr vis = camera->GetVisual(_event.pos);
//  if (vis)
//    vis = vis->GetRootVisual();

  if (this->hoverVis)
  {
    if (!this->selectedVis)
    {
      this->selectedVis = this->hoverVis;
      this->hoverVis.reset();

      rendering::VisualPtr joint(
          new rendering::Visual(this->selectedVis->GetName() +
          "_JOINTCREATOR_VISUAL_", this->selectedVis));
      this->jointVis = joint;
      this->jointVis->Load();
      this->parent = this->selectedVis;
      this->jointLine =
          jointVis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
      this->jointLine->AddPoint(math::Vector3(0, 0, 0));
      this->jointLine->AddPoint(math::Vector3(0, 0, 0.01));

      switch (this->jointType)
      {
        case JOINT_FIXED:
        {
          this->jointLine->setMaterial("Gazebo/Red");
          break;
        }
        case JOINT_HINGE:
        {
          this->jointLine->setMaterial("Gazebo/Orange");
          break;
        }
        case JOINT_HINGE2:
        {
          this->jointLine->setMaterial("Gazebo/Yellow");
          break;
        }
        case JOINT_SLIDER:
        {
          this->jointLine->setMaterial("Gazebo/Green");
          break;
        }
        case JOINT_SCREW:
        {
          this->jointLine->setMaterial("Gazebo/Black");
          break;
        }
        case JOINT_UNIVERSAL:
        {
          this->jointLine->setMaterial("Gazebo/Blue");
          break;
        }
        case JOINT_BALL:
        {
          this->jointLine->setMaterial("Gazebo/Purple");
          break;
        }
        default:
          break;

      }
    }
    else if (this->selectedVis != vis)
    {
      this->jointLine = NULL;
      if (this->hoverVis)
        this->hoverVis->SetHighlighted(false);
      if (this->selectedVis)
        this->selectedVis->SetHighlighted(false);
      // create the joint by publishing to server!?
      this->child = vis;
      this->selectedVis.reset();
      this->hoverVis.reset();
      this->jointVis.reset();
      this->parent.reset();
      this->CreateJoint(JointMaker::JOINT_NONE);
    }
  }

  return true;
}


/////////////////////////////////////////////////
void JointMaker::CreateJoint(JointMaker::JointType _type)
{
  this->jointType = _type;
  if (_type != JointMaker::JOINT_NONE)
  {
    // Add an event filter, which allows the TerrainEditor to capture
    // mouse events.
    MouseEventHandler::Instance()->AddPressFilter("model_joint",
        boost::bind(&JointMaker::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("model_joint",
        boost::bind(&JointMaker::OnMouseMove, this, _1));
  }
  else
  {
    // Remove the event filters.
    MouseEventHandler::Instance()->RemovePressFilter("model_joint");
    MouseEventHandler::Instance()->RemoveMoveFilter("model_joint");
  }
}


/////////////////////////////////////////////////
bool JointMaker::OnMouseMove(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  rendering::VisualPtr vis = camera->GetVisual(_event.pos);

//  if (vis)
//    vis = vis->GetRootVisual();

  // Highlight visual on hover
  if (vis)
  {
    if (this->hoverVis && this->hoverVis != this->selectedVis)
      this->hoverVis->SetHighlighted(false);

    // only highlight editor parts
    if (!gui::get_entity_id(vis->GetName()))
    {
      this->hoverVis = vis;
      if (!this->hoverVis->IsPlane())
      {
        this->hoverVis->SetHighlighted(true);
  //    event::Events::setSelectedEntity(vis->GetName(), "normal");
      }
    }
  }

  // Case when a parent part is already selected and currently
  // extending the joint line to a child part
  if (this->selectedVis && this->hoverVis && this->jointLine)
  {
    math::Vector3 parentPos;
    if (!this->hoverVis->IsPlane())
    {
      if (this->parent)
        parentPos = this->parent->GetWorldPose().pos;
      this->jointLine->SetPoint(1,
          this->hoverVis->GetWorldPose().pos - parentPos);
    }
    else
    {
      math::Vector3 pt;
      camera->GetWorldPointOnPlane(_event.pos.x, _event.pos.y,
          math::Plane(math::Vector3(0, 0, 1)), pt);

//      this->jointLine->SetChild(this->hoverVis, pt);
      if (this->parent)
        parentPos = this->parent->GetWorldPose().pos;
      this->jointLine->SetPoint(1,
          this->hoverVis->GetWorldPose().pos - parentPos + pt);
    }
  }


  return true;
}
