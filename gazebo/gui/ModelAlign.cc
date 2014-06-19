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

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/RayQuery.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/ModelAlignPrivate.hh"
#include "gazebo/gui/ModelAlign.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelAlign::ModelAlign()
  : dataPtr(new ModelAlignPrivate)
{
  this->dataPtr->initialized = false;
}

/////////////////////////////////////////////////
ModelAlign::~ModelAlign()
{
  this->dataPtr->modelPub.reset();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ModelAlign::Init()
{
  if (this->dataPtr->initialized)
    return;

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (!cam)
    return;

  if (!cam->GetScene())
    return;

  this->dataPtr->userCamera = cam;
  this->dataPtr->scene =  cam->GetScene();

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->modelPub =
      this->dataPtr->node->Advertise<msgs::Model>("~/model/modify");

  this->dataPtr->connections.push_back(
      gui::Events::ConnectAlignMode(
        boost::bind(&ModelAlign::OnAlignMode, this, _1, _2)));

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelAlign::Reset()
{
  this->dataPtr->targetVis.reset();
  for (std::map<rendering::VisualPtr, math::Pose>::iterator it
      = this->dataPtr->selectedVisuals.begin();
      it != this->dataPtr->selectedVisuals.end(); ++it)
  {
    if (it->first)
      it->first->SetHighlighted(false);
  }
  this->dataPtr->selectedVisuals.clear();
}

/////////////////////////////////////////////////
void ModelAlign::OnAlignMode(std::string _axis, std::string _config)
{
  if (this->dataPtr->selectedVisuals.size() <= 1)
    return;

  math::Pose targetWorldPose = this->dataPtr->targetVis->GetWorldPose();
  math::Box targetBbox = this->dataPtr->targetVis->GetBoundingBox();
  math::Vector3 targetMin = targetWorldPose.rot * targetBbox.min
      + targetWorldPose.pos;
  math::Vector3 targetMax = targetWorldPose.rot * targetBbox.max
      + targetWorldPose.pos;

  for (std::map<rendering::VisualPtr, math::Pose>::iterator it
      = this->dataPtr->selectedVisuals.begin(),
      j = --this->dataPtr->selectedVisuals.end();
      it != j; ++it)
  {
    math::Pose worldPose = it->first->GetWorldPose();
    math::Box bbox = it->first->GetBoundingBox();
    math::Vector3 boxWorldMin = worldPose.rot * bbox.min + worldPose.pos;
    math::Vector3 boxWorldMax = worldPose.rot * bbox.max + worldPose.pos;

    math::Vector3 trans;
    if (_config == "min")
      trans = targetMin - boxWorldMin;
    else if (_config == "cener")
      trans = targetWorldPose.pos - it->first->GetWorldPose().pos;
    else if (_config == "max")
      trans = targetMax - boxWorldMax;

    if (_axis == "x")
    {
      trans.y = trans.z = 0;
      it->first->SetWorldPosition(it->first->GetWorldPose().pos + trans);
    }
    if (_axis == "y")
    {
      trans.x = trans.z = 0;
      it->first->SetWorldPosition(it->first->GetWorldPose().pos + trans);
    }
    if (_axis == "z")
    {
      trans.x = trans.y = 0;
      it->first->SetWorldPosition(it->first->GetWorldPose().pos + trans);
    }
  }

}

/////////////////////////////////////////////////
void ModelAlign::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  rendering::VisualPtr vis
      = this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.pos);

  if (vis && !vis->IsPlane() &&
      this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
  {
  }
  else
    this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void ModelAlign::OnMouseMoveEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelAlign::OnMouseReleaseEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.pos);
  if (vis && !vis->IsPlane() &&
      this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
  {
    rendering::VisualPtr rootVis = vis->GetRootVisual();

    if (this->dataPtr->selectedVisuals.find(rootVis) !=
        this->dataPtr->selectedVisuals.end())
    {
      rootVis->SetHighlighted(true);
      this->dataPtr->selectedVisuals[rootVis] = rootVis->GetWorldPose();
      this->dataPtr->targetVis = rootVis;
    }
  }
  else
    this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelAlign::OnKeyPressEvent(const common::KeyEvent &_event)
{
  this->dataPtr->keyEvent = _event;
}

//////////////////////////////////////////////////
void ModelAlign::OnKeyReleaseEvent(const common::KeyEvent &_event)
{
  this->dataPtr->keyEvent = _event;

  this->dataPtr->keyEvent.key = 0;
}

/////////////////////////////////////////////////
void ModelAlign::PublishVisualPose(rendering::VisualPtr _vis)
{
  if (!_vis)
    return;

  // Check to see if the visual is a model.
  if (gui::get_entity_id(_vis->GetName()))
  {
    msgs::Model msg;
    msg.set_id(gui::get_entity_id(_vis->GetName()));
    msg.set_name(_vis->GetName());

    msgs::Set(msg.mutable_pose(), _vis->GetWorldPose());
    this->dataPtr->modelPub->Publish(msg);
  }
}
