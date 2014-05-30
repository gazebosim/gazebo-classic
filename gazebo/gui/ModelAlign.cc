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

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"

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
  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelAlign::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  //this->dataPtr->mouseStart = _event.pressPos;
  this->SetMouseMoveVisual(rendering::VisualPtr());

  rendering::VisualPtr vis;
  rendering::VisualPtr mouseVis
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
void ModelAlign::SetMouseMoveVisual(rendering::VisualPtr _vis)
{
  this->dataPtr->mouseMoveVis = _vis;
}
