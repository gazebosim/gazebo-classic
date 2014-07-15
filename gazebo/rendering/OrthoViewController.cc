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
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/OrbitViewController.hh"
#include "gazebo/rendering/OrthoViewController.hh"

#define TYPE_STRING "ortho"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
OrthoViewController::OrthoViewController(UserCameraPtr _camera)
  : OrbitViewController(_camera)
{
  this->typeString = TYPE_STRING;
  this->init = false;
}

//////////////////////////////////////////////////
OrthoViewController::~OrthoViewController()
{
  this->refVisual.reset();
}

//////////////////////////////////////////////////
void OrthoViewController::Init()
{
  // TODO change scale
  this->scale = 10;
  this->camera->SetProjectionType(Camera::ORTHOGRAPHIC);
  double ratio = this->camera->GetAspectRatio();
  this->camera->SetOrthoWindowSize(this->scale, this->scale/ratio);

  OrbitViewController::Init();
}

//////////////////////////////////////////////////
void OrthoViewController::HandleMouseEvent(const common::MouseEvent &_event)
{
  if (!this->enabled)
    return;

  math::Vector2i drag = _event.pos - _event.prevPos;

  math::Vector3 directionVec(0, 0, 0);

  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();
  double orthoWidth = this->camera->GetOrthoWindowWidth();
  double orthoHeight = this->camera->GetOrthoWindowHeight();

  // If the event is the initial press of a mouse button, then update
  // the focal point and distance.
  if (_event.pressPos == _event.pos)
  {
    if (!this->camera->GetScene()->GetFirstContact(
         this->camera, _event.pressPos, this->focalPoint))
    {
      math::Vector3 origin, dir;
      this->camera->GetCameraToViewportRay(
          _event.pressPos.x, _event.pressPos.y, origin, dir);
      this->focalPoint = origin + dir * 10.0;
    }

    // pseudo distance
    this->distance = this->scale;

    this->yaw = this->camera->GetWorldRotation().GetAsEuler().z;
    this->pitch = this->camera->GetWorldRotation().GetAsEuler().y;
  }

  // Turn on the reference visual.
  this->refVisual->SetVisible(true);

  // Middle mouse button or Shift + Left button is used to Orbit.
  if (_event.dragging &&
      (_event.buttons & common::MouseEvent::MIDDLE ||
      (_event.buttons & common::MouseEvent::LEFT && _event.shift)))
  {
    // Compute the delta yaw and pitch.
    double dy = this->NormalizeYaw(drag.x * _event.moveScale * -0.4);
    double dp = this->NormalizePitch(drag.y * _event.moveScale * 0.4);

    // Limit rotation to pitch only if the "y" key is pressed.
    if (!this->key.empty() && this->key == "y")
      dy = 0.0;
    // Limit rotation to yaw if the "z" key is pressed.
    else if (!this->key.empty() && this->key == "z")
      dp = 0.0;

    this->Orbit(dy, dp);
  }
  // The left mouse button is used to translate the camera.
  else if ((_event.buttons & common::MouseEvent::LEFT) && _event.dragging)
  {
    math::Vector3 translation;

    //double factor = 2.0;
    double factor = 1.0;

    // The control key increases zoom speed by a factor of two.
    if (_event.control)
      factor *= 2.0;

    // If the "x", "y", or "z" key is pressed, then lock translation to the
    // indicated axis.
    if (!this->key.empty())
    {
      if (this->key == "x")
        translation.Set((drag.y / static_cast<float>(height)) *
                        orthoHeight * factor, 0.0, 0.0);
      else if (this->key == "y")
        translation.Set(0.0, (drag.x / static_cast<float>(width)) *
                        orthoWidth * factor, 0.0);
      else if (this->key == "z")
        translation.Set(0.0, 0.0, (drag.y / static_cast<float>(height)) *
                        orthoHeight * factor);
      else
        gzerr << "Unable to handle key [" << this->key << "] in orbit view "
              << "controller.\n";

      // Translate in the global coordinate frame
      this->TranslateGlobal(translation);
    }
    else
    {
      // Translate in the "y" "z" plane.
      translation.Set(0.0,
          (drag.x / static_cast<float>(width)) * orthoWidth * factor,
          (drag.y / static_cast<float>(height)) * orthoHeight * factor);

      // Translate in the local coordinate frame
      this->TranslateLocal(translation);
    }
  }
  // The right mouse button is used to zoom the camera.
  else if ((_event.buttons & common::MouseEvent::RIGHT) && _event.dragging)
  {
    double amount = 1.0 + (drag.y / static_cast<float>(height));
    this->Zoom(amount, _event.pressPos);
  }
  // The scroll wheel controls zoom.
  else if (_event.type == common::MouseEvent::SCROLL)
  {
    if (!this->camera->GetScene()->GetFirstContact(
         this->camera, _event.pos, this->focalPoint))
    {
      math::Vector3 origin, dir;
      this->camera->GetCameraToViewportRay(
          _event.pos.x, _event.pos.y, origin, dir);
      this->focalPoint = origin + dir * 10.0;
    }

    // pseudo distance
    this->distance = this->scale;

    double factor = 1.0;

    // The control key increases zoom speed by a factor of two.
    if (_event.control)
      factor *= 2;

    // This assumes that _event.scroll.y is -1 or +1
    double zoomFactor = 10;
    double amount = 1.0 + _event.scroll.y * factor * _event.moveScale
        * zoomFactor;
    this->Zoom(amount, _event.pos);
  }
  else
    this->refVisual->SetVisible(false);
}

//////////////////////////////////////////////////
void OrthoViewController::Zoom(float _amount, math::Vector2i _screenPos)
{
  // Zoom to mouse cursor position
  // Three step process:
  // Translate mouse point to center of screen
  // Zoom by changing the orthographic window size
  // Translate back to mouse cursor position

  math::Vector3 translation;
  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();
  double orthoWidth = this->camera->GetOrthoWindowWidth();
  double orthoHeight = this->camera->GetOrthoWindowHeight();

  translation.Set(0.0,
      ((width/2.0 - _screenPos.x) / static_cast<float>(width))
      * orthoWidth,
      ((height/2.0 - _screenPos.y) / static_cast<float>(height))
      * orthoHeight);
  this->TranslateLocal(translation);

  this->scale *= _amount;

  double ratio = this->camera->GetAspectRatio();
  this->camera->SetOrthoWindowSize(this->scale, this->scale/ratio);

  double newOrthoWidth = this->camera->GetOrthoWindowWidth();
  double newOrthoHeight = this->camera->GetOrthoWindowHeight();

  translation.Set(0.0,
      ((_screenPos.x - width/2.0) / static_cast<float>(width))
      * newOrthoWidth,
      ((_screenPos.y - height/2.0) / static_cast<float>(height))
      * newOrthoHeight);
  this->TranslateLocal(translation);

  this->UpdateRefVisual();
}
/*
//////////////////////////////////////////////////
void OrthoViewController::UpdateRefVisual()
{
  // Update the pose of the reference visual
  this->refVisual->SetPosition(this->focalPoint);

  // Update the size of the referenve visual based on the distance to the
  // focal point.
  double scale = this->distance * atan(GZ_DTOR(1.0));
  this->refVisual->SetScale(math::Vector3(scale, scale, scale * 0.5));
}*/
