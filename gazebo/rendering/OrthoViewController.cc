/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/rendering/OrthoViewControllerPrivate.hh"
#include "gazebo/rendering/OrthoViewController.hh"

#define TYPE_STRING "ortho"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
OrthoViewController::OrthoViewController(UserCameraPtr _camera)
  : OrbitViewController(_camera, "OrthoViewController"),
    dataPtr(new OrthoViewControllerPrivate)
{
  this->typeString = TYPE_STRING;
  this->init = false;
}

//////////////////////////////////////////////////
OrthoViewController::~OrthoViewController()
{
  delete this->dataPtr;
  this->dataPtr = NULL;

  this->refVisual.reset();
}

//////////////////////////////////////////////////
void OrthoViewController::Init()
{
  this->dataPtr->scale = 100;
  this->distance = 1000.0/this->dataPtr->scale;

  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();

  if (width > 0 && height > 0)
  {
    // set up the view projection
    this->Zoom(1.0);
  }

  OrbitViewController::Init();
}

//////////////////////////////////////////////////
void OrthoViewController::Init(const math::Vector3 &_focalPoint,
    double _yaw, double _pitch)
{
  this->dataPtr->scale = 100;

  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();

  if (width > 0 && height > 0)
  {
    // set up the view projection
    this->Zoom(1.0);
  }

  OrbitViewController::Init(_focalPoint, _yaw, _pitch);
}

//////////////////////////////////////////////////
std::string OrthoViewController::GetTypeString()
{
  return TYPE_STRING;
}

//////////////////////////////////////////////////
void OrthoViewController::HandleMouseEvent(const common::MouseEvent &_event)
{
  if (!this->enabled)
    return;

  math::Vector2i drag = _event.Pos() - _event.PrevPos();

  math::Vector3 directionVec(0, 0, 0);

  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();
  double orthoWidth = width/this->dataPtr->scale;
  double orthoHeight = height/this->dataPtr->scale;

  // If the event is the initial press of a mouse button, then update
  // the focal point and distance.
  if (_event.PressPos() == _event.Pos())
  {
    if (!this->camera->GetScene()->GetFirstContact(
         this->camera, _event.PressPos(), this->focalPoint))
    {
      ignition::math::Vector3d origin, dir;
      this->camera->CameraToViewportRay(
          _event.PressPos().X(), _event.PressPos().Y(), origin, dir);
      this->focalPoint = origin + dir * 10.0;
    }

    // pseudo distance
    this->distance = 1000.0/this->dataPtr->scale;

    this->yaw = this->camera->WorldRotation().Euler().Z();
    this->pitch = this->camera->WorldRotation().Euler().Y();
  }

  // Turn on the reference visual.
  this->refVisual->SetVisible(true);

  // Middle mouse button or Shift + Left button is used to Orbit.
  if (_event.Dragging() &&
      (_event.Buttons() & common::MouseEvent::MIDDLE ||
      (_event.Buttons() & common::MouseEvent::LEFT && _event.Shift())))
  {
    // Compute the delta yaw and pitch.
    double dy = this->NormalizeYaw(drag.x * _event.MoveScale() * -0.4);
    double dp = this->NormalizePitch(drag.y * _event.MoveScale() * 0.4);

    // Limit rotation to pitch only if the "y" key is pressed.
    if (!this->key.empty() && this->key == "y")
      dy = 0.0;
    // Limit rotation to yaw if the "z" key is pressed.
    else if (!this->key.empty() && this->key == "z")
      dp = 0.0;

    this->Orbit(dy, dp);
  }
  // The left mouse button is used to translate the camera.
  else if ((_event.Buttons() & common::MouseEvent::LEFT) && _event.Dragging())
  {
    math::Vector3 translation;

    double factor = 1.0;

    // The control key increases zoom speed by a factor of two.
    if (_event.Control())
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
  else if ((_event.Buttons() & common::MouseEvent::RIGHT) && _event.Dragging())
  {
    double amount = 1.0 + (drag.y / static_cast<float>(height));
    this->Zoom(amount, _event.PressPos());
  }
  // The scroll wheel controls zoom.
  else if (_event.Type() == common::MouseEvent::SCROLL)
  {
    if (!this->camera->GetScene()->GetFirstContact(
         this->camera, _event.Pos(), this->focalPoint))
    {
      ignition::math::Vector3d origin, dir;
      this->camera->CameraToViewportRay(
          _event.Pos().X(), _event.Pos().Y(), origin, dir);
      this->focalPoint = origin + dir * 10.0;
    }

    // pseudo distance
    this->distance = 1000.0/this->dataPtr->scale;

    double factor = 1.0;

    // The control key increases zoom speed by a factor of two.
    if (_event.Control())
      factor *= 2;

    // This assumes that _event.scroll.y is -1 or +1
    double zoomFactor = 10;
    double amount = 1.0 + _event.Scroll().Y() * factor * _event.MoveScale()
        * zoomFactor;
    this->Zoom(amount, _event.Pos());
  }
  else
    this->refVisual->SetVisible(false);
}

//////////////////////////////////////////////////
void OrthoViewController::Zoom(const float _amount,
                               const math::Vector2i &_screenPos)
{
  // Zoom to mouse cursor position
  // Three step process:
  // Translate mouse point to center of screen
  // Zoom by changing the orthographic window size
  // Translate back to mouse cursor position

  math::Vector3 translation;
  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();

  double orthoWidth = width / this->dataPtr->scale;
  double orthoHeight = height / this->dataPtr->scale;

  translation.Set(0.0,
      ((width/2.0 - _screenPos.x) / static_cast<float>(width))
      * orthoWidth,
      ((height/2.0 - _screenPos.y) / static_cast<float>(height))
      * orthoHeight);
  this->TranslateLocal(translation);

  this->dataPtr->scale /= _amount;

  // build custom projection matrix from custom near and far clipping planes,
  // had to set a negative near clippping plane to workaround a camera
  // culling issue in orthographic view.
  Ogre::Matrix4 proj;
  proj = this->BuildScaledOrthoMatrix(
      -width / this->dataPtr->scale / 2.0,
       width / this->dataPtr->scale / 2.0,
      -height / this->dataPtr->scale / 2.0,
       height / this->dataPtr->scale / 2.0,
      -500, 500);

  this->camera->GetOgreCamera()->setCustomProjectionMatrix(true, proj);

  double newOrthoWidth = width / this->dataPtr->scale;
  double newOrthoHeight = height / this->dataPtr->scale;

  translation.Set(0.0,
      ((_screenPos.x - width/2.0) / static_cast<float>(width))
      * newOrthoWidth,
      ((_screenPos.y - height/2.0) / static_cast<float>(height))
      * newOrthoHeight);

  this->TranslateLocal(translation);

  this->UpdateRefVisual();
}

//////////////////////////////////////////////////
void OrthoViewController::Resize(
    const unsigned int _width, const unsigned int _height)
{
  Ogre::Matrix4 proj = this->BuildScaledOrthoMatrix(
       _width / this->dataPtr->scale / -2.0,
       _width / this->dataPtr->scale / 2.0,
       _height / this->dataPtr->scale / -2.0,
       _height / this->dataPtr->scale / 2.0,
      -500, 500);

  this->camera->GetOgreCamera()->setCustomProjectionMatrix(true, proj);
}

//////////////////////////////////////////////////
Ogre::Matrix4 OrthoViewController::BuildScaledOrthoMatrix(
    const float _left, const float _right,
    const float _bottom, const float _top,
    const float _near, const float _far) const
{
  float invw = 1.0f / (_right - _left);
  float invh = 1.0f / (_top - _bottom);
  float invd = 1.0f / (_far - _near);

  Ogre::Matrix4 proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2.0f * invw;
  proj[0][3] = -(_right + _left) * invw;
  proj[1][1] = 2.0f * invh;
  proj[1][3] = -(_top + _bottom) * invh;
  proj[2][2] = -2.0f * invd;
  proj[2][3] = -(_far + _near) * invd;
  proj[3][3] = 1.0f;

  return proj;
}
