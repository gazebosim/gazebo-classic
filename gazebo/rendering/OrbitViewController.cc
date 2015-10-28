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
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/OrbitViewController.hh"

#define TYPE_STRING "orbit"

using namespace gazebo;
using namespace rendering;

static const float PITCH_LIMIT_LOW = -M_PI*0.5 + 0.001;
static const float PITCH_LIMIT_HIGH = M_PI*0.5 - 0.001;

//////////////////////////////////////////////////
OrbitViewController::OrbitViewController(UserCameraPtr _camera,
    const std::string &_name)
  : ViewController(_camera), distance(5.0f)
{
  this->typeString = TYPE_STRING;
  this->init = false;

  // Create a visual that is used a reference point.
  this->refVisual.reset(new Visual(_name, this->camera->GetScene()));

  this->refVisual->Load();
  this->refVisual->AttachMesh("unit_sphere");
  this->refVisual->SetScale(math::Vector3(0.2, 0.2, 0.1));
  this->refVisual->SetCastShadows(false);
  this->refVisual->SetMaterial("Gazebo/YellowTransparent");
  this->refVisual->SetVisible(false);
  this->refVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

//////////////////////////////////////////////////
OrbitViewController::~OrbitViewController()
{
  this->refVisual.reset();
}

//////////////////////////////////////////////////
void OrbitViewController::Init(const math::Vector3 &_focalPoint,
    const double _yaw, const double _pitch)
{
  this->yaw = _yaw;
  this->pitch = _pitch;

  this->focalPoint = _focalPoint;
  this->distance = this->camera->GetWorldPosition().Distance(this->focalPoint);

  this->init = true;
}

//////////////////////////////////////////////////
void OrbitViewController::Init()
{
  double dist = -1;
  math::Vector3 fp;

  // Try to get a point on a plane to use as the reference point
  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();

  if (this->camera->GetWorldPointOnPlane(width/2.0, height/2.0,
        math::Plane(math::Vector3(0, 0, 1)), fp))
  {
    dist = this->camera->GetWorldPosition().Distance(fp);
  }

  // If the plane is too far away.
  if (dist < 0 || dist > 20 || math::isnan(dist))
  {
    // First, see if the camera is looking at the origin.
    math::Vector3 dir = this->camera->GetDirection();
    dir.Normalize();
    math::Vector3 origin(0, 0, 0);
    math::Vector3 cameraPos = this->camera->GetWorldPose().pos;
    double distOrigin = cameraPos.Distance(origin);

    dist = origin.GetDistToLine(cameraPos, cameraPos + dir * distOrigin);

    if (math::equal(dist, 0.0, 1e-3))
      dist = distOrigin;
    else
    {
      // If camera is not looking at the origin, see if the camera's
      // direction projected on the ground plane interescts the origin.
      // Otherwise, choose a default distance of 10m for the focal point
      cameraPos.z = 0;
      distOrigin = cameraPos.Distance(origin);
      dist = origin.GetDistToLine(cameraPos, cameraPos + dir * distOrigin);
      if (math::equal(dist, 0.0, 1e-3))
        dist = distOrigin;
      else
        dist = 10;

      cameraPos = this->camera->GetWorldPose().pos;
    }
    if (dist > 10)
      dist = 10.0;
    fp = cameraPos + dir * dist;
  }

  fp.Correct();
  this->Init(fp);
}

//////////////////////////////////////////////////
void OrbitViewController::Update()
{
}

//////////////////////////////////////////////////
void OrbitViewController::HandleKeyPressEvent(const std::string &_key)
{
  if (_key == "x" || _key == "y" || _key == "z")
    this->key = _key;
}

//////////////////////////////////////////////////
void OrbitViewController::HandleKeyReleaseEvent(const std::string &_key)
{
  if (_key == "x" || _key == "y" || _key == "z")
    this->key.clear();
}

//////////////////////////////////////////////////
void OrbitViewController::HandleMouseEvent(const common::MouseEvent &_event)
{
  if (!this->enabled)
    return;

  math::Vector2i drag = _event.Pos() - _event.PrevPos();

  math::Vector3 directionVec(0, 0, 0);

  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();

  // If the event is the initial press of a mouse button, then update
  // the focal point and distance.
  if (_event.PressPos() == _event.Pos())
  {
    if (!this->camera->GetScene()->GetFirstContact(
         this->camera, _event.PressPos(), this->focalPoint))
    {
      math::Vector3 origin, dir;
      this->camera->GetCameraToViewportRay(
          _event.PressPos().X(), _event.PressPos().Y(), origin, dir);
      this->focalPoint = origin + dir * 10.0;
    }

    this->distance = this->camera->GetWorldPose().pos.Distance(
        this->focalPoint);

    this->yaw = this->camera->GetWorldRotation().GetAsEuler().z;
    this->pitch = this->camera->GetWorldRotation().GetAsEuler().y;
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
    this->distance =
      this->camera->GetWorldPose().pos.Distance(this->focalPoint);


    double fovY = this->camera->GetVFOV().Radian();
    double fovX = 2.0f * atan(tan(fovY / 2.0f) *
        this->camera->GetAspectRatio());

    math::Vector3 translation;

    double factor = 2.0;

    // The control key increases zoom speed by a factor of two.
    if (_event.Control())
      factor *= 2.0;

    // If the "x", "y", or "z" key is pressed, then lock translation to the
    // indicated axis.
    if (!this->key.empty())
    {
      if (this->key == "x")
        translation.Set((drag.y / static_cast<float>(height)) *
                        this->distance * tan(fovY / 2.0) * factor, 0.0, 0.0);
      else if (this->key == "y")
        translation.Set(0.0, (drag.x / static_cast<float>(width)) *
                        this->distance * tan(fovX / 2.0) * factor, 0.0);
      else if (this->key == "z")
        translation.Set(0.0, 0.0, (drag.y / static_cast<float>(height)) *
                        this->distance * tan(fovY / 2.0) * factor);
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
          (drag.x / static_cast<float>(width)) *
          this->distance * tan(fovX / 2.0) * factor,
          (drag.y / static_cast<float>(height)) *
          this->distance * tan(fovY / 2.0) * factor);

      // Translate in the local coordinate frame
      this->TranslateLocal(translation);
    }
  }
  // The right mouse button is used to zoom the camera.
  else if ((_event.Buttons() & common::MouseEvent::RIGHT) && _event.Dragging())
  {
    double fovY = this->camera->GetVFOV().Radian();
    this->Zoom((-drag.y / static_cast<float>(height)) *
               this->distance * tan(fovY / 2.0) * 6.0);
  }
  // The scroll wheel controls zoom.
  else if (_event.Type() == common::MouseEvent::SCROLL)
  {
    if (!this->camera->GetScene()->GetFirstContact(
         this->camera, _event.Pos(), this->focalPoint))
    {
      math::Vector3 origin, dir;
      this->camera->GetCameraToViewportRay(
          _event.Pos().X(), _event.Pos().Y(), origin, dir);
      this->focalPoint = origin + dir * 10.0;
    }

    this->distance = this->camera->GetWorldPose().pos.Distance(
        this->focalPoint);

    int factor = 80;

    // The control key increases zoom speed by a factor of two.
    if (_event.Control())
      factor *= 2;

    // This assumes that _event.scroll.y is -1 or +1
    this->Zoom(-(_event.Scroll().Y() * factor) * _event.MoveScale() *
               (this->distance / 5.0));
  }
  else
    this->refVisual->SetVisible(false);
}

//////////////////////////////////////////////////
void OrbitViewController::TranslateLocal(const math::Vector3 &_vec)
{
  this->camera->SetWorldPosition(
      this->camera->GetWorldPose().pos +
      this->camera->GetWorldPose().rot * _vec);
  this->UpdateRefVisual();
}

//////////////////////////////////////////////////
void OrbitViewController::TranslateGlobal(const math::Vector3 &_vec)
{
  this->camera->SetWorldPosition(
      this->camera->GetWorldPose().pos + _vec);
  this->UpdateRefVisual();
}

//////////////////////////////////////////////////
void OrbitViewController::SetDistance(float _d)
{
  this->distance = _d;
}

//////////////////////////////////////////////////
void OrbitViewController::SetFocalPoint(const math::Vector3 &_fp)
{
  this->focalPoint = _fp;
  this->refVisual->SetPosition(this->focalPoint);
}

//////////////////////////////////////////////////
math::Vector3 OrbitViewController::GetFocalPoint() const
{
  return this->focalPoint;
}

//////////////////////////////////////////////////
double OrbitViewController::NormalizeYaw(double _v)
{
  _v = fmod(_v, M_PI*2);
  if (_v < 0.0f)
  {
    _v = M_PI * 2 + _v;
  }

  return _v;
}

//////////////////////////////////////////////////
double OrbitViewController::NormalizePitch(double _v)
{
  if (_v < PITCH_LIMIT_LOW)
    _v = PITCH_LIMIT_LOW;
  else if (_v > PITCH_LIMIT_HIGH)
    _v = PITCH_LIMIT_HIGH;

  return _v;
}

//////////////////////////////////////////////////
void OrbitViewController::Zoom(float _amount)
{
  this->distance -= _amount;

  math::Vector3 delta = this->camera->GetWorldPosition() - this->focalPoint;
  delta.Normalize();
  delta *= this->distance;
  this->camera->SetWorldPosition(this->focalPoint + delta);

  this->UpdateRefVisual();
}

//////////////////////////////////////////////////
std::string OrbitViewController::GetTypeString()
{
  return TYPE_STRING;
}

//////////////////////////////////////////////////
void OrbitViewController::UpdateRefVisual()
{
  // Update the pose of the reference visual
  this->refVisual->SetPosition(this->focalPoint);

  // Update the size of the referenve visual based on the distance to the
  // focal point.
  double scale = this->distance * atan(GZ_DTOR(1.0));
  this->refVisual->SetScale(math::Vector3(scale, scale, scale * 0.5));
}

/////////////////////////////////////////////////
void OrbitViewController::Orbit(double _dy, double _dp)
{
  Ogre::SceneNode *cameraNode = this->camera->GetSceneNode();
  Ogre::Node *parentNode = cameraNode->getParent();
  Ogre::Vector3 pos = cameraNode->_getDerivedPosition();

  // First detach the camera from it's parent. We need to do this in order
  // to attach the camera to the reference visual
  if (parentNode)
    parentNode->removeChild(cameraNode);

  // Add the camera node to to the reference visual, and update the
  // reference visual's position.
  this->refVisual->GetSceneNode()->addChild(this->camera->GetSceneNode());
  this->refVisual->SetPosition(this->focalPoint);

  // Move the camera to it's starting location. Now we can rotate the
  // reference visual, which in turns rotates the camera.
  cameraNode->_setDerivedPosition(pos);
  cameraNode->setOrientation(Ogre::Quaternion());

  // Rotate and update the reference visual.
  this->yaw = this->NormalizeYaw(this->yaw + _dy);
  this->pitch = this->NormalizePitch(this->pitch + _dp);
  this->refVisual->SetRotation(math::Quaternion(0, this->pitch, this->yaw));

  // Get the final position of the camera. Special case when the orbit view
  // camera has just been initialized.
  if (!this->init)
    pos = cameraNode->_getDerivedPosition();

  // Store the new location of the camera
  Ogre::Quaternion rot = cameraNode->_getDerivedOrientation();

  // Detach the camera from the reference visual.
  this->refVisual->GetSceneNode()->removeChild(cameraNode);

  // Reattach the camera to the reference visual.
  if (parentNode)
  {
    parentNode->addChild(cameraNode);
    cameraNode->_setDerivedPosition(pos);
    this->camera->SetWorldRotation(Conversions::Convert(rot));
  }

  this->init = false;
  this->UpdateRefVisual();
}

/////////////////////////////////////////////////
double OrbitViewController::Yaw() const
{
  return this->yaw;
}

/////////////////////////////////////////////////
double OrbitViewController::Pitch() const
{
  return this->pitch;
}
