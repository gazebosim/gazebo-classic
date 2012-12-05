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
#include "common/MouseEvent.hh"
#include "math/Vector2i.hh"
#include "math/Angle.hh"

#include "rendering/Scene.hh"
#include "rendering/Visual.hh"
#include "rendering/UserCamera.hh"
#include "rendering/OrbitViewController.hh"

#define TYPE_STRING "orbit"
#define MIN_DISTANCE 0.01

using namespace gazebo;
using namespace rendering;


static const float PITCH_LIMIT_LOW = -M_PI*0.5 + 0.001;
static const float PITCH_LIMIT_HIGH = M_PI*0.5 - 0.001;

//////////////////////////////////////////////////
OrbitViewController::OrbitViewController(UserCameraPtr _camera)
  : ViewController(_camera), distance(5.0f)
{
  this->minDist = MIN_DISTANCE;
  this->maxDist = 0;
  this->typeString = TYPE_STRING;

  this->refVisual.reset(new Visual("OrbitViewController",
                        this->camera->GetScene()));

  this->refVisual->Init();
  this->refVisual->AttachMesh("unit_sphere");
  this->refVisual->SetScale(math::Vector3(0.2, 0.2, 0.1));
  this->refVisual->SetCastShadows(false);
  this->refVisual->SetMaterial("Gazebo/YellowTransparent");
  this->refVisual->SetVisible(false);
}

//////////////////////////////////////////////////
OrbitViewController::~OrbitViewController()
{
  this->refVisual.reset();
}

//////////////////////////////////////////////////
void OrbitViewController::Init(const math::Vector3 &_focalPoint)
{
  math::Vector3 rpy = this->camera->GetWorldPose().rot.GetAsEuler();
  this->yaw = rpy.z;
  this->pitch = rpy.y;

  this->focalPoint = _focalPoint;
  this->distance = this->camera->GetWorldPosition().Distance(this->focalPoint);
  if (this->distance <= 1.0)
    std::cout << "Distance[" << this->distance << "]\n";
}

//////////////////////////////////////////////////
void OrbitViewController::SetDistanceRange(double _minDist, double _maxDist)
{
  this->minDist = _minDist;
  this->maxDist = _maxDist;
  if (this->distance > this->maxDist)
    this->distance = this->maxDist;
  if (this->distance < this->minDist)
    this->distance = this->minDist;
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

  math::Vector2i drag = _event.pos - _event.prevPos;

  math::Vector3 directionVec(0, 0, 0);

  int width = this->camera->GetViewportWidth();
  int height = this->camera->GetViewportHeight();

  if (_event.buttons & common::MouseEvent::MIDDLE)
  {
    this->refVisual->SetVisible(true);

    if (_event.pressPos == _event.pos)
    {
      this->focalPoint = this->camera->GetScene()->GetFirstContact(this->camera,
          math::Vector2i(width*0.5, height *0.5));
      this->distance = this->camera->GetWorldPose().pos.Distance(
          this->focalPoint);
    }

    /// Lock rotation to an axis if the "y" or "z" key is pressed.
    if (!this->key.empty() && (this->key == "y" || this->key == "z"))
    {
      // Limit rotation about the "y" axis.
      if (this->key == "y")
      {
        this->pitch += drag.y * _event.moveScale * 0.2;
        this->NormalizePitch(this->pitch);
      }
      // Limit rotation about the "z" axis.
      else
      {
        this->yaw += drag.x * _event.moveScale * -0.2;
        this->NormalizeYaw(this->yaw);
      }
    }
    // Otherwise rotate about "y" and "z".
    else
    {
      this->yaw += drag.x * _event.moveScale * -0.2;
      this->pitch += drag.y * _event.moveScale * 0.2;
      this->NormalizeYaw(this->yaw);
      this->NormalizePitch(this->pitch);
    }
  }
  else if (_event.buttons & common::MouseEvent::LEFT)
  {
    this->refVisual->SetVisible(true);

    this->distance =
      this->camera->GetWorldPose().pos.Distance(this->focalPoint);

    double fovY = this->camera->GetVFOV().Radian();
    double fovX = 2.0f * atan(tan(fovY / 2.0f) *
        this->camera->GetAspectRatio());

    math::Vector3 translation;

    // If the "x", "y", or "z" key is pressed, then lock translation to the
    // indicated axis.
    if (!this->key.empty())
    {
      if (this->key == "x")
        translation.Set((drag.y / static_cast<float>(height)) *
                        this->distance * tan(fovY / 2.0) * 2.0, 0.0, 0.0);
      else if (this->key == "y")
        translation.Set(0.0, (drag.x / static_cast<float>(width)) *
                        this->distance * tan(fovX / 2.0) * 2.0, 0.0);
      else if (this->key == "z")
        translation.Set(0.0, 0.0, (drag.y / static_cast<float>(height)) *
                        this->distance * tan(fovY / 2.0) * 2.0);
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
          this->distance * tan(fovX / 2.0) * 2.0,
          (drag.y / static_cast<float>(height)) *
          this->distance * tan(fovY / 2.0) * 2.0);

      // Translate in the local coordinate frame
      this->TranslateLocal(translation);
    }
  }
  else if (_event.type == common::MouseEvent::SCROLL)
  {
    int factor = 40;

    if (!_event.alt)
    {
      if (this->posCache.x != _event.pos.x ||
          this->posCache.y != _event.pos.y )
      {
        this->worldFocal =
          this->camera->GetScene()->GetFirstContact(this->camera,
                                                    _event.pos);
        this->distance = this->camera->GetWorldPose().pos.Distance(
            this->focalPoint);
      }
      this->posCache = _event.pos;

      // This is not perfect, but it does a decent enough job.
      if (_event.scroll.y < 0)
        this->focalPoint += (this->worldFocal - this->focalPoint) * 0.04;
      else
        this->focalPoint += (this->focalPoint - this->worldFocal) * 0.04;
    }
    else
      factor = 80;

    // This assumes that _event.scroll.y is -1 or +1
    this->Zoom(-(_event.scroll.y * factor) * _event.moveScale *
               (this->distance / 10.0));
  }
  else
    this->refVisual->SetVisible(false);

  this->refVisual->SetPosition(this->focalPoint);
  this->UpdatePose();
}

//////////////////////////////////////////////////
void OrbitViewController::TranslateLocal(math::Vector3 vec)
{
  this->focalPoint += this->camera->GetWorldPose().rot * vec;
}

//////////////////////////////////////////////////
void OrbitViewController::TranslateGlobal(math::Vector3 vec)
{
  this->focalPoint += vec;
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
void OrbitViewController::SetYaw(double _yaw)
{
  this->yaw = _yaw;
}

//////////////////////////////////////////////////
void OrbitViewController::SetPitch(double _pitch)
{
  this->pitch = _pitch;
}

//////////////////////////////////////////////////
void OrbitViewController::NormalizeYaw(float &v)
{
  v = fmod(v, M_PI*2);
  if (v < 0.0f)
  {
    v = M_PI * 2 + v;
  }
}

//////////////////////////////////////////////////
void OrbitViewController::NormalizePitch(float &v)
{
  if (v < PITCH_LIMIT_LOW)
    v = PITCH_LIMIT_LOW;
  else if (v > PITCH_LIMIT_HIGH)
    v = PITCH_LIMIT_HIGH;
}

//////////////////////////////////////////////////
void OrbitViewController::Zoom(float _amount)
{
  this->distance -= _amount;
  if (this->distance <= this->minDist)
    this->distance = this->minDist;
  if (this->maxDist > 0 && this->distance >= this->maxDist)
    this->distance = this->maxDist;
}

//////////////////////////////////////////////////
std::string OrbitViewController::GetTypeString()
{
  return TYPE_STRING;
}


//////////////////////////////////////////////////
void OrbitViewController::UpdatePose()
{
  math::Vector3 pos;

  pos.x = this->distance * -cos(this->yaw) * cos(this->pitch);
  pos.y = this->distance * -sin(this->yaw) * cos(this->pitch);
  pos.z = this->distance * sin(this->pitch);

  pos += this->focalPoint;

  this->camera->SetWorldPosition(pos);

  math::Quaternion rot;
  math::Vector3 rpy(0, this->pitch, this->yaw);
  rot.SetFromEuler(rpy);
  this->camera->SetWorldRotation(rot);
}
