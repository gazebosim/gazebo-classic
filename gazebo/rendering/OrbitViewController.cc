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
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/math/Vector2i.hh"
#include "gazebo/math/Angle.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/OrbitViewController.hh"

#define TYPE_STRING "orbit"
// delete #define MIN_DISTANCE 0.01

using namespace gazebo;
using namespace rendering;


static const float PITCH_LIMIT_LOW = -M_PI*0.5 + 0.001;
static const float PITCH_LIMIT_HIGH = M_PI*0.5 - 0.001;

//////////////////////////////////////////////////
OrbitViewController::OrbitViewController(UserCameraPtr _camera)
  : ViewController(_camera), distance(5.0f)
{
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
  this->yaw = this->pitch = 0.0;
  // this->yaw = rpy.z;
  // this->pitch = rpy.y;

  this->dy = this->dp = 0.0;

  this->focalPoint = _focalPoint;
  this->distance = this->camera->GetWorldPosition().Distance(this->focalPoint);
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

  this->dp = 0;
  this->dy = 0;

  if (_event.pressPos == _event.pos)
  {
    math::Vector3 rpy = this->camera->GetWorldPose().rot.GetAsEuler();

    this->focalPoint = this->camera->GetScene()->GetFirstContact(
        this->camera, _event.pressPos);

    this->distance = this->camera->GetWorldPose().pos.Distance(
        this->focalPoint);
  }

  if (_event.buttons & common::MouseEvent::MIDDLE)
  {
    this->refVisual->SetVisible(true);

    /// Lock rotation to an axis if the "y" or "z" key is pressed.
    if (!this->key.empty() && (this->key == "y" || this->key == "z"))
    {
      // Limit rotation about the "y" axis.
      if (this->key == "y")
      {
        this->dp = drag.y * _event.moveScale * 0.1;
        this->NormalizePitch(this->dp);
        // Delete this->pitch += drag.y * _event.moveScale * 0.4;
        // Delete this->NormalizePitch(this->pitch);
      }
      // Limit rotation about the "z" axis.
      else
      {
        this->dy = drag.x * _event.moveScale * -0.1;
        this->NormalizeYaw(this->dy);

        // Delete this->yaw += drag.x * _event.moveScale * -0.4;
        // Delete this->NormalizeYaw(this->yaw);
      }
    }
    // Otherwise rotate about "y" and "z".
    else
    {
      this->dy = drag.x * _event.moveScale * -0.4;
      this->dp = drag.y * _event.moveScale * 0.4;
      this->NormalizeYaw(this->dy);
      this->NormalizePitch(this->dp);
    }

    this->Orbit();
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
    this->refVisual->SetVisible(true);

    this->focalPoint = this->camera->GetScene()->GetFirstContact(this->camera,
        _event.pos);

    this->distance = this->camera->GetWorldPose().pos.Distance(
        this->focalPoint);

    int factor = 80;
    if (_event.alt)
      factor *= 2;

    // This assumes that _event.scroll.y is -1 or +1
    this->Zoom(-(_event.scroll.y * factor) * _event.moveScale *
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

  std::cout << "SetFocalPoint[" << this->focalPoint << "]\n";
  this->refVisual->SetPosition(this->focalPoint);
}

//////////////////////////////////////////////////
math::Vector3 OrbitViewController::GetFocalPoint() const
{
  return this->focalPoint;
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
void OrbitViewController::Orbit()
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
  this->yaw += this->dy;
  this->pitch += this->dp;
  this->refVisual->SetRotation(math::Quaternion(0, this->pitch, this->yaw));

  // Get the final position of the camera
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
    cameraNode->_setDerivedOrientation(rot);
  }

  this->UpdateRefVisual();
}
