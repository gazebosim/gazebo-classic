/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "common/Global.hh"
#include "common/MouseEvent.hh"
#include "math/Vector2i.hh"

#include "rendering/Visual.hh"
#include "rendering/UserCamera.hh"
#include "rendering/OrbitViewController.hh"

using namespace gazebo;
using namespace rendering;


static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = M_PI - 0.001;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
OrbitViewController::OrbitViewController(UserCamera *camera)
  : ViewController(camera), distance(5.0f)
{
  math::Vector3 rpy = this->camera->GetWorldPose().rot.GetAsEuler();

  this->yaw = rpy.z;
  this->pitch = rpy.y;

  this->focalPoint.Set(0,0,0);
  this->distance = this->camera->GetWorldPosition().Distance(this->focalPoint);
  this->refVisual = new Visual("OrbitViewController", this->camera->GetSceneNode());
  this->refVisual->Init();
  this->refVisual->AttachMesh("unit_sphere");
  this->refVisual->SetScale(math::Vector3(0.2,0.2,0.1));
  this->refVisual->SetCastShadows(false);
  this->refVisual->SetMaterial("Gazebo/YellowTransparent");
  this->refVisual->SetVisible(false);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OrbitViewController::~OrbitViewController()
{
  delete this->refVisual;
  this->refVisual = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Update
void OrbitViewController::Update()
{
  math::Vector3 pos;
  pos.x = this->distance * cos( this->yaw ) * sin( this->pitch );
  pos.z = this->distance * cos( this->pitch );
  pos.y = this->distance * sin( this->yaw ) * sin( this->pitch );

  pos += this->focalPoint;

  this->camera->SetWorldPosition(pos);

  math::Quaternion rot;
  rot.SetFromEuler( math::Vector3(0, M_PI*0.5 - this->pitch, this->yaw - M_PI) );
  this->camera->SetWorldRotation(rot);
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse event
void OrbitViewController::HandleMouseEvent(const common::MouseEvent &event)
{
  if (!this->camera->GetUserMovable())
    return;

  math::Vector2i drag = event.pos - event.prevPos;

  math::Vector3 directionVec(0,0,0);

  if (event.left == common::MouseEvent::DOWN)
  {
    this->refVisual->SetVisible(true);
    this->yaw += drag.x * event.moveScale * -0.1;
    this->pitch += drag.y * event.moveScale * -0.1;

    this->NormalizeYaw(this->yaw);
    this->NormalizePitch(this->pitch);
  }
  else if (event.middle == common::MouseEvent::SCROLL)
  {
    this->refVisual->SetVisible(true);
    distance +=  50.0 * event.scroll.y * event.moveScale;
  }
  else if (event.right == common::MouseEvent::DOWN)
  {
    this->refVisual->SetVisible(true);
    this->Translate(math::Vector3(0, drag.x * event.moveScale, drag.y * event.moveScale));
  }
  else if (event.middle == common::MouseEvent::DOWN)
  {
    this->refVisual->SetVisible(true);
    this->Translate(math::Vector3(drag.y * event.moveScale,0,0));
  }
  else
    this->refVisual->SetVisible(false);
}

////////////////////////////////////////////////////////////////////////////////
// Translate the focal point
void OrbitViewController::Translate(math::Vector3 vec)
{
  this->focalPoint += this->camera->GetWorldPose().rot * vec;
  this->refVisual->SetPosition(this->focalPoint);
}

////////////////////////////////////////////////////////////////////////////////
// Normalize yaw value
void OrbitViewController::NormalizeYaw(float &v)
{
  v = fmod(v, M_PI*2);
  if (v < 0.0f)
  {
    v = M_PI * 2 + v;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Normalize pitch value
void OrbitViewController::NormalizePitch(float &v)
{
  if (v < PITCH_LIMIT_LOW)
    v = PITCH_LIMIT_LOW;
  else if (v > PITCH_LIMIT_HIGH)
    v = PITCH_LIMIT_HIGH;
}
