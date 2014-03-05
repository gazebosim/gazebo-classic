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
/* Desc: Camera for viewing the world
 * Author: Nate Koenig
 * Date: 19 Jun 2008
 */

#include <sstream>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/GUIOverlay.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/WindowManager.hh"
#include "gazebo/rendering/FPSViewController.hh"
#include "gazebo/rendering/OrbitViewController.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/UserCamera.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
UserCamera::UserCamera(const std::string &_name, ScenePtr _scene)
  : Camera(_name, _scene)
{
  this->gui = new GUIOverlay();

  this->orbitViewController = NULL;
  this->fpsViewController = NULL;
  this->viewController = NULL;

  this->selectionBuffer = NULL;

  // Set default UserCamera render rate to 30Hz
  this->SetRenderRate(30.0);
}

//////////////////////////////////////////////////
UserCamera::~UserCamera()
{
  delete this->orbitViewController;
  delete this->fpsViewController;

  delete this->gui;
  this->gui = NULL;

  this->connections.clear();
}

//////////////////////////////////////////////////
void UserCamera::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void UserCamera::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void UserCamera::Init()
{
  this->orbitViewController = new OrbitViewController(
      boost::dynamic_pointer_cast<UserCamera>(shared_from_this()));
  this->fpsViewController = new FPSViewController(
      boost::dynamic_pointer_cast<UserCamera>(shared_from_this()));
  this->viewController = this->orbitViewController;

  Camera::Init();

  // Don't yaw along variable axis, causes leaning
  this->camera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
  this->camera->setDirection(1, 0, 0);

  this->SetHFOV(GZ_DTOR(60));

  // Careful when setting this value.
  // A far clip that is too close will have bad side effects on the
  // lighting. When using deferred shading, the light's use geometry that
  // trigger shaders. If the far clip is too close, the light's geometry is
  // clipped and wholes appear in the lighting.
  switch (RenderEngine::Instance()->GetRenderPathType())
  {
    case RenderEngine::VERTEX:
      this->SetClipDist(0.1, 100);
      break;

    case RenderEngine::DEFERRED:
    case RenderEngine::FORWARD:
      this->SetClipDist(.1, 5000);
      break;

    default:
      this->SetClipDist(.1, 5000);
      break;
  }

  // Removing for now because the axis doesn't not move properly when the
  // window is resized
  /*
  this->axisNode =
    this->sceneNode->createChildSceneNode(this->name + "AxisNode");

  const Ogre::Vector3 *corners =
    this->camera->getWorldSpaceCorners();

  double width = corners[1].y - corners[0].y;

  this->axisNode->setPosition(corners[0].x + 0.01,
                              corners[0].y + 0.0005,
                              corners[0].z + 0.0005);
  axisNode->setScale(width * 0.05, width * 0.05, width * 0.05);
  axisNode->setInheritOrientation(false);

  Ogre::ManualObject *x =
    this->scene->GetManager()->createManualObject("MyXAxis");
  x->begin("Gazebo/Red", Ogre::RenderOperation::OT_LINE_LIST);
  x->position(0, 0, 0);
  x->position(1, 0, 0);
  x->end();
  x->setVisibilityFlags(GZ_VISIBILITY_GUI);

  Ogre::ManualObject *y =
    this->scene->GetManager()->createManualObject("MyYAxis");
  y->begin("Gazebo/Green", Ogre::RenderOperation::OT_LINE_LIST);
  y->position(0, 0, 0);
  y->position(0, 1, 0);
  y->end();
  y->setVisibilityFlags(GZ_VISIBILITY_GUI);

  Ogre::ManualObject *z =
    this->scene->GetManager()->createManualObject("MyZAxis");
  z->begin("Gazebo/Blue", Ogre::RenderOperation::OT_LINE_LIST);
  z->position(0, 0, 0);
  z->position(0, 0, 1);
  z->end();
  z->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->axisNode->attachObject(x);
  this->axisNode->attachObject(y);
  this->axisNode->attachObject(z);
  */
}

//////////////////////////////////////////////////
void UserCamera::SetWorldPose(const math::Pose &_pose)
{
  Camera::SetWorldPose(_pose);
  this->viewController->Init();
}

//////////////////////////////////////////////////
void UserCamera::Update()
{
  Camera::Update();

  if (this->gui)
    this->gui->Update();
}

//////////////////////////////////////////////////
void UserCamera::AnimationComplete()
{
  this->viewController->Init();
}

//////////////////////////////////////////////////
void UserCamera::PostRender()
{
  Camera::PostRender();
}

//////////////////////////////////////////////////
void UserCamera::Fini()
{
  Camera::Fini();
}

//////////////////////////////////////////////////
void UserCamera::HandleMouseEvent(const common::MouseEvent &_evt)
{
  if (!this->gui || !this->gui->HandleMouseEvent(_evt))
  {
    if (this->selectionBuffer)
      this->selectionBuffer->Update();

    // DEBUG: this->selectionBuffer->ShowOverlay(true);

    // Don't update the camera if it's being animated.
    if (!this->animState)
      this->viewController->HandleMouseEvent(_evt);
  }
}

/////////////////////////////////////////////////
void UserCamera::HandleKeyPressEvent(const std::string &_key)
{
  if (this->gui)
    this->gui->HandleKeyPressEvent(_key);
  this->viewController->HandleKeyPressEvent(_key);
}

/////////////////////////////////////////////////
void UserCamera::HandleKeyReleaseEvent(const std::string &_key)
{
  if (this->gui)
    this->gui->HandleKeyReleaseEvent(_key);
  this->viewController->HandleKeyReleaseEvent(_key);
}

/////////////////////////////////////////////////
bool UserCamera::AttachToVisualImpl(VisualPtr _visual, bool _inheritOrientation,
                                     double /*_minDist*/, double /*_maxDist*/)
{
  Camera::AttachToVisualImpl(_visual, _inheritOrientation);
  if (_visual)
  {
    math::Pose origPose = this->GetWorldPose();
    double yaw = _visual->GetWorldPose().rot.GetAsEuler().z;

    double zDiff = origPose.pos.z - _visual->GetWorldPose().pos.z;
    double pitch = 0;

    if (fabs(zDiff) > 1e-3)
    {
      double dist = _visual->GetWorldPose().pos.Distance(
          this->GetWorldPose().pos);
      pitch = acos(zDiff/dist);
    }

    this->RotateYaw(yaw);
    this->RotatePitch(pitch);

    math::Box bb = _visual->GetBoundingBox();
    math::Vector3 pos = bb.GetCenter();
    pos.z = bb.max.z;

    this->SetViewController(OrbitViewController::GetTypeString(), pos);
  }
  else
    this->SetViewController(FPSViewController::GetTypeString());

  return true;
}

//////////////////////////////////////////////////
bool UserCamera::TrackVisualImpl(VisualPtr _visual)
{
  Camera::TrackVisualImpl(_visual);
  /*if (_visual)
    this->SetViewController(OrbitViewController::GetTypeString());
  else
    this->SetViewController(FPSViewController::GetTypeString());
    */

  return true;
}

//////////////////////////////////////////////////
void UserCamera::SetViewController(const std::string &type)
{
  if (this->viewController->GetTypeString() == type)
    return;

  if (type == OrbitViewController::GetTypeString())
    this->viewController = this->orbitViewController;
  else if (type == FPSViewController::GetTypeString())
    this->viewController = this->fpsViewController;
  else
    gzthrow("Invalid view controller type: " + type);

  this->viewController->Init();
}

//////////////////////////////////////////////////
void UserCamera::SetViewController(const std::string &type,
                                    const math::Vector3 &_pos)
{
  if (this->viewController->GetTypeString() == type)
    return;

  if (type == OrbitViewController::GetTypeString())
    this->viewController = this->orbitViewController;
  else if (type == FPSViewController::GetTypeString())
    this->viewController = this->fpsViewController;
  else
    gzthrow("Invalid view controller type: " + type);

  this->viewController->Init(_pos);
}

//////////////////////////////////////////////////
unsigned int UserCamera::GetImageWidth() const
{
  return this->viewport->getActualWidth();
}

//////////////////////////////////////////////////
unsigned int UserCamera::GetImageHeight() const
{
  return this->viewport->getActualHeight();
}

//////////////////////////////////////////////////
void UserCamera::Resize(unsigned int /*_w*/, unsigned int /*_h*/)
{
  if (this->viewport)
  {
    this->viewport->setDimensions(0, 0, 1, 1);
    double ratio = static_cast<double>(this->viewport->getActualWidth()) /
                   static_cast<double>(this->viewport->getActualHeight());

    double hfov =
      this->sdf->Get<double>("horizontal_fov");
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));

    if (this->gui)
    {
      this->gui->Resize(this->viewport->getActualWidth(),
                        this->viewport->getActualHeight());
    }

    delete [] this->saveFrameBuffer;
    this->saveFrameBuffer = NULL;
  }
}

//////////////////////////////////////////////////
void UserCamera::SetViewportDimensions(float /*x_*/, float /*y_*/,
                                       float /*w_*/, float /*h_*/)
{
  // this->viewport->setDimensions(x, y, w, h);
}

//////////////////////////////////////////////////
float UserCamera::GetAvgFPS() const
{
  return RenderEngine::Instance()->GetWindowManager()->GetAvgFPS(
      this->windowId);
}

//////////////////////////////////////////////////
float UserCamera::GetTriangleCount() const
{
  return RenderEngine::Instance()->GetWindowManager()->GetTriangleCount(
      this->windowId);
}

//////////////////////////////////////////////////
void UserCamera::ToggleShowVisual()
{
  // this->visual->ToggleVisible();
}

//////////////////////////////////////////////////
void UserCamera::ShowVisual(bool /*_s*/)
{
  // this->visual->SetVisible(_s);
}

//////////////////////////////////////////////////
bool UserCamera::MoveToPosition(const math::Pose &_pose, double _time)
{
  return Camera::MoveToPosition(_pose, _time);
}

//////////////////////////////////////////////////
void UserCamera::MoveToVisual(const std::string &_name)
{
  VisualPtr visualPtr = this->scene->GetVisual(_name);
  if (visualPtr)
    this->MoveToVisual(visualPtr);
  else
    gzerr << "MoveTo Unknown visual[" << _name << "]\n";
}

//////////////////////////////////////////////////
void UserCamera::MoveToVisual(VisualPtr _visual)
{
  if (!_visual)
    return;

  if (this->scene->GetManager()->hasAnimation("cameratrack"))
  {
    this->scene->GetManager()->destroyAnimation("cameratrack");
  }

  math::Box box = _visual->GetBoundingBox();
  math::Vector3 size = box.GetSize();
  double maxSize = std::max(std::max(size.x, size.y), size.z);

  math::Vector3 start = this->GetWorldPose().pos;
  start.Correct();
  math::Vector3 end = box.GetCenter() + _visual->GetWorldPose().pos;
  end.Correct();
  math::Vector3 dir = end - start;
  dir.Correct();
  dir.Normalize();

  double dist = start.Distance(end) - maxSize;

  math::Vector3 mid = start + dir*(dist*.5);
  mid.z = box.GetCenter().z + box.GetSize().z + 2.0;

  dir = end - mid;
  dir.Correct();

  dist = mid.Distance(end) - maxSize;

  double yawAngle = atan2(dir.y, dir.x);
  double pitchAngle = atan2(-dir.z, sqrt(dir.x*dir.x + dir.y*dir.y));
  math::Quaternion pitchYawOnly(0, pitchAngle, yawAngle);
  Ogre::Quaternion pitchYawFinal(pitchYawOnly.w, pitchYawOnly.x,
    pitchYawOnly.y, pitchYawOnly.z);

  dir.Normalize();

  double scale = maxSize / tan((this->GetHFOV()/2.0).Radian());

  end = mid + dir*(dist - scale);

  // dist = start.Distance(end);
  // double vel = 5.0;
  double time = 0.5;  // dist / vel;

  Ogre::Animation *anim =
    this->scene->GetManager()->createAnimation("cameratrack", time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);


  Ogre::TransformKeyFrame *key;

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  /*key = strack->createNodeKeyFrame(time * 0.5);
  key->setTranslate(Ogre::Vector3(mid.x, mid.y, mid.z));
  key->setRotation(pitchYawFinal);
  */

  key = strack->createNodeKeyFrame(time);
  key->setTranslate(Ogre::Vector3(end.x, end.y, end.z));
  key->setRotation(pitchYawFinal);

  this->animState =
    this->scene->GetManager()->createAnimationState("cameratrack");

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();

  // this->orbitViewController->SetFocalPoint(_visual->GetWorldPose().pos);
  this->onAnimationComplete =
    boost::bind(&UserCamera::OnMoveToVisualComplete, this);
}

/////////////////////////////////////////////////
void UserCamera::OnMoveToVisualComplete()
{
  this->orbitViewController->SetDistance(this->GetWorldPose().pos.Distance(
        this->orbitViewController->GetFocalPoint()));
}

//////////////////////////////////////////////////
void UserCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL);

  if (this->gui)
    this->gui->Init(this->renderTarget);

  this->initialized = true;

  this->selectionBuffer = new SelectionBuffer(this->name,
      this->scene->GetManager(), this->renderTarget);
}

//////////////////////////////////////////////////
GUIOverlay *UserCamera::GetGUIOverlay()
{
  return this->gui;
}

//////////////////////////////////////////////////
void UserCamera::EnableViewController(bool _value) const
{
  this->viewController->SetEnabled(_value);
}

//////////////////////////////////////////////////
VisualPtr UserCamera::GetVisual(const math::Vector2i &_mousePos,
                                std::string &_mod)
{
  VisualPtr result;

  if (!this->selectionBuffer)
    return result;

  // Update the selection buffer
  this->selectionBuffer->Update();

  Ogre::Entity *entity =
    this->selectionBuffer->OnSelectionClick(_mousePos.x, _mousePos.y);

  _mod = "";
  if (entity)
  {
    // Make sure we set the _mod only if we have found a selection object
    if (entity->getName().substr(0, 15) == "__SELECTION_OBJ" &&
        !entity->getUserAny().isEmpty() &&
        entity->getUserAny().getType() == typeid(std::string))
    {
      try
      {
        _mod = Ogre::any_cast<std::string>(entity->getUserAny());
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
        gzthrow("Unable to get visual " + _mod);
      }
    }

    if (!entity->getUserAny().isEmpty())
    {
      try
      {
        result = this->scene->GetVisual(
            Ogre::any_cast<std::string>(entity->getUserAny()));
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
        gzthrow("Unable to get visual " + _mod);
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
void UserCamera::SetFocalPoint(const math::Vector3 &_pt)
{
  this->orbitViewController->SetFocalPoint(_pt);
}

//////////////////////////////////////////////////
VisualPtr UserCamera::GetVisual(const math::Vector2i &_mousePos) const
{
  VisualPtr result;

  Ogre::Entity *entity =
    this->selectionBuffer->OnSelectionClick(_mousePos.x, _mousePos.y);

  if (entity && !entity->getUserAny().isEmpty())
  {
    result = this->scene->GetVisual(
        Ogre::any_cast<std::string>(entity->getUserAny()));
  }

  return result;
}

//////////////////////////////////////////////////
std::string UserCamera::GetViewControllerTypeString()
{
  GZ_ASSERT(this->viewController, "ViewController != NULL");
  return this->viewController->GetTypeString();
}
