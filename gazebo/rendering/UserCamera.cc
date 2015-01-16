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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/WindowManager.hh"
#include "gazebo/rendering/FPSViewController.hh"
#include "gazebo/rendering/OrbitViewController.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCameraPrivate.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/UserCamera.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
UserCamera::UserCamera(const std::string &_name, ScenePtr _scene,
    bool _stereoEnabled)
  : Camera(_name, _scene),
    dataPtr(new UserCameraPrivate)
{
  this->dataPtr->orbitViewController = NULL;
  this->dataPtr->fpsViewController = NULL;
  this->dataPtr->viewController = NULL;
  this->dataPtr->selectionBuffer = NULL;
  this->dataPtr->joyTwistControl = true;
  this->dataPtr->joystickButtonToggleLast = false;
  this->dataPtr->joyPoseControl = true;
  this->dataPtr->rightCamera = NULL;
  this->dataPtr->rightViewport = NULL;
  this->dataPtr->stereoEnabled = _stereoEnabled;

  // Set default UserCamera render rate to 120Hz when stereo rendering is
  // enabled. Otherwise use 60Hz.
  this->SetRenderRate(_stereoEnabled ? 120.0 : 60.0);

  this->SetUseSDFPose(false);
}

//////////////////////////////////////////////////
UserCamera::~UserCamera()
{
  delete this->dataPtr->orbitViewController;
  delete this->dataPtr->fpsViewController;

  this->connections.clear();

  delete this->dataPtr;
  this->dataPtr = NULL;
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
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->joySubTwist =
    this->dataPtr->node->Subscribe("~/user_camera/joy_twist",
    &UserCamera::OnJoyTwist, this);

  this->dataPtr->joySubPose =
    this->dataPtr->node->Subscribe("~/user_camera/joy_pose",
    &UserCamera::OnJoyPose, this);

  this->dataPtr->posePub =
    this->dataPtr->node->Advertise<msgs::Pose>("~/user_camera/pose", 1, 30.0);
}

//////////////////////////////////////////////////
void UserCamera::Init()
{
  this->dataPtr->orbitViewController = new OrbitViewController(
      boost::dynamic_pointer_cast<UserCamera>(shared_from_this()));
  this->dataPtr->fpsViewController = new FPSViewController(
      boost::dynamic_pointer_cast<UserCamera>(shared_from_this()));
  this->dataPtr->viewController = this->dataPtr->orbitViewController;

  Camera::Init();

  // Don't yaw along variable axis, causes leaning
  this->camera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
  this->camera->setDirection(1, 0, 0);
  this->camera->setAutoAspectRatio(false);

  // Right camera
  if (this->dataPtr->stereoEnabled)
  {
    this->dataPtr->rightCamera = this->scene->GetManager()->createCamera(
        "StereoUserRight");
    this->dataPtr->rightCamera->pitch(Ogre::Degree(90));

    // Don't yaw along variable axis, causes leaning
    this->dataPtr->rightCamera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    this->dataPtr->rightCamera->setDirection(1, 0, 0);

    this->dataPtr->rightCamera->setAutoAspectRatio(false);

    this->sceneNode->attachObject(this->dataPtr->rightCamera);
  }

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
  this->dataPtr->viewController->Init();
}

//////////////////////////////////////////////////
void UserCamera::Update()
{
  Camera::Update();

  if (this->dataPtr->viewController)
    this->dataPtr->viewController->Update();

  // publish camera pose
  this->dataPtr->posePub->Publish(msgs::Convert(this->GetWorldPose()));
}

//////////////////////////////////////////////////
void UserCamera::AnimationComplete()
{
  this->dataPtr->viewController->Init();
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
  if (this->dataPtr->selectionBuffer)
    this->dataPtr->selectionBuffer->Update();

  // DEBUG: this->dataPtr->selectionBuffer->ShowOverlay(true);

  // Don't update the camera if it's being animated.
  if (!this->animState)
    this->dataPtr->viewController->HandleMouseEvent(_evt);
}

/////////////////////////////////////////////////
void UserCamera::HandleKeyPressEvent(const std::string &_key)
{
  this->dataPtr->viewController->HandleKeyPressEvent(_key);
}

/////////////////////////////////////////////////
void UserCamera::HandleKeyReleaseEvent(const std::string &_key)
{
  this->dataPtr->viewController->HandleKeyReleaseEvent(_key);
}

/////////////////////////////////////////////////
bool UserCamera::IsCameraSetInWorldFile()
{
  // note: this function is used in rendering::Heightmap
  // to check if user specified custom pose for the camera.
  // Otherwise, camera is raised based on heightmap height
  // automatically.
  return this->dataPtr->isCameraSetInWorldFile;
}

//////////////////////////////////////////////////
void UserCamera::SetUseSDFPose(bool _value)
{
  // true if user specified custom pose for the camera.
  this->dataPtr->isCameraSetInWorldFile = _value;
}

//////////////////////////////////////////////////
void UserCamera::SetJoyTwistControl(bool _value)
{
  this->dataPtr->joyTwistControl = _value;
}

//////////////////////////////////////////////////
void UserCamera::SetJoyPoseControl(bool _value)
{
  this->dataPtr->joyPoseControl = _value;
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

    this->Yaw(yaw);
    this->Pitch(pitch);

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
  if (this->dataPtr->viewController->GetTypeString() == type)
    return;

  if (type == OrbitViewController::GetTypeString())
    this->dataPtr->viewController = this->dataPtr->orbitViewController;
  else if (type == FPSViewController::GetTypeString())
    this->dataPtr->viewController = this->dataPtr->fpsViewController;
  else
    gzthrow("Invalid view controller type: " + type);

  this->dataPtr->viewController->Init();
}

//////////////////////////////////////////////////
void UserCamera::SetViewController(const std::string &type,
                                    const math::Vector3 &_pos)
{
  if (this->dataPtr->viewController->GetTypeString() == type)
    return;

  if (type == OrbitViewController::GetTypeString())
    this->dataPtr->viewController = this->dataPtr->orbitViewController;
  else if (type == FPSViewController::GetTypeString())
    this->dataPtr->viewController = this->dataPtr->fpsViewController;
  else
    gzthrow("Invalid view controller type: " + type);

  this->dataPtr->viewController->Init(_pos);
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

    if (this->dataPtr->stereoEnabled)
    {
      this->dataPtr->rightCamera->setAspectRatio(ratio);
      this->dataPtr->rightCamera->setFOVy(Ogre::Radian(vfov));
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
unsigned int UserCamera::GetTriangleCount() const
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

  // this->dataPtr->orbitViewController->SetFocalPoint(
  //    _visual->GetWorldPose().pos);
  this->onAnimationComplete =
    boost::bind(&UserCamera::OnMoveToVisualComplete, this);
}

/////////////////////////////////////////////////
void UserCamera::OnMoveToVisualComplete()
{
  this->dataPtr->orbitViewController->SetDistance(
      this->GetWorldPose().pos.Distance(
      this->dataPtr->orbitViewController->GetFocalPoint()));
}

//////////////////////////////////////////////////
void UserCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  // Setup stereo rendering viewports
  if (this->dataPtr->stereoEnabled)
  {
    float focalLength = 1.0;

    // Defaulting to 0.03m stereo baseline.
    Ogre::Vector2 offset(0.03f, 0.0f);

    this->camera->setFocalLength(focalLength);
    this->camera->setFrustumOffset(offset);

    this->dataPtr->rightCamera->setFocalLength(focalLength);
    this->dataPtr->rightCamera->setFrustumOffset(-offset);

    this->dataPtr->rightViewport =
      this->renderTarget->addViewport(this->dataPtr->rightCamera, 1);
    this->dataPtr->rightViewport->setBackgroundColour(
        Conversions::Convert(this->scene->GetBackgroundColor()));

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 9
    this->viewport->setDrawBuffer(Ogre::CBT_BACK_LEFT);
    this->dataPtr->rightViewport->setDrawBuffer(Ogre::CBT_BACK_RIGHT);
#endif

    this->dataPtr->rightViewport->setVisibilityMask(GZ_VISIBILITY_ALL);
  }

  this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL);

  this->initialized = true;

  this->dataPtr->selectionBuffer = new SelectionBuffer(this->scopedUniqueName,
      this->scene->GetManager(), this->renderTarget);
}

//////////////////////////////////////////////////
void UserCamera::EnableViewController(bool _value) const
{
  this->dataPtr->viewController->SetEnabled(_value);
}

//////////////////////////////////////////////////
VisualPtr UserCamera::GetVisual(const math::Vector2i &_mousePos,
                                std::string &_mod)
{
  VisualPtr result;

  if (!this->dataPtr->selectionBuffer)
    return result;

  // Update the selection buffer
  this->dataPtr->selectionBuffer->Update();

  Ogre::Entity *entity =
    this->dataPtr->selectionBuffer->OnSelectionClick(_mousePos.x, _mousePos.y);

  _mod = "";
  if (entity)
  {
    // Make sure we set the _mod only if we have found a selection object
    if (entity->getName().substr(0, 15) == "__SELECTION_OBJ" &&
        !entity->getUserObjectBindings().getUserAny().isEmpty() &&
        entity->getUserObjectBindings().getUserAny().getType() ==
        typeid(std::string))
    {
      try
      {
        _mod = Ogre::any_cast<std::string>(
            entity->getUserObjectBindings().getUserAny());
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Ogre Error:" << e.getFullDescription() << "\n";
        gzthrow("Unable to get visual " + _mod);
      }
    }

    if (!entity->getUserObjectBindings().getUserAny().isEmpty())
    {
      try
      {
        result = this->scene->GetVisual(
            Ogre::any_cast<std::string>(
              entity->getUserObjectBindings().getUserAny()));
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
  this->dataPtr->orbitViewController->SetFocalPoint(_pt);
}

//////////////////////////////////////////////////
VisualPtr UserCamera::GetVisual(const math::Vector2i &_mousePos) const
{
  VisualPtr result;

  Ogre::Entity *entity =
    this->dataPtr->selectionBuffer->OnSelectionClick(_mousePos.x, _mousePos.y);

  if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
  {
    result = this->scene->GetVisual(
        Ogre::any_cast<std::string>(
          entity->getUserObjectBindings().getUserAny()));
  }

  return result;
}

//////////////////////////////////////////////////
std::string UserCamera::GetViewControllerTypeString()
{
  GZ_ASSERT(this->dataPtr->viewController, "ViewController != NULL");
  return this->dataPtr->viewController->GetTypeString();
}

//////////////////////////////////////////////////
void UserCamera::OnJoyTwist(ConstJoystickPtr &_msg)
{
  // Scaling factor applied to rotations.
  static math::Vector3 rpyFactor(0, 0.01, 0.05);

  // toggle using joystick to move camera
  if (this->dataPtr->joystickButtonToggleLast == false &&
      _msg->buttons().size() == 2 && _msg->buttons(0) == 1)
  {
    this->dataPtr->joyTwistControl =
      !this->dataPtr->joyTwistControl;

    this->dataPtr->joystickButtonToggleLast = true;

    if (this->dataPtr->joyTwistControl)
      gzmsg << "Joystick camera viewpoint control active.\n";
    else
      gzmsg << "Joystick camera viewpoint control deactivated.\n";
  }
  else if (_msg->buttons().size() == 2 && _msg->buttons(0) == 0)
  {
    // detect button release
    this->dataPtr->joystickButtonToggleLast = false;
  }

  // This function was establish when integrating the space navigator
  // joystick.
  if (this->dataPtr->joyTwistControl &&
      (_msg->has_translation() || _msg->has_rotation()))
  {
    math::Pose pose = this->GetWorldPose();

    // Get the joystick XYZ
    if (_msg->has_translation())
    {
      const double transRotRatio = 0.05;
      math::Vector3 trans = msgs::Convert(_msg->translation()) * transRotRatio;
      pose.pos = pose.rot.RotateVector(trans) + pose.pos;
    }

    // Get the jostick RPY. We are disabling rotation around x.
    if (_msg->has_rotation())
    {
      math::Vector3 rot = msgs::Convert(_msg->rotation()) * rpyFactor;
      pose.rot.SetFromEuler(pose.rot.GetAsEuler() + rot);
    }

    this->SetWorldPose(pose);
  }
}

//////////////////////////////////////////////////
void UserCamera::OnJoyPose(ConstPosePtr &_msg)
{
  if (!this->dataPtr->joyPoseControl)
    return;

  if (_msg->has_position() && _msg->has_orientation())
  {
    // Get the XYZ
    math::Pose pose(msgs::Convert(_msg->position()),
                    msgs::Convert(_msg->orientation()));
    this->SetWorldPose(pose);
  }
}

//////////////////////////////////////////////////
void UserCamera::SetClipDist(float _near, float _far)
{
  Camera::SetClipDist(_near, _far);

  // Update right camera, if it exists.
  if (this->dataPtr->stereoEnabled)
  {
    this->dataPtr->rightCamera->setNearClipDistance(
      this->camera->getNearClipDistance());
    this->dataPtr->rightCamera->setFarClipDistance(
      this->camera->getFarClipDistance());
    this->dataPtr->rightCamera->setRenderingDistance(
      this->camera->getRenderingDistance());
  }
}
