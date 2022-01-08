/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/bind/bind.hpp>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/WindowManager.hh"
#include "gazebo/rendering/FPSViewController.hh"
#include "gazebo/rendering/OrbitViewController.hh"
#include "gazebo/rendering/OrthoViewController.hh"
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
  this->dataPtr->orthoViewController = NULL;
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
  // Some padding is added for safety.
  this->SetRenderRate(_stereoEnabled ? 124.0 : 62.0);

  this->SetUseSDFPose(false);
}

//////////////////////////////////////////////////
UserCamera::~UserCamera()
{
  delete this->dataPtr->orbitViewController;
  delete this->dataPtr->orthoViewController;
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
  this->dataPtr->orthoViewController = new OrthoViewController(
      boost::dynamic_pointer_cast<UserCamera>(shared_from_this()));
  this->dataPtr->fpsViewController = new FPSViewController(
      boost::dynamic_pointer_cast<UserCamera>(shared_from_this()));
  this->dataPtr->viewController = this->dataPtr->orbitViewController;

  Camera::Init();

  // Don't yaw along variable axis, causes leaning
  this->SetFixedYawAxis(true, ignition::math::Vector3d::UnitZ);
  this->sceneNode->setDirection(1, 0, 0);
  this->camera->setAutoAspectRatio(false);

  // Right camera
  if (this->dataPtr->stereoEnabled)
  {
    this->dataPtr->rightCamera = this->scene->OgreSceneManager()->createCamera(
        "StereoUserRight");
    this->dataPtr->rightCamera->pitch(Ogre::Degree(90));

    // Don't yaw along variable axis, causes leaning
    this->dataPtr->rightCamera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    this->dataPtr->rightCamera->setDirection(1, 0, 0);

    this->dataPtr->rightCamera->setAutoAspectRatio(false);

    this->sceneNode->attachObject(this->dataPtr->rightCamera);
  }

  this->SetHFOV(static_cast<ignition::math::Angle>(IGN_DTOR(60)));

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
    this->scene->OgreSceneManager()->createManualObject("MyXAxis");
  x->begin("Gazebo/Red", Ogre::RenderOperation::OT_LINE_LIST);
  x->position(0, 0, 0);
  x->position(1, 0, 0);
  x->end();
  x->setVisibilityFlags(GZ_VISIBILITY_GUI);

  Ogre::ManualObject *y =
    this->scene->OgreSceneManager()->createManualObject("MyYAxis");
  y->begin("Gazebo/Green", Ogre::RenderOperation::OT_LINE_LIST);
  y->position(0, 0, 0);
  y->position(0, 1, 0);
  y->end();
  y->setVisibilityFlags(GZ_VISIBILITY_GUI);

  Ogre::ManualObject *z =
    this->scene->OgreSceneManager()->createManualObject("MyZAxis");
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
void UserCamera::SetInitialPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->initialPose = _pose;
  this->SetWorldPose(_pose);
}

//////////////////////////////////////////////////
ignition::math::Pose3d UserCamera::InitialPose() const
{
  return this->dataPtr->initialPose;
}

//////////////////////////////////////////////////
void UserCamera::SetWorldPose(const ignition::math::Pose3d &_pose)
{
  Camera::SetWorldPose(_pose);
  this->dataPtr->viewController->Init();
}

//////////////////////////////////////////////////
void UserCamera::Update()
{
  IGN_PROFILE("rendering::UserCamera::Update");
  Camera::Update();

  if (this->dataPtr->viewController)
    this->dataPtr->viewController->Update();

  // publish camera pose
  this->dataPtr->posePub->Publish(msgs::Convert(this->WorldPose()));
}

//////////////////////////////////////////////////
void UserCamera::AnimationComplete()
{
  this->dataPtr->viewController->Init();
}

//////////////////////////////////////////////////
void UserCamera::Render(const bool /*_force*/)
{
  IGN_PROFILE("rendering::UserCamera::Render");
  if (this->initialized)
  {
    this->newData = true;
    this->RenderImpl();
  }
}

//////////////////////////////////////////////////
void UserCamera::PostRender()
{
  IGN_PROFILE("rendering::UserCamera::PostRender");
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
bool UserCamera::AttachToVisualImpl(VisualPtr _visual,
    const bool _inheritOrientation,
    const double /*_minDist*/, const double /*_maxDist*/)
{
  Camera::AttachToVisualImpl(_visual, _inheritOrientation);
  if (_visual)
  {
    ignition::math::Pose3d origPose = this->WorldPose();
    ignition::math::Angle yaw = _visual->WorldPose().Rot().Euler().Z();

    double zDiff = origPose.Pos().Z() - _visual->WorldPose().Pos().Z();
    ignition::math::Angle pitch = 0;

    if (fabs(zDiff) > 1e-3)
    {
      double dist = _visual->WorldPose().Pos().Distance(
          this->WorldPose().Pos());
      pitch = acos(zDiff/dist);
    }

    this->Yaw(yaw);
    this->Pitch(pitch);

    auto bb = _visual->BoundingBox();
    auto pos = bb.Center();
    pos.Z(bb.Max().Z());

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
void UserCamera::SetViewController(const std::string &_type)
{
  if (_type.empty() ||
      this->dataPtr->viewController->GetTypeString() == _type)
  {
    return;
  }

  std::string vc = this->dataPtr->viewController->GetTypeString();

  if (_type == OrbitViewController::GetTypeString())
  {
    this->dataPtr->viewController = this->dataPtr->orbitViewController;
    this->dataPtr->viewController->Init();

    this->dataPtr->prevViewControllerName = vc;
  }
  else if (_type == OrthoViewController::GetTypeString())
  {
    this->dataPtr->viewController = this->dataPtr->orthoViewController;
    if (vc == "orbit")
    {
      this->dataPtr->viewController->Init(
          this->dataPtr->orbitViewController->FocalPoint(),
          this->dataPtr->orbitViewController->Yaw(),
          this->dataPtr->orbitViewController->Pitch());
    }
    else
      this->dataPtr->viewController->Init();

    this->dataPtr->prevViewControllerName = vc;
  }
  else if (_type == FPSViewController::GetTypeString())
  {
    this->dataPtr->viewController = this->dataPtr->fpsViewController;
    this->dataPtr->viewController->Init();

    this->dataPtr->prevViewControllerName = vc;
  }
  else
  {
    gzerr << "Invalid view controller type[" << _type << "]. "
      << "The view controller is not changed.\n";
  }
}

//////////////////////////////////////////////////
void UserCamera::SetViewController(const std::string &_type,
                                   const ignition::math::Vector3d &_pos)
{
  if (_type.empty() ||
      this->dataPtr->viewController->GetTypeString() == _type)
  {
    return;
  }

  std::string vc = this->dataPtr->viewController->GetTypeString();

  if (_type == OrbitViewController::GetTypeString())
    this->dataPtr->viewController = this->dataPtr->orbitViewController;
  else if (_type == OrthoViewController::GetTypeString())
    this->dataPtr->viewController = this->dataPtr->orthoViewController;
  else if (_type == FPSViewController::GetTypeString())
    this->dataPtr->viewController = this->dataPtr->fpsViewController;
  else
    gzthrow("Invalid view controller type: " + _type);

  this->dataPtr->prevViewControllerName = vc;

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
void UserCamera::Resize(unsigned int _w, unsigned int _h)
{
  this->UpdateFOV();
  this->dataPtr->viewController->Resize(_w, _h);

  // reload ogre compositors on window resize
  // otherwise some compositors can cause the client to crash
  Ogre::CompositorManager *compMgr =
      Ogre::CompositorManager::getSingletonPtr();
  if (compMgr->hasCompositorChain(this->viewport))
  {
    Ogre::CompositorChain *chain =
        compMgr->getCompositorChain(this->viewport);
    std::vector<std::pair<std::string, bool>> compositors;
    Ogre::CompositorChain::InstanceIterator it = chain->getCompositors();
    while (it.hasMoreElements())
    {
      Ogre::CompositorInstance* nextCompInst = it.getNext();
      compositors.push_back(
          std::make_pair(nextCompInst->getCompositor()->getName(),
          nextCompInst->getEnabled()));
    }
    compMgr->removeCompositorChain(this->viewport);
    for (unsigned int i = 0; i < compositors.size(); ++i)
    {
      compMgr->addCompositor(this->viewport, compositors[i].first);
      compMgr->setCompositorEnabled(this->viewport, compositors[i].first,
          compositors[i].second);
    }
  }
}

//////////////////////////////////////////////////
void UserCamera::UpdateFOV()
{
  Camera::UpdateFOV();

  if (this->dataPtr->stereoEnabled && this->viewport)
  {
    double ratio = static_cast<double>(this->viewport->getActualWidth()) /
      static_cast<double>(this->viewport->getActualHeight());

    double hfov =
      this->sdf->Get<double>("horizontal_fov");
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);

    this->dataPtr->rightCamera->setAspectRatio(ratio);
    this->dataPtr->rightCamera->setFOVy(Ogre::Radian(this->LimitFOV(vfov)));
  }
}

//////////////////////////////////////////////////
void UserCamera::SetViewportDimensions(float /*x_*/, float /*y_*/,
                                       float /*w_*/, float /*h_*/)
{
  // this->viewport->setDimensions(x, y, w, h);
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

  if (this->scene->OgreSceneManager()->hasAnimation("cameratrack"))
  {
    this->scene->OgreSceneManager()->destroyAnimation("cameratrack");
  }

  // Start from current position
  ignition::math::Vector3d start = this->WorldPose().Pos();
  start.Correct();

  // Center of visual
  ignition::math::AxisAlignedBox box = _visual->BoundingBox();
  ignition::math::Vector3d visCenter = box.Center() +
    _visual->WorldPose().Pos();
  visCenter.Correct();

  // Direction from start to visual center
  ignition::math::Vector3d dir = visCenter - start;
  dir.Correct();
  dir.Normalize();

  // Distance to move
  ignition::math::Vector3d size = box.Size();
  double maxSize = size.Max();
  double dist = start.Distance(visCenter) - maxSize;

  // Find midway point and change its Z
  ignition::math::Vector3d mid = start + dir*(dist*.5);
  mid.Z(box.Center().Z() + box.Size().Z() + 2.0);

  // Direction from mid to visual center
  dir = visCenter - mid;
  dir.Correct();
  dir.Normalize();

  // Get new distance
  dist = mid.Distance(visCenter) - maxSize;

  // Scale to fit in view
  double scale = maxSize / tan((this->HFOV()/2.0).Radian());

  // End position
  auto end = mid + dir*(dist - scale);

  // Orientation
  auto mat = ignition::math::Matrix4d::LookAt(end, visCenter);

  // Time
  double time = 0.5;

  // Ogre animation
  Ogre::Animation *anim =
    this->scene->OgreSceneManager()->createAnimation("cameratrack", time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  // Start keyframe
  Ogre::TransformKeyFrame *key;
  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Conversions::Convert(start));
  key->setRotation(this->sceneNode->getOrientation());

  // End keyframe
  key = strack->createNodeKeyFrame(time);
  key->setTranslate(Conversions::Convert(end));
  key->setRotation(Conversions::Convert(mat.Rotation()));

  this->animState =
    this->scene->OgreSceneManager()->createAnimationState("cameratrack");

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
      this->WorldPose().Pos().Distance(
      this->dataPtr->orbitViewController->FocalPoint()));
}


//////////////////////////////////////////////////
void UserCamera::SetDevicePixelRatio(const double _ratio)
{
  this->dataPtr->devicePixelRatio = _ratio;
}

//////////////////////////////////////////////////
double UserCamera::DevicePixelRatio() const
{
  return this->dataPtr->devicePixelRatio;
}


//////////////////////////////////////////////////
void UserCamera::CameraToViewportRay(const int _screenx,
    const int _screeny,
    ignition::math::Vector3d &_origin,
    ignition::math::Vector3d &_dir) const
{
  int ratio = static_cast<int>(this->dataPtr->devicePixelRatio);
  int screenx = ratio * _screenx;
  int screeny = ratio * _screeny;

  Camera::CameraToViewportRay(screenx, screeny, _origin, _dir);
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
    auto const &ignBgColor = this->scene->BackgroundColor();
    this->dataPtr->rightViewport->setBackgroundColour(
        Conversions::Convert(ignBgColor));

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR > 9
    this->viewport->setDrawBuffer(Ogre::CBT_BACK_LEFT);
    this->dataPtr->rightViewport->setDrawBuffer(Ogre::CBT_BACK_RIGHT);
#endif

    this->dataPtr->rightViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_SELECTABLE);
  }

  this->viewport->setVisibilityMask(
      GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_SELECTABLE);

  this->initialized = true;

  this->dataPtr->selectionBuffer = new SelectionBuffer(this->scopedUniqueName,
      this->scene->OgreSceneManager(), this->renderTarget);
}

//////////////////////////////////////////////////
void UserCamera::EnableViewController(bool _value) const
{
  this->dataPtr->viewController->SetEnabled(_value);
}

//////////////////////////////////////////////////
VisualPtr UserCamera::Visual(const ignition::math::Vector2i &_mousePos,
    std::string &_mod) const
{
  VisualPtr result;

  if (!this->dataPtr->selectionBuffer)
    return result;

  int ratio = static_cast<int>(this->dataPtr->devicePixelRatio);
  ignition::math::Vector2i mousePos(
      ratio * _mousePos.X(), ratio * _mousePos.Y());

  Ogre::Entity *entity = this->dataPtr->selectionBuffer->OnSelectionClick(
      mousePos.X(), mousePos.Y());

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
void UserCamera::SetFocalPoint(const ignition::math::Vector3d &_pt)
{
  this->dataPtr->orbitViewController->SetFocalPoint(_pt);
}

//////////////////////////////////////////////////
VisualPtr UserCamera::Visual(const ignition::math::Vector2i &_mousePos) const
{
  VisualPtr result;

  int ratio = static_cast<int>(this->dataPtr->devicePixelRatio);
  ignition::math::Vector2i mousePos(
      ratio * _mousePos.X(), ratio * _mousePos.Y());

  Ogre::Entity *entity = this->dataPtr->selectionBuffer->OnSelectionClick(
      mousePos.X(), mousePos.Y());

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
  static ignition::math::Vector3d rpyFactor(0, 0.01, 0.05);

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
    ignition::math::Pose3d pose = this->WorldPose();

    // Get the joystick XYZ
    if (_msg->has_translation())
    {
      const double transRotRatio = 0.05;
      ignition::math::Vector3d trans =
        msgs::ConvertIgn(_msg->translation()) * transRotRatio;
      pose.Pos() = pose.Rot().RotateVector(trans) + pose.Pos();
    }

    // Get the jostick RPY. We are disabling rotation around x.
    if (_msg->has_rotation())
    {
      ignition::math::Vector3d rot =
        msgs::ConvertIgn(_msg->rotation()) * rpyFactor;
      pose.Rot().Euler(pose.Rot().Euler() + rot);
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
    ignition::math::Pose3d pose(msgs::ConvertIgn(_msg->position()),
                                msgs::ConvertIgn(_msg->orientation()));
    this->SetWorldPose(pose);
  }
}

//////////////////////////////////////////////////
void UserCamera::SetClipDist(const float _near, const float _far)
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

//////////////////////////////////////////////////
bool UserCamera::StereoEnabled() const
{
  return this->dataPtr->stereoEnabled;
}

//////////////////////////////////////////////////
void UserCamera::EnableStereo(bool _enable)
{
#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR > 9
  if (this->dataPtr->rightViewport)
  {
    if (_enable)
    {
      this->dataPtr->rightViewport->setDrawBuffer(Ogre::CBT_BACK_RIGHT);
      this->dataPtr->rightViewport->setAutoUpdated(true);
      this->viewport->setDrawBuffer(Ogre::CBT_BACK_LEFT);
    }
    else
    {
      this->dataPtr->rightViewport->setAutoUpdated(false);
      this->dataPtr->rightViewport->setDrawBuffer(Ogre::CBT_BACK);
      this->viewport->setDrawBuffer(Ogre::CBT_BACK);
    }
  }
  else
  {
    gzwarn << "Tried to enable/disable stereo. "
           << "However, stereo is turned off via the gui.ini file.\n";
  }
#else
  gzwarn << "Tried to EnableStereo("
         << _enable
         << "). However, Ogre version >= 1.10.0 is required.\n";
#endif
}

/////////////////////////////////////////////////
bool UserCamera::SetProjectionType(const std::string &_type)
{
  if (_type == "orthographic")
    this->SetViewController("ortho");
  else if (!this->dataPtr->prevViewControllerName.empty())
    this->SetViewController(this->dataPtr->prevViewControllerName);

  return Camera::SetProjectionType(_type);
}

/////////////////////////////////////////////////
ignition::math::Vector2i UserCamera::Project(
    const ignition::math::Vector3d &_pt) const
{
  auto pt = Camera::Project(_pt);
  return pt / this->dataPtr->devicePixelRatio;
}
