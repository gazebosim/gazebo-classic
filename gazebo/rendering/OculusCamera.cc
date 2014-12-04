/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <OVR.h>
#include <sstream>
#include <string>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"
#include "gazebo/rendering/RenderEngine.hh"
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
#include "gazebo/rendering/OculusCamera.hh"


using namespace gazebo;
using namespace rendering;

const float g_defaultNearClip = 0.1f;
const float g_defaultFarClip = 500.0f;
const float g_defaultIPD = 0.064f;
const float g_defautlProjectionCenterOffset = 0.14529906f;
const float g_defaultDistortion[4] = {1.0f, 0.22f, 0.24f, 0};

//////////////////////////////////////////////////
OculusCamera::OculusCamera(const std::string &_name, ScenePtr _scene)
  : Camera(_name, _scene)
{
  // Oculus is not ready yet.
  this->ready = false;

  // Set default OculusCamera render rate to 120Hz
  this->SetRenderRate(120.0);

  OVR::System::Init(OVR::Log::ConfigureDefaultLog(OVR::LogMask_None));

  this->deviceManager = OVR::DeviceManager::Create();
  if (!this->deviceManager)
    return;

  this->stereoConfig = new OVR::Util::Render::StereoConfig();
  if (!this->stereoConfig)
    return;

  this->centerOffset = this->stereoConfig->GetProjectionCenterOffset();

  this->hmd =
    this->deviceManager->EnumerateDevices<OVR::HMDDevice>().CreateDevice();
  if (this->hmd)
  {
    OVR::HMDInfo devinfo;
    this->hmd->GetDeviceInfo(&devinfo);
    this->stereoConfig->SetHMDInfo(devinfo);
    this->sensor = this->hmd->GetSensor();
  }
  else
  {
    this->sensor =
      this->deviceManager->EnumerateDevices<OVR::SensorDevice>().CreateDevice();
  }

  if (!this->sensor)
    return;

  this->sensorFusion = new OVR::SensorFusion();
  if (!sensorFusion)
    return;

  this->sensorFusion->AttachToSensor(this->sensor);
  this->sensorFusion->SetPredictionEnabled(true);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->controlSub = this->node->Subscribe("~/world_control",
                                           &OculusCamera::OnControl, this);

  // Oculus is now ready.
  this->ready = true;
}

//////////////////////////////////////////////////
OculusCamera::~OculusCamera()
{
  this->connections.clear();
}

//////////////////////////////////////////////////
void OculusCamera::Load(sdf::ElementPtr _sdf)
{
  if (this->Ready())
    Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void OculusCamera::OnControl(ConstWorldControlPtr &_data)
{
  if (_data->has_reset() && _data->reset().has_all() && _data->reset().all())
  {
    this->ResetSensor();
  }
}

//////////////////////////////////////////////////
void OculusCamera::Load()
{
  if (this->Ready())
    Camera::Load();
}

//////////////////////////////////////////////////
void OculusCamera::Init()
{
  if (!this->Ready())
    return;

  Camera::Init();

  this->SetHFOV(GZ_DTOR(60));

  // Oculus
  {
    this->rightCamera = this->scene->GetManager()->createCamera(
      "OculusUserRight");
    this->rightCamera->pitch(Ogre::Degree(90));

    // Don't yaw along variable axis, causes leaning
    this->rightCamera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    this->rightCamera->setDirection(1, 0, 0);

    this->sceneNode->attachObject(this->rightCamera);

    this->rightCamera->setAutoAspectRatio(false);
    this->camera->setAutoAspectRatio(false);

    this->rightCamera->setNearClipDistance(g_defaultNearClip);
    this->rightCamera->setFarClipDistance(g_defaultFarClip);

    this->camera->setNearClipDistance(g_defaultNearClip);
    this->camera->setFarClipDistance(g_defaultFarClip);
  }

  // Careful when setting this value.
  // A far clip that is too close will have bad side effects on the
  // lighting. When using deferred shading, the light's use geometry that
  // trigger shaders. If the far clip is too close, the light's geometry is
  // clipped and wholes appear in the lighting.
  switch (RenderEngine::Instance()->GetRenderPathType())
  {
    case RenderEngine::VERTEX:
      this->SetClipDist(g_defaultNearClip, g_defaultFarClip);
      break;

    case RenderEngine::DEFERRED:
    case RenderEngine::FORWARD:
      this->SetClipDist(g_defaultNearClip, g_defaultFarClip);
      break;

    default:
      this->SetClipDist(g_defaultNearClip, g_defaultFarClip);
      break;
  }
}

//////////////////////////////////////////////////
void OculusCamera::Update()
{
  if (!this->Ready())
    return;

  Camera::Update();

  OVR::Quatf q = this->sensorFusion->GetPredictedOrientation();

  // Set the orientation, and correct for the oculus coordinate system
  this->sceneNode->setOrientation(Ogre::Quaternion(q.w, -q.z, -q.x, q.y));

  this->sceneNode->needUpdate();
}

//////////////////////////////////////////////////
void OculusCamera::ResetSensor()
{
  this->sensorFusion->Reset();
}

//////////////////////////////////////////////////
bool OculusCamera::Ready()
{
  return this->ready;
}

//////////////////////////////////////////////////
void OculusCamera::PostRender()
{
  Camera::PostRender();
}

//////////////////////////////////////////////////
void OculusCamera::Fini()
{
  Camera::Fini();
}

/////////////////////////////////////////////////
bool OculusCamera::AttachToVisualImpl(VisualPtr _visual,
    bool _inheritOrientation,
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
  }

  return true;
}

//////////////////////////////////////////////////
bool OculusCamera::TrackVisualImpl(VisualPtr _visual)
{
  Camera::TrackVisualImpl(_visual);

  return true;
}

//////////////////////////////////////////////////
unsigned int OculusCamera::GetImageWidth() const
{
  return this->viewport->getActualWidth();
}

//////////////////////////////////////////////////
unsigned int OculusCamera::GetImageHeight() const
{
  return this->viewport->getActualHeight();
}

//////////////////////////////////////////////////
void OculusCamera::Resize(unsigned int /*_w*/, unsigned int /*_h*/)
{
  if (this->viewport)
  {
    this->viewport->setDimensions(0, 0, 0.5, 1);
    this->rightViewport->setDimensions(0.5, 0, 0.5, 1);

    delete [] this->saveFrameBuffer;
    this->saveFrameBuffer = NULL;
  }
}

//////////////////////////////////////////////////
float OculusCamera::GetAvgFPS() const
{
  return RenderEngine::Instance()->GetWindowManager()->GetAvgFPS(
      this->windowId);
}

//////////////////////////////////////////////////
unsigned int OculusCamera::GetTriangleCount() const
{
  return RenderEngine::Instance()->GetWindowManager()->GetTriangleCount(
      this->windowId);
}

//////////////////////////////////////////////////
bool OculusCamera::MoveToPosition(const math::Pose &_pose, double _time)
{
  return Camera::MoveToPosition(_pose, _time);
}

//////////////////////////////////////////////////
void OculusCamera::MoveToVisual(const std::string &_name)
{
  VisualPtr visualPtr = this->scene->GetVisual(_name);
  if (visualPtr)
    this->MoveToVisual(visualPtr);
  else
    gzerr << "MoveTo Unknown visual[" << _name << "]\n";
}

//////////////////////////////////////////////////
void OculusCamera::MoveToVisual(VisualPtr _visual)
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
  Ogre::Quaternion yawFinal(Ogre::Radian(yawAngle), Ogre::Vector3(0, 0, 1));
  Ogre::Quaternion pitchFinal(Ogre::Radian(pitchAngle), Ogre::Vector3(0, 1, 0));

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

  key = strack->createNodeKeyFrame(time);
  key->setTranslate(Ogre::Vector3(end.x, end.y, end.z));
  key->setRotation(yawFinal);

  this->animState =
    this->scene->GetManager()->createAnimationState("cameratrack");

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
void OculusCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  this->viewport->setDimensions(0, 0, 0.5, 1.0);

  this->rightViewport =
    this->renderTarget->addViewport(this->rightCamera, 1,
        0.5f, 0, 0.5f, 1.0f);
  this->rightViewport->setBackgroundColour(
        Conversions::Convert(this->scene->GetBackgroundColor()));

  RTShaderSystem::AttachViewport(this->rightViewport, this->GetScene());

  this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL);
  this->rightViewport->setVisibilityMask(GZ_VISIBILITY_ALL);

  this->initialized = true;

  this->Oculus();
}

//////////////////////////////////////////////////
void OculusCamera::Oculus()
{
  if (!this->Ready())
    return;

  Ogre::MaterialPtr matLeft =
    Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Oculus");
  Ogre::MaterialPtr matRight = matLeft->clone("Ogre/Compositor/Oculus/Right");

  Ogre::GpuProgramParametersSharedPtr pParamsLeft =
    matLeft->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::GpuProgramParametersSharedPtr pParamsRight =
    matRight->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::Vector4 hmdwarp;

  if (this->stereoConfig)
  {
    hmdwarp = Ogre::Vector4(this->stereoConfig->GetDistortionK(0),
                            this->stereoConfig->GetDistortionK(1),
                            this->stereoConfig->GetDistortionK(2),
                            this->stereoConfig->GetDistortionK(3));
  }
  else
  {
    hmdwarp = Ogre::Vector4(g_defaultDistortion[0],
                            g_defaultDistortion[1],
                            g_defaultDistortion[2],
                            g_defaultDistortion[3]);
  }

  pParamsLeft->setNamedConstant("HmdWarpParam", hmdwarp);
  pParamsRight->setNamedConstant("HmdWarpParam", hmdwarp);

  Ogre::Vector4 hmdchrom;
  if (this->stereoConfig)
  {
    hmdchrom = Ogre::Vector4(
        this->stereoConfig->GetHMDInfo().ChromaAbCorrection);
  }
  else
  {
    hmdchrom = Ogre::Vector4(0.996, -0.004, 1.014, 0.0f);
  }

  pParamsLeft->setNamedConstant("ChromAbParam", hmdchrom);
  pParamsRight->setNamedConstant("ChromAbParam", hmdchrom);

  pParamsLeft->setNamedConstant("LensCenter", 0.5f +
      (this->stereoConfig->GetProjectionCenterOffset()/2.0f));

  pParamsRight->setNamedConstant("LensCenter", 0.5f -
      (this->stereoConfig->GetProjectionCenterOffset()/2.0f));

  Ogre::CompositorPtr comp =
    Ogre::CompositorManager::getSingleton().getByName("OculusRight");
  comp->getTechnique(0)->getOutputTargetPass()->getPass(0)->setMaterialName(
      "Ogre/Compositor/Oculus/Right");

  for (int i = 0; i < 2; ++i)
  {
    Ogre::Camera *cam = i == 0 ? this->camera : this->rightCamera;

    int idx = i * 2 - 1;
    if (this->stereoConfig)
    {
      // Setup cameras.
      cam->setNearClipDistance(this->stereoConfig->GetEyeToScreenDistance());
      cam->setFarClipDistance(g_defaultFarClip);
      cam->setPosition(0, idx * this->stereoConfig->GetIPD() * 0.5f * -1.0, 0);
      cam->setAspectRatio(this->stereoConfig->GetAspect());
      cam->setFOVy(Ogre::Radian(this->stereoConfig->GetYFOVRadians()));

      // Oculus requires offset projection, create a custom projection matrix
      Ogre::Matrix4 proj = Ogre::Matrix4::IDENTITY;
      proj.setTrans(Ogre::Vector3(
            -this->stereoConfig->GetProjectionCenterOffset() * idx, 0, 0));
      cam->setCustomProjectionMatrix(true, proj * cam->getProjectionMatrix());
    }
    else
    {
      cam->setNearClipDistance(g_defaultNearClip);
      cam->setFarClipDistance(g_defaultFarClip);
      cam->setPosition(idx * g_defaultIPD * 0.5f, 0, 0);
    }

    if (i == 0)
    {
      this->compositors[i] =
        Ogre::CompositorManager::getSingleton().addCompositor(
            this->viewport, "OculusLeft");
      if (!this->compositors[i])
        gzerr << "Invalid compositor\n";
      this->compositors[i]->setEnabled(true);
    }
    else
    {
      this->compositors[i] =
        Ogre::CompositorManager::getSingleton().addCompositor(
            this->rightViewport, "OculusRight");
      this->compositors[i]->setEnabled(true);
    }
  }
}

/////////////////////////////////////////////////
void OculusCamera::AdjustAspect(double _v)
{
  if (!this->Ready())
    return;

  for (int i = 0; i < 2; ++i)
  {
    Ogre::Camera *cam = i == 0 ? this->camera : this->rightCamera;
    cam->setAspectRatio(cam->getAspectRatio() + _v);
  }
}
