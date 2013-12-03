/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <sstream>

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

#include "OVR.h"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
OculusCamera::OculusCamera(const std::string &_name, ScenePtr _scene)
  : Camera(_name, _scene)
{
  // Set default OculusCamera render rate to 30Hz
  this->SetRenderRate(30.0);

  OVR::System::Init(OVR::Log::ConfigureDefaultLog(OVR::LogMask_All));

  m_deviceManager = OVR::DeviceManager::Create();
  if (!m_deviceManager)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create Device Manager");
    return;
  }

  Ogre::LogManager::getSingleton().logMessage("Oculus: Created Device Manager");
  m_stereoConfig = new OVR::Util::Render::StereoConfig();
  if (!m_stereoConfig)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create StereoConfig");
    return;
  }
  m_centreOffset = m_stereoConfig->GetProjectionCenterOffset();
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created StereoConfig");

  m_hmd = m_deviceManager->EnumerateDevices<OVR::HMDDevice>().CreateDevice();
  if (m_hmd)
  {
    OVR::HMDInfo devinfo;
    m_hmd->GetDeviceInfo(&devinfo);
    m_stereoConfig->SetHMDInfo(devinfo);
    m_sensor = m_hmd->GetSensor();
  }
  else
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create HMD");
    m_sensor = m_deviceManager->EnumerateDevices<OVR::SensorDevice>().CreateDevice();
  }

  Ogre::LogManager::getSingleton().logMessage("Oculus: Created HMD");
  if (!m_sensor)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create sensor");
    return ;
  }
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created sensor");
  m_sensorFusion = new OVR::SensorFusion();
  if (!m_sensorFusion)
  {
    Ogre::LogManager::getSingleton().logMessage("Oculus: Failed to create SensorFusion");
    return ;
  }
  Ogre::LogManager::getSingleton().logMessage("Oculus: Created SensorFusion");
  m_sensorFusion->AttachToSensor(m_sensor);

  // m_magCalibration = new OVR::Util::MagCalibration();
  // m_magCalibration->BeginAutoCalibration(*m_sensorFusion);

  m_oculusReady = true;
  Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus setup completed successfully");

}

//////////////////////////////////////////////////
OculusCamera::~OculusCamera()
{
  this->connections.clear();
}

//////////////////////////////////////////////////
void OculusCamera::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void OculusCamera::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void OculusCamera::Init()
{
  Camera::Init();

  // Oculus
  {
    this->leftCamera = this->scene->GetManager()->createCamera("UserLeft");
    this->leftCamera->pitch(Ogre::Degree(90));

    // Don't yaw along variable axis, causes leaning
    this->leftCamera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    this->leftCamera->setDirection(1, 0, 0);

    this->pitchNode->attachObject(this->leftCamera);
    this->leftCamera->setAutoAspectRatio(true);

    this->leftCamera->setNearClipDistance(0.1);
    this->leftCamera->setFarClipDistance(200);
    this->leftCamera->setRenderingDistance(200);
  }
  // Orig
  // this->camera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
  // this->camera->setDirection(1, 0, 0);

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
    this->pitchNode->createChildSceneNode(this->name + "AxisNode");

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
void OculusCamera::SetWorldPose(const math::Pose &_pose)
{
  Camera::SetWorldPose(_pose);
}

//////////////////////////////////////////////////
void OculusCamera::Update()
{
  Camera::Update();

  OVR::Quatf q = m_sensorFusion->GetPredictedOrientation();

  // Set the orientation, and correct for the oculus coordinate system
  this->SetWorldRotation(math::Quaternion(q.w, -q.z, -q.x, q.y));
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

//////////////////////////////////////////////////
void OculusCamera::HandleMouseEvent(const common::MouseEvent & /*_evt*/)
{
}

/////////////////////////////////////////////////
void OculusCamera::HandleKeyPressEvent(const std::string & /*_key*/)
{
}

/////////////////////////////////////////////////
void OculusCamera::HandleKeyReleaseEvent(const std::string & /*_key*/)
{
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
bool OculusCamera::TrackVisualImpl(VisualPtr _visual)
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
void OculusCamera::SetViewController(const std::string & /*_type*/)
{
}

//////////////////////////////////////////////////
void OculusCamera::SetViewController(const std::string & /*_type*/,
                                    const math::Vector3 &/*_pos*/)
{
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
    this->leftViewport->setDimensions(0.5,0,0.5,1);

    double ratio = static_cast<double>(this->viewport->getActualWidth()) /
                   static_cast<double>(this->viewport->getActualHeight());

    double hfov =
      this->sdf->Get<double>("horizontal_fov");
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);

    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));

    this->leftCamera->setAspectRatio(ratio);
    this->leftCamera->setFOVy(Ogre::Radian(vfov));

    delete [] this->saveFrameBuffer;
    this->saveFrameBuffer = NULL;
  }
}

//////////////////////////////////////////////////
void OculusCamera::SetViewportDimensions(float /*x_*/, float /*y_*/,
                                       float /*w_*/, float /*h_*/)
{
  // this->viewport->setDimensions(x, y, w, h);
}

//////////////////////////////////////////////////
float OculusCamera::GetAvgFPS() const
{
  return RenderEngine::Instance()->GetWindowManager()->GetAvgFPS(
      this->windowId);
}

//////////////////////////////////////////////////
float OculusCamera::GetTriangleCount() const
{
  return RenderEngine::Instance()->GetWindowManager()->GetTriangleCount(
      this->windowId);
}

//////////////////////////////////////////////////
void OculusCamera::ToggleShowVisual()
{
  // this->visual->ToggleVisible();
}

//////////////////////////////////////////////////
void OculusCamera::ShowVisual(bool /*_s*/)
{
  // this->visual->SetVisible(_s);
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
  Ogre::NodeAnimationTrack *ptrack = anim->createNodeTrack(1, this->pitchNode);


  Ogre::TransformKeyFrame *key;

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = ptrack->createNodeKeyFrame(0);
  key->setRotation(this->pitchNode->getOrientation());

  /*key = strack->createNodeKeyFrame(time * 0.5);
  key->setTranslate(Ogre::Vector3(mid.x, mid.y, mid.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(time * 0.5);
  key->setRotation(pitchFinal);
  */

  key = strack->createNodeKeyFrame(time);
  key->setTranslate(Ogre::Vector3(end.x, end.y, end.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(time);
  key->setRotation(pitchFinal);

  this->animState =
    this->scene->GetManager()->createAnimationState("cameratrack");

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();
}

/////////////////////////////////////////////////
void OculusCamera::OnMoveToVisualComplete()
{
}

//////////////////////////////////////////////////
void OculusCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  this->leftViewport =
    this->renderTarget->addViewport(this->leftCamera, 1,
        0.5f, 0, 0.5f, 1.0f);
  this->leftViewport->setBackgroundColour(
        Conversions::Convert(this->scene->GetBackgroundColor()));

  RTShaderSystem::AttachViewport(this->leftViewport, this->GetScene());

  this->leftCamera->setAspectRatio(this->camera->getAspectRatio());
  this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL);
  this->leftViewport->setVisibilityMask(GZ_VISIBILITY_ALL);

  this->initialized = true;

  // this->selectionBuffer = new SelectionBuffer(this->name,
  //    this->scene->GetManager(), this->renderTarget);

  this->Oculus();
}


//////////////////////////////////////////////////
void OculusCamera::EnableViewController(bool /*_value*/) const
{
}

//////////////////////////////////////////////////
VisualPtr OculusCamera::GetVisual(const math::Vector2i & /*_mousePos*/,
                                  std::string & /*_mod*/)
{
  VisualPtr result;
  return result;
}

//////////////////////////////////////////////////
void OculusCamera::SetFocalPoint(const math::Vector3 & /*_pt*/)
{
}

//////////////////////////////////////////////////
VisualPtr OculusCamera::GetVisual(const math::Vector2i & /*_mousePos*/) const
{
  VisualPtr result;

  return result;
}

//////////////////////////////////////////////////
std::string OculusCamera::GetViewControllerTypeString()
{
  return "";
}

void OculusCamera::Oculus()
{
  Ogre::MaterialPtr matLeft =
    Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Oculus");
  Ogre::MaterialPtr matRight = matLeft->clone("Ogre/Compositor/Oculus/Right");

  Ogre::GpuProgramParametersSharedPtr pParamsLeft =
    matLeft->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::GpuProgramParametersSharedPtr pParamsRight =
    matRight->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  Ogre::Vector4 hmdwarp;

  if(m_stereoConfig)
  {
    hmdwarp = Ogre::Vector4(m_stereoConfig->GetDistortionK(0),
        m_stereoConfig->GetDistortionK(1),
        m_stereoConfig->GetDistortionK(2),
        m_stereoConfig->GetDistortionK(3));
  }
  else
  {
    hmdwarp = Ogre::Vector4(1.0, 0.22, 0.24, 0);
    /*hmdwarp = Ogre::Vector4(g_defaultDistortion[0],
        g_defaultDistortion[1],
        g_defaultDistortion[2],
        g_defaultDistortion[3]);
        */
  }

  float defaultProjCenterOffset = 0.14529906;

  pParamsLeft->setNamedConstant("HmdWarpParam", hmdwarp);
  pParamsRight->setNamedConstant("HmdWarpParam", hmdwarp);
  pParamsLeft->setNamedConstant("LensCentre", 0.5f +
      (m_stereoConfig->GetProjectionCenterOffset()/2.0f));

  pParamsRight->setNamedConstant("LensCentre", 0.5f -
      (m_stereoConfig->GetProjectionCenterOffset()/2.0f));

  Ogre::CompositorPtr comp =
    Ogre::CompositorManager::getSingleton().getByName("OculusRight");
  comp->getTechnique(0)->getOutputTargetPass()->getPass(0)->setMaterialName(
      "Ogre/Compositor/Oculus/Right");

  double g_defaultFarClip = 200;
  Ogre::Camera *cam;
  for(int i=0;i<2;++i)
  {
    cam = i == 0 ? this->camera : this->leftCamera;

    if(m_stereoConfig)
    {
      // Setup cameras.
      cam->setNearClipDistance(m_stereoConfig->GetEyeToScreenDistance());
      cam->setFarClipDistance(g_defaultFarClip);
      cam->setPosition((i * 2 - 1) * m_stereoConfig->GetIPD() * 0.5f, 0, 0);
      cam->setAspectRatio(m_stereoConfig->GetAspect());
      cam->setFOVy(Ogre::Radian(m_stereoConfig->GetYFOVRadians()));

      // Oculus requires offset projection, create a custom projection matrix
      Ogre::Matrix4 proj = Ogre::Matrix4::IDENTITY;
      float temp = m_stereoConfig->GetProjectionCenterOffset();
      proj.setTrans(Ogre::Vector3(-m_stereoConfig->GetProjectionCenterOffset() * (2 * i - 1), 0, 0));
      cam->setCustomProjectionMatrix(true, proj * cam->getProjectionMatrix());
    }
    else
    {
      /*cameras[i]->setNearClipDistance(g_defaultNearClip);
      cameras[i]->setFarClipDistance(g_defaultFarClip);
      cameras[i]->setPosition((i*2-1) * g_defaultIPD * 0.5f, 0, 0);
      */
      if (i == 0)
        this->camera->setPosition(0,(i*2-1) * 0.064 * 0.5f, 0);
      else
        this->leftCamera->setPosition(0, (i*2-1) * 0.064 * 0.5f, 0);
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
            this->leftViewport, "OculusRight");
      this->compositors[i]->setEnabled(true);
    }
  }
}
