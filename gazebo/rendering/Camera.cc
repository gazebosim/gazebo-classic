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

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#ifndef _WIN32
  #include <dirent.h>
#else
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/rendering/skyx/include/SkyX.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/VideoEncoder.hh"
#include "gazebo/common/CommonIface.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Distortion.hh"
#include "gazebo/rendering/CameraPrivate.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderEvents.hh"

using namespace gazebo;
using namespace rendering;


unsigned int CameraPrivate::cameraCounter = 0;

//////////////////////////////////////////////////
Camera::Camera(const std::string &_name, ScenePtr _scene,
               bool _autoRender)
  : dataPtr(new CameraPrivate)
{
  this->initialized = false;
  this->cameraProjectiveMatrix = ignition::math::Matrix4d::Identity;
  this->cameraUsingIntrinsics = false;
  this->sdf.reset(new sdf::Element);
  sdf::initFile("camera.sdf", this->sdf);

  this->animState = NULL;
  this->windowId = 0;
  this->scene = _scene;

  this->newData = false;

  this->textureWidth = this->textureHeight = 0;

  this->saveFrameBuffer = NULL;
  this->saveCount = 0;
  this->bayerFrameBuffer = NULL;

  this->name = _name;
  this->scopedName = this->scene->Name() + "::" + _name;
  this->scopedUniqueName = this->scopedName + "(" +
    boost::lexical_cast<std::string>(++this->dataPtr->cameraCounter) + ")";

  this->renderTarget = NULL;
  this->renderTexture = NULL;

  this->captureData = false;
  this->captureDataOnce = false;

  this->camera = NULL;
  this->viewport = NULL;

  this->sceneNode = NULL;

  this->screenshotPath = getenv(HOMEDIR);
  this->screenshotPath += "/.gazebo/pictures";

  // Connect to the render signal
  this->connections.push_back(
      event::Events::ConnectPreRender(std::bind(&Camera::Update, this)));

  if (_autoRender)
  {
    this->connections.push_back(event::Events::ConnectRender(
          std::bind(&Camera::Render, this, false)));
    this->connections.push_back(
        event::Events::ConnectPostRender(
          std::bind(&Camera::PostRender, this)));
  }

  this->lastRenderWallTime = common::Time::GetWallTime();

  // Set default render rate to unlimited
  this->SetRenderRate(0.0);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->antiAliasingValue = 4;

  this->dataPtr->cameraIntrinsicMatrix = ignition::math::Matrix3d::Identity;
}

//////////////////////////////////////////////////
Camera::~Camera()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Camera::Load(sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);
  this->Load();
}

//////////////////////////////////////////////////
void Camera::Load()
{
  sdf::ElementPtr imgElem = this->sdf->GetElement("image");
  if (imgElem)
  {
    this->imageWidth = imgElem->Get<int>("width");
    this->imageHeight = imgElem->Get<int>("height");
    this->imageFormat = this->OgrePixelFormat(
        imgElem->Get<std::string>("format"));

    this->dataPtr->antiAliasingValue = imgElem->Get<int>("anti_aliasing");
  }
  else
    gzthrow("Camera has no <image> tag.");

  // Create the directory to store frames
  if (this->sdf->HasElement("save") &&
      this->sdf->GetElement("save")->Get<bool>("enabled"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("save");
    std::string command;

    command = "mkdir " + elem->Get<std::string>("path")+ " 2>>/dev/null";
    if (system(command.c_str()) < 0)
      gzerr << "Error making directory\n";
  }

  if (this->sdf->HasElement("horizontal_fov"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("horizontal_fov");
    ignition::math::Angle angle = elem->Get<double>();
    if (angle < 0.01 || angle > M_PI*2)
    {
      gzthrow("Camera horizontal field of view invalid.");
    }
    this->SetHFOV(angle);
  }

  // Only create a command subscription for real cameras. Ignore camera's
  // created for visualization purposes.
  if (this->name.find("_GUIONLY_") == std::string::npos)
  {
    std::string topicName = this->Name();
    boost::replace_all(topicName, "::", "/");

    this->dataPtr->cmdSub = this->dataPtr->node->Subscribe(
        "~/" + topicName + "/cmd", &Camera::OnCmdMsg, this, true);
  }

  if (this->sdf->HasElement("distortion"))
  {
    this->dataPtr->distortion.reset(new Distortion());
    this->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));
  }

  this->LoadCameraIntrinsics();
}

//////////////////////////////////////////////////
void Camera::LoadCameraIntrinsics()
{
  if (this->sdf->HasElement("lens"))
  {
    sdf::ElementPtr sdfLens = this->sdf->GetElement("lens");
    if (sdfLens->HasElement("intrinsics"))
    {
      sdf::ElementPtr sdfIntrinsics = sdfLens->GetElement("intrinsics");
      this->UpdateCameraIntrinsics(
          sdfIntrinsics->Get<double>("fx"),
          sdfIntrinsics->Get<double>("fy"),
          sdfIntrinsics->Get<double>("cx"),
          sdfIntrinsics->Get<double>("cy"),
          sdfIntrinsics->Get<double>("s"));
    }
  }
}

//////////////////////////////////////////////////
void Camera::UpdateCameraIntrinsics(
    const double _cameraIntrinsicsFx, const double _cameraIntrinsicsFy,
    const double _cameraIntrinsicsCx, const double _cameraIntrinsicsCy,
    const double _cameraIntrinsicsS)
{
  double clipNear = 0.5;
  double clipFar = 2.5;

  sdf::ElementPtr clipElem = this->sdf->GetElement("clip");
  if (clipElem)
  {
    clipNear = clipElem->Get<double>("near");
    clipFar = clipElem->Get<double>("far");
  }

  this->cameraProjectiveMatrix = this->BuildProjectionMatrix(
    this->imageWidth, this->imageHeight,
    _cameraIntrinsicsFx, _cameraIntrinsicsFy,
    _cameraIntrinsicsCx, _cameraIntrinsicsCy,
    _cameraIntrinsicsS, clipNear, clipFar);
  this->dataPtr->cameraIntrinsicMatrix = this->BuildIntrinsicMatrix(
             _cameraIntrinsicsFx, _cameraIntrinsicsFy,
             _cameraIntrinsicsCx, _cameraIntrinsicsCy);

  if (this->camera != nullptr)
  {
    this->camera->setCustomProjectionMatrix(true,
        Conversions::Convert(this->cameraProjectiveMatrix));
  }

  this->cameraUsingIntrinsics = true;
}

//////////////////////////////////////////////////
ignition::math::Matrix4d Camera::BuildNDCMatrix(
    const double _left, const double _right,
    const double _bottom, const double _top,
    const double _near, const double _far)
{
  double inverseWidth = 1.0 / (_right - _left);
  double inverseHeight = 1.0 / (_top - _bottom);
  double inverseDistance = 1.0 / (_far - _near);

  return ignition::math::Matrix4d(
           2.0 * inverseWidth,
           0.0,
           0.0,
           -(_right + _left) * inverseWidth,
           0.0,
           2.0 * inverseHeight,
           0.0,
           -(_top + _bottom) * inverseHeight,
           0.0,
           0.0,
           -2.0 * inverseDistance,
           -(_far + _near) * inverseDistance,
           0.0,
           0.0,
           0.0,
           1.0);
}

//////////////////////////////////////////////////
ignition::math::Matrix4d Camera::BuildPerspectiveMatrix(
    const double _intrinsicsFx, const double _intrinsicsFy,
    const double _intrinsicsCx, const double _intrinsicsCy,
    const double _intrinsicsS,
    const double _clipNear, const double _clipFar)
{
  return ignition::math::Matrix4d(
           _intrinsicsFx,
           _intrinsicsS,
           -_intrinsicsCx,
           0.0,
           0.0,
           _intrinsicsFy,
           -_intrinsicsCy,
           0.0,
           0.0,
           0.0,
           _clipNear + _clipFar,
           _clipNear * _clipFar,
           0.0,
           0.0,
           -1.0,
           0.0);
}

//////////////////////////////////////////////////
ignition::math::Matrix4d Camera::BuildProjectiveMatrix(
    const double _imageWidth, const double _imageHeight,
    const double _intrinsicsFx, const double _intrinsicsFy,
    const double _intrinsicsCx, double _intrinsicsCy,
    const double _intrinsicsS,
    const double _clipNear, const double _clipFar)
{
  return BuildProjectionMatrix(_imageWidth, _imageHeight,
    _intrinsicsFx, _intrinsicsFy, _intrinsicsCx, _intrinsicsCy, _intrinsicsS,
    _clipNear, _clipFar);
}

//////////////////////////////////////////////////
ignition::math::Matrix4d Camera::BuildProjectionMatrix(
    const double _imageWidth, const double _imageHeight,
    const double _intrinsicsFx, const double _intrinsicsFy,
    const double _intrinsicsCx, double _intrinsicsCy,
    const double _intrinsicsS,
    const double _clipNear, const double _clipFar)
{
  return Camera::BuildNDCMatrix(
           0, _imageWidth, 0, _imageHeight, _clipNear, _clipFar) *
         Camera::BuildPerspectiveMatrix(
           _intrinsicsFx, _intrinsicsFy,
           _intrinsicsCx, _imageHeight - _intrinsicsCy,
           _intrinsicsS, _clipNear, _clipFar);
}

//////////////////////////////////////////////////
ignition::math::Matrix3d Camera::BuildIntrinsicMatrix(
    const double _intrinsicsFx, const double _intrinsicsFy,
    const double _intrinsicsCx, double _intrinsicsCy)
{
  return ignition::math::Matrix3d(
          _intrinsicsFx,
          0,
          _intrinsicsCx,
          0,
          _intrinsicsFy,
          _intrinsicsCy,
          0,
          0,
          1
  );
}

//////////////////////////////////////////////////
void Camera::Init()
{
  this->SetSceneNode(
      this->scene->OgreSceneManager()->getRootSceneNode()->createChildSceneNode(
        this->scopedUniqueName + "_SceneNode"));

  this->CreateCamera();

  this->camera->setAutoAspectRatio(true);

  this->sceneNode->setInheritScale(false);

  this->saveCount = 0;

  this->SetClipDist();

  this->dataPtr->trackIsStatic = false;
  this->dataPtr->trackUseModelFrame = true;
  this->dataPtr->trackMinDistance = 8.0;
  this->dataPtr->trackMaxDistance = 8.0;
  this->dataPtr->trackPos = ignition::math::Vector3d(-5.0, 0.0, 3.0);
  this->dataPtr->trackInheritYaw = false;
}

//////////////////////////////////////////////////
void Camera::Fini()
{
  this->dataPtr->videoEncoder.Reset();

  if (this->saveFrameBuffer)
    delete [] this->saveFrameBuffer;
  this->saveFrameBuffer = NULL;

  if (this->bayerFrameBuffer)
    delete [] this->bayerFrameBuffer;
  this->bayerFrameBuffer = NULL;

  this->initialized = false;

  this->dataPtr->cmdSub.reset();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
  this->dataPtr->node.reset();

  this->dataPtr->distortion.reset();
  this->dataPtr->trackedVisual.reset();

  if (this->viewport && this->scene)
    RTShaderSystem::DetachViewport(this->viewport, this->scene);

  if (this->renderTarget)
    this->renderTarget->removeAllViewports();
  this->renderTarget = NULL;

  if (this->renderTexture)
    Ogre::TextureManager::getSingleton().remove(this->renderTexture->getName());
  this->renderTexture = NULL;

  if (this->camera)
  {
    this->scene->OgreSceneManager()->destroyCamera(this->scopedUniqueName);
    this->camera = NULL;
  }

  this->sceneNode = NULL;
  this->cameraNode = nullptr;
  this->viewport = NULL;

  this->scene.reset();
  this->connections.clear();

  if (this->sdf)
    this->sdf->Reset();
  this->sdf.reset();
}

//////////////////////////////////////////////////
void Camera::SetWindowId(unsigned int _windowId)
{
  this->windowId = _windowId;
}

//////////////////////////////////////////////////
unsigned int Camera::WindowId() const
{
  return this->windowId;
}

//////////////////////////////////////////////////
void Camera::SetScene(ScenePtr _scene)
{
  this->scene = _scene;
}

//////////////////////////////////////////////////
void Camera::Update()
{
  IGN_PROFILE("rendering::Camera::Update");
  std::lock_guard<std::mutex> lock(this->dataPtr->receiveMutex);

  // Process all the command messages.
  for (CameraPrivate::CameraCmdMsgs_L::iterator iter =
      this->dataPtr->commandMsgs.begin();
      iter != this->dataPtr->commandMsgs.end(); ++iter)
  {
    if ((*iter)->has_follow_model())
      this->TrackVisual((*iter)->follow_model());
  }
  this->dataPtr->commandMsgs.clear();

  std::list<msgs::Request>::iterator iter = this->requests.begin();
  while (iter != this->requests.end())
  {
    bool erase = false;
    if ((*iter).request() == "track_visual")
    {
      if (this->TrackVisualImpl((*iter).data()))
        erase = true;
    }
    else if ((*iter).request() == "attach_visual")
    {
      msgs::TrackVisual msg;
      msg.ParseFromString((*iter).data());
      bool result = false;

      if (msg.id() < ignition::math::MAX_UI32)
        result = this->AttachToVisualImpl(msg.id(),
            msg.inherit_orientation(), msg.min_dist(), msg.max_dist());
      else
        result = this->AttachToVisualImpl(msg.name(),
            msg.inherit_orientation(), msg.min_dist(), msg.max_dist());

      if (result)
        erase = true;
    }

    if (erase)
      iter = this->requests.erase(iter);
    else
      ++iter;
  }

  // Update animations
  if (this->animState)
  {
    if (this->animState->hasEnded())
    {
      try
      {
        this->scene->OgreSceneManager()->destroyAnimation(
            std::string(this->animState->getAnimationName()));
      } catch(Ogre::Exception &_e)
      {
      }

      this->animState = NULL;

      this->AnimationComplete();

      if (this->onAnimationComplete)
        this->onAnimationComplete();

      if (!this->dataPtr->moveToPositionQueue.empty())
      {
        this->MoveToPosition(this->dataPtr->moveToPositionQueue[0].first,
                             this->dataPtr->moveToPositionQueue[0].second);
        this->dataPtr->moveToPositionQueue.pop_front();
      }
    }
    else
    {
      common::Time wallTime = common::Time::GetWallTime();
      this->animState->addTime((wallTime - this->prevAnimTime).Double());
      this->prevAnimTime = wallTime;
    }
  }
  else if (this->dataPtr->trackedVisual)
  {
    double scaling = 0;
    ignition::math::Vector3d direction =
      this->dataPtr->trackedVisual->WorldPose().Pos() - this->WorldPose().Pos();

    if (!this->dataPtr->trackIsStatic)
    {
      if (direction.Length() < this->dataPtr->trackMinDistance)
        scaling = direction.Length() - this->dataPtr->trackMinDistance;
      else if (direction.Length() > this->dataPtr->trackMaxDistance)
        scaling = direction.Length() - this->dataPtr->trackMaxDistance;
    }
    else
    {
      if (this->dataPtr->trackUseModelFrame)
      {
        if (this->dataPtr->trackInheritYaw)
        {
          double yaw =
              this->dataPtr->trackedVisual->WorldPose().Rot().Yaw();
          ignition::math::Quaterniond rot =
              ignition::math::Quaterniond(0.0, 0.0, yaw);
          direction += rot.RotateVector(this->dataPtr->trackPos);
        }
        else
        {
          direction += this->dataPtr->trackPos;
        }
      }
      else
      {
        direction = this->dataPtr->trackPos - this->WorldPose().Pos();
      }

      scaling = direction.Length();
    }

    ignition::math::Vector3d displacement = direction;
    displacement.Normalize();
    displacement *= scaling;

    ignition::math::Vector3d localPos =
      Conversions::ConvertIgn(this->sceneNode->_getDerivedPosition());
    ignition::math::Vector3d pos = localPos + displacement;

    this->SetWorldPosition(pos);
  }
}

//////////////////////////////////////////////////
void Camera::Render(const bool _force)
{
  IGN_PROFILE("rendering::Camera::Render");
  if (this->initialized && (_force ||
       common::Time::GetWallTime() - this->lastRenderWallTime >=
        this->dataPtr->renderPeriod))
  {
    this->newData = true;
    this->RenderImpl();
  }
}

//////////////////////////////////////////////////
void Camera::RenderImpl()
{
  if (this->renderTarget)
  {
    {
      IGN_PROFILE("rendering::Camera::RenderImpl pre-render");
      Events::cameraPreRender(this->Name());
    }
    {
      IGN_PROFILE("rendering::Camera::RenderImpl update");
      this->renderTarget->update();
    }
    {
      IGN_PROFILE("rendering::Camera::RenderImpl post-render");
      Events::cameraPostRender(this->Name());
    }
  }
}

//////////////////////////////////////////////////
void Camera::ReadPixelBuffer()
{
  if (this->newData && (this->captureData || this->captureDataOnce ||
      this->dataPtr->videoEncoder.IsEncoding()))
  {
    size_t size;
    unsigned int width = this->ImageWidth();
    unsigned int height = this->ImageHeight();

    // Get access to the buffer and make an image and write it to file
    size = Ogre::PixelUtil::getMemorySize(width, height, 1,
        static_cast<Ogre::PixelFormat>(this->imageFormat));

    // Allocate buffer
    if (!this->saveFrameBuffer)
      this->saveFrameBuffer = new unsigned char[size];

    memset(this->saveFrameBuffer, 128, size);

    Ogre::PixelBox box(width, height, 1,
        static_cast<Ogre::PixelFormat>(this->imageFormat),
        this->saveFrameBuffer);

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR < 8
    // Case for UserCamera where there is no RenderTexture but
    // a RenderTarget (RenderWindow) exists. We can not call SetRenderTarget
    // because that overrides the this->renderTarget variable
    if (this->renderTarget && !this->renderTexture)
    {
      // Create the render texture
      this->renderTexture = (Ogre::TextureManager::getSingleton().createManual(
        this->renderTarget->getName() + "_tex",
        "General",
        Ogre::TEX_TYPE_2D,
        this->ImageWidth(),
        this->ImageHeight(),
        0,
        (Ogre::PixelFormat)this->imageFormat,
        Ogre::TU_RENDERTARGET)).getPointer();
        Ogre::RenderTexture *rtt
            = this->renderTexture->getBuffer()->getRenderTarget();

      // Setup the viewport to use the texture
      Ogre::Viewport *vp = rtt->addViewport(this->camera);
      vp->setClearEveryFrame(true);
      vp->setShadowsEnabled(true);
      vp->setOverlaysEnabled(false);
    }

    // This update is only needed for client side data captures
    if (this->renderTexture->getBuffer()->getRenderTarget()
        != this->renderTarget)
      this->renderTexture->getBuffer()->getRenderTarget()->update();

    // The code below is equivalent to
    // this->viewport->getTarget()->copyContentsToMemory(box);
    // which causes problems on some machines if running ogre-1.7.4
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer;
    pixelBuffer = this->renderTexture->getBuffer();
    pixelBuffer->blitToMemory(box);
#else
    // There is a fix in ogre-1.8 for a buffer overrun problem in
    // OgreGLXWindow.cpp's copyContentsToMemory(). It fixes reading
    // pixels from buffer into memory.
    this->viewport->getTarget()->copyContentsToMemory(box);
#endif
  }
}

//////////////////////////////////////////////////
common::Time Camera::LastRenderWallTime() const
{
  return this->lastRenderWallTime;
}

//////////////////////////////////////////////////
void Camera::PostRender()
{
  IGN_PROFILE("rendering::Camera::PostRender");
  this->ReadPixelBuffer();

  // Only record last render time if data was actually generated
  // (If a frame was rendered).
  if (this->newData)
    this->lastRenderWallTime = common::Time::GetWallTime();

  if (this->newData && (this->captureData || this->captureDataOnce ||
      this->dataPtr->videoEncoder.IsEncoding()))
  {
    unsigned int width = this->ImageWidth();
    unsigned int height = this->ImageHeight();
    const unsigned char *buffer = this->saveFrameBuffer;

    if (this->captureDataOnce)
    {
      this->SaveFrame(this->FrameFilename());
      this->captureDataOnce = false;
    }
    else if (this->dataPtr->videoEncoder.IsEncoding())
    {
      this->dataPtr->videoEncoder.AddFrame(buffer, width, height);
    }

    if (this->sdf->HasElement("save") &&
        this->sdf->GetElement("save")->Get<bool>("enabled"))
    {
      this->SaveFrame(this->FrameFilename());
    }

    // do last minute conversion if Bayer pattern is requested, go from R8G8B8
    if ((this->ImageFormat() == "BAYER_RGGB8") ||
         (this->ImageFormat() == "BAYER_BGGR8") ||
         (this->ImageFormat() == "BAYER_GBRG8") ||
         (this->ImageFormat() == "BAYER_GRBG8"))
    {
      if (!this->bayerFrameBuffer)
        this->bayerFrameBuffer = new unsigned char[width * height];

      this->ConvertRGBToBAYER(this->bayerFrameBuffer,
          this->saveFrameBuffer, this->ImageFormat(),
          width, height);

      buffer = this->bayerFrameBuffer;
    }

    this->newImageFrame(buffer, width, height, this->ImageDepth(),
                    this->ImageFormat());
  }

  this->newData = false;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Camera::WorldPosition() const
{
  return Conversions::ConvertIgn(this->sceneNode->_getDerivedPosition());
}

//////////////////////////////////////////////////
ignition::math::Quaterniond Camera::WorldRotation() const
{
  Ogre::Quaternion rot = this->sceneNode->_getDerivedOrientation();
  return ignition::math::Quaterniond(rot.w, rot.x, rot.y, rot.z);
}

//////////////////////////////////////////////////
void Camera::SetWorldPose(const ignition::math::Pose3d &_pose)
{
  this->SetWorldPosition(_pose.Pos());
  this->SetWorldRotation(_pose.Rot());
}

//////////////////////////////////////////////////
ignition::math::Pose3d Camera::WorldPose() const
{
  return ignition::math::Pose3d(this->WorldPosition(), this->WorldRotation());
}

//////////////////////////////////////////////////
void Camera::SetWorldPosition(const ignition::math::Vector3d &_pos)
{
  if (this->animState)
    return;

  this->sceneNode->_setDerivedPosition(Conversions::Convert(_pos));
  this->sceneNode->needUpdate();
}

//////////////////////////////////////////////////
void Camera::SetWorldRotation(const ignition::math::Quaterniond &_quat)
{
  if (this->animState)
    return;

  ignition::math::Vector3d rpy = _quat.Euler();

  // Set the roll and yaw for sceneNode
  ignition::math::Quaterniond s(rpy.X(), rpy.Y(), rpy.Z());

  this->sceneNode->_setDerivedOrientation(Conversions::Convert(s));

  this->sceneNode->needUpdate();
}

//////////////////////////////////////////////////
void Camera::Translate(const ignition::math::Vector3d &_direction)
{
  this->sceneNode->translate(this->sceneNode->getOrientation() *
      Conversions::Convert(_direction));
}

//////////////////////////////////////////////////
void Camera::Roll(const ignition::math::Angle &_angle,
    ReferenceFrame _relativeTo)
{
  this->sceneNode->pitch(Ogre::Radian(_angle.Radian()),
      Conversions::Convert(_relativeTo));
}

//////////////////////////////////////////////////
void Camera::Yaw(const ignition::math::Angle &_angle,
    ReferenceFrame _relativeTo)
{
  this->sceneNode->roll(Ogre::Radian(_angle.Radian()),
      Conversions::Convert(_relativeTo));
}

//////////////////////////////////////////////////
void Camera::Pitch(const ignition::math::Angle &_angle,
    ReferenceFrame _relativeTo)
{
  this->sceneNode->yaw(Ogre::Radian(_angle.Radian()),
      Conversions::Convert(_relativeTo));
}

//////////////////////////////////////////////////
void Camera::SetClipDist()
{
  sdf::ElementPtr clipElem = this->sdf->GetElement("clip");
  if (!clipElem)
    gzthrow("Camera has no <clip> tag.");

  if (this->camera)
  {
    this->camera->setNearClipDistance(clipElem->Get<double>("near"));
    this->camera->setFarClipDistance(clipElem->Get<double>("far"));
    this->camera->setRenderingDistance(clipElem->Get<double>("far"));
  }
  else
    gzerr << "Setting clip distances failed -- no camera yet\n";
}

//////////////////////////////////////////////////
void Camera::SetClipDist(const float _near, const float _far)
{
  sdf::ElementPtr elem = this->sdf->GetElement("clip");

  elem->GetElement("near")->Set(_near);
  elem->GetElement("far")->Set(_far);

  this->SetClipDist();
}

//////////////////////////////////////////////////
void Camera::SetFixedYawAxis(const bool _useFixed,
    const ignition::math::Vector3d &_fixedAxis)
{
  this->cameraNode->setFixedYawAxis(_useFixed,
      Conversions::Convert(_fixedAxis));
  this->dataPtr->yawFixed = _useFixed;
  this->dataPtr->yawFixedAxis = _fixedAxis;
}

//////////////////////////////////////////////////
void Camera::SetHFOV(const ignition::math::Angle &_angle)
{
  this->sdf->GetElement("horizontal_fov")->Set(_angle.Radian());
  this->UpdateFOV();
}

//////////////////////////////////////////////////
ignition::math::Angle Camera::HFOV() const
{
  return ignition::math::Angle(this->sdf->Get<double>("horizontal_fov"));
}

//////////////////////////////////////////////////
ignition::math::Angle Camera::VFOV() const
{
  return ignition::math::Angle(this->camera->getFOVy().valueRadians());
}

//////////////////////////////////////////////////
void Camera::SetImageSize(const unsigned int _w, const unsigned int _h)
{
  this->SetImageWidth(_w);
  this->SetImageHeight(_h);
}

//////////////////////////////////////////////////
void Camera::SetImageWidth(const unsigned int _w)
{
  sdf::ElementPtr elem = this->sdf->GetElement("image");
  elem->GetElement("width")->Set(_w);
}

//////////////////////////////////////////////////
void Camera::SetImageHeight(const unsigned int _h)
{
  sdf::ElementPtr elem = this->sdf->GetElement("image");
  elem->GetElement("height")->Set(_h);
}

//////////////////////////////////////////////////
unsigned int Camera::ImageWidth() const
{
  unsigned int width = 0;
  if (this->viewport)
  {
    width = this->viewport->getActualWidth();
  }
  else
  {
    sdf::ElementPtr elem = this->sdf->GetElement("image");
    width = elem->Get<int>("width");
  }
  return width;
}

//////////////////////////////////////////////////
unsigned int Camera::ImageHeight() const
{
  unsigned int height = 0;
  if (this->viewport)
  {
    height = this->viewport->getActualHeight();
  }
  else
  {
    sdf::ElementPtr elem = this->sdf->GetElement("image");
    height = elem->Get<int>("height");
  }
  return height;
}

//////////////////////////////////////////////////
unsigned int Camera::ImageDepth() const
{
  sdf::ElementPtr imgElem = this->sdf->GetElement("image");
  std::string imgFmt = imgElem->Get<std::string>("format");

  if (imgFmt == "L8" || imgFmt == "L_INT8")
    return 1;
  else if (imgFmt == "L16" || imgFmt == "L_INT16" || imgFmt == "L_UINT16")
    return 2;
  else if (imgFmt == "R8G8B8" || imgFmt == "RGB_INT8")
    return 3;
  else if (imgFmt == "B8G8R8" || imgFmt == "BGR_INT8")
    return 3;
  else if (imgFmt == "R16G16B16" || imgFmt == "RGB_INT16"
      || imgFmt == "RGB_UINT16")
    return 6;
  else if ((imgFmt == "BAYER_RGGB8") || (imgFmt == "BAYER_BGGR8") ||
            (imgFmt == "BAYER_GBRG8") || (imgFmt == "BAYER_GRBG8"))
    return 1;
  else if (imgFmt == "FLOAT32" || imgFmt == "R_FLOAT32")
    return 4;
  else if (imgFmt == "FLOAT16" || imgFmt == "R_FLOAT16")
    return 2;
  else
  {
    gzerr << "Error parsing image format ("
          << imgFmt << "), using default Ogre::PF_R8G8B8\n";
    return 3;
  }
}

//////////////////////////////////////////////////
unsigned int Camera::ImageMemorySize() const
{
  return  Ogre::PixelUtil::getMemorySize(this->ImageWidth(),
      this->ImageHeight(), 1, static_cast<Ogre::PixelFormat>(
      this->imageFormat));
}

//////////////////////////////////////////////////
std::string Camera::ImageFormat() const
{
  sdf::ElementPtr imgElem = this->sdf->GetElement("image");
  return imgElem->Get<std::string>("format");
}

//////////////////////////////////////////////////
unsigned int Camera::TextureWidth() const
{
  return this->renderTexture->getBuffer(0, 0)->getWidth();
}

//////////////////////////////////////////////////
unsigned int Camera::TextureHeight() const
{
  return this->renderTexture->getBuffer(0, 0)->getHeight();
}

//////////////////////////////////////////////////
size_t Camera::ImageByteSize() const
{
  sdf::ElementPtr elem = this->sdf->GetElement("image");
  return this->ImageByteSize(elem->Get<int>("width"),
                             elem->Get<int>("height"),
                             this->ImageFormat());
}

//////////////////////////////////////////////////
size_t Camera::ImageByteSize(const unsigned int _width,
    const unsigned int _height, const std::string &_format)
{
  Ogre::PixelFormat fmt =
    (Ogre::PixelFormat)(Camera::OgrePixelFormat(_format));

  return Ogre::PixelUtil::getMemorySize(_width, _height, 1, fmt);
}

//////////////////////////////////////////////////
int Camera::OgrePixelFormat(const std::string &_format)
{
  int result;

  if (_format == "L8" || _format == "L_INT8")
    result = static_cast<int>(Ogre::PF_L8);
  else if (_format == "L16" || _format == "L_INT16" || _format == "L_UINT16")
    result = static_cast<int>(Ogre::PF_L16);
  else if (_format == "R8G8B8" || _format == "RGB_INT8")
    result = static_cast<int>(Ogre::PF_BYTE_RGB);
  else if (_format == "B8G8R8" || _format == "BGR_INT8")
    result = static_cast<int>(Ogre::PF_BYTE_BGR);
  else if (_format == "FLOAT32" || _format == "R_FLOAT32")
    result = static_cast<int>(Ogre::PF_FLOAT32_R);
  else if (_format == "FLOAT16" || _format == "R_FLOAT16")
    result = static_cast<int>(Ogre::PF_FLOAT16_R);
  else if (_format == "R16G16B16" || _format == "RGB_INT16"
      || _format == "RGB_UINT16")
    result = static_cast<int>(Ogre::PF_SHORT_RGB);
  else if ((_format == "BAYER_RGGB8") ||
            (_format == "BAYER_BGGR8") ||
            (_format == "BAYER_GBRG8") ||
            (_format == "BAYER_GRBG8"))
  {
    // let ogre generate rgb8 images for all bayer format requests
    // then post process to produce actual bayer images
    result = static_cast<int>(Ogre::PF_BYTE_RGB);
  }
  else
  {
    gzerr << "Error parsing image format (" << _format
          << "), using default Ogre::PF_R8G8B8\n";
    result = static_cast<int>(Ogre::PF_R8G8B8);
  }

  return result;
}

//////////////////////////////////////////////////
void Camera::EnableSaveFrame(const bool _enable)
{
  sdf::ElementPtr elem = this->sdf->GetElement("save");
  elem->GetAttribute("enabled")->Set(_enable);
  this->captureData = _enable;
}

//////////////////////////////////////////////////
bool Camera::CaptureData() const
{
  return this->captureData;
}

//////////////////////////////////////////////////
void Camera::SetSaveFramePathname(const std::string &_pathname)
{
  sdf::ElementPtr elem = this->sdf->GetElement("save");
  elem->GetElement("path")->Set(_pathname);

  // Create the directory to store frames
  if (elem->Get<bool>("enabled"))
  {
    std::string command;
    command = "mkdir -p " + _pathname + " 2>>/dev/null";
    if (system(command.c_str()) <0)
      gzerr << "Error making directory\n";
  }
}

//////////////////////////////////////////////////
Ogre::Camera *Camera::OgreCamera() const
{
  return this->camera;
}

//////////////////////////////////////////////////
Ogre::Viewport *Camera::OgreViewport() const
{
  return this->viewport;
}

//////////////////////////////////////////////////
double Camera::NearClip() const
{
  if (this->camera)
    return this->camera->getNearClipDistance();
  else
    return 0;
}

//////////////////////////////////////////////////
double Camera::FarClip() const
{
  if (this->camera)
    return this->camera->getFarClipDistance();
  else
    return 0;
}

//////////////////////////////////////////////////
double Camera::ImageFocalLengthX() const
{
  return this->dataPtr->cameraIntrinsicMatrix(0, 0);
}

//////////////////////////////////////////////////
double Camera::ImageFocalLengthY() const
{
  return this->dataPtr->cameraIntrinsicMatrix(1, 1);
}

//////////////////////////////////////////////////
double Camera::ImageOpticalCenterX() const
{
  return this->dataPtr->cameraIntrinsicMatrix(0, 2);
}

//////////////////////////////////////////////////
double Camera::ImageOpticalCenterY() const
{
  return this->dataPtr->cameraIntrinsicMatrix(1, 2);
}

//////////////////////////////////////////////////
unsigned int Camera::ViewportWidth() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualWidth();
  else if (this->camera && this->camera->getViewport())
    return this->camera->getViewport()->getActualWidth();
  else
    return 0;
}

//////////////////////////////////////////////////
unsigned int Camera::ViewportHeight() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualHeight();
  else if (this->camera && this->camera->getViewport())
    return this->camera->getViewport()->getActualHeight();
  else
    return 0;
}

//////////////////////////////////////////////////
void Camera::SetAspectRatio(const float ratio)
{
  if (!this->cameraUsingIntrinsics)
    this->camera->setAspectRatio(ratio);
}

//////////////////////////////////////////////////
float Camera::AspectRatio() const
{
  return this->camera->getAspectRatio();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Camera::Up() const
{
  Ogre::Vector3 up = this->camera->getRealUp();
  return ignition::math::Vector3d(up.x, up.y, up.z);
}

//////////////////////////////////////////////////
ignition::math::Vector3d Camera::Right() const
{
  Ogre::Vector3 right = this->camera->getRealRight();
  return ignition::math::Vector3d(right.x, right.y, right.z);
}

//////////////////////////////////////////////////
void Camera::SetSceneNode(Ogre::SceneNode *_node)
{
  this->sceneNode = _node;
}

//////////////////////////////////////////////////
Ogre::SceneNode *Camera::SceneNode() const
{
  return this->sceneNode;
}

//////////////////////////////////////////////////
const unsigned char *Camera::ImageData(const unsigned int _i) const
{
  if (_i != 0)
    gzerr << "Camera index must be zero for cam";

  // do last minute conversion if Bayer pattern is requested, go from R8G8B8
  if ((this->ImageFormat() == "BAYER_RGGB8") ||
       (this->ImageFormat() == "BAYER_BGGR8") ||
       (this->ImageFormat() == "BAYER_GBRG8") ||
       (this->ImageFormat() == "BAYER_GRBG8"))
  {
    return this->bayerFrameBuffer;
  }
  else
    return this->saveFrameBuffer;
}

//////////////////////////////////////////////////
std::string Camera::Name() const
{
  return this->name;
}

//////////////////////////////////////////////////
std::string Camera::ScopedName() const
{
  return this->scopedName;
}

//////////////////////////////////////////////////
bool Camera::SaveFrame(const std::string &_filename)
{
  return Camera::SaveFrame(this->saveFrameBuffer, this->ImageWidth(),
                          this->ImageHeight(), this->ImageDepth(),
                          this->ImageFormat(), _filename);
}

//////////////////////////////////////////////////
std::string Camera::FrameFilename()
{
  sdf::ElementPtr saveElem = this->sdf->GetElement("save");

  std::string path = saveElem->Get<std::string>("path");
  boost::filesystem::path pathToFile;

  std::string friendlyName = this->scopedUniqueName;

  boost::replace_all(friendlyName, "::", "_");

  if (this->captureDataOnce)
  {
    pathToFile = this->screenshotPath;
    std::string timestamp = common::Time::GetWallTimeAsISOString();
    boost::replace_all(timestamp, ":", "_");
    pathToFile /= friendlyName + "-" + timestamp + ".jpg";
  }
  else
  {
    pathToFile = (path.empty()) ? "." : path;
    pathToFile /= str(boost::format("%s-%04d.jpg")
        % friendlyName.c_str() % this->saveCount);
    this->saveCount++;
  }

  // Create a directory if not present
  if (!boost::filesystem::exists(pathToFile.parent_path()))
    boost::filesystem::create_directories(pathToFile.parent_path());

  return pathToFile.string();
}

/////////////////////////////////////////////////
std::string Camera::ScreenshotPath() const
{
  return this->screenshotPath;
}

/////////////////////////////////////////////////
bool Camera::SaveFrame(const unsigned char *_image,
                       const unsigned int _width, const unsigned int _height,
                       const int _depth,
                       const std::string &_format,
                       const std::string &_filename)
{
  if (!_image)
  {
    gzerr << "Can't save an empty image\n";
    return false;
  }

  Ogre::ImageCodec::ImageData *imgData;
  Ogre::Codec * pCodec;
  size_t size, pos;

  // Create image data structure
  imgData  = new Ogre::ImageCodec::ImageData();
  imgData->width  =  _width;
  imgData->height = _height;
  imgData->depth  = _depth;
  imgData->format = (Ogre::PixelFormat)Camera::OgrePixelFormat(_format);
  size = Camera::ImageByteSize(_width, _height, _format);

  // Wrap buffer in a chunk
  Ogre::MemoryDataStreamPtr stream(
      new Ogre::MemoryDataStream(const_cast<unsigned char*>(_image),
        size, false));

  // Get codec
  Ogre::String filename = _filename;
  pos = filename.find_last_of(".");
  Ogre::String extension;

  while (pos != filename.length() - 1)
    extension += filename[++pos];

  // Get the codec
  pCodec = Ogre::Codec::getCodec(extension);

  // Write out
  Ogre::Codec::CodecDataPtr codecDataPtr(imgData);

  // OGRE 1.9 renames codeToFile to encodeToFile
  #if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))
  pCodec->codeToFile(stream, filename, codecDataPtr);
  #else
  pCodec->encodeToFile(stream, filename, codecDataPtr);
  #endif
  return true;
}

//////////////////////////////////////////////////
void Camera::ToggleShowWireframe()
{
  if (this->camera)
  {
    if (this->camera->getPolygonMode() == Ogre::PM_WIREFRAME)
      this->camera->setPolygonMode(Ogre::PM_SOLID);
    else
      this->camera->setPolygonMode(Ogre::PM_WIREFRAME);
  }
}

//////////////////////////////////////////////////
void Camera::ShowWireframe(const bool s)
{
  if (this->camera)
  {
    if (s)
    {
      this->camera->setPolygonMode(Ogre::PM_WIREFRAME);
    }
    else
    {
      this->camera->setPolygonMode(Ogre::PM_SOLID);
    }
  }
}

//////////////////////////////////////////////////
void Camera::CameraToViewportRay(const int _screenx, const int _screeny,
    ignition::math::Vector3d &_origin,
    ignition::math::Vector3d &_dir) const
{
  Ogre::Ray ray = this->camera->getCameraToViewportRay(
      static_cast<float>(_screenx) / this->ViewportWidth(),
      static_cast<float>(_screeny) / this->ViewportHeight());

  _origin.Set(ray.getOrigin().x, ray.getOrigin().y, ray.getOrigin().z);
  _dir.Set(ray.getDirection().x, ray.getDirection().y, ray.getDirection().z);
}

//////////////////////////////////////////////////
void Camera::ConvertRGBToBAYER(unsigned char *_dst,
    const unsigned char *_src, const std::string &_format, const int _width,
    const int _height)
{
  if (_src)
  {
    // do last minute conversion if Bayer pattern is requested, go from R8G8B8
    if (_format == "BAYER_RGGB8")
    {
      for (int i = 0; i < _width; i++)
      {
        for (int j = 0; j < _height; j++)
        {
          //
          // RG
          // GB
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, red
              _dst[i+j*_width] = _src[i*3+j*_width*3+2];
            else  // odd row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
          else  // odd column
            if (i%2)  // even row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
            else  // odd row, blue
              _dst[i+j*_width] = _src[i*3+j*_width*3+0];
        }
      }
    }
    else if (_format == "BAYER_BGGR8")
    {
      for (int i = 0; i < _width; i++)
      {
        for (int j = 0; j < _height; j++)
        {
          //
          // BG
          // GR
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, blue
              _dst[i+j*_width] = _src[i*3+j*_width*3+0];
            else  // odd row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
          else  // odd column
            if (i%2)  // even row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
            else  // odd row, red
              _dst[i+j*_width] = _src[i*3+j*_width*3+2];
        }
      }
    }
    else if (_format == "BAYER_GBRG8")
    {
      for (int i = 0; i < _width; i++)
      {
        for (int j = 0; j < _height; j++)
        {
          //
          // GB
          // RG
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
            else  // odd row, blue
              _dst[i+j*_width] = _src[i*3+j*_width*3+2];
          else  // odd column
            if (i%2)  // even row, red
              _dst[i+j*_width] = _src[i*3+j*_width*3+0];
            else  // odd row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
        }
      }
    }
    else if (_format == "BAYER_GRBG8")
    {
      for (int i = 0; i < _width; i++)
      {
        for (int j = 0; j < _height; j++)
        {
          //
          // GR
          // BG
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
            else  // odd row, red
              _dst[i+j*_width] = _src[i*3+j*_width*3+0];
          else  // odd column
            if (i%2)  // even row, blue
              _dst[i+j*_width] = _src[i*3+j*_width*3+2];
            else  // odd row, green
              _dst[i+j*_width] = _src[i*3+j*_width*3+1];
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Camera::SetCaptureData(const bool _value)
{
  this->captureData = _value;
}

//////////////////////////////////////////////////
void Camera::SetCaptureDataOnce()
{
  this->captureDataOnce = true;
}

//////////////////////////////////////////////////
bool Camera::StartVideo(const std::string &_format,
                        const std::string &_filename)
{
  return this->dataPtr->videoEncoder.Start(_format, _filename,
      this->ImageWidth(), this->ImageHeight());
}

//////////////////////////////////////////////////
bool Camera::StopVideo()
{
  return this->dataPtr->videoEncoder.Stop();
}

//////////////////////////////////////////////////
bool Camera::SaveVideo(const std::string &_filename)
{
  // This will stop video encoding, save the video file, and reset
  // video encoding.
  return this->dataPtr->videoEncoder.SaveToFile(_filename);
}

//////////////////////////////////////////////////
bool Camera::ResetVideo()
{
  this->dataPtr->videoEncoder.Reset();
  return true;
}

//////////////////////////////////////////////////
void Camera::CreateRenderTexture(const std::string &_textureName)
{
  unsigned int fsaa = 0;

  std::vector<unsigned int> fsaaLevels =
      RenderEngine::Instance()->FSAALevels();

  // check if target fsaa is supported
  unsigned int targetFSAA = this->dataPtr->antiAliasingValue;
  auto const it = std::find(fsaaLevels.begin(), fsaaLevels.end(), targetFSAA);
  if (it != fsaaLevels.end())
    fsaa = targetFSAA;

  // Full-screen anti-aliasing only works correctly in 1.8 and above
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR < 8
  fsaa = 0;
#endif

  // Create the render texture
  this->renderTexture = (Ogre::TextureManager::getSingleton().createManual(
      _textureName,
      "General",
      Ogre::TEX_TYPE_2D,
      this->ImageWidth(),
      this->ImageHeight(),
      0,
      static_cast<Ogre::PixelFormat>(this->imageFormat),
      Ogre::TU_RENDERTARGET,
      0,
      false,
      fsaa)).getPointer();

  this->SetRenderTarget(this->renderTexture->getBuffer()->getRenderTarget());

  this->initialized = true;
}

//////////////////////////////////////////////////
ScenePtr Camera::GetScene() const
{
  return this->scene;
}

//////////////////////////////////////////////////
void Camera::CreateCamera()
{
  this->camera = this->scene->OgreSceneManager()->createCamera(
      this->scopedUniqueName);
  this->cameraNode = this->sceneNode->createChildSceneNode(
      this->scopedUniqueName + "_cameraNode");
  this->cameraNode->attachObject(this->camera);

  if (this->sdf->HasElement("projection_type"))
    this->SetProjectionType(this->sdf->Get<std::string>("projection_type"));

  this->SetFixedYawAxis(false);
  this->cameraNode->yaw(Ogre::Degree(-90.0));
  this->cameraNode->roll(Ogre::Degree(-90.0));

  if (cameraUsingIntrinsics)
  {
    this->camera->setCustomProjectionMatrix(true,
      Conversions::Convert(this->cameraProjectiveMatrix));
  }
}

//////////////////////////////////////////////////
bool Camera::WorldPointOnPlane(const int _x, const int _y,
    const ignition::math::Planed &_plane,
    ignition::math::Vector3d &_result)
{
  ignition::math::Vector3d origin, dir;
  double dist;

  // Cast two rays from the camera into the world
  this->CameraToViewportRay(_x, _y, origin, dir);

  dist = _plane.Distance(origin, dir);

  _result = origin + dir * dist;

  if (!ignition::math::equal(dist, -1.0))
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
double Camera::LimitFOV(const double _fov)
{
  return std::min(std::max(0.001, _fov), M_PI * 0.999);
}

//////////////////////////////////////////////////
void Camera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  this->renderTarget = _target;

  if (this->renderTarget)
  {
    // Setup the viewport to use the texture
    this->viewport = this->renderTarget->addViewport(this->camera);
    this->viewport->setClearEveryFrame(true);
    this->viewport->setShadowsEnabled(true);
    this->viewport->setOverlaysEnabled(false);

    if (this->camera->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
      this->scene->SetShadowsEnabled(false);

    RTShaderSystem::AttachViewport(this->viewport, this->GetScene());

    auto const &ignBG = this->scene->BackgroundColor();
    this->viewport->setBackgroundColour(Conversions::Convert(ignBG));
    this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    this->UpdateFOV();

    // Setup Deferred rendering for the camera
    if (RenderEngine::Instance()->GetRenderPathType() == RenderEngine::DEFERRED)
    {
      // Deferred shading GBuffer compositor
      this->dataPtr->dsGBufferInstance =
        Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
            "DeferredShading/GBuffer");

      // Deferred lighting GBuffer compositor
      this->dataPtr->dlGBufferInstance =
        Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
            "DeferredLighting/GBuffer");

      // Deferred shading: Merging compositor
      this->dataPtr->dsMergeInstance =
        Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
            "DeferredShading/ShowLit");

      // Deferred lighting: Merging compositor
      this->dataPtr->dlMergeInstance =
        Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
            "DeferredLighting/ShowLit");

      // Screen space ambient occlusion
      // this->dataPtr->this->ssaoInstance =
      //  Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
      //      "DeferredShading/SSAO");

      this->dataPtr->dsGBufferInstance->setEnabled(false);
      this->dataPtr->dsMergeInstance->setEnabled(false);

      this->dataPtr->dlGBufferInstance->setEnabled(true);
      this->dataPtr->dlMergeInstance->setEnabled(true);

      // this->dataPtr->this->ssaoInstance->setEnabled(false);
    }

    if (this->dataPtr->distortion)
    {
      this->dataPtr->distortion->SetCamera(shared_from_this());
      this->renderTarget->update();
    }

    if (this->GetScene()->GetSkyX() != NULL)
      this->renderTarget->addListener(this->GetScene()->GetSkyX());
  }
}

//////////////////////////////////////////////////
void Camera::AttachToVisual(const uint32_t _visualId,
                            const bool _inheritOrientation,
                            const double _minDist, const double _maxDist)
{
  msgs::Request request;
  msgs::TrackVisual track;

  track.set_name(this->Name() + "_attach_to_visual_track");
  track.set_id(_visualId);
  track.set_min_dist(_minDist);
  track.set_max_dist(_maxDist);
  track.set_inherit_orientation(_inheritOrientation);

  std::string *serializedData = request.mutable_data();
  track.SerializeToString(serializedData);

  request.set_request("attach_visual");
  request.set_id(_visualId);
  this->requests.push_back(request);
}

//////////////////////////////////////////////////
void Camera::AttachToVisual(const std::string &_visualName,
                            const bool _inheritOrientation,
                            const double _minDist, const double _maxDist)
{
  msgs::Request request;
  msgs::TrackVisual track;

  VisualPtr visual = this->scene->GetVisual(_visualName);

  if (visual)
    track.set_id(visual->GetId());
  else
  {
    gzerr << "Unable to attach to visual with name[" << _visualName << "]\n";
    track.set_id(ignition::math::MAX_UI32);
  }

  track.set_name(_visualName);
  track.set_min_dist(_minDist);
  track.set_max_dist(_maxDist);
  track.set_inherit_orientation(_inheritOrientation);

  std::string *serializedData = request.mutable_data();
  track.SerializeToString(serializedData);

  request.set_request("attach_visual");
  request.set_id(0);
  this->requests.push_back(request);
}

//////////////////////////////////////////////////
void Camera::TrackVisual(const std::string &_name)
{
  msgs::Request request;
  request.set_request("track_visual");
  request.set_data(_name);
  request.set_id(0);
  this->requests.push_back(request);
}

//////////////////////////////////////////////////
bool Camera::AttachToVisualImpl(const uint32_t _id,
    const bool _inheritOrientation, const double _minDist,
    const double _maxDist)
{
  VisualPtr visual = this->scene->GetVisual(_id);
  return this->AttachToVisualImpl(visual, _inheritOrientation,
                                  _minDist, _maxDist);
}

//////////////////////////////////////////////////
bool Camera::AttachToVisualImpl(const std::string &_name,
    const bool _inheritOrientation, const double _minDist,
    const double _maxDist)
{
  VisualPtr visual = this->scene->GetVisual(_name);
  return this->AttachToVisualImpl(visual, _inheritOrientation,
                                  _minDist, _maxDist);
}

//////////////////////////////////////////////////
bool Camera::AttachToVisualImpl(VisualPtr _visual,
    const bool _inheritOrientation,
    const double /*_minDist*/, const double /*_maxDist*/)
{
  if (this->sceneNode->getParent())
      this->sceneNode->getParent()->removeChild(this->sceneNode);

  if (_visual)
  {
    _visual->GetSceneNode()->addChild(this->sceneNode);
    this->sceneNode->setInheritOrientation(_inheritOrientation);
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
bool Camera::TrackVisualImpl(const std::string &_name)
{
  VisualPtr visual = this->scene->GetVisual(_name);
  if (visual)
    return this->TrackVisualImpl(visual);

  this->TrackVisualImpl(visual);

  if (_name.empty())
    return true;

  return false;
}

//////////////////////////////////////////////////
bool Camera::TrackVisualImpl(VisualPtr _visual)
{
  bool result = false;
  if (_visual)
  {
    this->dataPtr->trackedVisual = _visual;
    this->cameraNode->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    this->cameraNode->setAutoTracking(true, _visual->GetSceneNode());
    result = true;
  }
  else if (this->cameraNode->getAutoTrackTarget() != 0)
  {
    this->cameraNode->setAutoTracking(false);
    this->dataPtr->trackedVisual.reset();
    auto cameraNodeDerivedRot = this->cameraNode->_getDerivedOrientation();
    // reset cameraNode to initial pose
    this->cameraNode->setFixedYawAxis(false);
    this->cameraNode->setOrientation(Ogre::Quaternion::IDENTITY);
    this->cameraNode->yaw(Ogre::Degree(-90.0));
    this->cameraNode->roll(Ogre::Degree(-90.0));
    // compensate for the pose change after resetting cameraNode
    // so that the derived camera orientation remains the same
    auto rot = cameraNodeDerivedRot *
        this->cameraNode->_getDerivedOrientation().Inverse() *
        this->sceneNode->_getDerivedOrientation();
    this->SetWorldPose({this->WorldPosition(), {rot.w, rot.x, rot.y, rot.z}});
    this->cameraNode->setFixedYawAxis(this->dataPtr->yawFixed,
        Conversions::Convert(this->dataPtr->yawFixedAxis));
  }

  return result;
}

//////////////////////////////////////////////////
Ogre::Texture *Camera::RenderTexture() const
{
  return this->renderTexture;
}

/////////////////////////////////////////////////
ignition::math::Vector3d Camera::Direction() const
{
  return Conversions::ConvertIgn(this->camera->getDerivedDirection());
}

/////////////////////////////////////////////////
bool Camera::IsVisible(VisualPtr _visual)
{
  if (this->camera && _visual)
  {
    ignition::math::AxisAlignedBox bbox = _visual->BoundingBox();
    Ogre::AxisAlignedBox box;
    box.setMinimum(bbox.Min().X(), bbox.Min().Y(), bbox.Min().Z());
    box.setMaximum(bbox.Max().X(), bbox.Max().Y(), bbox.Max().Z());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 11
    box.transform(_visual->GetSceneNode()->_getFullTransform());
#else
    box.transformAffine(_visual->GetSceneNode()->_getFullTransform());
#endif

    // update cam node to ensure transform is update-to-date
    this->cameraNode->_update(false, true);

    return this->camera->isVisible(box);
  }

  return false;
}

/////////////////////////////////////////////////
bool Camera::IsVisible(const std::string &_visualName)
{
  return this->IsVisible(this->scene->GetVisual(_visualName));
}

/////////////////////////////////////////////////
bool Camera::IsAnimating() const
{
  return this->animState != NULL;
}

/////////////////////////////////////////////////
bool Camera::MoveToPosition(const ignition::math::Pose3d &_pose,
    const double _time)
{
  if (this->animState)
  {
    this->dataPtr->moveToPositionQueue.push_back(std::make_pair(_pose, _time));
    return false;
  }

  Ogre::TransformKeyFrame *key;
  ignition::math::Vector3d rpy = _pose.Rot().Euler();
  ignition::math::Vector3d start = this->WorldPose().Pos();

  Ogre::Quaternion localRotOgre = this->sceneNode->getOrientation();
  ignition::math::Quaterniond localRot = ignition::math::Quaterniond(
    localRotOgre.w, localRotOgre.x, localRotOgre.y, localRotOgre.z);
  double dyaw =  localRot.Euler().Z() - rpy.Z();

  if (dyaw > M_PI)
    rpy.Z() += 2*M_PI;
  else if (dyaw < -M_PI)
    rpy.Z() -= 2*M_PI;

  ignition::math::Quaterniond pitchYawOnly(0, rpy.Y(), rpy.Z());
  Ogre::Quaternion pitchYawFinal(pitchYawOnly.W(), pitchYawOnly.X(),
    pitchYawOnly.Y(), pitchYawOnly.Z());

  std::string trackName = "cameratrack";
  int i = 0;
  while (this->scene->OgreSceneManager()->hasAnimation(trackName))
  {
    trackName = std::string("cameratrack_") +
      boost::lexical_cast<std::string>(i);
    i++;
  }

  Ogre::Animation *anim =
    this->scene->OgreSceneManager()->createAnimation(trackName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.X(), start.Y(), start.Z()));
  key->setRotation(this->sceneNode->getOrientation());

  key = strack->createNodeKeyFrame(_time);
  key->setTranslate(Ogre::Vector3(_pose.Pos().X(), _pose.Pos().Y(),
        _pose.Pos().Z()));
  key->setRotation(pitchYawFinal);

  this->animState =
    this->scene->OgreSceneManager()->createAnimationState(trackName);

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();

  return true;
}

/////////////////////////////////////////////////
bool Camera::MoveToPositions(const std::vector<ignition::math::Pose3d> &_pts,
                             const double _time,
                             std::function<void()> _onComplete)
{
  if (this->animState)
    return false;

  this->onAnimationComplete = _onComplete;

  Ogre::TransformKeyFrame *key;
  ignition::math::Vector3d start = this->WorldPose().Pos();

  std::string trackName = "cameratrack";
  int i = 0;
  while (this->scene->OgreSceneManager()->hasAnimation(trackName))
  {
    trackName = std::string("cameratrack_") +
      boost::lexical_cast<std::string>(i);
    i++;
  }

  Ogre::Animation *anim =
    this->scene->OgreSceneManager()->createAnimation(trackName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.X(), start.Y(), start.Z()));
  key->setRotation(this->sceneNode->getOrientation());

  double dt = _time / (_pts.size()-1);
  double tt = 0;

  Ogre::Quaternion localRotOgre = this->sceneNode->getOrientation();
  ignition::math::Quaterniond localRot = ignition::math::Quaterniond(
    localRotOgre.w, localRotOgre.x, localRotOgre.y, localRotOgre.z);
  double prevYaw = localRot.Euler().Z();
  for (unsigned int j = 0; j < _pts.size(); j++)
  {
    ignition::math::Vector3d pos = _pts[j].Pos();
    ignition::math::Vector3d rpy = _pts[j].Rot().Euler();
    double dyaw = prevYaw - rpy.Z();

    if (dyaw > M_PI)
      rpy.Z() += 2*M_PI;
    else if (dyaw < -M_PI)
      rpy.Z() -= 2*M_PI;

    prevYaw = rpy.Z();

    ignition::math::Quaterniond pitchYawOnly(0, rpy.Y(), rpy.Z());
    Ogre::Quaternion pitchYawFinal(pitchYawOnly.W(), pitchYawOnly.X(),
      pitchYawOnly.Y(), pitchYawOnly.Z());

    key = strack->createNodeKeyFrame(tt);
    key->setTranslate(Ogre::Vector3(pos.X(), pos.Y(), pos.Z()));
    key->setRotation(pitchYawFinal);

    tt += dt;
  }

  this->animState =
      this->scene->OgreSceneManager()->createAnimationState(trackName);

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();

  return true;
}

//////////////////////////////////////////////////
void Camera::SetRenderRate(const double _hz)
{
  if (_hz > 0.0)
    this->dataPtr->renderPeriod = 1.0 / _hz;
  else
    this->dataPtr->renderPeriod = 0.0;
}

//////////////////////////////////////////////////
double Camera::RenderRate() const
{
  return 1.0 / this->dataPtr->renderPeriod.Double();
}

//////////////////////////////////////////////////
void Camera::AnimationComplete()
{
}

//////////////////////////////////////////////////
bool Camera::Initialized() const
{
  return this->initialized && this->scene->Initialized();
}

//////////////////////////////////////////////////
void Camera::OnCmdMsg(ConstCameraCmdPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->receiveMutex);
  this->dataPtr->commandMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
DistortionPtr Camera::LensDistortion() const
{
  return this->dataPtr->distortion;
}

//////////////////////////////////////////////////
void Camera::UpdateFOV()
{
  if (this->viewport)
  {
    this->viewport->setDimensions(0, 0, 1, 1);
    double ratio = static_cast<double>(this->viewport->getActualWidth()) /
      static_cast<double>(this->viewport->getActualHeight());

    double hfov = this->HFOV().Radian();
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
    if (this->cameraUsingIntrinsics)
    {
        this->camera->setCustomProjectionMatrix(true,
          Conversions::Convert(this->cameraProjectiveMatrix));
    }
    else
    {
      this->camera->setAspectRatio(ratio);
      this->camera->setFOVy(Ogre::Radian(this->LimitFOV(vfov)));
      this->CalculateIntrinsicsFromProjectionMatrix();
    }
  }
}

//////////////////////////////////////////////////
float Camera::AvgFPS() const
{
  if (this->renderTarget)
  {
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 11
    return this->renderTarget->getStatistics().avgFPS;
#else
    return this->renderTarget->getAverageFPS();
#endif
  }
  else
  {
    return 0.0f;
  }
}

//////////////////////////////////////////////////
unsigned int Camera::TriangleCount() const
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 11
  return this->renderTarget->getStatistics().triangleCount;
#else
  return this->renderTarget->getTriangleCount();
#endif
}

//////////////////////////////////////////////////
bool Camera::SetProjectionType(const std::string &_type)
{
  bool result = true;

  if (_type == "orthographic")
  {
    // Shadows do not work properly with orthographic projection
    this->scene->SetShadowsEnabled(false);
    this->camera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  }
  else if (_type == "perspective")
  {
    this->camera->setProjectionType(Ogre::PT_PERSPECTIVE);
    this->camera->setCustomProjectionMatrix(false);
    this->scene->SetShadowsEnabled(true);
  }
  else
  {
    gzerr << "Invalid projection type[" << _type << "]. "
      << "Valid values are 'perspective' and 'orthographic'.\n";
    result = false;
  }

  return result;
}

//////////////////////////////////////////////////
std::string Camera::ProjectionType() const
{
  if (this->camera->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
  {
    return "orthographic";
  }
  // There are only two types of projection in OGRE.
  else
  {
    return "perspective";
  }
}

//////////////////////////////////////////////////
bool Camera::SetBackgroundColor(const ignition::math::Color &_color)
{
  if (this->OgreViewport())
  {
    this->OgreViewport()->setBackgroundColour(Conversions::Convert(_color));

    // refresh distortion to prevent improper compositor initialization
    // https://github.com/osrf/gazebo/pull/3033
    if (this->dataPtr->distortion)
    {
      this->dataPtr->distortion->RefreshCompositor(shared_from_this());
      this->renderTarget->update();
    }

    return true;
  }
  return false;
}

//////////////////////////////////////////////////
ignition::math::Matrix4d Camera::ProjectionMatrix() const
{
  return Conversions::ConvertIgn(this->camera->getProjectionMatrix());
}

///////////////////////////////////////////////
void Camera::CalculateIntrinsicsFromProjectionMatrix()
{
  const ignition::math::Matrix4d& projectionMat = this->ProjectionMatrix();

  const double width = this->imageWidth;
  const double height = this->imageHeight;

  double fX = (projectionMat(0, 0) * width) / 2;
  double fY = (projectionMat(1, 1) * height) / 2;
  double cX = -(width * (projectionMat(0, 2) - 1.0)) / 2;
  double cY = height + (height * (projectionMat(1, 2) - 1.0)) / 2;

  this->dataPtr->cameraIntrinsicMatrix = this->BuildIntrinsicMatrix(
      fX, fY, cX, cY);
}

//////////////////////////////////////////////////
event::ConnectionPtr Camera::ConnectNewImageFrame(
    std::function<void (const unsigned char *, unsigned int, unsigned int,
    unsigned int, const std::string &)> _subscriber)
{
  return this->newImageFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
VisualPtr Camera::TrackedVisual() const
{
  return this->dataPtr->trackedVisual;
}

/////////////////////////////////////////////////
bool Camera::TrackIsStatic() const
{
  return this->dataPtr->trackIsStatic;
}

/////////////////////////////////////////////////
void Camera::SetTrackIsStatic(const bool _isStatic)
{
  this->dataPtr->trackIsStatic = _isStatic;
}

/////////////////////////////////////////////////
bool Camera::TrackUseModelFrame() const
{
  return this->dataPtr->trackUseModelFrame;
}

/////////////////////////////////////////////////
void Camera::SetTrackUseModelFrame(const bool _useModelFrame)
{
  this->dataPtr->trackUseModelFrame = _useModelFrame;
}

/////////////////////////////////////////////////
ignition::math::Vector3d Camera::TrackPosition() const
{
  return this->dataPtr->trackPos;
}

/////////////////////////////////////////////////
void Camera::SetTrackPosition(const ignition::math::Vector3d &_pos)
{
  this->dataPtr->trackPos = _pos;
}

/////////////////////////////////////////////////
double Camera::TrackMinDistance() const
{
  return this->dataPtr->trackMinDistance;
}

/////////////////////////////////////////////////
double Camera::TrackMaxDistance() const
{
  return this->dataPtr->trackMaxDistance;
}

/////////////////////////////////////////////////
void Camera::SetTrackMinDistance(const double _dist)
{
  this->dataPtr->trackMinDistance = _dist;
}

/////////////////////////////////////////////////
void Camera::SetTrackMaxDistance(const double _dist)
{
  this->dataPtr->trackMaxDistance = _dist;
}

/////////////////////////////////////////////////
bool Camera::TrackInheritYaw() const
{
  return this->dataPtr->trackInheritYaw;
}

/////////////////////////////////////////////////
void Camera::SetTrackInheritYaw(const bool _inheritYaw)
{
  this->dataPtr->trackInheritYaw = _inheritYaw;
}

/////////////////////////////////////////////////
ignition::math::Vector2i Camera::Project(
    const ignition::math::Vector3d &_pt) const
{
  // Convert from 3D world pos to 2D screen pos
  Ogre::Vector3 pos = this->OgreCamera()->getProjectionMatrix() *
      this->OgreCamera()->getViewMatrix() * Conversions::Convert(_pt);

  ignition::math::Vector2i screenPos;
  screenPos.X() = ((pos.x / 2.0) + 0.5) * this->ViewportWidth();
  screenPos.Y() = (1 - ((pos.y / 2.0) + 0.5)) * this->ViewportHeight();

  return screenPos;
}
