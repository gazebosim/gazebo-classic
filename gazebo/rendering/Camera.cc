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

#include <dirent.h>
#include <sstream>
#include <boost/filesystem.hpp>
#include <sdf/sdf.hh>

// Moved to top to avoid osx compilation errors
#include "gazebo/math/Rand.hh"

#include "gazebo/rendering/skyx/include/SkyX.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Distortion.hh"
#include "gazebo/rendering/CameraPrivate.hh"
#include "gazebo/rendering/Camera.hh"

using namespace gazebo;
using namespace rendering;


unsigned int CameraPrivate::cameraCounter = 0;

//////////////////////////////////////////////////
Camera::Camera(const std::string &_name, ScenePtr _scene,
               bool _autoRender)
  : dataPtr(new CameraPrivate)
{
  this->initialized = false;
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
  this->scopedName = this->scene->GetName() + "::" + _name;
  this->scopedUniqueName = this->scopedName + "(" +
    boost::lexical_cast<std::string>(++this->dataPtr->cameraCounter) + ")";

  this->renderTarget = NULL;
  this->renderTexture = NULL;

  this->captureData = false;
  this->captureDataOnce = false;

  this->camera = NULL;
  this->viewport = NULL;

  this->sceneNode = NULL;

  this->screenshotPath = getenv("HOME");
  this->screenshotPath += "/.gazebo/pictures";

  // Connect to the render signal
  this->connections.push_back(
      event::Events::ConnectPostRender(boost::bind(&Camera::Update, this)));

  if (_autoRender)
  {
    this->connections.push_back(event::Events::ConnectRender(
          boost::bind(&Camera::Render, this, false)));
    this->connections.push_back(
        event::Events::ConnectPostRender(
          boost::bind(&Camera::PostRender, this)));
  }

  this->lastRenderWallTime = common::Time::GetWallTime();

  // Set default render rate to unlimited
  this->SetRenderRate(0.0);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
}

//////////////////////////////////////////////////
Camera::~Camera()
{
  delete [] this->saveFrameBuffer;
  delete [] this->bayerFrameBuffer;

  this->sceneNode = NULL;

  if (this->renderTexture && this->scene->GetInitialized())
    Ogre::TextureManager::getSingleton().remove(this->renderTexture->getName());
  this->renderTexture = NULL;
  this->renderTarget = NULL;

  if (this->camera && this->scene && this->scene->GetManager())
  {
    this->scene->GetManager()->destroyCamera(this->scopedUniqueName);
    this->camera = NULL;
  }

  this->connections.clear();

  this->sdf->Reset();
  this->sdf.reset();

  delete this->dataPtr;
  this->dataPtr = NULL;
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
    this->imageFormat = this->GetOgrePixelFormat(
        imgElem->Get<std::string>("format"));
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
    double angle = elem->Get<double>();
    if (angle < 0.01 || angle > M_PI)
    {
      gzthrow("Camera horizontal field of view invalid.");
    }
    this->SetHFOV(angle);
  }

  // Only create a command subscription for real cameras. Ignore camera's
  // created for visualization purposes.
  if (this->name.find("_GUIONLY_") == std::string::npos)
  {
    this->dataPtr->cmdSub = this->dataPtr->node->Subscribe(
        "~/" + this->GetName() + "/cmd", &Camera::OnCmdMsg, this, true);
  }

  if (this->sdf->HasElement("distortion"))
  {
    this->dataPtr->distortion.reset(new Distortion());
    this->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));
  }
}

//////////////////////////////////////////////////
void Camera::Init()
{
  this->SetSceneNode(
      this->scene->GetManager()->getRootSceneNode()->createChildSceneNode(
        this->scopedUniqueName + "_SceneNode"));

  this->CreateCamera();

  this->sceneNode->attachObject(this->camera);
  this->camera->setAutoAspectRatio(true);

  this->sceneNode->setInheritScale(false);

  this->saveCount = 0;

  this->SetClipDist();
}

//////////////////////////////////////////////////
void Camera::Fini()
{
  this->initialized = false;
  this->connections.clear();
  this->dataPtr->node.reset();

  RTShaderSystem::DetachViewport(this->viewport, this->scene);

  if (this->renderTarget && this->scene->GetInitialized())
    this->renderTarget->removeAllViewports();

  this->viewport = NULL;
  this->renderTarget = NULL;

  this->connections.clear();
}

//////////////////////////////////////////////////
void Camera::SetWindowId(unsigned int windowId_)
{
  this->windowId = windowId_;
}

//////////////////////////////////////////////////
unsigned int Camera::GetWindowId() const
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
  boost::mutex::scoped_lock lock(this->dataPtr->receiveMutex);

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

      if (msg.id() < GZ_UINT32_MAX)
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
    this->animState->addTime(
        (common::Time::GetWallTime() - this->prevAnimTime).Double());
    this->prevAnimTime = common::Time::GetWallTime();

    if (this->animState->hasEnded())
    {
      try
      {
        this->scene->GetManager()->destroyAnimation(
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
  }
  else if (this->dataPtr->trackedVisual)
  {
    math::Vector3 direction = this->dataPtr->trackedVisual->GetWorldPose().pos -
                              this->GetWorldPose().pos;

    double yaw = atan2(direction.y, direction.x);
    double pitch = atan2(-direction.z,
                         sqrt(pow(direction.x, 2) + pow(direction.y, 2)));

    Ogre::Quaternion localRotOgre = this->sceneNode->getOrientation();
    math::Quaternion localRot = math::Quaternion(
      localRotOgre.w, localRotOgre.x, localRotOgre.y, localRotOgre.z);
    double currPitch = localRot.GetAsEuler().y;
    double currYaw = localRot.GetAsEuler().z;

    double pitchError = currPitch - pitch;

    double yawError = currYaw - yaw;
    if (yawError > M_PI)
      yawError -= M_PI*2;
    if (yawError < -M_PI)
      yawError += M_PI*2;

    double pitchAdj = this->dataPtr->trackVisualPitchPID.Update(
        pitchError, 0.01);
    double yawAdj = this->dataPtr->trackVisualYawPID.Update(
        yawError, 0.01);

    this->SetWorldRotation(math::Quaternion(0, currPitch + pitchAdj,
          currYaw + yawAdj));

    double origDistance = 8.0;
    double distance = direction.GetLength();
    double error = origDistance - distance;

    double scaling = this->dataPtr->trackVisualPID.Update(error, 0.3);

    math::Vector3 displacement = direction;
    displacement.Normalize();
    displacement *= scaling;

    math::Vector3 localPos =
      Conversions::Convert(this->sceneNode->_getDerivedPosition());
    math::Vector3 pos = localPos + displacement;

    this->SetWorldPosition(pos);
  }
}

//////////////////////////////////////////////////
void Camera::Render(bool _force)
{
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
    this->renderTarget->update();
  }
}

//////////////////////////////////////////////////
void Camera::ReadPixelBuffer()
{
  if (this->newData && (this->captureData || this->captureDataOnce))
  {
    size_t size;
    unsigned int width = this->GetImageWidth();
    unsigned int height = this->GetImageHeight();

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
        this->GetImageWidth(),
        this->GetImageHeight(),
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
common::Time Camera::GetLastRenderWallTime()
{
  return this->lastRenderWallTime;
}

//////////////////////////////////////////////////
void Camera::PostRender()
{
  this->ReadPixelBuffer();

  this->lastRenderWallTime = common::Time::GetWallTime();

  if (this->newData && (this->captureData || this->captureDataOnce))
  {
    if (this->captureDataOnce)
    {
      this->SaveFrame(this->GetFrameFilename());
      this->captureDataOnce = false;
    }

    if (this->sdf->HasElement("save") &&
        this->sdf->GetElement("save")->Get<bool>("enabled"))
    {
      this->SaveFrame(this->GetFrameFilename());
    }

    unsigned int width = this->GetImageWidth();
    unsigned int height = this->GetImageHeight();
    const unsigned char *buffer = this->saveFrameBuffer;

    // do last minute conversion if Bayer pattern is requested, go from R8G8B8
    if ((this->GetImageFormat() == "BAYER_RGGB8") ||
         (this->GetImageFormat() == "BAYER_BGGR8") ||
         (this->GetImageFormat() == "BAYER_GBRG8") ||
         (this->GetImageFormat() == "BAYER_GRBG8"))
    {
      if (!this->bayerFrameBuffer)
        this->bayerFrameBuffer = new unsigned char[width * height];

      this->ConvertRGBToBAYER(this->bayerFrameBuffer,
          this->saveFrameBuffer, this->GetImageFormat(),
          width, height);

      buffer = this->bayerFrameBuffer;
    }

    this->newImageFrame(buffer, width, height, this->GetImageDepth(),
                    this->GetImageFormat());
  }

  this->newData = false;
}

//////////////////////////////////////////////////
math::Vector3 Camera::GetWorldPosition() const
{
  return Conversions::Convert(this->sceneNode->_getDerivedPosition());
}

//////////////////////////////////////////////////
math::Quaternion Camera::GetWorldRotation() const
{
  Ogre::Quaternion rot = this->sceneNode->_getDerivedOrientation();
  return math::Quaternion(rot.w, rot.x, rot.y, rot.z);
}

//////////////////////////////////////////////////
void Camera::SetWorldPose(const math::Pose &_pose)
{
  this->SetWorldPosition(_pose.pos);
  this->SetWorldRotation(_pose.rot);
}

//////////////////////////////////////////////////
math::Pose Camera::GetWorldPose() const
{
  return math::Pose(this->GetWorldPosition(), this->GetWorldRotation());
}

//////////////////////////////////////////////////
void Camera::SetWorldPosition(const math::Vector3 &_pos)
{
  if (this->animState)
    return;

  this->sceneNode->_setDerivedPosition(Ogre::Vector3(_pos.x, _pos.y, _pos.z));
  this->sceneNode->needUpdate();
}

//////////////////////////////////////////////////
void Camera::SetWorldRotation(const math::Quaternion &_quant)
{
  if (this->animState)
    return;

  math::Vector3 rpy = _quant.GetAsEuler();

  // Set the roll and yaw for sceneNode
  math::Quaternion s(rpy.x, rpy.y, rpy.z);

  this->sceneNode->_setDerivedOrientation(Ogre::Quaternion(s.w, s.x, s.y, s.z));

  this->sceneNode->needUpdate();
}

//////////////////////////////////////////////////
void Camera::Translate(const math::Vector3 &_direction)
{
  this->sceneNode->translate(this->sceneNode->getOrientation() *
      Conversions::Convert(_direction));
}

//////////////////////////////////////////////////
void Camera::Roll(const math::Angle &_angle,
    Ogre::Node::TransformSpace _relativeTo)
{
  this->sceneNode->pitch(Ogre::Radian(_angle.Radian()), _relativeTo);
}

//////////////////////////////////////////////////
void Camera::Yaw(const math::Angle &_angle,
    Ogre::Node::TransformSpace _relativeTo)
{
  this->sceneNode->roll(Ogre::Radian(_angle.Radian()), _relativeTo);
}

//////////////////////////////////////////////////
void Camera::Pitch(const math::Angle &_angle,
    Ogre::Node::TransformSpace _relativeTo)
{
  this->sceneNode->yaw(Ogre::Radian(_angle.Radian()), _relativeTo);
}

//////////////////////////////////////////////////
void Camera::RotateYaw(math::Angle _angle)
{
  this->Yaw(_angle);
}

//////////////////////////////////////////////////
void Camera::RotatePitch(math::Angle _angle)
{
  this->Pitch(_angle);
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
void Camera::SetClipDist(float _near, float _far)
{
  sdf::ElementPtr elem = this->sdf->GetElement("clip");

  elem->GetElement("near")->Set(_near);
  elem->GetElement("far")->Set(_far);

  this->SetClipDist();
}

//////////////////////////////////////////////////
void Camera::SetHFOV(math::Angle _angle)
{
  this->sdf->GetElement("horizontal_fov")->Set(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle Camera::GetHFOV() const
{
  return math::Angle(this->sdf->Get<double>("horizontal_fov"));
}

//////////////////////////////////////////////////
math::Angle Camera::GetVFOV() const
{
  return math::Angle(this->camera->getFOVy().valueRadians());
}

//////////////////////////////////////////////////
void Camera::SetImageSize(unsigned int _w, unsigned int _h)
{
  this->SetImageWidth(_w);
  this->SetImageHeight(_h);
}

//////////////////////////////////////////////////
void Camera::SetImageWidth(unsigned int _w)
{
  sdf::ElementPtr elem = this->sdf->GetElement("image");
  elem->GetElement("width")->Set(_w);
}

//////////////////////////////////////////////////
void Camera::SetImageHeight(unsigned int _h)
{
  sdf::ElementPtr elem = this->sdf->GetElement("image");
  elem->GetElement("height")->Set(_h);
}

//////////////////////////////////////////////////
unsigned int Camera::GetImageWidth() const
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
unsigned int Camera::GetImageHeight() const
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
unsigned int Camera::GetImageDepth() const
{
  sdf::ElementPtr imgElem = this->sdf->GetElement("image");
  std::string imgFmt = imgElem->Get<std::string>("format");

  if (imgFmt == "L8" || imgFmt == "L_INT8")
    return 1;
  else if (imgFmt == "R8G8B8" || imgFmt == "RGB_INT8")
    return 3;
  else if (imgFmt == "B8G8R8" || imgFmt == "BGR_INT8")
    return 3;
  else if ((imgFmt == "BAYER_RGGB8") || (imgFmt == "BAYER_BGGR8") ||
            (imgFmt == "BAYER_GBRG8") || (imgFmt == "BAYER_GRBG8"))
    return 1;
  else
  {
    gzerr << "Error parsing image format ("
          << imgFmt << "), using default Ogre::PF_R8G8B8\n";
    return 3;
  }
}

//////////////////////////////////////////////////
std::string Camera::GetImageFormat() const
{
  sdf::ElementPtr imgElem = this->sdf->GetElement("image");
  return imgElem->Get<std::string>("format");
}

//////////////////////////////////////////////////
unsigned int Camera::GetTextureWidth() const
{
  return this->renderTexture->getBuffer(0, 0)->getWidth();
}

//////////////////////////////////////////////////
unsigned int Camera::GetTextureHeight() const
{
  return this->renderTexture->getBuffer(0, 0)->getHeight();
}


//////////////////////////////////////////////////
size_t Camera::GetImageByteSize() const
{
  sdf::ElementPtr elem = this->sdf->GetElement("image");
  return this->GetImageByteSize(elem->Get<int>("width"),
                                elem->Get<int>("height"),
                                this->GetImageFormat());
}

// Get the image size in bytes
size_t Camera::GetImageByteSize(unsigned int _width, unsigned int _height,
                                const std::string &_format)
{
  Ogre::PixelFormat fmt =
    (Ogre::PixelFormat)(Camera::GetOgrePixelFormat(_format));

  return Ogre::PixelUtil::getMemorySize(_width, _height, 1, fmt);
}

int Camera::GetOgrePixelFormat(const std::string &_format)
{
  int result;

  if (_format == "L8" || _format == "L_INT8")
    result = static_cast<int>(Ogre::PF_L8);
  else if (_format == "R8G8B8" || _format == "RGB_INT8")
    result = static_cast<int>(Ogre::PF_BYTE_RGB);
  else if (_format == "B8G8R8" || _format == "BGR_INT8")
    result = static_cast<int>(Ogre::PF_BYTE_BGR);
  else if (_format == "FLOAT32")
    result = static_cast<int>(Ogre::PF_FLOAT32_R);
  else if (_format == "FLOAT16")
    result = static_cast<int>(Ogre::PF_FLOAT16_R);
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
void Camera::EnableSaveFrame(bool _enable)
{
  sdf::ElementPtr elem = this->sdf->GetElement("save");
  elem->GetAttribute("enabled")->Set(_enable);
  this->captureData = _enable;
}

//////////////////////////////////////////////////
bool Camera::GetCaptureData() const
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
Ogre::Camera *Camera::GetOgreCamera() const
{
  return this->camera;
}

//////////////////////////////////////////////////
Ogre::Viewport *Camera::GetViewport() const
{
  return this->viewport;
}

//////////////////////////////////////////////////
double Camera::GetNearClip()
{
  if (this->camera)
    return this->camera->getNearClipDistance();
  else
    return 0;
}

//////////////////////////////////////////////////
double Camera::GetFarClip()
{
  if (this->camera)
    return this->camera->getFarClipDistance();
  else
    return 0;
}

//////////////////////////////////////////////////
unsigned int Camera::GetViewportWidth() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualWidth();
  else if (this->camera && this->camera->getViewport())
    return this->camera->getViewport()->getActualWidth();
  else
    return 0;
}

//////////////////////////////////////////////////
unsigned int Camera::GetViewportHeight() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualHeight();
  else if (this->camera && this->camera->getViewport())
    return this->camera->getViewport()->getActualHeight();
  else
    return 0;
}

//////////////////////////////////////////////////
void Camera::SetAspectRatio(float ratio)
{
  this->camera->setAspectRatio(ratio);
}

//////////////////////////////////////////////////
float Camera::GetAspectRatio() const
{
  return this->camera->getAspectRatio();
}

//////////////////////////////////////////////////
math::Vector3 Camera::GetUp()
{
  Ogre::Vector3 up = this->camera->getRealUp();
  return math::Vector3(up.x, up.y, up.z);
}

//////////////////////////////////////////////////
math::Vector3 Camera::GetRight()
{
  Ogre::Vector3 right = this->camera->getRealRight();
  return math::Vector3(right.x, right.y, right.z);
}

//////////////////////////////////////////////////
void Camera::SetSceneNode(Ogre::SceneNode *_node)
{
  this->sceneNode = _node;
}

//////////////////////////////////////////////////
Ogre::SceneNode *Camera::GetSceneNode() const
{
  return this->sceneNode;
}

//////////////////////////////////////////////////
const unsigned char *Camera::GetImageData(unsigned int _i)
{
  if (_i != 0)
    gzerr << "Camera index must be zero for cam";

  // do last minute conversion if Bayer pattern is requested, go from R8G8B8
  if ((this->GetImageFormat() == "BAYER_RGGB8") ||
       (this->GetImageFormat() == "BAYER_BGGR8") ||
       (this->GetImageFormat() == "BAYER_GBRG8") ||
       (this->GetImageFormat() == "BAYER_GRBG8"))
  {
    return this->bayerFrameBuffer;
  }
  else
    return this->saveFrameBuffer;
}

//////////////////////////////////////////////////
std::string Camera::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
std::string Camera::GetScopedName() const
{
  return this->scopedName;
}

//////////////////////////////////////////////////
bool Camera::SaveFrame(const std::string &_filename)
{
  return Camera::SaveFrame(this->saveFrameBuffer, this->GetImageWidth(),
                          this->GetImageHeight(), this->GetImageDepth(),
                          this->GetImageFormat(), _filename);
}

//////////////////////////////////////////////////
std::string Camera::GetFrameFilename()
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
std::string Camera::GetScreenshotPath() const
{
  return this->screenshotPath;
}

/////////////////////////////////////////////////
bool Camera::SaveFrame(const unsigned char *_image,
                       unsigned int _width, unsigned int _height, int _depth,
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
  imgData->format = (Ogre::PixelFormat)Camera::GetOgrePixelFormat(_format);
  size = Camera::GetImageByteSize(_width, _height, _format);

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
void Camera::ShowWireframe(bool s)
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
void Camera::GetCameraToViewportRay(int _screenx, int _screeny,
                                    math::Vector3 &_origin,
                                    math::Vector3 &_dir)
{
  Ogre::Ray ray = this->camera->getCameraToViewportRay(
      static_cast<float>(_screenx) / this->GetViewportWidth(),
      static_cast<float>(_screeny) / this->GetViewportHeight());

  _origin.Set(ray.getOrigin().x, ray.getOrigin().y, ray.getOrigin().z);
  _dir.Set(ray.getDirection().x, ray.getDirection().y, ray.getDirection().z);
}


//////////////////////////////////////////////////
void Camera::ConvertRGBToBAYER(unsigned char* dst, unsigned char* src,
                               std::string format, int width, int height)
{
  if (src)
  {
    // do last minute conversion if Bayer pattern is requested, go from R8G8B8
    if (format == "BAYER_RGGB8")
    {
      for (int i = 0; i < width; i++)
      {
        for (int j = 0; j < height; j++)
        {
          //
          // RG
          // GB
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, red
              dst[i+j*width] = src[i*3+j*width*3+2];
            else  // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
          else  // odd column
            if (i%2)  // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else  // odd row, blue
              dst[i+j*width] = src[i*3+j*width*3+0];
        }
      }
    }
    else if (format == "BAYER_BGGR8")
    {
      for (int i = 0; i < width; i++)
      {
        for (int j = 0; j < height; j++)
        {
          //
          // BG
          // GR
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, blue
              dst[i+j*width] = src[i*3+j*width*3+0];
            else  // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
          else  // odd column
            if (i%2)  // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else  // odd row, red
              dst[i+j*width] = src[i*3+j*width*3+2];
        }
      }
    }
    else if (format == "BAYER_GBRG8")
    {
      for (int i = 0; i < width; i++)
      {
        for (int j = 0; j < height; j++)
        {
          //
          // GB
          // RG
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else  // odd row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
          else  // odd column
            if (i%2)  // even row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
            else  // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
        }
      }
    }
    else if (format == "BAYER_GRBG8")
    {
      for (int i = 0; i < width; i++)
      {
        for (int j = 0; j < height; j++)
        {
          //
          // GR
          // BG
          //
          // determine position
          if (j%2)  // even column
            if (i%2)  // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else  // odd row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
          else  // odd column
            if (i%2)  // even row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
            else  // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Camera::SetCaptureData(bool _value)
{
  this->captureData = _value;
}

//////////////////////////////////////////////////
void Camera::SetCaptureDataOnce()
{
  this->captureDataOnce = true;
}

//////////////////////////////////////////////////
void Camera::CreateRenderTexture(const std::string &_textureName)
{
  int fsaa = 4;

  // Full-screen anti-aliasing only works correctly in 1.8 and above
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR < 8
  fsaa = 0;
#endif

  // Create the render texture
  this->renderTexture = (Ogre::TextureManager::getSingleton().createManual(
      _textureName,
      "General",
      Ogre::TEX_TYPE_2D,
      this->GetImageWidth(),
      this->GetImageHeight(),
      0,
      (Ogre::PixelFormat)this->imageFormat,
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
  this->camera = this->scene->GetManager()->createCamera(
      this->scopedUniqueName);

  this->camera->setFixedYawAxis(false);
  this->camera->yaw(Ogre::Degree(-90.0));
  this->camera->roll(Ogre::Degree(-90.0));
}

//////////////////////////////////////////////////
bool Camera::GetWorldPointOnPlane(int _x, int _y,
                                  const math::Plane &_plane,
                                  math::Vector3 &_result)
{
  math::Vector3 origin, dir;
  double dist;

  // Cast two rays from the camera into the world
  this->GetCameraToViewportRay(_x, _y, origin, dir);

  dist = _plane.Distance(origin, dir);

  _result = origin + dir * dist;

  if (!math::equal(dist, -1.0))
    return true;
  else
    return false;
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

    RTShaderSystem::AttachViewport(this->viewport, this->GetScene());

    this->viewport->setBackgroundColour(
        Conversions::Convert(this->scene->GetBackgroundColor()));
    this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    double ratio = static_cast<double>(this->viewport->getActualWidth()) /
                   static_cast<double>(this->viewport->getActualHeight());

    double hfov = this->GetHFOV().Radian();
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));

    // Setup Deferred rendering for the camera
    /*if (RenderEngine::Instance()->GetRenderPathType() == RenderEngine::DEFERRED)
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
    }*/

    if (this->dataPtr->distortion)
      this->dataPtr->distortion->SetCamera(shared_from_this());

    /*if (this->GetScene()->skyx != NULL)
      this->renderTarget->addListener(this->GetScene()->skyx);
      */
  }
}

//////////////////////////////////////////////////
void Camera::AttachToVisual(uint32_t _visualId,
                            bool _inheritOrientation,
                            double _minDist, double _maxDist)
{
  msgs::Request request;
  msgs::TrackVisual track;

  track.set_name(this->GetName() + "_attach_to_visual_track");
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
                            bool _inheritOrientation,
                            double _minDist, double _maxDist)
{
  msgs::Request request;
  msgs::TrackVisual track;

  VisualPtr visual = this->scene->GetVisual(_visualName);

  if (visual)
    track.set_id(visual->GetId());
  else
  {
    gzerr << "Unable to attach to visual with name[" << _visualName << "]\n";
    track.set_id(GZ_UINT32_MAX);
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
bool Camera::AttachToVisualImpl(uint32_t _id,
    bool _inheritOrientation, double _minDist, double _maxDist)
{
  VisualPtr visual = this->scene->GetVisual(_id);
  return this->AttachToVisualImpl(visual, _inheritOrientation,
                                  _minDist, _maxDist);
}

//////////////////////////////////////////////////
bool Camera::AttachToVisualImpl(const std::string &_name,
    bool _inheritOrientation, double _minDist, double _maxDist)
{
  VisualPtr visual = this->scene->GetVisual(_name);
  return this->AttachToVisualImpl(visual, _inheritOrientation,
                                  _minDist, _maxDist);
}

//////////////////////////////////////////////////
bool Camera::AttachToVisualImpl(VisualPtr _visual, bool _inheritOrientation,
    double /*_minDist*/, double /*_maxDist*/)
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
  else
    this->dataPtr->trackedVisual.reset();

  if (_name.empty())
    return true;

  return false;
}

//////////////////////////////////////////////////
bool Camera::TrackVisualImpl(VisualPtr _visual)
{
  // if (this->sceneNode->getParent())
  //  this->sceneNode->getParent()->removeChild(this->sceneNode);

  bool result = false;
  if (_visual)
  {
    this->dataPtr->trackVisualPID.Init(0.25, 0, 0, 0, 0, 1.0, 0.0);
    this->dataPtr->trackVisualPitchPID.Init(0.05, 0, 0, 0, 0, 1.0, 0.0);
    this->dataPtr->trackVisualYawPID.Init(0.05, 0, 0, 0, 0, 1.0, 0.0);

    this->dataPtr->trackedVisual = _visual;
    result = true;
  }
  else
  {
    this->dataPtr->trackedVisual.reset();
  }

  return result;
}

//////////////////////////////////////////////////
Ogre::Texture *Camera::GetRenderTexture() const
{
  return this->renderTexture;
}

/////////////////////////////////////////////////
math::Vector3 Camera::GetDirection() const
{
  return Conversions::Convert(this->camera->getDerivedDirection());
}

/////////////////////////////////////////////////
bool Camera::IsVisible(VisualPtr _visual)
{
  if (this->camera && _visual)
  {
    math::Box bbox = _visual->GetBoundingBox();
    Ogre::AxisAlignedBox box;
    box.setMinimum(bbox.min.x, bbox.min.y, bbox.min.z);
    box.setMaximum(bbox.max.x, bbox.max.y, bbox.max.z);

    box.transformAffine(_visual->GetSceneNode()->_getFullTransform());
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
bool Camera::MoveToPosition(const math::Pose &_pose, double _time)
{
  if (this->animState)
  {
    this->dataPtr->moveToPositionQueue.push_back(std::make_pair(_pose, _time));
    return false;
  }

  Ogre::TransformKeyFrame *key;
  math::Vector3 rpy = _pose.rot.GetAsEuler();
  math::Vector3 start = this->GetWorldPose().pos;

  Ogre::Quaternion localRotOgre = this->sceneNode->getOrientation();
  math::Quaternion localRot = math::Quaternion(
    localRotOgre.w, localRotOgre.x, localRotOgre.y, localRotOgre.z);
  double dyaw =  localRot.GetAsEuler().z - rpy.z;

  if (dyaw > M_PI)
    rpy.z += 2*M_PI;
  else if (dyaw < -M_PI)
    rpy.z -= 2*M_PI;

  math::Quaternion pitchYawOnly(0, rpy.y, rpy.z);
  Ogre::Quaternion pitchYawFinal(pitchYawOnly.w, pitchYawOnly.x,
    pitchYawOnly.y, pitchYawOnly.z);

  std::string trackName = "cameratrack";
  int i = 0;
  while (this->scene->GetManager()->hasAnimation(trackName))
  {
    trackName = std::string("cameratrack_") +
      boost::lexical_cast<std::string>(i);
    i++;
  }

  Ogre::Animation *anim =
    this->scene->GetManager()->createAnimation(trackName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = strack->createNodeKeyFrame(_time);
  key->setTranslate(Ogre::Vector3(_pose.pos.x, _pose.pos.y, _pose.pos.z));
  key->setRotation(pitchYawFinal);


  this->animState =
    this->scene->GetManager()->createAnimationState(trackName);

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();

  return true;
}

/////////////////////////////////////////////////
bool Camera::MoveToPositions(const std::vector<math::Pose> &_pts,
                             double _time, boost::function<void()> _onComplete)
{
  if (this->animState)
    return false;

  this->onAnimationComplete = _onComplete;

  Ogre::TransformKeyFrame *key;
  math::Vector3 start = this->GetWorldPose().pos;

  std::string trackName = "cameratrack";
  int i = 0;
  while (this->scene->GetManager()->hasAnimation(trackName))
  {
    trackName = std::string("cameratrack_") +
      boost::lexical_cast<std::string>(i);
    i++;
  }

  Ogre::Animation *anim =
    this->scene->GetManager()->createAnimation(trackName, _time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  double dt = _time / (_pts.size()-1);
  double tt = 0;

  Ogre::Quaternion localRotOgre = this->sceneNode->getOrientation();
  math::Quaternion localRot = math::Quaternion(
    localRotOgre.w, localRotOgre.x, localRotOgre.y, localRotOgre.z);
  double prevYaw = localRot.GetAsEuler().z;
  for (unsigned int j = 0; j < _pts.size(); j++)
  {
    math::Vector3 pos = _pts[j].pos;
    math::Vector3 rpy = _pts[j].rot.GetAsEuler();
    double dyaw = prevYaw - rpy.z;

    if (dyaw > M_PI)
      rpy.z += 2*M_PI;
    else if (dyaw < -M_PI)
      rpy.z -= 2*M_PI;

    prevYaw = rpy.z;

    math::Quaternion pitchYawOnly(0, rpy.y, rpy.z);
    Ogre::Quaternion pitchYawFinal(pitchYawOnly.w, pitchYawOnly.x,
      pitchYawOnly.y, pitchYawOnly.z);

    key = strack->createNodeKeyFrame(tt);
    key->setTranslate(Ogre::Vector3(pos.x, pos.y, pos.z));
    key->setRotation(pitchYawFinal);

    tt += dt;
  }

  this->animState = this->scene->GetManager()->createAnimationState(trackName);

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();

  return true;
}

//////////////////////////////////////////////////
void Camera::SetRenderRate(double _hz)
{
  if (_hz > 0.0)
    this->dataPtr->renderPeriod = 1.0 / _hz;
  else
    this->dataPtr->renderPeriod = 0.0;
}

//////////////////////////////////////////////////
double Camera::GetRenderRate() const
{
  return 1.0 / this->dataPtr->renderPeriod.Double();
}

//////////////////////////////////////////////////
void Camera::AnimationComplete()
{
}

//////////////////////////////////////////////////
bool Camera::GetInitialized() const
{
  return this->initialized && this->scene->GetInitialized();
}

//////////////////////////////////////////////////
void Camera::OnCmdMsg(ConstCameraCmdPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->dataPtr->receiveMutex);
  this->dataPtr->commandMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
DistortionPtr Camera::GetDistortion() const
{
  return this->dataPtr->distortion;
}
