/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include <string>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/skyx/include/SkyX.h"
#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/OSVRCameraPrivate.hh"
#include "gazebo/rendering/OSVRCamera.hh"
#include "osvr/ClientKit/InterfaceStateC.h"

using namespace gazebo;
using namespace rendering;

const float g_defaultNearClip = 0.01f;
const float g_defaultFarClip = 500.0f;

//////////////////////////////////////////////////
OSVRCamera::OSVRCamera(const std::string &_name, ScenePtr _scene)
  : Camera(_name, _scene), dataPtr(new OSVRCameraPrivate)
{
  osvr::clientkit::ClientContext *ctx =
      new osvr::clientkit::ClientContext("gazebo");
  if (!ctx)
  {
    gzerr << "Extreme badness happened trying to allocate OSVR context.";
    return;
  }
  this->dataPtr->osvrContext = ctx;
  if (!ctx->checkStatus())
  {
    gzerr << "OSVR client context did not start correctly. Please make sure "
          << "the OSVR server is running.";
    return;
  }
  this->dataPtr->osvrInterface = ctx->getInterface("/me/head");
  //this->dataPtr->osvrInterface.registerCallback(&osvrOrientationCallback, this);
  // todo: query OSVR for the headset type, and branch accordingly
  this->SetRenderRate(80.0);
  gzmsg << "OSVR ClientKit connection established." << std::endl;
  // todo: log more stuff about the headset we're using
#if 0
  // Log some useful information
  gzmsg << "\tType: " << this->dataPtr->hmd->Type << std::endl;
  gzmsg << "\tProduct Name: " << this->dataPtr->hmd->ProductName << std::endl;
  gzmsg << "\tProduct ID: " << this->dataPtr->hmd->ProductId << std::endl;
  gzmsg << "\tFirmware: " << this->dataPtr->hmd->FirmwareMajor << "."
    << this->dataPtr->hmd->FirmwareMinor << std::endl;
  gzmsg << "\tResolution: " << this->dataPtr->hmd->Resolution.w << "x"
    << this->dataPtr->hmd->Resolution.h << std::endl;
  gzmsg << "\tPosition tracking: "
    << (this->dataPtr->hmd->TrackingCaps & ovrTrackingCap_Position)
    << std::endl;
#endif

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->controlSub = this->dataPtr->node->Subscribe("~/world_control",
                                           &OSVRCamera::OnControl, this);

  // OSVR is now ready.
  this->dataPtr->ready = true;
}

//////////////////////////////////////////////////
OSVRCamera::~OSVRCamera()
{
  if (this->dataPtr->externalSceneManager)
  {
    RenderEngine::Instance()->Root()->destroySceneManager(
        this->dataPtr->externalSceneManager);
  }
  delete this->dataPtr->osvrContext;
  this->connections.clear();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
void OSVRCamera::Load(sdf::ElementPtr _sdf)
{
  if (this->Ready())
    Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void OSVRCamera::OnControl(ConstWorldControlPtr &_data)
{
  if (_data->has_reset() && _data->reset().has_all() && _data->reset().all())
  {
    this->ResetSensor();
  }
}

//////////////////////////////////////////////////
void OSVRCamera::Load()
{
  if (this->Ready())
    Camera::Load();
}

//////////////////////////////////////////////////
void OSVRCamera::Init()
{
  if (!this->Ready())
    return;

  Camera::Init();

  // OSVR
  {
    this->dataPtr->rightCamera = this->scene->OgreSceneManager()->createCamera(
      "OSVRUserRight");
    this->dataPtr->rightCamera->pitch(Ogre::Degree(90));

    // Don't yaw along variable axis, causes leaning
    this->dataPtr->rightCamera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    this->dataPtr->rightCamera->setDirection(1, 0, 0);

    this->sceneNode->attachObject(this->dataPtr->rightCamera);

    this->dataPtr->rightCamera->setAutoAspectRatio(false);
    this->camera->setAutoAspectRatio(false);

    this->dataPtr->rightCamera->setNearClipDistance(g_defaultNearClip);
    this->dataPtr->rightCamera->setFarClipDistance(g_defaultFarClip);

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
void OSVRCamera::RenderImpl()
{
  printf("Render\n");
#if 0
  ovrHmd_BeginFrameTiming(this->dataPtr->hmd, this->dataPtr->frameIndex);
#endif
  this->dataPtr->renderTextureLeft->getBuffer()->getRenderTarget()->update();
  this->dataPtr->renderTextureRight->getBuffer()->getRenderTarget()->update();
  this->renderTarget->update();
#if 0
  ovrHmd_EndFrameTiming(this->dataPtr->hmd);
#endif
  this->dataPtr->frameIndex++;
}

//////////////////////////////////////////////////
void OSVRCamera::Update()
{
  if (!this->Ready())
    return;

  Camera::Update();

  this->dataPtr->osvrContext->update(); // this will fire our callback
  // alternatively we could try osvrGetPoseState(interface, timestamp, state)
  // not clear which gives the freshest data (or if there's any difference)

//////////////////////////////////////////////////
/// \brief OSVR orientation callback
//static void osvrOrientationCallback(void *userdata,
//                                    const OSVR_TimeValue *timestamp,
//                                    const OSVR_OrientationReport *rpt)
  OSVR_OrientationState orient;
  OSVR_TimeValue timestamp;
  OSVR_ReturnCode rc = osvrGetOrientationState(
                           this->dataPtr->osvrInterface.get(),
                           &timestamp, &orient);
  /*
  if (rc == OSVR_RETURN_SUCCESS)
  {
    if (this->dataPtr->osvrTrackingWarned)
    {
      gzmsg << "OSVR: Head tracking enabled.\n";
      this->dataPtr->osvrTrackingWarned = false;
    }

    this->sceneNode->setOrientation(Ogre::Quaternion(
        osvrQuatGetW(&orient),
        osvrQuatGetX(&orient),
        osvrQuatGetY(&orient),
        osvrQuatGetZ(&orient)));  // may need to swap/invert some axes (?)
  }
  else if (!this->dataPtr->osvrTrackingWarned)
  {
    this->sceneNode->setOrientation(Ogre::Quaternion(1, 0, 0, 0));
    gzmsg << "OSVR: didn't get an orientation. doh.\n";
    this->dataPtr->osvrTrackingWarned = true;
  }
  */

  this->sceneNode->needUpdate();
}

//////////////////////////////////////////////////
void OSVRCamera::ResetSensor()
{
}

//////////////////////////////////////////////////
bool OSVRCamera::Ready()
{
  return this->dataPtr->ready;
}

//////////////////////////////////////////////////
void OSVRCamera::PostRender()
{
  Camera::PostRender();
}

//////////////////////////////////////////////////
void OSVRCamera::Fini()
{
  Camera::Fini();
}

/////////////////////////////////////////////////
bool OSVRCamera::AttachToVisualImpl(VisualPtr _visual,
    bool _inheritOrientation,
    double /*_minDist*/, double /*_maxDist*/)
{
  Camera::AttachToVisualImpl(_visual, _inheritOrientation);
  if (_visual)
  {
    math::Pose origPose = this->WorldPose();
    double yaw = _visual->GetWorldPose().rot.GetAsEuler().z;

    double zDiff = origPose.pos.z - _visual->GetWorldPose().pos.z;
    double pitch = 0;

    if (fabs(zDiff) > 1e-3)
    {
      double dist = _visual->GetWorldPose().pos.Distance(
          this->WorldPose().Pos());
      pitch = acos(zDiff/dist);
    }

    this->Yaw(ignition::math::Angle(yaw));
    this->Pitch(ignition::math::Angle(pitch));

    math::Box bb = _visual->GetBoundingBox();
    math::Vector3 pos = bb.GetCenter();
    pos.z = bb.max.z;
  }

  return true;
}

//////////////////////////////////////////////////
bool OSVRCamera::TrackVisualImpl(VisualPtr _visual)
{
  Camera::TrackVisualImpl(_visual);

  return true;
}

//////////////////////////////////////////////////
unsigned int OSVRCamera::GetImageWidth() const
{
  return this->viewport->getActualWidth();
}

//////////////////////////////////////////////////
unsigned int OSVRCamera::GetImageHeight() const
{
  return this->viewport->getActualHeight();
}

//////////////////////////////////////////////////
void OSVRCamera::Resize(unsigned int /*_w*/, unsigned int /*_h*/)
{
  printf("OSVRCamera::Resize()\n");
  if (this->viewport)
  {
    this->viewport->setDimensions(0, 0, 0.5, 1);
    this->dataPtr->rightViewport->setDimensions(0.5, 0, 0.5, 1);

    delete [] this->saveFrameBuffer;
    this->saveFrameBuffer = NULL;
  }
}

//////////////////////////////////////////////////
bool OSVRCamera::MoveToPosition(const math::Pose &_pose, double _time)
{
  return Camera::MoveToPosition(_pose.Ign(), _time);
}

//////////////////////////////////////////////////
void OSVRCamera::MoveToVisual(const std::string &_name)
{
  VisualPtr visualPtr = this->scene->GetVisual(_name);
  if (visualPtr)
    this->MoveToVisual(visualPtr);
  else
    gzerr << "MoveTo Unknown visual[" << _name << "]\n";
}

//////////////////////////////////////////////////
void OSVRCamera::MoveToVisual(VisualPtr _visual)
{
  if (!_visual)
    return;

  if (this->scene->OgreSceneManager()->hasAnimation("cameratrack"))
  {
    this->scene->OgreSceneManager()->destroyAnimation("cameratrack");
  }

  math::Box box = _visual->GetBoundingBox();
  math::Vector3 size = box.GetSize();
  double maxSize = std::max(std::max(size.x, size.y), size.z);

  math::Vector3 start = this->WorldPose().Pos();
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

  double scale = maxSize / tan((this->HFOV()/2.0).Radian());

  end = mid + dir*(dist - scale);

  // dist = start.Distance(end);
  // double vel = 5.0;
  double time = 0.5;  // dist / vel;

  Ogre::Animation *anim =
    this->scene->OgreSceneManager()->createAnimation("cameratrack", time);
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
    this->scene->OgreSceneManager()->createAnimationState("cameratrack");

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
void OSVRCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  printf("OSVRCamera::SetRenderTarget()\n");
  this->renderTarget = _target;
  this->CreateDistortion();

  Ogre::RenderTexture *rt =
    this->dataPtr->renderTextureLeft->getBuffer()->getRenderTarget();

  rt->addViewport(this->camera);
  rt->getViewport(0)->setClearEveryFrame(true);
  rt->getViewport(0)->setOverlaysEnabled(false);
  rt->getViewport(0)->setShadowsEnabled(true);
  rt->getViewport(0)->setBackgroundColour(
        Conversions::Convert(this->scene->BackgroundColor()));
  rt->getViewport(0)->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  RTShaderSystem::AttachViewport(rt->getViewport(0), this->GetScene());

  if (this->GetScene()->GetSkyX() != NULL)
    rt->addListener(this->GetScene()->GetSkyX());

  rt = this->dataPtr->renderTextureRight->getBuffer()->getRenderTarget();
  rt->addViewport(this->dataPtr->rightCamera);
  rt->getViewport(0)->setClearEveryFrame(true);
  rt->getViewport(0)->setShadowsEnabled(true);
  rt->getViewport(0)->setOverlaysEnabled(false);
  rt->getViewport(0)->setBackgroundColour(
        Conversions::Convert(this->scene->BackgroundColor()));
  rt->getViewport(0)->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  RTShaderSystem::AttachViewport(rt->getViewport(0), this->GetScene());

  if (this->GetScene()->GetSkyX() != NULL)
    rt->addListener(this->GetScene()->GetSkyX());

  // todo: not this
  float fov_horiz = 90; //this->dataPtr->hmd->DefaultEyeFov[ovrEye_Left];
  //float fovRight = 90; //this->dataPtr->hmd->DefaultEyeFov[ovrEye_Right];
  float fov_vert = 90;

  //float combinedTanHalfFovHorizontal =
  //  std::max(fovLeft.LeftTan, fovLeft.RightTan);
  //float combinedTanHalfFovVertical = std::max(fovLeft.UpTan, fovLeft.DownTan);

  float aspectRatio = fov_horiz / fov_vert; //combinedTanHalfFovHorizontal / combinedTanHalfFovVertical;

  //double vfov = 2.0 * atan(combinedTanHalfFovVertical);
  this->camera->setAspectRatio(aspectRatio);
  //this->camera->setFOVy(Ogre::Radian(vfov));
  this->camera->setFOVy(Ogre::Radian(fov_vert * 3.1415f / 180.0f));
  this->dataPtr->rightCamera->setAspectRatio(aspectRatio);
  //this->dataPtr->rightCamera->setFOVy(Ogre::Radian(vfov));
  this->dataPtr->rightCamera->setFOVy(Ogre::Radian(fov_vert * 3.1415f/180.0f));

  Ogre::Matrix4 left_proj = this->camera->getProjectionMatrix();
  gzmsg << "OSVR left projection matrix:\n" << left_proj << "\n\n";

#if 0
  ovrMatrix4f projL = ovrMatrix4f_Projection(fovLeft, 0.001,
      g_defaultFarClip, true);
  ovrMatrix4f projR = ovrMatrix4f_Projection(fovRight, 0.001,
      g_defaultFarClip, true);

  this->camera->setCustomProjectionMatrix(true,
    Ogre::Matrix4(
      projL.M[0][0], projL.M[0][1], projL.M[0][2], projL.M[0][3],
      projL.M[1][0], projL.M[1][1], projL.M[1][2], projL.M[1][3],
      projL.M[2][0], projL.M[2][1], projL.M[2][2], projL.M[2][3],
      projL.M[3][0], projL.M[3][1], projL.M[3][2], projL.M[3][3]));

  this->dataPtr->rightCamera->setCustomProjectionMatrix(true,
    Ogre::Matrix4(
      projR.M[0][0], projR.M[0][1], projR.M[0][2], projR.M[0][3],
      projR.M[1][0], projR.M[1][1], projR.M[1][2], projR.M[1][3],
      projR.M[2][0], projR.M[2][1], projR.M[2][2], projR.M[2][3],
      projR.M[3][0], projR.M[3][1], projR.M[3][2], projR.M[3][3]));

  // This seems like a mistake, but it's here on purpose.
  // Problem: Shadows get rendered incorrectly with
  // setCustomProjectionMatrix.
  // Solution (HACK): Pass false to setCustomProjectionMatrix. Found this
  // solution here: http://www.ogre3d.org/forums/viewtopic.php?f=2&t=60904
  // and here: http://www.ogre3d.org/forums/viewtopic.php?f=2&t=78461
  this->camera->setCustomProjectionMatrix(false,
    Ogre::Matrix4(
      projL.M[0][0], projL.M[0][1], projL.M[0][2], projL.M[0][3],
      projL.M[1][0], projL.M[1][1], projL.M[1][2], projL.M[1][3],
      projL.M[2][0], projL.M[2][1], projL.M[2][2], projL.M[2][3],
      projL.M[3][0], projL.M[3][1], projL.M[3][2], projL.M[3][3]));

  this->dataPtr->rightCamera->setCustomProjectionMatrix(false,
    Ogre::Matrix4(
      projR.M[0][0], projR.M[0][1], projR.M[0][2], projR.M[0][3],
      projR.M[1][0], projR.M[1][1], projR.M[1][2], projR.M[1][3],
      projR.M[2][0], projR.M[2][1], projR.M[2][2], projR.M[2][3],
      projR.M[3][0], projR.M[3][1], projR.M[3][2], projR.M[3][3]));
#endif
  this->initialized = true;
}

//////////////////////////////////////////////////
void OSVRCamera::CreateDistortion()
{
  if (!this->Ready())
    return;

  // Create a separate scene manager to holds a distorted mesh and a camera.
  // The distorted mesh receives the left and right camera images, and the
  // camera in the externalSceneManager renders the distorted meshes.
  this->dataPtr->externalSceneManager =
    RenderEngine::Instance()->Root()->createSceneManager(Ogre::ST_GENERIC);
  this->dataPtr->externalSceneManager->setAmbientLight(
      Ogre::ColourValue(0.5, 0.5, 0.5));

#if 0
  ovrFovPort fovLeft = this->dataPtr->hmd->DefaultEyeFov[ovrEye_Left];
  ovrFovPort fovRight = this->dataPtr->hmd->DefaultEyeFov[ovrEye_Right];

  // Get the texture sizes
  ovrSizei textureSizeLeft = ovrHmd_GetFovTextureSize(this->dataPtr->hmd,
       ovrEye_Left, fovLeft, 1.0f);
  ovrSizei textureSizeRight = ovrHmd_GetFovTextureSize(this->dataPtr->hmd,
      ovrEye_Right, fovRight, 1.0f);
#endif

  // Create the left and right render textures.
  this->dataPtr->renderTextureLeft =
    Ogre::TextureManager::getSingleton().createManual(
      "OSVRTextureLeft",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      1182, // textureSizeLeft.w,
      1461, // textureSizeLeft.h,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);

  this->dataPtr->renderTextureRight =
    Ogre::TextureManager::getSingleton().createManual(
      "OSVRTextureRight",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      1182, //textureSizeRight.w,
      1461, //textureSizeRight.h,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);

  // Create the left and right materials.
  Ogre::MaterialPtr matLeft =
    Ogre::MaterialManager::getSingleton().getByName("OSVR/LeftEye");
  Ogre::MaterialPtr matRight =
    Ogre::MaterialManager::getSingleton().getByName("OSVR/RightEye");

  // Attach materials to the render textures.
  matLeft->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(
      this->dataPtr->renderTextureLeft);
  matRight->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(
      this->dataPtr->renderTextureRight);

#if 0
  // Get eye description information
  ovrEyeRenderDesc eyeRenderDesc[2];
  eyeRenderDesc[0] = ovrHmd_GetRenderDesc(
      this->dataPtr->hmd, ovrEye_Left, fovLeft);
  eyeRenderDesc[1] = ovrHmd_GetRenderDesc(
      this->dataPtr->hmd, ovrEye_Right, fovRight);

  double combinedTanHalfFovHorizontal = std::max(
    std::max(fovLeft.LeftTan, fovLeft.RightTan),
    std::max(fovRight.LeftTan, fovRight.RightTan));
  double combinedTanHalfFovVertical = std::max(
    std::max(fovLeft.UpTan, fovLeft.DownTan),
    std::max(fovRight.UpTan, fovRight.DownTan));

  // Hold some values that are needed when creating the distortion meshes.
  ovrVector2f uvScaleOffset[2];
  ovrRecti viewports[2];
  viewports[0].Pos.x = 0;
  viewports[0].Pos.y = 0;
  viewports[0].Size.w = textureSizeLeft.w;
  viewports[0].Size.h = textureSizeLeft.h;
  viewports[1].Pos.x = textureSizeLeft.w;
  viewports[1].Pos.y = 0;
  viewports[1].Size.w = textureSizeRight.w;
  viewports[1].Size.h = textureSizeRight.h;
  #endif

  // Create a scene node in the external scene to hold the distortion
  // meshes.
  Ogre::SceneNode *meshNode =
    this->dataPtr->externalSceneManager->getRootSceneNode()
    ->createChildSceneNode();

  // Create the Distortion Meshes:
  for (int eyeIndex = 0; eyeIndex < 2; ++eyeIndex)
  {
#if 0
    ovrDistortionMesh meshData;

    // Make the FOV symmetrical. Refer to Section 8.5.2 of the oculus SDF
    // developers manual.
    eyeRenderDesc[eyeIndex].Fov.RightTan = combinedTanHalfFovHorizontal;
    eyeRenderDesc[eyeIndex].Fov.LeftTan = combinedTanHalfFovHorizontal;
    eyeRenderDesc[eyeIndex].Fov.UpTan = combinedTanHalfFovVertical;
    eyeRenderDesc[eyeIndex].Fov.DownTan = combinedTanHalfFovVertical;

    ovrHmd_CreateDistortionMesh(
        this->dataPtr->hmd,
        eyeRenderDesc[eyeIndex].Eye,
        eyeRenderDesc[eyeIndex].Fov,
        0,
        &meshData);
#endif

    Ogre::GpuProgramParametersSharedPtr params;

    if (eyeIndex == 0)
    {
#if 0
      ovrHmd_GetRenderScaleAndOffset(eyeRenderDesc[eyeIndex].Fov,
        textureSizeLeft, viewports[eyeIndex], uvScaleOffset);
      params =
        matLeft->getTechnique(0)->getPass(0)->getVertexProgramParameters();
#endif
    }
    else
    {
#if 0
      ovrHmd_GetRenderScaleAndOffset(eyeRenderDesc[eyeIndex].Fov,
        textureSizeRight, viewports[eyeIndex], uvScaleOffset);
      params =
        matRight->getTechnique(0)->getPass(0)->getVertexProgramParameters();
#endif
    }

#if 0
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR  >= 9
    params->setNamedConstant("eyeToSourceUVScale",
        Ogre::Vector2(uvScaleOffset[0].x, uvScaleOffset[0].y));
    params->setNamedConstant("eyeToSourceUVOffset",
        Ogre::Vector2(uvScaleOffset[1].x, uvScaleOffset[1].y));
#else
    params->setNamedConstant("eyeToSourceUVScale",
        Ogre::Vector4(uvScaleOffset[0].x, uvScaleOffset[0].y, 0, 0));
    params->setNamedConstant("eyeToSourceUVOffset",
        Ogre::Vector4(uvScaleOffset[1].x, uvScaleOffset[1].y, 0, 0));
#endif
#endif
    Ogre::ManualObject *externalObj;

    // create ManualObject
    if (eyeIndex == 0)
    {
      externalObj =
        this->dataPtr->externalSceneManager->createManualObject(
            "OSVRRenderObjectLeft");
      externalObj->begin("OSVR/LeftEye",
          Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }
    else
    {
      externalObj =
        this->dataPtr->externalSceneManager->createManualObject(
            "OSVRRenderObjectRight");
      externalObj->begin("OSVR/RightEye",
          Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    const float x = 10.0f;
    const float y = 10.0f;
    externalObj->position(0, 0, 0);
    externalObj->textureCoord(0, 0);
    externalObj->textureCoord(0, 0);
    externalObj->textureCoord(0, 0);
    externalObj->colour(1, 0, 1);

    externalObj->position(x, 0, 0);
    externalObj->textureCoord(1, 0);
    externalObj->textureCoord(1, 0);
    externalObj->textureCoord(1, 0);
    externalObj->colour(1, 0, 1);

    externalObj->position(x, y, 0);
    externalObj->textureCoord(1, 1);
    externalObj->textureCoord(1, 1);
    externalObj->textureCoord(1, 1);
    externalObj->colour(1, 0, 1);

    externalObj->index(0);
    externalObj->index(1);
    externalObj->index(2);
    /*
    for (unsigned int i = 0; i < meshData.VertexCount; ++i)
    {
      //ovrDistortionVertex v = meshData.pVertexData[i];
      //externalObj->position(v.ScreenPosNDC.x, v.ScreenPosNDC.y, 0);
      //externalObj->textureCoord(v.TanEyeAnglesR.x, v.TanEyeAnglesR.y);
      //externalObj->textureCoord(v.TanEyeAnglesG.x, v.TanEyeAnglesG.y);
      //externalObj->textureCoord(v.TanEyeAnglesB.x, v.TanEyeAnglesB.y);

      //float vig = std::max(v.VignetteFactor, 0.0f);
      float vig = 0.5; //std::max(v.VignetteFactor, 0.0f);
      externalObj->colour(vig, vig, vig, vig);
    }

    for (unsigned int i = 0; i < meshData.IndexCount; ++i)
    {
      externalObj->index(meshData.pIndexData[i]);
    }
    */

    // Manual render object complete
    externalObj->end();

    // Attach externalObj object to the node
    meshNode->attachObject(externalObj);

    // Free up memory
    //ovrHmd_DestroyDistortionMesh(&meshData);
  }

  // Position the node in the scene
  meshNode->setPosition(0, 0, -1);
  //meshNode->setScale(1, 1, -1);  // MQ this causes badness

  // Create the external camera
  this->dataPtr->externalCamera =
    this->dataPtr->externalSceneManager->createCamera(
        "_OSVRExternalCamera_INTERNAL_");
  this->dataPtr->externalCamera->setFarClipDistance(50);
  this->dataPtr->externalCamera->setNearClipDistance(0.001);
  this->dataPtr->externalCamera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  this->dataPtr->externalCamera->setOrthoWindow(2, 2);
  this->dataPtr->externalSceneManager->getRootSceneNode()->attachObject(
      this->dataPtr->externalCamera);

  // Create the external viewport
  this->dataPtr->externalViewport = this->renderTarget->addViewport(
      this->dataPtr->externalCamera);
  this->dataPtr->externalViewport->setBackgroundColour(
      Ogre::ColourValue::White);
  this->dataPtr->externalViewport->setOverlaysEnabled(true);

  // Set up IPD in meters:
  //float ipd = ovrHmd_GetFloat(this->dataPtr->hmd, OVR_KEY_IPD,  0.064f);
  float ipd = 0.064f;
  this->camera->setPosition(-ipd * 0.5, 0, 0);
  this->dataPtr->rightCamera->setPosition(ipd * 0.5, 0, 0);
  printf("done creating OSVR distortion setup\n");
}

/////////////////////////////////////////////////
void OSVRCamera::AdjustAspect(double _v)
{
  if (!this->Ready())
    return;

  for (int i = 0; i < 2; ++i)
  {
    Ogre::Camera *cam = i == 0 ? this->camera : this->dataPtr->rightCamera;
    cam->setAspectRatio(cam->getAspectRatio() + _v);
  }
}
