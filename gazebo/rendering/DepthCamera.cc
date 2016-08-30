/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _WIN32
  #include <dirent.h>
#else
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/DepthCameraPrivate.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
DepthCamera::DepthCamera(const std::string &_namePrefix, ScenePtr _scene,
                         bool _autoRender)
  : Camera(_namePrefix, _scene, _autoRender),
    dataPtr(new DepthCameraPrivate)
{
  this->depthTarget = NULL;
  this->dataPtr->depthBuffer = NULL;
  this->dataPtr->depthMaterial = NULL;
  this->dataPtr->pcdTarget = NULL;
  this->dataPtr->pcdBuffer = NULL;
  this->dataPtr->pcdMaterial = NULL;
  this->dataPtr->outputPoints = false;
}

//////////////////////////////////////////////////
DepthCamera::~DepthCamera()
{
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;

  if (this->dataPtr->pcdBuffer)
    delete [] this->dataPtr->pcdBuffer;
}

//////////////////////////////////////////////////
void DepthCamera::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);
  this->dataPtr->outputPoints =
    (_sdf->GetElement("depth_camera")->Get<std::string>("output")
    == "points");
}

//////////////////////////////////////////////////
void DepthCamera::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void DepthCamera::Init()
{
  Camera::Init();
}

//////////////////////////////////////////////////
void DepthCamera::Fini()
{
  Camera::Fini();
}

//////////////////////////////////////////////////
void DepthCamera::CreateDepthTexture(const std::string &_textureName)
{
  // Create the depth buffer
  std::string depthMaterialName = this->Name() + "_RttMat_Camera_Depth";

  this->depthTexture = Ogre::TextureManager::getSingleton().createManual(
      _textureName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      this->ImageWidth(), this->ImageHeight(), 0,
      Ogre::PF_FLOAT32_R,
      Ogre::TU_RENDERTARGET).getPointer();

  this->depthTarget = this->depthTexture->getBuffer()->getRenderTarget();
  this->depthTarget->setAutoUpdated(false);

  this->SetDepthTarget(this->depthTarget);

  this->depthViewport->setOverlaysEnabled(false);
  this->depthViewport->setBackgroundColour(
      Ogre::ColourValue(Ogre::ColourValue(0, 0, 0)));

  // Create materials for all the render textures.
  Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create(
      depthMaterialName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  matPtr->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  matPtr->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  matPtr->getTechnique(0)->getPass(0)->createTextureUnitState(
      _textureName);

  this->dataPtr->depthMaterial = (Ogre::Material*)(
      Ogre::MaterialManager::getSingleton().getByName("Gazebo/DepthMap").get());

  this->dataPtr->depthMaterial->load();

  if (this->dataPtr->outputPoints)
  {
    this->dataPtr->pcdTexture =
        Ogre::TextureManager::getSingleton().createManual(
        _textureName + "_pcd",
        "General",
        Ogre::TEX_TYPE_2D,
        this->ImageWidth(), this->ImageHeight(), 0,
        Ogre::PF_FLOAT32_RGBA,
        Ogre::TU_RENDERTARGET).getPointer();

    this->dataPtr->pcdTarget =
        this->dataPtr->pcdTexture->getBuffer()->getRenderTarget();
    this->dataPtr->pcdTarget->setAutoUpdated(false);

    this->dataPtr->pcdViewport =
        this->dataPtr->pcdTarget->addViewport(this->camera);
    this->dataPtr->pcdViewport->setClearEveryFrame(true);
    this->dataPtr->pcdViewport->setBackgroundColour(
        Conversions::Convert(this->scene->BackgroundColor()));
    this->dataPtr->pcdViewport->setOverlaysEnabled(false);
    this->dataPtr->pcdViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    this->dataPtr->pcdMaterial = (Ogre::Material*)(
    Ogre::MaterialManager::getSingleton().getByName("Gazebo/XYZPoints").get());

    this->dataPtr->pcdMaterial->getTechnique(0)->getPass(0)->
        createTextureUnitState(this->renderTexture->getName());

    this->dataPtr->pcdMaterial->load();
  }

/*
  // Create a custom render queue invocation sequence for the depth
  // render texture
  Ogre::RenderQueueInvocationSequence* invocationSequence =
    Ogre::Root::getSingleton().createRenderQueueInvocationSequence(_textureName
    + "_DepthMap");

  // Add a render queue invocation to the sequence, and disable shadows for it
  Ogre::RenderQueueInvocation* invocation =
    invocationSequence->add(Ogre::RENDER_QUEUE_MAIN, _textureName + "_main");
  invocation->setSuppressShadows(true);

  // Set the render queue invocation sequence for the depth render texture
  // viewport
  this->depthViewport->setRenderQueueInvocationSequenceName(
  _textureName + "_DepthMap");
*/
}

//////////////////////////////////////////////////
void DepthCamera::PostRender()
{
  this->depthTarget->swapBuffers();
  if (this->dataPtr->outputPoints)
    this->dataPtr->pcdTarget->swapBuffers();

  if (this->newData && this->captureData)
  {
    unsigned int width = this->ImageWidth();
    unsigned int height = this->ImageHeight();

    if (!this->dataPtr->outputPoints)
    {
      Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

      // Get access to the buffer and make an image and write it to file
      pixelBuffer = this->depthTexture->getBuffer();

      size_t size = Ogre::PixelUtil::getMemorySize(width, height, 1,
          Ogre::PF_FLOAT32_R);

      // Blit the depth buffer if needed
      if (!this->dataPtr->depthBuffer)
        this->dataPtr->depthBuffer = new float[size];

      Ogre::PixelBox dstBox(width, height,
          1, Ogre::PF_FLOAT32_R, this->dataPtr->depthBuffer);

      pixelBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);
      pixelBuffer->blitToMemory(dstBox);
      pixelBuffer->unlock();  // FIXME: do we need to lock/unlock still?

      this->dataPtr->newDepthFrame(
          this->dataPtr->depthBuffer, width, height, 1, "FLOAT32");
    }
    else
    {
      Ogre::HardwarePixelBufferSharedPtr pcdPixelBuffer;

      // Get access to the buffer and make an image and write it to file
      pcdPixelBuffer = this->dataPtr->pcdTexture->getBuffer();

      // Blit the depth buffer if needed
      if (!this->dataPtr->pcdBuffer)
        this->dataPtr->pcdBuffer = new float[width * height * 4];

      memset(this->dataPtr->pcdBuffer, 0, width * height * 4);

      Ogre::Box pcd_src_box(0, 0, width, height);
      Ogre::PixelBox pcd_dst_box(width, height,
          1, Ogre::PF_FLOAT32_RGBA, this->dataPtr->pcdBuffer);

      pcdPixelBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);
      pcdPixelBuffer->blitToMemory(pcd_src_box, pcd_dst_box);
      pcdPixelBuffer->unlock();

      this->dataPtr->newRGBPointCloud(
          this->dataPtr->pcdBuffer, width, height, 1, "RGBPOINTS");
    }
  }

  // also new image frame for camera texture
  Camera::PostRender();

  this->newData = false;
}

//////////////////////////////////////////////////
void DepthCamera::UpdateRenderTarget(Ogre::RenderTarget *_target,
          Ogre::Material *_material, const std::string &_matName)
{
  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::Pass *pass;

  renderSys = this->scene->OgreSceneManager()->getDestinationRenderSystem();
  // Get pointer to the material pass
  pass = _material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  this->OgreCamera()->setFarClipDistance(this->FarClip());

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = _target->getViewport(0);

  // return farClip in case no renderable object is inside frustrum
  vp->setBackgroundColour(Ogre::ColourValue(this->FarClip(),
      this->FarClip(), this->FarClip()));

  Ogre::CompositorManager::getSingleton().setCompositorEnabled(
                                                vp, _matName, true);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(_target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(this->OgreCamera(), true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

  // These two lines don't seem to do anything useful
  renderSys->_setProjectionMatrix(
      this->OgreCamera()->getProjectionMatrixRS());
  renderSys->_setViewMatrix(this->OgreCamera()->getViewMatrix(true));

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  pass->_updateAutoParamsNoLights(&autoParamDataSource);
#else
  pass->_updateAutoParams(&autoParamDataSource, 1);
#endif

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
    pass->getVertexProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
    pass->getVertexProgramParameters());
#else
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(), 1);
#endif
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters());
#else
      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(), 1);
#endif
  }
}

//////////////////////////////////////////////////
void DepthCamera::RenderImpl()
{
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();

  Ogre::ShadowTechnique shadowTech = sceneMgr->getShadowTechnique();

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  sceneMgr->_suppressRenderStateChanges(true);

  this->UpdateRenderTarget(this->depthTarget,
                  this->dataPtr->depthMaterial, "Gazebo/DepthMap");

  // Does actual rendering
  this->depthTarget->update(false);

  sceneMgr->_suppressRenderStateChanges(false);
  sceneMgr->setShadowTechnique(shadowTech);

  // for camera image
  Camera::RenderImpl();

  if (this->dataPtr->outputPoints)
  {
    sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
    sceneMgr->_suppressRenderStateChanges(true);

    this->UpdateRenderTarget(this->dataPtr->pcdTarget,
                  this->dataPtr->pcdMaterial, "Gazebo/XYZPoints");

    this->dataPtr->pcdTarget->update(false);

    sceneMgr->_suppressRenderStateChanges(false);
    sceneMgr->setShadowTechnique(shadowTech);
  }
}

//////////////////////////////////////////////////
const float* DepthCamera::DepthData() const
{
  return this->dataPtr->depthBuffer;
}

//////////////////////////////////////////////////
void DepthCamera::SetDepthTarget(Ogre::RenderTarget *_target)
{
  this->depthTarget = _target;

  if (this->depthTarget)
  {
    // Setup the viewport to use the texture
    this->depthViewport = this->depthTarget->addViewport(this->camera);
    this->depthViewport->setClearEveryFrame(true);
    this->depthViewport->setBackgroundColour(
        Conversions::Convert(this->scene->BackgroundColor()));
    this->depthViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    double ratio = static_cast<double>(this->depthViewport->getActualWidth()) /
                   static_cast<double>(this->depthViewport->getActualHeight());

    double hfov = this->HFOV().Radian();
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
    // gzerr << "debug " << hfov << " " << vfov << " " << ratio << "\n";
    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));
  }
}

//////////////////////////////////////////////////
event::ConnectionPtr DepthCamera::ConnectNewDepthFrame(
    std::function<void (const float *, unsigned int, unsigned int, unsigned int,
    const std::string &)>  _subscriber)
{
  return this->dataPtr->newDepthFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
void DepthCamera::DisconnectNewDepthFrame(event::ConnectionPtr &_c)
{
  this->dataPtr->newDepthFrame.Disconnect(_c->Id());
}

//////////////////////////////////////////////////
event::ConnectionPtr DepthCamera::ConnectNewRGBPointCloud(
    std::function<void (const float *, unsigned int, unsigned int, unsigned int,
    const std::string &)>  _subscriber)
{
  return this->dataPtr->newRGBPointCloud.Connect(_subscriber);
}

//////////////////////////////////////////////////
void DepthCamera::DisconnectNewRGBPointCloud(event::ConnectionPtr &_c)
{
  this->dataPtr->newRGBPointCloud.Disconnect(_c->Id());
}
