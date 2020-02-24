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

#ifndef _WIN32
  #include <dirent.h>
#else
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/DepthCameraPrivate.hh"
#include "gazebo/rendering/RTShaderSystem.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
DepthCamera::DepthCamera(const std::string &_namePrefix, ScenePtr _scene,
                         bool _autoRender)
  : Camera(_namePrefix, _scene, _autoRender),
    dataPtr(new DepthCameraPrivate)
{
  this->dataPtr->outputPoints = false;
  this->dataPtr->outputReflectance = false;
}

//////////////////////////////////////////////////
DepthCamera::~DepthCamera()
{
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;

  if (this->dataPtr->reflectanceBuffer)
    delete [] this->dataPtr->reflectanceBuffer;

  if (this->dataPtr->pcdBuffer)
    delete [] this->dataPtr->pcdBuffer;
}

//////////////////////////////////////////////////
void DepthCamera::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);
  std::string outputs = _sdf->GetElement("depth_camera")->
                              Get<std::string>("output");
  std::size_t found = outputs.find("points");
  this->dataPtr->outputPoints =  found != std::string::npos;
  found = outputs.find("reflectance");
  this->dataPtr->outputReflectance =  found != std::string::npos;
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
  if (this->dataPtr->reflectanceViewport && this->scene)
    RTShaderSystem::DetachViewport(this->dataPtr->reflectanceViewport,
                                   this->scene);

  if (this->dataPtr->reflectanceTarget)
    this->dataPtr->reflectanceTarget->removeAllViewports();
  this->dataPtr->reflectanceTarget = nullptr;

  if (this->dataPtr->reflectanceTextures)
    Ogre::TextureManager::getSingleton()
          .remove(this->dataPtr->reflectanceTextures->getName());
  this->dataPtr->reflectanceTextures = nullptr;

  this->dataPtr->reflectanceMaterialSwitcher.reset();
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

    auto const &ignBG = this->scene->BackgroundColor();
    this->dataPtr->pcdViewport->setBackgroundColour(
        Conversions::Convert(ignBG));
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
void DepthCamera::CreateReflectanceTexture(const std::string &_textureName)
{
  if (this->dataPtr->outputReflectance)
  {
    this->dataPtr->reflectanceTextures =
      Ogre::TextureManager::getSingleton().createManual(
      _textureName + "_reflectance",
      "General",
      Ogre::TEX_TYPE_2D,
      this->ImageWidth(), this->ImageHeight(), 0,
      Ogre::PF_FLOAT32_R,
      Ogre::TU_RENDERTARGET).getPointer();

    this->dataPtr->reflectanceTarget =
        this->dataPtr->reflectanceTextures->getBuffer()->getRenderTarget();
    this->dataPtr->reflectanceTarget->setAutoUpdated(false);

    this->dataPtr->reflectanceViewport =
        this->dataPtr->reflectanceTarget->addViewport(this->camera);
    this->dataPtr->reflectanceViewport->setClearEveryFrame(true);

    this->dataPtr->reflectanceViewport->setBackgroundColour(
        Ogre::ColourValue(Ogre::ColourValue(0, 0, 0)));

    this->dataPtr->reflectanceViewport->setOverlaysEnabled(false);
    this->dataPtr->reflectanceViewport->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    this->dataPtr->reflectanceViewport->setMaterialScheme("reflectance_map");

    this->dataPtr->reflectanceMaterialSwitcher.reset(
        new ReflectanceMaterialSwitcher(this->scene,
                                        this->dataPtr->reflectanceViewport));
    this->dataPtr->reflectanceMaterialSwitcher->
                   SetMaterialScheme("reflectance_map");
  }
}

//////////////////////////////////////////////////
void DepthCamera::PostRender()
{
  this->depthTarget->swapBuffers();
  if (this->dataPtr->outputPoints)
    this->dataPtr->pcdTarget->swapBuffers();
  if (this->dataPtr->outputReflectance)
    this->dataPtr->reflectanceTarget->swapBuffers();

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

    if (this->dataPtr->outputReflectance)
    {
     Ogre::HardwarePixelBufferSharedPtr reflectancePixelBuffer;

     reflectancePixelBuffer = this->dataPtr->reflectanceTextures->getBuffer();

     // Blit the depth buffer if needed
     if (!this->dataPtr->reflectanceBuffer)
       this->dataPtr->reflectanceBuffer = new float[width * height * 1];

     memset(this->dataPtr->reflectanceBuffer, 0, width * height * 1);

     Ogre::Box reflectance_src_box(0, 0, width, height);
     Ogre::PixelBox reflectance_dst_box(width, height,
         1, Ogre::PF_FLOAT32_R, this->dataPtr->reflectanceBuffer);

     reflectancePixelBuffer->lock(Ogre::HardwarePixelBuffer::HBL_NORMAL);
     reflectancePixelBuffer->blitToMemory(reflectance_src_box,
                                          reflectance_dst_box);
     reflectancePixelBuffer->unlock();

     this->dataPtr->newReflectanceFrame(
         this->dataPtr->reflectanceBuffer, width, height, 1, "REFLECTANCE");
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
  Ogre::Viewport *vp = nullptr;
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

  if (this->dataPtr->outputReflectance)
  {
    this->dataPtr->reflectanceTarget->update(false);
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
    auto const &ignBG = this->scene->BackgroundColor();
    this->depthViewport->setBackgroundColour(Conversions::Convert(ignBG));
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
event::ConnectionPtr DepthCamera::ConnectNewRGBPointCloud(
    std::function<void (const float *, unsigned int, unsigned int, unsigned int,
    const std::string &)>  _subscriber)
{
  return this->dataPtr->newRGBPointCloud.Connect(_subscriber);
}

//////////////////////////////////////////////////
event::ConnectionPtr DepthCamera::ConnectNewReflectanceFrame(
    std::function<void (const float*, unsigned int, unsigned int, unsigned int,
    const std::string &)>  _subscriber)
{
  return this->dataPtr->newReflectanceFrame.Connect(_subscriber);
}

/////////////////////////////////////////////////
ReflectanceMaterialSwitcher::ReflectanceMaterialSwitcher(
  ScenePtr _scene, Ogre::Viewport* _viewport)
{
  this->viewport = _viewport;
  this->materialScheme = "";

  if (!this->viewport)
  {
    gzerr << "Cannot create a material switcher for the reflectance material. "
          << "viewport is nullptr" << std::endl;
    return;
  }

  this->materialListener.reset(new ReflectanceMaterialListener(_scene));
  this->renderTargetListener.reset(new ReflectanceRenderTargetListener(
      this->materialListener));
}

/////////////////////////////////////////////////
void ReflectanceMaterialSwitcher::SetMaterialScheme(const std::string &_scheme)
{
  if (!this->viewport)
    return;

  this->materialScheme = _scheme;
  if (_scheme.empty())
  {
    this->viewport->setMaterialScheme(
        this->originalMaterialScheme);
    this->viewport->getTarget()->removeListener(
        this->renderTargetListener.get());
  }
  else
  {
    this->originalMaterialScheme =
        this->viewport->getMaterialScheme();

    this->viewport->setMaterialScheme(_scheme);
    this->viewport->getTarget()->addListener(
        this->renderTargetListener.get());
  }
}

/////////////////////////////////////////////////
std::string ReflectanceMaterialSwitcher::MaterialScheme() const
{
  return this->materialScheme;
}

//////////////////////////////////////////////////
ReflectanceRenderTargetListener::ReflectanceRenderTargetListener(
  const ReflectanceMaterialListenerPtr &_switcher)
  :materialListener(_switcher)
{
}

//////////////////////////////////////////////////
void ReflectanceRenderTargetListener::preRenderTargetUpdate(
  const Ogre::RenderTargetEvent &/*_evt*/)
{
  Ogre::MaterialManager::getSingleton().addListener(
      this->materialListener.get());
}

//////////////////////////////////////////////////
void ReflectanceRenderTargetListener::postRenderTargetUpdate(
  const Ogre::RenderTargetEvent & /*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(
      this->materialListener.get());
}

/////////////////////////////////////////////////
ReflectanceMaterialListener::ReflectanceMaterialListener(ScenePtr _scene)
:scene(_scene)
{
}

/////////////////////////////////////////////////
Ogre::Technique *ReflectanceMaterialListener::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String & /*_schemeName*/,
    Ogre::Material *_originalMaterial, uint16_t /*_lodIndex*/,
    const Ogre::Renderable *_rend)
{
  // printf("_schemeName: %s\n", _schemeName.c_str());
  if (_rend && typeid(*_rend) == typeid(Ogre::SubEntity))
  {
    std::string material = "";
    std::string reflectanceMap = "";

    const Ogre::SubEntity *subEntity =
      static_cast<const Ogre::SubEntity *>(_rend);

    if (!subEntity)
    {
      gzerr << "Unable to get an Ogre sub-entity in reflectance "
          << "material listener" << std::endl;
      return nullptr;
    }

    // use the original material for gui visuals
    if (!(subEntity->getParent()->getVisibilityFlags() &
        (GZ_VISIBILITY_ALL &  ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE))))
    {
      Ogre::Technique *originalTechnique = _originalMaterial->getTechnique(0);
      if (originalTechnique)
        return originalTechnique;
    }
    else
    {
      Ogre::Entity *entity = subEntity->getParent();
      if (!entity)
      {
        gzerr << "Unable to get an Ogre entity in reflectance material listener"
            << std::endl;
        return nullptr;
      }

      if (entity->getUserObjectBindings().getUserAny().isEmpty())
        return nullptr;

      std::string userAny = "";
      try
      {
        userAny = Ogre::any_cast<std::string>(
            entity->getUserObjectBindings().getUserAny());
      }
      catch(Ogre::Exception &e)
      {
        gzerr << "Unable to cast Ogre user data in reflectance "
            << "material listener" << std::endl;
        return nullptr;
      }

      rendering::VisualPtr visual = scene->GetVisual(userAny);

      if (!visual)
        return nullptr;

      const Ogre::Any reflectanceMapAny = visual->GetSceneNode()->
                        getUserObjectBindings().getUserAny("reflectance_map");
      if (!reflectanceMapAny.isEmpty())
      {
        material = "Gazebo/Reflectance";
        reflectanceMap = Ogre::any_cast<std::string>(reflectanceMapAny);
      }
      else
      {
        material = "Gazebo/Black";
      }

      // set the material for the models
      Ogre::ResourcePtr res =
          Ogre::MaterialManager::getSingleton().getByName(material);
      if (res.isnullptr())
      {
        Ogre::MaterialManager::getSingleton().load(material,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      }
      Ogre::MaterialPtr mat;
      // OGRE 1.9 changes the shared pointer definition
      #if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))
      // Make sure we keep the same depth properties so that
      // certain overlay objects can be picked by the mouse.
      mat = static_cast<Ogre::MaterialPtr>(res);
      #else
      mat = res.staticCast<Ogre::Material>();
      #endif

      Ogre::Technique *technique = mat->getTechnique(0);
      if (!reflectanceMap.empty())
      {
        Ogre::TextureUnitState *tus = technique->getPass(0)->
                                              getTextureUnitState(0);
        tus->setTextureName(reflectanceMap);
      }

      return technique;
    }
  }
  return nullptr;
}
