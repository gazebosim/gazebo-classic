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
#include <memory>

#include "gazebo/common/Console.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/selection_buffer/SelectionRenderListener.hh"
#include "gazebo/rendering/selection_buffer/MaterialSwitcher.hh"
#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"

using namespace gazebo;
using namespace rendering;

namespace gazebo
{
  namespace rendering
  {
    struct SelectionBufferPrivate
    {
      /// \brief This is the material listener - Note: it is controlled by a
      /// separate RenderTargetListener, not applied globally to all
      /// targets. The class associates a color to an ogre entity
      std::unique_ptr<MaterialSwitcher> materialSwitchListener;

      /// \brief A render target listener that sets up the material switcher
      /// to run on every update of this render target.
      std::unique_ptr<SelectionRenderListener> selectionTargetListener;

      /// \brief Ogre scene manager
      Ogre::SceneManager *sceneMgr = nullptr;

      /// \brief Pointer to the camera that will be used as the reference
      /// for selection
      Ogre::Camera *camera = nullptr;

      /// \brief Selection buffer's render to texture camera
      Ogre::Camera *selectionCamera  = nullptr;

      /// \brief Ogre render target
      Ogre::RenderTarget *renderTarget  = nullptr;

      /// \brief Ogre texture
      Ogre::TexturePtr texture;

      /// \brief Ogre render texture
      Ogre::RenderTexture *renderTexture  = nullptr;

      /// \brief Render texture data buffer
      uint8_t *buffer = nullptr;

      /// \brief Ogre pixel box that contains description of the data buffer
      Ogre::PixelBox *pixelBox = nullptr;

      /// \brief A 2D overlay used for debugging the selection buffer. It
      /// is hidden by default.
      Ogre::Overlay *selectionDebugOverlay;
    };
  }
}

/////////////////////////////////////////////////
SelectionBuffer::SelectionBuffer(const std::string &_cameraName,
    Ogre::SceneManager *_mgr, Ogre::RenderTarget *_renderTarget)
: dataPtr(new SelectionBufferPrivate)
{
  this->dataPtr->sceneMgr = _mgr;
  this->dataPtr->renderTarget = _renderTarget;

  this->dataPtr->camera = this->dataPtr->sceneMgr->getCamera(_cameraName);

  this->dataPtr->selectionCamera =
      this->dataPtr->sceneMgr->createCamera(_cameraName + "_selection_buffer");

  this->dataPtr->materialSwitchListener.reset(new MaterialSwitcher());
  this->dataPtr->selectionTargetListener.reset(new SelectionRenderListener(
      this->dataPtr->materialSwitchListener.get()));
  this->CreateRTTBuffer();
  this->CreateRTTOverlays();
}

/////////////////////////////////////////////////
SelectionBuffer::~SelectionBuffer()
{
  this->DeleteRTTBuffer();

  // remove selection buffer camera
  this->dataPtr->sceneMgr->destroyCamera(this->dataPtr->selectionCamera);
}

/////////////////////////////////////////////////
void SelectionBuffer::Update()
{
  if (!this->dataPtr->renderTexture)
    return;

  this->dataPtr->materialSwitchListener->Reset();

  // FIXME: added try-catch block to prevent crash in deferred rendering mode.
  // RTT does not like VPL.material as it references a texture in the compositor
  // pipeline. A possible workaround is to add the deferred rendering
  // compositors to the renderTexture
  try
  {
    this->dataPtr->renderTexture->update();
  }
  catch(...)
  {
  }

  this->dataPtr->renderTexture->copyContentsToMemory(*this->dataPtr->pixelBox,
      Ogre::RenderTarget::FB_FRONT);
}

/////////////////////////////////////////////////
void SelectionBuffer::DeleteRTTBuffer()
{
  if (!this->dataPtr->texture.isNull() && this->dataPtr->texture->isLoaded())
    this->dataPtr->texture->unload();
  if (this->dataPtr->buffer)
  {
    delete [] this->dataPtr->buffer;
    this->dataPtr->buffer = nullptr;
  }
  if (this->dataPtr->pixelBox)
    delete this->dataPtr->pixelBox;
}

/////////////////////////////////////////////////
void SelectionBuffer::CreateRTTBuffer()
{
  try
  {
    // 1x1 pixel buffer
    unsigned int width = 1;
    unsigned int height = 1;

    this->dataPtr->texture = Ogre::TextureManager::getSingleton().createManual(
        "SelectionPassTex",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
  }
  catch(...)
  {
    this->dataPtr->renderTexture = NULL;
    gzerr << "Unable to create selection buffer.\n";
    return;
  }

  this->dataPtr->renderTexture =
    this->dataPtr->texture->getBuffer()->getRenderTarget();
  this->dataPtr->renderTexture->setAutoUpdated(false);
  this->dataPtr->renderTexture->setPriority(0);
  this->dataPtr->renderTexture->addViewport(this->dataPtr->selectionCamera);
  this->dataPtr->renderTexture->getViewport(0)->setOverlaysEnabled(false);
  this->dataPtr->renderTexture->getViewport(0)->setClearEveryFrame(true);
  this->dataPtr->renderTexture->addListener(
      this->dataPtr->selectionTargetListener.get());
  this->dataPtr->renderTexture->getViewport(0)->setMaterialScheme("aa");
  this->dataPtr->renderTexture->getViewport(0)->setVisibilityMask(
      GZ_VISIBILITY_SELECTABLE);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer =
    this->dataPtr->texture->getBuffer();
  size_t bufferSize = pixelBuffer->getSizeInBytes();

  this->dataPtr->buffer = new uint8_t[bufferSize];
  this->dataPtr->pixelBox = new Ogre::PixelBox(pixelBuffer->getWidth(),
      pixelBuffer->getHeight(), pixelBuffer->getDepth(),
      pixelBuffer->getFormat(), this->dataPtr->buffer);
}

/////////////////////////////////////////////////
Ogre::Entity *SelectionBuffer::OnSelectionClick(int _x, int _y)
{
  if (!this->dataPtr->renderTexture)
    return NULL;

  unsigned int targetWidth = this->dataPtr->renderTarget->getWidth();
  unsigned int targetHeight = this->dataPtr->renderTarget->getHeight();

  if (_x < 0 || _y < 0 || _x >= static_cast<int>(targetWidth)
      || _y >= static_cast<int>(targetHeight))
    return nullptr;

  // 1x1 selection buffer, adapted from rviz
  // http://docs.ros.org/indigo/api/rviz/html/c++/selection__manager_8cpp.html
  unsigned int width = 1;
  unsigned int height = 1;
  float x1 = static_cast<float>(_x) /
      static_cast<float>(targetWidth - 1) - 0.5f;
  float y1 = static_cast<float>(_y) /
      static_cast<float>(targetHeight - 1) - 0.5f;
  float x2 = static_cast<float>(_x+width) /
      static_cast<float>(targetWidth - 1) - 0.5f;
  float y2 = static_cast<float>(_y+height) /
      static_cast<float>(targetHeight - 1) - 0.5f;
  Ogre::Matrix4 scaleMatrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 transMatrix = Ogre::Matrix4::IDENTITY;
  scaleMatrix[0][0] = 1.0 / (x2-x1);
  scaleMatrix[1][1] = 1.0 / (y2-y1);
  transMatrix[0][3] -= x1+x2;
  transMatrix[1][3] += y1+y2;
  this->dataPtr->selectionCamera->setCustomProjectionMatrix(true,
      scaleMatrix * transMatrix * this->dataPtr->camera->getProjectionMatrix());
  this->dataPtr->selectionCamera->setPosition(
      this->dataPtr->camera->getDerivedPosition());
  this->dataPtr->selectionCamera->setOrientation(
      this->dataPtr->camera->getDerivedOrientation());
  Ogre::Viewport* renderViewport = this->dataPtr->renderTexture->getViewport(0);
  renderViewport->setDimensions(0, 0, width, height);

  // update render texture
  this->Update();

  size_t posInStream = 0;

  common::Color::BGRA color(0);
  if (!this->dataPtr->buffer)
  {
    gzerr << "Selection buffer is null.\n";
    return nullptr;
  }
  memcpy(static_cast<void *>(&color), this->dataPtr->buffer + posInStream, 4);
  common::Color cv;
  cv.SetFromARGB(color);
  cv.a = 1.0;
  const std::string &entName =
    this->dataPtr->materialSwitchListener->GetEntityName(cv);

  if (entName.empty())
    return 0;
  else
    return this->dataPtr->sceneMgr->getEntity(entName);
}

/////////////////////////////////////////////////
void SelectionBuffer::CreateRTTOverlays()
{
  Ogre::OverlayManager *mgr = Ogre::OverlayManager::getSingletonPtr();

  if (mgr && mgr->getByName("SelectionDebugOverlay"))
    return;

  Ogre::MaterialPtr baseWhite =
    Ogre::MaterialManager::getSingleton().getDefaultSettings();

  Ogre::MaterialPtr selectionBufferTexture =
    baseWhite->clone("SelectionDebugMaterial");
  Ogre::TextureUnitState *textureUnit =
    selectionBufferTexture->getTechnique(0)->getPass(0)->
    createTextureUnitState();
  textureUnit->setTextureName("SelectionPassTex");

  this->dataPtr->selectionDebugOverlay = mgr->create("SelectionDebugOverlay");

  Ogre::OverlayContainer *panel =
    static_cast<Ogre::OverlayContainer *>(
        mgr->createOverlayElement("Panel", "SelectionDebugPanel"));

  if (panel)
  {
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setPosition(10, 10);
    panel->setDimensions(400, 280);
    panel->setMaterialName("SelectionDebugMaterial");
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR  <= 9
    this->dataPtr->selectionDebugOverlay->add2D(panel);
    this->dataPtr->selectionDebugOverlay->hide();
#endif
  }
  else
  {
    gzlog << "Unable to create selection buffer overlay. "
      "This will not effect Gazebo unless you're trying to debug "
      "the selection buffer.\n";
  }
}

/////////////////////////////////////////////////
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR  <= 9
void SelectionBuffer::ShowOverlay(bool _show)
{
  if (_show)
    this->dataPtr->selectionDebugOverlay->show();
  else
    this->dataPtr->selectionDebugOverlay->hide();
}
#else
void SelectionBuffer::ShowOverlay(bool /*_show*/)
{
  gzerr << "Selection debug overlay disabled for Ogre > 1.9\n";
}
#endif
