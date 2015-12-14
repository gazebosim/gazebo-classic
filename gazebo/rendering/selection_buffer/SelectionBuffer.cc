/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
      /// targets.
      MaterialSwitcher *materialSwitchListener;

      SelectionRenderListener *selectionTargetListener;

      Ogre::SceneManager *sceneMgr;
      Ogre::Camera *camera;
      Ogre::RenderTarget *renderTarget;
      Ogre::TexturePtr texture;
      Ogre::RenderTexture *renderTexture;
      uint8_t *buffer;
      Ogre::PixelBox *pixelBox;
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
  this->dataPtr->renderTexture = 0;
  this->dataPtr->buffer = 0;
  this->dataPtr->pixelBox = 0;

  this->dataPtr->camera = this->dataPtr->sceneMgr->getCamera(_cameraName);
  this->dataPtr->materialSwitchListener = new MaterialSwitcher();
  this->dataPtr->selectionTargetListener = new SelectionRenderListener(
      this->dataPtr->materialSwitchListener);
  this->CreateRTTBuffer();
  this->CreateRTTOverlays();
}

/////////////////////////////////////////////////
SelectionBuffer::~SelectionBuffer()
{
  this->DeleteRTTBuffer();
  delete this->dataPtr->selectionTargetListener;
  delete this->dataPtr->materialSwitchListener;
}

/////////////////////////////////////////////////
void SelectionBuffer::Update()
{
  if (!this->dataPtr->renderTexture)
    return;

  this->UpdateBufferSize();
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
    delete [] this->dataPtr->buffer;
  if (this->dataPtr->pixelBox)
    delete this->dataPtr->pixelBox;
}

/////////////////////////////////////////////////
void SelectionBuffer::CreateRTTBuffer()
{
  unsigned int width = this->dataPtr->renderTarget->getWidth();
  unsigned int height = this->dataPtr->renderTarget->getHeight();

  try
  {
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

  this->dataPtr->renderTexture = this->dataPtr->texture->getBuffer()->getRenderTarget();
  this->dataPtr->renderTexture->setAutoUpdated(false);
  this->dataPtr->renderTexture->setPriority(0);
  this->dataPtr->renderTexture->addViewport(this->dataPtr->camera);
  this->dataPtr->renderTexture->getViewport(0)->setOverlaysEnabled(false);
  this->dataPtr->renderTexture->getViewport(0)->setClearEveryFrame(true);
  this->dataPtr->renderTexture->addListener(this->dataPtr->selectionTargetListener);
  this->dataPtr->renderTexture->getViewport(0)->setMaterialScheme("aa");
  this->dataPtr->renderTexture->getViewport(0)->setVisibilityMask(
      GZ_VISIBILITY_SELECTABLE);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->dataPtr->texture->getBuffer();
  size_t bufferSize = pixelBuffer->getSizeInBytes();

  this->dataPtr->buffer = new uint8_t[bufferSize];
  this->dataPtr->pixelBox = new Ogre::PixelBox(pixelBuffer->getWidth(),
      pixelBuffer->getHeight(), pixelBuffer->getDepth(),
      pixelBuffer->getFormat(), this->dataPtr->buffer);
}

/////////////////////////////////////////////////
void SelectionBuffer::UpdateBufferSize()
{
  if (!this->dataPtr->renderTexture)
    return;

  unsigned int width = this->dataPtr->renderTarget->getWidth();
  unsigned int height = this->dataPtr->renderTarget->getHeight();

  if (width != this->dataPtr->renderTexture->getWidth() ||
     height != this->dataPtr->renderTexture->getHeight())
  {
    this->DeleteRTTBuffer();
    this->CreateRTTBuffer();
  }
}

/////////////////////////////////////////////////
Ogre::Entity *SelectionBuffer::OnSelectionClick(int _x, int _y)
{
  if (!this->dataPtr->renderTexture)
    return NULL;

  if (_x < 0 || _y < 0
      || _x >= static_cast<int>(this->dataPtr->renderTarget->getWidth())
      || _y >= static_cast<int>(this->dataPtr->renderTarget->getHeight()))
    return 0;

  this->Update();
  size_t posInStream = (this->dataPtr->pixelBox->getWidth() * _y) * 4;
  posInStream += _x * 4;
  common::Color::BGRA color(0);
  memcpy(static_cast<void *>(&color), this->dataPtr->buffer + posInStream, 4);
  common::Color cv;
  cv.SetFromARGB(color);

  cv.a = 1.0;
  const std::string &entName = this->dataPtr->materialSwitchListener->GetEntityName(cv);

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
