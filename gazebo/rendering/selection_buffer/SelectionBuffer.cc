/*
 * Copyright 2011 Nate Koenig
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
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/selection_buffer/SelectionRenderListener.hh"
#include "gazebo/rendering/selection_buffer/MaterialSwitcher.hh"
#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
SelectionBuffer::SelectionBuffer(const std::string &_cameraName,
    Ogre::SceneManager *_mgr, Ogre::RenderTarget *_renderTarget)
: sceneMgr(_mgr), renderTarget(_renderTarget),
  buffer(0), pixelBox(0)
{
  this->camera = this->sceneMgr->getCamera(_cameraName);
  this->materialSwitchListener = new MaterialSwitcher();
  this->selectionTargetListener = new SelectionRenderListener(
      this->materialSwitchListener);
  unsigned int width = this->renderTarget->getWidth();
  unsigned int height = this->renderTarget->getHeight();

  this->texture = Ogre::TextureManager::getSingleton().createManual(
      "SelectionPassTex",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);

  this->renderTexture = this->texture->getBuffer()->getRenderTarget();
  this->renderTexture->setAutoUpdated(false);
  this->renderTexture->setPriority(0);
  this->renderTexture->addViewport(this->camera);
  this->renderTexture->getViewport(0)->setOverlaysEnabled(false);
  this->renderTexture->getViewport(0)->setClearEveryFrame(true);
  this->renderTexture->getViewport(0)->setMaterialScheme("aa");
  this->renderTexture->getViewport(0)->setVisibilityMask(
      ~GZ_VISIBILITY_NOT_SELECTABLE);

  this->renderTexture->addListener(this->selectionTargetListener);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->texture->getBuffer();

  size_t bufferSize = pixelBuffer->getSizeInBytes();
  this->buffer = new uint8_t[bufferSize];
  this->pixelBox = new Ogre::PixelBox(pixelBuffer->getWidth(),
      pixelBuffer->getHeight(), pixelBuffer->getDepth(),
      pixelBuffer->getFormat(), this->buffer);
  this->CreateRTTOverlays();
}

/////////////////////////////////////////////////
SelectionBuffer::~SelectionBuffer()
{
  Ogre::TextureManager::getSingleton().unload("SelectionPassTex");
  delete this->pixelBox;
  delete [] this->buffer;
  delete this->selectionTargetListener;
  delete this->materialSwitchListener;
}

/////////////////////////////////////////////////
void SelectionBuffer::Update()
{
  this->UpdateBufferSize();
  this->materialSwitchListener->Reset();
  this->renderTexture->update();
  this->renderTexture->copyContentsToMemory(*this->pixelBox,
      Ogre::RenderTarget::FB_FRONT);
}

/////////////////////////////////////////////////
void SelectionBuffer::UpdateBufferSize()
{
  unsigned int width = this->renderTarget->getWidth();
  unsigned int height = this->renderTarget->getHeight();

  if (width != this->renderTexture->getWidth() ||
     height != this->renderTexture->getHeight())
  {
    Ogre::TextureManager::getSingleton().unload("SelectionPassTex");
    delete [] this->buffer;
    delete this->pixelBox;
    this->texture = Ogre::TextureManager::getSingleton().createManual(
        "SelectionPassTex",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    this->renderTexture = this->texture->getBuffer()->getRenderTarget();
    this->renderTexture->setAutoUpdated(false);
    this->renderTexture->setPriority(0);
    this->renderTexture->addViewport(this->camera);
    this->renderTexture->getViewport(0)->setOverlaysEnabled(false);
    this->renderTexture->getViewport(0)->setClearEveryFrame(true);
    this->renderTexture->addListener(this->selectionTargetListener);
    this->renderTexture->getViewport(0)->setMaterialScheme("aa");
    this->renderTexture->getViewport(0)->setVisibilityMask(
        GZ_VISIBILITY_ALL & ~GZ_VISIBILITY_NOT_SELECTABLE);
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->texture->getBuffer();
    size_t bufferSize = pixelBuffer->getSizeInBytes();

    this->buffer = new uint8_t[bufferSize];
    this->pixelBox = new Ogre::PixelBox(pixelBuffer->getWidth(),
        pixelBuffer->getHeight(), pixelBuffer->getDepth(),
        pixelBuffer->getFormat(), this->buffer);
  }
}

/////////////////////////////////////////////////
Ogre::Entity *SelectionBuffer::OnSelectionClick(int _x, int _y)
{
  if (_x < 0 || _y < 0 )
    return 0;

  this->Update();
  size_t posInStream = (this->pixelBox->getWidth() * _y) * 4;
  posInStream += _x * 4;
  common::Color::BGRA color(0);
  memcpy(static_cast<void *>(&color), this->buffer + posInStream, 4);
  common::Color cv;
  cv.SetFromARGB(color);

  cv.a = 1.0;
  const std::string &entName = this->materialSwitchListener->GetEntityName(cv);

  if (entName.empty())
    return 0;
  else
    return this->sceneMgr->getEntity(entName);
}

/////////////////////////////////////////////////
void SelectionBuffer::CreateRTTOverlays()
{
  Ogre::OverlayManager *mgr = Ogre::OverlayManager::getSingletonPtr();

  if (mgr->getByName("SelectionDebugOverlay"))
    return;

  Ogre::MaterialPtr baseWhite =
    Ogre::MaterialManager::getSingleton().getDefaultSettings();

  Ogre::MaterialPtr selectionBufferTexture =
    baseWhite->clone("SelectionDebugMaterial");
  Ogre::TextureUnitState *textureUnit =
    selectionBufferTexture->getTechnique(0)->getPass(0)->
    createTextureUnitState();
  textureUnit->setTextureName("SelectionPassTex");

  this->selectionDebugOverlay = mgr->create("SelectionDebugOverlay");

  Ogre::OverlayContainer *panel =
    static_cast<Ogre::OverlayContainer *>(
        mgr->createOverlayElement("Panel", "SelectionDebugPanel"));

  panel->setMetricsMode(Ogre::GMM_PIXELS);
  panel->setPosition(10, 10);
  panel->setDimensions(400, 280);
  panel->setMaterialName("SelectionDebugMaterial");
  this->selectionDebugOverlay->add2D(panel);
  this->selectionDebugOverlay->hide();
}

/////////////////////////////////////////////////
void SelectionBuffer::ShowOverlay(bool _show)
{
  if (_show)
    this->selectionDebugOverlay->show();
  else
    this->selectionDebugOverlay->hide();
}
