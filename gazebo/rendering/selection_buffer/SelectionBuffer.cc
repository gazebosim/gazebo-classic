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

#include "gazebo/common/Console.hh"
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
  this->CreateRTTBuffer();
  this->CreateRTTOverlays();
}

/////////////////////////////////////////////////
SelectionBuffer::~SelectionBuffer()
{
  this->DeleteRTTBuffer();
  delete this->selectionTargetListener;
  delete this->materialSwitchListener;
}

/////////////////////////////////////////////////
void SelectionBuffer::Update()
{
  if (!this->renderTexture)
    return;

  this->UpdateBufferSize();
  this->materialSwitchListener->Reset();

  // FIXME: added try-catch block to prevent crash in deferred rendering mode.
  // RTT does not like VPL.material as it references a texture in the compositor
  // pipeline. A possible workaround is to add the deferred rendering
  // compositors to the renderTexture
  try
  {
    this->renderTexture->update();
  }
  catch(...)
  {
  }

  this->renderTexture->copyContentsToMemory(*this->pixelBox,
      Ogre::RenderTarget::FB_FRONT);
}

/////////////////////////////////////////////////
void SelectionBuffer::DeleteRTTBuffer()
{
  if (!this->texture.isNull() && this->texture->isLoaded())
    this->texture->unload();
  if (this->buffer)
    delete [] this->buffer;
  if (this->pixelBox)
    delete this->pixelBox;
}

/////////////////////////////////////////////////
void SelectionBuffer::CreateRTTBuffer()
{
  unsigned int width = this->renderTarget->getWidth();
  unsigned int height = this->renderTarget->getHeight();

  try
  {
    this->texture = Ogre::TextureManager::getSingleton().createManual(
        "SelectionPassTex",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
  }
  catch(...)
  {
    this->renderTexture = NULL;
    gzerr << "Unable to create selection buffer.\n";
    return;
  }

  this->renderTexture = this->texture->getBuffer()->getRenderTarget();
  this->renderTexture->setAutoUpdated(false);
  this->renderTexture->setPriority(0);
  this->renderTexture->addViewport(this->camera);
  this->renderTexture->getViewport(0)->setOverlaysEnabled(false);
  this->renderTexture->getViewport(0)->setClearEveryFrame(true);
  this->renderTexture->addListener(this->selectionTargetListener);
  this->renderTexture->getViewport(0)->setMaterialScheme("aa");
  this->renderTexture->getViewport(0)->setVisibilityMask(
      GZ_VISIBILITY_SELECTABLE);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->texture->getBuffer();
  size_t bufferSize = pixelBuffer->getSizeInBytes();

  this->buffer = new uint8_t[bufferSize];
  this->pixelBox = new Ogre::PixelBox(pixelBuffer->getWidth(),
      pixelBuffer->getHeight(), pixelBuffer->getDepth(),
      pixelBuffer->getFormat(), this->buffer);
}

/////////////////////////////////////////////////
void SelectionBuffer::UpdateBufferSize()
{
  if (!this->renderTexture)
    return;

  unsigned int width = this->renderTarget->getWidth();
  unsigned int height = this->renderTarget->getHeight();

  if (width != this->renderTexture->getWidth() ||
     height != this->renderTexture->getHeight())
  {
    this->DeleteRTTBuffer();
    this->CreateRTTBuffer();
  }
}

/////////////////////////////////////////////////
Ogre::Entity *SelectionBuffer::OnSelectionClick(int _x, int _y)
{
  if (!this->renderTexture)
    return NULL;

  if (_x < 0 || _y < 0
      || _x >= static_cast<int>(this->renderTarget->getWidth())
      || _y >= static_cast<int>(this->renderTarget->getHeight()))
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

  this->selectionDebugOverlay = mgr->create("SelectionDebugOverlay");

  Ogre::OverlayContainer *panel =
    static_cast<Ogre::OverlayContainer *>(
        mgr->createOverlayElement("Panel", "SelectionDebugPanel"));

  if (panel)
  {
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setPosition(10, 10);
    panel->setDimensions(400, 280);
    panel->setMaterialName("SelectionDebugMaterial");
#if OGRE_VERSION_MAJOR > 1 && OGRE_VERSION_MINOR  <= 9
    this->selectionDebugOverlay->add2D(panel);
    this->selectionDebugOverlay->hide();
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
#if OGRE_VERSION_MAJOR > 1 && OGRE_VERSION_MINOR  <= 9
void SelectionBuffer::ShowOverlay(bool _show)
{
  if (_show)
    this->selectionDebugOverlay->show();
  else
    this->selectionDebugOverlay->hide();
}
#else
void SelectionBuffer::ShowOverlay(bool /*_show*/)
{
  gzerr << "Selection debug overlay disabled for Ogre > 1.9\n";
}
#endif
