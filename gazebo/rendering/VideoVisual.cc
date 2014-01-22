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

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/common/Video.hh"
#include "gazebo/rendering/VideoVisual.hh"
#include "gazebo/common/Events.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
VideoVisual::VideoVisual(const std::string &_name, VisualPtr _parent)
  : Visual(_name, _parent)
{
  this->video = new common::Video();
  this->video->Load("/home/nkoenig/Videos/pr2_risotto/risotto_robot.mp4");

  this->width = this->video->GetWidth();
  this->height = this->video->GetHeight();
  double ratio = this->width / static_cast<double>(this->height);

  this->imageBuffer = new unsigned char[this->height * this->width * 3];

  this->connections.push_back(event::Events::ConnectPreRender(
        boost::bind(&VideoVisual::PreRender, this)));

  // Create the texture
  this->texture = Ogre::TextureManager::getSingleton().createManual(
    _name + "__VideoTexture__",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D,
    this->width, this->height,
    0,
    Ogre::PF_BYTE_BGR,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  Ogre::MaterialPtr material =
    Ogre::MaterialManager::getSingleton().create(
        _name + "__VideoMaterial__", "General");
  material->getTechnique(0)->getPass(0)->createTextureUnitState(
      _name + "__VideoTexture__");
  material->setReceiveShadows(false);

  double factor = 1.0;

  Ogre::ManualObject mo(_name + "__VideoObject__");
  mo.begin(_name + "__VideoMaterial__",
           Ogre::RenderOperation::OT_TRIANGLE_LIST);

  mo.position(0, factor * ratio, factor);
  mo.textureCoord(0, 0);

  mo.position(0, 0, factor);
  mo.textureCoord(1, 0);

  mo.position(0, 0, 0);
  mo.textureCoord(1, 1);

  mo.position(0, ratio * factor, 0);
  mo.textureCoord(0, 1);

  mo.triangle(0, 3, 2);
  mo.triangle(2, 1, 0);
  mo.end();

  mo.convertToMesh(_name + "__VideoMesh__");

  Ogre::MovableObject *obj = (Ogre::MovableObject*)
    this->sceneNode->getCreator()->createEntity(_name + "__VideoEntity__",
                                                _name + "__VideoMesh__");
  obj->setCastShadows(false);
  this->AttachObject(obj);
}

/////////////////////////////////////////////////
VideoVisual::~VideoVisual()
{
  delete this->video;
  this->video = NULL;

  delete [] this->imageBuffer;
  this->imageBuffer = NULL;
}

/////////////////////////////////////////////////
void VideoVisual::PreRender()
{
  this->video->GetNextFrame(&this->imageBuffer);

  // Get the pixel buffer
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->texture->getBuffer();

  // Lock the pixel buffer and get a pixel box
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);

  const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
  uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);

  // The request pixel format of the texture is not always the one that OGRE
  // creates.
  bool unusedAlpha = Ogre::PixelUtil::getNumElemBytes(
      this->texture->getFormat()) > 3 ? true : false;

  // If OGRE actually created a texture with no alpha channel, then we
  // can use memcpy
  if (!unusedAlpha)
  {
    memcpy(pDest, this->imageBuffer, this->height*this->width*3);
  }
  else
  {
    int index;
    for (int j = 0; j < this->height; ++j)
    {
      for (int i = 0; i < this->width; ++i)
      {
        index = j*(this->width*3) + (i*3);
        *pDest++ = this->imageBuffer[index + 2];  // B
        *pDest++ = this->imageBuffer[index + 1];  // G
        *pDest++ = this->imageBuffer[index + 0];  // R
        *pDest++ = 255;  // Alpha
      }
    }
  }

  // Unlock the pixel buffer
  pixelBuffer->unlock();
}
