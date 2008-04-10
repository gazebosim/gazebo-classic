/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: A camera sensor using OpenGL
 * Author: Nate Koenig
 * Date: 15 July 2003
 * CVS: $Id: MonoCameraSensor.cc 4436 2008-03-24 17:42:45Z robotos $
 */

#include <sstream>
#include <OgreImageCodec.h>
#include <Ogre.h>

#include "Global.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "OgreFrameListener.hh"

#include "SensorFactory.hh"
#include "CameraManager.hh"
#include "MonoCameraSensor.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("camera", MonoCameraSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
MonoCameraSensor::MonoCameraSensor(Body *body)
    : CameraSensor(body)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
MonoCameraSensor::~MonoCameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void MonoCameraSensor::LoadChild( XMLConfigNode *node )
{
  CameraSensor::LoadChild(node);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void MonoCameraSensor::InitChild()
{

  this->ogreTextureName = this->GetName() + "_RttTex";
  this->ogreMaterialName = this->GetName() + "_RttMat";

  // Create the render texture
  this->renderTexture = Ogre::TextureManager::getSingleton().createManual(
                          this->ogreTextureName,
                          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                          Ogre::TEX_TYPE_2D,
                          this->imageWidth, this->imageHeight, 0,
                          Ogre::PF_R8G8B8,
                          Ogre::TU_RENDERTARGET);

  this->renderTarget = this->renderTexture->getBuffer()->getRenderTarget();

  // Create the camera
  this->camera = OgreCreator::CreateCamera(this->GetName(),
                 this->nearClip, this->farClip, this->hfov, this->renderTarget);

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
                            this->ogreMaterialName,
                            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


  mat->getTechnique(0)->getPass(0)->createTextureUnitState(this->ogreTextureName);

  Ogre::HardwarePixelBufferSharedPtr mBuffer;

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->renderTexture->getBuffer(0, 0);

  this->textureWidth = mBuffer->getWidth();
  this->textureHeight = mBuffer->getHeight();

  CameraSensor::InitChild();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void MonoCameraSensor::FiniChild()
{
  CameraSensor::FiniChild();
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void MonoCameraSensor::UpdateChild(UpdateParams &params)
{
  CameraSensor::UpdateChild(params);

  if (this->saveFrames)
    this->SaveFrame();
}

////////////////////////////////////////////////////////////////////////////////
// Return the material the camera renders to
std::string MonoCameraSensor::GetMaterialName() const
{
  return this->ogreMaterialName;
}


//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const unsigned char *MonoCameraSensor::GetImageData(unsigned int i)
{
  if (i!=0)
    gzerr(0) << "Camera index must be zero for mono cam";

  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  size_t size;

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->renderTexture->getBuffer(0, 0);

  size = this->imageWidth * this->imageHeight * 3;

  // Allocate buffer
  if (!this->saveFrameBuffer)
    this->saveFrameBuffer = new unsigned char[size];

  mBuffer->lock(Ogre::HardwarePixelBuffer::HBL_READ_ONLY);

  int top = (int)((mBuffer->getHeight() - this->imageHeight) / 2.0);
  int left = (int)((mBuffer->getWidth() - this->imageWidth) / 2.0);
  int right = left + this->imageWidth;
  int bottom = top + this->imageHeight;

  // Get the center of the texture in RGB 24 bit format
  mBuffer->blitToMemory(
    Ogre::Box(left, top, right, bottom),

    Ogre::PixelBox(
      this->imageWidth,
      this->imageHeight,
      1,
      Ogre::PF_B8G8R8,
      this->saveFrameBuffer)
  );

  mBuffer->unlock();

  return this->saveFrameBuffer;
}

//////////////////////////////////////////////////////////////////////////////
// Save the current frame to disk
void MonoCameraSensor::SaveFrame()
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  std::ostringstream sstream;
  Ogre::ImageCodec::ImageData *imgData;
  Ogre::Codec * pCodec;
  size_t size, pos;

  this->GetImageData();

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->renderTexture->getBuffer(0, 0);

  // Create image data structure
  imgData  = new Ogre::ImageCodec::ImageData();

  imgData->width = this->imageWidth;
  imgData->height = this->imageHeight;
  imgData->depth = 1;
  imgData->format = Ogre::PF_B8G8R8;
  size = this->GetImageByteSize();

  // Wrap buffer in a chunk
  Ogre::MemoryDataStreamPtr stream(new Ogre::MemoryDataStream( this->saveFrameBuffer, size, false));

  char tmp[1024];
  if (!this->savePathname.empty())
  {
    sprintf(tmp, "%s/%s-%04d.jpg", this->savePathname.c_str(),
            this->GetName().c_str(), this->saveCount);
  }
  else
  {
    sprintf(tmp, "%s-%04d.jpg", this->GetName().c_str(), this->saveCount);
  }

  // Get codec
  Ogre::String filename = tmp;
  pos = filename.find_last_of(".");
  Ogre::String extension;

  while (pos != filename.length() - 1)
    extension += filename[++pos];

  // Get the codec
  pCodec = Ogre::Codec::getCodec(extension);

  // Write out
  Ogre::Codec::CodecDataPtr codecDataPtr(imgData);
  pCodec->codeToFile(stream, filename, codecDataPtr);

  this->saveCount++;
}
