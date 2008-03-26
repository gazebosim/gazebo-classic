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

/* Desc: Stereo Camera Sensor
 * Author: Nate Koenig
 * Date: 25 March 2008
 * CVS: $Id:$
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
#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"

#include "SensorFactory.hh"
#include "CameraManager.hh"
#include "StereoCameraSensor.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("stereocamera", StereoCameraSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
StereoCameraSensor::StereoCameraSensor(Body *body)
    : CameraSensor(body)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
StereoCameraSensor::~StereoCameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void StereoCameraSensor::LoadChild( XMLConfigNode *node )
{
  CameraSensor::LoadChild(node);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void StereoCameraSensor::InitChild()
{
  this->leftOgreTextureName = this->GetName() + "_LEFTRttTex";
  this->rightOgreTextureName = this->GetName() + "_RIGHTRttTex";

  this->ogreMaterialName = this->GetName() + "_RttMat";

  // Create the render texture
  this->leftRenderTexture = Ogre::TextureManager::getSingleton().createManual(
                          this->leftOgreTextureName,
                          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                          Ogre::TEX_TYPE_2D,
                          this->imageWidth, this->imageHeight, 0,
                          Ogre::PF_R8G8B8,
                          Ogre::TU_RENDERTARGET);

  this->leftRenderTarget = this->leftRenderTexture->getBuffer()->getRenderTarget();

  this->rightRenderTexture = Ogre::TextureManager::getSingleton().createManual(
                          this->rightOgreTextureName,
                          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                          Ogre::TEX_TYPE_2D,
                          this->imageWidth, this->imageHeight, 0,
                          Ogre::PF_R8G8B8,
                          Ogre::TU_RENDERTARGET);

  this->rightRenderTarget = this->rightRenderTexture->getBuffer()->getRenderTarget();

  // Create the camera
  this->camera = OgreCreator::CreateCamera(this->GetName(),
                 this->nearClip, this->farClip, this->hfov, this->leftRenderTarget);

  // Hack to make the camera use the right render target too
  {
    Ogre::Viewport *cviewport;

    // Setup the viewport to use the texture
    cviewport = this->rightRenderTarget->addViewport(camera);
    cviewport->setClearEveryFrame(true);
    cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
    cviewport->setOverlaysEnabled(false);
  }

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
                            this->ogreMaterialName,
                            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  Ogre::HardwarePixelBufferSharedPtr mBuffer;

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->leftRenderTexture->getBuffer(0, 0);

  this->textureWidth = mBuffer->getWidth();
  this->textureHeight = mBuffer->getHeight();

  this->leftCameraListener.Init(this, true);
  this->rightCameraListener.Init(this, false);

  this->leftRenderTarget->addListener(&this->leftCameraListener);
  this->rightRenderTarget->addListener(&this->rightCameraListener);

  CameraSensor::InitChild();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void StereoCameraSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void StereoCameraSensor::UpdateChild(UpdateParams &params)
{
  CameraSensor::UpdateChild(params);
}

////////////////////////////////////////////////////////////////////////////////
// Return the material the camera renders to
std::string StereoCameraSensor::GetMaterialName() const
{
  return this->ogreMaterialName;
}

//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const unsigned char *StereoCameraSensor::GetImageData()
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  size_t size;

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->leftRenderTexture->getBuffer(0, 0);

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
void StereoCameraSensor::SaveFrame()
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  std::ostringstream sstream;
  Ogre::ImageCodec::ImageData *imgData;
  Ogre::Codec * pCodec;
  size_t size, pos;

  this->GetImageData();

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->leftRenderTexture->getBuffer(0, 0);

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

void StereoCameraSensor::StereoCameraListener::Init(
    StereoCameraSensor *cam, bool isLeft)
{
  this->sensor = cam;
  this->camera = this->sensor->GetOgreCamera();
  this->isLeftCamera = isLeft;
}

void StereoCameraSensor::StereoCameraListener::preViewportUpdate(const Ogre::RenderTargetViewportEvent &evt)
{
  if (this->isLeftCamera)
    printf("Left Pre\n");
  else
    printf("Rightt Pre\n");

/*	if(evt.source != mViewport)
		return;
	Real offset = mStereoMgr->getEyesSpacing()/2;
	if(mIsLeftEye)
	{
		offset = -offset;
	}
	mCamera->setFrustumOffset(-offset,0);
	mPos = mCamera->getPosition();
	Vector3 pos = mPos;
	pos += offset * mCamera->getRight();
	mCamera->setPosition(pos);
	mStereoMgr->updateAllDependentRenderTargets();
	mStereoMgr->chooseDebugPlaneMaterial(mIsLeftEye);
  */

}

void StereoCameraSensor::StereoCameraListener::postViewportUpdate(const Ogre::RenderTargetViewportEvent &evt)
{
  if (this->isLeftCamera)
    printf("Left Post\n");
  else
    printf("Rightt Post\n");

/*	mCamera->setFrustumOffset(0,0);
	mCamera->setPosition(mPos);
  */

}
