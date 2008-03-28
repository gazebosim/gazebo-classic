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
#include <GL/gl.h>
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

  this->baseline = node->GetDouble("baseline",0,1);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void StereoCameraSensor::InitChild()
{
  this->ogreTextureName[0] = this->GetName() + "_RttTex_Stereo_Left";
  //this->ogreTextureName[1] = this->GetName() + "_RttTex_Stereo_Right";

  this->ogreDepthTextureName[0] = this->GetName() +"_RttTex_Stereo_Left_Depth";
  //this->ogreDepthTextureName[1] = this->GetName() +"_RttTex_Stereo_Right_Depth";

  this->ogreMaterialName[0] = this->GetName() + "_RttMat_Stereo_Left";
  //this->ogreMaterialName[1] = this->GetName() + "_RttMat_Stereo_Right";

  this->ogreDepthMaterialName[0] = this->GetName()+"_RttMat_Stereo_Left_Depth";
  //this->ogreDepthMaterialName[1] = this->GetName()+"_RttMat_Stereo_Right_Depth";


  // Create the left render texture
  this->renderTexture[0] = Ogre::TextureManager::getSingleton().createManual(
                          this->ogreTextureName[0],
                          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                          Ogre::TEX_TYPE_2D,
                          this->imageWidth, this->imageHeight, 0,
                          Ogre::PF_R8G8B8,
                          //Ogre::PF_L8,
                          //Ogre::PF_FLOAT16_R,
                          Ogre::TU_RENDERTARGET);

  this->renderTarget[0] = this->renderTexture[0]->getBuffer()->getRenderTarget();
/*
  // Create the right render texture
  this->renderTexture[1] = Ogre::TextureManager::getSingleton().createManual(
                          this->ogreTextureName[1],
                          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                          Ogre::TEX_TYPE_2D,
                          this->imageWidth, this->imageHeight, 0,
                          Ogre::PF_R8G8B8,
                          Ogre::TU_RENDERTARGET);

  this->renderTarget[1] = this->renderTexture[1]->getBuffer()->getRenderTarget();
  */

  // Create the left depth render texture
  /*this->depthRenderTexture[0] = 
    Ogre::TextureManager::getSingleton().createManual(
        this->ogreDepthTextureName[0],
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        this->imageWidth, this->imageHeight, 0,
        Ogre::PF_FLOAT32_R,
        //Ogre::PF_DEPTH,
        //Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

  this->depthRenderTarget[0] = this->depthRenderTexture[0]->getBuffer()->getRenderTarget();
  */

  // Create the right depth render texture
  /*this->depthRenderTexture[1] = 
    Ogre::TextureManager::getSingleton().createManual(
        this->ogreDepthTextureName[1],
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        this->imageWidth, this->imageHeight, 0,
        Ogre::PF_FLOAT32_R,
        //Ogre::PF_DEPTH,
        //Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

  this->depthRenderTarget[1] = this->depthRenderTexture[1]->getBuffer()->getRenderTarget();
  */


  // Create the camera
  this->camera = OgreCreator::CreateCamera(this->GetName(),
                 this->nearClip, this->farClip, this->hfov, this->renderTarget[0]);


  // Hack to make the camera use the right render target too
  /*{
    Ogre::Viewport *cviewport;

    // Setup the viewport to use the texture
    cviewport = this->renderTarget[1]->addViewport(camera);
    cviewport->setClearEveryFrame(true);
    cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
    cviewport->setOverlaysEnabled(false);

    cviewport = this->depthRenderTarget[0]->addViewport(camera);
    cviewport->setClearEveryFrame(true);
    cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
    cviewport->setOverlaysEnabled(false);

    cviewport = this->depthRenderTarget[1]->addViewport(camera);
    cviewport->setClearEveryFrame(true);
    cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
    cviewport->setOverlaysEnabled(false);
  }*/

  Ogre::MaterialPtr leftmat = Ogre::MaterialManager::getSingleton().create(
                      this->ogreMaterialName[0],
                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  Ogre::MaterialPtr depthMat = Ogre::MaterialManager::getSingleton().getByName("Gazebo/DepthMap");
  
  /*Ogre::MaterialPtr rightmat = Ogre::MaterialManager::getSingleton().create(
                      this->ogreMaterialName[1],
                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                      */
 
  //depthMat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(this->renderTexture[0]->getName());
  depthMat->getTechnique(0)->getPass(0)->createTextureUnitState(this->renderTexture[0]->getName());

  /*leftmat->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
  leftmat->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  Ogre::TextureUnitState *texUnit = leftmat->getTechnique(0)->getPass(0)->createTextureUnitState(this->ogreTextureName[0]);
  texUnit->setTextureBorderColour(Ogre::ColourValue::White);
  texUnit->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
  */
  
  //rightmat->getTechnique(0)->getPass(0)->createTextureUnitState(this->ogreTextureName[1]);

  //Ogre::MaterialPtr leftdepthmat = Ogre::MaterialManager::getSingleton().create(
                      //this->ogreDepthMaterialName[0],
                      //Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  /*Ogre::MaterialPtr rightdepthmat=Ogre::MaterialManager::getSingleton().create(
                      this->ogreDepthMaterialName[1],
                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                      */

  //Ogre::TextureUnitState *t = leftdepthmat->getTechnique(0)->getPass(0)->createTextureUnitState(this->ogreDepthTextureName[0]);
  //Ogre::TextureUnitState *t = leftdepthmat->getTechnique(0)->getPass(0)->createTextureUnitState(OgreAdaptor::Instance()->sceneMgr->getShadowTexture(0)->getName());
  //leftdepthmat->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  //t->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  //leftdepthmat->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
  //leftdepthmat->getTechnique(0)->getPass(0)->setColourWriteEnabled(false);

  //rightdepthmat->getTechnique(0)->getPass(0)->createTextureUnitState(this->ogreDepthTextureName[1]);
  //rightdepthmat->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  //rightdepthmat->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
  //rightdepthmat->getTechnique(0)->getPass(0)->setColourWriteEnabled(false);


  Ogre::HardwarePixelBufferSharedPtr mBuffer;

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->renderTexture[0]->getBuffer(0, 0);

  this->textureWidth = mBuffer->getWidth();
  this->textureHeight = mBuffer->getHeight();

  //this->leftCameraListener.Init(this, this->renderTarget[0], true);
  //this->rightCameraListener.Init(this, this->renderTarget[1], false);

  //this->renderTarget[0]->addListener(&this->leftCameraListener);
  //this->renderTarget[1]->addListener(&this->rightCameraListener);

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

  this->renderTarget[0]->writeContentsToFile("texture.png");
}

////////////////////////////////////////////////////////////////////////////////
// Return the material the camera renders to
std::string StereoCameraSensor::GetMaterialName() const
{
  return "Gazebo/DepthMap";//this->ogreMaterialName[0];
}

//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const unsigned char *StereoCameraSensor::GetImageData(unsigned int i)
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  size_t size;

  if (i > 1)
  {
    gzmsg(0) <<  "Camera index must be 0=Left or 1=Right for stereo camera";
    i = 1;
  }

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->depthRenderTexture[i]->getBuffer(0, 0);

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

//  for (int i=0; i<2; i++)
  {
    this->GetImageData(0);

    // Get access to the buffer and make an image and write it to file
    mBuffer = this->depthRenderTexture[0]->getBuffer(0, 0);

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
      //if (i==0)
        sprintf(tmp, "%s/%s-%04d-left.png", this->savePathname.c_str(),
            this->GetName().c_str(), this->saveCount);
      //else
        //sprintf(tmp, "%s/%s-%04d-right.png", this->savePathname.c_str(),
            //this->GetName().c_str(), this->saveCount);
    }
    else
    {
      //if (i==0)
        sprintf(tmp, "%s-%04d-left.png", this->GetName().c_str(), this->saveCount);
      //else
       // sprintf(tmp, "%s-%04d-right.png", this->GetName().c_str(), this->saveCount);
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
  }

  this->saveCount++;
}

//void StereCameraSensor::UpdateAllDependentRenderTargets()
//{
/*  Ogre::RenderTargetList::iterator iter;

  for( iter = mRenderTargetList.begin(); iter != mRenderTargetList.end(); ++iter )
  {                                                                                 (*iter)->update();
  }
  */
//}

////////////////////////////////////////////////////////////////////////////////
/// Get the baselien of the camera
double StereoCameraSensor::GetBaseline() const
{
  return this->baseline;
}


void StereoCameraSensor::StereoCameraListener::Init(
    StereoCameraSensor *cam, Ogre::RenderTarget *target, bool isLeft)
{
  this->sensor = cam;
  this->camera = this->sensor->GetOgreCamera();
  this->renderTarget = target;
  this->isLeftCamera = isLeft;
}

void StereoCameraSensor::StereoCameraListener::preViewportUpdate(const Ogre::RenderTargetViewportEvent &evt)
{
	if(evt.source != this->renderTarget->getViewport(0))
  {
    printf("Invalid viewport");
		return;
  }

	double offset = this->sensor->GetBaseline()/2;

	if(this->isLeftCamera)
	{
		offset = -offset;
	}

	this->camera->setFrustumOffset(-offset,0);

	this->pos = this->camera->getPosition();
  Ogre::Vector3 pos = this->pos;

	pos += offset * this->camera->getRight();
	this->camera->setPosition(pos);

	//this->sensor->UpdateAllDependentRenderTargets();
	//this->sensor->chooseDebugPlaneMaterial(mIsLeftEye);
}

void StereoCameraSensor::StereoCameraListener::postViewportUpdate(const Ogre::RenderTargetViewportEvent &evt)
{
  //this->sensor->ReadDepthImage();

	this->camera->setFrustumOffset(0,0);
	this->camera->setPosition(this->pos);
}
