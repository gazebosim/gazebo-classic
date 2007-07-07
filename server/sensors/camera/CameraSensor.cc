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
 * CVS: $Id: Camera.cc 46 2007-06-10 00:12:44Z natepak $
 */

#include <sstream>
#include <OgreImageCodec.h>
#include <Ogre.h>

#include "Global.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"

#include "SensorFactory.hh"
#include "CameraManager.hh"
#include "CameraSensor.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor 
CameraSensor::CameraSensor(Body *body)
  : Sensor(body)
{
  this->imageWidth = this->imageHeight = 0;

  this->saveFrameBuffer = NULL;
  this->saveCount = 0;
  this->saveFrames = false;

  CameraManager::Instance()->AddCamera(this);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
CameraSensor::~CameraSensor()
{
  if (this->saveFrameBuffer)
    delete [] this->saveFrameBuffer;
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void CameraSensor::LoadChild( XMLConfigNode *node )
{
  this->nearClip = node->GetDouble("nearClip",0.1,0);
  this->farClip = node->GetDouble("farClip",100,0);
  this->saveFrames = node->GetBool("saveFrames",false,0);
  this->savePathname = node->GetString("saveFramePath","",0);

  this->imageWidth = node->GetTupleInt("imageSize",0,640);
  this->imageHeight = node->GetTupleInt("imageSize",1,480);

  this->hfov = DTOR(node->GetDouble("hfov",60,0));

  // Create the directory to store frames
  if (this->saveFrames)
  {
    char tmp[1024];
    sprintf(tmp, "mkdir %s 2>>/dev/null", this->savePathname.c_str());
    system(tmp);
  }

  // Do some sanity checks
  if (this->imageWidth == 0 || this->imageHeight == 0)
  {
    gzthrow("image has zero size");
  }
  if (this->hfov < 0.01 || this->hfov > M_PI)
  {
    gzthrow("Camera horizontal field of veiw invalid.");
  }
  if (this->nearClip < 0.01)
  {
    gzthrow("near clipping plane (min depth) is zero");
  }

}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void CameraSensor::InitChild()
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
  this->camera = OgreAdaptor::Instance()->sceneMgr->createCamera(this->GetName());
  this->camera->setNearClipDistance(this->nearClip);
  this->camera->setFarClipDistance(this->farClip);


  // Setup the viewport to use the texture
  this->viewport = this->renderTarget->addViewport(this->camera);
  this->viewport->setClearEveryFrame(true);
  this->viewport->setBackgroundColour( Ogre::ColourValue::Black );
  this->viewport->setOverlaysEnabled(false);

  
  this->camera->setAspectRatio(
      Ogre::Real(this->viewport->getActualWidth()) / 
      Ogre::Real(this->viewport->getActualHeight()) );


  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
      this->ogreMaterialName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  Ogre::TextureUnitState *t = mat->getTechnique(0)->getPass(0)->createTextureUnitState(this->ogreTextureName);


  // Create a scene node to control pitch motion
  this->pitchNode = this->sceneNode->createChildSceneNode(this->GetName() + "PitchNode");
  this->pitchNode->attachObject(this->camera);
  this->pitchNode->pitch(Ogre::Degree(0));

  this->saveCount = 0;
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void CameraSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void CameraSensor::UpdateChild(UpdateParams &params)
{  
  this->pose.pos.x = this->sceneNode->getWorldPosition().x;
  this->pose.pos.y = this->sceneNode->getWorldPosition().y;
  this->pose.pos.z = this->sceneNode->getWorldPosition().z;
  this->pose.rot.u = this->sceneNode->getWorldOrientation().w;
  this->pose.rot.x = this->sceneNode->getWorldOrientation().x;
  this->pose.rot.y = this->sceneNode->getWorldOrientation().y;
  this->pose.rot.z = this->sceneNode->getWorldOrientation().z;

  if (this->saveFrames)
    this->SaveFrame();
}

////////////////////////////////////////////////////////////////////////////////
// Get the global pose of the camera
Pose3d CameraSensor::GetWorldPose() const
{
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
// Return the material the camera renders to
std::string CameraSensor::GetMaterialName() const
{
  return this->ogreMaterialName;
}

////////////////////////////////////////////////////////////////////////////////
// Translate the camera
void CameraSensor::Translate( const Vector3 &direction )
{
  Ogre::Vector3 vec(direction.x, direction.y, direction.z);

  this->sceneNode->translate(this->sceneNode->getOrientation() * this->pitchNode->getOrientation() * vec);

}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the yaw axis
void CameraSensor::RotateYaw( float angle )
{
  this->sceneNode->yaw(Ogre::Degree(angle), Ogre::Node::TS_WORLD);
}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the pitch axis
void CameraSensor::RotatePitch( float angle )
{
  this->pitchNode->pitch(Ogre::Degree(angle));
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the width of the image
unsigned int CameraSensor::GetImageWidth() const
{
  return this->imageWidth;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
unsigned int CameraSensor::GetImageHeight() const
{
  return this->imageHeight;
}


//////////////////////////////////////////////////////////////////////////////
// Enable or disable saving
void CameraSensor::EnableSaveFrame(bool enable)
{
  this->saveFrames = enable;
}


//////////////////////////////////////////////////////////////////////////////
// Save the current frame to disk
void CameraSensor::SaveFrame()
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  std::ostringstream sstream;
  Ogre::ImageCodec::ImageData *imgData;
  Ogre::Codec * pCodec;
  size_t size, pos;

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->renderTexture->getBuffer(0, 0);
 
  // Create image data structure
  imgData  = new Ogre::ImageCodec::ImageData();

  imgData->width = mBuffer->getWidth();
  imgData->height = mBuffer->getHeight();
  imgData->depth = mBuffer->getDepth();
  imgData->format = mBuffer->getFormat();
  size = mBuffer->getSizeInBytes();

  // Allocate buffer
  if (!this->saveFrameBuffer)
    this->saveFrameBuffer = new unsigned char[size];

  mBuffer->lock(Ogre::HardwarePixelBuffer::HBL_READ_ONLY);

  // Read pixels
  mBuffer->blitToMemory( 
      Ogre::PixelBox(
        mBuffer->getWidth(), 
        mBuffer->getHeight(), 
        mBuffer->getDepth(), 
        mBuffer->getFormat(), 
        this->saveFrameBuffer)
      );

  mBuffer->unlock();

  // Wrap buffer in a chunk 
  Ogre::MemoryDataStreamPtr stream(new Ogre::MemoryDataStream( this->saveFrameBuffer, size, false));

  char tmp[1024];
  sprintf(tmp, "%s/%s-%04d.jpg", this->savePathname.c_str(), 
      this->GetName().c_str(), this->saveCount);

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
