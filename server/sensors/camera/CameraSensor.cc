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

#include <Ogre.h>
#include <CEGUISystem.h>
#include <OgreImageCodec.h>
#include <CEGUISchemeManager.h>
#include <OgreCEGUIRenderer.h>
#include <OgreLogManager.h>
#include <OgreWindowEventUtilities.h>
#include <OgreCEGUIRenderer.h>

#include "Global.hh"
#include "GazeboError.hh"
#include "Body.hh"
#include "OgreTextRenderer.hh"
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

  this->saveEnable = false;
  this->saveCount = 0;
  this->savePathname = NULL;

  CameraManager::Instance()->AddCamera(this);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void CameraSensor::LoadChild( XMLConfigNode *node )
{
  this->nearClip = node->GetDouble("nearClip",0.1,0);
  this->farClip = node->GetDouble("farClip",100,0);

  this->imageWidth = node->GetTupleInt("imageSize",0,640);
  this->imageHeight = node->GetTupleInt("imageSize",1,480);

  this->hfov = DTOR(node->GetDouble("hfov",60,0));

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
  // Create the render texture
  /*this->renderTexture = Ogre::TextureManager::getSingleton().createManual("Camera1",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, this->imageWidth, this->imageHeight, 0,  Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

  this->renderTarget = this->renderTexture->getBuffer()->getRenderTarget();

  this->renderTarget->setAutoUpdated(true);
  */

  // Create the camera
  this->camera = OgreAdaptor::Instance()->sceneMgr->createCamera(this->GetName());
  this->camera->setNearClipDistance(this->nearClip);
  this->camera->setFarClipDistance(this->farClip);

  //this->translateYawNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode(this->GetName() + "_TranslateYawSceneNode", Ogre::Vector3(0,0,0));
  //this->pitchNode = this->translateYawNode->createChildSceneNode(this->GetName() + "PitchNode");
 
  this->pitchNode = this->sceneNode->createChildSceneNode(this->GetName() + "PitchNode");

  this->pitchNode->attachObject(this->camera);

  this->pitchNode->pitch(Ogre::Degree(0));

  // Setup the viewport
  this->viewport = OgreAdaptor::Instance()->window->addViewport(this->camera);
  this->viewport->setBackgroundColour(Ogre::ColourValue::Black);

  this->camera->setAspectRatio( Ogre::Real(this->viewport->getActualWidth()) / Ogre::Real(this->viewport->getActualHeight()));

  OgreTextRenderer::Instance()->AddTextBox(this->GetName(), "", 10, 10, 100, 20, Ogre::ColourValue::White);
  this->UpdateText();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void CameraSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void CameraSensor::UpdateChild()
{  
  /*
  //this->renderTexture->writeContentsToTimestampedFile("test",".jpg");

  // Later on I update the render texture then get a textureptr
  this->renderTarget->update();

  // TESTING:
//  this->renderTarget->writeContentsToFile("testFile.jpeg");

  //Ogre::TexturePtr mRndrTexPtr = Ogre::TextureManager::getSingleton().getByName(this->renderTexture.getName());

  // Get access to the buffer and make an image and write it to file
  Ogre::HardwarePixelBufferSharedPtr mBuffer = this->renderTexture->getBuffer(0, 0);

  // copyToMemory
  Ogre::ImageCodec::ImageData *imgData = new Ogre::ImageCodec::ImageData();

  imgData->width = this->imageWidth;
  imgData->height = this->imageHeight;
  imgData->depth = 1;
  imgData->format = Ogre::PF_BYTE_RGBA;
  size_t size = imgData->width * imgData->height * 4;

  // Allocate buffer
  uchar* pBuffer = new uchar[size];

  // DELETE ME:
  //mBuffer->blitToMemory( Ogre::Box(0, 0, this->imageWidth, this->imageHeight), Ogre::PixelBox(this->imageWidth, this->imageHeight, 1, imgData->format, pBuffer));

  // Read pixels
  mBuffer->blitToMemory( Ogre::Box((int)(this->imageWidth/2.0), (int)(this->imageHeight/2.0), this->imageWidth, this->imageHeight), Ogre::PixelBox(this->imageWidth, this->imageHeight, 1, imgData->format, pBuffer));

  // Wrap buffer in a chunk 
  Ogre::MemoryDataStreamPtr stream(new Ogre::MemoryDataStream( pBuffer, size, false));

  // Get codec
  Ogre::String filename = "test.jpg"; 
  size_t pos = filename.find_last_of(".");
  Ogre::String extension;

  while( pos != filename.length() - 1 )
    extension += filename[++pos];

  // Get the codec
  Ogre::Codec * pCodec = Ogre::Codec::getCodec(extension); 

  // Write out
  Ogre::Codec::CodecDataPtr codecDataPtr(imgData); pCodec->codeToFile(stream, filename, codecDataPtr);

  delete [] pBuffer; 
  */

  /*  // Save image frames
      if (this->saveEnable)
      this->SaveFrame();
      */

  return;
}

////////////////////////////////////////////////////////////////////////////////
// Translate the camera
void CameraSensor::Translate( const Vector3 &direction )
{
  Ogre::Vector3 vec(direction.x, direction.y, direction.z);

  //this->translateYawNode->translate(this->translateYawNode->getOrientation() * this->pitchNode->getOrientation() * vec);
  this->sceneNode->translate(this->sceneNode->getOrientation() * this->pitchNode->getOrientation() * vec);

  this->UpdateText();
}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the yaw axis
void CameraSensor::RotateYaw( float angle )
{
  //this->translateYawNode->yaw(Ogre::Degree(angle));
  this->sceneNode->yaw(Ogre::Degree(angle), Ogre::Node::TS_WORLD);
  this->UpdateText();
}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the pitch axis
void CameraSensor::RotatePitch( float angle )
{
  this->pitchNode->pitch(Ogre::Degree(angle));
  this->UpdateText();
}

//////////////////////////////////////////////////////////////////////////////
// Get the image dimensions
void CameraSensor::GetImageSize(int *w, int *h)
{
  *w = this->imageWidth;
  *h = this->imageHeight;
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Set the base filename for saved frames
void CameraSensor::SetSavePath(const char * /*pathname*/)
{
/*  char tmp[1024];
    
  this->savePathname = pathname;
  this->saveCount = 0;

  sprintf(tmp, "mkdir %s 2>>/dev/null", this->savePathname);
  system(tmp);
  */

  return;
}


//////////////////////////////////////////////////////////////////////////////
// Enable or disable saving
void CameraSensor::EnableSaveFrame(bool /*enable*/)
{
  //this->saveEnable = enable;
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Save the current frame to disk
void CameraSensor::SaveFrame()
{
  /*char tmp[1024];
  FILE *fp;
  
  sprintf(tmp, "%s/%04d.pnm", this->savePathname, this->saveCount);

  fp = fopen( tmp, "wb" );

  if (!fp)
  {
    PRINT_ERR1( "unable to open file %s\n for writing", tmp );
    return;
  }
  
  fprintf( fp, "P6\n# Gazebo\n%d %d\n255\n", this->imageWidth, this->imageHeight);
  for (int i = this->imageHeight-1; i >= 0; i--)
    fwrite( this->rgbImage + i * this->imageWidth * 3, 1, this->imageWidth * 3, fp );

  fclose( fp );
  this->saveCount++;
  */

  return;
}

//////////////////////////////////////////////////////////////////////////////
// Update the GUI text
void CameraSensor::UpdateText()
{
  std::ostringstream stream;

  stream.precision(2);
  stream.flags(std::ios::showpoint | std::ios::fixed);
  stream.fill('0');

  stream << this->GetName() << "\n";
  stream << "\tXYZ [" 
    << this->sceneNode->getWorldPosition().x  << " "
    << this->sceneNode->getWorldPosition().y << " "
    << this->sceneNode->getWorldPosition().z << "]\n";

  stream << "\tRPY [" 
    << this->sceneNode->getWorldOrientation().getRoll().valueDegrees() << " " 
    << this->pitchNode->getWorldOrientation().getPitch().valueDegrees() << " "
    << this->sceneNode->getWorldOrientation().getYaw().valueDegrees() << "]";

  OgreTextRenderer::Instance()->SetText(this->GetName(), stream.str());
}
