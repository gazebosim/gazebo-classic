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
 * CVS: $Id: Camera.cc,v 1.1.2.1 2006/12/16 22:41:17 natepak Exp $
 */

#if HAVE_CONFIG_H
  #include <config.h>
#endif

#include <Ogre.h>
#include <OgreImageCodec.h>

#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"
#include "Camera.hh"

//////////////////////////////////////////////////////////////////////////////
// Constructor 
Camera::Camera()
{
  //this->cameraPose = GzPoseSet(GzVectorZero(), GzQuaternIdent());

  this->imageWidth = this->imageHeight = 0;

  this->saveEnable = false;
  this->saveCount = 0;
  this->savePathname = NULL;

  return;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
Camera::~Camera()
{
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Initialize the sensor
int Camera::Init(int width, int height, double hfov, double minDepth, 
    double maxDepth, int zBufferDepth)
{
  this->imageWidth = width;
  this->imageHeight = height;

  this->nearClip = minDepth;
  this->farClip = maxDepth;

  this->hfov = hfov;

  // Do some sanity checks
  if (this->imageWidth == 0 || this->imageHeight == 0)
  {
    printf("image has zero size\n");
    return -1;
  }
  if (this->hfov < 0.01 || this->hfov > M_PI)
  {
    printf("bad field of view\n");
    return -1;
  }
  if (this->nearClip < 0.01)
  {
    printf("near clipping plane (min depth) is zero\n");
    return -1;
  }

  // Create the render texture
  /*this->renderTexture = Ogre::TextureManager::getSingleton().createManual("Camera1",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, this->imageWidth, this->imageHeight, 0,  Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

  this->renderTarget = this->renderTexture->getBuffer()->getRenderTarget();

  this->renderTarget->setAutoUpdated(true);
  */

  // Create the camera
  this->camera = OgreAdaptor::Instance()->sceneMgr->createCamera("Camera1");
  this->camera->setNearClipDistance(minDepth);
  //this->camera->setFarClipDistance(maxDepth);

  this->translateYawNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode("Camera1_TranslateYawSceneNode", Ogre::Vector3(0,2,2));
  this->pitchNode = this->translateYawNode->createChildSceneNode("Camera1_PitchNode");
  this->pitchNode->attachObject(this->camera);
  this->pitchNode->pitch(Ogre::Degree(-30));

  // Setup the viewport
  this->viewport = OgreAdaptor::Instance()->window->addViewport(this->camera);
  this->viewport->setBackgroundColour(Ogre::ColourValue::Black);

  this->camera->setAspectRatio( Ogre::Real(this->viewport->getActualWidth()) / Ogre::Real(this->viewport->getActualHeight()));

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
int Camera::Fini()
{
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void Camera::Update()
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


//////////////////////////////////////////////////////////////////////////////
// Render the scene from the camera perspective
void Camera::Render()
{
  return;
}

const unsigned char *Camera::GetImageData()
{
  return NULL;
}

//////////////////////////////////////////////////////////////////////////////
// Get the Z-buffer value at the given image coordinate
double Camera::GetZValue(int x, int y)
{
  //GLfloat iz;

  // Flip y axis
  //y = this->imageHeight - 1 - y;

  // Get image z value of first spot
//  glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &iz);

  return 0.0;// iz;
}


//////////////////////////////////////////////////////////////////////////////
// Determine change in camera pose based on two image points
/*GzPose Camera::CalcCameraDelta(int mode, GzVector a, GzVector b)
{
  GzPose n;
  GLfloat ix, iy, iz;

  // Image coordates of the first spot
  ix = a.x;
  iy = this->imageHeight - 1 - a.y;
  iz = a.z;

  double dx, dy, dz;
  
  // Convert to division coordinates
  dx = 2 * ix / this->imageWidth - 1;
  dy = 2 * iy / this->imageHeight - 1;
  dz = (2 * iz  - 1);

  double px, py, pz, pw;

  // Compute perspective divisor; assumes well-formed projection matrix
  pw = this->cameraProjectionMatrix[14] / (dz + this->cameraProjectionMatrix[10]);

  // Convert to projected coordinate
  px = dx * pw;
  py = dy * pw;
  pz = dz * pw;

  double cx, cy, cz, cw;
  
  // Convert to camera frame (inverse projection)
  cx = 1 / this->cameraProjectionMatrix[0] * px;
  cy = 1 / this->cameraProjectionMatrix[5] * py;
  cz = -pw;
  cw = 1 / this->cameraProjectionMatrix[14] * pz + 1 / this->cameraProjectionMatrix[10] * pw;

  double ix_, iy_;
  double dx_, dy_;
  
  // Image coordates of the second spot
  ix_ = b.x;
  iy_ = this->imageHeight - 1 - b.y;

  // Convert to division coordinates
  dx_ = 2 * ix_ / this->imageWidth - 1;
  dy_ = 2 * iy_ / this->imageHeight - 1;

  GzPose n;
  n.pos = GzVectorZero();
  n.rot = GzQuaternIdent();
  
  double nx, ny, nz;

  // Translate along x, y
  if (mode == 0)
  {
    nz = 0.0;
    nx = - (cz + nz) / this->cameraProjectionMatrix[0] * dx_ - cx;
    ny = - (cz + nz) / this->cameraProjectionMatrix[5] * dy_ - cy;
  }

  // Translate (zoom) along z
  else if (mode == 1)
  {
    ny = 0.0;
    nz = - this->cameraProjectionMatrix[5] / (fabs(dy_) + 1e-16) * fabs(cy) - cz;    
    nx = 0;

    // Bound so we dont go too close
    if (nz > -cz - 2 * this->nearClip)
      nz = -cz - 2 * this->nearClip;

    // Bound so we dont go too far away
    else if (nz < -0.5 * this->farClip - cz)
      nz = -0.5 * this->farClip - cz;
  }
  else
  {
    nx = 0;
    ny = 0;
    nz = 0;
  }
  
  // Convert to Gazebo coordinate system (camera axis along +x)
  n.pos.x = +nz;
  n.pos.y = +nx;
  n.pos.z = -ny;

  // Rotate (pitch and yaw)
  if (mode == 2)
  {
    double rx, ry;
    rx = atan((dx_ - dx)  / this->cameraProjectionMatrix[0]);
    ry = atan((dy_ - dy)  / this->cameraProjectionMatrix[5]);
    n.rot = GzQuaternFromEuler(0, ry, rx);
  }
  
  return n;
}*/

void Camera::Translate( const Ogre::Vector3 &direction )
{
  this->translateYawNode->translate(this->translateYawNode->getOrientation() * this->pitchNode->getOrientation() * direction);
}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the yaw axis
void Camera::RotateYaw( float angle )
{
  this->translateYawNode->yaw(Ogre::Degree(angle));
}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the pitch axis
void Camera::RotatePitch( float angle )
{
  this->pitchNode->pitch(Ogre::Degree(angle));
}

//////////////////////////////////////////////////////////////////////////////
// Set the pose of the camera
/*void Camera::SetPose(GzPose pose)
{
  this->cameraPose = pose;
  return;
}*/


//////////////////////////////////////////////////////////////////////////////
// Get the pose of the camera
/*GzPose Camera::GetPose()
{
  return //this->cameraPose;
}*/


//////////////////////////////////////////////////////////////////////////////
// Set the camera FOV (horizontal)
void Camera::SetFOV(double fov)
{
  this->hfov = fov;
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Get the camera FOV (horizontal)
double Camera::GetFOV() const
{
  return this->hfov;
}


//////////////////////////////////////////////////////////////////////////////
// Get the image dimensions
void Camera::GetImageSize(int *w, int *h)
{
  *w = this->imageWidth;
  *h = this->imageHeight;
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Set the base filename for saved frames
void Camera::SetSavePath(const char *pathname)
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
void Camera::EnableSaveFrame(bool enable)
{
  //this->saveEnable = enable;
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Save the current frame to disk
void Camera::SaveFrame()
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
