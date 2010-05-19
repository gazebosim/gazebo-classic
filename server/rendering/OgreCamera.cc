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
 * CVS: $Id$
 */

#include <sstream>
#include <OgreImageCodec.h>
#include <Ogre.h>
#include <dirent.h>

#include "FPSViewController.hh"
#include "OrbitViewController.hh"
#include "PhysicsEngine.hh"
#include "Global.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "XMLConfig.hh"
#include "Simulator.hh"
#include "Model.hh"
#include "Body.hh"

#include "OgreAdaptor.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"

using namespace gazebo;

unsigned int OgreCamera::cameraCounter = 0;

//////////////////////////////////////////////////////////////////////////////
// Constructor
OgreCamera::OgreCamera(const std::string &namePrefix)
{
  this->name = "DefaultCameraName";

  this->animState = NULL;
  this->textureWidth = this->textureHeight = 0;

  this->saveFrameBuffer = NULL;
  this->saveCount = 0;
  this->bayerFrameBuffer = NULL;

  this->myCount = cameraCounter++;

  std::ostringstream stream;
  stream << namePrefix << "(" << this->myCount << ")";
  this->cameraName = stream.str();

  this->renderTarget = NULL;
  this->userMovable = true;

  Param::Begin(&this->camParameters);
  this->nearClipP = new ParamT<double>("nearClip",0.1,0);
  this->farClipP = new ParamT<double>("farClip",50,0);
  this->saveFramesP = new ParamT<bool>("saveFrames",false,0);
  this->savePathnameP = new ParamT<std::string>("saveFramePath","",0);
  this->imageSizeP = new ParamT< Vector2<int> >("imageSize", Vector2<int>(320, 240),0);
  this->visMaskP = new ParamT<std::string>("mask","none",0);
  this->hfovP = new ParamT<Angle>("hfov", Angle(DTOR(60)),0);
  this->imageFormatP = new ParamT<std::string>("imageFormat", "R8G8B8", 0);
  this->updateRateP = new ParamT<double>("updateRate",32,0);
  Param::End();

  this->captureData = false;

  this->camera = NULL;

  this->renderPeriod = Time(1.0/(**this->updateRateP));
  this->renderingEnabled = true;

  World::Instance()->ConnectShowWireframeSignal( boost::bind(&OgreCamera::ShowWireframe, this, _1) );

  this->pitchNode = NULL;
  this->sceneNode = NULL;
  this->origParentNode = NULL;

  this->viewController = new FPSViewController(this);
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
OgreCamera::~OgreCamera()
{
  if (this->saveFrameBuffer)
    delete [] this->saveFrameBuffer;

  if (this->bayerFrameBuffer)
    delete [] this->bayerFrameBuffer;

  delete this->nearClipP;
  delete this->farClipP;
  delete this->saveFramesP;
  delete this->savePathnameP;
  delete this->imageSizeP;
  delete this->imageFormatP;
  delete this->visMaskP;
  delete this->hfovP;
  delete this->viewController;

  if (this->pitchNode)
  {
    this->sceneNode->removeAndDestroyChild( this->cameraName + "PitchNode");
    this->pitchNode = NULL;
  }
  if (this->camera)
  {
    OgreCreator::DeleteCamera(this->camera);
    this->camera = NULL;
  }
  OgreAdaptor::Instance()->UnregisterCamera(this);
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void OgreCamera::LoadCam( XMLConfigNode *node )
{
  // This should be last in the constructor
  CameraManager::Instance()->AddCamera(this);

  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  this->visibilityMask = GZ_ALL_CAMERA; 

  if (node)
  {
    this->nearClipP->Load(node);
    this->farClipP->Load(node);
    this->saveFramesP->Load(node);
    this->savePathnameP->Load(node);
    this->imageSizeP->Load(node);
    this->imageFormatP->Load(node);
    this->visMaskP->Load(node);
    this->hfovP->Load(node);

    if (this->visMaskP->GetValue() == "laser")
    {
      this->visibilityMask ^= GZ_LASER_CAMERA;
    }

    if (this->imageFormatP->GetValue() == "L8")
      this->imageFormat = Ogre::PF_L8;
    else if (this->imageFormatP->GetValue() == "R8G8B8")
      this->imageFormat = Ogre::PF_R8G8B8;
    else if (this->imageFormatP->GetValue() == "B8G8R8")
      this->imageFormat = Ogre::PF_B8G8R8;
    else if ( (this->imageFormatP->GetValue() == "BAYER_RGGB8") ||
              (this->imageFormatP->GetValue() == "BAYER_BGGR8") ||
              (this->imageFormatP->GetValue() == "BAYER_GBRG8") ||
              (this->imageFormatP->GetValue() == "BAYER_GRBG8") )
    {
      // let ogre generate rgb8 images for all bayer format requests
      // then post process to produce actual bayer images
      this->imageFormat = Ogre::PF_R8G8B8;
    }
    else
    {
      std::cerr << "Error parsing image format (" << this->imageFormatP->GetValue() << "), using default Ogre::PF_R8G8B8\n";
      this->imageFormat = Ogre::PF_R8G8B8;
    }
  }

  // Create the directory to store frames
  if (this->saveFramesP->GetValue())
  {
    std::string command;
    command = "mkdir " + this->savePathnameP->GetValue() + " 2>>/dev/null";
    if (system(command.c_str()) < 0)
      std::cerr << "Error making directory\n";
  }

  if (this->hfovP->GetValue() < Angle(0.01) || 
      this->hfovP->GetValue() > Angle(M_PI))
  {
    gzthrow("Camera horizontal field of veiw invalid.");
  }
  if (this->nearClipP->GetValue() <= 0)
  {
    gzthrow("near clipping plane (min depth) <= zero");
  }

  this->lastUpdate = Simulator::Instance()->GetSimTime();

}

//////////////////////////////////////////////////////////////////////////////
/// Save camera info in xml format
void OgreCamera::SaveCam(std::string &prefix, std::ostream &stream)
{
  stream << prefix << (*this->nearClipP) << "\n";
  stream << prefix << (*this->farClipP) << "\n";
  stream << prefix << (*this->saveFramesP) << "\n";
  stream << prefix << (*this->savePathnameP) << "\n";
  stream << prefix << (*this->imageSizeP) << "\n";
  stream << prefix << (*this->imageFormatP) << "\n";
  stream << prefix << (*this->visMaskP) << "\n";
  stream << prefix << (*this->hfovP) << "\n";
}
 
//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void OgreCamera::InitCam()
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  this->camera = OgreCreator::CreateCamera(this->cameraName, 
      **this->nearClipP, **this->farClipP, *(**this->hfovP), 
      this->renderTarget );

  // Create a scene node to control pitch motion
  this->pitchNode = this->sceneNode->createChildSceneNode( this->cameraName + "PitchNode");
  this->pitchNode->pitch(Ogre::Degree(0));
  this->pitchNode->attachObject(this->camera);
  this->camera->setAutoAspectRatio(true);

  this->saveCount = 0;

  OgreAdaptor::Instance()->RegisterCamera(this);

  this->origParentNode = (Ogre::SceneNode*)this->sceneNode->getParent();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void OgreCamera::FiniCam()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void OgreCamera::UpdateCam()
{
  this->viewController->Update();

  if (this->animState)
  {
    this->animState->addTime(0.01);
    if (this->animState->hasEnded())
    {
      this->animState = NULL;

      OgreAdaptor::Instance()->sceneMgr->destroyAnimation("cameratrack");
      OgreAdaptor::Instance()->sceneMgr->destroyAnimationState("cameratrack");
    }
  }

  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (this->sceneNode)
  {
    Ogre::Vector3 v = this->sceneNode->_getDerivedPosition();

    this->pose.pos.x = v.x;
    this->pose.pos.y = v.y;
    this->pose.pos.z = v.z;
  }

  if (this->pitchNode)
  {
    Ogre::Quaternion q = this->pitchNode->_getDerivedOrientation();

    this->pose.rot.u = q.w;
    this->pose.rot.x = q.x;
    this->pose.rot.y = q.y;
    this->pose.rot.z = q.z;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set to true to enable rendering
void OgreCamera::SetRenderingEnabled(bool value)
{
  this->renderingEnabled = value;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether the rendering is enabled
bool OgreCamera::GetRenderingEnabled() const
{
  return this->renderingEnabled;
}

////////////////////////////////////////////////////////////////////////////////
// Render the camera
void OgreCamera::Render()
{
  // disable rendering if "-r" option is given
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  // disable rendering if sensor not set to active
  if (!this->renderingEnabled)
    return;

  Time physics_dt = World::Instance()->GetPhysicsEngine()->GetStepTime();
  if (((Simulator::Instance()->GetSimTime()-this->lastUpdate-this->renderPeriod)/physics_dt) >= 0)
  {
    {
      boost::recursive_mutex::scoped_lock md_lock(*Simulator::Instance()->GetMDMutex());
      this->renderTarget->update();
    }

    if (this->captureData)
    {
      boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());

      Ogre::HardwarePixelBufferSharedPtr pixelBuffer;
      Ogre::RenderTexture *rTexture;
      Ogre::Viewport* renderViewport;

      size_t size;

      // Get access to the buffer and make an image and write it to file
      pixelBuffer = this->renderTexture->getBuffer();
      rTexture = pixelBuffer->getRenderTarget();

      Ogre::PixelFormat format = pixelBuffer->getFormat();
      renderViewport = rTexture->getViewport(0);

      size = Ogre::PixelUtil::getMemorySize((**this->imageSizeP).x,
          (**this->imageSizeP).y, 
          1, 
          format);

      // Allocate buffer
      if (!this->saveFrameBuffer)
        this->saveFrameBuffer = new unsigned char[size];

      memset(this->saveFrameBuffer,128,size);

      Ogre::PixelBox box((**this->imageSizeP).x, (**this->imageSizeP).y,
          1, this->imageFormat, this->saveFrameBuffer);

      pixelBuffer->blitToMemory( box );

      if (this->saveFramesP->GetValue())
      {
        this->SaveFrame();
      }
    }

    this->lastUpdate = Simulator::Instance()->GetSimTime();
  }

}


////////////////////////////////////////////////////////////////////////////////
// Get the global pose of the camera
Pose3d OgreCamera::GetWorldPose() const
{
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the global pose of the camera
void OgreCamera::SetWorldPose(const Pose3d &pose)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  this->pose = pose;
  this->pose.Correct();
  this->sceneNode->setPosition( this->pose.pos.x, this->pose.pos.y, this->pose.pos.z);
  this->pitchNode->setOrientation( this->pose.rot.u, this->pose.rot.x, this->pose.rot.y, this->pose.rot.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the world position
void OgreCamera::SetPosition(const Vector3 &pos)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;


  this->pose.pos = pos;
  this->pose.Correct();
  
  this->sceneNode->setPosition( this->pose.pos.x, this->pose.pos.y, this->pose.pos.z);
}
 
////////////////////////////////////////////////////////////////////////////////
/// Set the clip distances
void OgreCamera::SetClipDist(float near, float far)
{
  this->nearClipP->SetValue(near);
  this->farClipP->SetValue(far);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the camera FOV (horizontal)  
void OgreCamera::SetFOV( float radians )
{
  this->hfovP->SetValue(radians);
}


////////////////////////////////////////////////////////////////////////////////
// Translate the camera
void OgreCamera::Translate( const Vector3 &direction )
{
  Ogre::Vector3 vec(direction.x, direction.y, direction.z);

  this->sceneNode->translate(this->sceneNode->getOrientation() * this->pitchNode->getOrientation() * vec);

}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the yaw axis
void OgreCamera::RotateYaw( float angle )
{
  this->sceneNode->roll(Ogre::Radian(angle), Ogre::Node::TS_WORLD);
}

//////////////////////////////////////////////////////////////////////////////
// Rotate the camera around the pitch axis
void OgreCamera::RotatePitch( float angle )
{
  this->pitchNode->yaw(Ogre::Radian(angle));
}

//////////////////////////////////////////////////////////////////////////////
/// Get the horizontal field of view of the camera
Angle OgreCamera::GetHFOV() const
{
  return this->hfovP->GetValue();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical field of view of the camera
Angle OgreCamera::GetVFOV() const
{
  return Angle(this->camera->getFOVy().valueRadians());
}

//////////////////////////////////////////////////////////////////////////////
/// Get the width of the image
unsigned int OgreCamera::GetImageWidth() const
{
  return this->imageSizeP->GetValue().x;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
unsigned int OgreCamera::GetImageHeight() const
{
  return this->imageSizeP->GetValue().y;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
int OgreCamera::GetImageDepth() const
{
  if (this->imageFormatP->GetValue() == "L8")
    return 1;
  else if (this->imageFormatP->GetValue() == "R8G8B8")
    return 3;
  else if (this->imageFormatP->GetValue() == "B8G8R8")
    return 3;
  else if ( (this->imageFormatP->GetValue() == "BAYER_RGGB8") ||
            (this->imageFormatP->GetValue() == "BAYER_BGGR8") ||
            (this->imageFormatP->GetValue() == "BAYER_GBRG8") ||
            (this->imageFormatP->GetValue() == "BAYER_GRBG8") )
    return 1;
  else
  {
    std::cerr << "Error parsing image format (" << this->imageFormatP->GetValue() << "), using default Ogre::PF_R8G8B8\n";
    return 3;
  }
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
std::string OgreCamera::GetImageFormat() const
{
  return this->imageFormatP->GetValue();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the width of the texture
unsigned int OgreCamera::GetTextureWidth() const
{
  return this->textureWidth;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the texture
unsigned int OgreCamera::GetTextureHeight() const
{
  return this->textureHeight;
}


//////////////////////////////////////////////////////////////////////////////
// Get the image size in bytes
size_t OgreCamera::GetImageByteSize() const
{

  return Ogre::PixelUtil::getMemorySize((**this->imageSizeP).x,
                                        (**this->imageSizeP).y, 
                                        1, 
                                        this->imageFormat);
}


//////////////////////////////////////////////////////////////////////////////
// Enable or disable saving
void OgreCamera::EnableSaveFrame(bool enable)
{
  this->saveFramesP->SetValue( enable );
}

//////////////////////////////////////////////////////////////////////////////
// Set the save frame pathname
void OgreCamera::SetSaveFramePathname(const std::string &pathname)
{
  this->savePathnameP->SetValue( pathname );

  // Create the directory to store frames
  if (this->saveFramesP->GetValue())
  {
    std::string command;
    command = "mkdir " + this->savePathnameP->GetValue() + " 2>>/dev/null";
    if (system(command.c_str()) <0)
      std::cerr << "Error making directory\n";
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Toggle saving of frames
void OgreCamera::ToggleSaveFrame()
{
  this->saveFramesP->SetValue(!this->saveFramesP->GetValue());
}

////////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the ogre camera
Ogre::Camera *OgreCamera::GetOgreCamera() const
{
  return this->camera;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the near clip distance
double OgreCamera::GetNearClip()
{
  return this->nearClipP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the far clip distance
double OgreCamera::GetFarClip()
{
  return this->farClipP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport width in pixels
unsigned int OgreCamera::GetViewportWidth() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualWidth();
  else
    return this->camera->getViewport()->getActualWidth();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport height in pixels
unsigned int OgreCamera::GetViewportHeight() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualHeight();
  else
    return this->camera->getViewport()->getActualHeight();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the aspect ratio
void OgreCamera::SetAspectRatio( float ratio )
{
  this->camera->setAspectRatio( ratio );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport up vector
Vector3 OgreCamera::GetUp()
{
  //std::cout << "ogre cam position " <<  this->camera->getRealPosition() << std::endl;
  Ogre::Vector3 up = this->camera->getRealUp();
  //std::cout << "ogre up " <<  up << std::endl;
  return Vector3(up.x,up.y,up.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport right vector
Vector3 OgreCamera::GetRight()
{
  Ogre::Vector3 right = this->camera->getRealRight();
  //std::cout << "ogre right " <<  right << std::endl;
  return Vector3(right.x,right.y,right.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the camera position in the world
Vector3 OgreCamera::GetCameraPosition()
{
  Ogre::Vector3 camPos = this->camera->getRealPosition();
  return Vector3(camPos.x,camPos.y,camPos.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the user can move the camera via the GUI
void OgreCamera::SetUserMovable( bool movable )
{
  this->userMovable = movable;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether the user can move the camera via the GUI
bool OgreCamera::GetUserMovable() const
{
  return this->userMovable;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the camera
std::string OgreCamera::GetCameraName()
{
  return this->cameraName;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the camera's scene node
void OgreCamera::SetCameraSceneNode( Ogre::SceneNode *node )
{
  this->sceneNode = node;
}

//////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the image data
const unsigned char *OgreCamera::GetImageData(unsigned int i)
{
  if (i!=0)
    gzerr(0) << "Camera index must be zero for mono cam";

  int width = this->imageSizeP->GetValue().x;
  int height = this->imageSizeP->GetValue().y;

  // do last minute conversion if Bayer pattern is requested, go from R8G8B8
  if ( (this->imageFormatP->GetValue() == "BAYER_RGGB8") ||
       (this->imageFormatP->GetValue() == "BAYER_BGGR8") ||
       (this->imageFormatP->GetValue() == "BAYER_GBRG8") ||
       (this->imageFormatP->GetValue() == "BAYER_GRBG8") )
  {
    if (!this->bayerFrameBuffer)
      this->bayerFrameBuffer = new unsigned char[width*height];
    this->ConvertRGBToBAYER(this->bayerFrameBuffer,this->saveFrameBuffer,this->imageFormatP->GetValue(),width,height);
    return this->bayerFrameBuffer;
  }
  else
    return this->saveFrameBuffer;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the camera's name
std::string OgreCamera::GetCamName()
{
  return this->name;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the camera's name
void OgreCamera::SetCamName( const std::string &_name )
{
  this->name = _name;
}


//////////////////////////////////////////////////////////////////////////////
// Save the current frame to disk
void OgreCamera::SaveFrame()
{
  Ogre::HardwarePixelBufferSharedPtr mBuffer;
  std::ostringstream sstream;
  Ogre::ImageCodec::ImageData *imgData;
  Ogre::Codec * pCodec;
  size_t size, pos;

  this->GetImageData();

  // Create a directory if not present
  DIR *dir = opendir( this->savePathnameP->GetValue().c_str() );
  if (!dir)
  {
    std::string command;
    command = "mkdir " + this->savePathnameP->GetValue() + " 2>>/dev/null";
    if (system(command.c_str()) < 0)
      std::cerr << "Error making directory\n";
  }

  // Get access to the buffer and make an image and write it to file
  mBuffer = this->renderTexture->getBuffer(0, 0);

  // Create image data structure
  imgData  = new Ogre::ImageCodec::ImageData();

  imgData->width = this->imageSizeP->GetValue().x;
  imgData->height = this->imageSizeP->GetValue().y;
  imgData->depth = this->GetImageDepth();
  imgData->format = this->imageFormat;
  size = this->GetImageByteSize();

  // Wrap buffer in a chunk
  Ogre::MemoryDataStreamPtr stream(new Ogre::MemoryDataStream( this->saveFrameBuffer, size, false));

  char tmp[1024];
  if (!this->savePathnameP->GetValue().empty())
  {
    double simTime = Simulator::Instance()->GetSimTime().Double();
    int min = (int)(simTime / 60.0);
    int sec = (int)(simTime - min*60);
    int msec = (int)(simTime*1000 - min*60000 - sec*1000);

    sprintf(tmp, "%s/%s-%04d-%03dm_%02ds_%03dms.jpg", this->savePathnameP->GetValue().c_str(), this->GetCamName().c_str(), this->saveCount, min, sec, msec);
  }
  else
  {
    sprintf(tmp, "%s-%04d.jpg", this->GetCamName().c_str(), this->saveCount);
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

//////////////////////////////////////////////////////////////////////////////
// Move the camera to focus on an entity
void OgreCamera::MoveToEntity(Entity *entity)
{
  if (!entity)
    return;

  if (OgreAdaptor::Instance()->sceneMgr->hasAnimation("cameratrack"))
  {
    OgreAdaptor::Instance()->sceneMgr->destroyAnimation("cameratrack");
    OgreAdaptor::Instance()->sceneMgr->destroyAnimationState("cameratrack");
  }

  Ogre::Animation *anim = OgreAdaptor::Instance()->sceneMgr->createAnimation("cameratrack",.5);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0,this->sceneNode);
  Ogre::NodeAnimationTrack *ptrack = anim->createNodeTrack(1,this->pitchNode);

  Vector3 start = this->GetWorldPose().pos;
  Vector3 end = entity->GetAbsPose().pos;
  Vector3 dir = end - start;

  double yawAngle = atan2(dir.y,dir.x);
  double pitchAngle = atan2(-dir.z, sqrt(dir.x*dir.x + dir.y*dir.y));
  Ogre::Quaternion yawFinal(Ogre::Radian(yawAngle), Ogre::Vector3(0,0,1));
  Ogre::Quaternion pitchFinal(Ogre::Radian(pitchAngle), Ogre::Vector3(0,1,0));

  Ogre::TransformKeyFrame *key;

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = ptrack->createNodeKeyFrame(0);
  key->setRotation(this->pitchNode->getOrientation());

  Vector3 min, max, size;
  OgreVisual *vis = entity->GetVisualNode();
  OgreCreator::GetVisualBounds(vis, min,max);
  size = max-min;

  double scale = std::max(std::max(size.x, size.y), size.z);
  scale += 0.5;

  dir.Normalize();
  double dist = start.Distance(end);

  Vector3 mid = start + dir*(dist*.5 - scale);
  key = strack->createNodeKeyFrame(.2);
  key->setTranslate( Ogre::Vector3(mid.x, mid.y, mid.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(.2);
  key->setRotation(pitchFinal);

  end = start + dir*(dist - scale);
  key = strack->createNodeKeyFrame(.5);
  key->setTranslate( Ogre::Vector3(end.x, end.y, end.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(.5);
  key->setRotation(pitchFinal);

  this->animState = OgreAdaptor::Instance()->sceneMgr->createAnimationState("cameratrack");
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
}

//////////////////////////////////////////////////////////////////////////////
/// Set the camera to track an entity
void OgreCamera::TrackModel( Model *model )
{
  this->sceneNode->getParent()->removeChild(this->sceneNode);

  if (model)
  {
    Body *b = model->GetCanonicalBody();
    b->GetVisualNode()->GetSceneNode()->addChild(this->sceneNode);
    this->camera->setAutoTracking(true, b->GetVisualNode()->GetSceneNode() );
  }
  else
  {
    this->origParentNode->addChild(this->sceneNode);
    this->camera->setAutoTracking(false, NULL);
    this->camera->setPosition(Ogre::Vector3(0,0,0));
    this->camera->setOrientation(Ogre::Quaternion(-.5,-.5,.5,.5));
  }
}

//////////////////////////////////////////////////////////////////////////////
// Set whether to view the world in wireframe
void OgreCamera::ShowWireframe(bool s)
{
  if (this->camera)
  {
    if (s)
    {
      this->camera->setPolygonMode(Ogre::PM_WIREFRAME);
    }
    else
    {
      this->camera->setPolygonMode(Ogre::PM_SOLID);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get a world space ray as cast from the camer through the viewport
void OgreCamera::GetCameraToViewportRay(int screenx, int screeny,
                                        Vector3 &origin, Vector3 &dir)
{
  Ogre::Ray ray = this->camera->getCameraToViewportRay(
      (float)screenx / this->GetViewportWidth(),
      (float)screeny / this->GetViewportHeight());

  origin.Set(ray.getOrigin().x, ray.getOrigin().y, ray.getOrigin().z);
  dir.Set(ray.getDirection().x, ray.getDirection().y, ray.getDirection().z);
}


////////////////////////////////////////////////////////////////////////////////
/// post process, convert from rgb to bayer
void OgreCamera::ConvertRGBToBAYER(unsigned char* dst, unsigned char* src, std::string format,int width, int height)
{
  if (src)
  {
    // do last minute conversion if Bayer pattern is requested, go from R8G8B8
    if (format == "BAYER_RGGB8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // RG
          // GB
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
          else // odd column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
        }
      }
    }
    else if (format == "BAYER_BGGR8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // BG
          // GR
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
          else // odd column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
        }
      }
    }
    else if (format == "BAYER_GBRG8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // GB
          // RG
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
          else // odd column
            if (i%2) // even row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
        }
      }
    }
    else if (format == "BAYER_GRBG8")
    {
      for (int i=0;i<width;i++)
      {
        for (int j=0;j<height;j++)
        {
          //
          // GR
          // BG
          //
          // determine position
          if (j%2) // even column
            if (i%2) // even row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
            else // odd row, red
              dst[i+j*width] = src[i*3+j*width*3+0];
          else // odd column
            if (i%2) // even row, blue
              dst[i+j*width] = src[i*3+j*width*3+2];
            else // odd row, green
              dst[i+j*width] = src[i*3+j*width*3+1];
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Frame started
void OgreCamera::FrameStarted(double timeSinceLastFrame)
{
  if (this->animState)
  {
    //std::cout << "Add Time[" << timeSinceLastFrame << "]\n";
    //this->animState->addTime(timeSinceLastFrame);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the direction the camera is facing
Vector3 OgreCamera::GetDirection() const
{
  Vector3 result;
  result.x = this->camera->getDerivedDirection().x;
  result.y = this->camera->getDerivedDirection().y;
  result.z = this->camera->getDerivedDirection().z;
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the direction of the camera
void OgreCamera::SetDirection(Vector3 vec)
{
  /*this->pitchNode->setDirection(Ogre::Vector3(vec.x, vec.y, vec.z), Ogre::Node::TS_WORLD, Ogre::Vector3::NEGATIVE_UNIT_X);
  this->camera->setDirection(Ogre::Vector3(vec.x, vec.y, vec.z));
  Ogre::Quaternion q = this->camera->getOrientation();
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Hande a mouse event
void OgreCamera::HandleMouseEvent(const MouseEvent &evt)
{
  this->viewController->HandleMouseEvent(evt);
}
