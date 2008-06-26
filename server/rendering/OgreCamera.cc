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
 * CVS: $Id: OgreCamera.cc 6338 2008-04-16 17:24:32Z natepak $
 */

#include <sstream>
#include <OgreImageCodec.h>
#include <Ogre.h>

#include "Global.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "XMLConfig.hh"

#include "OgreAdaptor.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"

using namespace gazebo;

unsigned int OgreCamera::cameraCounter = 0;

//////////////////////////////////////////////////////////////////////////////
// Constructor
OgreCamera::OgreCamera(const std::string &namePrefix)
{
  this->imageWidth = this->imageHeight = 0;
  this->textureWidth = this->textureHeight = 0;

  this->saveFrameBuffer = NULL;
  this->saveCount = 0;
  this->saveFrames = false;


  this->myCount = cameraCounter++;

  std::ostringstream stream;
  stream << namePrefix << "(" << this->myCount << ")";
  this->cameraName = stream.str();

  this->renderTarget = NULL;
  this->userMovable = true;

  // This should be last in the constructor
  CameraManager::Instance()->AddCamera(this);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
OgreCamera::~OgreCamera()
{
  if (this->saveFrameBuffer)
    delete [] this->saveFrameBuffer;
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void OgreCamera::LoadCam( XMLConfigNode *node )
{

  this->visibilityMask = GZ_ALL_CAMERA; 

  if (node)
  {
    this->nearClip = node->GetDouble("nearClip",0.1,0);
    this->farClip = node->GetDouble("farClip",100,0);
    this->saveFrames = node->GetBool("saveFrames",false,0);
    this->savePathname = node->GetString("saveFramePath","",0);

    this->imageWidth = node->GetTupleInt("imageSize",0,640);
    this->imageHeight = node->GetTupleInt("imageSize",1,480);

    this->hfov = DTOR(node->GetDouble("hfov",60,0));

    if (node->GetString("mask","none",0) == "laser")
    {
      this->visibilityMask ^= GZ_LASER_CAMERA;
    }
  }
  else
  {
    this->nearClip = 0.1;
    this->farClip = 100;
    this->hfov = DTOR(60);
    this->saveFrames = false;
  }

  // Create the directory to store frames
  if (this->saveFrames)
  {
    std::string command;
    command = "mkdir " + this->savePathname + " 2>>/dev/null";
    system(command.c_str());
  }

  if (this->hfov < 0.01 || this->hfov > M_PI)
  {
    gzthrow("Camera horizontal field of veiw invalid.");
  }
  if (this->nearClip <= 0)
  {
    gzthrow("near clipping plane (min depth) <= zero");
  }

}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void OgreCamera::InitCam()
{
  this->camera = OgreCreator::CreateCamera(this->cameraName, this->nearClip, this->farClip, this->hfov, this->renderTarget );

  // Create a scene node to control pitch motion
  this->pitchNode = this->sceneNode->createChildSceneNode( this->cameraName + "PitchNode");
  this->pitchNode->pitch(Ogre::Degree(0));
  this->pitchNode->attachObject(this->camera);

  this->saveCount = 0;

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

  if (World::Instance()->GetWireframe())
  {
    this->camera->setPolygonMode(Ogre::PM_WIREFRAME);
  }
  else
  {
    this->camera->setPolygonMode(Ogre::PM_SOLID);
  }

  Ogre::Vector3 v = this->sceneNode->getWorldPosition();

  this->pose.pos.x = v.x;
  this->pose.pos.y = v.y;
  this->pose.pos.z = v.z;

  Ogre::Quaternion q = this->pitchNode->getWorldOrientation();

  this->pose.rot.u = q.w;
  this->pose.rot.x = q.x;
  this->pose.rot.y = q.y;
  this->pose.rot.z = q.z;
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
  this->pose = pose;
  this->sceneNode->setPosition( this->pose.pos.x, this->pose.pos.y, this->pose.pos.z);
  this->pitchNode->setOrientation( this->pose.rot.u, this->pose.rot.x, this->pose.rot.y, this->pose.rot.z);
}
 
////////////////////////////////////////////////////////////////////////////////
/// Set the clip distances
void OgreCamera::SetClipDist(float near, float far)
{
  this->nearClip = near;
  this->farClip = far;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the camera FOV (horizontal)  
void OgreCamera::SetFOV( float radians )
{
  this->hfov = radians;

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
double OgreCamera::GetHFOV() const
{
  return this->hfov;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical field of view of the camera
double OgreCamera::GetVFOV() const
{
  return this->camera->getFOVy().valueRadians();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the width of the image
unsigned int OgreCamera::GetImageWidth() const
{
  return this->imageWidth;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the height of the image
unsigned int OgreCamera::GetImageHeight() const
{
  return this->imageHeight;
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
  return this->imageHeight * this->imageWidth * 3;
}


//////////////////////////////////////////////////////////////////////////////
// Enable or disable saving
void OgreCamera::EnableSaveFrame(bool enable)
{
  this->saveFrames = enable;
}

//////////////////////////////////////////////////////////////////////////////
/// Toggle saving of frames
void OgreCamera::ToggleSaveFrame()
{
  this->saveFrames = !this->saveFrames;
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
  return this->nearClip;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the far clip distance
double OgreCamera::GetFarClip()
{
  return this->farClip;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the viewport width in pixels
unsigned int OgreCamera::GetViewportWidth() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualWidth();
  else
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the viewport height in pixels
unsigned int OgreCamera::GetViewportHeight() const
{
  if (this->renderTarget)
    return this->renderTarget->getViewport(0)->getActualHeight();
  else
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the aspect ratio
void OgreCamera::SetAspectRatio( float ratio )
{
  this->camera->setAspectRatio( ratio );
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


