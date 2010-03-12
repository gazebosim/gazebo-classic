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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Class to manage all cameras
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#include "OgreCamera.hh"
#include "CameraManager.hh"

using namespace gazebo;

CameraManager *CameraManager::myself;

////////////////////////////////////////////////////////////////////////////////
// Constructor
CameraManager::CameraManager()
{
  this->activeCamera = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
CameraManager::~CameraManager()
{ }

////////////////////////////////////////////////////////////////////////////////
/// Get instance of myself
CameraManager *CameraManager::Instance()
{
  if (!myself)
  {
    myself = new CameraManager();
  }

  return myself;
}

////////////////////////////////////////////////////////////////////////////////
/// Add new camera
void CameraManager::AddCamera( OgreCamera *camera )
{
  this->cameras.push_back(camera);
  this->addSignal(camera);
}

////////////////////////////////////////////////////////////////////////////////
/// Del all cameras
void CameraManager::Clear()
{
  this->cameras.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of cameras
unsigned int CameraManager::GetNumCameras() const
{
  return this->cameras.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a camera
OgreCamera *CameraManager::GetCamera(int index)
{
  return this->cameras[index];
}

////////////////////////////////////////////////////////////////////////////////
/// Get a camera based on the camera's name
OgreCamera *CameraManager::GetCamera(const std::string &cameraName) const
{
  std::deque< OgreCamera* >::const_iterator iter;

  for (iter = this->cameras.begin(); iter != this->cameras.end(); iter++)
  {
    if ((*iter)->GetCameraName() == cameraName)
      return (*iter);
  }

  return NULL;
}
 
////////////////////////////////////////////////////////////////////////////////
/// Set a camera to be active.
void CameraManager::SetActiveCamera(unsigned int index)
{
  assert( index < this->cameras.size() );

  this->activeCamera = index;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the active camera
OgreCamera *CameraManager::GetActiveCamera()
{
  if (this->activeCamera < this->cameras.size())
    return this->cameras[this->activeCamera];
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the next camera in the queue to be active
void CameraManager::IncActiveCamera()
{
  this->activeCamera = (this->activeCamera+1)  % this->cameras.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the prev camera in the queue to be active
void CameraManager::DecActiveCamera()
{
  this->activeCamera = (this->activeCamera-1)  % this->cameras.size();
}
