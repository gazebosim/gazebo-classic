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

#include "CameraSensor.hh"
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
{}

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
void CameraManager::AddCamera( CameraSensor *camera )
{
  this->cameras.push_back(camera);
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of cameras
unsigned int CameraManager::GetNumCameras() const
{
  return this->cameras.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a camera
CameraSensor *CameraManager::GetCamera(int index)
{
  return this->cameras[index];
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
CameraSensor *CameraManager::GetActiveCamera()
{
  return this->cameras[this->activeCamera];
}
