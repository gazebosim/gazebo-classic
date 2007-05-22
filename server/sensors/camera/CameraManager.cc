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
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id:$
 */

#include "Camera.hh"
#include "CameraManager.hh"

using namespace gazebo;

CameraManager *CameraManager::myself;

////////////////////////////////////////////////////////////////////////////////
// Constructor
CameraManager::CameraManager()
{
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
/// Create a new camera
Camera *CameraManager::CreateCamera()
{
  Camera *newCamera = new Camera();

  this->cameras.push_back(newCamera);

  return newCamera;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a camera
Camera *CameraManager::GetCamera(int index)
{
  return this->cameras[index];
}

////////////////////////////////////////////////////////////////////////////////
/// Set a camera to be active.
void CameraManager::SetActiveCamera( Camera *camera)
{
  this->activeCamera = camera;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the active camera
Camera *CameraManager::GetActiveCamera()
{
  return this->activeCamera;
}
