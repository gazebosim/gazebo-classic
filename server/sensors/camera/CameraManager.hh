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

#ifndef CAMERAMANAGER_HH
#define CAMERAMANAGER_HH

#include <deque>

namespace gazebo
{

class Camera;

/// Class to manage all the cameras in the world
class CameraManager
{
  /// Constructor
  private: CameraManager();

  /// Destructor
  public: ~CameraManager();

  /// Get instance of the camera manager
  public: static CameraManager *Instance();

  /// Create a new camera
  /// \return Pointer to the new camera
  public: Camera *CreateCamera();

  /// Get a camera
  /// \param index Index of the camera to get
  /// \return Pointer to the camera
  public: Camera *GetCamera(int index);

  /// Set a camera to be active.
  /// \param Pointer to the camera to activate
  public: void SetActiveCamera( Camera *camera);

  /// Return the active camera
  /// \return Pointer to the active camera
  public: Camera *GetActiveCamera();

  /// Static self pointer
  private: static CameraManager *myself;

  /// List of all the cameras
  private: std::deque< Camera* > cameras;

  /// The active camera
  private: Camera *activeCamera;
};

}
#endif
