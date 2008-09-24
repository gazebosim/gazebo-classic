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
 * SVN: $Id$
 */

#ifndef CAMERAMANAGER_HH
#define CAMERAMANAGER_HH

#include <deque>
#include <boost/signal.hpp>

namespace gazebo
{
  
  class OgreCamera;
  
  /// \brief Class to manage all the cameras in the world
  class CameraManager
  {
    /// \brief Constructor
    private: CameraManager();
  
    /// \brief Destructor
    public: ~CameraManager();
  
    /// \brief Get instance of the camera manager
    public: static CameraManager *Instance();
  
    /// \brief Add camera to the manager
    public: void AddCamera( OgreCamera *camera );
  
    /// \brief Del all the cameras
    public: void Clear();
  
    /// \brief Return the number of cameras
    public: unsigned int GetNumCameras() const;
  
    /// \brief Get a camera
    /// \param index Index of the camera to get
    /// \return Pointer to the camera
    public: OgreCamera *GetCamera(int index);

    /// \brief Get a camera based on the camera's name
    public: OgreCamera *GetCamera(const std::string &cameraName) const;
  
    /// \brief Set a camera to be active.
    public: void SetActiveCamera( unsigned int index );
  
    /// \brief Return the active camera
    /// \return Pointer to the active camera
    public: OgreCamera *GetActiveCamera();
  
    /// \brief Set the next camera in the queue to be active
    public: void IncActiveCamera();
  
    /// \brief Set the prev camera in the queue to be active
    public: void DecActiveCamera();

    /// \brief Connect a boost::slot the the AddCamera signal
    public: template<typename T>
            void ConnectAddCameraSignal( T subscriber )
            {
              addSignal.connect(subscriber);
            }
  
    /// Static self pointer
    private: static CameraManager *myself;
  
    /// List of all the cameras
    private: std::deque< OgreCamera* > cameras;
  
    /// The active camera
    private: unsigned int activeCamera;

    private: boost::signal<void (OgreCamera*)> addSignal;
  };
  
}

#endif
