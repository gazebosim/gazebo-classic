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
/*
 * Desc: A generic camera controller
 * Author: Nathan Koenig
 * Date: 07 July 2007
 * SVN: $Id$
 */

#ifndef GENERIC_CAMERA_HH
#define GENERIC_CAMERA_HH

#include "Controller.hh"

namespace gazebo
{
  class CameraSensor;
  class Cameraface;

/// @addtogroup controllers
/// @{
/** \defgroup genericcamera generic camera

\{
*/

/// \brief Generic camera controller.
/// 
/// This is a controller that simulates a generic camera
class Generic_Camera : public Controller
{
  /// \brief Constructor
  /// \param iface The libgazebo interface for the controller
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Generic_Camera(Iface *iface, Entity *parent);

  /// \brief Destructor
  public: virtual ~Generic_Camera();

  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// \brief Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild(UpdateParams &params);

  /// \brief Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// \brief Put camera data to the iface
  private: void PutCameraData();

  /// The camera interface
  private: CameraIface *cameraIface;

  /// The parent sensor
  private: CameraSensor *myParent;

};

/** /} */
/// @}

}

#endif

