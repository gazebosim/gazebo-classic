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
 * Desc: Sick LMS 200 laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */

#ifndef SICKLMS200_LASER_HH
#define SICKLMS200_LASER_HH

#include "Controller.hh"

namespace gazebo
{
  class LaserIface;
  class FiducialIface;
  class RaySensor;

/// @addtogroup controllers
/// @{
/** \defgroup sicklms200 sicklms200

\{
*/

/// \brief Sick LMS 200 laser controller.
/// 
/// This is a controller that simulates a Sick LMS 200
class SickLMS200_Laser : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: SickLMS200_Laser(Entity *parent);

  /// \brief Destructor
  public: virtual ~SickLMS200_Laser();

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

  /// \brief Put laser data to the iface
  private: void PutLaserData();

  /// \brief Put fiducial data to the iface
  private: void PutFiducialData();

  /// The laser interface
  private: LaserIface *laserIface;

  private: FiducialIface *fiducialIface;

  /// The parent sensor
  private: RaySensor *myParent;

};

/** /} */
/// @}

}

#endif

